/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "BLI_array.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_lasso_2d.hh"
#include "BLI_math_geom.h"
#include "BLI_rect.h"
#include "BLI_task.hh"

#include "BKE_brush.hh"
#include "BKE_context.hh"
#include "BKE_crazyspace.hh"
#include "BKE_curves.hh"
#include "BKE_material.h"
#include "BKE_paint.hh"
#include "BKE_report.hh"

#include "DEG_depsgraph_query.hh"

#include "ED_curves.hh"
#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "DNA_material_types.h"

#include "WM_api.hh"

namespace blender::ed::greasepencil {

/**
 * Apply the stroke carver to a drawing.
 */
static bool execute_carver_on_drawing(const int layer_index,
                                      const int frame_number,
                                      const Object &ob_eval,
                                      Object &obact,
                                      const ARegion &region,
                                      const float4x4 &projection,
                                      const float4x4 &layer_to_world,
                                      const Span<int2> mcoords,
                                      const bool keep_caps,
                                      bke::greasepencil::Drawing &drawing)
{
  const bke::CurvesGeometry &src = drawing.strokes();
  const OffsetIndices<int> src_points_by_curve = src.points_by_curve();

  /* Get evaluated geometry. */
  bke::crazyspace::GeometryDeformation deformation =
      bke::crazyspace::get_evaluated_grease_pencil_drawing_deformation(
          &ob_eval, obact, layer_index, frame_number);

  /* Compute screen space positions. */
  Array<float2> screen_space_positions(src.points_num());
  threading::parallel_for(src.points_range(), 4096, [&](const IndexRange src_points) {
    for (const int src_point : src_points) {
      screen_space_positions[src_point] = ED_view3d_project_float_v2_m4(
          &region, deformation.positions[src_point], projection);
    }
  });

  Array<float2> cut_pos2d(mcoords.size());
  threading::parallel_for(mcoords.index_range(), 4096, [&](const IndexRange i_indexrange) {
    for (const int i : i_indexrange) {
      cut_pos2d[i] = float2(mcoords[i]);
    }
  });

  bke::CurvesGeometry cut = bke::CurvesGeometry(mcoords.size(), 1);
  cut.offsets_for_write().last() = mcoords.size();

  const Span<float3> normals = drawing.curve_plane_normals();

  Array<float4> normal_planes(src.points_num());
  threading::parallel_for(src.curves_range(), 4096, [&](const IndexRange src_curves) {
    for (const int src_curve : src_curves) {
      const float3 &normal = normals[src_curve];
      for (const int src_point : src_points_by_curve[src_curves]) {
        normal_planes[src_point] = float4(normal,
                                          -math::dot(deformation.positions[src_point], normal));
      }
    }
  });

  MutableSpan<float3> positions = cut.positions_for_write();
  float4 plane = float4(0.0f, 1.0f, 0.0f, 0.0f); /* TODO */
  for (const int i : mcoords.index_range()) {
    ED_view3d_win_to_3d_on_plane(&region, plane, cut_pos2d[i], false, positions[i]);
  }

  const bke::AttributeAccessor attributes = src.attributes();
  const VArray<int> materials = *attributes.lookup_or_default<int>(
      "material_index", bke::AttrDomain::Curve, -1);

  VectorSet<int> fill_material_indices;
  for (const int mat_i : IndexRange(obact.totcol)) {
    Material *material = BKE_object_material_get(&obact, mat_i + 1);
    if (material != nullptr && material->gp_style != nullptr &&
        (material->gp_style->flag & GP_MATERIAL_FILL_SHOW) != 0)
    {
      fill_material_indices.add_new(mat_i);
    }
  }

  Array<bool> use_fill(src.curves_num());
  for (const int i : src.curves_range()) {
    const int mat_index = materials[i];
    use_fill[i] = fill_material_indices.contains(mat_index);
  }

  bke::CurvesGeometry carved_strokes = ed::curves::clipping::curves_geometry_cut(
      src,
      cut,
      use_fill,
      keep_caps,
      region,
      layer_to_world,
      normal_planes,
      screen_space_positions,
      cut_pos2d);

  /* Set the new geometry. */
  drawing.strokes_for_write() = std::move(carved_strokes);
  drawing.tag_topology_changed();

  return true;
}

/**
 * Apply the stroke carver to all layers.
 */
static int stroke_carver_execute(const bContext *C, const Span<int2> mcoords)
{
  const Scene *scene = CTX_data_scene(C);
  const ARegion *region = CTX_wm_region(C);
  const RegionView3D *rv3d = CTX_wm_region_view3d(C);
  const Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
  Object *obact = CTX_data_active_object(C);
  Object *ob_eval = DEG_get_evaluated_object(depsgraph, obact);

  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(obact->data);

  Paint *paint = BKE_paint_get_active_from_context(C);
  Brush *brush = BKE_paint_brush(paint);
  if (brush->gpencil_settings == nullptr) {
    BKE_brush_init_gpencil_settings(brush);
  }
  const bool keep_caps = (brush->gpencil_settings->flag & GP_BRUSH_ERASER_KEEP_CAPS) != 0;
  const bool active_layer_only = (brush->gpencil_settings->flag & GP_BRUSH_ACTIVE_LAYER_ONLY) != 0;
  std::atomic<bool> changed = false;

  if (active_layer_only) {
    /* Apply carver on drawings of active layer. */
    if (!grease_pencil.has_active_layer()) {
      return OPERATOR_CANCELLED;
    }
    const bke::greasepencil::Layer &layer = *grease_pencil.get_active_layer();
    const float4x4 layer_to_world = layer.to_world_space(*ob_eval);
    const float4x4 projection = ED_view3d_ob_project_mat_get_from_obmat(rv3d, layer_to_world);
    const Vector<ed::greasepencil::MutableDrawingInfo> drawings =
        ed::greasepencil::retrieve_editable_drawings_from_layer(*scene, grease_pencil, layer);
    threading::parallel_for_each(drawings, [&](const ed::greasepencil::MutableDrawingInfo &info) {
      if (execute_carver_on_drawing(info.layer_index,
                                    info.frame_number,
                                    *ob_eval,
                                    *obact,
                                    *region,
                                    projection,
                                    layer_to_world,
                                    mcoords,
                                    keep_caps,
                                    info.drawing))
      {
        changed = true;
      }
    });
  }
  else {
    /* Apply cutter on every editable drawing. */
    const Vector<ed::greasepencil::MutableDrawingInfo> drawings =
        ed::greasepencil::retrieve_editable_drawings(*scene, grease_pencil);
    threading::parallel_for_each(drawings, [&](const ed::greasepencil::MutableDrawingInfo &info) {
      const bke::greasepencil::Layer &layer = *grease_pencil.layer(info.layer_index);
      const float4x4 layer_to_world = layer.to_world_space(*ob_eval);
      const float4x4 projection = ED_view3d_ob_project_mat_get_from_obmat(rv3d, layer_to_world);
      if (execute_carver_on_drawing(info.layer_index,
                                    info.frame_number,
                                    *ob_eval,
                                    *obact,
                                    *region,
                                    projection,
                                    layer_to_world,
                                    mcoords,
                                    keep_caps,
                                    info.drawing))
      {
        changed = true;
      }
    });
  }

  if (changed) {
    DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
    WM_event_add_notifier(C, NC_GEOM | ND_DATA, &grease_pencil);
  }

  return OPERATOR_FINISHED;
}

static int grease_pencil_stroke_carver(bContext *C, wmOperator *op)
{
  const Array<int2> mcoords = WM_gesture_lasso_path_to_array(C, op);

  if (mcoords.is_empty()) {
    return OPERATOR_PASS_THROUGH;
  }

  return stroke_carver_execute(C, mcoords);
}

}  // namespace blender::ed::greasepencil

void GREASE_PENCIL_OT_stroke_carver(wmOperatorType *ot)
{
  using namespace blender::ed::greasepencil;

  ot->name = "Grease Pencil Carver";
  ot->idname = "GREASE_PENCIL_OT_stroke_carver";
  ot->description = "Cuts stroke point in the intersect lasso";

  ot->invoke = WM_gesture_lasso_invoke;
  ot->modal = WM_gesture_lasso_modal;
  ot->exec = grease_pencil_stroke_carver;
  ot->poll = grease_pencil_painting_poll;
  ot->cancel = WM_gesture_lasso_cancel;

  ot->flag = OPTYPE_UNDO | OPTYPE_REGISTER;

  WM_operator_properties_gesture_lasso(ot);
}
