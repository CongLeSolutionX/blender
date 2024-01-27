/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "BKE_attribute.hh"
#include "BKE_brush.hh"
#include "BKE_context.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_material.h"
#include "BKE_scene.h"

#include "BLI_bit_span_ops.hh"
#include "BLI_bit_vector.hh"
#include "BLI_math_geom.h"
#include "BLI_math_vector.hh"
#include "BLI_vector_set.hh"

#include "DNA_brush_types.h"
#include "DNA_material_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_view3d_types.h"

#include "ED_curves.hh"
#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

namespace blender::ed::greasepencil {

DrawingPlacement::DrawingPlacement(const Scene &scene,
                                   const ARegion &region,
                                   const View3D &view3d,
                                   const Object &object)
    : region_(&region), view3d_(&view3d), transforms_(object)
{
  /* Initialize DrawingPlacementPlane from toolsettings. */
  switch (scene.toolsettings->gp_sculpt.lock_axis) {
    case GP_LOCKAXIS_VIEW:
      plane_ = DrawingPlacementPlane::View;
      break;
    case GP_LOCKAXIS_Y:
      plane_ = DrawingPlacementPlane::Front;
      placement_normal_ = float3(0, 1, 0);
      break;
    case GP_LOCKAXIS_X:
      plane_ = DrawingPlacementPlane::Side;
      placement_normal_ = float3(1, 0, 0);
      break;
    case GP_LOCKAXIS_Z:
      plane_ = DrawingPlacementPlane::Top;
      placement_normal_ = float3(0, 0, 1);
      break;
    case GP_LOCKAXIS_CURSOR: {
      plane_ = DrawingPlacementPlane::Cursor;
      float3x3 mat;
      BKE_scene_cursor_rot_to_mat3(&scene.cursor, mat.ptr());
      placement_normal_ = mat * float3(0, 0, 1);
      break;
    }
  }
  /* Initialize DrawingPlacementDepth from toolsettings. */
  switch (scene.toolsettings->gpencil_v3d_align) {
    case GP_PROJECT_VIEWSPACE:
      depth_ = DrawingPlacementDepth::ObjectOrigin;
      placement_loc_ = transforms_.layer_space_to_world_space.location();
      break;
    case (GP_PROJECT_VIEWSPACE | GP_PROJECT_CURSOR):
      depth_ = DrawingPlacementDepth::Cursor;
      placement_loc_ = float3(scene.cursor.location);
      break;
    case (GP_PROJECT_VIEWSPACE | GP_PROJECT_DEPTH_VIEW):
      depth_ = DrawingPlacementDepth::Surface;
      surface_offset_ = scene.toolsettings->gpencil_surface_offset;
      /* Default to view placement with the object origin if we don't hit a surface. */
      placement_loc_ = transforms_.layer_space_to_world_space.location();
      break;
    case (GP_PROJECT_VIEWSPACE | GP_PROJECT_DEPTH_STROKE):
      depth_ = DrawingPlacementDepth::NearestStroke;
      /* Default to view placement with the object origin if we don't hit a stroke. */
      placement_loc_ = transforms_.layer_space_to_world_space.location();
      break;
  }

  if (ELEM(plane_,
           DrawingPlacementPlane::Front,
           DrawingPlacementPlane::Side,
           DrawingPlacementPlane::Top,
           DrawingPlacementPlane::Cursor) &&
      ELEM(depth_, DrawingPlacementDepth::ObjectOrigin, DrawingPlacementDepth::Cursor))
  {
    plane_from_point_normal_v3(placement_plane_, placement_loc_, placement_normal_);
  }
}

DrawingPlacement::~DrawingPlacement()
{
  if (depth_cache_ != nullptr) {
    ED_view3d_depths_free(depth_cache_);
  }
}

bool DrawingPlacement::use_project_to_surface() const
{
  return depth_ == DrawingPlacementDepth::Surface;
}

bool DrawingPlacement::use_project_to_nearest_stroke() const
{
  return depth_ == DrawingPlacementDepth::NearestStroke;
}

void DrawingPlacement::cache_viewport_depths(Depsgraph *depsgraph, ARegion *region, View3D *view3d)
{
  const eV3DDepthOverrideMode mode = (depth_ == DrawingPlacementDepth::Surface) ?
                                         V3D_DEPTH_NO_GPENCIL :
                                         V3D_DEPTH_GPENCIL_ONLY;
  ED_view3d_depth_override(depsgraph, region, view3d, nullptr, mode, &this->depth_cache_);
}

void DrawingPlacement::set_origin_to_nearest_stroke(const float2 co)
{
  BLI_assert(depth_cache_ != nullptr);
  float depth;
  if (ED_view3d_depth_read_cached(depth_cache_, int2(co), 4, &depth)) {
    float3 origin;
    ED_view3d_depth_unproject_v3(region_, int2(co), depth, origin);

    placement_loc_ = origin;
  }
  else {
    /* If nothing was hit, use origin. */
    placement_loc_ = transforms_.layer_space_to_world_space.location();
  }
  plane_from_point_normal_v3(placement_plane_, placement_loc_, placement_normal_);
}

float3 DrawingPlacement::project(const float2 co) const
{
  float3 proj_point;
  if (depth_ == DrawingPlacementDepth::Surface) {
    /* Project using the viewport depth cache. */
    BLI_assert(depth_cache_ != nullptr);
    float depth;
    if (ED_view3d_depth_read_cached(depth_cache_, int2(co), 4, &depth)) {
      ED_view3d_depth_unproject_v3(region_, int2(co), depth, proj_point);
      float3 normal;
      ED_view3d_depth_read_cached_normal(region_, depth_cache_, int2(co), normal);
      proj_point += normal * surface_offset_;
    }
    /* If we didn't hit anything, use the view plane for placement. */
    else {
      ED_view3d_win_to_3d(view3d_, region_, placement_loc_, co, proj_point);
    }
  }
  else {
    if (ELEM(plane_,
             DrawingPlacementPlane::Front,
             DrawingPlacementPlane::Side,
             DrawingPlacementPlane::Top,
             DrawingPlacementPlane::Cursor))
    {
      ED_view3d_win_to_3d_on_plane(region_, placement_plane_, co, false, proj_point);
    }
    else if (plane_ == DrawingPlacementPlane::View) {
      ED_view3d_win_to_3d(view3d_, region_, placement_loc_, co, proj_point);
    }
  }
  return math::transform_point(transforms_.world_space_to_layer_space, proj_point);
}

void DrawingPlacement::project(const Span<float2> src, MutableSpan<float3> dst) const
{
  threading::parallel_for(src.index_range(), 1024, [&](const IndexRange range) {
    for (const int i : range) {
      dst[i] = this->project(src[i]);
    }
  });
}

static Array<int> get_frame_numbers_for_layer(const bke::greasepencil::Layer &layer,
                                              const int current_frame,
                                              const bool use_multi_frame_editing)
{
  Vector<int> frame_numbers({current_frame});
  if (use_multi_frame_editing) {
    for (const auto [frame_number, frame] : layer.frames().items()) {
      if (frame_number != current_frame && frame.is_selected()) {
        frame_numbers.append_unchecked(frame_number);
      }
    }
  }
  return frame_numbers.as_span();
}

Array<MutableDrawingInfo> retrieve_editable_drawings(const Scene &scene,
                                                     GreasePencil &grease_pencil)
{
  using namespace blender::bke::greasepencil;
  const int current_frame = scene.r.cfra;
  const ToolSettings *toolsettings = scene.toolsettings;
  const bool use_multi_frame_editing = (toolsettings->gpencil_flags &
                                        GP_USE_MULTI_FRAME_EDITING) != 0;

  Vector<MutableDrawingInfo> editable_drawings;
  Span<const Layer *> layers = grease_pencil.layers();
  for (const int layer_i : layers.index_range()) {
    const Layer &layer = *layers[layer_i];
    if (!layer.is_editable()) {
      continue;
    }
    const Array<int> frame_numbers = get_frame_numbers_for_layer(
        layer, current_frame, use_multi_frame_editing);
    for (const int frame_number : frame_numbers) {
      if (Drawing *drawing = grease_pencil.get_editable_drawing_at(layer, frame_number)) {
        editable_drawings.append({*drawing, layer_i, frame_number});
      }
    }
  }

  return editable_drawings.as_span();
}

Array<DrawingInfo> retrieve_visible_drawings(const Scene &scene, const GreasePencil &grease_pencil)
{
  using namespace blender::bke::greasepencil;
  const int current_frame = scene.r.cfra;
  const ToolSettings *toolsettings = scene.toolsettings;
  const bool use_multi_frame_editing = (toolsettings->gpencil_flags &
                                        GP_USE_MULTI_FRAME_EDITING) != 0;

  Vector<DrawingInfo> visible_drawings;
  Span<const Layer *> layers = grease_pencil.layers();
  for (const int layer_i : layers.index_range()) {
    const Layer &layer = *layers[layer_i];
    if (!layer.is_visible()) {
      continue;
    }
    const Array<int> frame_numbers = get_frame_numbers_for_layer(
        layer, current_frame, use_multi_frame_editing);
    for (const int frame_number : frame_numbers) {
      if (const Drawing *drawing = grease_pencil.get_drawing_at(layer, frame_number)) {
        visible_drawings.append({*drawing, layer_i, frame_number});
      }
    }
  }

  return visible_drawings.as_span();
}

Array<MutableDrawingInfo> retrieve_editable_drawings_of_active_layer(const Scene &scene,
                                                                     GreasePencil &grease_pencil)
{
  using namespace blender::bke::greasepencil;
  const int current_frame = scene.r.cfra;
  const ToolSettings *toolsettings = scene.toolsettings;
  const bool use_multi_frame_editing = (toolsettings->gpencil_flags &
                                        GP_USE_MULTI_FRAME_EDITING) != 0;

  Vector<MutableDrawingInfo> editable_drawings;
  if (!grease_pencil.has_active_layer()) {
    return {};
  }
  const Layer &layer = *grease_pencil.get_active_layer();
  const std::optional<int> layer_index = grease_pencil.get_layer_index(layer);
  if (!layer.is_editable() || !layer_index.has_value()) {
    return {};
  }
  const Array<int> frame_numbers = get_frame_numbers_for_layer(
      layer, current_frame, use_multi_frame_editing);
  for (const int frame_number : frame_numbers) {
    if (Drawing *drawing = grease_pencil.get_editable_drawing_at(layer, frame_number)) {
      editable_drawings.append({*drawing, layer_index.value(), frame_number});
    }
  }

  return editable_drawings.as_span();
}

static VectorSet<int> get_editable_material_indices(Object &object)
{
  BLI_assert(object.type == OB_GREASE_PENCIL);
  VectorSet<int> editable_material_indices;
  for (const int mat_i : IndexRange(object.totcol)) {
    Material *material = BKE_object_material_get(&object, mat_i + 1);
    /* The editable materials are unlocked and not hidden. */
    if (material != nullptr && material->gp_style != nullptr &&
        (material->gp_style->flag & GP_MATERIAL_LOCKED) == 0 &&
        (material->gp_style->flag & GP_MATERIAL_HIDE) == 0)
    {
      editable_material_indices.add_new(mat_i);
    }
  }
  return editable_material_indices;
}

static VectorSet<int> get_hidden_material_indices(Object &object)
{
  BLI_assert(object.type == OB_GREASE_PENCIL);
  VectorSet<int> hidden_material_indices;
  for (const int mat_i : IndexRange(object.totcol)) {
    Material *material = BKE_object_material_get(&object, mat_i + 1);
    if (material != nullptr && material->gp_style != nullptr &&
        (material->gp_style->flag & GP_MATERIAL_HIDE) != 0)
    {
      hidden_material_indices.add_new(mat_i);
    }
  }
  return hidden_material_indices;
}

IndexMask retrieve_editable_strokes(Object &object,
                                    const bke::greasepencil::Drawing &drawing,
                                    IndexMaskMemory &memory)
{
  using namespace blender;

  /* Get all the editable material indices */
  VectorSet<int> editable_material_indices = get_editable_material_indices(object);
  if (editable_material_indices.is_empty()) {
    return {};
  }

  const bke::CurvesGeometry &curves = drawing.strokes();
  const IndexRange curves_range = drawing.strokes().curves_range();
  const bke::AttributeAccessor attributes = curves.attributes();

  const VArray<int> materials = *attributes.lookup<int>("material_index", bke::AttrDomain::Curve);
  if (!materials) {
    /* If the attribute does not exist then the default is the first material. */
    if (editable_material_indices.contains(0)) {
      return curves_range;
    }
    return {};
  }
  /* Get all the strokes that have their material unlocked. */
  return IndexMask::from_predicate(
      curves_range, GrainSize(4096), memory, [&](const int64_t curve_i) {
        const int material_index = materials[curve_i];
        return editable_material_indices.contains(material_index);
      });
}

IndexMask retrieve_editable_points(Object &object,
                                   const bke::greasepencil::Drawing &drawing,
                                   IndexMaskMemory &memory)
{
  /* Get all the editable material indices */
  VectorSet<int> editable_material_indices = get_editable_material_indices(object);
  if (editable_material_indices.is_empty()) {
    return {};
  }

  const bke::CurvesGeometry &curves = drawing.strokes();
  const IndexRange points_range = drawing.strokes().points_range();
  const bke::AttributeAccessor attributes = curves.attributes();

  /* Propagate the material index to the points. */
  const VArray<int> materials = *attributes.lookup<int>("material_index", bke::AttrDomain::Point);
  if (!materials) {
    /* If the attribute does not exist then the default is the first material. */
    if (editable_material_indices.contains(0)) {
      return points_range;
    }
    return {};
  }
  /* Get all the points that are part of a stroke with an unlocked material. */
  return IndexMask::from_predicate(
      points_range, GrainSize(4096), memory, [&](const int64_t point_i) {
        const int material_index = materials[point_i];
        return editable_material_indices.contains(material_index);
      });
}

IndexMask retrieve_editable_elements(Object &object,
                                     const bke::greasepencil::Drawing &drawing,
                                     const bke::AttrDomain selection_domain,
                                     IndexMaskMemory &memory)
{
  if (selection_domain == bke::AttrDomain::Curve) {
    return ed::greasepencil::retrieve_editable_strokes(object, drawing, memory);
  }
  else if (selection_domain == bke::AttrDomain::Point) {
    return ed::greasepencil::retrieve_editable_points(object, drawing, memory);
  }
  return {};
}

IndexMask retrieve_visible_strokes(Object &object,
                                   const bke::greasepencil::Drawing &drawing,
                                   IndexMaskMemory &memory)
{
  using namespace blender;

  /* Get all the hidden material indices. */
  VectorSet<int> hidden_material_indices = get_hidden_material_indices(object);

  if (hidden_material_indices.is_empty()) {
    return drawing.strokes().curves_range();
  }

  const bke::CurvesGeometry &curves = drawing.strokes();
  const IndexRange curves_range = drawing.strokes().curves_range();
  const bke::AttributeAccessor attributes = curves.attributes();

  /* Get all the strokes that have their material visible. */
  const VArray<int> materials = *attributes.lookup_or_default<int>(
      "material_index", bke::AttrDomain::Curve, -1);
  return IndexMask::from_predicate(
      curves_range, GrainSize(4096), memory, [&](const int64_t curve_i) {
        const int material_index = materials[curve_i];
        return !hidden_material_indices.contains(material_index);
      });
}

IndexMask retrieve_editable_and_selected_strokes(Object &object,
                                                 const bke::greasepencil::Drawing &drawing,
                                                 IndexMaskMemory &memory)
{
  using namespace blender;

  const bke::CurvesGeometry &curves = drawing.strokes();
  const IndexRange curves_range = drawing.strokes().curves_range();

  const IndexMask editable_strokes = retrieve_editable_strokes(object, drawing, memory);
  const IndexMask selected_strokes = ed::curves::retrieve_selected_curves(curves, memory);

  BitVector<> editable_strokes_bits(curves.curves_num(), false);
  editable_strokes.to_bits(editable_strokes_bits);
  BitVector<> selected_strokes_bits(curves.curves_num(), false);
  selected_strokes.to_bits(selected_strokes_bits);

  editable_strokes_bits &= selected_strokes_bits;
  return IndexMask::from_bits(curves_range, editable_strokes_bits, memory);
}

IndexMask retrieve_editable_and_selected_points(Object &object,
                                                const bke::greasepencil::Drawing &drawing,
                                                IndexMaskMemory &memory)
{
  const bke::CurvesGeometry &curves = drawing.strokes();
  const IndexRange points_range = drawing.strokes().points_range();

  const IndexMask editable_points = retrieve_editable_points(object, drawing, memory);
  const IndexMask selected_points = ed::curves::retrieve_selected_points(curves, memory);

  BitVector<> editable_points_bits(curves.points_num(), false);
  editable_points.to_bits(editable_points_bits);
  BitVector<> selected_points_bits(curves.points_num(), false);
  selected_points.to_bits(selected_points_bits);

  editable_points_bits &= selected_points_bits;
  return IndexMask::from_bits(points_range, editable_points_bits, memory);
}

IndexMask retrieve_editable_and_selected_elements(Object &object,
                                                  const bke::greasepencil::Drawing &drawing,
                                                  const bke::AttrDomain selection_domain,
                                                  IndexMaskMemory &memory)
{
  if (selection_domain == bke::AttrDomain::Curve) {
    return ed::greasepencil::retrieve_editable_and_selected_strokes(object, drawing, memory);
  }
  else if (selection_domain == bke::AttrDomain::Point) {
    return ed::greasepencil::retrieve_editable_and_selected_points(object, drawing, memory);
  }
  return {};
}

Array<PointTransferData> compute_topology_change(
    const bke::CurvesGeometry &src,
    bke::CurvesGeometry &dst,
    const Span<Vector<PointTransferData>> src_to_dst_points,
    const bool keep_caps)
{
  const int src_curves_num = src.curves_num();
  const OffsetIndices<int> src_points_by_curve = src.points_by_curve();
  const VArray<bool> src_cyclic = src.cyclic();

  int dst_points_num = 0;
  for (const Vector<PointTransferData> &src_transfer_data : src_to_dst_points) {
    dst_points_num += src_transfer_data.size();
  }
  if (dst_points_num == 0) {
    dst.resize(0, 0);
    return Array<PointTransferData>(0);
  }

  /* Set the intersection parameters in the destination domain : a pair of int and float
   * numbers for which the integer is the index of the corresponding segment in the
   * source curves, and the float part is the (0,1) factor representing its position in
   * the segment.
   */
  Array<PointTransferData> dst_transfer_data(dst_points_num);

  Array<int> src_pivot_point(src_curves_num, -1);
  Array<int> dst_interm_curves_offsets(src_curves_num + 1, 0);
  int dst_point = -1;
  for (const int src_curve : src.curves_range()) {
    const IndexRange src_points = src_points_by_curve[src_curve];

    for (const int src_point : src_points) {
      for (const PointTransferData &dst_point_transfer : src_to_dst_points[src_point]) {
        if (dst_point_transfer.is_src_point) {
          dst_transfer_data[++dst_point] = dst_point_transfer;
          continue;
        }

        /* Add an intersection with the eraser and mark it as a cut. */
        dst_transfer_data[++dst_point] = dst_point_transfer;

        /* For cyclic curves, mark the pivot point as the last intersection with the eraser
         * that starts a new segment in the destination.
         */
        if (src_cyclic[src_curve] && dst_point_transfer.is_cut) {
          src_pivot_point[src_curve] = dst_point;
        }
      }
    }
    /* We store intermediate curve offsets represent an intermediate state of the
     * destination curves before cutting the curves at eraser's intersection. Thus, it
     * contains the same number of curves than in the source, but the offsets are
     * different, because points may have been added or removed. */
    dst_interm_curves_offsets[src_curve + 1] = dst_point + 1;
  }

  /* Cyclic curves. */
  Array<bool> src_now_cyclic(src_curves_num);
  threading::parallel_for(src.curves_range(), 4096, [&](const IndexRange src_curves) {
    for (const int src_curve : src_curves) {
      const int pivot_point = src_pivot_point[src_curve];

      if (pivot_point == -1) {
        /* Either the curve was not cyclic or it wasn't cut : no need to change it. */
        src_now_cyclic[src_curve] = src_cyclic[src_curve];
        continue;
      }

      /* A cyclic curve was cut :
       *  - this curve is not cyclic anymore,
       *  - and we have to shift points to keep the closing segment.
       */
      src_now_cyclic[src_curve] = false;

      const int dst_interm_first = dst_interm_curves_offsets[src_curve];
      const int dst_interm_last = dst_interm_curves_offsets[src_curve + 1];
      std::rotate(dst_transfer_data.begin() + dst_interm_first,
                  dst_transfer_data.begin() + pivot_point,
                  dst_transfer_data.begin() + dst_interm_last);
    }
  });

  /* Compute the destination curve offsets. */
  Vector<int> dst_curves_offset;
  Vector<int> dst_to_src_curve;
  dst_curves_offset.append(0);
  for (int src_curve : src.curves_range()) {
    const IndexRange dst_points(dst_interm_curves_offsets[src_curve],
                                dst_interm_curves_offsets[src_curve + 1] -
                                    dst_interm_curves_offsets[src_curve]);
    int length_of_current = 0;

    for (int dst_point : dst_points) {

      if ((length_of_current > 0) && dst_transfer_data[dst_point].is_cut) {
        /* This is the new first point of a curve. */
        dst_curves_offset.append(dst_point);
        dst_to_src_curve.append(src_curve);
        length_of_current = 0;
      }
      ++length_of_current;
    }

    if (length_of_current != 0) {
      /* End of a source curve. */
      dst_curves_offset.append(dst_points.one_after_last());
      dst_to_src_curve.append(src_curve);
    }
  }
  const int dst_curves_num = dst_curves_offset.size() - 1;
  if (dst_curves_num == 0) {
    dst.resize(0, 0);
    return dst_transfer_data;
  }

  /* Build destination curves geometry. */
  dst.resize(dst_points_num, dst_curves_num);
  array_utils::copy(dst_curves_offset.as_span(), dst.offsets_for_write());
  const OffsetIndices<int> dst_points_by_curve = dst.points_by_curve();

  /* Attributes. */
  const bke::AttributeAccessor src_attributes = src.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst.attributes_for_write();
  const bke::AnonymousAttributePropagationInfo propagation_info{};

  /* Copy curves attributes. */
  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Curve,
                         propagation_info,
                         {"cyclic"},
                         dst_to_src_curve,
                         dst_attributes);
  if (src_cyclic.get_if_single().value_or(true)) {
    array_utils::gather(
        src_now_cyclic.as_span(), dst_to_src_curve.as_span(), dst.cyclic_for_write());
  }

  dst.update_curve_types();

  /* Display intersections with flat caps. */
  if (!keep_caps) {
    bke::SpanAttributeWriter<int8_t> dst_start_caps =
        dst_attributes.lookup_or_add_for_write_span<int8_t>("start_cap", bke::AttrDomain::Curve);
    bke::SpanAttributeWriter<int8_t> dst_end_caps =
        dst_attributes.lookup_or_add_for_write_span<int8_t>("end_cap", bke::AttrDomain::Curve);

    threading::parallel_for(dst.curves_range(), 4096, [&](const IndexRange dst_curves) {
      for (const int dst_curve : dst_curves) {
        const IndexRange dst_curve_points = dst_points_by_curve[dst_curve];
        if (dst_transfer_data[dst_curve_points.first()].is_cut) {
          dst_start_caps.span[dst_curve] = GP_STROKE_CAP_TYPE_FLAT;
        }

        if (dst_curve == dst_curves.last()) {
          continue;
        }

        const PointTransferData &next_point_transfer =
            dst_transfer_data[dst_points_by_curve[dst_curve + 1].first()];

        if (next_point_transfer.is_cut) {
          dst_end_caps.span[dst_curve] = GP_STROKE_CAP_TYPE_FLAT;
        }
      }
    });

    dst_start_caps.finish();
    dst_end_caps.finish();
  }

  /* Copy/Interpolate point attributes. */
  for (bke::AttributeTransferData &attribute : bke::retrieve_attributes_for_transfer(
           src_attributes, dst_attributes, ATTR_DOMAIN_MASK_POINT, propagation_info))
  {
    bke::attribute_math::convert_to_static_type(attribute.dst.span.type(), [&](auto dummy) {
      using T = decltype(dummy);
      auto src_attr = attribute.src.typed<T>();
      auto dst_attr = attribute.dst.span.typed<T>();

      threading::parallel_for(dst.points_range(), 4096, [&](const IndexRange dst_points) {
        for (const int dst_point : dst_points) {
          const PointTransferData &point_transfer = dst_transfer_data[dst_point];
          if (point_transfer.is_src_point) {
            dst_attr[dst_point] = src_attr[point_transfer.src_point];
          }
          else {
            dst_attr[dst_point] = bke::attribute_math::mix2<T>(
                point_transfer.factor,
                src_attr[point_transfer.src_point],
                src_attr[point_transfer.src_next_point]);
          }
        }
      });

      attribute.dst.finish();
    });
  }

  return dst_transfer_data;
}

}  // namespace blender::ed::greasepencil
