/* SPDX-FileCopyrightText: 2020 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edsculpt
 */

#include "BLI_array_utils.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_math_base.hh"

#include "BKE_attribute.hh"
#include "BKE_context.hh"
#include "BKE_layer.hh"
#include "BKE_mesh.hh"
#include "BKE_paint.hh"
#include "BKE_pbvh_api.hh"
#include "BKE_subdiv_ccg.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "mesh_brush_common.hh"
#include "paint_intern.hh"
#include "sculpt_intern.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "bmesh.hh"

#include <cmath>
#include <cstdlib>

namespace blender::ed::sculpt_paint::mask {

enum class FilterType {
  Smooth = 0,
  Sharpen = 1,
  Grow = 2,
  Shrink = 3,
  ContrastIncrease = 5,
  ContrastDecrease = 6,
};

static EnumPropertyItem prop_mask_filter_types[] = {
    {int(FilterType::Smooth), "SMOOTH", 0, "Smooth Mask", ""},
    {int(FilterType::Sharpen), "SHARPEN", 0, "Sharpen Mask", ""},
    {int(FilterType::Grow), "GROW", 0, "Grow Mask", ""},
    {int(FilterType::Shrink), "SHRINK", 0, "Shrink Mask", ""},
    {int(FilterType::ContrastIncrease), "CONTRAST_INCREASE", 0, "Increase Contrast", ""},
    {int(FilterType::ContrastDecrease), "CONTRAST_DECREASE", 0, "Decrease Contrast", ""},
    {0, nullptr, 0, nullptr, nullptr},
};

static void mask_increase_contrast(const Span<float> src, const MutableSpan<float> dst)
{
  const float contrast = 0.1f;
  const float delta = contrast * 0.5f;
  const float gain = math::rcp(1.0f - contrast);
  const float offset = gain * -delta;
  for (const int i : src.index_range()) {
    dst[i] = gain * src[i] + offset;
  }
}

static void mask_decrease_contrast(const Span<float> src, const MutableSpan<float> dst)
{
  const float contrast = -0.1f;
  const float delta = contrast * 0.5f;
  const float gain = 1.0f - contrast;
  const float offset = gain * -delta;
  for (const int i : src.index_range()) {
    dst[i] = gain * src[i] + offset;
  }
}

struct FilterLocalData {
  Vector<int> visible_verts;
  Vector<float> node_mask;
  Vector<float> new_mask;
  Vector<Vector<int>> vert_neighbors;
};

static void apply_new_mask_mesh(Object &object,
                                const Span<bool> hide_vert,
                                const Span<PBVHNode *> nodes,
                                const OffsetIndices<int> node_verts,
                                const Span<float> new_mask,
                                MutableSpan<float> mask)
{
  threading::EnumerableThreadSpecific<Vector<int>> all_tls;
  threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
    Vector<int> &tls = all_tls.local();
    threading::isolate_task([&]() {
      for (const int i : range) {
        const Span<int> verts = hide::node_visible_verts(*nodes[i], hide_vert, tls);
        const Span<float> new_node_mask = new_mask.slice(node_verts[i]);
        if (array_utils::indexed_data_equal<float>(mask, verts, new_node_mask)) {
          return;
        }
        undo::push_node(object, nodes[i], undo::Type::Mask);
        array_utils::scatter(new_node_mask, verts, mask);
        BKE_pbvh_node_mark_update_mask(nodes[i]);
      }
    });
  });
}

static void smooth_mask_mesh(const OffsetIndices<int> faces,
                             const Span<int> corner_verts,
                             const GroupedSpan<int> vert_to_face_map,
                             const Span<bool> hide_poly,
                             const Span<float> mask,
                             PBVHNode &node,
                             FilterLocalData &tls,
                             MutableSpan<float> new_mask)
{
  const Span<int> verts = bke::pbvh::node_unique_verts(node);

  tls.vert_neighbors.reinitialize(verts.size());
  const MutableSpan<Vector<int>> neighbors = tls.vert_neighbors;
  calc_vert_neighbors(faces, corner_verts, vert_to_face_map, hide_poly, verts, neighbors);

  average_neighbor_mask_mesh(mask, neighbors, new_mask);
}

BLI_NOINLINE static void sharpen_masks(const Span<float> old_masks,
                                       const MutableSpan<float> new_mask)
{
  for (const int i : old_masks.index_range()) {
    float val = new_mask[i];
    float mask = old_masks[i];
    val -= mask;
    if (mask > 0.5f) {
      mask += 0.05f;
    }
    else {
      mask -= 0.05f;
    }
    mask += val / 2.0f;
    new_mask[i] = mask;
  }
}

static void sharpen_mask_mesh(const OffsetIndices<int> faces,
                              const Span<int> corner_verts,
                              const GroupedSpan<int> vert_to_face_map,
                              const Span<bool> hide_poly,
                              const Span<float> mask,
                              PBVHNode &node,
                              FilterLocalData &tls,
                              MutableSpan<float> new_mask)
{
  const Span<int> verts = bke::pbvh::node_unique_verts(node);

  tls.node_mask.reinitialize(verts.size());
  const MutableSpan<float> node_mask = tls.node_mask;
  array_utils::gather<float>(mask, verts, node_mask);

  tls.vert_neighbors.reinitialize(verts.size());
  const MutableSpan<Vector<int>> neighbors = tls.vert_neighbors;
  calc_vert_neighbors(faces, corner_verts, vert_to_face_map, hide_poly, verts, neighbors);

  average_neighbor_mask_mesh(mask, neighbors, new_mask);

  sharpen_masks(node_mask, new_mask);

  mask::clamp_mask(new_mask);
}

static void grow_mask_mesh(const OffsetIndices<int> faces,
                           const Span<int> corner_verts,
                           const GroupedSpan<int> vert_to_face_map,
                           const Span<bool> hide_poly,
                           const Span<float> mask,
                           PBVHNode &node,
                           FilterLocalData &tls,
                           MutableSpan<float> new_mask)
{
  const Span<int> verts = bke::pbvh::node_unique_verts(node);

  tls.vert_neighbors.reinitialize(verts.size());
  const MutableSpan<Vector<int>> neighbors = tls.vert_neighbors;
  calc_vert_neighbors(faces, corner_verts, vert_to_face_map, hide_poly, verts, neighbors);

  for (const int i : verts.index_range()) {
    new_mask[i] = 0.0f;
    for (const int neighbor : neighbors[i]) {
      new_mask[i] = std::max(mask[neighbor], new_mask[i]);
    }
  }
}

static void shrink_mask_mesh(const OffsetIndices<int> faces,
                             const Span<int> corner_verts,
                             const GroupedSpan<int> vert_to_face_map,
                             const Span<bool> hide_poly,
                             const Span<float> mask,
                             PBVHNode &node,
                             FilterLocalData &tls,
                             MutableSpan<float> new_mask)
{
  const Span<int> verts = bke::pbvh::node_unique_verts(node);

  tls.vert_neighbors.reinitialize(verts.size());
  const MutableSpan<Vector<int>> neighbors = tls.vert_neighbors;
  calc_vert_neighbors(faces, corner_verts, vert_to_face_map, hide_poly, verts, neighbors);

  for (const int i : verts.index_range()) {
    new_mask[i] = 1.0f;
    for (const int neighbor : neighbors[i]) {
      new_mask[i] = std::min(mask[neighbor], new_mask[i]);
    }
  }
}

static void increase_contrast_mask_mesh(const Object &object,
                                        const Span<bool> hide_vert,
                                        PBVHNode &node,
                                        FilterLocalData &tls,
                                        MutableSpan<float> mask)
{
  const Span<int> verts = hide::node_visible_verts(node, hide_vert, tls.visible_verts);

  tls.node_mask.reinitialize(verts.size());
  const MutableSpan<float> node_mask = tls.node_mask;
  array_utils::gather<float>(mask, verts, node_mask);

  tls.new_mask.reinitialize(verts.size());
  const MutableSpan<float> new_mask = tls.new_mask;
  mask_increase_contrast(node_mask, new_mask);
  mask::clamp_mask(new_mask);

  if (node_mask.as_span() == new_mask.as_span()) {
    return;
  }

  undo::push_node(object, &node, undo::Type::Mask);
  array_utils::scatter(new_mask.as_span(), verts, mask);
  BKE_pbvh_node_mark_update_mask(&node);
}

static void decrease_contrast_mask_mesh(const Object &object,
                                        const Span<bool> hide_vert,
                                        PBVHNode &node,
                                        FilterLocalData &tls,
                                        MutableSpan<float> mask)
{
  const Span<int> verts = hide::node_visible_verts(node, hide_vert, tls.visible_verts);

  tls.node_mask.reinitialize(verts.size());
  const MutableSpan<float> node_mask = tls.node_mask;
  array_utils::gather<float>(mask, verts, node_mask);

  tls.new_mask.reinitialize(verts.size());
  const MutableSpan<float> new_mask = tls.new_mask;
  mask_decrease_contrast(node_mask, new_mask);
  mask::clamp_mask(new_mask);

  if (node_mask.as_span() == new_mask.as_span()) {
    return;
  }

  undo::push_node(object, &node, undo::Type::Mask);
  array_utils::scatter(new_mask.as_span(), verts, mask);
  BKE_pbvh_node_mark_update_mask(&node);
}

static int sculpt_mask_filter_exec(bContext *C, wmOperator *op)
{
  Object &ob = *CTX_data_active_object(C);
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
  const Scene *scene = CTX_data_scene(C);
  const FilterType filter_type = FilterType(RNA_enum_get(op->ptr, "filter_type"));

  const View3D *v3d = CTX_wm_view3d(C);
  const Base *base = CTX_data_active_base(C);
  if (!BKE_base_is_visible(v3d, base)) {
    return OPERATOR_CANCELLED;
  }

  MultiresModifierData *mmd = BKE_sculpt_multires_active(scene, &ob);
  BKE_sculpt_mask_layers_ensure(CTX_data_depsgraph_pointer(C), CTX_data_main(C), &ob, mmd);

  BKE_sculpt_update_object_for_edit(depsgraph, &ob, false);

  SculptSession &ss = *ob.sculpt;
  PBVH &pbvh = *ob.sculpt->pbvh;

  SCULPT_vertex_random_access_ensure(ss);

  int num_verts = SCULPT_vertex_count_get(ss);

  Vector<PBVHNode *> nodes = bke::pbvh::search_gather(pbvh, {});
  undo::push_begin(ob, op);

  int iterations = RNA_int_get(op->ptr, "iterations");

  /* Auto iteration count calculates the number of iteration based on the vertices of the mesh to
   * avoid adding an unnecessary amount of undo steps when using the operator from a shortcut.
   * One iteration per 50000 vertices in the mesh should be fine in most cases.
   * Maybe we want this to be configurable. */
  if (RNA_boolean_get(op->ptr, "auto_iteration_count")) {
    iterations = int(num_verts / 50000.0f) + 1;
  }

  threading::EnumerableThreadSpecific<FilterLocalData> all_tls;
  switch (BKE_pbvh_type(pbvh)) {
    case PBVH_FACES: {
      Mesh &mesh = *static_cast<Mesh *>(ob.data);
      const OffsetIndices<int> faces = mesh.faces();
      const Span<int> corner_verts = mesh.corner_verts();
      const GroupedSpan<int> vert_to_face_map = ss.vert_to_face_map;
      bke::MutableAttributeAccessor attributes = mesh.attributes_for_write();
      const VArraySpan hide_vert = *attributes.lookup<bool>(".hide_vert", bke::AttrDomain::Point);
      const VArraySpan hide_poly = *attributes.lookup<bool>(".hide_poly", bke::AttrDomain::Face);
      bke::SpanAttributeWriter mask = attributes.lookup_for_write_span<float>(".sculpt_mask");

      Array<int> node_vert_offset_data;
      OffsetIndices node_offsets = create_node_vert_offsets(nodes, node_vert_offset_data);
      Array<float> new_masks(node_offsets.total_size());

      for (int i = 0; i < iterations; i++) {
        switch (filter_type) {
          case FilterType::Smooth: {
            threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
              FilterLocalData &tls = all_tls.local();
              threading::isolate_task([&]() {
                for (const int i : range) {
                  smooth_mask_mesh(faces,
                                   corner_verts,
                                   vert_to_face_map,
                                   hide_poly,
                                   mask.span,
                                   *nodes[i],
                                   tls,
                                   new_masks.as_mutable_span().slice(node_offsets[i]));
                }
              });
            });
            apply_new_mask_mesh(ob, hide_vert, nodes, node_offsets, new_masks, mask.span);
            break;
          }
          case FilterType::Sharpen: {
            threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
              FilterLocalData &tls = all_tls.local();
              threading::isolate_task([&]() {
                for (const int i : range) {
                  sharpen_mask_mesh(faces,
                                    corner_verts,
                                    vert_to_face_map,
                                    hide_poly,
                                    mask.span,
                                    *nodes[i],
                                    tls,
                                    new_masks.as_mutable_span().slice(node_offsets[i]));
                }
              });
            });
            apply_new_mask_mesh(ob, hide_vert, nodes, node_offsets, new_masks, mask.span);
            break;
          }
          case FilterType::Grow: {
            threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
              FilterLocalData &tls = all_tls.local();
              for (const int i : range) {
                grow_mask_mesh(faces,
                               corner_verts,
                               vert_to_face_map,
                               hide_poly,
                               mask.span,
                               *nodes[i],
                               tls,
                               new_masks.as_mutable_span().slice(node_offsets[i]));
              }
            });
            apply_new_mask_mesh(ob, hide_vert, nodes, node_offsets, new_masks, mask.span);
            break;
          }
          case FilterType::Shrink: {
            threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
              FilterLocalData &tls = all_tls.local();
              for (const int i : range) {
                shrink_mask_mesh(faces,
                                 corner_verts,
                                 vert_to_face_map,
                                 hide_poly,
                                 mask.span,
                                 *nodes[i],
                                 tls,
                                 new_masks.as_mutable_span().slice(node_offsets[i]));
              }
            });
            apply_new_mask_mesh(ob, hide_vert, nodes, node_offsets, new_masks, mask.span);
            break;
          }
          case FilterType::ContrastIncrease: {
            threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
              FilterLocalData &tls = all_tls.local();
              threading::isolate_task([&]() {
                for (const int i : range) {
                  increase_contrast_mask_mesh(ob, hide_vert, *nodes[i], tls, mask.span);
                }
              });
            });
            break;
          }
          case FilterType::ContrastDecrease: {
            threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
              FilterLocalData &tls = all_tls.local();
              threading::isolate_task([&]() {
                for (const int i : range) {
                  decrease_contrast_mask_mesh(ob, hide_vert, *nodes[i], tls, mask.span);
                }
              });
            });
            break;
          }
        }
      }
      mask.finish();
      break;
    }
    case PBVH_GRIDS: {
      for (int i = 0; i < iterations; i++) {
      }
      break;
    }
    case PBVH_BMESH: {
      for (int i = 0; i < iterations; i++) {
      }
      break;
    }
  }

  undo::push_end(ob);

  SCULPT_tag_update_overlays(C);

  return OPERATOR_FINISHED;
}

void SCULPT_OT_mask_filter(wmOperatorType *ot)
{
  ot->name = "Mask Filter";
  ot->idname = "SCULPT_OT_mask_filter";
  ot->description = "Applies a filter to modify the current mask";

  ot->exec = sculpt_mask_filter_exec;
  ot->poll = SCULPT_mode_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_enum(ot->srna,
               "filter_type",
               prop_mask_filter_types,
               int(FilterType::Smooth),
               "Type",
               "Filter that is going to be applied to the mask");
  RNA_def_int(ot->srna,
              "iterations",
              1,
              1,
              100,
              "Iterations",
              "Number of times that the filter is going to be applied",
              1,
              100);
  RNA_def_boolean(
      ot->srna,
      "auto_iteration_count",
      true,
      "Auto Iteration Count",
      "Use an automatic number of iterations based on the number of vertices of the sculpt");
}

}  // namespace blender::ed::sculpt_paint::mask
