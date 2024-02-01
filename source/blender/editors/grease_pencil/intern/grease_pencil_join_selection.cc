/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "BKE_attribute.hh"
#include "BKE_context.hh"
#include "BKE_curves_utils.hh"
#include "BKE_grease_pencil.hh"

#include "DNA_scene_types.h"

#include "DEG_depsgraph.hh"

#include "ED_grease_pencil.hh"

#include "WM_api.hh"

#include <algorithm>

namespace blender::ed::greasepencil {

namespace {

/**
 * Struct holding information regarding the set of points identified by \a range.
 * All the points belonging to a `PointsRange` are contiguous
 */
struct PointsRange {
  bke::CurvesGeometry *owning_curves;
  IndexRange range;
  bool belongs_to_active_layer;
};

enum class ActionOnNextRange { NOTHING, REVERSE_EXISTING, REVERSE_ADDITION, REVERSE_BOTH };

/**
 * Iterates over \a drawings and returns a vector with all the selected ranges of points.
 * In the parameter \a r_total_points_selected will be written the number of points selected
 *
 *  A range of points is defined as a group contiguous and visually connected points
 */
Vector<PointsRange> retrieve_selection_ranges(Object &object,
                                              const Span<MutableDrawingInfo> drawings,
                                              const int active_layer_index,
                                              int64_t &r_total_points_selected,
                                              IndexMaskMemory &memory)
{
  Vector<PointsRange> selected_ranges{};
  r_total_points_selected = 0;

  for (const MutableDrawingInfo &info : drawings) {
    IndexMask points_selection = retrieve_editable_and_selected_points(
        object, info.drawing, memory);
    if (points_selection.is_empty()) {
      continue;
    }
    r_total_points_selected += points_selection.size();

    const Vector<IndexRange> initial_ranges = points_selection.to_ranges();
    const bool is_active_layer = info.layer_index == active_layer_index;

    /**
     * Splitting the source selection by ranges doesn't take into account the strokes,
     * i.e, if both the end of an stroke and the beginning of the next are selected, all the
     * indices end up in the same range. Let's refine the splitting
     */
    Vector<IndexRange> ranges{};
    const Array<int> points_map = info.drawing.strokes().point_to_curve_map();
    for (const IndexRange initial_range : initial_ranges) {
      if (points_map[initial_range.first()] == points_map[initial_range.last()]) {
        selected_ranges.append(
            {&info.drawing.strokes_for_write(), initial_range, is_active_layer});
        continue;
      }

      int range_begin = initial_range.first();
      int range_size = 1;
      int previous_curve = points_map[range_begin];
      for (const int64_t index : initial_range.drop_front(1)) {
        const int current_curve = points_map[index];
        if (previous_curve != current_curve) {
          IndexRange range{range_begin, range_size};
          selected_ranges.append({&info.drawing.strokes_for_write(), range, is_active_layer});
          range_begin = index;
          range_size = 1;
          previous_curve = current_curve;
        }
        else {
          range_size++;
        }
      }

      IndexRange range{range_begin, range_size};
      selected_ranges.append({&info.drawing.strokes_for_write(), range, is_active_layer});
    }
  }

  return selected_ranges;
}

void apply_action(ActionOnNextRange action,
                  const IndexRange current_range,
                  const IndexRange adding_range,
                  bke::CurvesGeometry &dst_curves)
{
  switch (action) {
    case ActionOnNextRange::NOTHING:
      return;
    case ActionOnNextRange::REVERSE_EXISTING: {
      dst_curves.reverse_points(current_range);
      break;
    }
    case ActionOnNextRange::REVERSE_ADDITION: {
      const IndexRange src_range_on_dst = {current_range.last() + 1, adding_range.size()};
      dst_curves.reverse_points(src_range_on_dst);
      break;
    }
    case ActionOnNextRange::REVERSE_BOTH: {
      apply_action(ActionOnNextRange::REVERSE_EXISTING, current_range, adding_range, dst_curves);
      apply_action(ActionOnNextRange::REVERSE_ADDITION, current_range, adding_range, dst_curves);
      break;
    }
  }
}

/**
 * Given \a range, computes which one of \a ranges is closer to it.
 * Does not evaluate the whole span, but just from the position \a starting_from onward.
 *
 * Returns the index of the closest range. The parameter \a r_action will hold the action to take
 * on this range
 */
int64_t compute_closest_range_to(PointsRange &range,
                                 const Span<PointsRange> &ranges,
                                 int64_t starting_from,
                                 ActionOnNextRange &r_action)
{
  auto get_range_begin_end = [](const PointsRange &points_range) -> std::pair<float3, float3> {
    const Span<float3> current_range_positions = points_range.owning_curves->positions();
    const float3 range_begin = current_range_positions[points_range.range.first()];
    const float3 range_end = current_range_positions[points_range.range.last()];

    return {range_begin, range_end};
  };

  const auto [cur_range_begin, cur_range_end] = get_range_begin_end(range);
  float min_dist = FLT_MAX;

  int64_t ret_value = starting_from;
  ActionOnNextRange action = ActionOnNextRange::NOTHING;

  const int64_t iterations = ranges.size() - starting_from;
  for (const int64_t i : IndexRange(starting_from, iterations)) {
    const auto [range_begin, range_end] = get_range_begin_end(ranges[i]);

    float dist = math::distance_squared(cur_range_end, range_begin);
    if (dist < min_dist) {
      action = ActionOnNextRange::NOTHING;
      ret_value = i;
      min_dist = dist;
    }

    dist = math::distance_squared(cur_range_begin, range_begin);
    if (dist < min_dist) {
      action = ActionOnNextRange::REVERSE_EXISTING;
      ret_value = i;
      min_dist = dist;
    }

    dist = math::distance_squared(cur_range_end, range_end);
    if (dist < min_dist) {
      action = ActionOnNextRange::REVERSE_ADDITION;
      ret_value = i;
      min_dist = dist;
    }

    dist = math::distance_squared(cur_range_begin, range_end);
    if (dist < min_dist) {
      action = ActionOnNextRange::REVERSE_BOTH;
      ret_value = i;
      min_dist = dist;
    }
  }

  r_action = action;
  return ret_value;
}

void copy_range_to_dst(const PointsRange &points_range,
                       int &dst_starting_point,
                       bke::CurvesGeometry &dst_curves)
{
  bke::AnonymousAttributePropagationInfo propagation_info{};

  Array<int> src_raw_offsets(2);
  Array<int> dst_raw_offsets(2);

  const int64_t selection_size = points_range.range.size();
  src_raw_offsets[0] = points_range.range.first();
  src_raw_offsets[1] = points_range.range.last() + 1;

  dst_raw_offsets[0] = dst_starting_point;
  dst_starting_point += selection_size;
  dst_raw_offsets[1] = dst_starting_point;

  OffsetIndices<int> src_offsets{src_raw_offsets};
  OffsetIndices<int> dst_offsets{dst_raw_offsets};

  copy_attributes_group_to_group(points_range.owning_curves->attributes(),
                                 bke::AttrDomain::Point,
                                 propagation_info,
                                 {},
                                 src_offsets,
                                 dst_offsets,
                                 IndexMask{1},
                                 dst_curves.attributes_for_write());
}

void copy_point_attributes(MutableSpan<PointsRange> selected_ranges,
                           bke::CurvesGeometry &dst_curves)
{
  /* The algorithm for joining the points goes as follows:
   * 1. Pick the first range of the selected ranges of points, which will be the working range
   * 2. Copy the attributes of this range to dst_curves
   * 3. Lookup in the remaining ranges for the one closer to the working range
   * 4. Copy its attributes
   * 5. In order to minimize the length of the stroke connecting them, reverse their points as
   * needed
   * 6. Extend the working range with the new range
   * 7. Remove the new range from the list of remaining ranges. Lookup for the next one and
   * continue
   */

  const PointsRange &first_range = selected_ranges.first();
  PointsRange working_range = {&dst_curves, {0, first_range.range.size()}, true};

  int next_point_index = 0;
  copy_range_to_dst(first_range, next_point_index, dst_curves);

  const int64_t ranges = selected_ranges.size() - 1;
  for (const int64_t i : IndexRange(1, ranges)) {
    ActionOnNextRange action;
    const int64_t closest_range = compute_closest_range_to(
        working_range, selected_ranges, i, action);
    std::swap(selected_ranges[i], selected_ranges[closest_range]);
    PointsRange &next_range = selected_ranges[i];
    copy_range_to_dst(next_range, next_point_index, dst_curves);
    apply_action(action, working_range.range, next_range.range, dst_curves);
    working_range.range = {0, next_point_index};
  }
}

void copy_curve_attributes(Span<PointsRange> ranges_selected, bke::CurvesGeometry &dst_curves)
{
  /* The decission of which stroke use to copy the curve attribues is a bit arbitrary, since the
   * original selection may embrace several strokes. The criteria is as follows:
   *  - If the selection contained points from the active layer, the first selected stroke from it
   * is used.
   *  - Otherwise, the first selected stroke is used.
   * Reasoning behind is that the user will probably want to keep similar curve parameters for
   * all the strokes in a layer.
   * Also, the "cyclic" attribute is deliberately set to false, since user
   * probably wants to set it manually
   */

  auto src_range = [&]() -> const PointsRange & {
    auto it = std::find_if(ranges_selected.begin(),
                           ranges_selected.end(),
                           [](const PointsRange &range) { return range.belongs_to_active_layer; });

    return it != ranges_selected.end() ? *it : ranges_selected.first();
  }();

  const bke::CurvesGeometry &src_curves = *src_range.owning_curves;
  const Array<int> points_map = src_curves.point_to_curve_map();
  const int first_selected_curve = points_map[src_range.range.first()];

  const int final_curve_index = dst_curves.curves_num() - 1;
  const Array<int> dst_curves_raw_offsets = {final_curve_index, dst_curves.curves_num()};
  const OffsetIndices<int> dst_curve_offsets{dst_curves_raw_offsets};

  bke::AnonymousAttributePropagationInfo propagation_info{};
  gather_attributes_to_groups(src_curves.attributes(),
                              bke::AttrDomain::Curve,
                              propagation_info,
                              {"cyclic"},
                              dst_curve_offsets,
                              IndexMask({first_selected_curve, 1}),
                              dst_curves.attributes_for_write());
  dst_curves.cyclic_for_write()[0] = false;
}

/**
 * Removes the selection state of all the affected CurvesGeometry, except the one
 * of the active layer. Points in the active layer do not get unselected
 */
void clear_selection_attribute(Span<PointsRange> ranges_selected,
                               const bke::AttrDomain selection_domain)
{
  for (const PointsRange &range : ranges_selected) {
    if (range.belongs_to_active_layer) {
      continue;
    }

    bke::CurvesGeometry &curves = *range.owning_curves;
    bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
    bke::SpanAttributeWriter<bool> selection = attributes.lookup_or_add_for_write_span<bool>(
        ".selection", selection_domain);

    const IndexMask mask = selection_domain == bke::AttrDomain::Point ?
                               IndexMask{curves.points_num()} :
                               IndexMask{curves.curves_num()};

    masked_fill(selection.span, false, mask);
    selection.finish();
  }
}

void remove_selected_points_in_active_layer(Span<PointsRange> ranges_selected,
                                            bke::CurvesGeometry &dst_curves)
{
  IndexMaskMemory memory;
  bke::AnonymousAttributePropagationInfo propagation_info{};

  Vector<int64_t> mask_content;
  for (const PointsRange &points_range : ranges_selected) {
    if (!points_range.belongs_to_active_layer) {
      continue;
    }

    Array<int64_t> range_content(points_range.range.size());
    IndexMask(points_range.range).to_indices(range_content.as_mutable_span());
    mask_content.extend(range_content);
  }

  IndexMask mask = IndexMask::from_indices(mask_content.as_span(), memory);
  dst_curves.remove_points(mask, propagation_info);
}

void append_strokes_from(bke::CurvesGeometry &&other, bke::CurvesGeometry &dst)
{
  bke::AnonymousAttributePropagationInfo propagation_info{};

  const int initial_points_num = dst.points_num();
  const int initial_curves_num = dst.curves_num();
  const int other_points_num = other.points_num();
  const int other_curves_num = other.curves_num();

  dst.resize(initial_points_num + other_points_num, initial_curves_num + other_curves_num);

  Array other_raw_offsets{0, other_points_num};
  Array dst_raw_offsets{initial_points_num, initial_points_num + other_points_num};

  OffsetIndices<int> other_point_offsets{other_raw_offsets};
  OffsetIndices<int> dst_point_offsets{dst_raw_offsets};

  copy_attributes_group_to_group(other.attributes(),
                                 bke::AttrDomain::Point,
                                 propagation_info,
                                 {},
                                 other_point_offsets,
                                 dst_point_offsets,
                                 IndexMask{1},
                                 dst.attributes_for_write());

  other_raw_offsets = {0, other_curves_num};
  dst_raw_offsets = {initial_curves_num, initial_curves_num + other_curves_num};

  OffsetIndices<int> other_curve_offsets{other_raw_offsets};
  OffsetIndices<int> dst_curve_offsets{dst_raw_offsets};

  copy_attributes_group_to_group(other.attributes(),
                                 bke::AttrDomain::Curve,
                                 propagation_info,
                                 {},
                                 other_curve_offsets,
                                 dst_curve_offsets,
                                 IndexMask{1},
                                 dst.attributes_for_write());
}

/* -------------------------------------------------------------------- */
/** \name Join Selection Operator
 * \{ */

/**
 * This operator builds a new stroke from the points/curves selected. It makes a copy of all the
 * selected points and joins them in a single stroke, which is added to the active layer.
 */
int grease_pencil_join_selection_exec(bContext *C, wmOperator * /* op */)
{
  using namespace bke::greasepencil;

  const Scene *scene = CTX_data_scene(C);
  const bke::AttrDomain selection_domain = ED_grease_pencil_selection_domain_get(
      scene->toolsettings);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  const Layer *active_layer = grease_pencil.get_active_layer();
  if (active_layer == nullptr) {
    return OPERATOR_CANCELLED;
  }

  const std::optional<int> opt_layer_index = grease_pencil.get_layer_index(*active_layer);
  BLI_assert(opt_layer_index.has_value());
  const int active_layer_index = *opt_layer_index;

  Drawing *dst_drawing = grease_pencil.get_editable_drawing_at(*grease_pencil.get_active_layer(),
                                                               scene->r.cfra);
  if (dst_drawing == nullptr) {
    return OPERATOR_CANCELLED;
  }

  IndexMaskMemory memory;
  int64_t selected_points_count;
  const Array<MutableDrawingInfo> editable_drawings = retrieve_editable_drawings(*scene,
                                                                                 grease_pencil);
  Vector<PointsRange> ranges_selected = retrieve_selection_ranges(
      *object, editable_drawings, active_layer_index, selected_points_count, memory);
  if (ranges_selected.size() <= 1) {
    /* Nothing to join */
    return OPERATOR_FINISHED;
  }

  /* Temporary geometry where to perform the logic
   * Once it gets stable, it is appended all at once to the destination curves */
  bke::CurvesGeometry tmp_curves(selected_points_count, 1);

  copy_point_attributes(ranges_selected, tmp_curves);
  copy_curve_attributes(ranges_selected, tmp_curves);

  clear_selection_attribute(ranges_selected, selection_domain);

  bke::CurvesGeometry &dst_curves = dst_drawing->strokes_for_write();
  remove_selected_points_in_active_layer(ranges_selected, dst_curves);
  append_strokes_from(std::move(tmp_curves), dst_curves);

  dst_curves.update_curve_types();
  dst_curves.tag_topology_changed();
  dst_drawing->tag_topology_changed();

  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GEOM | ND_DATA, &grease_pencil);

  return OPERATOR_FINISHED;
}

void GREASE_PENCIL_OT_join_selection(wmOperatorType *ot)
{
  ot->name = "Join Selection";
  ot->idname = "GREASE_PENCIL_OT_join_selection";
  ot->description = "New stroke from selected points/strokes";

  ot->exec = grease_pencil_join_selection_exec;
  ot->poll = editable_grease_pencil_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

}  // namespace

/** \} */

}  // namespace blender::ed::greasepencil

void ED_operatortypes_grease_pencil_join()
{
  using namespace blender::ed::greasepencil;

  WM_operatortype_append(GREASE_PENCIL_OT_join_selection);
}
