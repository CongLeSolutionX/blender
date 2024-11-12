/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_curves_utils.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "ED_curves.hh"

#include "DEG_depsgraph.hh"

namespace blender::ed::curves {

/**
 * Merges copy intervals at curve endings to minimize number of copy operations.
 * For example given in function 'extrude_curves' intervals [0, 3, 4, 4, 4] became [0, 4, 4].
 * Leading to only two copy operations.
 */
static Span<int> compress_intervals(const OffsetIndices<int> intervals_by_curve,
                                    MutableSpan<int> intervals)
{
  const int *src = intervals.data();
  /* Skip the first curve, as all the data stays in the same place. */
  int *dst = intervals.data() + intervals_by_curve[0].drop_back(1).size();

  for (const int curve : IndexRange(1, intervals_by_curve.size() - 1)) {
    const IndexRange range = intervals_by_curve[curve].drop_back(1);
    const int width = range.size() - 1;
    std::copy_n(src + range.first() + 1, width, dst);
    dst += width;
  }
  (*dst) = src[intervals_by_curve[intervals_by_curve.size() - 1].last()];
  return {intervals.data(), dst - intervals.data() + 1};
}

/**
 * Creates copy intervals for selection #range in the context of #curve_points.
 * Slices the current curve points from the #range and returns size of the new range.
 * If whole #range was handled returns 0, otherwise leftover has to be handled with the next curve.
 */
static int handle_range(const int interval_offset,
                        const IndexRange curve_points,
                        int &current_interval,
                        IndexRange &range,
                        MutableSpan<int> curves_intervals,
                        bool &is_first_selected)
{
  if (current_interval == 0) {
    is_first_selected = range.first() == curve_points.start() && range.size() == 1;
    if (!is_first_selected) {
      current_interval++;
    }
  }
  const int current_interval_index = interval_offset + current_interval;
  const int left_endpoint = math::min(curve_points.last(), range.last());

  curves_intervals[current_interval_index] = range.first();
  curves_intervals[current_interval_index + 1] = left_endpoint;

  range = range.take_back(range.last() - left_endpoint);
  current_interval += 2;
  return range.size();
}

static void finish_curve(const int curve_index,
                         const int last_interval,
                         const IndexRange curve_points,
                         MutableSpan<int> new_offsets,
                         MutableSpan<int> curves_intervals,
                         MutableSpan<int> curves_intervals_offsets,
                         int &interval_offset,
                         bool &is_first_selected)
{
  const int last_interval_index = interval_offset + last_interval;
  int appended_point = 0;
  if (curves_intervals[last_interval_index] != curve_points.last() ||
      curves_intervals[last_interval_index - 1] != curves_intervals[last_interval_index])
  {
    /* Append last element of the current curve if it is not extruded or extruded together with
     * preceding points. */
    curves_intervals[last_interval_index + 1] = curve_points.last();
    appended_point++;
  }
  else if (is_first_selected && last_interval == 1) {
    /* Extrusion from one point. */
    curves_intervals[last_interval_index + 1] = curves_intervals[last_interval_index];
    is_first_selected = false;
    appended_point++;
  }
  curves_intervals_offsets[curve_index + 1] = last_interval_index + appended_point + 1;
  new_offsets[curve_index + 1] = new_offsets[curve_index] + curve_points.size() + last_interval +
                                 appended_point - 1;
  interval_offset += last_interval + appended_point + 1;
}

static void finish_curve_or_full_copy(const int curve_index,
                                      const int current_interval,
                                      const std::optional<IndexRange> prev_range,
                                      const IndexRange curve_points,
                                      MutableSpan<int> new_offsets,
                                      MutableSpan<int> curves_intervals,
                                      MutableSpan<int> curves_intervals_offsets,
                                      int &interval_offset,
                                      bool &is_first_selected)
{
  if (prev_range.has_value() && prev_range.value().last() >= curve_points.start()) {
    finish_curve(curve_index,
                 current_interval - 1,
                 curve_points,
                 new_offsets,
                 curves_intervals,
                 curves_intervals_offsets,
                 interval_offset,
                 is_first_selected);
  }
  else {
    /* Copy full curve if previous selected point was not on this curve. */
    is_first_selected = false;
    curves_intervals[interval_offset] = curve_points.first();
    curves_intervals[interval_offset + 1] = curve_points.last();
    curves_intervals_offsets[curve_index + 1] = interval_offset + 2;
    new_offsets[curve_index + 1] = new_offsets[curve_index] + curve_points.size();
    interval_offset += 2;
  }
}

static void calc_curves_extrusion(const IndexMask &selection,
                                  const OffsetIndices<int> points_by_curve,
                                  MutableSpan<int> new_offsets,
                                  MutableSpan<int> curves_intervals,
                                  MutableSpan<int> curves_intervals_offsets,
                                  MutableSpan<bool> is_first_selected)
{
  std::optional<IndexRange> prev_range;
  int current_interval = 0;
  int curve_index = 0;
  int interval_offset = 0;
  curves_intervals[interval_offset] = points_by_curve[0].start();
  curves_intervals_offsets[0] = 0;
  new_offsets[0] = points_by_curve[0].start();

  selection.foreach_range([&](const IndexRange range) {
    IndexRange curve_points = points_by_curve[curve_index];
    /* Beginning of the range outside current curve. */
    if (range.first() > curve_points.last()) {
      do {
        finish_curve_or_full_copy(curve_index,
                                  current_interval,
                                  prev_range,
                                  curve_points,
                                  new_offsets,
                                  curves_intervals,
                                  curves_intervals_offsets,
                                  interval_offset,
                                  is_first_selected[curve_index]);
        curve_points = points_by_curve[++curve_index];
      } while (range.first() > curve_points.last());
      current_interval = 0;
      curves_intervals[interval_offset] = points_by_curve[curve_index].start();
    }

    IndexRange range_to_handle = range;
    while (handle_range(interval_offset,
                        curve_points,
                        current_interval,
                        range_to_handle,
                        curves_intervals,
                        is_first_selected[curve_index]))
    {
      finish_curve(curve_index,
                   current_interval - 1,
                   curve_points,
                   new_offsets,
                   curves_intervals,
                   curves_intervals_offsets,
                   interval_offset,
                   is_first_selected[curve_index]);
      curve_points = points_by_curve[++curve_index];
      curves_intervals[interval_offset] = curve_points.start();
      current_interval = 0;
    }
    prev_range = range;
  });

  do {
    finish_curve_or_full_copy(curve_index,
                              current_interval,
                              prev_range,
                              points_by_curve[curve_index],
                              new_offsets,
                              curves_intervals,
                              curves_intervals_offsets,
                              interval_offset,
                              is_first_selected[curve_index]);
    curve_index++;
    prev_range.reset();
  } while (curve_index < points_by_curve.size());
}

static void extrude_curves(Curves &curves_id)
{
  const bke::AttrDomain selection_domain = bke::AttrDomain(curves_id.selection_domain);
  if (selection_domain != bke::AttrDomain::Point) {
    return;
  }

  IndexMaskMemory memory;
  const IndexMask extruded_points = retrieve_selected_points(curves_id, memory);
  if (extruded_points.is_empty()) {
    return;
  }

  const bke::CurvesGeometry &curves = curves_id.geometry.wrap();
  const OffsetIndices<int> points_by_curve = curves.points_by_curve();

  bke::CurvesGeometry new_curves = bke::curves::copy_only_curve_domain(curves);

  const int curves_num = curves.curves_num();

  MutableSpan<int> new_offsets = new_curves.offsets_for_write();

  /* Buffer for intervals of all curves. Beginning and end of a curve can be determined only by
   * #curve_interval_ranges. For ex. [0, 3, 4, 4, 4] indicates one copy interval for first curve
   * [0, 3] and two for second [4, 4][4, 4]. The first curve will be copied as is without changes,
   * in the second one (consisting only one point - 4) first point will be duplicated (extruded).
   */
  Array<int> curves_intervals(extruded_points.size() * 2 + curves_num * 2);

  /* Points to intervals for each curve in the curves_intervals array.
   * For example above value would be [0, 3, 5]. Meaning that [0 .. 2] are indices for curve 0 in
   * curves_intervals array, [3 .. 4] for curve 1. */
  Array<int> curves_intervals_offsets(curves_num + 1);

  /* Per curve boolean indicating if first interval in a curve is selected.
   * Other can be calculated as in a curve two adjacent intervals can not have same selection
   * state. */
  Array<bool> is_first_selected(curves_num);

  calc_curves_extrusion(extruded_points,
                        points_by_curve,
                        new_offsets,
                        curves_intervals,
                        curves_intervals_offsets,
                        is_first_selected);

  new_curves.resize(new_offsets.last(), new_curves.curves_num());

  const bke::AttributeAccessor src_attributes = curves.attributes();

  std::array<GVArraySpan, 3> src_selection;
  std::array<bke::GSpanAttributeWriter, 3> dst_selections;

  const Span<StringRef> selection_attr_names = get_curves_selection_attribute_names(curves);
  for (const int selection_i : selection_attr_names.index_range()) {
    const StringRef selection_name = selection_attr_names[selection_i];

    GVArray src_selection_array = *src_attributes.lookup(selection_name, bke::AttrDomain::Point);
    if (!src_selection_array) {
      src_selection_array = VArray<bool>::ForSingle(true, curves.points_num());
    }

    src_selection[selection_i] = src_selection_array;
    dst_selections[selection_i] = ensure_selection_attribute(
        new_curves,
        bke::AttrDomain::Point,
        src_selection_array.type().is<bool>() ? CD_PROP_BOOL : CD_PROP_FLOAT,
        selection_name);
  }

  const OffsetIndices<int> intervals_by_curve = curves_intervals_offsets.as_span();

  threading::parallel_for(curves.curves_range(), 256, [&](IndexRange curves_range) {
    for (const int curve : curves_range) {
      const int first_index = intervals_by_curve[curve].start();
      const int first_value = curves_intervals[first_index];
      bool is_selected = is_first_selected[curve];

      for (const int i : intervals_by_curve[curve].drop_back(1)) {
        const int dest_index = new_offsets[curve] + curves_intervals[i] - first_value + i -
                               first_index;
        const int size = curves_intervals[i + 1] - curves_intervals[i] + 1;

        for (const int selection_i : selection_attr_names.index_range()) {
          GMutableSpan dst_span = dst_selections[selection_i].span.slice(
              IndexRange(dest_index, size));
          if (is_selected) {
            src_selection[selection_i].type().copy_assign_n(
                src_selection[selection_i].slice(IndexRange(curves_intervals[i], size)).data(),
                dst_span.data(),
                size);
          }
          else {
            fill_selection(dst_span, false);
          }
        }

        is_selected = !is_selected;
      }
    }
  });

  for (const int selection_i : selection_attr_names.index_range()) {
    dst_selections[selection_i].finish();
  }

  const OffsetIndices<int> intervals = compress_intervals(intervals_by_curve, curves_intervals);

  bke::MutableAttributeAccessor dst_attributes = new_curves.attributes_for_write();

  for (auto &attribute : bke::retrieve_attributes_for_transfer(
           src_attributes,
           dst_attributes,
           ATTR_DOMAIN_MASK_POINT,
           bke::attribute_filter_from_skip_ref(
               {".selection", ".selection_handle_left", ".selection_handle_right"})))
  {
    const CPPType &type = attribute.src.type();
    threading::parallel_for(intervals.index_range(), 512, [&](IndexRange range) {
      for (const int i : range) {
        const IndexRange src = intervals[i].extend(1);
        const IndexRange dst = src.shift(i);
        type.copy_assign_n(
            attribute.src.slice(src).data(), attribute.dst.span.slice(dst).data(), src.size());
      }
    });
    attribute.dst.finish();
  }
  curves_id.geometry.wrap() = std::move(new_curves);
  DEG_id_tag_update(&curves_id.id, ID_RECALC_GEOMETRY);
}

static int curves_extrude_exec(bContext *C, wmOperator * /*op*/)
{
  for (Curves *curves_id : get_unique_editable_curves(*C)) {
    extrude_curves(*curves_id);
  }
  return OPERATOR_FINISHED;
}

void CURVES_OT_extrude(wmOperatorType *ot)
{
  ot->name = "Extrude";
  ot->description = "Extrude selected control point(s)";
  ot->idname = "CURVES_OT_extrude";

  ot->exec = curves_extrude_exec;
  ot->poll = editable_curves_in_edit_mode_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

}  // namespace blender::ed::curves
