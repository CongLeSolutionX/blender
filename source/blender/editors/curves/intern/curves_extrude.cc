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
  /* Skip the first curve, as all the data stays in the same place.
   * -1 to drop index denoting curve's right endpoint.
   */
  int *dst = intervals.data() + intervals_by_curve[0].size() - 1;

  for (const int curve : IndexRange(1, intervals_by_curve.size() - 1)) {
    const IndexRange range = intervals_by_curve[curve];
    /* -2 one to drop index denoting curve's beginning, second one for ending. */
    const int width = range.size() - 2;
    std::copy_n(src + range.first() + 1, width, dst);
    dst += width;
  }
  (*dst) = src[intervals_by_curve[intervals_by_curve.size() - 1].last()];
  return {intervals.data(), dst - intervals.data() + 1};
}

static void calc_curves_extrusion(const IndexMask &selection,
                                  const Span<int> offsets,
                                  MutableSpan<int> copy_interval_offsets,
                                  MutableSpan<int> curves_intervals_offsets,
                                  MutableSpan<bool> is_first_selected)
{
  Vector<int> selection_offsets;
  selection.foreach_range([&](const IndexRange range) {
    selection_offsets.append(range.first());
    selection_offsets.append(range.last());
  });
  selection_offsets.append(offsets.last() + 1);

  is_first_selected[0] = false;
  curves_intervals_offsets[0] = 0;

  int selection_i = 0;
  int curve_i = 0;
  bool right_endpoint = false;
  int dst = 0;
  bool is_selected = false;
  int prev_curve_endpoint = offsets.last();
  int curve_endpoint = offsets[curve_i] - right_endpoint;
  int selection_endpoint = selection_offsets[selection_i];

  while (curve_endpoint < offsets.last()) {
    if (selection_endpoint < curve_endpoint) {
      copy_interval_offsets[dst++] = selection_endpoint;
      selection_endpoint = selection_offsets[++selection_i];
      is_selected = !is_selected;
    }
    else {
      if (!right_endpoint) {
        curves_intervals_offsets[curve_i] = dst;
        is_first_selected[curve_i] = false;
      }
      bool duplicate = is_selected;
      if (selection_endpoint == curve_endpoint) {
        duplicate = true;
        const bool two_selection_steps = selection_endpoint == selection_offsets[++selection_i];
        selection_i += two_selection_steps;
        selection_endpoint = selection_offsets[selection_i];
        is_selected = two_selection_steps ? is_selected : !is_selected;
        if (!right_endpoint && !is_selected && (offsets[curve_i + 1] - offsets[curve_i] > 1)) {
          is_first_selected[curve_i] = true;
        }
      }
      copy_interval_offsets[dst] = curve_endpoint;
      dst += duplicate && prev_curve_endpoint != curve_endpoint;
      copy_interval_offsets[dst++] = curve_endpoint;
      right_endpoint = !right_endpoint;
      curve_i += right_endpoint;
      prev_curve_endpoint = curve_endpoint;
      curve_endpoint = offsets[curve_i] - right_endpoint;
    }
  }
  curves_intervals_offsets[curve_i] = dst;
}

static void calc_new_offsets(const Span<int> old_offsets,
                             const Span<int> curves_intervals_offsets,
                             MutableSpan<int> new_offsets)
{
  new_offsets[0] = 0;
  const IndexRange range = old_offsets.index_range().drop_back(1).shift(1);
  threading::parallel_for(range, 256, [&](IndexRange index_range) {
    for (const int i : index_range) {
      /* -1 subtracts last interval endpoint and gives number of intervals.
       * Another -1 from number of intervals gives number of new points created for curve.
       * Multiplied by i because -2 are accumulated for each curve.
       */
      new_offsets[i] = old_offsets[i] + curves_intervals_offsets[i] - 2 * i;
    }
  });
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

  bke::CurvesGeometry new_curves = bke::curves::copy_only_curve_domain(curves);

  const int curves_num = curves.curves_num();

  /* Buffer for intervals of all curves. Beginning and end of a curve can be determined only by
   * #curve_interval_ranges. For ex. [0, 3, 4, 4, 4] indicates one copy interval for first curve
   * [0, 3] and two for second [4, 4][4, 4]. The first curve will be copied as is without changes,
   * in the second one (consisting only one point - 4) first point will be duplicated (extruded).
   */
  Array<int> copy_interval_offsets(extruded_points.size() * 2 + curves_num * 2);

  /* Points to intervals for each curve in the copy_intervals array.
   * For example above value would be [0, 3, 5]. Meaning that [0 .. 2] are indices for curve 0 in
   * copy_intervals array, [3 .. 4] for curve 1. */
  Array<int> curves_intervals_offsets(curves_num + 1);

  /* Per curve boolean indicating if first interval in a curve is selected.
   * Other can be calculated as in a curve two adjacent intervals can not have same selection
   * state. */
  Array<bool> is_first_selected(curves_num);

  calc_curves_extrusion(extruded_points,
                        curves.offsets(),
                        copy_interval_offsets,
                        curves_intervals_offsets,
                        is_first_selected);

  MutableSpan<int> new_offsets = new_curves.offsets_for_write();
  calc_new_offsets(curves.offsets(), curves_intervals_offsets, new_offsets);
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
  const OffsetIndices<int> copy_intervals = copy_interval_offsets.as_span().slice(
      0, curves_intervals_offsets.last());

  threading::parallel_for(curves.curves_range(), 256, [&](IndexRange curves_range) {
    for (const int curve : curves_range) {
      const int first_index = intervals_by_curve[curve].start();
      const int first_value = copy_intervals[first_index].start();
      bool is_selected = is_first_selected[curve];

      for (const int i : intervals_by_curve[curve].drop_back(1)) {
        const IndexRange src = copy_intervals[i].extend_back(1);
        const IndexRange dst = src.shift(new_offsets[curve] - first_value + i - first_index);

        for (const int selection_i : selection_attr_names.index_range()) {
          GMutableSpan dst_span = dst_selections[selection_i].span.slice(dst);
          if (is_selected) {
            GSpan src_span = src_selection[selection_i].slice(src);
            src_selection[selection_i].type().copy_assign_n(
                src_span.data(), dst_span.data(), src.size());
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

  const OffsetIndices<int> compact_intervals = compress_intervals(intervals_by_curve,
                                                                  copy_interval_offsets);

  bke::MutableAttributeAccessor dst_attributes = new_curves.attributes_for_write();

  for (auto &attribute : bke::retrieve_attributes_for_transfer(
           src_attributes,
           dst_attributes,
           ATTR_DOMAIN_MASK_POINT,
           bke::attribute_filter_from_skip_ref(
               {".selection", ".selection_handle_left", ".selection_handle_right"})))
  {
    const CPPType &type = attribute.src.type();
    threading::parallel_for(compact_intervals.index_range(), 512, [&](IndexRange range) {
      for (const int i : range) {
        const IndexRange src = compact_intervals[i].extend_back(1);
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
