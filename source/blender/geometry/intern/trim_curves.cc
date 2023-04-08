/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BLI_array_utils.hh"
#include "BLI_length_parameterize.hh"
#include "BLI_math_solvers.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_curves_utils.hh"

#include "GEO_trim_curves.hh"

namespace blender::geometry {

using BisectCurveSequence = Array<Array<bke::curves::CurvePoint, 12>, 12>;

/* -------------------------------------------------------------------- */
/** \name Utility
 * \{ */

static float trim_sample_length(const Span<float> accumulated_lengths,
                                const float sample_length,
                                const GeometryNodeCurveSampleMode mode)
{
  float length = mode == GEO_NODE_CURVE_SAMPLE_FACTOR ?
                     sample_length * accumulated_lengths.last() :
                     sample_length;
  return std::clamp(length, 0.0f, accumulated_lengths.last());
}

static int compute_curve_shape(bke::curves::CurvePoint local_start_point,
                               bke::curves::CurvePoint local_end_point,
                               CurveType curve_type,
                               int point_count,
                               bke::curves::IndexRangeCyclic &r_local_src_sample_range)
{
  UNUSED_VARS(curve_type);
  r_local_src_sample_range = bke::curves::IndexRangeCyclic::get_range_between_endpoints(
      local_start_point, local_end_point, point_count);
  const int count = r_local_src_sample_range.size() + !local_start_point.is_controlpoint() +
                    !local_end_point.is_controlpoint();
  BLI_assert(count > 1);
  return count;
}

/* -------------------------------------------------------------------- */
/** \name Lookup Curve Points
 * \{ */

/**
 * Find the point on the curve defined by the distance along the curve. Assumes curve resolution is
 * constant for all curve segments and evaluated curve points are uniformly spaced between the
 * segment endpoints in relation to the curve parameter.
 *
 * \param lengths: Accumulated length for the evaluated curve.
 * \param sample_length: Distance along the curve to determine the #CurvePoint for.
 * \param cyclic: If curve is cyclic.
 * \param resolution: Curve resolution (number of evaluated points per segment).
 * \param num_curve_points: Total number of control points in the curve.
 * \return Point on the piecewise segment matching the given distance.
 */
static bke::curves::CurvePoint lookup_point_uniform_spacing(const Span<float> lengths,
                                                            const float sample_length,
                                                            const bool cyclic,
                                                            const int resolution,
                                                            const int num_curve_points)
{
  BLI_assert(!cyclic || lengths.size() / resolution >= 2);
  const int last_index = num_curve_points - 1;
  if (sample_length <= 0.0f) {
    return {{0, 1}, 0.0f};
  }
  if (sample_length >= lengths.last()) {
    return cyclic ? bke::curves::CurvePoint{{last_index, 0}, 1.0} :
                    bke::curves::CurvePoint{{last_index - 1, last_index}, 1.0};
  }
  int eval_index;
  float eval_factor;
  length_parameterize::sample_at_length(lengths, sample_length, eval_index, eval_factor);

  const int index = eval_index / resolution;
  const int next_index = (index == last_index) ? 0 : index + 1;
  const float parameter = (eval_factor + eval_index) / resolution - index;

  return bke::curves::CurvePoint{{index, next_index}, parameter};
}

/**
 * Find the point on the 'evaluated' polygonal curve.
 */
static bke::curves::CurvePoint lookup_point_polygonal(const Span<float> lengths,
                                                      const float sample_length,
                                                      const bool cyclic,
                                                      const int evaluated_size)
{
  const int last_index = evaluated_size - 1;
  if (sample_length <= 0.0f) {
    return {{0, 1}, 0.0f};
  }
  if (sample_length >= lengths.last()) {
    return cyclic ? bke::curves::CurvePoint{{last_index, 0}, 1.0} :
                    bke::curves::CurvePoint{{last_index - 1, last_index}, 1.0};
  }

  int eval_index;
  float eval_factor;
  length_parameterize::sample_at_length(lengths, sample_length, eval_index, eval_factor);

  const int next_eval_index = (eval_index == last_index) ? 0 : eval_index + 1;
  return bke::curves::CurvePoint{{eval_index, next_eval_index}, eval_factor};
}

/**
 * Find the point on a Bezier curve using the 'bezier_offsets' cache.
 */
static bke::curves::CurvePoint lookup_point_bezier(const Span<int> bezier_offsets,
                                                   const Span<float> lengths,
                                                   const float sample_length,
                                                   const bool cyclic,
                                                   const int num_curve_points)
{
  const int last_index = num_curve_points - 1;
  if (sample_length <= 0.0f) {
    return {{0, 1}, 0.0f};
  }
  if (sample_length >= lengths.last()) {
    return cyclic ? bke::curves::CurvePoint{{last_index, 0}, 1.0} :
                    bke::curves::CurvePoint{{last_index - 1, last_index}, 1.0};
  }
  int eval_index;
  float eval_factor;
  length_parameterize::sample_at_length(lengths, sample_length, eval_index, eval_factor);

  /* Find the segment index from the offset mapping. */
  const int *offset = std::upper_bound(bezier_offsets.begin(), bezier_offsets.end(), eval_index);
  const int left = offset - bezier_offsets.begin() - 1;
  const int right = left == last_index ? 0 : left + 1;

  const int prev_offset = bezier_offsets[left];
  const float offset_in_segment = eval_factor + (eval_index - prev_offset);
  const int segment_resolution = bezier_offsets[left + 1] - prev_offset;
  const float parameter = std::clamp(offset_in_segment / segment_resolution, 0.0f, 1.0f);

  return {{left, right}, parameter};
}

static bke::curves::CurvePoint lookup_point_bezier(
    const bke::CurvesGeometry &src_curves,
    const OffsetIndices<int> evaluated_points_by_curve,
    const int64_t curve_index,
    const Span<float> accumulated_lengths,
    const float sample_length,
    const bool cyclic,
    const int resolution,
    const int num_curve_points)
{
  if (bke::curves::bezier::has_vector_handles(
          num_curve_points, evaluated_points_by_curve[curve_index].size(), cyclic, resolution))
  {
    const Span<int> bezier_offsets = src_curves.bezier_evaluated_offsets_for_curve(curve_index);
    return lookup_point_bezier(
        bezier_offsets, accumulated_lengths, sample_length, cyclic, num_curve_points);
  }
  return lookup_point_uniform_spacing(
      accumulated_lengths, sample_length, cyclic, resolution, num_curve_points);
}

static bke::curves::CurvePoint lookup_curve_point(
    const bke::CurvesGeometry &src_curves,
    const OffsetIndices<int> evaluated_points_by_curve,
    const CurveType curve_type,
    const int64_t curve_index,
    const Span<float> accumulated_lengths,
    const float sample_length,
    const bool cyclic,
    const int resolution,
    const int num_curve_points)
{
  if (num_curve_points == 1) {
    return {{0, 0}, 0.0f};
  }

  if (curve_type == CURVE_TYPE_CATMULL_ROM) {
    return lookup_point_uniform_spacing(
        accumulated_lengths, sample_length, cyclic, resolution, num_curve_points);
  }
  else if (curve_type == CURVE_TYPE_BEZIER) {
    return lookup_point_bezier(src_curves,
                               evaluated_points_by_curve,
                               curve_index,
                               accumulated_lengths,
                               sample_length,
                               cyclic,
                               resolution,
                               num_curve_points);
  }
  else if (curve_type == CURVE_TYPE_POLY) {
    return lookup_point_polygonal(accumulated_lengths, sample_length, cyclic, num_curve_points);
  }
  /* Handle evaluated curve. */
  BLI_assert(resolution > 0);
  return lookup_point_polygonal(
      accumulated_lengths, sample_length, cyclic, evaluated_points_by_curve[curve_index].size());
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Utility Functions
 * \{ */

static void fill_bezier_data(bke::CurvesGeometry &dst_curves, const IndexMask &selection)
{
  if (!dst_curves.has_curve_with_type(CURVE_TYPE_BEZIER)) {
    return;
  }
  const OffsetIndices dst_points_by_curve = dst_curves.points_by_curve();
  MutableSpan<float3> handle_positions_left = dst_curves.handle_positions_left_for_write();
  MutableSpan<float3> handle_positions_right = dst_curves.handle_positions_right_for_write();
  MutableSpan<int8_t> handle_types_left = dst_curves.handle_types_left_for_write();
  MutableSpan<int8_t> handle_types_right = dst_curves.handle_types_right_for_write();

  selection.foreach_index(GrainSize(4096), [&](const int curve_i) {
    const IndexRange points = dst_points_by_curve[curve_i];
    handle_types_right.slice(points).fill(int8_t(BEZIER_HANDLE_FREE));
    handle_types_left.slice(points).fill(int8_t(BEZIER_HANDLE_FREE));
    handle_positions_left.slice(points).fill({0.0f, 0.0f, 0.0f});
    handle_positions_right.slice(points).fill({0.0f, 0.0f, 0.0f});
  });
}
static void fill_nurbs_data(bke::CurvesGeometry &dst_curves, const IndexMask &selection)
{
  if (!dst_curves.has_curve_with_type(CURVE_TYPE_NURBS)) {
    return;
  }
  bke::curves::fill_points(
      dst_curves.points_by_curve(), selection, 0.0f, dst_curves.nurbs_weights_for_write());
}

template<typename T>
static int64_t copy_point_data_between_endpoints(const Span<T> src_data,
                                                 MutableSpan<T> dst_data,
                                                 const bke::curves::IndexRangeCyclic src_range,
                                                 int64_t dst_index)
{
  int64_t increment;
  if (src_range.cycles()) {
    increment = src_range.size_before_loop();
    dst_data.slice(dst_index, increment).copy_from(src_data.slice(src_range.first(), increment));
    dst_index += increment;

    increment = src_range.size_after_loop();
    dst_data.slice(dst_index, increment)
        .copy_from(src_data.slice(src_range.curve_range().first(), increment));
    dst_index += increment;
  }
  else {
    increment = src_range.one_after_last() - int64_t(src_range.first());
    dst_data.slice(dst_index, increment).copy_from(src_data.slice(src_range.first(), increment));
    dst_index += increment;
  }
  return dst_index;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Sampling Utilities
 * \{ */

template<typename T>
static T interpolate_catmull_rom(const Span<T> src_data,
                                 const bke::curves::CurvePoint insertion_point,
                                 const bool src_cyclic)
{
  BLI_assert(insertion_point.index >= 0 && insertion_point.next_index < src_data.size());
  int i0;
  if (insertion_point.index == 0) {
    i0 = src_cyclic ? src_data.size() - 1 : insertion_point.index;
  }
  else {
    i0 = insertion_point.index - 1;
  }
  int i3 = insertion_point.next_index + 1;
  if (i3 == src_data.size()) {
    i3 = src_cyclic ? 0 : insertion_point.next_index;
  }
  return bke::curves::catmull_rom::interpolate<T>(src_data[i0],
                                                  src_data[insertion_point.index],
                                                  src_data[insertion_point.next_index],
                                                  src_data[i3],
                                                  insertion_point.parameter);
}

static bke::curves::bezier::Insertion knot_insert_bezier(
    const Span<float3> positions,
    const Span<float3> handles_left,
    const Span<float3> handles_right,
    const bke::curves::CurvePoint insertion_point)
{
  BLI_assert(
      insertion_point.index + 1 == insertion_point.next_index ||
      (insertion_point.next_index >= 0 && insertion_point.next_index < insertion_point.index));
  return bke::curves::bezier::insert(positions[insertion_point.index],
                                     handles_right[insertion_point.index],
                                     handles_left[insertion_point.next_index],
                                     positions[insertion_point.next_index],
                                     insertion_point.parameter);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Sample Curve Interval (Trim)
 * \{ */

/**
 * Sample source curve data in the interval defined by the points [start_point, end_point].
 * Uses linear interpolation to compute the endpoints.
 *
 * \tparam include_start_point: If False, the 'start_point' point sample will not be copied
 * and not accounted for in the destination range.
 * \param src_data: Source to sample from.
 * \param dst_data: Destination to write samples to.
 * \param src_range: Interval within [start_point, end_point] to copy from the source point domain.
 * \param dst_range: Interval to copy point data to in the destination buffer.
 * \param start_point: Point on the source curve to start sampling from.
 * \param end_point: Last point to sample in the source curve.
 */
template<typename T, bool include_start_point = true>
static void sample_interval_linear(const Span<T> src_data,
                                   MutableSpan<T> dst_data,
                                   bke::curves::IndexRangeCyclic src_range,
                                   const IndexRange dst_range,
                                   const bke::curves::CurvePoint start_point,
                                   const bke::curves::CurvePoint end_point)
{
  int64_t dst_index = dst_range.first();

  if (start_point.is_controlpoint()) {
    /* 'start_point' is included in the copy iteration. */
    if constexpr (!include_start_point) {
      /* Skip first. */
      src_range = src_range.drop_front();
    }
  }
  else if constexpr (!include_start_point) {
    /* Do nothing (excluded). */
  }
  else {
    /* General case, sample 'start_point' */
    dst_data[dst_index] = bke::attribute_math::mix2(
        start_point.parameter, src_data[start_point.index], src_data[start_point.next_index]);
    ++dst_index;
  }

  dst_index = copy_point_data_between_endpoints(src_data, dst_data, src_range, dst_index);
  if (dst_range.size() == 1) {
    BLI_assert(dst_index == dst_range.one_after_last());
    return;
  }

  /* Handle last case */
  if (end_point.is_controlpoint()) {
    /* 'end_point' is included in the copy iteration. */
  }
  else {
    dst_data[dst_index] = bke::attribute_math::mix2(
        end_point.parameter, src_data[end_point.index], src_data[end_point.next_index]);
#ifndef NDEBUG
    ++dst_index;
#endif
  }
  BLI_assert(dst_index == dst_range.one_after_last());
}

template<typename T>
static void sample_interval_catmull_rom(const Span<T> src_data,
                                        MutableSpan<T> dst_data,
                                        bke::curves::IndexRangeCyclic src_range,
                                        const IndexRange dst_range,
                                        const bke::curves::CurvePoint start_point,
                                        const bke::curves::CurvePoint end_point,
                                        const bool src_cyclic)
{
  int64_t dst_index = dst_range.first();

  if (start_point.is_controlpoint()) {
  }
  else {
    /* General case, sample 'start_point' */
    dst_data[dst_index] = interpolate_catmull_rom(src_data, start_point, src_cyclic);
    ++dst_index;
  }

  dst_index = copy_point_data_between_endpoints(src_data, dst_data, src_range, dst_index);
  if (dst_range.size() == 1) {
    BLI_assert(dst_index == dst_range.one_after_last());
    return;
  }

  /* Handle last case */
  if (end_point.is_controlpoint()) {
    /* 'end_point' is included in the copy iteration. */
  }
  else {
    dst_data[dst_index] = interpolate_catmull_rom(src_data, end_point, src_cyclic);
#ifndef NDEBUG
    ++dst_index;
#endif
  }
  BLI_assert(dst_index == dst_range.one_after_last());
}

template<bool include_start_point = true>
static void sample_interval_bezier(const Span<float3> src_positions,
                                   const Span<float3> src_handles_l,
                                   const Span<float3> src_handles_r,
                                   const Span<int8_t> src_types_l,
                                   const Span<int8_t> src_types_r,
                                   MutableSpan<float3> dst_positions,
                                   MutableSpan<float3> dst_handles_l,
                                   MutableSpan<float3> dst_handles_r,
                                   MutableSpan<int8_t> dst_types_l,
                                   MutableSpan<int8_t> dst_types_r,
                                   bke::curves::IndexRangeCyclic src_range,
                                   const IndexRange dst_range,
                                   const bke::curves::CurvePoint start_point,
                                   const bke::curves::CurvePoint end_point)
{
  bke::curves::bezier::Insertion start_point_insert;
  int64_t dst_index = dst_range.first();

  bool start_point_trimmed = false;
  if (start_point.is_controlpoint()) {
    /* The 'start_point' control point is included in the copy iteration. */
    if constexpr (!include_start_point) {
      src_range = src_range.drop_front();
    }
  }
  else if constexpr (!include_start_point) {
    /* Do nothing, 'start_point' is excluded. */
  }
  else {
    /* General case, sample 'start_point'. */
    start_point_insert = knot_insert_bezier(
        src_positions, src_handles_l, src_handles_r, start_point);
    dst_positions[dst_range.first()] = start_point_insert.position;
    dst_handles_l[dst_range.first()] = start_point_insert.left_handle;
    dst_handles_r[dst_range.first()] = start_point_insert.right_handle;
    dst_types_l[dst_range.first()] = src_types_l[start_point.index];
    dst_types_r[dst_range.first()] = src_types_r[start_point.index];

    start_point_trimmed = true;
    ++dst_index;
  }

  /* Copy point data between the 'start_point' and 'end_point'. */
  int64_t increment = src_range.cycles() ? src_range.size_before_loop() :
                                           src_range.one_after_last() - src_range.first();

  const IndexRange dst_range_to_end(dst_index, increment);
  const IndexRange src_range_to_end(src_range.first(), increment);
  dst_positions.slice(dst_range_to_end).copy_from(src_positions.slice(src_range_to_end));
  dst_handles_l.slice(dst_range_to_end).copy_from(src_handles_l.slice(src_range_to_end));
  dst_handles_r.slice(dst_range_to_end).copy_from(src_handles_r.slice(src_range_to_end));
  dst_types_l.slice(dst_range_to_end).copy_from(src_types_l.slice(src_range_to_end));
  dst_types_r.slice(dst_range_to_end).copy_from(src_types_r.slice(src_range_to_end));
  dst_index += increment;

  if (dst_range.size() == 1) {
    BLI_assert(dst_index == dst_range.one_after_last());
    return;
  }

  increment = src_range.size_after_loop();
  if (src_range.cycles() && increment > 0) {
    const IndexRange dst_range_looped(dst_index, increment);
    const IndexRange src_range_looped(src_range.curve_range().first(), increment);
    dst_positions.slice(dst_range_looped).copy_from(src_positions.slice(src_range_looped));
    dst_handles_l.slice(dst_range_looped).copy_from(src_handles_l.slice(src_range_looped));
    dst_handles_r.slice(dst_range_looped).copy_from(src_handles_r.slice(src_range_looped));
    dst_types_l.slice(dst_range_looped).copy_from(src_types_l.slice(src_range_looped));
    dst_types_r.slice(dst_range_looped).copy_from(src_types_r.slice(src_range_looped));
    dst_index += increment;
  }

  if (start_point_trimmed) {
    dst_handles_l[dst_range.first() + 1] = start_point_insert.handle_next;
    /* No need to change handle type (remains the same). */
  }

  /* Handle 'end_point' */
  bke::curves::bezier::Insertion end_point_insert;
  if (end_point.parameter == 0.0f) {
    if (end_point.index == start_point.index) {
      /* Start point is same point or in the same segment. */
      if (start_point.parameter == 0.0f) {
        /* Same point. */
        BLI_assert(dst_range.size() == 1LL + src_range.size_range());
        dst_handles_l[dst_range.first()] = dst_positions[dst_range.first()];
        dst_handles_r[dst_range.last()] = dst_positions[dst_range.first()];
      }
      else if (start_point.parameter == 1.0f) {
        /* Start is next controlpoint, do nothing. */
      }
      else {
        /* Within the segment. */
        BLI_assert(dst_range.size() == 1LL + src_range.size_range() || dst_range.size() == 2);
        dst_handles_r[dst_range.last()] = start_point_insert.handle_prev;
      }
    }
    /* Start point is considered 'before' the endpoint and ignored. */
  }
  else if (end_point.parameter == 1.0f) {
    if (end_point.next_index == start_point.index) {
      /* Start point is same or in 'next' segment. */
      if (start_point.parameter == 0.0f) {
        /* Same point */
        BLI_assert(dst_range.size() == 1LL + src_range.size_range());
        dst_handles_l[dst_range.first()] = dst_positions[dst_range.first()];
        dst_handles_r[dst_range.last()] = dst_positions[dst_range.first()];
      }
      else if (start_point.parameter == 1.0f) {
        /* Start is next controlpoint, do nothing. */
      }
      else {
        /* In next segment. */
        BLI_assert(dst_range.size() == 1LL + src_range.size_range() || dst_range.size() == 2);
        dst_handles_r[dst_range.last()] = start_point_insert.handle_prev;
      }
    }
  }
  else {
    /* Trimmed in both ends within the same (and only) segment! Ensure both end points is not a
     * loop. */
    if (start_point.index == end_point.index && start_point.parameter < 1.0f) {
      BLI_assert(dst_range.size() == 2 || dst_range.size() == 2ll + src_range.size_range() ||
                 dst_range.size() == 1LL + src_range.size_range());

      if (start_point.parameter > end_point.parameter && start_point.parameter < 1.0f) {
        /* Start point comes after the endpoint within the segment. */
        BLI_assert(end_point.parameter >= 0.0f);

        const float parameter = end_point.parameter / start_point.parameter;
        end_point_insert = bke::curves::bezier::insert(dst_positions[dst_index - 1],
                                                       start_point_insert.handle_prev,
                                                       start_point_insert.left_handle,
                                                       start_point_insert.position,
                                                       parameter);

        /* Update start-point handle. */
        dst_handles_l[dst_range.first()] = end_point_insert.handle_next;
      }
      else {
        /* Start point lies before the endpoint within the segment. */

        const float parameter = (end_point.parameter - start_point.parameter) /
                                (1.0f - start_point.parameter);
        /* Unused only when parameter == 0.0f! */
        const float3 handle_next = start_point.parameter == 0.0f ?
                                       src_handles_l[end_point.next_index] :
                                       start_point_insert.handle_next;
        end_point_insert = bke::curves::bezier::insert(dst_positions[dst_index - 1],
                                                       dst_handles_r[dst_index - 1],
                                                       handle_next,
                                                       src_positions[end_point.next_index],
                                                       parameter);
      }
    }
    else {
      /* General case, compute the insertion point. */
      end_point_insert = knot_insert_bezier(
          src_positions, src_handles_l, src_handles_r, end_point);

      if ((start_point.parameter >= end_point.parameter && end_point.index == start_point.index) ||
          (start_point.parameter == 0.0f && end_point.next_index == start_point.index))
      {
        /* Start point is next controlpoint. */
        dst_handles_l[dst_range.first()] = end_point_insert.handle_next;
        /* No need to change handle type (remains the same). */
      }
    }

    dst_handles_r[dst_index - 1] = end_point_insert.handle_prev;
    dst_types_r[dst_index - 1] = src_types_l[end_point.index];

    dst_handles_l[dst_index] = end_point_insert.left_handle;
    dst_handles_r[dst_index] = end_point_insert.right_handle;
    dst_positions[dst_index] = end_point_insert.position;
    dst_types_l[dst_index] = src_types_l[end_point.next_index];
    dst_types_r[dst_index] = src_types_r[end_point.next_index];
#ifndef NDEBUG
    ++dst_index;
#endif
  }
  BLI_assert(dst_index == dst_range.one_after_last());
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Trim Curves
 * \{ */

static void trim_attribute_linear(const bke::CurvesGeometry &src_curves,
                                  bke::CurvesGeometry &dst_curves,
                                  const IndexMask &selection,
                                  const Span<bke::curves::CurvePoint> start_points,
                                  const Span<bke::curves::CurvePoint> end_points,
                                  const Span<bke::curves::IndexRangeCyclic> src_ranges,
                                  MutableSpan<bke::AttributeTransferData> transfer_attributes)
{
  const OffsetIndices src_points_by_curve = src_curves.points_by_curve();
  const OffsetIndices dst_points_by_curve = dst_curves.points_by_curve();
  for (bke::AttributeTransferData &attribute : transfer_attributes) {
    bke::attribute_math::convert_to_static_type(attribute.meta_data.data_type, [&](auto dummy) {
      using T = decltype(dummy);

      selection.foreach_index(GrainSize(512), [&](const int curve_i) {
        const IndexRange src_points = src_points_by_curve[curve_i];
        sample_interval_linear<T>(attribute.src.template typed<T>().slice(src_points),
                                  attribute.dst.span.typed<T>(),
                                  src_ranges[curve_i],
                                  dst_points_by_curve[curve_i],
                                  start_points[curve_i],
                                  end_points[curve_i]);
      });
    });
  }
}

static void trim_polygonal_curves(const bke::CurvesGeometry &src_curves,
                                  bke::CurvesGeometry &dst_curves,
                                  const IndexMask &selection,
                                  const Span<bke::curves::CurvePoint> start_points,
                                  const Span<bke::curves::CurvePoint> end_points,
                                  const Span<bke::curves::IndexRangeCyclic> src_ranges,
                                  MutableSpan<bke::AttributeTransferData> transfer_attributes)
{
  const OffsetIndices src_points_by_curve = src_curves.points_by_curve();
  const OffsetIndices dst_points_by_curve = dst_curves.points_by_curve();
  const Span<float3> src_positions = src_curves.positions();
  MutableSpan<float3> dst_positions = dst_curves.positions_for_write();

  selection.foreach_index(GrainSize(512), [&](const int curve_i) {
    const IndexRange src_points = src_points_by_curve[curve_i];
    const IndexRange dst_points = dst_points_by_curve[curve_i];

    sample_interval_linear<float3>(src_positions.slice(src_points),
                                   dst_positions,
                                   src_ranges[curve_i],
                                   dst_points,
                                   start_points[curve_i],
                                   end_points[curve_i]);
  });
  fill_bezier_data(dst_curves, selection);
  fill_nurbs_data(dst_curves, selection);
  trim_attribute_linear(src_curves,
                        dst_curves,
                        selection,
                        start_points,
                        end_points,
                        src_ranges,
                        transfer_attributes);
}

static void trim_catmull_rom_curves(const bke::CurvesGeometry &src_curves,
                                    bke::CurvesGeometry &dst_curves,
                                    const IndexMask &selection,
                                    const Span<bke::curves::CurvePoint> start_points,
                                    const Span<bke::curves::CurvePoint> end_points,
                                    const Span<bke::curves::IndexRangeCyclic> src_ranges,
                                    MutableSpan<bke::AttributeTransferData> transfer_attributes)
{
  const OffsetIndices src_points_by_curve = src_curves.points_by_curve();
  const OffsetIndices dst_points_by_curve = dst_curves.points_by_curve();
  const Span<float3> src_positions = src_curves.positions();
  const VArray<bool> src_cyclic = src_curves.cyclic();
  MutableSpan<float3> dst_positions = dst_curves.positions_for_write();

  selection.foreach_index(GrainSize(512), [&](const int curve_i) {
    const IndexRange src_points = src_points_by_curve[curve_i];
    const IndexRange dst_points = dst_points_by_curve[curve_i];

    sample_interval_catmull_rom<float3>(src_positions.slice(src_points),
                                        dst_positions,
                                        src_ranges[curve_i],
                                        dst_points,
                                        start_points[curve_i],
                                        end_points[curve_i],
                                        src_cyclic[curve_i]);
  });
  fill_bezier_data(dst_curves, selection);
  fill_nurbs_data(dst_curves, selection);

  for (bke::AttributeTransferData &attribute : transfer_attributes) {
    bke::attribute_math::convert_to_static_type(attribute.meta_data.data_type, [&](auto dummy) {
      using T = decltype(dummy);

      selection.foreach_index(GrainSize(512), [&](const int curve_i) {
        const IndexRange src_points = src_points_by_curve[curve_i];
        const IndexRange dst_points = dst_points_by_curve[curve_i];

        sample_interval_catmull_rom<T>(attribute.src.template typed<T>().slice(src_points),
                                       attribute.dst.span.typed<T>(),
                                       src_ranges[curve_i],
                                       dst_points,
                                       start_points[curve_i],
                                       end_points[curve_i],
                                       src_cyclic[curve_i]);
      });
    });
  }
}

static void trim_bezier_curves(const bke::CurvesGeometry &src_curves,
                               bke::CurvesGeometry &dst_curves,
                               const IndexMask &selection,
                               const Span<bke::curves::CurvePoint> start_points,
                               const Span<bke::curves::CurvePoint> end_points,
                               const Span<bke::curves::IndexRangeCyclic> src_ranges,
                               MutableSpan<bke::AttributeTransferData> transfer_attributes)
{
  const OffsetIndices src_points_by_curve = src_curves.points_by_curve();
  const Span<float3> src_positions = src_curves.positions();
  const VArraySpan<int8_t> src_types_l{src_curves.handle_types_left()};
  const VArraySpan<int8_t> src_types_r{src_curves.handle_types_right()};
  const Span<float3> src_handles_l = src_curves.handle_positions_left();
  const Span<float3> src_handles_r = src_curves.handle_positions_right();

  const OffsetIndices dst_points_by_curve = dst_curves.points_by_curve();
  MutableSpan<float3> dst_positions = dst_curves.positions_for_write();
  MutableSpan<int8_t> dst_types_l = dst_curves.handle_types_left_for_write();
  MutableSpan<int8_t> dst_types_r = dst_curves.handle_types_right_for_write();
  MutableSpan<float3> dst_handles_l = dst_curves.handle_positions_left_for_write();
  MutableSpan<float3> dst_handles_r = dst_curves.handle_positions_right_for_write();

  selection.foreach_index(GrainSize(512), [&](const int curve_i) {
    const IndexRange src_points = src_points_by_curve[curve_i];
    const IndexRange dst_points = dst_points_by_curve[curve_i];

    sample_interval_bezier(src_positions.slice(src_points),
                           src_handles_l.slice(src_points),
                           src_handles_r.slice(src_points),
                           src_types_l.slice(src_points),
                           src_types_r.slice(src_points),
                           dst_positions,
                           dst_handles_l,
                           dst_handles_r,
                           dst_types_l,
                           dst_types_r,
                           src_ranges[curve_i],
                           dst_points,
                           start_points[curve_i],
                           end_points[curve_i]);
  });
  fill_nurbs_data(dst_curves, selection);
  trim_attribute_linear(src_curves,
                        dst_curves,
                        selection,
                        start_points,
                        end_points,
                        src_ranges,
                        transfer_attributes);
}

static void trim_evaluated_curves(const bke::CurvesGeometry &src_curves,
                                  bke::CurvesGeometry &dst_curves,
                                  const IndexMask &selection,
                                  const Span<bke::curves::CurvePoint> start_points,
                                  const Span<bke::curves::CurvePoint> end_points,
                                  const Span<bke::curves::IndexRangeCyclic> src_ranges,
                                  MutableSpan<bke::AttributeTransferData> transfer_attributes)
{
  const OffsetIndices src_points_by_curve = src_curves.points_by_curve();
  const OffsetIndices src_evaluated_points_by_curve = src_curves.evaluated_points_by_curve();
  const OffsetIndices dst_points_by_curve = dst_curves.points_by_curve();
  const Span<float3> src_eval_positions = src_curves.evaluated_positions();
  MutableSpan<float3> dst_positions = dst_curves.positions_for_write();

  selection.foreach_index(GrainSize(512), [&](const int curve_i) {
    const IndexRange src_evaluated_points = src_evaluated_points_by_curve[curve_i];
    const IndexRange dst_points = dst_points_by_curve[curve_i];
    sample_interval_linear<float3>(src_eval_positions.slice(src_evaluated_points),
                                   dst_positions,
                                   src_ranges[curve_i],
                                   dst_points,
                                   start_points[curve_i],
                                   end_points[curve_i]);
  });
  fill_bezier_data(dst_curves, selection);
  fill_nurbs_data(dst_curves, selection);

  for (bke::AttributeTransferData &attribute : transfer_attributes) {
    bke::attribute_math::convert_to_static_type(attribute.meta_data.data_type, [&](auto dummy) {
      using T = decltype(dummy);

      selection.foreach_segment(GrainSize(512), [&](const IndexMaskSegment segment) {
        Vector<std::byte> evaluated_buffer;
        for (const int64_t curve_i : segment) {
          const IndexRange src_points = src_points_by_curve[curve_i];

          /* Interpolate onto the evaluated point domain and sample the evaluated domain. */
          evaluated_buffer.reinitialize(sizeof(T) * src_evaluated_points_by_curve[curve_i].size());
          MutableSpan<T> evaluated = evaluated_buffer.as_mutable_span().cast<T>();
          src_curves.interpolate_to_evaluated(curve_i, attribute.src.slice(src_points), evaluated);
          sample_interval_linear<T>(evaluated,
                                    attribute.dst.span.typed<T>(),
                                    src_ranges[curve_i],
                                    dst_points_by_curve[curve_i],
                                    start_points[curve_i],
                                    end_points[curve_i]);
        }
      });
    });
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Bisect Curves
 * \{ */

static void bisect_attribute_linear(const bke::CurvesGeometry &src_curves,
                                    bke::CurvesGeometry &dst_curves,
                                    const IndexMask selection,
                                    const Span<BisectCurveSequence> bisect_sequences,
                                    const Span<int> curve_offsets,
                                    MutableSpan<bke::AttributeTransferData> transfer_attributes,
                                    bool sample_evaluated = false)
{
  for (bke::AttributeTransferData &attribute : transfer_attributes) {
    attribute_math::convert_to_static_type(attribute.meta_data.data_type, [&](auto dummy) {
      using T = decltype(dummy);

      threading::parallel_for(selection.index_range(), 512, [&](const IndexRange range) {
        for (const int64_t src_curve_i : selection.slice(range)) {
          /* Compute bisections */
          int dst_curve_i = curve_offsets[src_curve_i];

          for (const Array<bke::curves::CurvePoint, 12> &sequence :
               bisect_sequences[src_curve_i]) {
            BLI_assert(sequence.size() > 1);
            bke::curves::IndexRangeCyclic src_range;
            const IndexRange dst_points = dst_curves.points_for_curve(dst_curve_i);

            Span<T> attribute_span;
            const IndexRange src_points = sample_evaluated ?
                                              src_curves.evaluated_points_for_curve(src_curve_i) :
                                              src_curves.points_for_curve(src_curve_i);
            GArray<> evaluated_data;
            GMutableSpan evaluated_span;
            if (sample_evaluated) {
              evaluated_data = GArray<>(CPPType::get<T>(), src_points.size());
              evaluated_span = evaluated_data.as_mutable_span();
              src_curves.interpolate_to_evaluated(
                  src_curve_i,
                  attribute.src.slice(src_curves.points_for_curve(src_curve_i)),
                  evaluated_span);
              attribute_span = evaluated_span.typed<T>();
            }
            else {
              attribute_span = attribute.src.template typed<T>().slice(src_points);
            }

            /* Initial case, include start sample. */
            int sequence_size = compute_curve_shape(
                sequence[0], sequence[1], CURVE_TYPE_BEZIER, src_points.size(), src_range);
            sample_interval_linear<T, true>(attribute_span,
                                            attribute.dst.span.typed<T>(),
                                            src_range,
                                            dst_points.slice(0, sequence_size),
                                            sequence[0],
                                            sequence[1]);

            int sampled_total = sequence_size;
            for (const int64_t index : IndexRange(2, sequence.size() - 2)) {
              /* Sample following sequences. */
              sequence_size = -1 + compute_curve_shape(sequence[index - 1],
                                                       sequence[index],
                                                       CURVE_TYPE_BEZIER,
                                                       src_points.size(),
                                                       src_range);
              sample_interval_linear<T, false>(attribute_span,
                                               attribute.dst.span.typed<T>(),
                                               src_range,
                                               dst_points.slice(sampled_total, sequence_size),
                                               sequence[index - 1],
                                               sequence[index]);
              sampled_total += sequence_size;
            }
            dst_curve_i++;
          }
        }
      });
    });
  }
}

static void bisect_polygonal_curves(const bke::CurvesGeometry &src_curves,
                                    bke::CurvesGeometry &dst_curves,
                                    const IndexMask selection,
                                    const Span<BisectCurveSequence> bisect_sequences,
                                    const Span<int> curve_offsets,
                                    MutableSpan<bke::AttributeTransferData> transfer_attributes,
                                    bool sample_evaluated = false)
{
  const Span<float3> src_positions = sample_evaluated ? src_curves.evaluated_positions() :
                                                        src_curves.positions();
  MutableSpan<float3> dst_positions = dst_curves.positions_for_write();

  threading::parallel_for(selection.index_range(), 512, [&](const IndexRange range) {
    Vector<int> dst_selection;
    for (const int64_t src_curve_i : selection.slice(range)) {
      /* Compute bisections */
      int dst_curve_i = curve_offsets[src_curve_i];
      const IndexRange src_points = sample_evaluated ?
                                        src_curves.evaluated_points_for_curve(src_curve_i) :
                                        src_curves.points_for_curve(src_curve_i);

      for (const Array<bke::curves::CurvePoint, 12> &sequence : bisect_sequences[src_curve_i]) {
        BLI_assert(sequence.size() > 1);
        bke::curves::IndexRangeCyclic src_range;
        const IndexRange dst_points = dst_curves.points_for_curve(dst_curve_i);

        /* Initial case, include start sample. */
        int sequence_size = compute_curve_shape(
            sequence[0], sequence[1], CURVE_TYPE_BEZIER, src_points.size(), src_range);
        sample_interval_linear<float3, true>(src_positions.slice(src_points),
                                             dst_positions,
                                             src_range,
                                             dst_points.slice(0, sequence_size),
                                             sequence[0],
                                             sequence[1]);

        int sampled_total = sequence_size;
        for (const int64_t index : IndexRange(2, sequence.size() - 2)) {
          /* Sample following sequences. */
          sequence_size = -1 + compute_curve_shape(sequence[index - 1],
                                                   sequence[index],
                                                   CURVE_TYPE_BEZIER,
                                                   src_points.size(),
                                                   src_range);
          sample_interval_linear<float3, false>(src_positions.slice(src_points),
                                                dst_positions,
                                                src_range,
                                                dst_points.slice(sampled_total, sequence_size),
                                                sequence[index - 1],
                                                sequence[index]);
          sampled_total += sequence_size;
        }
        dst_curve_i++;
      }
      IndexRange dst_curves_range{curve_offsets[src_curve_i],
                                  int64_t(dst_curve_i) - curve_offsets[src_curve_i]};
      fill_bezier_data(dst_curves, dst_curves_range);
      fill_nurbs_data(dst_curves, dst_curves_range);
    }
  });
  bisect_attribute_linear(src_curves,
                          dst_curves,
                          selection,
                          bisect_sequences,
                          curve_offsets,
                          transfer_attributes,
                          sample_evaluated);
}

static void bisect_bezier_curves(const bke::CurvesGeometry &src_curves,
                                 bke::CurvesGeometry &dst_curves,
                                 const IndexMask selection,
                                 const Span<BisectCurveSequence> bisect_sequences,
                                 const Span<int> curve_offsets,
                                 MutableSpan<bke::AttributeTransferData> transfer_attributes)
{
  const Span<float3> src_positions = src_curves.positions();
  const VArraySpan<int8_t> src_types_l{src_curves.handle_types_left()};
  const VArraySpan<int8_t> src_types_r{src_curves.handle_types_right()};
  const Span<float3> src_handles_l = src_curves.handle_positions_left();
  const Span<float3> src_handles_r = src_curves.handle_positions_right();

  MutableSpan<float3> dst_positions = dst_curves.positions_for_write();
  MutableSpan<int8_t> dst_types_l = dst_curves.handle_types_left_for_write();
  MutableSpan<int8_t> dst_types_r = dst_curves.handle_types_right_for_write();
  MutableSpan<float3> dst_handles_l = dst_curves.handle_positions_left_for_write();
  MutableSpan<float3> dst_handles_r = dst_curves.handle_positions_right_for_write();

  threading::parallel_for(selection.index_range(), 512, [&](const IndexRange range) {
    for (const int64_t src_curve_i : selection.slice(range)) {
      /* Compute bisections */
      const IndexRange src_points = src_curves.points_for_curve(src_curve_i);
      int dst_curve_i = curve_offsets[src_curve_i];

      for (const Array<bke::curves::CurvePoint, 12> &sequence : bisect_sequences[src_curve_i]) {
        BLI_assert(sequence.size() > 1);
        bke::curves::IndexRangeCyclic src_range;
        const IndexRange dst_points = dst_curves.points_for_curve(dst_curve_i);

        /* Initial case, include start sample. */
        int sequence_size = compute_curve_shape(
            sequence[0], sequence[1], CURVE_TYPE_BEZIER, src_points.size(), src_range);
        sample_interval_bezier<true>(src_positions.slice(src_points),
                                     src_handles_l.slice(src_points),
                                     src_handles_r.slice(src_points),
                                     src_types_l.slice(src_points),
                                     src_types_r.slice(src_points),
                                     dst_positions,
                                     dst_handles_l,
                                     dst_handles_r,
                                     dst_types_l,
                                     dst_types_r,
                                     src_range,
                                     dst_points.slice(0, sequence_size),
                                     sequence[0],
                                     sequence[1]);

        int sampled_total = sequence_size;
        for (const int64_t index : IndexRange(2, sequence.size() - 2)) {
          /* Sample following sequences. */
          sequence_size = -1 + compute_curve_shape(sequence[index - 1],
                                                   sequence[index],
                                                   CURVE_TYPE_BEZIER,
                                                   src_points.size(),
                                                   src_range);
          sample_interval_bezier<false>(src_positions.slice(src_points),
                                        src_handles_l.slice(src_points),
                                        src_handles_r.slice(src_points),
                                        src_types_l.slice(src_points),
                                        src_types_r.slice(src_points),
                                        dst_positions,
                                        dst_handles_l,
                                        dst_handles_r,
                                        dst_types_l,
                                        dst_types_r,
                                        src_range,
                                        dst_points.slice(sampled_total, sequence_size),
                                        sequence[index - 1],
                                        sequence[index]);
          sampled_total += sequence_size;
        }
        dst_curve_i++;
      }
      IndexRange dst_curves_range{curve_offsets[src_curve_i],
                                  int(dst_curve_i - curve_offsets[src_curve_i])};
      fill_nurbs_data(dst_curves, dst_curves_range);
    }
  });
  bisect_attribute_linear(
      src_curves, dst_curves, selection, bisect_sequences, curve_offsets, transfer_attributes);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Compute trim parameters
 * \{ */

/**
 * Compute the selected range of points for every selected curve.
 */
static void compute_curve_trim_parameters(const bke::CurvesGeometry &curves,
                                          const IndexMask &selection,
                                          const VArray<float> &starts,
                                          const VArray<float> &ends,
                                          const GeometryNodeCurveSampleMode mode,
                                          MutableSpan<int> dst_curve_size,
                                          MutableSpan<bke::curves::CurvePoint> start_points,
                                          MutableSpan<bke::curves::CurvePoint> end_points,
                                          MutableSpan<bke::curves::IndexRangeCyclic> src_ranges)
{
  const OffsetIndices points_by_curve = curves.points_by_curve();
  const OffsetIndices evaluated_points_by_curve = curves.evaluated_points_by_curve();
  const VArray<bool> src_cyclic = curves.cyclic();
  const VArray<int> resolution = curves.resolution();
  const VArray<int8_t> curve_types = curves.curve_types();
  curves.ensure_can_interpolate_to_evaluated();

  selection.foreach_index(GrainSize(128), [&](const int curve_i) {
    CurveType curve_type = CurveType(curve_types[curve_i]);

    int point_count;
    if (curve_type == CURVE_TYPE_NURBS) {
      /* The result curve is a poly curve. */
      point_count = evaluated_points_by_curve[curve_i].size();
    }
    else {
      point_count = points_by_curve[curve_i].size();
    }
    if (point_count == 1) {
      /* Single point. */
      dst_curve_size[curve_i] = 1;
      src_ranges[curve_i] = bke::curves::IndexRangeCyclic(0, 0, 1, 1);
      start_points[curve_i] = {{0, 0}, 0.0f};
      end_points[curve_i] = {{0, 0}, 0.0f};
      return;
    }

    const bool cyclic = src_cyclic[curve_i];
    const Span<float> lengths = curves.evaluated_lengths_for_curve(curve_i, cyclic);
    BLI_assert(lengths.size() > 0);

    const float start_length = trim_sample_length(lengths, starts[curve_i], mode);
    float end_length;

    bool equal_sample_point;
    if (cyclic) {
      end_length = trim_sample_length(lengths, ends[curve_i], mode);
      const float cyclic_start = start_length == lengths.last() ? 0.0f : start_length;
      const float cyclic_end = end_length == lengths.last() ? 0.0f : end_length;
      equal_sample_point = cyclic_start == cyclic_end;
    }
    else {
      end_length = ends[curve_i] <= starts[curve_i] ?
                       start_length :
                       trim_sample_length(lengths, ends[curve_i], mode);
      equal_sample_point = start_length == end_length;
    }

    start_points[curve_i] = lookup_curve_point(curves,
                                               evaluated_points_by_curve,
                                               curve_type,
                                               curve_i,
                                               lengths,
                                               start_length,
                                               cyclic,
                                               resolution[curve_i],
                                               point_count);
    if (equal_sample_point) {
      end_points[curve_i] = start_points[curve_i];
      if (end_length <= start_length) {
        /* Single point. */
        dst_curve_size[curve_i] = 1;
        if (start_points[curve_i].is_controlpoint()) {
          /* Only iterate if control point. */
          const int single_point_index = start_points[curve_i].parameter == 1.0f ?
                                             start_points[curve_i].next_index :
                                             start_points[curve_i].index;
          src_ranges[curve_i] = bke::curves::IndexRangeCyclic::get_range_from_size(
              single_point_index, 1, point_count);
        }
        /* else: leave empty range */
      }
      else {
        /* Split. */
        src_ranges[curve_i] = bke::curves::IndexRangeCyclic::get_range_between_endpoints(
                                  start_points[curve_i], end_points[curve_i], point_count)
                                  .push_loop();
        const int count = 1 + !start_points[curve_i].is_controlpoint() + point_count;
        BLI_assert(count > 1);
        dst_curve_size[curve_i] = count;
      }
    }
    else {
      /* General case. */
      end_points[curve_i] = lookup_curve_point(curves,
                                               evaluated_points_by_curve,
                                               curve_type,
                                               curve_i,
                                               lengths,
                                               end_length,
                                               cyclic,
                                               resolution[curve_i],
                                               point_count);

      src_ranges[curve_i] = bke::curves::IndexRangeCyclic::get_range_between_endpoints(
          start_points[curve_i], end_points[curve_i], point_count);
      const int count = src_ranges[curve_i].size() + !start_points[curve_i].is_controlpoint() +
                        !end_points[curve_i].is_controlpoint();
      BLI_assert(count > 1);
      dst_curve_size[curve_i] = count;
    }
    BLI_assert(dst_curve_size[curve_i] > 0);
  });
}

/** \} */

bke::CurvesGeometry trim_curves(const bke::CurvesGeometry &src_curves,
                                const IndexMask &selection,
                                const VArray<float> &starts,
                                const VArray<float> &ends,
                                const GeometryNodeCurveSampleMode mode,
                                const bke::AttributeFilter &attribute_filter)
{
  const OffsetIndices src_points_by_curve = src_curves.points_by_curve();
  IndexMaskMemory memory;
  const IndexMask unselected = selection.complement(src_curves.curves_range(), memory);

  BLI_assert(selection.size() > 0);
  BLI_assert(selection.last() <= src_curves.curves_num());
  BLI_assert(starts.size() == src_curves.curves_num());
  BLI_assert(starts.size() == ends.size());
  src_curves.ensure_evaluated_lengths();

  bke::CurvesGeometry dst_curves = bke::curves::copy_only_curve_domain(src_curves);
  MutableSpan<int> dst_curve_offsets = dst_curves.offsets_for_write();
  Array<bke::curves::CurvePoint, 16> start_points(src_curves.curves_num());
  Array<bke::curves::CurvePoint, 16> end_points(src_curves.curves_num());
  Array<bke::curves::IndexRangeCyclic, 16> src_ranges(src_curves.curves_num());
  compute_curve_trim_parameters(src_curves,
                                selection,
                                starts,
                                ends,
                                mode,
                                dst_curve_offsets,
                                start_points,
                                end_points,
                                src_ranges);
  offset_indices::copy_group_sizes(src_points_by_curve, unselected, dst_curve_offsets);
  offset_indices::accumulate_counts_to_offsets(dst_curve_offsets);
  const OffsetIndices dst_points_by_curve = dst_curves.points_by_curve();
  dst_curves.resize(dst_curves.offsets().last(), dst_curves.curves_num());

  /* Populate curve domain. */
  const bke::AttributeAccessor src_attributes = src_curves.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_curves.attributes_for_write();
  Set<std::string> transfer_curve_skip = {"cyclic", "curve_type", "nurbs_order", "knots_mode"};
  if (dst_curves.has_curve_with_type(CURVE_TYPE_NURBS)) {
    /* If a NURBS curve is copied keep */
    transfer_curve_skip.remove("nurbs_order");
    transfer_curve_skip.remove("knots_mode");
  }

  /* Fetch custom point domain attributes for transfer (copy). */
  Vector<bke::AttributeTransferData> transfer_attributes = bke::retrieve_attributes_for_transfer(
      src_attributes,
      dst_attributes,
      ATTR_DOMAIN_MASK_POINT,
      bke::attribute_filter_with_skip_ref(attribute_filter,
                                          {"position",
                                           "handle_left",
                                           "handle_right",
                                           "handle_type_left",
                                           "handle_type_right",
                                           "nurbs_weight"}));

  auto trim_catmull = [&](const IndexMask &selection) {
    trim_catmull_rom_curves(src_curves,
                            dst_curves,
                            selection,
                            start_points,
                            end_points,
                            src_ranges,
                            transfer_attributes);
  };
  auto trim_poly = [&](const IndexMask &selection) {
    trim_polygonal_curves(src_curves,
                          dst_curves,
                          selection,
                          start_points,
                          end_points,
                          src_ranges,
                          transfer_attributes);
  };
  auto trim_bezier = [&](const IndexMask &selection) {
    trim_bezier_curves(src_curves,
                       dst_curves,
                       selection,
                       start_points,
                       end_points,
                       src_ranges,
                       transfer_attributes);
  };
  auto trim_evaluated = [&](const IndexMask &selection) {
    dst_curves.fill_curve_types(selection, CURVE_TYPE_POLY);
    /* Ensure evaluated positions are available. */
    src_curves.evaluated_positions();
    trim_evaluated_curves(src_curves,
                          dst_curves,
                          selection,
                          start_points,
                          end_points,
                          src_ranges,
                          transfer_attributes);
  };

  /* Populate point domain. */
  bke::curves::foreach_curve_by_type(src_curves.curve_types(),
                                     src_curves.curve_type_counts(),
                                     selection,
                                     trim_catmull,
                                     trim_poly,
                                     trim_bezier,
                                     trim_evaluated);

  /* Cleanup/close context */
  for (bke::AttributeTransferData &attribute : transfer_attributes) {
    attribute.dst.finish();
  }

  /* Copy unselected */
  if (unselected.is_empty()) {
    /* Since all curves were trimmed, none of them are cyclic and the attribute can be removed. */
    dst_curves.attributes_for_write().remove("cyclic");
  }
  else {
    /* Only trimmed curves are no longer cyclic. */
    if (bke::SpanAttributeWriter cyclic = dst_attributes.lookup_for_write_span<bool>("cyclic")) {
      index_mask::masked_fill(cyclic.span, false, selection);
      cyclic.finish();
    }

    Set<std::string> copy_point_skip;
    if (!dst_curves.has_curve_with_type(CURVE_TYPE_NURBS) &&
        src_curves.has_curve_with_type(CURVE_TYPE_NURBS))
    {
      copy_point_skip.add("nurbs_weight");
    }

    bke::copy_attributes_group_to_group(
        src_attributes,
        bke::AttrDomain::Point,
        bke::AttrDomain::Point,
        bke::attribute_filter_with_skip_ref(attribute_filter, copy_point_skip),
        src_points_by_curve,
        dst_points_by_curve,
        unselected,
        dst_attributes);
  }

  dst_curves.remove_attributes_based_on_types();
  dst_curves.tag_topology_changed();
  return dst_curves;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Bisect Curves
 * \{ */

static void bisect_polyline_segment(const Span<float3> src_positions,
                                    const blender::geometry::BisectArgs &args,
                                    const int64_t index,
                                    const int64_t next_index,
                                    Vector<bke::curves::CurvePoint> &bisect_points,
                                    Vector<bool> &bisect_infront)
{
  float d0 = dist_signed_to_plane_v3(src_positions[index], args.plane);
  float d1 = dist_signed_to_plane_v3(src_positions[next_index], args.plane);

  bool sign0 = d0 >= 0.0f; /* True: Infront! */
  bool sign1 = d1 >= 0.0f;

  if (sign0 != sign1) {
    /* Bisect */
    d0 = std::fabs(d0);
    float tot = d0 + std::fabs(d1);
    float d_inter = d0 / tot;

    bisect_points.append({int(index), int(next_index), d_inter});
    bisect_infront.append(!sign0);
  }
}

static void bisect_bezier_segment(const Span<float3> src_positions,
                                  const Span<float3> src_handles_l,
                                  const Span<float3> src_handles_r,
                                  const Span<int8_t> src_types_l,
                                  const Span<int8_t> src_types_r,
                                  const blender::geometry::BisectArgs &args,
                                  const int64_t index,
                                  const int64_t next_index,
                                  Vector<bke::curves::CurvePoint> &bisect_points,
                                  Vector<bool> &bisect_infront)
{
  if (src_types_r[index] == BEZIER_HANDLE_VECTOR &&
      src_types_l[next_index] == BEZIER_HANDLE_VECTOR) {
    /* Treat as polyline. */
    bisect_polyline_segment(src_positions, args, index, next_index, bisect_points, bisect_infront);
  }
  else {

    float dist[4];

    double roots[3];
    int nroot;

    dist[0] = dist_signed_to_plane_v3(src_positions[index], args.plane);
    dist[1] = dist_signed_to_plane_v3(src_handles_r[index], args.plane);
    dist[2] = dist_signed_to_plane_v3(src_handles_l[next_index], args.plane);
    dist[3] = dist_signed_to_plane_v3(src_positions[next_index], args.plane);

    /* Check for plane intersection */
    bool sign_0 = dist[0] >= 0.0;
    bool sign_1 = dist[1] >= 0.0;
    if (sign_0 == sign_1 && sign_0 == (dist[2] >= 0) && sign_0 == (dist[3] >= 0)) {
      return;
    }

    double a = dist[3] - 3.0 * dist[2] + 3.0 * dist[1] - dist[0];
    double b = 3.0 * dist[2] - 6.0 * dist[1] + 3.0 * dist[0];
    double c = 3.0 * dist[1] - 3.0 * dist[0];
    double d = dist[0];
    nroot = cubic_roots(a, b, c, d, roots);

    /* Sort */
    if (nroot == 3) {
      if (roots[2] < roots[0]) {
        std::swap(roots[0], roots[2]);
      }
      if (roots[2] < roots[1]) {
        std::swap(roots[1], roots[2]);
      }
      else if (roots[1] < roots[0]) {
        std::swap(roots[0], roots[1]);
      }
    }
    else if (nroot == 2 && roots[0] > roots[1]) {
      std::swap(roots[0], roots[1]);
    }

    /* Determine if the segment after the intersection is infront of the plane. */
    bool infront = !sign_0; /* Negate the alignment of the first control point as it is associated
                               to the segment before the intersection. */

    if (dist[0] == 0.0) {
      /* Check the sign of the derivative at ~0 as the first control point lies within the plane
       * (negation not needed here!). */
      if (dist[1] == 0.0) {
        infront = dist[2] == 0.0 ? dist[3] >= 0 : dist[2] >= 0.0;
      }
      else {
        infront = sign_1; /* Equivalent to c >= 0.0 (coeffient for the first degree term is
                             dominant at ~0) */
      }
    }

    /* List bisections, track if following spline is infront through alternation. */
    for (int i = 0; i < nroot; i++) {
      /**/
      if (roots[i] <= 1.0 + DBL_EPSILON && roots[i] >= -DBL_EPSILON) {
        /* Bisect */
        bisect_points.append({int(index), int(next_index), clamp_f((float)roots[i], 0.0, 1.0)});
        bisect_infront.append(infront);
        infront = !infront;
      }
    }
  }
}

static bool keep_bisect_spline_no_intersect(const float3 sample_position,
                                            const blender::geometry::BisectArgs &args)
{
  /* Check one (first) control point. */
  float d0 = dist_signed_to_plane_v3(sample_position, args.plane);
  bool infront = d0 >= 0.0f;
  return (infront && !args.clear_outer) || (!infront && !args.clear_inner);
}

static int bisect_spline(const Span<float3> src_positions,
                         const blender::geometry::BisectArgs &args,
                         const bool src_cyclic,
                         const CurveType dst_type,
                         Span<bke::curves::CurvePoint> bisect_points,
                         Span<bool> bisect_infront,
                         BisectCurveSequence &r_curves_bisect_sequence,
                         Array<int, 12> &r_curve_sizes)
{
  bool do_clear = args.clear_inner || args.clear_outer;
  int64_t excl_last;
  bke::curves::CurvePoint lprev, lend;
  bool lprev_infront;

  const int64_t num_intersections = bisect_points.size();
  if (src_cyclic) {
    BLI_assert(num_intersections > 1);
    excl_last = 1;
    lprev = lend = bisect_points.last();
    lprev_infront = bisect_infront.last();
  }
  else {
    /* Non-cyclic */
    excl_last = 0;
    double dist = dist_signed_to_plane_v3(src_positions[0], args.plane);
    int num_control_points = int(src_positions.size());
    lprev = {0, 1, 0.0f};
    lend = {num_control_points - 2, num_control_points - 1, 1.0f};
    lprev_infront = dist == 0.0 ? !bisect_infront.first() : dist > 0.0;
    // lend_infront = false;
  }

  bke::curves::IndexRangeCyclic sample_range;
  if (do_clear) {
    /* Keep inner or outer. */
    bool keep = args.clear_inner;
    Vector<Array<bke::curves::CurvePoint, 12>> bisect_splits;
    Vector<int> split_size;
    int num_total_cures = 0;

    for (const int64_t i : IndexRange(0, num_intersections - excl_last)) {
      /* Skip inner/outer and ignore any duplicates generated from numerical errors. */
      if (keep == lprev_infront && lprev != bisect_points[i]) {
        /* Keep curve partition */
        bisect_splits.append(Array<bke::curves::CurvePoint, 12>{lprev, bisect_points[i]});
        split_size.append(compute_curve_shape(
            lprev, bisect_points[i], dst_type, src_positions.size(), sample_range));
        num_total_cures++;
      }
      lprev = bisect_points[i];
      lprev_infront = bisect_infront[i];
    }
    if (keep == lprev_infront && lprev != lend) {
      /* Keep curve partition */
      bisect_splits.append(Array<bke::curves::CurvePoint, 12>{lprev, lend});
      split_size.append(
          compute_curve_shape(lprev, lend, dst_type, src_positions.size(), sample_range));
      num_total_cures++;
    }

    /* Result is the source spline split into pieces on the remaining side
     */
    r_curves_bisect_sequence = Array<Array<bke::curves::CurvePoint, 12>, 12>{
        bisect_splits.as_span()};
    r_curve_sizes = Array<int, 12>{split_size.as_span()};
    return num_total_cures;
  }
  else {
    /* Keep both inner and outer (insert controlpoints at curve-plane intersections). */
    blender::Vector<bke::curves::CurvePoint> bisect_sequence;
    bisect_sequence.reserve(num_intersections + 1);

    /* Keep all, resample the spline in each interval*/
    int spline_size =
        1; /* Adjust for the inclusion of the first control point (offset and size!). */
    bisect_sequence.append(lprev);

    for (const int64_t i : IndexRange(0, num_intersections - excl_last)) {
      if (bisect_sequence.last() == bisect_points[i]) {
        continue; /* Skip singular points as they can be epsilon generated */
      }
      int size = compute_curve_shape(
          bisect_sequence.last(), bisect_points[i], dst_type, src_positions.size(), sample_range);
      spline_size += size - 1;
      bisect_sequence.append(bisect_points[i]);
    }
    if (bisect_sequence.last() == lend) {
      /* Unlikely case, but ignore duplicate. */
    }
    else {
      int size = compute_curve_shape(
          bisect_sequence.last(), lend, dst_type, src_positions.size(), sample_range);
      spline_size += size - 1;
      bisect_sequence.append(lend);
    }

    /* Result is the source spline with control points inserted at the plane/spline intersections
     */
    r_curves_bisect_sequence = Array<Array<bke::curves::CurvePoint, 12>, 12>{
        bisect_sequence.as_span()};
    r_curve_sizes = Array<int, 12>{spline_size};
    return 1;
  }
}

/* -------------------------------------------------------------------- */
/** \name Bisect Curves
 * \{ */

bke::CurvesGeometry bisect_curves(const bke::CurvesGeometry &src_curves,
                                  IndexMask selection,
                                  const BisectArgs &args,
                                  const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  BLI_assert(selection.size() > 0);
  BLI_assert(selection.last() <= src_curves.curves_num());
  // src_curves.ensure_evaluated_lengths();

  if (src_curves.has_curve_with_type({CURVE_TYPE_CATMULL_ROM, CURVE_TYPE_NURBS})) {
    src_curves.evaluated_positions();
  }

  const VArraySpan<bool> src_cyclic{src_curves.cyclic()};
  const VArraySpan<int8_t> src_curve_types{src_curves.curve_types()};

  Vector<BisectCurveSequence, 12> bisect_sequences(src_curves.curves_num());
  Vector<Array<int, 12>> bisect_sizes(src_curves.curves_num());
  Array<bool, 12> bisect_curve(src_curves.curves_num());
  Array<int8_t, 12> dst_curve_types(src_curves.curves_num());
  std::atomic<int> num_dst_curves = src_curves.curves_num() -
                                    selection.size(); /* Account for curve selection. */

  /* Compute curve bisect points and resulting Curves shape. */
  auto compute_poly_parameters = [&](const IndexMask selection) {
    const Span<float3> src_positions = src_curves.positions();

    threading::parallel_for(selection.index_range(), 512, [&](const IndexRange range) {
      Vector<bke::curves::CurvePoint> bisect_points;
      Vector<bool> bisect_infront;
      for (const int64_t curve_i : selection.slice(range)) {
        /* Compute bisections */
        const IndexRange src_points = src_curves.points_for_curve(curve_i);
        for (const int64_t segment_i : IndexRange(0, src_points.size() - 1)) {
          bisect_polyline_segment(
              src_positions, args, segment_i, segment_i + 1, bisect_points, bisect_infront);
        }
        if (src_cyclic[curve_i]) {
          bisect_polyline_segment(src_positions.slice(src_points),
                                  args,
                                  src_points.size() - 1,
                                  0,
                                  bisect_points,
                                  bisect_infront);
        }
        /* Store and reset bisect trackers */
        if (bisect_points.size() == 0 &&
            keep_bisect_spline_no_intersect(src_positions[src_points.first()], args)) {
          bisect_curve[curve_i] = false;
          num_dst_curves++;
          dst_curve_types[curve_i] = src_curve_types[curve_i];
        }
        else {
          bisect_curve[curve_i] = true;
          num_dst_curves += bisect_spline(src_positions.slice(src_points),
                                          args,
                                          src_cyclic[curve_i],
                                          CURVE_TYPE_POLY,
                                          bisect_points,
                                          bisect_infront,
                                          bisect_sequences[curve_i],
                                          bisect_sizes[curve_i]);
          dst_curve_types[curve_i] = int8_t(CURVE_TYPE_POLY);
        }
        bisect_points.clear();
        bisect_infront.clear();
      }
    });
  };
  auto compute_eval_parameters = [&](const IndexMask selection) {
    const Span<float3> eval_positions = src_curves.evaluated_positions();

    threading::parallel_for(selection.index_range(), 512, [&](const IndexRange range) {
      Vector<bke::curves::CurvePoint> bisect_points;
      Vector<bool> bisect_infront;
      for (const int64_t curve_i : selection.slice(range)) {
        /* Compute bisections */
        const IndexRange eval_points = src_curves.evaluated_points_for_curve(curve_i);
        for (const int64_t segment_i : IndexRange(0, eval_points.size() - 1)) {
          bisect_polyline_segment(
              eval_positions, args, segment_i, segment_i + 1, bisect_points, bisect_infront);
        }
        if (src_cyclic[curve_i]) {
          bisect_polyline_segment(eval_positions.slice(eval_points),
                                  args,
                                  eval_points.size() - 1,
                                  0,
                                  bisect_points,
                                  bisect_infront);
        }
        /* Store and reset bisect trackers */
        if (bisect_points.size() == 0 &&
            keep_bisect_spline_no_intersect(eval_positions[eval_points.first()], args)) {
          bisect_curve[curve_i] = false;
          num_dst_curves++;
          dst_curve_types[curve_i] = src_curve_types[curve_i];
        }
        else {
          bisect_curve[curve_i] = true;
          num_dst_curves += bisect_spline(eval_positions.slice(eval_points),
                                          args,
                                          src_cyclic[curve_i],
                                          CURVE_TYPE_POLY,
                                          bisect_points,
                                          bisect_infront,
                                          bisect_sequences[curve_i],
                                          bisect_sizes[curve_i]);
          dst_curve_types[curve_i] = int8_t(CURVE_TYPE_POLY);
        }
        bisect_points.clear();
        bisect_infront.clear();
      }
    });
  };
  auto compute_bezier_parameters = [&](const IndexMask selection) {
    const Span<float3> src_positions = src_curves.positions();
    const VArraySpan<int8_t> src_types_l{src_curves.handle_types_left()};
    const VArraySpan<int8_t> src_types_r{src_curves.handle_types_right()};
    const Span<float3> src_handles_l = src_curves.handle_positions_left();
    const Span<float3> src_handles_r = src_curves.handle_positions_right();

    threading::parallel_for(selection.index_range(), 512, [&](const IndexRange range) {
      Vector<bke::curves::CurvePoint> bisect_points;
      Vector<bool> bisect_infront;
      for (const int64_t curve_i : selection.slice(range)) {
        /* Compute bisections */
        const IndexRange src_points = src_curves.points_for_curve(curve_i);
        for (const int64_t segment_i : IndexRange(0, src_points.size() - 1)) {
          bisect_bezier_segment(src_positions.slice(src_points),
                                src_handles_l.slice(src_points),
                                src_handles_r.slice(src_points),
                                src_types_l.slice(src_points),
                                src_types_r.slice(src_points),
                                args,
                                segment_i,
                                segment_i + 1,
                                bisect_points,
                                bisect_infront);
        }
        if (src_cyclic[curve_i]) {
          bisect_bezier_segment(src_positions.slice(src_points),
                                src_handles_l.slice(src_points),
                                src_handles_r.slice(src_points),
                                src_types_l.slice(src_points),
                                src_types_r.slice(src_points),
                                args,
                                src_points.size() - 1,
                                0,
                                bisect_points,
                                bisect_infront);
        }
        /* Store and reset bisect trackers */
        if (bisect_points.size() == 0 &&
            keep_bisect_spline_no_intersect(src_positions[src_points.first()], args)) {
          bisect_curve[curve_i] = false;
          num_dst_curves++;
          dst_curve_types[curve_i] = src_curve_types[curve_i];
        }
        else {
          bisect_curve[curve_i] = true;
          num_dst_curves += bisect_spline(src_positions.slice(src_points),
                                          args,
                                          src_cyclic[curve_i],
                                          CURVE_TYPE_BEZIER,
                                          bisect_points,
                                          bisect_infront,
                                          bisect_sequences[curve_i],
                                          bisect_sizes[curve_i]);
          dst_curve_types[curve_i] = int8_t(CURVE_TYPE_BEZIER);
        }
        bisect_points.clear();
        bisect_infront.clear();
      }
    });
  };

  /* Compute bisections. */
  bke::curves::foreach_curve_by_type(src_curves.curve_types(),
                                     src_curves.curve_type_counts(),
                                     selection,
                                     compute_eval_parameters,
                                     compute_poly_parameters,
                                     compute_bezier_parameters,
                                     compute_eval_parameters);

  /* Create destination curves.
   */
  bke::CurvesGeometry dst_curves(0, num_dst_curves);
  MutableSpan<int> dst_curve_offsets = dst_curves.offsets_for_write();
  MutableSpan<int8_t> dst_curve_types_span = dst_curves.curve_types_for_write();
  MutableSpan<bool> dst_cyclic = dst_curves.cyclic_for_write();
  /* Accumulate new masks and destination shape. */
  int curve_counter = 0;
  Vector<int> curve_offsets;
  Vector<int64_t> selection_indices;
  Vector<int64_t> inverse_selection_indices;
  Vector<int64_t> copy_curve_offsets; /* Tracks the offset to copied curves. */
  Vector<int64_t> curve_domain_gather_indices;

  /* Recompute selection and it's inverse (the inverse may not the exact inverse) */
  selection_indices.reserve(selection.size());
  inverse_selection_indices.reserve(src_curves.curves_num() - selection.size());
  curve_offsets.reserve(src_curves.curves_num());
  curve_domain_gather_indices.reserve(num_dst_curves);
  for (int64_t curve_i : IndexRange(src_curves.curves_num())) {
    curve_offsets.append(curve_counter);
    if (!bisect_curve[curve_i]) {
      /* Copied curves */
      dst_curve_offsets[curve_counter] = src_curves.points_num_for_curve(curve_i);
      dst_curve_types_span[curve_counter] = dst_curve_types[curve_i];
      dst_cyclic[curve_counter] = src_cyclic[curve_i];

      inverse_selection_indices.append(curve_i);
      copy_curve_offsets.append(curve_counter);
      curve_domain_gather_indices.append(curve_i);
      ++curve_counter;
    }
    else if (bisect_sequences[curve_i].size() == 0) {
      /* Ignore deleted curves! */
    }
    else {
      selection_indices.append(curve_i);
      for (int size : bisect_sizes[curve_i]) {
        dst_curve_offsets[curve_counter] = size;
        dst_curve_types_span[curve_counter] = dst_curve_types[curve_i];
        dst_cyclic[curve_counter] = src_cyclic[curve_i] && !(args.clear_inner || args.clear_outer);
        curve_domain_gather_indices.append(curve_i);
        ++curve_counter;
      }
    }
  }

  /* Finalize and update the geometry container. */
  bke::curves::accumulate_counts_to_offsets(dst_curve_offsets);
  dst_curves.resize(dst_curves.offsets().last(), dst_curves.curves_num());
  dst_curves.update_curve_types();
  IndexMask inverse_selection = IndexMask(inverse_selection_indices);
  selection = IndexMask(selection_indices);

  /* Populate curve domain. */
  if (dst_curves.has_curve_with_type(CURVE_TYPE_NURBS)) {
    /* Transfer NURBS args for copied curves */
    MutableSpan<int8_t> dst_nurbs_orders = dst_curves.nurbs_orders_for_write();
    MutableSpan<int8_t> dst_nurbs_knot_modes = dst_curves.nurbs_knots_modes_for_write();
    const VArraySpan<int8_t> src_nurbs_orders = src_curves.nurbs_orders();
    const VArraySpan<int8_t> src_nurbs_knot_modes = src_curves.nurbs_knots_modes();

    for (int64_t curve_i : inverse_selection) {
      int dst_curve_i = curve_offsets[curve_i];
      dst_nurbs_orders[dst_curve_i] = src_nurbs_orders[curve_i];
      dst_nurbs_knot_modes[dst_curve_i] = dst_nurbs_knot_modes[curve_i];
    }
  }

  /* Gather (copy) attribute domain */
  const bke::AttributeAccessor src_attributes = src_curves.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_curves.attributes_for_write();
  Set<std::string> transfer_curve_skip = {"cyclic", "curve_type", "nurbs_order", "knots_mode"};
  if (dst_curves.has_curve_with_type(CURVE_TYPE_NURBS)) {
    transfer_curve_skip.remove("nurbs_order");
    transfer_curve_skip.remove("knots_mode");
  }
  bke::gather_attribute_domain(src_attributes,
                               dst_attributes,
                               curve_domain_gather_indices.as_span(),
                               ATTR_DOMAIN_CURVE,
                               propagation_info,
                               transfer_curve_skip);

  /* Fetch custom point domain attributes for transfer (copy). */
  Vector<bke::AttributeTransferData> transfer_attributes = bke::retrieve_attributes_for_transfer(
      src_attributes,
      dst_attributes,
      ATTR_DOMAIN_MASK_POINT,
      propagation_info,
      {"position",
       "handle_left",
       "handle_right",
       "handle_type_left",
       "handle_type_right",
       "nurbs_weight"});

  /* Sample POINT domain functions. */
  auto sample_poly_curves = [&](const IndexMask selection) {
    bisect_polygonal_curves(src_curves,
                            dst_curves,
                            selection,
                            bisect_sequences,
                            curve_offsets,
                            transfer_attributes,
                            false);
  };
  auto sample_eval_curves = [&](const IndexMask selection) {
    bisect_polygonal_curves(src_curves,
                            dst_curves,
                            selection,
                            bisect_sequences,
                            curve_offsets,
                            transfer_attributes,
                            true);
  };
  auto sample_bezier_curves = [&](const IndexMask selection) {
    bisect_bezier_curves(
        src_curves, dst_curves, selection, bisect_sequences, curve_offsets, transfer_attributes);
  };

  /* Sample POINT domain. */
  bke::curves::foreach_curve_by_type(src_curves.curve_types(),
                                     src_curves.curve_type_counts(),
                                     selection,
                                     sample_eval_curves,
                                     sample_poly_curves,
                                     sample_bezier_curves,
                                     sample_eval_curves);

  /* Cleanup/close context */
  for (bke::AttributeTransferData &attribute : transfer_attributes) {
    attribute.dst.finish();
  }

  /* Copy non-seleceted or non-intersecting curves */
  if (!inverse_selection.is_empty()) {
    Set<std::string> copy_point_skip;
    if (!dst_curves.has_curve_with_type(CURVE_TYPE_NURBS) &&
        src_curves.has_curve_with_type(CURVE_TYPE_NURBS)) {
      copy_point_skip.add("nurbs_weight");
    }

    /* Copy point domain. */
    for (auto &attribute : bke::retrieve_attributes_for_transfer(src_attributes,
                                                                 dst_attributes,
                                                                 ATTR_DOMAIN_MASK_POINT,
                                                                 propagation_info,
                                                                 copy_point_skip)) {

      /* Gather the copied curves only */
      bke::curves::copy_point_data(src_curves,
                                   dst_curves,
                                   inverse_selection,
                                   copy_curve_offsets,
                                   attribute.src,
                                   attribute.dst.span);
      attribute.dst.finish();
    }
  }

  dst_curves.tag_topology_changed();
  return dst_curves;
}

/** \} */

}  // namespace blender::geometry
