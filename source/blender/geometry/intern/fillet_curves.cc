/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_curves_utils.hh"

#include "BLI_math_rotation_legacy.hh"
#include "BLI_task.hh"

#include "GEO_fillet_curves.hh"

namespace blender::geometry {
static void duplicate_fillet_point_data(const OffsetIndices<int> src_points_by_curve,
                                        const OffsetIndices<int> dst_points_by_curve,
                                        const IndexMask &curve_selection,
                                        const Span<int> all_point_offsets,
                                        const GSpan src,
                                        GMutableSpan dst)
{
  curve_selection.foreach_index(GrainSize(512), [&](const int curve_i) {
    const IndexRange src_points = src_points_by_curve[curve_i];
    const IndexRange dst_points = dst_points_by_curve[curve_i];
    const IndexRange offsets_range = bke::curves::per_curve_point_offsets_range(src_points,
                                                                                curve_i);
    bke::attribute_math::gather_to_groups(all_point_offsets.slice(offsets_range),
                                          IndexRange(src_points.size()),
                                          src.slice(src_points),
                                          dst.slice(dst_points));
  });
}

static void calculate_result_offsets(const OffsetIndices<int> src_points_by_curve,
                                     const IndexMask &selection,
                                     const IndexMask &unselected,
                                     const VArray<float> &radii,
                                     const VArray<int> &counts,
                                     const Span<bool> cyclic,
                                     MutableSpan<int> dst_curve_offsets,
                                     MutableSpan<int> dst_point_offsets,
                                     Span<bool> ends_with_zero_edge)
{
  /* Fill the offsets array with the curve point counts, then accumulate them to form offsets. */
  offset_indices::copy_group_sizes(src_points_by_curve, unselected, dst_curve_offsets);
  selection.foreach_index(GrainSize(512), [&](const int curve_i) {
    const IndexRange src_points = src_points_by_curve[curve_i];
    const IndexRange offsets_range = bke::curves::per_curve_point_offsets_range(src_points,
                                                                                curve_i);

    MutableSpan<int> point_offsets = dst_point_offsets.slice(offsets_range);
    MutableSpan<int> point_counts = point_offsets.drop_back(1);
    counts.materialize_compressed(src_points, point_counts);

    Span<bool> zero_edge_buffer = ends_with_zero_edge.slice(src_points);
    for (const int i : src_points.index_range()) {
      int &count = point_counts[i];
      count = std::max(count, 0) + 1;

      bool has_zero_edge = zero_edge_buffer[i];
      if (has_zero_edge) {
        count -= 1;
      }
    }

    if (!cyclic[curve_i]) {
      /* Endpoints on non-cyclic curves cannot be filleted. */
      point_counts.first() = 1;
      point_counts.last() = 1;
    }

    /* Implicitly "deselect" points with zero radius. */
    devirtualize_varray(radii, [&](const auto radii) {
      for (const int i : IndexRange(src_points.size())) {
        if (radii[src_points[i]] == 0.0f) {
          point_counts[i] = 1;
        }
      }
    });

    offset_indices::accumulate_counts_to_offsets(point_offsets);

    dst_curve_offsets[curve_i] = point_offsets.last();
  });
  offset_indices::accumulate_counts_to_offsets(dst_curve_offsets);
}

static void calculate_directions(const Span<float3> positions, MutableSpan<float3> directions)
{
  for (const int i : positions.index_range().drop_back(1)) {
    directions[i] = math::normalize(positions[i + 1] - positions[i]);
  }
  directions.last() = math::normalize(positions.first() - positions.last());
}

static void calculate_angles(const Span<float3> directions, MutableSpan<float> angles)
{
  angles.first() = M_PI - angle_v3v3(-directions.last(), directions.first());
  for (const int i : directions.index_range().drop_front(1)) {
    angles[i] = M_PI - angle_v3v3(-directions[i - 1], directions[i]);
  }
}

/** Each vertex to be filleted must share displacement to the next edge.
 * Limit the radii input values such that the total sum of displacements on any given edge are less
 * than the source edge length.
 */
static void limit_radii(const Span<float3> positions,
                        const Span<float> angles,
                        const Span<float> radii,
                        const bool cyclic,
                        MutableSpan<float> radii_clamped,
                        MutableSpan<bool> has_zero_length_edge)
{
  IndexRange positions_range = positions.index_range();
  int point_count = positions_range.size();

  /* Pre-calculate displacements because they will be compared across index when computing factors.
   * A displacement is how far the new point will project along the segment
   *
   * Pre-calculate distances between each point because they will be compared across index when
   * computing factors Distances are indexed by edge. In this case, the edge vertices are [i,i+1]
   */
  Array<float> displacements(point_count);
  IndexRange edges_range_r = cyclic ? positions_range : positions_range.drop_back(1);
  Array<float> distances_r(edges_range_r.size());
  for (const int i : positions_range) {
    displacements[i] = radii[i] * std::tan(angles[i] / 2.0f);

    if (i < edges_range_r.size()) {
      // Cyclical curves have a final edge at [n,0]
      int i_next = i == positions_range.last() ? 0 : i + 1;

      float3 position = positions[i];
      float3 position_next = positions[i_next];

      distances_r[i] = math::distance(position, position_next);
    }
  }

  /* Pre-calculate factors because they will be compared across index when checking for zero-length
   * edges A "factor" is the ratio of segment length to required displacement for both vertices on
   * the edge combined.
   */
  Array<float> factors_r(positions_range.size());
  Array<float> factors_l(positions_range.size());
  const int i_last = positions_range.last();
  for (const int i : positions_range) {
    const float displacement = displacements[i];

    bool has_previous_edge = i != 0 || cyclic;
    int i_prev = i == 0 ? i_last : i - 1;

    bool has_next_edge = i != i_last || cyclic;
    int i_next = i == i_last ? 0 : i + 1;

    if (has_previous_edge) {
      const float displacement_prev = displacements[i_prev];
      const float segment_length_prev = distances_r[i_prev];
      const float total_displacement_prev = displacement_prev + displacement;
      const float factor_prev = std::clamp(
          math::safe_divide(segment_length_prev, total_displacement_prev), 0.0f, 1.0f);
      factors_l[i] = factor_prev;
    }
    else {
      factors_l[i] = 0;
    }

    if (has_next_edge) {
      const float displacement_next = displacements[i_next];
      const float segment_length_next = distances_r[i];
      const float total_displacement_next = displacement_next + displacement;
      const float factor_next = std::clamp(
          math::safe_divide(segment_length_next, total_displacement_next), 0.0f, 1.0f);
      factors_r[i] = factor_next;
    }
    else {
      factors_r[i] = 0;
    }
  }

  /* Limit radii using pre-computed values
   * New radii are limited such that the total displacement of projected points is less than the
   * edge-length new points are projected onto
   *
   * In addition, check for zero-length edges.
   * A zero-length edge will be created when the total displacement is exactly equal to the edge
   * length.
   */
  for (const int i : positions_range) {
    bool has_next_edge = i != i_last || cyclic;
    int i_next = i == i_last ? 0 : i + 1;

    const float factor_r = factors_r[i];
    const float factor_l = factors_l[i];
    const float min_factor = std::min(factor_r, factor_l);
    radii_clamped[i] = radii[i] * min_factor;

    if (has_next_edge) {
      const float factor_next_r = factors_r[i_next];
      const float factor_next_l = factors_l[i_next];
      const float min_factor_next = std::min(factor_next_r, factor_next_l);

      bool is_clamped_right = min_factor < 1 && factor_r <= factor_l;
      bool is_next_clamped_left = min_factor_next < 1 && factor_next_l <= factor_next_r;
      has_zero_length_edge[i] = is_clamped_right && is_next_clamped_left;
    }
    else {
      has_zero_length_edge[i] = false;
    }
  }
}

static void calculate_fillet_positions(const Span<float3> src_positions,
                                       const Span<float> angles,
                                       const Span<float> radii,
                                       const Span<float3> directions,
                                       const OffsetIndices<int> dst_offsets,
                                       Span<bool> zero_edge_buffer,
                                       MutableSpan<float3> dst)
{
  const int i_src_last = src_positions.index_range().last();
  threading::parallel_for(src_positions.index_range(), 512, [&](IndexRange range) {
    for (const int i_src : range) {
      const IndexRange arc = dst_offsets[i_src];
      const float3 &src = src_positions[i_src];
      bool is_zero_edge = zero_edge_buffer[i_src];
      if (arc.size() == 1 && !is_zero_edge) {
        dst[arc.first()] = src;
        continue;
      }

      const int i_src_prev = i_src == 0 ? i_src_last : i_src - 1;
      const float angle = angles[i_src];
      const float radius = radii[i_src];
      const float displacement = radius * std::tan(angle / 2.0f);
      const float3 prev_dir = -directions[i_src_prev];
      const float3 &next_dir = directions[i_src];
      const float3 arc_start = src + prev_dir * displacement;
      const float3 arc_end = src + next_dir * displacement;

      dst[arc.first()] = arc_start;
      IndexRange middle = arc.drop_front(1);
      /* If this is a zero-length edge, the endpoint of the arc is the startpoint of the following
       * point. Therefore last point is actually still considered the "middle" of the arc
       */
      if (!is_zero_edge) {
        dst[arc.last()] = arc_end;
        middle = middle.drop_back(1);
      }

      if (middle.is_empty()) {
        continue;
      }

      const float3 axis = -math::normalize(math::cross(prev_dir, next_dir));
      const float3 center_direction = math::normalize(math::midpoint(next_dir, prev_dir));
      const float distance_to_center = std::sqrt(pow2f(radius) + pow2f(displacement));
      const float3 center = src + center_direction * distance_to_center;

      /* Rotate each middle fillet point around the center. */
      const float segment_angle = angle / (middle.size() + 1);
      for (const int i : IndexRange(middle.size())) {
        const int point_i = middle[i];
        dst[point_i] = math::rotate_around_axis(arc_start, center, axis, segment_angle * (i + 1));
      }
    }
  });
}

/**
 * Set handles for the "Bezier" mode where we rely on setting the inner handles to approximate a
 * circular arc. The outer (previous and next) handles outside the result fillet segment are set
 * to vector handles.
 */
static void calculate_bezier_handles_bezier_mode(const Span<float3> src_handles_l,
                                                 const Span<float3> src_handles_r,
                                                 const Span<int8_t> src_types_l,
                                                 const Span<int8_t> src_types_r,
                                                 const Span<float> angles,
                                                 const Span<float> radii,
                                                 const Span<float3> directions,
                                                 const Span<bool> zero_edge_buffer,
                                                 const OffsetIndices<int> dst_offsets,
                                                 const Span<float3> dst_positions,
                                                 MutableSpan<float3> dst_handles_l,
                                                 MutableSpan<float3> dst_handles_r,
                                                 MutableSpan<int8_t> dst_types_l,
                                                 MutableSpan<int8_t> dst_types_r)
{
  const int i_src_first = src_handles_l.index_range().first();
  const int i_src_last = src_handles_l.index_range().last();
  const int i_dst_first = dst_positions.index_range().first();
  const int i_dst_last = dst_positions.index_range().last();

  threading::parallel_for(src_handles_l.index_range(), 512, [&](IndexRange range) {
    for (const int i_src : range) {
      const IndexRange arc = dst_offsets[i_src];
      bool is_zero_edge = zero_edge_buffer[i_src];
      const int i_dst_arc_start = arc.first();

      /* If this point shares a zero-length edge with the next point, the arc end-point is the
       * start-point of the next arc. The left-side handle is calculated during this loop.
       * The right-side handle is calculated during the next loop.
       */
      int i_dst_arc_end = arc.last() + (is_zero_edge ? 1 : 0);
      if (i_dst_arc_end > i_dst_last) {
        i_dst_arc_end = i_dst_first;
      }

      const bool was_not_filleted = arc.size() == 1 && !is_zero_edge;
      if (was_not_filleted or !radii[i_src]) {
        dst_handles_l[i_dst_arc_start] = src_handles_l[i_src];
        dst_handles_r[i_dst_arc_start] = src_handles_r[i_src];
        dst_types_l[i_dst_arc_start] = src_types_l[i_src];
        dst_types_r[i_dst_arc_start] = src_types_r[i_src];
        continue;
      }

      /* Calculate the point's handles on the outside of the fillet segment,
       * connecting to the next or previous result points.
       *
       * The inner handles are aligned with the aligned with the outer vector
       * handles, but have a specific length to best approximate a circle.
       */

      // No index check needed; the first point is not filleted unless the curve is cyclic
      const int i_src_prev = i_src == i_src_first ? i_src_last : i_src - 1;
      const bool is_previous_zero_edge = zero_edge_buffer[i_src_prev];
      const float angle = angles[i_src];
      const float radius = radii[i_src];
      const float handle_length = (4.0f / 3.0f) * radius * std::tan(angle / 4.0f);
      const float3 prev_dir = -directions[i_src_prev];
      const float3 &next_dir = directions[i_src];

      const float3 &arc_start = dst_positions[i_dst_arc_start];
      const float3 &arc_end = dst_positions[i_dst_arc_end];

      if (!is_previous_zero_edge) {
        /* Shared point: Left-side handle is calculated from the left-side point */
        const int i_dst_prev = i_dst_arc_start == i_dst_first ? i_dst_last : i_dst_arc_start - 1;
        dst_handles_l[i_dst_arc_start] = bke::curves::bezier::calculate_vector_handle(
            dst_positions[i_dst_arc_start], dst_positions[i_dst_prev]);
        dst_types_l[i_dst_arc_start] = BEZIER_HANDLE_VECTOR;
      }

      dst_handles_r[i_dst_arc_start] = arc_start - prev_dir * handle_length;
      dst_types_r[i_dst_arc_start] = BEZIER_HANDLE_ALIGN;

      if (!is_zero_edge) {
        /* Shared point: Right-side handle is calculated from the right-side point */
        const int i_dst_next = i_dst_arc_end == i_dst_last ? 0 : i_dst_arc_end + 1;
        dst_handles_r[i_dst_arc_end] = bke::curves::bezier::calculate_vector_handle(
            dst_positions[i_dst_arc_end], dst_positions[i_dst_next]);
        dst_types_r[i_dst_arc_end] = BEZIER_HANDLE_VECTOR;
      }

      dst_handles_l[i_dst_arc_end] = arc_end - next_dir * handle_length;
      dst_types_l[i_dst_arc_end] = BEZIER_HANDLE_ALIGN;
    }
  });
}

/**
 * In the poly fillet mode, all the inner handles are set to vector handles, along with the "outer"
 * (previous and next) handles at each fillet.
 */
static void calculate_bezier_handles_poly_mode(const Span<float3> src_handles_l,
                                               const Span<float3> src_handles_r,
                                               const Span<int8_t> src_types_l,
                                               const Span<int8_t> src_types_r,
                                               const OffsetIndices<int> dst_offsets,
                                               const Span<float3> dst_positions,
                                               MutableSpan<float3> dst_handles_l,
                                               MutableSpan<float3> dst_handles_r,
                                               MutableSpan<int8_t> dst_types_l,
                                               MutableSpan<int8_t> dst_types_r)
{
  const int i_dst_last = dst_positions.index_range().last();
  threading::parallel_for(src_handles_l.index_range(), 512, [&](IndexRange range) {
    for (const int i_src : range) {
      const IndexRange arc = dst_offsets[i_src];
      if (arc.size() == 1) {
        dst_handles_l[arc.first()] = src_handles_l[i_src];
        dst_handles_r[arc.first()] = src_handles_r[i_src];
        dst_types_l.slice(arc).fill(BEZIER_HANDLE_VECTOR);
        dst_types_r.slice(arc).fill(BEZIER_HANDLE_VECTOR);
        continue;
      }

      /* The fillet's next and previous handles are vector handles, as are the inner handles. */
      dst_types_l.slice(arc).fill(BEZIER_HANDLE_VECTOR);
      dst_types_r.slice(arc).fill(BEZIER_HANDLE_VECTOR);

      /* Calculate the point's handles on the outside of the fillet segment. This point
       * won't be selected for a fillet if it is the first or last in a non-cyclic curve. */

      const int i_dst_prev = arc.first() == 0 ? i_dst_last : arc.one_before_start();
      const int i_dst_next = arc.last() == i_dst_last ? 0 : arc.one_after_last();
      dst_handles_l[arc.first()] = bke::curves::bezier::calculate_vector_handle(
          dst_positions[arc.first()], dst_positions[i_dst_prev]);
      dst_handles_r[arc.last()] = bke::curves::bezier::calculate_vector_handle(
          dst_positions[arc.last()], dst_positions[i_dst_next]);

      /* Set the values for the inner handles. */
      const IndexRange middle = arc.drop_front(1).drop_back(1);
      for (const int i : middle) {
        dst_handles_r[i] = bke::curves::bezier::calculate_vector_handle(dst_positions[i],
                                                                        dst_positions[i - 1]);
        dst_handles_l[i] = bke::curves::bezier::calculate_vector_handle(dst_positions[i],
                                                                        dst_positions[i + 1]);
      }
    }
  });
}

static bke::CurvesGeometry fillet_curves(
    const bke::CurvesGeometry &src_curves,
    const IndexMask &curve_selection,
    const VArray<float> &radius_input,
    const VArray<int> &counts,
    const bool limit_radius,
    const bool remove_zero_length_edges,
    const bool use_bezier_mode,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const OffsetIndices src_points_by_curve = src_curves.points_by_curve();
  const Span<float3> positions = src_curves.positions();
  const VArraySpan<bool> cyclic{src_curves.cyclic()};
  const bke::AttributeAccessor src_attributes = src_curves.attributes();
  IndexMaskMemory memory;
  const IndexMask unselected = curve_selection.complement(src_curves.curves_range(), memory);

  // Create mostly un-initialized curve geometry
  bke::CurvesGeometry dst_curves = bke::curves::copy_only_curve_domain(src_curves);

  Array<float3> directions;
  Array<float> angles;
  Array<float> radii;
  Array<bool> ends_with_zero_edge;
  const int src_points_num = src_curves.points_num();

  /*
   * Pre-calculate radii and zero-edges for all curves in order to allocated destination curve
   * memory up-front.
   *
   * If zero-length edge removal is disabled, allocating these arrays on a curve-by-curve basis
   * would conserve memory. However, adding that conditional would complicate the code and the
   * memory savings would be minimal.
   *
   * Currently, memory is allocated for all curves, even those that are not selected.
   * This is done simply for consistency when calling indexer functions.
   */
  directions = Array<float3>(src_points_num);
  angles = Array<float>(src_points_num);
  radii = Array<float>(src_points_num);
  ends_with_zero_edge = Array<bool>(src_points_num);
  curve_selection.foreach_index(GrainSize(512), [&](const int curve_i) {
    const IndexRange src_points = src_points_by_curve[curve_i];
    const Span<float3> positions_buffer = positions.slice(src_points);

    MutableSpan<float3> directions_buffer = directions.as_mutable_span().slice(src_points);
    calculate_directions(positions_buffer, directions_buffer);

    MutableSpan<float> angles_buffer = angles.as_mutable_span().slice(src_points);
    calculate_angles(directions_buffer, angles_buffer);

    MutableSpan<float> radii_buffer = radii.as_mutable_span().slice(src_points);
    MutableSpan<bool> zero_edge_buffer = ends_with_zero_edge.as_mutable_span().slice(src_points);

    if (limit_radius) {
      Array<float> input_radii_buffer = Array<float>(src_points.size());
      radius_input.materialize_compressed(src_points, input_radii_buffer);

      limit_radii(positions_buffer,
                  angles_buffer,
                  input_radii_buffer,
                  cyclic[curve_i],
                  radii_buffer,
                  zero_edge_buffer);

      if (!remove_zero_length_edges) {
        zero_edge_buffer.fill(false);
      }
    }
    else {
      radius_input.materialize_compressed(src_points, radii);
      zero_edge_buffer.fill(false);
    }
  });

  /* Stores the offset of every result point for every original point.
   * The extra length is used in order to store an extra zero for every curve. */
  Array<int> dst_point_offsets(src_curves.points_num() + src_curves.curves_num());
  MutableSpan<int> dst_curve_offsets = dst_curves.offsets_for_write();
  calculate_result_offsets(src_points_by_curve,
                           curve_selection,
                           unselected,
                           radius_input,
                           counts,
                           cyclic,
                           dst_curve_offsets,
                           dst_point_offsets,
                           ends_with_zero_edge);

  OffsetIndices dst_points_by_curve = dst_curves.points_by_curve();
  Span<int> all_point_offsets = dst_point_offsets.as_span();
  const int dst_curve_size = dst_curve_offsets.last();

  dst_curves.resize(dst_curve_size, dst_curves.curves_num());
  bke::MutableAttributeAccessor dst_attributes = dst_curves.attributes_for_write();
  MutableSpan<float3> dst_positions = dst_curves.positions_for_write();

  VArraySpan<int8_t> src_types_l;
  VArraySpan<int8_t> src_types_r;
  Span<float3> src_handles_l;
  Span<float3> src_handles_r;
  MutableSpan<int8_t> dst_types_l;
  MutableSpan<int8_t> dst_types_r;
  MutableSpan<float3> dst_handles_l;
  MutableSpan<float3> dst_handles_r;
  if (src_curves.has_curve_with_type(CURVE_TYPE_BEZIER)) {
    src_types_l = src_curves.handle_types_left();
    src_types_r = src_curves.handle_types_right();
    src_handles_l = src_curves.handle_positions_left();
    src_handles_r = src_curves.handle_positions_right();

    dst_types_l = dst_curves.handle_types_left_for_write();
    dst_types_r = dst_curves.handle_types_right_for_write();
    dst_handles_l = dst_curves.handle_positions_left_for_write();
    dst_handles_r = dst_curves.handle_positions_right_for_write();
  }

  curve_selection.foreach_segment(GrainSize(512), [&](const IndexMaskSegment segment) {
    for (const int curve_i : segment) {
      const IndexRange src_points = src_points_by_curve[curve_i];
      const IndexRange offsets_range = bke::curves::per_curve_point_offsets_range(src_points,
                                                                                  curve_i);
      const OffsetIndices<int> offsets(all_point_offsets.slice(offsets_range));
      const IndexRange dst_points = dst_points_by_curve[curve_i];
      const Span<float3> src_positions = positions.slice(src_points);

      Span<float3> directions_buffer = directions.as_span().slice(src_points);
      Span<float> angles_buffer = angles.as_span().slice(src_points);
      Span<float> radii_buffer = radii.as_span().slice(src_points);
      Span<bool> zero_edge_buffer = ends_with_zero_edge.as_span().slice(src_points);

      calculate_fillet_positions(positions.slice(src_points),
                                 angles_buffer,
                                 radii_buffer,
                                 directions_buffer,
                                 offsets,
                                 zero_edge_buffer,
                                 dst_positions.slice(dst_points));

      if (src_curves.has_curve_with_type(CURVE_TYPE_BEZIER)) {
        if (use_bezier_mode) {
          calculate_bezier_handles_bezier_mode(src_handles_l.slice(src_points),
                                               src_handles_r.slice(src_points),
                                               src_types_l.slice(src_points),
                                               src_types_r.slice(src_points),
                                               angles_buffer,
                                               radii_buffer,
                                               directions_buffer,
                                               zero_edge_buffer,
                                               offsets,
                                               dst_positions.slice(dst_points),
                                               dst_handles_l.slice(dst_points),
                                               dst_handles_r.slice(dst_points),
                                               dst_types_l.slice(dst_points),
                                               dst_types_r.slice(dst_points));
        }
        else {
          calculate_bezier_handles_poly_mode(src_handles_l.slice(src_points),
                                             src_handles_r.slice(src_points),
                                             src_types_l.slice(src_points),
                                             src_types_r.slice(src_points),
                                             offsets,
                                             dst_positions.slice(dst_points),
                                             dst_handles_l.slice(dst_points),
                                             dst_handles_r.slice(dst_points),
                                             dst_types_l.slice(dst_points),
                                             dst_types_r.slice(dst_points));
        }
      }
    }
  });

  for (auto &attribute : bke::retrieve_attributes_for_transfer(
           src_attributes,
           dst_attributes,
           ATTR_DOMAIN_MASK_POINT,
           propagation_info,
           {"position", "handle_type_left", "handle_type_right", "handle_right", "handle_left"}))
  {
    duplicate_fillet_point_data(src_points_by_curve,
                                dst_points_by_curve,
                                curve_selection,
                                all_point_offsets,
                                attribute.src,
                                attribute.dst.span);

    attribute.dst.finish();
  }

  bke::copy_attributes_group_to_group(src_attributes,
                                      bke::AttrDomain::Point,
                                      propagation_info,
                                      {},
                                      src_points_by_curve,
                                      dst_points_by_curve,
                                      unselected,
                                      dst_attributes);

  return dst_curves;
}

bke::CurvesGeometry fillet_curves_poly(
    const bke::CurvesGeometry &src_curves,
    const IndexMask &curve_selection,
    const VArray<float> &radius,
    const VArray<int> &count,
    const bool limit_radius,
    const bool remove_zero_length_edges,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  return fillet_curves(src_curves,
                       curve_selection,
                       radius,
                       count,
                       limit_radius,
                       remove_zero_length_edges,
                       false,
                       propagation_info);
}

bke::CurvesGeometry fillet_curves_bezier(
    const bke::CurvesGeometry &src_curves,
    const IndexMask &curve_selection,
    const VArray<float> &radius,
    const bool limit_radius,
    const bool remove_zero_length_edges,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  return fillet_curves(src_curves,
                       curve_selection,
                       radius,
                       VArray<int>::ForSingle(1, src_curves.points_num()),
                       limit_radius,
                       remove_zero_length_edges,
                       true,
                       propagation_info);
}
}  // namespace blender::geometry
