/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef WITH_POTRACE

#  include <iostream>

#  include "potracelib.h"

#  include "BKE_attribute.hh"
#  include "BKE_curves.hh"

#  include "BLI_array_utils.hh"
#  include "BLI_index_mask.hh"
#  include "BLI_map.hh"
#  include "BLI_math_matrix.hh"
#  include "BLI_math_matrix_types.hh"
#  include "BLI_math_quaternion_types.hh"
#  include "BLI_math_vector_types.hh"
#  include "BLI_offset_indices.hh"
#  include "BLI_task.hh"

#  include "GEO_2d_grid_to_curve.hh"

namespace blender::geometry {

namespace potrace {

static constexpr const int64_t word_size = sizeof(potrace_word) * 8;

static int64_t words_in_line(const int64_t length)
{
  const int64_t words_num = length / word_size + int64_t(length % word_size != 0);
  return words_num;
}

int2 aligned_resolution(const int2 resolution)
{
  return int2(words_in_line(resolution.x) * word_size, resolution.y);
}

static potrace_word prefix_for_fill(const int64_t first_i)
{
  const int64_t extra_in_begin = word_size - (first_i % word_size) - 1;
  return (potrace_word(1) << extra_in_begin) - 1;
}

static potrace_word suffix_for_fill(const int64_t last_i)
{
  const int64_t extra_in_end = word_size - (last_i % word_size) - 1;
  return ~((potrace_word(1) << extra_in_end) - 1);
}

potrace_state_t *image_from_mask(const Params params, const IndexMask &mask)
{
  const int64_t line_words_num = words_in_line(params.resolution.x);
  const int64_t line_bits_num = line_words_num * word_size;
  Array<potrace_word> segments(line_words_num * params.resolution.y, potrace_word(0));

  threading::parallel_for(
      IndexRange(params.resolution.y),
      4096,
      [&](const IndexRange range) {
        const IndexMask local_mask = mask.slice_content(range.start() * line_bits_num,
                                                        range.size() * line_bits_num);
        if (local_mask.is_empty()) {
          return;
        }
        for (const int64_t line_i : range) {
          const IndexMask line_mask = local_mask.slice_content(line_i * line_bits_num,
                                                               line_i * line_bits_num);
          line_mask.foreach_segment([&](IndexMaskSegment segment) {
            if (unique_sorted_indices::non_empty_is_range(segment.base_span())) {
              const int first_segment_i = segment[0] / word_size;
              const int last_segment_i = segment.last() / word_size;
              if (UNLIKELY(first_segment_i == last_segment_i)) {
                segments[first_segment_i] |= prefix_for_fill(segment[0]) &
                                             suffix_for_fill(segment.last());
                return;
              }
              BLI_assert(last_segment_i > first_segment_i);
              segments[first_segment_i] |= prefix_for_fill(segment[0]);
              segments.as_mutable_span()
                  .slice(IndexRange::from_begin_end_inclusive(first_segment_i, last_segment_i)
                             .drop_front(1)
                             .drop_back(1))
                  .fill(~potrace_word(0));
              segments[last_segment_i] |= suffix_for_fill(segment.last());
              return;
            }
            for (const int64_t i : segment) {
              segments[i / word_size] |= potrace_word(1) << (word_size - (i % word_size) - 1);
            }
          });
        }
      },
      threading::accumulated_task_sizes([&](const IndexRange range) {
        /* This is the way to avoid data race for destination segments and keep optimal threads
         * granulation for arbitrary mask. */
        return mask.slice_content(range.start() * line_bits_num, range.size() * line_bits_num)
            .size();
      }));

  potrace_bitmap_t bitmap;
  bitmap.w = params.resolution.x;
  bitmap.h = params.resolution.y;
  bitmap.dy = line_words_num;
  bitmap.map = segments.data();

  potrace_param_t *potrace_params = potrace_param_default();
  BLI_assert(potrace_params != nullptr);
  potrace_params->alphamax = params.smooth_threshold;
  potrace_params->opttolerance = params.optimization_tolerance;

  potrace_state_t *result_image = potrace_trace(potrace_params, &bitmap);
  potrace_param_free(potrace_params);

  if (result_image == nullptr) {
    return nullptr;
  }

  if (result_image->status != POTRACE_STATUS_OK) {
    BLI_assert(result_image->status == POTRACE_STATUS_INCOMPLETE);
    return nullptr;
  }

  return result_image;
}

void free_image(potrace_state_t *image)
{
  potrace_state_free(image);
}

enum struct SegmentType : int8_t {
  Bezier = POTRACE_CURVETO,
  Poly = POTRACE_CORNER,
};

static bool extra_bezier_point_between(const SegmentType a_segment_type,
                                       const SegmentType b_segment_type)
{
  if (a_segment_type == SegmentType::Poly && b_segment_type == SegmentType::Bezier) {
    return true;
  }
  return false;
}

static const constexpr int bezier_prev_right_handle = 0;
static const constexpr int bezier_left_handle = 1;
static const constexpr int bezier_control = 2;

static const constexpr int poly_control = 1;
static const constexpr int poly_right_handle = 2;
static const constexpr int poly_next_left_handle = 2;

}  // namespace potrace

static void potrace_gather_curves(const potrace_state_t *potrace,
                                  Vector<const potrace_path_t *> &r_list)
{
  for (const potrace_path_t *iter = potrace->plist; iter != nullptr; iter = iter->next) {
    r_list.append(iter);
  }
}

static void potrace_curve_parent_index(const Span<const potrace_path_t *> src_curves,
                                       MutableSpan<int> parent_index)
{
  Map<const potrace_path_t *, int> child_parent_index;

  for (const int parent_i : src_curves.index_range()) {
    for (potrace_path_t *iter = src_curves[parent_i]->childlist; iter != nullptr;
         iter = iter->sibling)
    {
      child_parent_index.add(iter, parent_i);
    }
  }

  threading::parallel_for(src_curves.index_range(), 4096, [&](const IndexRange range) {
    const Span<const potrace_path_t *> src_slice = src_curves.slice(range);
    MutableSpan<int> dst_slice = parent_index.slice(range);
    for (const int64_t i : range) {
      dst_slice[i] = child_parent_index.lookup_default(src_slice[i], int(i));
    }
  });
}

static float3 to_float3(const potrace_dpoint_s &point)
{
  return float3(point.x, point.y, 0.0f);
}

template<typename In, typename Out, typename Func>
static void parallel_transform(const Span<In> src,
                               const int64_t grain_size,
                               MutableSpan<Out> dst,
                               const Func func)
{
  BLI_assert(src.size() == dst.size());
  threading::parallel_for(src.index_range(), grain_size, [&](const IndexRange range) {
    const Span<In> src_slice = src.slice(range);
    MutableSpan<Out> dst_slice = dst.slice(range);
    std::transform(src_slice.begin(), src_slice.end(), dst_slice.begin(), func);
  });
}

static void potrace_curve_count_type_switch(const Span<const potrace_path_t *> src_curves,
                                            MutableSpan<int> curve_type_switch_offset_num)
{
  static constexpr const int plus_to_store_offsets = 2;
  threading::parallel_for(
      src_curves.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int curve_i : range) {
          const Span<int> segment_types(src_curves[curve_i]->curve.tag,
                                        src_curves[curve_i]->curve.n);
          const IndexRange range = segment_types.index_range().drop_back(1);
          const int different_types_num = std::count_if(
              range.begin(), range.end(), [&](const int64_t i) {
                return potrace::SegmentType(segment_types[i]) !=
                       potrace::SegmentType(segment_types[i + 1]);
              });
          curve_type_switch_offset_num[curve_i] = different_types_num + plus_to_store_offsets;
        }
      },
      threading::individual_task_sizes(
          [&](const int64_t curve_i) { return src_curves[curve_i]->curve.n; }));
}

static void type_group_sizes(const Span<int> segment_types, MutableSpan<int> indices)
{
  int offset = 0;
  int last_i = 0;
  for (const int i : segment_types.index_range().drop_front(1)) {
    if (potrace::SegmentType(segment_types[i - 1]) != potrace::SegmentType(segment_types[i])) {
      indices[offset] = i - last_i;
      offset++;
      last_i = i;
    }
  }
  indices[offset] = int(segment_types.size()) - last_i;
}

static void potrace_curve_type_switch_sizes(const Span<const potrace_path_t *> src_curves,
                                            const OffsetIndices<int> curve_type_switch_offset,
                                            MutableSpan<int> src_type_switch_sizes_num)
{
  threading::parallel_for(
      src_curves.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int curve_i : range) {
          const Span<int> segment_types(src_curves[curve_i]->curve.tag,
                                        src_curves[curve_i]->curve.n);
          MutableSpan<int> type_switch_sizes_num = src_type_switch_sizes_num.slice(
              curve_type_switch_offset[curve_i]);
          type_group_sizes(segment_types, type_switch_sizes_num);
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return curve_type_switch_offset[range].size(); }));
}

static void type_switch_extra_sizes(const Span<int> segment_types, MutableSpan<int> indices)
{
  int offset = 0;
  int last_i = 0;
  for (const int i : segment_types.index_range().drop_back(1)) {
    const potrace::SegmentType current = potrace::SegmentType(segment_types[i]);
    const potrace::SegmentType next = potrace::SegmentType(segment_types[i + 1]);
    if (current != next) {
      indices[offset] = int(potrace::extra_bezier_point_between(current, next));
      offset++;
      last_i = i;
    }
  }
  indices[offset] = int(potrace::extra_bezier_point_between(
      potrace::SegmentType(segment_types.last()), potrace::SegmentType(segment_types.first())));
}

static void potrace_curve_type_switch_extra_sizes(
    const Span<const potrace_path_t *> src_curves,
    const OffsetIndices<int> curve_type_switch_offset,
    MutableSpan<int> extra_type_switch_sizes_num)
{
  threading::parallel_for(
      src_curves.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int curve_i : range) {
          const Span<int> segment_types(src_curves[curve_i]->curve.tag,
                                        src_curves[curve_i]->curve.n);
          MutableSpan<int> type_switch_sizes_num = extra_type_switch_sizes_num.slice(
              curve_type_switch_offset[curve_i]);
          type_switch_extra_sizes(segment_types, type_switch_sizes_num);
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return curve_type_switch_offset[range].size(); }));
}

[[nodiscard]] static int64_t accumulate_groups_total(
    const OffsetIndices<int> curve_type_switch_offset, MutableSpan<int> src_type_switch_sizes_num)
{
  return threading::parallel_reduce<int64_t>(
      curve_type_switch_offset.index_range(),
      4096,
      0,
      [&](const IndexRange range, int64_t total_size) -> int64_t {
        for (const int64_t curve_i : range) {
          const IndexRange curve_segments = curve_type_switch_offset[curve_i];
          total_size += offset_indices::accumulate_counts_to_offsets(
                            src_type_switch_sizes_num.slice(curve_segments))
                            .total_size();
        }
        return total_size;
      },
      std::plus<int64_t>());
}

static void copy_poly_curve(const potrace_path_t &potrace_curve, MutableSpan<float3> positions)
{
  const Span<potrace_dpoint_t[3]> src_curve(potrace_curve.curve.c, potrace_curve.curve.n);
  parallel_transform(src_curve, 4096, positions, [&](const potrace_dpoint_t(&point)[3]) -> float3 {
    return to_float3(point[potrace::poly_control]);
  });
}

static IndexRange sum(const IndexRange a, const IndexRange b)
{
  return IndexRange(a.start() + b.start(), a.size() + b.size());
}

static void bezier_copy_as_bezier_positions(const Span<potrace_dpoint_t[3]> src,
                                            MutableSpan<float3> dst)
{
  parallel_transform(src, 4096, dst, [&](const potrace_dpoint_t(&point)[3]) -> float3 {
    return to_float3(point[potrace::bezier_control]);
  });
}

static void bezier_copy_as_bezier_left_handles(const Span<potrace_dpoint_t[3]> src,
                                               MutableSpan<float3> dst)
{
  parallel_transform(src, 4096, dst, [&](const potrace_dpoint_t(&point)[3]) -> float3 {
    return to_float3(point[potrace::bezier_left_handle]);
  });
}

static void bezier_copy_as_bezier_right_handles(const Span<potrace_dpoint_t[3]> src,
                                                const potrace::SegmentType next,
                                                const potrace_dpoint_t (&next_point)[3],
                                                MutableSpan<float3> dst)
{
  parallel_transform(
      src.drop_front(1), 4096, dst.drop_back(1), [&](const potrace_dpoint_t(&point)[3]) -> float3 {
        return to_float3(point[potrace::bezier_prev_right_handle]);
      });
  switch (next) {
    case potrace::SegmentType::Bezier:
      dst.last() = to_float3(next_point[potrace::bezier_prev_right_handle]);
      break;
    case potrace::SegmentType::Poly:
      dst.last() = math::midpoint(to_float3(next_point[potrace::poly_control]),
                                  to_float3(src.last()[potrace::bezier_control]));
      break;
  }
}

static void poly_copy_as_bezier_positions(const Span<potrace_dpoint_t[3]> src,
                                          const potrace::SegmentType next,
                                          MutableSpan<float3> dst)
{
  const int extra_point = next == potrace::SegmentType::Bezier ? 1 : 0;
  parallel_transform(
      src, 4096, dst.drop_back(extra_point), [&](const potrace_dpoint_t(&point)[3]) -> float3 {
        return to_float3(point[potrace::poly_control]);
      });
  if (next == potrace::SegmentType::Bezier) {
    dst.last() = to_float3(src.last()[potrace::bezier_control]);
  }
}

static void poly_copy_as_bezier_left_handles(const Span<potrace_dpoint_t[3]> src,
                                             const potrace::SegmentType prev,
                                             const potrace::SegmentType next,
                                             const potrace_dpoint_t (&prev_point)[3],
                                             MutableSpan<float3> dst)
{
  const int extra_point = next == potrace::SegmentType::Bezier ? 1 : 0;
  switch (prev) {
    case potrace::SegmentType::Poly:
      dst.first() = to_float3(prev_point[potrace::poly_right_handle]);
      break;
    case potrace::SegmentType::Bezier:
      dst.first() = math::midpoint(to_float3(prev_point[potrace::bezier_control]),
                                   to_float3(src.first()[potrace::poly_control]));
      break;
  }
  parallel_transform(src.drop_back(1),
                     4096,
                     dst.drop_front(1).drop_back(extra_point),
                     [&](const potrace_dpoint_t(&point)[3]) -> float3 {
                       return to_float3(point[potrace::poly_next_left_handle]);
                     });
  if (next == potrace::SegmentType::Bezier) {
    dst.last() = math::midpoint(to_float3(src.last()[potrace::poly_control]),
                                to_float3(src.last()[potrace::poly_next_left_handle]));
  }
}

static void poly_copy_as_bezier_right_handles(const Span<potrace_dpoint_t[3]> src,
                                              const potrace::SegmentType next,
                                              const potrace_dpoint_t (&next_point)[3],
                                              MutableSpan<float3> dst)
{
  const int extra_point = next == potrace::SegmentType::Bezier ? 1 : 0;
  parallel_transform(src.drop_back(extra_point),
                     4096,
                     dst.drop_back(extra_point + extra_point),
                     [&](const potrace_dpoint_t(&point)[3]) -> float3 {
                       return to_float3(point[potrace::poly_right_handle]);
                     });
  switch (next) {
    case potrace::SegmentType::Poly:
      dst.last() = to_float3(src.last()[potrace::poly_right_handle]);
      break;
    case potrace::SegmentType::Bezier:
      dst.last(1) = math::midpoint(to_float3(src.last()[potrace::poly_control]),
                                   to_float3(src.last()[potrace::poly_right_handle]));
      dst.last(0) = to_float3(next_point[potrace::bezier_prev_right_handle]);
      break;
  }
}

static void potrace_fill_bezier_or_poly_curve(
    const potrace_path_t &potrace_curve,
    const OffsetIndices<int> potrace_curve_segments_groups,
    const OffsetIndices<int> extra_curve_segments_group_points,
    MutableSpan<float3> positions,
    MutableSpan<float3> positions_left,
    MutableSpan<float3> positions_right)
{
  BLI_assert(potrace_curve_segments_groups.size() == extra_curve_segments_group_points.size());
  const Span<potrace_dpoint_t[3]> src_curve(potrace_curve.curve.c, potrace_curve.curve.n);
  const Span<int> segment_types(potrace_curve.curve.tag, potrace_curve.curve.n);

  threading::parallel_for(
      potrace_curve_segments_groups.index_range(),
      1024,
      [&](const IndexRange range) {
        for (const int segments_i : range) {
          const IndexRange src_segments = potrace_curve_segments_groups[segments_i];
          const IndexRange dst_segments = sum(src_segments,
                                              extra_curve_segments_group_points[segments_i]);

          const int next_i = math::mod_periodic<int64_t>(src_segments.last() + 1,
                                                         potrace_curve.curve.n);
          const potrace_dpoint_t(&next_point)[3] = src_curve[next_i];
          const potrace::SegmentType next_type = potrace::SegmentType(segment_types[next_i]);
          switch (potrace::SegmentType(segment_types[src_segments.first()])) {
            case potrace::SegmentType::Poly: {
              const int prev_i = math::mod_periodic<int64_t>(src_segments.first() - 1,
                                                             potrace_curve.curve.n);
              const potrace::SegmentType prev_type = potrace::SegmentType(segment_types[prev_i]);
              const potrace_dpoint_t(&prev_point)[3] = src_curve[prev_i];
              poly_copy_as_bezier_positions(
                  src_curve.slice(src_segments), next_type, positions.slice(dst_segments));
              poly_copy_as_bezier_left_handles(src_curve.slice(src_segments),
                                               prev_type,
                                               next_type,
                                               prev_point,
                                               positions_left.slice(dst_segments));
              poly_copy_as_bezier_right_handles(src_curve.slice(src_segments),
                                                next_type,
                                                next_point,
                                                positions_right.slice(dst_segments));
              break;
            }
            case potrace::SegmentType::Bezier:
              bezier_copy_as_bezier_positions(src_curve.slice(src_segments),
                                              positions.slice(dst_segments));
              bezier_copy_as_bezier_left_handles(src_curve.slice(src_segments),
                                                 positions_left.slice(dst_segments));
              bezier_copy_as_bezier_right_handles(src_curve.slice(src_segments),
                                                  next_type,
                                                  next_point,
                                                  positions_right.slice(dst_segments));
              break;
          }
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return potrace_curve_segments_groups[range].size(); }));
}

namespace potrace {

Curves *image_to_curve(const potrace_state_t *potrace_result,
                       const std::optional<std::string> &parent_index_id)
{
  Vector<const potrace_path_t *> curves_list;
  potrace_gather_curves(potrace_result, curves_list);
  if (curves_list.is_empty()) {
    return nullptr;
  }

  Array<int> parent_index_data;
  if (parent_index_id.has_value()) {
    parent_index_data.reinitialize(curves_list.size());
    potrace_curve_parent_index(curves_list, parent_index_data);
  }

  Array<int> curve_type_switch_offset_num(curves_list.size() + 1);
  potrace_curve_count_type_switch(curves_list, curve_type_switch_offset_num);
  const OffsetIndices<int> curve_type_switch_offset = offset_indices::accumulate_counts_to_offsets(
      curve_type_switch_offset_num);

  Array<int> src_type_switch_sizes_num(curve_type_switch_offset.total_size());
  potrace_curve_type_switch_sizes(
      curves_list, curve_type_switch_offset, src_type_switch_sizes_num);
  const int64_t src_total_segments = accumulate_groups_total(curve_type_switch_offset,
                                                             src_type_switch_sizes_num);

  Array<int> extra_type_switch_sizes_num(curve_type_switch_offset.total_size());
  potrace_curve_type_switch_extra_sizes(
      curves_list, curve_type_switch_offset, extra_type_switch_sizes_num);
  const int64_t extra_total_segments = accumulate_groups_total(curve_type_switch_offset,
                                                               extra_type_switch_sizes_num);

  const int64_t total_vertices = src_total_segments + extra_total_segments;
  if (total_vertices == 0) {
    return nullptr;
  }

  Curves *curves_id = bke::curves_new_nomain(total_vertices, curves_list.size());
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();
  curves.cyclic_for_write().fill(true);

  MutableSpan<int> dst_curve_offsets = curves.offsets_for_write();
  threading::parallel_for(curves_list.index_range(), 4096, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const IndexRange curve_segments_groups = curve_type_switch_offset[curve_i];
      const OffsetIndices<int> potrace_curve_segments_groups =
          src_type_switch_sizes_num.as_span().slice(curve_segments_groups);
      const OffsetIndices<int> extra_curve_segments_group_points =
          extra_type_switch_sizes_num.as_span().slice(curve_segments_groups);
      dst_curve_offsets[curve_i] = potrace_curve_segments_groups.total_size() +
                                   extra_curve_segments_group_points.total_size();
      BLI_assert(potrace_curve_segments_groups.total_size() > 0);
      BLI_assert(extra_curve_segments_group_points.total_size() >= 0);
    }
  });
  const OffsetIndices<int> curve_offsets = offset_indices::accumulate_counts_to_offsets(
      dst_curve_offsets);

  IndexMaskMemory memory;
  const IndexMask poly_curves_mask = IndexMask::from_predicate(
      curves_list.index_range(), GrainSize(4096), memory, [&](const int curve_i) {
        const IndexRange curve_segments_groups = curve_type_switch_offset[curve_i];
        if (curve_segments_groups.size() > 2) {
          return false;
        }
        BLI_assert(curve_segments_groups.size() == 2);
        const Span<int> segment_types(curves_list[curve_i]->curve.tag,
                                      curves_list[curve_i]->curve.n);
        const potrace::SegmentType curve_type = potrace::SegmentType(segment_types.first());
        if (curve_type != potrace::SegmentType::Poly) {
          return false;
        }
        BLI_assert(std::all_of(segment_types.begin(), segment_types.end(), [&](const int type) {
          return type == segment_types.first();
        }));
        return true;
      });
  const IndexMask bezier_curves_mask = poly_curves_mask.complement(curves_list.index_range(),
                                                                   memory);

  curves.fill_curve_types(poly_curves_mask, CURVE_TYPE_POLY);
  curves.fill_curve_types(bezier_curves_mask, CURVE_TYPE_BEZIER);

  if (!bezier_curves_mask.is_empty()) {
    curves.handle_types_left_for_write().fill(BEZIER_HANDLE_FREE);
    curves.handle_types_right_for_write().fill(BEZIER_HANDLE_FREE);
  }

  MutableSpan<float3> positions = curves.positions_for_write();
  MutableSpan<float3> positions_left = curves.handle_positions_left_for_write();
  MutableSpan<float3> positions_right = curves.handle_positions_right_for_write();

  poly_curves_mask.foreach_index(GrainSize(4096), [&](const int curve_i) {
    copy_poly_curve(*curves_list[curve_i], positions.slice(curve_offsets[curve_i]));
  });

  bezier_curves_mask.foreach_index(GrainSize(4096), [&](const int curve_i) {
    const IndexRange curve_segments_groups = curve_type_switch_offset[curve_i];
    const OffsetIndices<int> potrace_curve_segments_groups =
        src_type_switch_sizes_num.as_span().slice(curve_segments_groups);
    const OffsetIndices<int> extra_curve_segments_group_points =
        extra_type_switch_sizes_num.as_span().slice(curve_segments_groups);
    potrace_fill_bezier_or_poly_curve(*curves_list[curve_i],
                                      potrace_curve_segments_groups,
                                      extra_curve_segments_group_points,
                                      positions.slice(curve_offsets[curve_i]),
                                      positions_left.slice(curve_offsets[curve_i]),
                                      positions_right.slice(curve_offsets[curve_i]));
  });

  if (parent_index_id.has_value()) {
    bke::SpanAttributeWriter<int> parent_attribute =
        curves.attributes_for_write().lookup_or_add_for_write_only_span<int>(
            *parent_index_id, bke::AttrDomain::Curve);
    array_utils::copy(parent_index_data.as_span(), parent_attribute.span);
  }

  curves.update_curve_types();
  curves.tag_topology_changed();
  curves.tag_positions_changed();

  return curves_id;
}

float4x4 to_plane(const int2 resolution, const float2 min_point, const float2 max_point)
{
  const float2 resolution_factor = float2(1.0f) / float2(resolution) * (max_point - min_point);
  return math::from_loc_rot_scale<float4x4>(
      float3(min_point, 0.0f), math::Quaternion::identity(), float3(resolution_factor, 1.0f));
}

}  // namespace potrace

}  // namespace blender::geometry

#endif
