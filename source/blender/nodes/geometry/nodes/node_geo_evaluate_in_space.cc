/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

// debug includes

#include <limits>
#include <type_traits>

#include "DNA_pointcloud_types.h"

#include "BKE_geometry_fields.hh"
#include "BKE_geometry_set.hh"
#include "BKE_instances.hh"

#include "GEO_mesh_primitive_uv_sphere.hh"
#include "GEO_transform.hh"

#include "BLI_math_quaternion_types.hh"
#include "BLI_timeit.hh"

// debug includes

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_index_mask.hh"
#include "BLI_math_base.hh"
#include "BLI_math_bits.h"
#include "BLI_sort.hh"
#include "BLI_task.hh"
#include "BLI_task_size_hints.hh"
#include "BLI_virtual_array.hh"

#include "BLI_map.hh"

#include "node_geometry_util.hh"

namespace blender {

template<typename T> std::ostream &operator<<(std::ostream &stream, Vector<T> data);

template<typename T, int num>
std::ostream &operator<<(std::ostream &stream, const std::array<T, num> &data)
{
  stream << "{";
  for (const int64_t i : IndexRange(num)) {
    stream << data[i] << (num - 1 == i ? "" : "\t");
  }
  stream << "}";
  return stream;
}

template<typename T> std::ostream &operator<<(std::ostream &stream, const Span<T> span)
{
  for (const int64_t i : span.index_range()) {
    stream << span[i] << (span.size() - 1 == i ? "" : "\t");
  }
  return stream;
}

template<typename T> std::ostream &operator<<(std::ostream &stream, MutableSpan<T> span)
{
  stream << span.as_span();
  return stream;
}

template<typename T> std::ostream &operator<<(std::ostream &stream, Vector<T> data)
{
  stream << data.as_span();
  return stream;
}

template<typename T> std::ostream &operator<<(std::ostream &stream, Array<T> data)
{
  stream << data.as_span();
  return stream;
}

}  // namespace blender

namespace blender {
/*
template<typename T> inline T safe_inf_divide(const T &a, const T &b)
{
  static_assert(std::is_floating_point_v<T>);
  if constexpr (std::numeric_limits<T>::is_iec559) {
    return a / b;
  } else {
    if (b == 0) {
      return std::numeric_limits<T>::max() * (a > 0 ? 1 : -1);
    }
    return a / b;
  }
}
*/
}  // namespace blender

namespace blender::nodes::node_geo_evaluate_in_space_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Vector>("Position").implicit_field(implicit_field_inputs::position);
  b.add_input<decl::Vector>("Value").supports_field().hide_value();

  b.add_input<decl::Int>("Power").default_value(2).hide_value();
  b.add_input<decl::Float>("Error").min(1.0f).default_value(2.0f);

  b.add_output<decl::Vector>("Mean").field_source_reference_all();
  b.add_output<decl::Vector>("Difference Mean").field_source_reference_all();
}

template<typename InT, typename OutT, typename FuncT>
static void parallel_transform(const Span<InT> src,
                               const int grain_size,
                               MutableSpan<OutT> dst,
                               const FuncT func)
{
  BLI_assert(src.size() == dst.size());
  threading::parallel_for(src.index_range(), grain_size, [&](const IndexRange range) {
    const Span<InT> src_slice = src.slice(range);
    MutableSpan<OutT> dst_slice = dst.slice(range);
    std::transform(src_slice.begin(), src_slice.end(), dst_slice.begin(), func);
  });
}

namespace akdbt {

static constexpr int min_bucket_size = 16;

static int total_depth_from_total(const int total_elements)
{
  int levels = 1;
  for (int total_iter = total_elements; total_iter > min_bucket_size; total_iter = total_iter / 2)
  {
    levels++;
  }
  return levels;
}

static int total_joints_at_start(const int depth_i)
{
  return int((int64_t(1) << (depth_i)) - 1);
}

static int total_joints_for_depth(const int total_depth)
{
  BLI_assert(total_depth > 0);
  return total_joints_at_start(total_depth);
}

static int total_joints_at_depth(const int depth_i)
{
  return int(1 << depth_i);
}

static int joint_size_at_depth(const int total_depth, const int depth_i)
{
  BLI_assert(total_depth > 0);
  return int(1 << (total_depth - depth_i - 1));
}

static int total_buckets_at(const int depth_i)
{
  return int(1 << depth_i);
}

static int total_buckets_for(const int total_depth)
{
  BLI_assert(total_depth > 0);
  return int(1 << (total_depth - 1));
}

static IndexRange joints_range_at_depth(const int depth_i)
{
  return IndexRange::from_begin_size(total_joints_at_start(depth_i),
                                     total_joints_at_depth(depth_i));
}

static IndexRange joint_buckets_range_at_depth(const int total_depth,
                                               const int depth_i,
                                               const int joint_i)
{
  BLI_assert(total_depth > 0);
  const int joint_size = joint_size_at_depth(total_depth, depth_i);
  return IndexRange::from_begin_size(joint_size * joint_i, joint_size);
}

#ifdef DEBUG
[[maybe_unused]] static void test()
{
  BLI_assert(total_depth_from_total(100) == 4);

  BLI_assert(total_joints_at_start(0) == 0);
  BLI_assert(total_joints_at_start(1) == 1);
  BLI_assert(total_joints_at_start(2) == 3);
  BLI_assert(total_joints_at_start(3) == 7);

  BLI_assert(total_joints_at_depth(0) == 1);
  BLI_assert(total_joints_at_depth(1) == 2);
  BLI_assert(total_joints_at_depth(2) == 4);
  BLI_assert(total_joints_at_depth(3) == 8);

  BLI_assert(joint_size_at_depth(1, 0) == 1);

  BLI_assert(joint_size_at_depth(2, 0) == 2);
  BLI_assert(joint_size_at_depth(2, 1) == 1);

  BLI_assert(joint_size_at_depth(3, 0) == 4);
  BLI_assert(joint_size_at_depth(3, 1) == 2);
  BLI_assert(joint_size_at_depth(3, 2) == 1);

  BLI_assert(joints_range_at_depth(0) == IndexRange::from_begin_size(0, 1));
  BLI_assert(joints_range_at_depth(1) == IndexRange::from_begin_size(1, 2));
  BLI_assert(joints_range_at_depth(2) == IndexRange::from_begin_size(1 + 2, 4));
  BLI_assert(joints_range_at_depth(3) == IndexRange::from_begin_size(1 + 2 + 4, 8));

  BLI_assert(joint_buckets_range_at_depth(4, 0, 0) == IndexRange::from_begin_size(0, 8));

  BLI_assert(joint_buckets_range_at_depth(4, 1, 0) == IndexRange::from_begin_size(0, 4));
  BLI_assert(joint_buckets_range_at_depth(4, 1, 1) == IndexRange::from_begin_size(4, 4));

  BLI_assert(joint_buckets_range_at_depth(4, 2, 0) == IndexRange::from_begin_size(0, 2));
  BLI_assert(joint_buckets_range_at_depth(4, 2, 1) == IndexRange::from_begin_size(2, 2));
  BLI_assert(joint_buckets_range_at_depth(4, 2, 2) == IndexRange::from_begin_size(4, 2));
  BLI_assert(joint_buckets_range_at_depth(4, 2, 3) == IndexRange::from_begin_size(6, 2));

  BLI_assert(joint_buckets_range_at_depth(4, 3, 0) == IndexRange::from_begin_size(0, 1));
  BLI_assert(joint_buckets_range_at_depth(4, 3, 1) == IndexRange::from_begin_size(1, 1));
  BLI_assert(joint_buckets_range_at_depth(4, 3, 2) == IndexRange::from_begin_size(2, 1));
  BLI_assert(joint_buckets_range_at_depth(4, 3, 3) == IndexRange::from_begin_size(3, 1));
  BLI_assert(joint_buckets_range_at_depth(4, 3, 4) == IndexRange::from_begin_size(4, 1));
  BLI_assert(joint_buckets_range_at_depth(4, 3, 5) == IndexRange::from_begin_size(5, 1));
  BLI_assert(joint_buckets_range_at_depth(4, 3, 6) == IndexRange::from_begin_size(6, 1));
  BLI_assert(joint_buckets_range_at_depth(4, 3, 7) == IndexRange::from_begin_size(7, 1));
}
#endif

static OffsetIndices<int> fill_buckets_linear(const int total_elements, MutableSpan<int> r_offsets)
{
  for (const int64_t i : r_offsets.index_range().drop_back(1)) {
    r_offsets[i] = i * int64_t(total_elements) / (r_offsets.size() - 1);
  }
  r_offsets.last() = total_elements;
  return r_offsets.as_span();
}

template<typename FuncT>
static void for_each_to_bottom(const OffsetIndices<int> buckets_offsets,
                               const int total_depth,
                               const GrainSize grain_size,
                               const FuncT &func)
{
  for (const int depth_i : IndexRange(total_depth).drop_back(1)) {
    const IndexRange joints_range = joints_range_at_depth(depth_i);
    threading::parallel_for(
        joints_range.index_range(),
        grain_size.value,
        [&](const IndexRange range) {
          for (const int joint_i : range) {
            const IndexRange joint_buckets = joint_buckets_range_at_depth(
                total_depth, depth_i, joint_i);
            func(buckets_offsets[joint_buckets], joints_range[joint_i], depth_i);
          }
        },
        threading::accumulated_task_sizes([&](const IndexRange joints_range) {
          return joint_size_at_depth(total_depth, depth_i) * joints_range.size();
        }));
  }
}

template<typename FuncT>
static void for_each_to_top(const OffsetIndices<int> buckets_offsets,
                            const int total_depth,
                            const GrainSize grain_size,
                            const FuncT &func)
{
  const IndexRange depth_range(total_depth);
  for (const int r_depth_i : depth_range.drop_front(1)) {
    const int depth_i = depth_range.last(r_depth_i);
    const IndexRange joints_range = joints_range_at_depth(depth_i);
    const IndexRange prev_joints_range = joints_range_at_depth(depth_i + 1);
    threading::parallel_for(
        joints_range.index_range(),
        grain_size.value,
        [&](const IndexRange range) {
          for (const int joint_i : range) {
            const IndexRange joint_buckets = joint_buckets_range_at_depth(
                total_depth, depth_i, joint_i);
            const int2 sub_joints = int2(prev_joints_range.start()) + int2(joint_i * 2) +
                                    int2(0, 1);
            func(buckets_offsets[joint_buckets], joints_range[joint_i], sub_joints, depth_i);
          }
        },
        threading::accumulated_task_sizes([&](const IndexRange joints_range) {
          return joint_size_at_depth(total_depth, depth_i) * joints_range.size();
        }));
  }
}

template<typename LeafFunc>
static void for_each_leaf(const OffsetIndices<int> buckets_offsets,
                          const int total_depth,
                          const GrainSize grain_size,
                          const LeafFunc &leaf_func)
{
  const IndexRange joints_range = joints_range_at_depth(total_depth - 1);
  threading::parallel_for(
      joints_range.index_range(), grain_size.value, [&](const IndexRange range) {
        for (const int joint_i : range) {
          leaf_func(buckets_offsets[joint_i], int(joints_range[joint_i]), total_depth - 1);
        }
      });
}

#ifdef DEBUG
[[maybe_unused]] static void test2()
{
  const int total_elements = 100;
  const int total_depth = total_depth_from_total(total_elements);
  const int total_buckets = total_buckets_for(total_depth);
  const int total_joints = total_joints_for_depth(total_depth);

  BLI_assert(total_depth == 4);
  BLI_assert(total_buckets == 8);
  BLI_assert(total_joints == (8 + 4 + 2 + 1));

  Array<int> bucket_indices(total_buckets + 1);
  const OffsetIndices<int> buckets_offsets = fill_buckets_linear(total_elements, bucket_indices);

  const Array<int> test_indices({0, 12, 25, 37, 50, 62, 75, 87, 100});
  BLI_assert(bucket_indices.as_span() == test_indices.as_span());

  Map<int, IndexRange> leaf_joint_to_bucket;
  for_each_leaf(buckets_offsets,
                total_depth,
                GrainSize(1'000'000'000),
                [&](const IndexRange bucket_range, const int joint_index, const int depth_i) {
                  BLI_assert(total_depth - 1 == depth_i);
                  BLI_assert(leaf_joint_to_bucket.add(joint_index, bucket_range));
                });

  Map<int, IndexRange> test_leaf_joint_to_bucket;
  test_leaf_joint_to_bucket.add(7 + 0, buckets_offsets[0]);
  test_leaf_joint_to_bucket.add(7 + 1, buckets_offsets[1]);
  test_leaf_joint_to_bucket.add(7 + 2, buckets_offsets[2]);
  test_leaf_joint_to_bucket.add(7 + 3, buckets_offsets[3]);
  test_leaf_joint_to_bucket.add(7 + 4, buckets_offsets[4]);
  test_leaf_joint_to_bucket.add(7 + 5, buckets_offsets[5]);
  test_leaf_joint_to_bucket.add(7 + 6, buckets_offsets[6]);
  test_leaf_joint_to_bucket.add(7 + 7, buckets_offsets[7]);

  BLI_assert(test_leaf_joint_to_bucket == leaf_joint_to_bucket);

  Map<std::pair<int, int>, IndexRange> joints_to_bottom;
  for_each_to_bottom(
      buckets_offsets,
      total_depth,
      GrainSize(1'000'000'000),
      [&](const IndexRange bucket_range, int joint_index, int depth_i) {
        BLI_assert(joints_to_bottom.add(std::pair<int, int>(joint_index, depth_i), bucket_range));
      });

  Map<std::pair<int, int>, IndexRange> test_joints_to_bottom;
  test_joints_to_bottom.add(std::pair<int, int>(0, 0), buckets_offsets[IndexRange(0, 8)]);

  test_joints_to_bottom.add(std::pair<int, int>(1, 1), buckets_offsets[IndexRange(0, 4)]);
  test_joints_to_bottom.add(std::pair<int, int>(2, 1), buckets_offsets[IndexRange(4, 4)]);

  test_joints_to_bottom.add(std::pair<int, int>(3, 2), buckets_offsets[IndexRange(0, 2)]);
  test_joints_to_bottom.add(std::pair<int, int>(4, 2), buckets_offsets[IndexRange(2, 2)]);
  test_joints_to_bottom.add(std::pair<int, int>(5, 2), buckets_offsets[IndexRange(4, 2)]);
  test_joints_to_bottom.add(std::pair<int, int>(6, 2), buckets_offsets[IndexRange(6, 2)]);

  BLI_assert(joints_to_bottom == test_joints_to_bottom);

  Map<std::pair<int, int>, std::pair<int2, IndexRange>> joints_to_top;
  for_each_to_top(
      buckets_offsets,
      total_depth,
      GrainSize(1'000'000'000),
      [&](const IndexRange bucket_range, int joint_index, const int2 sub_joints, int depth_i) {
        BLI_assert(joints_to_top.add(std::pair<int, int>(joint_index, depth_i),
                                     std::pair<int2, IndexRange>(sub_joints, bucket_range)));
      });

  Map<std::pair<int, int>, std::pair<int2, IndexRange>> test_joints_to_top;
  test_joints_to_top.add(
      std::pair<int, int>(3, 2),
      std::pair<int2, IndexRange>(int2(7, 8), buckets_offsets[IndexRange(0, 2)]));
  test_joints_to_top.add(
      std::pair<int, int>(4, 2),
      std::pair<int2, IndexRange>(int2(9, 10), buckets_offsets[IndexRange(2, 2)]));
  test_joints_to_top.add(
      std::pair<int, int>(5, 2),
      std::pair<int2, IndexRange>(int2(11, 12), buckets_offsets[IndexRange(4, 2)]));
  test_joints_to_top.add(
      std::pair<int, int>(6, 2),
      std::pair<int2, IndexRange>(int2(13, 14), buckets_offsets[IndexRange(6, 2)]));

  test_joints_to_top.add(
      std::pair<int, int>(1, 1),
      std::pair<int2, IndexRange>(int2(3, 4), buckets_offsets[IndexRange(0, 4)]));
  test_joints_to_top.add(
      std::pair<int, int>(2, 1),
      std::pair<int2, IndexRange>(int2(5, 6), buckets_offsets[IndexRange(4, 4)]));

  test_joints_to_top.add(
      std::pair<int, int>(0, 0),
      std::pair<int2, IndexRange>(int2(1, 2), buckets_offsets[IndexRange(0, 8)]));

  BLI_assert(joints_to_top == test_joints_to_top);
}
#endif

static int parent_joint_of(const int depth_i, const int joint_i)
{
  BLI_assert(depth_i > 0);
}

#ifdef DEBUG
[[maybe_unused]] static void test3()
{
  const int total_elements = 100;
  const int total_depth = total_depth_from_total(total_elements);
  const int total_buckets = total_buckets_for(total_depth);
  const int total_joints = total_joints_for_depth(total_depth);

  BLI_assert(total_depth == 4);
  BLI_assert(total_buckets == 8);
  BLI_assert(total_joints == (8 + 4 + 2 + 1));

  Array<int> bucket_indices(total_buckets + 1);
  const OffsetIndices<int> buckets_offsets = fill_buckets_linear(total_elements, bucket_indices);

  const Array<int> test_indices({0, 12, 25, 37, 50, 62, 75, 87, 100});
  BLI_assert(bucket_indices.as_span() == test_indices.as_span());
}
#endif

static void from_positions(const Span<float3> positions,
                           const OffsetIndices<int> buckets_offsets,
                           const int total_depth,
                           MutableSpan<int> indices)
{
  array_utils::fill_index_range<int>(indices);

  for_each_to_bottom(
      buckets_offsets,
      total_depth,
      GrainSize(4096),
      [&](const IndexRange bucket_range, const int /*joint_index*/, const int depth_i) {
        const int axis_index = math::mod_periodic(depth_i, 3);
        MutableSpan<int> segment = indices.slice(bucket_range);
        std::sort(segment.begin(), segment.end(), [&](const int a, const int b) {
          return positions[a][axis_index] < positions[b][axis_index];
        });
      });

  for_each_leaf(buckets_offsets,
                total_depth,
                GrainSize(4096),
                [&](const IndexRange bucket_range, const int /*joint_index*/, const int depth_i) {
                  const int axis_index = math::mod_periodic(depth_i, 3);
                  MutableSpan<int> segment = indices.slice(bucket_range);
                  parallel_sort(segment.begin(), segment.end(), [&](const int a, const int b) {
                    return positions[a][axis_index] < positions[b][axis_index];
                  });
                });
}

template<typename T>
static void mean_sums(const OffsetIndices<int> buckets_offsets,
                      const int total_depth,
                      const Span<T> src_buckets_data,
                      MutableSpan<T> dst_joints_data)
{
  for_each_leaf(buckets_offsets,
                total_depth,
                GrainSize(4096),
                [&](const IndexRange bucket_range, const int joint_index, const int /*depth_i*/) {
                  const Span<T> bucket = src_buckets_data.slice(bucket_range);
                  dst_joints_data[joint_index] = std::accumulate(
                      bucket.begin(), bucket.end(), T(0));
                });

  for_each_to_top(buckets_offsets,
                  total_depth,
                  GrainSize(4096),
                  [&](const IndexRange /*buckets_range*/,
                      const int joint_index,
                      const int2 sub_joints,
                      const int /*depth_i*/) {
                    dst_joints_data[joint_index] = dst_joints_data[sub_joints[0]] +
                                                   dst_joints_data[sub_joints[1]];
                  });
}

template<typename T>
static void normalize_for_size(const OffsetIndices<int> buckets_offsets,
                               const int total_depth,
                               MutableSpan<T> dst_joints_data)
{
  for_each_leaf(buckets_offsets,
                total_depth,
                GrainSize(4096),
                [&](const IndexRange bucket_range, const int joint_index, const int /*depth_i*/) {
                  dst_joints_data[joint_index] *= math::rcp(double(bucket_range.size()));
                });
  for_each_to_bottom(
      buckets_offsets,
      total_depth,
      GrainSize(4096),
      [&](const IndexRange bucket_range, const int joint_index, const int /*nesting_i*/) {
        dst_joints_data[joint_index] *= math::rcp(double(bucket_range.size()));
      });
}

/*
template<typename PredicateT, typename FuncT>
static void for_each_to_bottom_skip(const OffsetIndices<int> buckets_offsets,
                                    const int total_depth,
                                    const PredicateT &predicate,
                                    const FuncT &func)
{
  Array<int, 32> bucket_indices_buffet_a(2);
  Array<int, 32> bucket_indices_buffer_b;

  bool moved_mask = false;
  int index = 0;
  for (const int depth_i : IndexRange(total_depth).drop_back(1)) {

    const IndexRange joints_range = joints_range_at_depth(depth_i);
    for (const int joint_i : joints_range.index_range()) {
      const IndexRange joint_buckets = joint_buckets_range_at_depth(total_depth, depth_i, joint_i);
      const bool skip = predicate(buckets_offsets[joint_buckets], joints_range[joint_i], depth_i);

      bucket_indices_buffet_a[index] = int(joint_buckets.start());
      index += bool(moved_mask != skip);
      bucket_indices_buffet_a[index] = int(joint_buckets.end());
    }

  }
}
*/

template<typename LeafFuncT, typename JointFuncT>
static void for_each_to_bottom_skip(const OffsetIndices<int> buckets_offsets,
                                    const int total_depth,
                                    const JointFuncT &joint_predicate,
                                    const LeafFuncT &leaf_func)
{
  Vector<std::pair<int, int>> stack = {{0, 0}};
  while (!stack.is_empty()) {
    const auto [depth_i, joint_i] = stack.pop_last();
    const IndexRange joints_range = joints_range_at_depth(depth_i);
    const IndexRange joint_buckets = joint_buckets_range_at_depth(total_depth, depth_i, joint_i);
    if (joint_predicate(buckets_offsets[joint_buckets], int(joints_range[joint_i]))) {
      continue;
    }
    if (depth_i == total_depth - 1) {
      leaf_func(buckets_offsets[joint_i]);
      continue;
    }
    stack.append({depth_i + 1, joint_i * 2 + 0});
    stack.append({depth_i + 1, joint_i * 2 + 1});
  }
}

static float rcp_pow(const float value, const int pow)
{
  return math::pow<float>(math::rcp(value), pow);
}

struct Item {
  int depth_i;
  int joint_i;
  int prefix_to_visit;
};

template<typename LeafFuncT, typename JointFuncT>
static void for_each_to_bottom_skip_new(const OffsetIndices<int> buckets_offsets,
                                        const int total_depth,
                                        const IndexRange range,
                                        const JointFuncT &joint_predicate,
                                        const LeafFuncT &leaf_func)
{
  Array<int, 0> indices(range.size());
  array_utils::fill_index_range<int>(indices, range.start());
  Vector<Item, 32> stack = {Item{0, 0, int(indices.size())}};
  while (!stack.is_empty()) {
    const Item item = stack.pop_last();
    const int depth_i = item.depth_i;
    const int joint_i = item.joint_i;
    const MutableSpan<int> to_visit = indices.as_mutable_span().take_front(item.prefix_to_visit);
    const IndexRange joints_range = joints_range_at_depth(depth_i);
    const IndexRange joint_buckets = joint_buckets_range_at_depth(total_depth, depth_i, joint_i);

    const auto end_of_prefix = std::stable_partition(
        to_visit.begin(), to_visit.end(), [&](const int i) -> bool {
          return joint_predicate(buckets_offsets[joint_buckets], int(joints_range[joint_i]), i);
        });

    MutableSpan<int> next_indices = to_visit.take_front(
        std::distance(to_visit.begin(), end_of_prefix));
    if (next_indices.is_empty()) {
      continue;
    }

    if (depth_i == total_depth - 1) {
      for (const int i : next_indices) {
        leaf_func(buckets_offsets[joint_i], i);
      }
      continue;
    }

    stack.append({depth_i + 1, joint_i * 2 + 0, int(next_indices.size())});
    stack.append({depth_i + 1, joint_i * 2 + 1, int(next_indices.size())});
  }
}

static void sample_average(const OffsetIndices<int> buckets_offsets,
                           const int total_depth,
                           const Span<float3> src_joints_centre,
                           const Span<float> src_joints_min_distance,
                           const Span<float3> src_joints_data,
                           const Span<float3> src_bucket_position,
                           const Span<float3> src_bucket_data,
                           const int power_value,
                           MutableSpan<float3> dst_buckets_data)
{
  BLI_assert(src_joints_centre.size() == src_joints_min_distance.size());
  BLI_assert(src_bucket_data.size() == dst_buckets_data.size());
  BLI_assert(src_bucket_data.size() == src_bucket_position.size());

  threading::parallel_for(dst_buckets_data.index_range(), 1024, [&](const IndexRange range) {
    for (const int i : range) {
      const float3 position = src_bucket_position[i];
      const float3 data = src_bucket_data[i];
      for_each_to_bottom_skip(
          buckets_offsets,
          total_depth,
          [&](const IndexRange bucket_range, const int joint_index) -> bool {
            const float distance = math::distance(src_joints_centre[joint_index], position);
            if (distance <= src_joints_min_distance[joint_index]) {
              return false;
            }
            dst_buckets_data[i] += (src_joints_data[joint_index] - data) *
                                   math::safe_rcp(math::pow<float>(distance, power_value)) *
                                   double(bucket_range.size());
            return true;
          },
          [&](const IndexRange bucket_range) {
            if (!bucket_range.contains(i)) {
              for (const int index : bucket_range) {
                const float distance = math::distance(src_bucket_position[index], position);
                dst_buckets_data[i] += (src_bucket_data[index] - data) *
                                       math::safe_rcp(math::pow<float>(distance, power_value));
              }
              return;
            }
            for (const int index : IndexRange::from_begin_end(bucket_range.start(), i)) {
              const float distance = math::distance(src_bucket_position[index], position);
              dst_buckets_data[i] += (src_bucket_data[index] - data) *
                                     math::safe_rcp(math::pow<float>(distance, power_value));
            }
            for (const int index :
                 IndexRange::from_begin_end(i + 1, bucket_range.one_after_last())) {
              const float distance = math::distance(src_bucket_position[index], position);
              dst_buckets_data[i] += (src_bucket_data[index] - data) *
                                     math::safe_rcp(math::pow<float>(distance, power_value));
            }
          });
    }
  });
}

static void sample_average_new(const OffsetIndices<int> buckets_offsets,
                               const int total_depth,
                               const Span<float3> src_joints_centre,
                               const Span<float> src_joints_min_distance,
                               const Span<float3> src_joints_value,
                               const Span<float3> src_bucket_position,
                               const Span<float3> src_bucket_value,
                               const int power_value,
                               MutableSpan<float3> dst_buckets_data)
{
  BLI_assert(src_joints_centre.size() == src_joints_min_distance.size());
  BLI_assert(src_bucket_value.size() == dst_buckets_data.size());
  BLI_assert(src_bucket_value.size() == src_bucket_position.size());

  threading::parallel_for(dst_buckets_data.index_range(), 1024, [&](const IndexRange range) {
    for_each_to_bottom_skip_new(
        buckets_offsets,
        total_depth,
        range,
        [&](const IndexRange buckets_range, const int joint_index, const int value_i) -> bool {
          const float3 self_value = src_bucket_value[value_i];
          const float distance = math::distance(src_joints_centre[joint_index], src_bucket_position[value_i]);
          if (distance <= src_joints_min_distance[joint_index]) {
            return true;
          }
          const float relation_factor = math::safe_rcp(math::pow<float>(distance, power_value));
          dst_buckets_data[value_i] += (src_joints_value[joint_index] - self_value) * relation_factor * buckets_range.size();
          return false;
        },
        [&](const IndexRange bucket_range, const int value_i) {
          const float3 position = src_bucket_position[value_i];
          const float3 self_value = src_bucket_value[value_i];
          for (const int index : bucket_range) {
            const float relation_factor = math::pow<float>(math::rcp(math::distance(src_bucket_position[index], position)), power_value);
            const float safe_relation_factor = index == value_i ? 0.0f : relation_factor;
            dst_buckets_data[value_i] += (src_bucket_value[index] - self_value) * safe_relation_factor;
          }
        });
  });
}

}  // namespace akdbt

static float minimal_dinstance_to(const float radius,
                                  const int distance_power,
                                  const float precision)
{
  BLI_assert(precision > 1.0f);
  /**
   * Centre of the sphere with a points inside the sphere and some of the points are the most near
   * and far to the sampler:
   *
   * 1 / (#distance + #radius) <= 1 / (#distance - #radius).
   *
   * They are equal at ~infinite distance. But with error they can be treat as equal much near
   * To approximate this use some factor (1 <= #precision <= infinite) to say how large error is
   * acceptable:
   *
   * 1 / (#distance + #radius) >= 1 / (#distance - #radius) * #precision.
   *
   * Version in arbitrary degree of the distance to each point:
   *
   * (1 / (#distance + #radius)) ^ #distance_power >= #precision * (1 / (#distance - #radius)) ^
   * #distance_power.
   * */
  const float precision_root = math::pow<float>(precision, math::rcp<float>(distance_power));
  return -((1.0f + precision_root) / (1.0f - precision_root) * radius);
}

static void cloud_radii_to_min_distance(const Span<float> src_radii,
                                        const int distance_power,
                                        const float precision,
                                        MutableSpan<float> dst_radii)
{
  parallel_transform<float, float>(src_radii, 1024 * 8, dst_radii, [&](const float radius) {
    return minimal_dinstance_to(radius, distance_power, precision);
  });
}

static std::pair<float3, float> min_packing_sphere(const Span<float3> points)
{
  const auto search_other = [&](const float3 start_point) {
    float distance = 0.0f;
    float3 position = start_point;
    for (const float3 point : points) {
      const float new_distance = math::distance(start_point, point);
      if (distance <= new_distance) {
        position = point;
        distance = new_distance;
      }
    }
    return position;
  };

  const float3 start_b = search_other(points.first());
  const float3 start_c = search_other(start_b);

  float3 centre = math::midpoint(start_b, start_c);
  float radius = math::distance(centre, start_c);
  for (const float3 point : points) {
    const float new_radius = math::distance(centre, point);
    if (new_radius <= radius) {
      continue;
    }

    centre = math::midpoint(centre + math::normalize(centre - point) * radius, point);
    radius = (radius + new_radius) * 0.5f;
  }

  return std::pair<float3, float>(centre, radius);
}

static void packing_spheres(const OffsetIndices<int> buckets_offsets,
                            const int total_depth,
                            const Span<float3> src_bucket_points,
                            MutableSpan<float3> dst_joints_centre,
                            MutableSpan<float> dst_joints_radii)
{
  akdbt::for_each_leaf(
      buckets_offsets,
      total_depth,
      GrainSize(4096),
      [&](const IndexRange bucket_range, const int joint_index, const int /*depth_i*/) {
        const auto [centre, radius] = min_packing_sphere(src_bucket_points.slice(bucket_range));
        dst_joints_centre[joint_index] = centre;
        dst_joints_radii[joint_index] = radius;
      });

  akdbt::for_each_to_top(buckets_offsets,
                         total_depth,
                         GrainSize(4096),
                         [&](const IndexRange buckets_range,
                             const int joint_index,
                             const int2 /*sub_joints*/,
                             const int /*depth_i*/) {
                           const auto [centre, radius] = min_packing_sphere(
                               src_bucket_points.slice(buckets_range));
                           dst_joints_centre[joint_index] = centre;
                           dst_joints_radii[joint_index] = radius;
                         });
}

class DifferenceSumFieldInput final : public bke::GeometryFieldInput {
 private:
  const Field<float3> positions_field_;
  const Field<float3> value_field_;
  const int distance_power_;
  const float precision_;

 public:
  DifferenceSumFieldInput(Field<float3> positions_field,
                          Field<float3> value_field,
                          const int distance_power,
                          const float precision)
      : bke::GeometryFieldInput(CPPType::get<float3>(), "Weighed Difference Sum"),
        positions_field_(std::move(positions_field)),
        value_field_(std::move(value_field)),
        distance_power_(distance_power),
        precision_(precision)
  {
  }

  GVArray get_varray_for_context(const bke::GeometryFieldContext &context,
                                 const IndexMask & /*mask*/) const final
  {
    std::cout << std::endl;
    if (!context.attributes()) {
      return {};
    }
    const int domain_size = context.attributes()->domain_size(context.domain());
    fn::FieldEvaluator evaluator{context, domain_size};
    evaluator.add(positions_field_);
    evaluator.add(value_field_);
    evaluator.evaluate();
    const VArraySpan<float3> positions = evaluator.get_evaluated<float3>(0);
    const VArraySpan<float3> src_values = evaluator.get_evaluated<float3>(1);

    const int total_depth = akdbt::total_depth_from_total(positions.size());
    const int total_buckets = akdbt::total_buckets_for(total_depth);
    const int total_joints = akdbt::total_joints_for_depth(total_depth);

    Array<int> start_indices(total_buckets + 1);
    {
      SCOPED_TIMER_AVERAGED("akdbt::fill_buckets_linear");
      akdbt::fill_buckets_linear(domain_size, start_indices);
    }
    const OffsetIndices<int> base_offsets(start_indices);

    Array<int> indices(domain_size);
    {
      SCOPED_TIMER_AVERAGED("akdbt::from_positions");
      akdbt::from_positions(positions, base_offsets, total_depth, indices);
    }

    Array<float3> bucket_positions(domain_size);
    Array<float3> bucket_values(domain_size);

    {
      SCOPED_TIMER_AVERAGED("array_utils::gather bucket_positions and bucket_values");
      array_utils::gather(
          Span<float3>(positions), indices.as_span(), bucket_positions.as_mutable_span());
      array_utils::gather(
          Span<float3>(src_values), indices.as_span(), bucket_values.as_mutable_span());
    }

    Array<float3> joints_values(total_joints);

    {
      SCOPED_TIMER_AVERAGED("akdbt::mean_sums joints_values");
      akdbt::mean_sums<float3>(base_offsets, total_depth, bucket_values, joints_values);
    }
    {
      SCOPED_TIMER_AVERAGED("akdbt::normalize_for_size joints_values");
      akdbt::normalize_for_size<float3>(base_offsets, total_depth, joints_values);
    }

    Array<float3> joints_positions(total_joints);
    Array<float> joints_radii(total_joints);
    {
      SCOPED_TIMER_AVERAGED("packing_spheres");
      packing_spheres(base_offsets, total_depth, bucket_positions, joints_positions, joints_radii);
    }

    {
      SCOPED_TIMER_AVERAGED("cloud_radii_to_min_distance");
      cloud_radii_to_min_distance(joints_radii, distance_power_, precision_, joints_radii);
    }

    Array<float3> sampled_bucket_values(domain_size, float3(0));
    {
      SCOPED_TIMER_AVERAGED("akdbt::sample_average");
      akdbt::sample_average(base_offsets,
                            total_depth,
                            joints_positions,
                            joints_radii,
                            joints_values,
                            bucket_positions,
                            bucket_values,
                            distance_power_,
                            sampled_bucket_values);
    }

    Array<float3> sampled_bucket_values_other(domain_size, float3(0));
    {
      SCOPED_TIMER_AVERAGED("akdbt::sample_average_new");
      akdbt::sample_average_new(base_offsets,
                                total_depth,
                                joints_positions,
                                joints_radii,
                                joints_values,
                                bucket_positions,
                                bucket_values,
                                distance_power_,
                                sampled_bucket_values_other);
      sampled_bucket_values = std::move(sampled_bucket_values_other);
    }

    Array<float3> dst_values(domain_size);
    {
      SCOPED_TIMER_AVERAGED("array_utils::scatter");
      array_utils::scatter(
          sampled_bucket_values.as_span(), indices.as_span(), dst_values.as_mutable_span());
    }

    return VArray<float3>::ForContainer(std::move(dst_values));
  }

 public:
  void for_each_field_input_recursive(FunctionRef<void(const FieldInput &)> fn) const
  {
    positions_field_.node().for_each_field_input_recursive(fn);
    value_field_.node().for_each_field_input_recursive(fn);
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef DEBUG
  akdbt::test();
  akdbt::test2();
  akdbt::test3();
#endif

  Field<float3> position_field = params.extract_input<Field<float3>>("Position");
  Field<float3> value_field = params.extract_input<Field<float3>>("Value");

  const int power_value = params.extract_input<int>("Power");
  const float precision_value = params.extract_input<float>("Error");

  if (params.output_is_required("Difference Mean")) {
    params.set_output("Difference Mean",
                      Field<float3>(std::make_shared<DifferenceSumFieldInput>(
                          position_field, value_field, power_value, precision_value)));
  }

  if (params.output_is_required("Mean")) {
    params.set_default_remaining_outputs();
  }
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_EVALUATE_IN_SPACE, "Field in Space", NODE_CLASS_CONVERTER);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_evaluate_in_space_cc
