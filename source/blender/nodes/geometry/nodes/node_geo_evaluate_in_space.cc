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
#include "BLI_function_ref.hh"
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
          if (UNLIKELY(positions[a][axis_index] == positions[b][axis_index])) {
            return a < b;
          }
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

template<typename T>
static void accumulate_size(const OffsetIndices<int> buckets_offsets,
                            const int total_depth,
                            MutableSpan<T> dst_joints_data)
{
  for_each_leaf(buckets_offsets,
                total_depth,
                GrainSize(4096),
                [&](const IndexRange bucket_range, const int joint_index, const int /*depth_i*/) {
                  dst_joints_data[joint_index] = T(bucket_range.size());
                });

  for_each_to_bottom(
      buckets_offsets,
      total_depth,
      GrainSize(4096),
      [&](const IndexRange bucket_range, const int joint_index, const int /*nesting_i*/) {
        dst_joints_data[joint_index] = T(bucket_range.size());
      });
}

static FunctionRef<void(int, MutableSpan<float>)> powered_rcp_for_squared(const int power_value)
{
  switch (power_value) {
    case 0:
      return [](const int /*power_value*/, MutableSpan<float> values) { values.fill(1.0f); };
    case 1:
      return [](const int /*power_value*/, MutableSpan<float> values) {
        std::transform(values.begin(), values.end(), values.begin(), [&](const float value) {
          return math::safe_rcp(math::sqrt(value));
        });
      };
    case 2:
      return [](const int /*power_value*/, MutableSpan<float> values) {
        std::transform(values.begin(), values.end(), values.begin(), [&](const float value) {
          return math::safe_rcp(value);
        });
      };
    case 3:
      return [](const int /*power_value*/, MutableSpan<float> values) {
        std::transform(values.begin(), values.end(), values.begin(), [&](const float value) {
          return math::safe_rcp(value * math::sqrt(value));
        });
      };
    case 4:
      return [](const int /*power_value*/, MutableSpan<float> values) {
        std::transform(values.begin(), values.end(), values.begin(), [&](const float value) {
          return math::safe_rcp(value * value);
        });
      };
    default:
      return [](const int power_value, MutableSpan<float> values) {
        std::transform(values.begin(), values.end(), values.begin(), [&](const float value) {
          return math::safe_rcp(math::pow(value, float(power_value) * 0.5f));
        });
      };
  }
}

template<typename T>
static void gather(const Span<T> src, const Span<int> indices, MutableSpan<T> dst)
{
  BLI_assert(indices.size() == dst.size());
  for (const int64_t i : dst.index_range()) {
    dst[i] = src[indices[i]];
  }
}

static void squared_distance(const float3 centre, const Span<float3> positions, MutableSpan<float> dst)
{
  BLI_assert(positions.size() == dst.size());
  std::transform(positions.begin(), positions.end(), dst.begin(), [&](const float3 &position) {
    return math::distance_squared(position, centre);
  });
}

static float3 accumulate_difference(const float3 value, const Span<float3> values, const Span<float> weight)
{
  BLI_assert(values.size() == weight.size());
  float3 accumulate_from_zero(0);
  for (const int index : weight.index_range()) {
    accumulate_from_zero += (values[index] - value) * weight[index];
  }
  return accumulate_from_zero;
}

struct Item {
  int depth_i;
  int joint_i;
  int prefix_to_visit;
};

template<typename LeafFuncT, typename JointPredicateT, typename JointFuncT>
static void for_each_to_bottom_skip(const OffsetIndices<int> buckets_offsets,
                                    const int total_depth,
                                    const IndexRange range,
                                    const JointPredicateT &joint_predicate,
                                    const JointFuncT &joint_func,
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
          return joint_predicate(int(joints_range[joint_i]), i);
        });

    const Span<int> finished_indices = to_visit.drop_front(
        std::distance(to_visit.begin(), end_of_prefix));
    joint_func(buckets_offsets[joint_buckets], int(joints_range[joint_i]), finished_indices);

    const Span<int> next_indices = to_visit.take_front(
        std::distance(to_visit.begin(), end_of_prefix));
    if (next_indices.is_empty()) {
      continue;
    }

    if (depth_i == total_depth - 1) {
      leaf_func(buckets_offsets[joint_i], next_indices);
      continue;
    }

    stack.append({depth_i + 1, joint_i * 2 + 1, int(next_indices.size())});
    stack.append({depth_i + 1, joint_i * 2 + 0, int(next_indices.size())});
  }
}

static void sample_average(const OffsetIndices<int> buckets_offsets,
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

  const FunctionRef<void(int, MutableSpan<float>)> squared_distance_invertion =
      powered_rcp_for_squared(power_value);

  constexpr int grain_size = 1024;
  for (const int grain_i : IndexRange((src_bucket_value.size() + grain_size - 1) / grain_size)) {
    const IndexRange range = src_bucket_value.index_range().drop_front(grain_i * grain_size).take_front(grain_size);

    Vector<float> buffer;
    buffer.reserve(range.size());

    for_each_to_bottom_skip(
        buckets_offsets,
        total_depth,
        range,
        [&](const int joint_index, const int value_i) -> bool {
          return math::distance_squared(src_joints_centre[joint_index],
                                        src_bucket_position[value_i]) <=
                 math::square(src_joints_min_distance[joint_index]);
        },
        [&](const IndexRange buckets_range, const int joint_index, const Span<int> value_indices) {
          buffer.resize(value_indices.size());
          for (const int value_i : value_indices.index_range()) {
            const int value_index = value_indices[value_i];
            buffer[value_i] = math::distance_squared(src_joints_centre[joint_index],
                                                     src_bucket_position[value_index]);
          }

          squared_distance_invertion(power_value, buffer.as_mutable_span());

          const float total_factor = buckets_range.size();
          for (const int value_i : value_indices.index_range()) {
            const int value_index = value_indices[value_i];
            dst_buckets_data[value_index] += (src_joints_value[joint_index] -
                                              src_bucket_value[value_index]) *
                                             buffer[value_i] * total_factor;
          }
        },
        [&](const IndexRange bucket_range, const Span<int> value_indices) {
          buffer.resize(bucket_range.size());
          for (const int value_i : value_indices) {
            const float3 position = src_bucket_position[value_i];

            for (const int index : bucket_range.index_range()) {
              buffer[index] = math::distance_squared(src_bucket_position[bucket_range[index]],
                                                     position);
            }

            squared_distance_invertion(power_value, buffer.as_mutable_span());

            const float3 self_value = src_bucket_value[value_i];
            for (const int i : bucket_range.index_range()) {
              const int index = bucket_range[i];
              const float relation_factor = buffer[i];
              const float safe_relation_factor = index == value_i ? 0.0f : relation_factor;
              dst_buckets_data[value_i] += (src_bucket_value[index] - self_value) * safe_relation_factor;
            }
          }
        });
  }
}

template<typename LeafFuncT, typename JointPredicateT, typename JointFuncT>
static void for_each_to_bottom_skip_old(const OffsetIndices<int> buckets_offsets,
                                    const int total_depth,
                                    const IndexRange range,
                                    const JointPredicateT &joint_predicate,
                                    const JointFuncT &joint_func,
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
          return joint_predicate(int(joints_range[joint_i]), i);
        });

    const Span<int> finished_indices = to_visit.drop_front(
        std::distance(to_visit.begin(), end_of_prefix));
    joint_func(buckets_offsets[joint_buckets], int(joints_range[joint_i]), finished_indices);

    const Span<int> next_indices = to_visit.take_front(
        std::distance(to_visit.begin(), end_of_prefix));
    if (next_indices.is_empty()) {
      continue;
    }

    if (depth_i == total_depth - 1) {
      leaf_func(buckets_offsets[joint_i], next_indices);
      continue;
    }

    stack.append({depth_i + 1, joint_i * 2 + 0, int(next_indices.size())});
    stack.append({depth_i + 1, joint_i * 2 + 1, int(next_indices.size())});
  }
}

static void sample_average_old(const OffsetIndices<int> buckets_offsets,
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

  const FunctionRef<void(int, MutableSpan<float>)> squared_distance_invertion =
      powered_rcp_for_squared(power_value);

  threading::parallel_for(src_bucket_value.index_range(), 1024, [&](const IndexRange range) {
    Vector<float> buffer;
    buffer.reserve(range.size());

    for_each_to_bottom_skip_old(
        buckets_offsets,
        total_depth,
        range,
        [&](const int joint_index, const int value_i) -> bool {
          return math::distance_squared(src_joints_centre[joint_index],
                                        src_bucket_position[value_i]) <=
                 math::square(src_joints_min_distance[joint_index]);
        },
        [&](const IndexRange buckets_range, const int joint_index, const Span<int> value_indices) {
          buffer.resize(value_indices.size());
          for (const int value_i : value_indices.index_range()) {
            const int value_index = value_indices[value_i];
            buffer[value_i] = math::distance_squared(src_joints_centre[joint_index],
                                                     src_bucket_position[value_index]);
          }

          squared_distance_invertion(power_value, buffer.as_mutable_span());

          const float total_factor = buckets_range.size();
          for (const int value_i : value_indices.index_range()) {
            const int value_index = value_indices[value_i];
            dst_buckets_data[value_index] += (src_joints_value[joint_index] -
                                              src_bucket_value[value_index]) *
                                             buffer[value_i] * total_factor;
          }
        },
        [&](const IndexRange bucket_range, const Span<int> value_indices) {
          buffer.resize(bucket_range.size());
          for (const int value_i : value_indices) {
            const float3 position = src_bucket_position[value_i];

            for (const int index : bucket_range.index_range()) {
              buffer[index] = math::distance_squared(src_bucket_position[bucket_range[index]],
                                                     position);
            }

            squared_distance_invertion(power_value, buffer.as_mutable_span());

            const float3 self_value = src_bucket_value[value_i];
            for (const int i : bucket_range.index_range()) {
              const int index = bucket_range[i];
              const float relation_factor = buffer[i];
              const float self_ignore_relation_factor = UNLIKELY(index == value_i) ?
                                                            0.0f :
                                                            relation_factor;
              dst_buckets_data[value_i] += (src_bucket_value[index] - self_value) *
                                           self_ignore_relation_factor;
            }
          }
        });
  });
}

template<typename LeafFuncT, typename JointPredicateT, typename JointFuncT>
static void for_each_to_bottom_skip_new(const OffsetIndices<int> buckets_offsets,
                                        const int total_depth,
                                        const IndexRange range,
                                        const JointPredicateT &joint_predicate,
                                        const JointFuncT &joint_func,
                                        const LeafFuncT &leaf_func)
{
  Array<int, 0> begin_indices(range.size());
  array_utils::fill_index_range<int>(begin_indices, range.start());

  Vector<std::pair<int, Array<int, 0>>> parent_joint_indices = {{0, std::move(begin_indices)}};

  for (const int depth_i : IndexRange(total_depth)) {
    const IndexRange joints_range = joints_range_at_depth(depth_i);

    Vector<std::pair<int, Array<int, 0>>> new_joint_indices;
    new_joint_indices.reserve(new_joint_indices.size() * 2);

    for (const int joint_data_i : parent_joint_indices.index_range()) {
      const int joint_i = parent_joint_indices[joint_data_i].first;

      const IndexRange joint_buckets = joint_buckets_range_at_depth(total_depth, depth_i, joint_i);

      MutableSpan<int> parent_indices =
          parent_joint_indices[joint_data_i].second.as_mutable_span();

      const auto end_of_prefix = std::stable_partition(
          parent_indices.begin(), parent_indices.end(), [&](const int i) -> bool {
            return joint_predicate(int(joints_range[joint_i]), i);
          });

      const Span<int> finished_indices = parent_indices.drop_front(
          std::distance(parent_indices.begin(), end_of_prefix));
      const Span<int> next_indices = parent_indices.take_front(
          std::distance(parent_indices.begin(), end_of_prefix));

      joint_func(buckets_offsets[joint_buckets], int(joints_range[joint_i]), finished_indices);

      if (next_indices.is_empty()) {
        continue;
      }

      if (IndexRange(total_depth).last() == depth_i) {
        leaf_func(buckets_offsets[joint_i], next_indices);
        continue;
      }

      const int next_joint_a = joint_i * 2 + 0;
      const int next_joint_b = joint_i * 2 + 1;

      new_joint_indices.append({next_joint_a, Array<int, 0>(next_indices)});
      if (next_indices.size() == parent_indices.size()) {
        new_joint_indices.append(
            {next_joint_b, std::move(parent_joint_indices[joint_data_i].second)});
      }
      else {
        new_joint_indices.append({next_joint_b, Array<int, 0>(next_indices)});
      }
    }

    parent_joint_indices = std::move(new_joint_indices);

    if (parent_joint_indices.is_empty()) {
      break;
    }
  }
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

  const FunctionRef<void(int, MutableSpan<float>)> squared_distance_invertion =
      powered_rcp_for_squared(power_value);

  threading::parallel_for(src_bucket_value.index_range(), 1024 * 16, [&](const IndexRange range) {
    Vector<float> buffer;
    buffer.reserve(range.size());

    for_each_to_bottom_skip_new(
        buckets_offsets,
        total_depth,
        range,
        [&](const int joint_index, const int value_i) -> bool {
          return math::distance_squared(src_joints_centre[joint_index],
                                        src_bucket_position[value_i]) <=
                 math::square(src_joints_min_distance[joint_index]);
        },
        [&](const IndexRange buckets_range, const int joint_index, const Span<int> value_indices) {
          buffer.resize(value_indices.size());
          for (const int value_i : value_indices.index_range()) {
            const int value_index = value_indices[value_i];
            buffer[value_i] = math::distance_squared(src_joints_centre[joint_index],
                                                     src_bucket_position[value_index]);
          }

          squared_distance_invertion(power_value, buffer.as_mutable_span());

          const float total_factor = buckets_range.size();
          for (const int value_i : value_indices.index_range()) {
            const int value_index = value_indices[value_i];
            dst_buckets_data[value_index] += (src_joints_value[joint_index] -
                                              src_bucket_value[value_index]) *
                                             buffer[value_i] * total_factor;
          }
        },
        [&](const IndexRange bucket_range, const Span<int> value_indices) {
          buffer.resize(bucket_range.size());
          for (const int value_i : value_indices) {
            const float3 position = src_bucket_position[value_i];

            for (const int index : bucket_range.index_range()) {
              buffer[index] = math::distance_squared(src_bucket_position[bucket_range[index]],
                                                     position);
            }

            squared_distance_invertion(power_value, buffer.as_mutable_span());

            const float3 self_value = src_bucket_value[value_i];
            for (const int i : bucket_range.index_range()) {
              const int index = bucket_range[i];
              const float relation_factor = buffer[i];
              const float self_ignore_relation_factor = UNLIKELY(index == value_i) ?
                                                            0.0f :
                                                            relation_factor;
              dst_buckets_data[value_i] += (src_bucket_value[index] - self_value) *
                                           self_ignore_relation_factor;
            }
          }
        });
  });
}

static void parents_to_childs(const Span<int> parents, const IndexRange parent_range, const IndexRange child_range, MutableSpan<int> childs)
{
  BLI_assert(parents.size() * 2 == childs.size());
  for (const int i : parents.index_range()) {
    const int parent_i = parents[i] - parent_range.start();
    childs[i * 2 + 0] = child_range[parent_i * 2 + 0];
    childs[i * 2 + 1] = child_range[parent_i * 2 + 1];
  }
}

template<typename JointPredicateT/*, typename BucketPredicateT*/, typename JointFuncT, typename LeafFuncT>
static void for_each_to_bottom_skip_fast(const int total_depth,
                                         const JointPredicateT &joint_predicate,
                                         // const BucketPredicateT &bucket_predicate,
                                         const JointFuncT &joint_func,
                                         const LeafFuncT &leaf_func)
{
  Vector<Array<int, 0>, 32> all_around_stack = {{}};
  Vector<int, 32> depth_stack = {0};
  Vector<int, 32> joint_i_stack = {0};

  while (!all_around_stack.is_empty()) {
    BLI_assert(all_around_stack.size() == depth_stack.size());
    BLI_assert(all_around_stack.size() == joint_i_stack.size());
    
    Array<int, 0> all_around_parent = all_around_stack.pop_last();
    const int depth_i = depth_stack.pop_last();
    const int joint_i = joint_i_stack.pop_last();

    const IndexRange joints_range = joints_range_at_depth(depth_i);
    const IndexRange child_joints_range = joints_range_at_depth(depth_i + 1);

    const int joint_index = joints_range[joint_i];

    const auto end_of_prefix = std::stable_partition(
        all_around_parent.begin(), all_around_parent.end(), [&](const int other_joint_index) -> bool {
          return joint_predicate(joint_index, other_joint_index);
        });

    const Span<int> all_far_parent = all_around_parent.as_span().drop_front(
        std::distance(all_around_parent.begin(), end_of_prefix));
    MutableSpan<int> all_near_parent = all_around_parent.as_mutable_span().take_front(
        std::distance(all_around_parent.begin(), end_of_prefix));

    const IndexRange joint_buckets = joint_buckets_range_at_depth(total_depth, depth_i, joint_i);
    joint_func(joint_buckets, all_far_parent);

    if (depth_i == IndexRange(total_depth).last()) {
      const IndexRange parent_joints_range = joints_range_at_depth(depth_i - 1);
      std::transform(all_near_parent.begin(), all_near_parent.end(), all_near_parent.begin(), [&](const int joint_index) {
        const int joint_i = joint_index - parent_joints_range.start();
        return joint_i;
      });
      const int self_bucket_index = joint_i;
      leaf_func(self_bucket_index, all_near_parent);
      continue;
    }

    const int left_child_i = joint_i * 2 + 0;
    const int right_child_i = joint_i * 2 + 1;

    const bool parent_has_other = depth_i > 1;

    Array<int, 0> left_child_near(all_near_parent.size() * 2 + int(parent_has_other));
    Array<int, 0> right_child_near(all_near_parent.size() * 2 + int(parent_has_other));


    parents_to_childs(all_near_parent.as_span(), joints_range, child_joints_range, left_child_near.as_mutable_span().drop_back(int(parent_has_other)));
    right_child_near.as_mutable_span().copy_from(left_child_near.as_span());
    
   //  if (parent_has_other) {
   //    left_child_near.last() = 
   //    right_child_near.last() = 
   //  }
    
    left_child_near.last() = child_joints_range[right_child_i];
    right_child_near.last() = child_joints_range[left_child_i];

    all_around_stack.append(std::move(right_child_near));
    depth_stack.append(depth_i + 1);
    joint_i_stack.append(right_child_i);

    all_around_stack.append(std::move(left_child_near));
    depth_stack.append(depth_i + 1);
    joint_i_stack.append(left_child_i);
  }
}

static void sample_average_fast(const OffsetIndices<int> buckets_offsets,
                               const int total_depth,
                               const int power_value,
                               const Span<float3> joints_centre,
                               const Span<float> joints_min_radius,
                               const Span<float> joints_min_distance,
                               const Span<float3> joints_value,
                               const Span<float> joints_value_factor,
                               const Span<float3> buckets_position,
                               const Span<float3> src_buckets_value,
                               MutableSpan<float3> dst_buckets_value)
{
  BLI_assert(joints_centre.size() == joints_min_radius.size());
  BLI_assert(joints_centre.size() == joints_min_distance.size());
  BLI_assert(joints_centre.size() == joints_value.size());
  BLI_assert(dst_buckets_value.size() == buckets_position.size());
  BLI_assert(dst_buckets_value.size() == src_buckets_value.size());

  const FunctionRef<void(int, MutableSpan<float>)> squared_distance_invertion = powered_rcp_for_squared(power_value);

  Vector<float> factors_buffer;
  Vector<float3> buffer;
  Vector<float> joints_factors_buffer;

  const IndexRange buckets_range = joints_range_at_depth(total_depth - 1);

  for_each_to_bottom_skip_fast(
      total_depth,
      [&](const int joint_index, const int other_joint_index) -> bool {
        const float distance_squared = math::distance_squared(joints_centre[joint_index], joints_centre[other_joint_index]);
        const int joint_min_radius = joints_min_radius[joint_index];
        const int other_joint_min_distance = joints_min_distance[other_joint_index];
        return math::square(joint_min_radius + other_joint_min_distance) > distance_squared;
      }, /*
      [&](const int joint_index, const int bucket_index) -> bool {
        const float distance_squared = math::distance_squared(joints_centre[joint_index], buckets_position[bucket_index]);
        const int joint_min_distance = joints_min_distance[joint_index];
        return math::square(joint_min_distance) > distance_squared;
      }, */
      [&](const IndexRange buckets, const Span<int> far_joints) {
        buffer.resize(far_joints.size() * 2);
        joints_factors_buffer.resize(far_joints.size());

        gather<float>(joints_value_factor, far_joints, joints_factors_buffer.as_mutable_span());

        {
          MutableSpan<float3> buffer_joint_value = buffer.as_mutable_span().take_front(far_joints.size());
          MutableSpan<float3> buffer_joint_position = buffer.as_mutable_span().take_back(far_joints.size());
          gather<float3>(joints_value, far_joints, buffer_joint_value);
          gather<float3>(joints_centre, far_joints, buffer_joint_position);
        }

        const Span<float3> other_value = buffer.as_span().take_front(far_joints.size());
        const Span<float3> other_position = buffer.as_span().take_back(far_joints.size());

        const IndexRange bucket_range = buckets_offsets[buckets];
        const Span<float3> self_position = buckets_position.slice(bucket_range);
        const Span<float3> self_value = src_buckets_value.slice(bucket_range);
        MutableSpan<float3> dst_value = dst_buckets_value.slice(bucket_range);
        
        factors_buffer.resize(far_joints.size());
        for (const int index : dst_value.index_range()) {
          const float3 position = self_position[index];
          const float3 value = self_value[index];

          squared_distance(position, other_position, factors_buffer);
          squared_distance_invertion(power_value, factors_buffer.as_mutable_span());

          for (const int i : factors_buffer.index_range()) {
            factors_buffer[i] *= joints_factors_buffer[i];
          }

          dst_value[index] += accumulate_difference(value, other_value, factors_buffer);
        }
      },
      [&](const int self_joint, const Span<int> near_joints) {
        BLI_assert(!near_joints.contains(self_joint));

        const IndexRange bucket_range = buckets_offsets[self_joint];
        const Span<float3> self_position = buckets_position.slice(bucket_range);
        const Span<float3> self_value = src_buckets_value.slice(bucket_range);
        MutableSpan<float3> dst_value = dst_buckets_value.slice(bucket_range);

        for (const int index : bucket_range.index_range()) {
          const float3 position = self_position[index];
          const float3 value = self_value[index];

          float3 accumulate_from_zero(0);
          for (const int near_joint_i : near_joints) {

          //  {
          //    const int near_joint_index = buckets_range[near_joint_i];
          //    const float distance_squared = math::distance_squared(position, joints_centre[near_joint_index]);
          //    const int joint_min_distance = joints_min_distance[near_joint_index];
          //    return math::square(joint_min_distance) > distance_squared;
          //  }

            const IndexRange near_bucket_range = buckets_offsets[near_joint_i];
            const Span<float3> other_position = buckets_position.slice(near_bucket_range);
            const Span<float3> other_value = src_buckets_value.slice(near_bucket_range);

            factors_buffer.resize(other_value.size());
            squared_distance(position, other_position, factors_buffer);
            squared_distance_invertion(power_value, factors_buffer.as_mutable_span());
            accumulate_from_zero += accumulate_difference(value, other_value, factors_buffer);
          }
          dst_value[index] += accumulate_from_zero;
        }

        factors_buffer.resize(bucket_range.size());
        for (const int self_i : bucket_range.index_range()) {
          const float3 position = self_position[self_i];
          const float3 value = self_value[self_i];

          squared_distance(position, self_position, factors_buffer);
          squared_distance_invertion(power_value, factors_buffer.as_mutable_span());
          factors_buffer[self_i] = 0.0f;
          dst_value[self_i] += accumulate_difference(value, self_value, factors_buffer);
        }
      });
}

constexpr int32_t three_dimention_mask = 0b1001'0010'0100'1001'0010'0100'1001'0010;

static std::array<bool, 3> dominant_difference(const int a_i, const int a_depth_i, const int b_i, const int b_depth_i)
{
  BLI_assert_msg(a_depth_i > 0, "Which dirrection should be at 0 level? Here is only one joint.");
  BLI_assert_msg(b_depth_i > 0, "Which dirrection should be at 0 level? Here is only one joint.");
  const int32_t a_stack = a_i << (32 - a_depth_i);
  const int32_t b_stack = b_i << (32 - b_depth_i);

  const int32_t stack_diff = a_stack ^ b_stack;

  constexpr int default_diff = 1;
  const int x_axis_dirrection_i = bitscan_reverse_i(stack_diff & three_dimention_mask | default_diff);
  const int y_axis_dirrection_i = bitscan_reverse_i((stack_diff << 1) & three_dimention_mask | default_diff);
  const int z_axis_dirrection_i = bitscan_reverse_i((stack_diff << 2) & three_dimention_mask | default_diff);

  const bool x_axis_dirrection = bool(a_stack & (1 << x_axis_dirrection_i));
  const bool y_axis_dirrection = bool(a_stack & (1 << y_axis_dirrection_i));
  const bool z_axis_dirrection = bool(a_stack & (1 << z_axis_dirrection_i));

  return {x_axis_dirrection, y_axis_dirrection, z_axis_dirrection};
}

static int joint_i_to_stack(const int depth_i, const int joint_i)
{
  return joint_i << (32 - depth_i);
}

static int stack_mask(const int depth_i)
{
  return (1 << (32 - depth_i)) - 1;
}

#ifdef DEBUG
[[maybe_unused]] static void test3()
{
  printf("RUN!\n");
  BLI_assert(joint_i_to_stack(1, 0) == 0b00000000'00000000'00000000'00000000);
  BLI_assert(joint_i_to_stack(1, 1) == 0b10000000'00000000'00000000'00000000);

  BLI_assert(joint_i_to_stack(2, 0) == 0b00000000'00000000'00000000'00000000);
  BLI_assert(joint_i_to_stack(2, 1) == 0b01000000'00000000'00000000'00000000);
  BLI_assert(joint_i_to_stack(2, 2) == 0b10000000'00000000'00000000'00000000);
  BLI_assert(joint_i_to_stack(2, 3) == 0b11000000'00000000'00000000'00000000);

  BLI_assert(joint_i_to_stack(3, 0) == 0b00000000'00000000'00000000'00000000);
  BLI_assert(joint_i_to_stack(3, 1) == 0b00100000'00000000'00000000'00000000);
  BLI_assert(joint_i_to_stack(3, 2) == 0b01000000'00000000'00000000'00000000);
  BLI_assert(joint_i_to_stack(3, 3) == 0b01100000'00000000'00000000'00000000);
  BLI_assert(joint_i_to_stack(3, 4) == 0b10000000'00000000'00000000'00000000);
  BLI_assert(joint_i_to_stack(3, 5) == 0b10100000'00000000'00000000'00000000);
  BLI_assert(joint_i_to_stack(3, 6) == 0b11000000'00000000'00000000'00000000);
  BLI_assert(joint_i_to_stack(3, 7) == 0b11100000'00000000'00000000'00000000);

  BLI_assert(stack_mask(1) == 0b10000000'00000000'00000000'00000000);
  BLI_assert(stack_mask(2) == 0b11000000'00000000'00000000'00000000);
  BLI_assert(stack_mask(3) == 0b11100000'00000000'00000000'00000000);
  
  BLI_assert(false);
}
#endif

static int lowest_joint_in_dirrection(const int prefix_joint, const int prefix_depth_i, const int total_depth, std::array<bool, 3> dirrection)
{
  BLI_assert(total_depth > prefix_depth_i);
  const int prefix_at_depth = prefix_joint << (total_depth - prefix_depth_i - 1);
  
  const int bottom_part_mask = (1 << (total_depth - prefix_depth_i - 1)) - 1;
  
  // prefix_at_depth | (bottom_part_mask & three_dimention_mask & 0);
  
  
  return 0;
}

template<typename LeafFuncT, typename JointPredicateT, typename JointFuncT>
static void for_each_to_bottom_latest(const OffsetIndices<int> buckets_offsets,
                                    const int total_depth,
                                    const IndexRange range,
                                    const JointPredicateT &joint_predicate,
                                    const JointFuncT &joint_func,
                                    const LeafFuncT &leaf_func)
{
  BLI_assert(!buckets_offsets.is_empty());

  Vector<int> joint_to_pass_depht_stack = {1, 1};
  Vector<int> joint_to_pass_i_stack = {0, 1};
  Vector<int> joint_to_sample_depth_stack = {1, 1};
  Vector<int> joint_to_sample_i_stack = {1, 0};

  while (!joint_to_pass_depht_stack.is_empty()) {
    BLI_assert(joint_to_pass_depht_stack.size() == joint_to_pass_i_stack.size());
    BLI_assert(joint_to_pass_depht_stack.size() == joint_to_sample_depth_stack.size());
    BLI_assert(joint_to_pass_depht_stack.size() == joint_to_sample_i_stack.size());

    const int joint_to_pass_depth = joint_to_pass_depht_stack.pop_last();
    const int joint_to_pass_i = joint_to_pass_i_stack.pop_last();
    const int joint_to_sample_depth = joint_to_sample_depth_stack.pop_last();
    const int joint_to_sample_i = joint_to_sample_i_stack.pop_last();

    const int joint_to_pass_index = joints_range_at_depth(joint_to_pass_depth)[joint_to_pass_i];
    const int joint_to_sample_index = joints_range_at_depth(joint_to_sample_depth)[joint_to_sample_i];

    const std::array<bool, 3> dominant_dirrection = dominant_difference(joint_to_pass_i, joint_to_pass_depth, joint_to_sample_i, joint_to_sample_depth);

    const int joint_to_pass_lowest = lowest_joint_in_dirrection(joint_to_pass_i, joint_to_pass_i, total_depth, dominant_dirrection);

    // const int2 lowest_sub_joint_to_pass = binary_search_to_bottom(joint_to_pass_depth, joint_to_sample_depth);
    
    
    
    
    
/*
    const Item item = stack.pop_last();
    const int depth_i = item.depth_i;
    const int joint_i = item.joint_i;
    const MutableSpan<int> to_visit = indices.as_mutable_span().take_front(item.prefix_to_visit);
    const IndexRange joints_range = joints_range_at_depth(depth_i);
    const IndexRange joint_buckets = joint_buckets_range_at_depth(total_depth, depth_i, joint_i);

    const auto end_of_prefix = std::stable_partition(
        to_visit.begin(), to_visit.end(), [&](const int i) -> bool {
          return joint_predicate(int(joints_range[joint_i]), i);
        });

    const Span<int> finished_indices = to_visit.drop_front(
        std::distance(to_visit.begin(), end_of_prefix));
    joint_func(buckets_offsets[joint_buckets], int(joints_range[joint_i]), finished_indices);

    const Span<int> next_indices = to_visit.take_front(
        std::distance(to_visit.begin(), end_of_prefix));
    if (next_indices.is_empty()) {
      continue;
    }

    if (depth_i == total_depth - 1) {
      leaf_func(buckets_offsets[joint_i], next_indices);
      continue;
    }

    stack.append({depth_i + 1, joint_i * 2 + 1, int(next_indices.size())});
    stack.append({depth_i + 1, joint_i * 2 + 0, int(next_indices.size())});
  */
  }
}

static void sample_average_latest(const OffsetIndices<int> buckets_offsets,
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

  const FunctionRef<void(int, MutableSpan<float>)> squared_distance_invertion =
      powered_rcp_for_squared(power_value);

  constexpr int grain_size = 1024;
  for (const int grain_i : IndexRange((src_bucket_value.size() + grain_size - 1) / grain_size)) {
    const IndexRange range = src_bucket_value.index_range().drop_front(grain_i * grain_size).take_front(grain_size);

    Vector<float> buffer;
    buffer.reserve(range.size());

    for_each_to_bottom_latest(
        buckets_offsets,
        total_depth,
        range,
        [&](const int joint_index, const int value_i) -> bool {
          return math::distance_squared(src_joints_centre[joint_index],
                                        src_bucket_position[value_i]) <=
                 math::square(src_joints_min_distance[joint_index]);
        },
        [&](const IndexRange buckets_range, const int joint_index, const Span<int> value_indices) {
          buffer.resize(value_indices.size());
          for (const int value_i : value_indices.index_range()) {
            const int value_index = value_indices[value_i];
            buffer[value_i] = math::distance_squared(src_joints_centre[joint_index],
                                                     src_bucket_position[value_index]);
          }

          squared_distance_invertion(power_value, buffer.as_mutable_span());

          const float total_factor = buckets_range.size();
          for (const int value_i : value_indices.index_range()) {
            const int value_index = value_indices[value_i];
            dst_buckets_data[value_index] += (src_joints_value[joint_index] -
                                              src_bucket_value[value_index]) *
                                             buffer[value_i] * total_factor;
          }
        },
        [&](const IndexRange bucket_range, const Span<int> value_indices) {
          buffer.resize(bucket_range.size());
          for (const int value_i : value_indices) {
            const float3 position = src_bucket_position[value_i];

            for (const int index : bucket_range.index_range()) {
              buffer[index] = math::distance_squared(src_bucket_position[bucket_range[index]],
                                                     position);
            }

            squared_distance_invertion(power_value, buffer.as_mutable_span());

            const float3 self_value = src_bucket_value[value_i];
            for (const int i : bucket_range.index_range()) {
              const int index = bucket_range[i];
              const float relation_factor = buffer[i];
              const float safe_relation_factor = index == value_i ? 0.0f : relation_factor;
              dst_buckets_data[value_i] += (src_bucket_value[index] - self_value) * safe_relation_factor;
            }
          }
        });
  }
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

static std::pair<float3, float> concatenate_spheres(const float3 a_centre, const float3 b_centre, const float a_radius, const float b_radius)
{
  const float3 segment = math::normalize(a_centre - b_centre);
  const float3 a_extremum = a_centre + segment * a_radius;
  const float3 b_extremum = b_centre - segment * b_radius;
  return {math::midpoint(a_extremum, b_extremum), math::distance(a_extremum, b_extremum) * 0.5f};
}

static void packing_spheres_exact(const OffsetIndices<int> buckets_offsets,
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
                         [&](const IndexRange /*buckets_range*/,
                             const int joint_index,
                             const int2 sub_joints,
                             const int /*depth_i*/) {
                           const auto [centre, radius] = concatenate_spheres(dst_joints_centre[sub_joints[0]], dst_joints_centre[sub_joints[1]], dst_joints_radii[sub_joints[0]], dst_joints_radii[sub_joints[1]]);
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


    Array<float> joints_values_factors(total_joints);
    {
      SCOPED_TIMER_AVERAGED("akdbt::accumulate_size");
      akdbt::accumulate_size<float>(base_offsets, total_depth, joints_values_factors);
    }

    Array<float3> joints_positions(total_joints);
    Array<float> joints_min_radii(total_joints);
    {
      SCOPED_TIMER_AVERAGED("packing_spheres");
      packing_spheres(base_offsets, total_depth, bucket_positions, joints_positions, joints_min_radii);
    }

    Array<float> joints_min_distance(total_joints);
    {
      SCOPED_TIMER_AVERAGED("cloud_radii_to_min_distance");
      cloud_radii_to_min_distance(joints_min_radii, distance_power_, precision_, joints_min_distance);
    }

    Array<float3> sampled_bucket_values(domain_size, float3(0));
    if constexpr (true) {
      SCOPED_TIMER_AVERAGED("akdbt::sample_average");
      akdbt::sample_average(base_offsets,
                            total_depth,
                            joints_positions,
                            joints_min_distance,
                            joints_values,
                            bucket_positions,
                            bucket_values,
                            distance_power_,
                            sampled_bucket_values);
    }

    Array<float3> sampled_bucket_values_latest(domain_size, float3(0));
    if constexpr (true) {
      SCOPED_TIMER_AVERAGED("akdbt::sample_average_latest");
      akdbt::sample_average_latest(base_offsets,
                            total_depth,
                            joints_positions,
                            joints_min_distance,
                            joints_values,
                            bucket_positions,
                            bucket_values,
                            distance_power_,
                            sampled_bucket_values_latest);
      sampled_bucket_values = std::move(sampled_bucket_values_latest);
    }

    Array<float3> sampled_bucket_values_new(domain_size, float3(0));
    if constexpr (!true) {
      SCOPED_TIMER_AVERAGED("akdbt::sample_average_fast");
      akdbt::sample_average_fast(base_offsets,
                                total_depth,
                                distance_power_,
                                joints_positions,
                                joints_min_radii,
                                joints_min_distance,
                                joints_values,
                                joints_values_factors,
                                bucket_positions,
                                bucket_values,
                                sampled_bucket_values_new);
      sampled_bucket_values = std::move(sampled_bucket_values_new);
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

#ifdef DEBUG
static_assert(false, "ifdef DEBUG");
#else
static_assert(false, "ifdef !DEBUG");
#endif

#ifdef NDEBUG
static_assert(false, "ifdef NDEBUG");
#else
static_assert(false, "ifdef !NDEBUG");
#endif

static void node_geo_exec(GeoNodeExecParams params)
{
  printf("RUNAAAA!\n");
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
