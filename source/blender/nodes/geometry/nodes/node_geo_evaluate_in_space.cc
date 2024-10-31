/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_index_mask.hh"
#include "BLI_math_base.hh"
#include "BLI_math_bits.h"
#include "BLI_task.hh"
#include "BLI_virtual_array.hh"

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

namespace blender::nodes::node_geo_evaluate_in_space_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Vector>("Position").implicit_field(implicit_field_inputs::position);
  b.add_input<decl::Vector>("Value").supports_field().hide_value();

  b.add_input<decl::Int>("Power").default_value(2);

  b.add_input<decl::Float>("Precision").subtype(PROP_FACTOR).min(1.0f).default_value(2.0f);

  b.add_output<decl::Vector>("Weighted Sum").field_source_reference_all();
  b.add_output<decl::Vector>("Weighted Difference Sum").field_source_reference_all();
  b.add_output<decl::Vector>("Brute Force Weighted Difference Sum").field_source_reference_all();
}

namespace akdbt {

static constexpr int min_bucket_size = 10;

static int total_depth_from_total(const int total_elements)
{
  int levels = 1;
  for (int total_iter = total_elements; total_iter > min_bucket_size; total_iter = total_iter / 2)
  {
    levels++;
  }
  return levels;
}

static int total_joints_at_start(const int total_depth)
{
  return int((int64_t(1) << (total_depth)) - 1);
}

static int total_joints_at_depth(const int depth_i)
{
  return int(1 << depth_i);
}

static int joint_size_at_depth(const int total_depth, const int depth_i)
{
  return int(1 << (total_depth - depth_i));
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
  const int joint_size = joint_size_at_depth(total_depth, depth_i);
  return IndexRange::from_begin_size(joint_size * joint_i, joint_size);
}

static void test()
{
  BLI_assert(total_depth_from_total(100) == 6);

  BLI_assert(total_joints_at_start(0) == 0);
  BLI_assert(total_joints_at_start(1) == 1);
  BLI_assert(total_joints_at_start(2) == 3);
  BLI_assert(total_joints_at_start(3) == 7);

  BLI_assert(total_joints_at_depth(0) == 1);
  BLI_assert(total_joints_at_depth(1) == 2);
  BLI_assert(total_joints_at_depth(2) == 4);
  BLI_assert(total_joints_at_depth(3) == 8);

  BLI_assert(joint_size_at_depth(1, 0) == 2);

  BLI_assert(joint_size_at_depth(2, 0) == 4);
  BLI_assert(joint_size_at_depth(2, 1) == 2);

  BLI_assert(joint_size_at_depth(3, 0) == 8);
  BLI_assert(joint_size_at_depth(3, 1) == 4);
  BLI_assert(joint_size_at_depth(3, 2) == 2);

  BLI_assert(joints_range_at_depth(0) == IndexRange(0, 1));
  BLI_assert(joints_range_at_depth(1) == IndexRange(1, 2));
  BLI_assert(joints_range_at_depth(2) == IndexRange(1 + 2, 4));
  BLI_assert(joints_range_at_depth(2) == IndexRange(1 + 2 + 4, 8));
}

static OffsetIndices<int> fill_buckets_linear(const int total_elements,
                                              const int total_depth,
                                              MutableSpan<int> r_offsets)
{
  for (const int i : r_offsets.index_range().drop_back(1)) {
    r_offsets[i] = (int64_t(total_elements) * i) >> total_depth;
  }
  r_offsets.last() = total_elements;
  return r_offsets.as_span();
}

template<typename FuncT>
static void for_each_to_bottom(const OffsetIndices<int> buckets_offsets,
                               const int total_depth,
                               const FuncT &func)
{
  for (const int depth_i : IndexRange(total_depth)) {
    const IndexRange joints_range = joints_range_at_depth(depth_i);
    for (const int joint_i : joints_range.index_range()) {
      const IndexRange joint_buckets = joint_buckets_range_at_depth(total_depth, depth_i, joint_i);
      func(buckets_offsets[joint_buckets], joints_range[joint_i], depth_i);
    }
  }
}

template<typename FuncT>
static void for_each_to_top(const OffsetIndices<int> buckets_offsets,
                            const int total_depth,
                            const FuncT &func)
{
  const IndexRange depth_range(total_depth);
  for (const int r_depth_i : depth_range.drop_front(1)) {
    const int depth_i = depth_range.last(r_depth_i);
    const IndexRange joints_range = joints_range_at_depth(depth_i);
    const IndexRange prev_joints_range = joints_range_at_depth(depth_i + 1);
    for (const int joint_i : joints_range.index_range()) {
      const IndexRange joint_buckets = joint_buckets_range_at_depth(total_depth, depth_i, joint_i);
      const int2 sub_joints = int2(prev_joints_range.start()) + int2(joint_i * 2) + int2(0, 1);
      func(buckets_offsets[joint_buckets], joints_range[joint_i], sub_joints, r_depth_i);
    }
  }
}

template<typename LeafFunc>
static void for_each_leaf(const OffsetIndices<int> buckets_offsets,
                          const int total_depth,
                          const LeafFunc &leaf_func)
{
  const IndexRange joints_range = joints_range_at_depth(total_depth);
  for (const int joint_i : joints_range.index_range()) {
    leaf_func(buckets_offsets[joint_i], int(joints_range[joint_i]), total_depth);
  }
}

static void from_positions(const Span<float3> positions,
                           const OffsetIndices<int> buckets_offsets,
                           const int total_depth,
                           MutableSpan<int> indices)
{
  array_utils::fill_index_range<int>(indices);

  for_each_to_bottom(
      buckets_offsets,
      total_depth,
      [&](const IndexRange data_range, const int /*data_index*/, const int nesting_i) {
        const int axis_index = math::mod_periodic(nesting_i, 3);
        MutableSpan<int> segment = indices.slice(data_range);
        std::sort(segment.begin(), segment.end(), [&](const int a, const int b) {
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
                [&](const IndexRange bucket_range, const int joint_index, const int /*depth_i*/) {
                  const Span<T> bucket = src_buckets_data.slice(bucket_range);
                  dst_joints_data[joint_index] = std::accumulate(
                      bucket.begin(), bucket.end(), T(0));
                });

  for_each_to_top(buckets_offsets,
                  total_depth,
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
  for_each_to_bottom(
      buckets_offsets,
      total_depth,
      [&](const IndexRange data_range, const int data_index, const int /*nesting_i*/) {
        dst_joints_data[data_index] *= math::rcp(double(data_range.size()));
      });
}

static float max_distance(const Span<float3> positions, const float3 centre)
{
  float max_value = 0.0f;
  for (const float3 position : positions) {
    max_value = math::max(max_value, math::distance(centre, position));
  }
  return max_value;
}

static void radius(const OffsetIndices<int> buckets_offsets,
                   const int total_nesting,
                   const Span<float3> src_buckets_data,
                   const Span<float3> src_joints_data,
                   MutableSpan<float> dst_joints_data)
{
  for_each_leaf(buckets_offsets,
                total_nesting,
                [&](const IndexRange bucket_range, const int joint_index, const int /*depth_i*/) {
                  const Span<float3> src_bucket_data = src_buckets_data.slice(bucket_range);
                  dst_joints_data[joint_index] = max_distance(src_bucket_data,
                                                              src_joints_data[joint_index]);
                });

  for_each_to_top(buckets_offsets,
                  total_nesting,
                  [&](const IndexRange /*buckets_range*/,
                      const int joint_index,
                      const int2 sub_joints,
                      const int /*depth_i*/) {
                    const float3 joint_position = src_joints_data[joint_index];
                    const float left_sub_radius = math::distance(joint_position,
                                                                 src_joints_data[sub_joints[0]]) +
                                                  dst_joints_data[sub_joints[0]];
                    const float right_sub_radius = math::distance(joint_position,
                                                                  src_joints_data[sub_joints[1]]) +
                                                   dst_joints_data[sub_joints[1]];
                    dst_joints_data[joint_index] = math::max(left_sub_radius, right_sub_radius);
                  });
}

static void radius_exact(const OffsetIndices<int> buckets_offsets,
                         const int total_nesting,
                         const Span<float3> src_buckets_data,
                         const Span<float3> src_joints_data,
                         MutableSpan<float> dst_joints_data)
{
  for_each_leaf(buckets_offsets,
                total_nesting,
                [&](const IndexRange bucket_range, const int joint_index, const int /*depth_i*/) {
                  const Span<float3> src_bucket_data = src_buckets_data.slice(bucket_range);
                  dst_joints_data[joint_index] = max_distance(src_bucket_data,
                                                              src_joints_data[joint_index]);
                });

  for_each_to_top(buckets_offsets,
                  total_nesting,
                  [&](const IndexRange buckets_range,
                      const int joint_index,
                      const int2 /*sub_joints*/,
                      const int /*depth_i*/) {
                    const Span<float3> src_bucket_data = src_buckets_data.slice(buckets_range);
                    dst_joints_data[joint_index] = max_distance(src_bucket_data,
                                                                src_joints_data[joint_index]);
                  });
}

static float3 akdt_average()
{
  return {};
}

}  // namespace akdbt

static float minimal_dinstance_to(const float radius,
                                  const int distance_power,
                                  const float precision)
{
  /**
   * Centre of the sphere with a points inside the centre and some virtual point which is the most
   * near to the sempler:
   *
   * 1 / distance <= 1 / (distance - radius).
   *
   * They are equal at ~infinite distance. But with error they can be treat as equal much near
   * To approximate this use some factor (1 <= precision <= infinite) to say how large error is
   * acceptable:
   *
   * 1 / distance >= 1 / (distance - radius) * precision.
   *
   * Version in arbitrary degree:
   *
   * (1 / distance) ^ distance_power >= precision * (1 / (distance - radius)) ^ distance_power.
   * */
  return radius / (math::pow<float>(precision, distance_power) - 1.0f);
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
    const int total_joints = akdbt::total_joints_at_start(total_depth);
    BLI_assert(count_bits_i(total_joints) == 1);
    BLI_assert(total_joints == int(1 << (total_depth)));

    std::cout << total_depth << ";\n";
    std::cout << total_joints << ";\n";

    Array<int> start_indices(total_joints);
    const OffsetIndices<int> base_offsets = akdbt::fill_buckets_linear(
        positions.size(), total_depth, start_indices);
    std::cout << start_indices << ";\n";

    Array<int> indices(positions.size());
    akdbt::from_positions(positions, base_offsets, total_depth, indices);

    Array<float3> bucket_positions(positions.size());
    array_utils::gather(
        Span<float3>(positions), indices.as_span(), bucket_positions.as_mutable_span());

    Array<float3> joints_positions(total_joints);
    akdbt::mean_sums<float3>(base_offsets, total_depth, bucket_positions, joints_positions);
    akdbt::normalize_for_size<float3>(base_offsets, total_depth, joints_positions);

    Array<float> joints_radii(total_joints);
    akdbt::radius_exact(
        base_offsets, total_depth, bucket_positions, joints_positions, joints_radii);

    std::transform(
        joints_radii.begin(), joints_radii.end(), joints_radii.begin(), [&](const float radius) {
          return minimal_dinstance_to(radius, distance_power_, precision_);
        });

    std::cout << joints_positions << ";\n";
    std::cout << joints_radii << ";\n";

    Array<float3> dst_values(positions.size());

    // return VArray<int>::ForContainer(std::move(indices));
    return VArray<float3>::ForContainer(std::move(dst_values));
  }

 public:
  void for_each_field_input_recursive(FunctionRef<void(const FieldInput &)> fn) const
  {
    positions_field_.node().for_each_field_input_recursive(fn);
    value_field_.node().for_each_field_input_recursive(fn);
  }
};

class BruteForceDifferenceSumFieldInput final : public bke::GeometryFieldInput {
 private:
  const Field<float3> positions_field_;
  const Field<float3> value_field_;
  const int distance_power_;

 public:
  BruteForceDifferenceSumFieldInput(Field<float3> positions_field,
                                    Field<float3> value_field,
                                    const int distance_power)
      : bke::GeometryFieldInput(CPPType::get<float3>(), "BF Weighed Difference Sum"),
        positions_field_(std::move(positions_field)),
        value_field_(std::move(value_field)),
        distance_power_(distance_power)
  {
  }

  GVArray get_varray_for_context(const bke::GeometryFieldContext &context,
                                 const IndexMask & /*mask*/) const final
  {
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

    Array<float3> dst_values(positions.size(), float3(0));
    threading::parallel_for(positions.index_range(), 4096, [&](const IndexRange range) {
      for (const int index : range) {
        const float3 point = positions[index];
        for (const int i : positions.index_range().take_front(index)) {
          const float3 vector = positions[i] - point;
          const float distance = math::length(vector);
          dst_values[index] += vector *
                               math::safe_rcp(math::pow<float>(distance, distance_power_));
        }
        for (const int i : positions.index_range().drop_front(index + 1)) {
          const float3 vector = positions[i] - point;
          const float distance = math::length(vector);
          dst_values[index] += vector *
                               math::safe_rcp(math::pow<float>(distance, distance_power_));
        }
      }
    });

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
  akdbt::test();

  Field<float3> position_field = params.extract_input<Field<float3>>("Position");
  Field<float3> value_field = params.extract_input<Field<float3>>("Value");

  const int power_value = params.extract_input<int>("Power");
  const float precision_value = params.extract_input<float>("Precision");

  if (params.output_is_required("Weighted Difference Sum")) {
    params.set_output("Weighted Difference Sum",
                      Field<float3>(std::make_shared<DifferenceSumFieldInput>(
                          position_field, value_field, power_value, precision_value)));
  }

  if (params.output_is_required("Brute Force Weighted Difference Sum")) {
    params.set_output("Brute Force Weighted Difference Sum",
                      Field<float3>(std::make_shared<BruteForceDifferenceSumFieldInput>(
                          position_field, value_field, power_value)));
  }

  if (params.output_is_required("Weighted Sum")) {
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
