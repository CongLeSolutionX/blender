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

static constexpr int min_size = 10;

static int akdt_nesting_for_size(const int total_elements)
{
  int levels = 0;
  for (int total_iter = total_elements; total_iter > min_size; total_iter = total_iter / 2) {
    levels++;
  }
  return levels;
}

static int akdt_total_for_base(const int total_nesting)
{
  return int((int64_t(1) << (total_nesting + 1)) - 1);
}

static int n_levels_sum(const int total_nesting)
{
  return int((int64_t(1) << (total_nesting + 1)) - 1);
}

static IndexRange level_range(const int nesting_i)
{
  const int start = n_levels_sum(nesting_i - 1);
  const int size = 1 << nesting_i;
  return IndexRange::from_begin_size(start, size);
}

static IndexRange level_data_range(const int nesting_i)
{
  const IndexRange range = level_range(nesting_i);
  return IndexRange::from_begin_size(range.start() + nesting_i * 2, range.size() + 2);
}

static void akdt_base_offsets(const int total_elements,
                              const int total_nesting,
                              Array<int> &r_offsets)
{
  const int total_base = 1 << total_nesting;
  r_offsets.reinitialize(total_base + 1);

  for (const int i : IndexRange(total_base)) {
    r_offsets[i] = total_elements * i / total_base;
  }
  r_offsets.as_mutable_span().last() = total_elements;
}

template<typename FuncT>
static void for_each_to_bottom(const OffsetIndices<int> base_offsets,
                               const int total_nesting,
                               const FuncT &func)
{
  BLI_assert(base_offsets.size() == int(1 << total_nesting));
  for (const int nesting_i : IndexRange(total_nesting + 1)) {
    const int segment_size = (1 << (total_nesting - nesting_i));
    const int total_segments = 1 << nesting_i;
    const int data_start = n_levels_sum(nesting_i - 1);

    for (const int segment_i : IndexRange(total_segments)) {
      const IndexRange data_segment = IndexRange::from_begin_size(segment_size * segment_i,
                                                                  segment_size);
      const IndexRange data_range = base_offsets[data_segment];
      const int data_index = data_start + segment_i;
      func(data_range, data_index, nesting_i);
    }
  }
}

template<typename FuncT>
static void for_each_to_top(const OffsetIndices<int> base_offsets,
                            const int total_nesting,
                            const FuncT &func)
{
  BLI_assert(base_offsets.size() == int(1 << total_nesting));
  for (const int nesting_i : IndexRange(total_nesting + 1)) {
    const int segment_size = 1 << nesting_i;
    const int total_segments = (1 << (total_nesting - nesting_i));
    const int data_start = n_levels_sum(total_nesting - nesting_i);

    for (const int segment_i : IndexRange(total_segments)) {
      const IndexRange data_segment = IndexRange::from_begin_size(segment_size * segment_i,
                                                                  segment_size);
      const IndexRange data_range = base_offsets[data_segment];
      const int data_index = data_start + segment_i;
      func(data_range, data_index, nesting_i);
    }
  }
}

template<typename LeafFunc, typename JoinFunc>
static void for_each_to_top(const OffsetIndices<int> base_offsets,
                            const int total_nesting,
                            const LeafFunc &leaf_func,
                            const JoinFunc &join_func)
{
  const IndexRange base_range = level_range(total_nesting);
  for (const int base_i : base_offsets.index_range()) {
    leaf_func(base_offsets[base_i], int(base_range[base_i]), total_nesting);
  }

  BLI_assert(base_offsets.size() == int(1 << total_nesting));
  for (const int nesting_i : IndexRange(total_nesting)) {
    const int total_segments = (1 << (total_nesting - nesting_i));

    const IndexRange level_range_value = level_range(total_nesting - nesting_i - 1);
    const IndexRange nested_level_range = level_range(total_nesting - nesting_i);
    BLI_assert(level_range_value.size() * 2 == nested_level_range.size());

    for (const int segment_i : IndexRange(total_segments)) {
      const int2 sub_indices = int2(nested_level_range.start()) + int2(segment_i) * 2 + int2(0, 1);
      join_func(level_range_value[segment_i], sub_indices, total_nesting - nesting_i - 1);
    }
  }
}

static void akdt_from_positions(const Span<float3> positions,
                                const OffsetIndices<int> offsets,
                                const int total_nesting,
                                MutableSpan<int> indices)
{
  array_utils::fill_index_range<int>(indices);

  for_each_to_bottom(
      offsets,
      total_nesting,
      [&](const IndexRange data_range, const int /*data_index*/, const int nesting_i) {
        const int axis_index = math::mod_periodic(nesting_i, 3);
        MutableSpan<int> segment = indices.slice(data_range);
        std::sort(segment.begin(), segment.end(), [&](const int a, const int b) {
          return positions[a][axis_index] < positions[b][axis_index];
        });
      });
}

static void akdt_mean_sums(const GroupedSpan<float3> data,
                           const int total_nesting,
                           MutableSpan<float3> dst_values)
{

  for_each_to_top(
      data.offsets,
      total_nesting,
      [&](const IndexRange range, const int index, const int /*nesting_i*/) {
        const Span<float3> src_base_segment = data.data.slice(range);
        dst_values[index] = std::accumulate(
            src_base_segment.begin(), src_base_segment.end(), float3(0.0f));
      },
      [&](const int index, const int2 sub_indices, const int /*nesting_i*/) {
        dst_values[index] = dst_values[sub_indices[0]] + dst_values[sub_indices[1]];
      });
}

static void akdt_normalize_for_size(const OffsetIndices<int> base_offsets,
                                    const int total_nesting,
                                    MutableSpan<float3> dst_values)
{
  for (const int nesting_i : IndexRange(total_nesting + 1)) {
    MutableSpan<float3> level_dst_values = dst_values.slice(level_range(nesting_i));
    const int segment_size = (1 << (total_nesting - nesting_i));
    for (const int i : level_dst_values.index_range()) {
      const IndexRange data_range(segment_size * i, segment_size);
      // std::cout << base_offsets[data_range].size() << ";\n";
      level_dst_values[i] /= base_offsets[data_range].size();
    }
  }
}

static float max_distance(const Span<float3> positions, const float3 centre)
{
  float max_value = 0.0f;
  for (const float3 position : positions) {
    max_value = math::max(max_value, math::distance(centre, position));
  }
  return max_value;
}

static void akdt_radius(const OffsetIndices<int> base_offsets,
                        const int total_nesting,
                        const Span<float3> src_values,
                        const Span<float3> src_data,
                        MutableSpan<float> dst_radius)
{
  {
    const Span<float3> level_src_data_values = src_data.slice(level_range(total_nesting));
    MutableSpan<float> level_dst_radius = dst_radius.slice(level_range(total_nesting));
    BLI_assert(level_dst_radius.size() == base_offsets.size());
    BLI_assert(level_src_data_values.size() == base_offsets.size());
    for (const int i : base_offsets.index_range()) {
      level_dst_radius[i] = max_distance(src_values.slice(base_offsets[i]),
                                         level_src_data_values[i]);
    }
  }

  for (const int r_nesting_i : IndexRange(total_nesting)) {
    const int nesting_i = total_nesting - r_nesting_i - 1;
    const int prev_nesting_i = nesting_i + 1;

    const Span<float> prev_level_dst_radii = dst_radius.slice(level_range(prev_nesting_i));
    const Span<float3> prev_level_dst_centre = src_data.slice(level_range(prev_nesting_i));

    MutableSpan<float> level_dst_radii = dst_radius.slice(level_range(nesting_i));
    const Span<float3> level_dst_centre = src_data.slice(level_range(nesting_i));

    BLI_assert(prev_level_dst_radii.size() == level_dst_radii.size() * 2);

    for (const int i : level_dst_radii.index_range()) {
      level_dst_radii[i] = math::max(
          math::distance(level_dst_centre[i], prev_level_dst_centre[i * 2 + 0]) +
              prev_level_dst_radii[i * 2 + 0],
          math::distance(level_dst_centre[i], prev_level_dst_centre[i * 2 + 1]) +
              prev_level_dst_radii[i * 2 + 1]);
    }
  }
}

static void akdt_radius_exact(const GroupedSpan<float3> data,
                              const int total_nesting,
                              const Span<float3> src_data,
                              MutableSpan<float> dst_radius)
{
  for (const int nesting_i : IndexRange(total_nesting + 1)) {
    const Span<float3> level_src_data_values = src_data.slice(level_range(nesting_i));
    MutableSpan<float> level_dst_radius = dst_radius.slice(level_range(nesting_i));

    const int segment_size = (1 << (total_nesting - nesting_i));
    for (const int i : level_dst_radius.index_range()) {
      const IndexRange data_range(segment_size * i, segment_size);
      level_dst_radius[i] = max_distance(data[data_range], level_src_data_values[i]);
    }
  }
}

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

static float3 akdt_average()
{
  return {};
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

    const int total_nesting = akdt_nesting_for_size(positions.size());

    Array<int> start_indices;
    akdt_base_offsets(positions.size(), total_nesting, start_indices);
    const OffsetIndices<int> base_offsets(start_indices);
    BLI_assert(count_bits_i(base_offsets.size()) == 1);
    BLI_assert(base_offsets.size() == int(1 << total_nesting));

    std::cout << total_nesting << ";\n";
    std::cout << start_indices << ";\n";

    Array<int> indices(positions.size());
    akdt_from_positions(positions, base_offsets, total_nesting, indices);

    Array<float3> akdt_positions(positions.size());
    array_utils::gather(
        Span<float3>(positions), indices.as_span(), akdt_positions.as_mutable_span());
    const GroupedSpan<float3> adst_data(base_offsets, akdt_positions.as_span());

    const int adst_data_size = n_levels_sum(total_nesting);

    Array<float3> centres(adst_data_size);
    akdt_mean_sums(adst_data, total_nesting, centres);
    akdt_normalize_for_size(base_offsets, total_nesting, centres);

    Array<float> radii(adst_data_size);
    akdt_radius_exact(adst_data, total_nesting, centres, radii);

    std::transform(radii.begin(), radii.end(), radii.begin(), [&](const float radius) {
      return minimal_dinstance_to(radius, distance_power_, precision_);
    });

    std::cout << centres << ";\n";
    std::cout << radii << ";\n";

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
