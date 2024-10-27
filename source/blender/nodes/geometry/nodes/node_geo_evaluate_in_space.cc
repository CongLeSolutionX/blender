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

namespace blender::nodes::node_geo_evaluate_in_space_cc {

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

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Vector>("Position").implicit_field(implicit_field_inputs::position);
  b.add_input<decl::Vector>("Value").supports_field().hide_value();

  b.add_input<decl::Int>("Power").default_value(2);

  // b.add_input<decl::Int>("Level");

  b.add_input<decl::Float>("Precision")
      .subtype(PROP_FACTOR)
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f);

  b.add_output<decl::Vector>("Weighted Sum").field_source_reference_all();
  b.add_output<decl::Int>("Weighed Difference Sum").field_source_reference_all();

  // b.add_output<decl::Int>("Index").field_source_reference_all();
}

struct AKDTree {
  Array<int> ranks;
  Array<int> rank_offsets;
  Array<int> binary_shift_rank_offsets;
  int brut_eforce_postfix;
  Array<int> indices;
}

static Array<int> akdt_ranks_for_total(const int total_elements)
{
  constexpr int min_rank = 1 << 6;
  Vector<int, 30> stack;

  for (int rank_iterator = min_rank; rank_iterator > 0; rank_iterator <<= 1) {
    if (total_elements & rank_iterator) {
      stack.append(1 << rank_iterator);
    }
  }
  std::reverse(stack.begin(), stack.end());
  return stack.as_span();
}

static void akdt_from_positions(const Span<float3> positions, MutableSpan<int> indices)
{
  int axis_index = 0;
  const int rank_index = indices.size();
  BLI_assert(count_bits_i(rank_index) == 1);
  for (const int rank_i : IndexRange(bitscan_forward_i(rank_index))) {
    const int grain_size = rank_index >> rank_i;
    const int grains_num = 1 << rank_i;
    for (const int grain_i : IndexRange(grains_num)) {
      MutableSpan<int> segment = indices.slice(grains_num * grain_size, grain_size);
      std::sort(segment.begin(), segment.end(), [&](const int a, const int b) {
        return positions[a][axis_index] < positions[b][axis_index];
      });
    }
    axis_index = math::mod_periodic(axis_index + 1, 3);
  }
}

static AKDTree akdt_from_3d_positions(const Span<float3> positions)
{
  AKDTree tree;
  tree.ranks = akdt_ranks_for_total(positions.size());
  tree.rank_offsets.reinitialize(tree.ranks.size() + 1);
  tree.rank_offsets.copy_fromt(tree.ranks);
  const OffsetIndices<int> rank_offsets = offset_indices::accumulate_counts_to_offsets(tree.rank_offsets);
  tree.brut_eforce_postfix = positions.size() - rank_offsets.total_size();

  tree.binary_shift_rank_offsets.copy_fromt(tree.ranks);
  std::transform(tree.binary_shift_rank_offsets.begin(), tree.binary_shift_rank_offsets.end(), tree.binary_shift_rank_offsets.begin(), [](const int size) {
    return size / 2;
  });
  offset_indices::accumulate_counts_to_offsets(tree.binary_shift_rank_offsets);

  tree.indices.reinitialize(rank_offsets.total_size());
  array_utils::fill_index_range<int>(tree.indices);

  for (const int rank_i : rank_offsets.index_range()) {
    MutableSpan<int> rank_indices = tree.indices.as_mutable_span().slice(rank_offsets[rank_i]);
    akdt_from_positions(positions, rank_indices);
  }

  return tree;
}

static void akdt_mean_sums(const AKDTree &tree, const Span<float3> src_values, MutableSpan<float3> dst_values)
{
  const OffsetIndices<int> rank_offsets(tree.rank_offsets);
  for (const int rank_i : rank_offsets.index_range()) {
    const Span<int> dst_rank_values = tree.indices.as_span().slice(rank_offsets[rank_i]);
    const Span<int> rank_indices = tree.indices.as_span().slice(rank_offsets[rank_i]);
    akdt_from_positions(positions, rank_indices);
  }
}

class DifferenceSumFieldInput final : public bke::GeometryFieldInput {
 private:
  const Field<float3> positions_field_;
  const Field<float3> value_field_;

 public:
  DifferenceSumFieldInput(Field<float3> positions_field, Field<float3> value_field)
      : bke::GeometryFieldInput(CPPType::get<int>(), "Weighed Difference Sum"),
        positions_field_(std::move(positions_field)),
        value_field_(std::move(value_field))
  {
  }

  GVArray get_varray_for_context(const bke::GeometryFieldContext &context,
                                 const IndexMask &mask) const final
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
    const VArraySpan<float3> values = evaluator.get_evaluated<float3>(1);

    const AKDTree tree = akdt_from_3d_positions(positions);

    Array<float3> centres(tree.binary_shift_rank_offsets.total_size());
    akdt_mean_sums(tree, positions, centres);
    
    

    return VArray<int>::ForContainer(std::move(indices));
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

  if (params.output_is_required("Weighed Difference Sum")) {
    params.set_output(
        "Weighed Difference Sum",
        Field<int>(std::make_shared<DifferenceSumFieldInput>(position_field, value_field)));
  }
/*
  if (params.output_is_required("Index")) {
    const int level = params.extract_input<int>("Level");
    params.set_output(
        "Index",
        Field<int>(std::make_shared<DifferenceSumIndexFieldInput>(position_field, value_field, level)));
  }
*/
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
