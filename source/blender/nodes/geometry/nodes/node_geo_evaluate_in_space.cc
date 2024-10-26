/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_math_bits.h"
#include "BLI_array_utils.hh"
#include "BLI_index_mask.hh"
#include "BLI_virtual_array.hh"
#include "BLI_task.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_evaluate_in_space_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Vector>("Position").implicit_field(implicit_field_inputs::position);
  b.add_input<decl::Vector>("Value").supports_field().hide_value();

  b.add_input<decl::Int>("Power").default_value(2);

  b.add_input<decl::Float>("Precision").subtype(PROP_FACTOR).min(0.0f).max(1.0f).default_value(0.5f);

  b.add_output<decl::Vector>("Weighted Sum").field_source_reference_all();
  b.add_output<decl::Int>("Weighed Difference Sum").field_source_reference_all();
}

static void invert_permutation(const Span<int> permutation, MutableSpan<int> reverse)
{
  BLI_assert(permutation.size() == reverse.size());
  threading::parallel_for(permutation.index_range(), 2048, [&](const IndexRange range) {
    for (const int64_t i : range) {
      reverse[permutation[i]] = i;
    }
  });
}

static uint64_t map_bits_to_x4(uint64_t bits)
{
  BLI_assert((bits & (~0b00000000'00000000'00000000'00000000'00000000'00000000'11111111'11111111)) == 0);
  bits |= bits << (8 * 3);
  bits &= 0b00000000'00000000'00000000'11111111'00000000'00000000'00000000'11111111;
  bits |= bits << (4 * 3);
  bits &= 0b00000000'00001111'00000000'00001111'00000000'00001111'00000000'00001111;
  bits |= bits << (2 * 3);
  bits &= 0b00000011'00000011'00000011'00000011'00000011'00000011'00000011'00000011;
  bits |= bits << (1 * 3);
  bits &= 0b00010001'00010001'00010001'00010001'00010001'00010001'00010001'00010001;
  return bits;
}

static Array<int> kdt_from_positions(const Span<float3> positions)
{
  Array<int> x_axis_lookup_indices(positions.size());
  Array<int> y_axis_lookup_indices(positions.size());
  Array<int> z_axis_lookup_indices(positions.size());

  {
    Array<int> x_axis_indices(positions.size());
    Array<int> y_axis_indices(positions.size());
    Array<int> z_axis_indices(positions.size());

    array_utils::fill_index_range<int>(x_axis_indices);
    array_utils::fill_index_range<int>(y_axis_indices);
    array_utils::fill_index_range<int>(z_axis_indices);

    std::sort(x_axis_indices.begin(), x_axis_indices.end(), [&](const int a, const int b) { return positions[a].x < positions[b].x; });
    std::sort(y_axis_indices.begin(), y_axis_indices.end(), [&](const int a, const int b) { return positions[a].y < positions[b].y; });
    std::sort(z_axis_indices.begin(), z_axis_indices.end(), [&](const int a, const int b) { return positions[a].z < positions[b].z; });

    invert_permutation(x_axis_indices, x_axis_lookup_indices);
    invert_permutation(y_axis_indices, y_axis_lookup_indices);
    invert_permutation(z_axis_indices, z_axis_lookup_indices);
  }

  Array<int> indices(positions.size());
  array_utils::fill_index_range<int>(indices);
  
  std::sort(indices.begin(), indices.end(), [&](const int a, const int b) {
    const uint32_t a_x_bits = x_axis_lookup_indices[a];
    const uint32_t a_y_bits = y_axis_lookup_indices[a];
    const uint32_t a_z_bits = z_axis_lookup_indices[a];

    const uint32_t b_x_bits = x_axis_lookup_indices[b];
    const uint32_t b_y_bits = y_axis_lookup_indices[b];
    const uint32_t b_z_bits = z_axis_lookup_indices[b];

    const uint64_t a_top_bits = (map_bits_to_x4(a_x_bits >> 16) << 0) |
                                (map_bits_to_x4(a_y_bits >> 16) << 1) |
                                (map_bits_to_x4(a_z_bits >> 16) << 2);

    const uint64_t b_top_bits = (map_bits_to_x4(b_x_bits >> 16) << 0) |
                                (map_bits_to_x4(b_y_bits >> 16) << 1) |
                                (map_bits_to_x4(b_z_bits >> 16) << 2);

    if (LIKELY(a_top_bits != b_top_bits)) {
      return a_top_bits < b_top_bits;
    }

    const uint64_t a_bottom_bits = (map_bits_to_x4(a_x_bits & 0b11111111'11111111) << 0) |
                                   (map_bits_to_x4(a_y_bits & 0b11111111'11111111) << 1) |
                                   (map_bits_to_x4(a_z_bits & 0b11111111'11111111) << 2);

    const uint64_t b_bottom_bits = (map_bits_to_x4(b_x_bits & 0b11111111'11111111) << 0) |
                                   (map_bits_to_x4(b_y_bits & 0b11111111'11111111) << 1) |
                                   (map_bits_to_x4(b_z_bits & 0b11111111'11111111) << 2);

    return a_bottom_bits < b_bottom_bits;
  });

  return indices;
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

    const Array<int> indices = kdt_from_positions(positions);

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
    params.set_output("Weighed Difference Sum", Field<int>(std::make_shared<DifferenceSumFieldInput>(position_field, value_field)));
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
