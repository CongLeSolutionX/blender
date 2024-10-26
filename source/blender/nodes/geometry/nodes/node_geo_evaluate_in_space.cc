/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_index_mask.hh"
#include "BLI_math_bits.h"
#include "BLI_task.hh"
#include "BLI_virtual_array.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_evaluate_in_space_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Vector>("Position").implicit_field(implicit_field_inputs::position);
  b.add_input<decl::Vector>("Value").supports_field().hide_value();

  b.add_input<decl::Int>("Power").default_value(2);

  b.add_input<decl::Float>("Precision")
      .subtype(PROP_FACTOR)
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f);

  b.add_output<decl::Vector>("Weighted Sum").field_source_reference_all();
  b.add_output<decl::Int>("Weighed Difference Sum").field_source_reference_all();
}

static void akdt_recursivly_build(std::array<Span<float>, 3> components, MutableSpan<int> indices)
{
  std::sort(indices.begin(), indices.end(), [&](const int a, const int b) {
    return components[0][a] < components[0][b];
  });

  if (indices.size() < 3) {
    return;
  }

  std::rotate(components.begin(), components.begin() + 1, components.end());

  kdt_recursivly(components, indices.take_front(indices.size() / 2));
  kdt_recursivly(components, indices.drop_front(indices.size() / 2));
}

static Array<int> akdt_from_positions(const Span<float3> positions)
{
  Array<float> x_position_component(positions.size());
  Array<float> y_position_component(positions.size());
  Array<float> z_position_component(positions.size());

  threading::parallel_for(positions.index_range(), 8192, [&](const IndexRange range) {
    const Span<float3> src_range = positions.slice(range);
    std::transform(src_range.begin(),
                   src_range.end(),
                   x_position_component.begin() + range.start(),
                   [](const float3 &position) { return position.x; });
    std::transform(src_range.begin(),
                   src_range.end(),
                   y_position_component.begin() + range.start(),
                   [](const float3 &position) { return position.y; });
    std::transform(src_range.begin(),
                   src_range.end(),
                   z_position_component.begin() + range.start(),
                   [](const float3 &position) { return position.z; });
  });

  Array<int> indices(positions.size());
  array_utils::fill_index_range<int>(indices);
  akdt_recursivly_build({x_position_component.as_span(),
                         y_position_component.as_span(),
                         z_position_component.as_span()},
                        indices);

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

    const Array<int> indices = akdt_from_positions(positions);

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
