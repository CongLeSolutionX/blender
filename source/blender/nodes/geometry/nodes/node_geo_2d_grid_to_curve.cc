/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"

#include "BLI_index_mask.hh"
#include "BLI_task.hh"

#include "GEO_2d_grid_to_curve.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_2d_grid_to_curve_cc {

#ifdef WITH_POTRACE
static constexpr const float smooth_max = geometry::potrace::Params::max_smooth_threshold;
#else
static constexpr const float smooth_max = {};
#endif

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Vector>("Min")
      .default_value(float3(float2(-1.0f), 0.0f))
      .description("Minimum boundary of grid");
  b.add_input<decl::Vector>("Max")
      .default_value(float3(float2(1.0f), 0.0f))
      .description("Maximum boundary of grid");

  b.add_input<decl::Int>("Resolution X").default_value(32).min(2);
  b.add_input<decl::Int>("Resolution Y").default_value(32).min(2);

  b.add_input<decl::Float>("Smooth Threshold").default_value(1.0f).min(0.0f).max(smooth_max);
  b.add_input<decl::Float>("Simplify").default_value(0.2f).min(0.0f);

  b.add_input<decl::Bool>("Grid Value").supports_field().hide_value();

  b.add_output<decl::Geometry>("Curve");
  b.add_output<decl::Int>("Parent Curve").description("Index of curve around").field_on_all();
}

#ifdef WITH_POTRACE

static void fill_line(const float3 first, const float3 last, MutableSpan<float3> points)
{
  const float delta_fator = 1.0f / float(points.size() - 1);
  for (const int64_t i : points.index_range()) {
    points[i] = bke::attribute_math::mix2(float(i) * delta_fator, first, last);
  }
}

class ImageBitMapFieldContext : public FieldContext {
 private:
  int2 data_resolution_;
  int2 resolution_;
  const float2 min_point_;
  const float2 max_point_;

 public:
  ImageBitMapFieldContext(const int2 data_resolution,
                          const int2 resolution,
                          const float2 min_point,
                          const float2 max_point)
      : data_resolution_(data_resolution),
        resolution_(resolution),
        min_point_(min_point),
        max_point_(max_point)
  {
  }

  int64_t points_num() const
  {
    return int64_t(data_resolution_.x) * int64_t(data_resolution_.y);
  }

  GVArray get_varray_for_input(const FieldInput &field_input,
                               const IndexMask & /*mask*/,
                               ResourceScope & /*scope*/) const
  {
    const bke::AttributeFieldInput *attribute_field_input =
        dynamic_cast<const bke::AttributeFieldInput *>(&field_input);
    if (attribute_field_input == nullptr) {
      return {};
    }
    if (attribute_field_input->attribute_name() != "position") {
      return {};
    }

    Array<float3> positions(this->points_num());
    threading::parallel_for(
        IndexRange(resolution_.y),
        4096,
        [&](const IndexRange range) {
          const float delta_fator = 1.0f / float(resolution_.y - 1);
          for (const int y_index : range) {
            const float y_position = bke::attribute_math::mix2(
                float(y_index) * delta_fator, min_point_.y, max_point_.y);
            fill_line(
                float3(min_point_.x, y_position, 0.0f),
                float3(max_point_.x, y_position, 0.0f),
                positions.as_mutable_span().slice(y_index * data_resolution_.x, resolution_.x));
          }
        },
        threading::accumulated_task_sizes(
            [&](const IndexRange range) { return range.size() * resolution_.x; }));

    return VArray<float3>::ForContainer(std::move(positions));
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  const float2 min_point = params.extract_input<float3>("Min").xy();
  const float2 max_point = params.extract_input<float3>("Max").xy();
  const int2 resolution = int2(params.extract_input<int>("Resolution X"),
                               params.extract_input<int>("Resolution Y"));

  if (resolution.x < 2 || resolution.y < 2) {
    params.error_message_add(NodeWarningType::Error, TIP_("Resolution must be greater than 1"));
    params.set_default_remaining_outputs();
    return;
  }

  if (min_point.x == max_point.x || min_point.y == max_point.y) {
    params.error_message_add(NodeWarningType::Error, TIP_("Plane area must be greater than 0"));
    params.set_default_remaining_outputs();
    return;
  }

  const int2 aligned_res = geometry::potrace::aligned_resolution(resolution);
  const ImageBitMapFieldContext context(aligned_res, resolution, min_point, max_point);

  FieldEvaluator evaluator(context, context.points_num());

  Field<bool> input_field = params.extract_input<Field<bool>>("Grid Value");
  evaluator.set_selection(std::move(input_field));
  evaluator.evaluate();

  const std::optional<std::string> parent_curve_id =
      params.get_output_anonymous_attribute_id_if_needed("Parent Curve");

  geometry::potrace::Params curves_params;
  curves_params.resolution = resolution;
  curves_params.smooth_threshold = math::clamp<float>(
      params.extract_input<float>("Smooth Threshold"), 0.0f, smooth_max);
  curves_params.optimization_tolerance = math::max<float>(params.extract_input<float>("Simplify"),
                                                          0.0f);

  potrace_state_t *potrace_image = geometry::potrace::image_from_mask(
      curves_params, evaluator.get_evaluated_selection_as_mask());

  if (potrace_image == nullptr) {
    params.error_message_add(NodeWarningType::Warning, TIP_("Can not generate curve"));
    params.set_default_remaining_outputs();
    return;
  }

  BLI_SCOPED_DEFER([&]() { geometry::potrace::free_image(potrace_image); });

  Curves *curve = geometry::potrace::image_to_curve(potrace_image, parent_curve_id);
  if (curve == nullptr) {
    params.set_default_remaining_outputs();
    return;
  }

  curve->geometry.wrap().transform(geometry::potrace::to_plane(resolution, min_point, max_point));

  params.set_output("Curve", GeometrySet::from_curves(curve));
}

#else

static void node_geo_exec(GeoNodeExecParams params)
{
  params.error_message_add(NodeWarningType::Warning, TIP_("Build without Potrace"));
  params.set_default_remaining_outputs();
}

#endif

static void node_register()
{
  static blender::bke::bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_2D_GRID_TO_CURVE, "2D Grid to Curve", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_2d_grid_to_curve_cc
