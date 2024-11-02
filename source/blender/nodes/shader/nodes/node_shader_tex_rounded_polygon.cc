/* SPDX-FileCopyrightText: 2024 Tenkai Raiko
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "node_shader_util.hh"
#include "node_util.hh"

#include "BKE_texture.h"

#include "BLI_noise.hh"

#include "NOD_multi_function.hh"

#include "RNA_access.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

namespace blender::nodes::node_shader_tex_rounded_polygon_cc {

NODE_STORAGE_FUNCS(NodeTexRoundedPolygon)

static void sh_node_tex_rounded_polygon_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();

  b.add_output<decl::Float>("R_gon Field").no_muted_links();
  b.add_output<decl::Vector>("Segment Coordinates").no_muted_links();
  b.add_output<decl::Float>("Max Unit Parameter").no_muted_links();
  b.add_output<decl::Float>("X_axis To Angle Bisector Angle").no_muted_links();

  b.add_input<decl::Vector>("Vector")
      .hide_value()
      .implicit_field(implicit_field_inputs::position)
      .description("(X, Y) components of the input vector. The Z component is ignored");
  b.add_input<decl::Float>("Scale").min(-1000.0f).max(1000.0f).default_value(1.0f).description(
      "Factor by which the input vector is scaled");
  b.add_input<decl::Float>("R_gon Sides")
      .min(2.0f)
      .max(1000.0f)
      .default_value(5.0f)
      .description("Number of rounded polygon sides");
  b.add_input<decl::Float>("R_gon Roundness")
      .min(0.0f)
      .max(1.0f)
      .default_value(0.0f)
      .subtype(PROP_FACTOR)
      .description("Corner roundness of the rounded polygon");
}

static void node_shader_buts_tex_rounded_polygon(uiLayout *layout,
                                                 bContext * /*C*/,
                                                 PointerRNA *ptr)
{
  uiItemR(
      layout, ptr, "normalize_r_gon_parameter", UI_ITEM_R_SPLIT_EMPTY_NAME, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "elliptical_corners", UI_ITEM_R_SPLIT_EMPTY_NAME, nullptr, ICON_NONE);
}

static void node_shader_init_tex_rounded_polygon(bNodeTree * /*ntree*/, bNode *node)
{
  NodeTexRoundedPolygon *tex = MEM_cnew<NodeTexRoundedPolygon>(__func__);
  BKE_texture_mapping_default(&tex->base.tex_mapping, TEXMAP_TYPE_POINT);
  BKE_texture_colormapping_default(&tex->base.color_mapping);
  tex->normalize_r_gon_parameter = false;
  tex->elliptical_corners = false;

  node->storage = tex;
}

static const char *gpu_shader_get_name()
{
  return "node_tex_rounded_polygon";
}

static int node_shader_gpu_tex_rounded_polygon(GPUMaterial *mat,
                                               bNode *node,
                                               bNodeExecData * /*execdata*/,
                                               GPUNodeStack *in,
                                               GPUNodeStack *out)
{
  node_shader_gpu_default_tex_coord(mat, node, &in[0].link);
  node_shader_gpu_tex_mapping(mat, node, in, out);

  const NodeTexRoundedPolygon &storage = node_storage(*node);
  float normalize_r_gon_parameter = storage.normalize_r_gon_parameter;
  float elliptical_corners = storage.elliptical_corners;
  float calculate_r_gon_parameter_field = out[1].hasoutput;
  float calculate_max_unit_parameter = out[2].hasoutput;

  const char *name = gpu_shader_get_name();

  return GPU_stack_link(mat,
                        node,
                        name,
                        in,
                        out,
                        GPU_constant(&normalize_r_gon_parameter),
                        GPU_constant(&elliptical_corners),
                        GPU_constant(&calculate_r_gon_parameter_field),
                        GPU_constant(&calculate_max_unit_parameter));
}

static void node_shader_update_tex_rounded_polygon(bNodeTree *ntree, bNode *node)
{
  (void)ntree;

  bNodeSocket *inVectorSock = bke::node_find_socket(node, SOCK_IN, "Vector");
  bNodeSocket *inR_gonSidesSock = bke::node_find_socket(node, SOCK_IN, "R_gon Sides");
  bNodeSocket *inR_gonRoundnessSock = bke::node_find_socket(node, SOCK_IN, "R_gon Roundness");

  bNodeSocket *outR_gonFieldSock = bke::node_find_socket(node, SOCK_OUT, "R_gon Field");
  bNodeSocket *outMaxUnitParameterSock = bke::node_find_socket(
      node, SOCK_OUT, "Max Unit Parameter");
  bNodeSocket *outX_axisToAngleBisectorAngleSock = bke::node_find_socket(
      node, SOCK_OUT, "X_axis To Angle Bisector Angle");

  node_sock_label(inVectorSock, "Vector 2D");
  node_sock_label(inR_gonSidesSock, "Sides");
  node_sock_label(inR_gonRoundnessSock, "Roundness");

  node_sock_label(outR_gonFieldSock, "Radius");
  node_sock_label(outMaxUnitParameterSock, "Segment Width");
  node_sock_label(outX_axisToAngleBisectorAngleSock, "Segment Rotation");
}

/* Define macros for programming language translation. */
#define atanf atan
#define atan2f atan2
#define cosf cos
#define fabsf abs
#define floorf floor
#define fractf fract
#define sinf sin
#define sqrtf sqrt
#define squaref square
#define tanf tan

#define make_float2 float2
#define make_float4 float4
#define ccl_device

using namespace math;

/* The actual rounded polygon functions are in rounded_polygon_generic.h. */
#include "../../../blenlib/intern/rounded_polygon_generic.h"

/* Undefine macros used for programming language translation. */
#undef atanf
#undef atan2f
#undef cosf
#undef fabsf
#undef floorf
#undef fractf
#undef sinf
#undef sqrtf
#undef squaref
#undef tanf

#undef make_float2
#undef make_float4
#undef ccl_device

class RoundedPolygonFunction : public mf::MultiFunction {
 private:
  bool normalize_r_gon_parameter_;
  bool elliptical_corners_;

  mf::Signature signature_;

 public:
  RoundedPolygonFunction(bool normalize_r_gon_parameter, bool elliptical_corners)
      : normalize_r_gon_parameter_(normalize_r_gon_parameter),
        elliptical_corners_(elliptical_corners)
  {
    signature_ = create_signature();
    this->set_signature(&signature_);
  }

  static mf::Signature create_signature()
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"rounded_polygon", signature};

    builder.single_input<float3>("Vector");
    builder.single_input<float>("Scale");

    builder.single_input<float>("R_gon Sides");
    builder.single_input<float>("R_gon Roundness");

    builder.single_output<float>("R_gon Field", mf::ParamFlag::SupportsUnusedOutput);
    builder.single_output<float3>("Segment Coordinates", mf::ParamFlag::SupportsUnusedOutput);
    builder.single_output<float>("Max Unit Parameter", mf::ParamFlag::SupportsUnusedOutput);
    builder.single_output<float>("X_axis To Angle Bisector Angle",
                                 mf::ParamFlag::SupportsUnusedOutput);

    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;

    const VArray<float3> &coord = params.readonly_single_input<float3>(param++, "Vector");
    const VArray<float> &scale = params.readonly_single_input<float>(param++, "Scale");

    const VArray<float> &r_gon_sides = params.readonly_single_input<float>(param++, "R_gon Sides");
    const VArray<float> &r_gon_roundness = params.readonly_single_input<float>(param++,
                                                                               "R_gon Roundness");

    MutableSpan<float> r_r_gon_field = params.uninitialized_single_output_if_required<float>(
        param++, "R_gon Field");
    MutableSpan<float3> r_segment_coordinates =
        params.uninitialized_single_output_if_required<float3>(param++, "Segment Coordinates");
    MutableSpan<float> r_max_unit_parameter =
        params.uninitialized_single_output_if_required<float>(param++, "Max Unit Parameter");
    MutableSpan<float> r_x_axis_A_angle_bisector =
        params.uninitialized_single_output_if_required<float>(param++,
                                                              "X_axis To Angle Bisector Angle");

    const bool calc_r_gon_field = !r_r_gon_field.is_empty();
    const bool calc_r_gon_parameter_field = !r_segment_coordinates.is_empty();
    const bool calc_max_unit_parameter = !r_max_unit_parameter.is_empty();
    const bool calc_x_axis_A_angle_bisector = !r_x_axis_A_angle_bisector.is_empty();

    mask.foreach_index([&](const int64_t i) {
      float4 out_variables = calculate_out_fields(calc_r_gon_parameter_field,
                                                  calc_max_unit_parameter,
                                                  normalize_r_gon_parameter_,
                                                  elliptical_corners_,
                                                  math::max(r_gon_sides[i], 2.0f),
                                                  math::clamp(r_gon_roundness[i], 0.0f, 1.0f),
                                                  scale[i] * float2(coord[i].x, coord[i].y));

      if (calc_r_gon_field) {
        r_r_gon_field[i] = out_variables.x;
      }
      if (calc_r_gon_parameter_field) {
        r_segment_coordinates[i] = float3(out_variables.y, out_variables.x - 1.0f, 0.0);
      }
      if (calc_max_unit_parameter) {
        r_max_unit_parameter[i] = out_variables.z;
      }
      if (calc_x_axis_A_angle_bisector) {
        r_x_axis_A_angle_bisector[i] = out_variables.w;
      }
    });
  }

  ExecutionHints get_execution_hints() const override
  {
    ExecutionHints hints;
    hints.allocates_array = false;
    hints.min_grain_size = 50;
    return hints;
  }
};

static void sh_node_rounded_polygon_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const NodeTexRoundedPolygon &storage = node_storage(builder.node());
  builder.construct_and_set_matching_fn<RoundedPolygonFunction>(storage.normalize_r_gon_parameter,
                                                                storage.elliptical_corners);
}

}  // namespace blender::nodes::node_shader_tex_rounded_polygon_cc

void register_node_type_sh_tex_rounded_polygon()
{
  namespace file_ns = blender::nodes::node_shader_tex_rounded_polygon_cc;

  static blender::bke::bNodeType ntype;

  sh_fn_node_type_base(
      &ntype, SH_NODE_TEX_ROUNDED_POLYGON, "Rounded Polygon Texture", NODE_CLASS_TEXTURE);
  ntype.declare = file_ns::sh_node_tex_rounded_polygon_declare;
  ntype.draw_buttons = file_ns::node_shader_buts_tex_rounded_polygon;
  ntype.initfunc = file_ns::node_shader_init_tex_rounded_polygon;
  blender::bke::node_type_storage(
      &ntype, "NodeTexRoundedPolygon", node_free_standard_storage, node_copy_standard_storage);
  ntype.gpu_fn = file_ns::node_shader_gpu_tex_rounded_polygon;
  ntype.updatefunc = file_ns::node_shader_update_tex_rounded_polygon;
  ntype.build_multi_function = file_ns::sh_node_rounded_polygon_build_multi_function;

  blender::bke::node_register_type(&ntype);
}
