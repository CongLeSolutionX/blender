/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_listbase.h"
#include "BLI_math_matrix.h"
#include "BLI_math_rotation.h"
#include "BLI_math_vector.h"
#include "BLI_string.h"

#include "RNA_enum_types.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_rotate_euler_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Vector>("Rotation").subtype(PROP_EULER).hide_value();

  const bNode *node = b.node_or_null();
  if (node != nullptr) {
    const auto type = FunctionNodeRotateEulerType(node->custom1);
    switch (type) {
      case FN_NODE_ROTATE_EULER_TYPE_EULER:
        b.add_input<decl::Vector>("Rotate By").subtype(PROP_EULER);
        break;
      case FN_NODE_ROTATE_EULER_TYPE_AXIS_ANGLE:
        b.add_input<decl::Vector>("Axis").default_value({0.0, 0.0, 1.0}).subtype(PROP_XYZ);
        b.add_input<decl::Float>("Angle").subtype(PROP_ANGLE);
        break;
      default:
        BLI_assert_unreachable();
        break;
    }
  }
  b.add_output<decl::Vector>("Rotation");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "type", UI_ITEM_R_EXPAND, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "space", UI_ITEM_R_EXPAND, nullptr, ICON_NONE);
}

static const mf::MultiFunction *get_multi_function(const bNode &bnode)
{
  static auto obj_euler_rot = mf::build::SI2_SO<float3, float3, float3>(
      "Rotate Euler by Euler/Object", [](const float3 &input, const float3 &rotation) {
        float input_mat[3][3];
        eul_to_mat3(input_mat, input);
        float rot_mat[3][3];
        eul_to_mat3(rot_mat, rotation);
        float mat_res[3][3];
        mul_m3_m3m3(mat_res, rot_mat, input_mat);
        float3 result;
        mat3_to_eul(result, mat_res);
        return result;
      });
  static auto obj_AA_rot = mf::build::SI3_SO<float3, float3, float, float3>(
      "Rotate Euler by AxisAngle/Object",
      [](const float3 &input, const float3 &axis, float angle) {
        float input_mat[3][3];
        eul_to_mat3(input_mat, input);
        float rot_mat[3][3];
        axis_angle_to_mat3(rot_mat, axis, angle);
        float mat_res[3][3];
        mul_m3_m3m3(mat_res, rot_mat, input_mat);
        float3 result;
        mat3_to_eul(result, mat_res);
        return result;
      });
  static auto local_euler_rot = mf::build::SI2_SO<float3, float3, float3>(
      "Rotate Euler by Euler/Local", [](const float3 &input, const float3 &rotation) {
        float input_mat[3][3];
        eul_to_mat3(input_mat, input);
        float rot_mat[3][3];
        eul_to_mat3(rot_mat, rotation);
        float mat_res[3][3];
        mul_m3_m3m3(mat_res, input_mat, rot_mat);
        float3 result;
        mat3_to_eul(result, mat_res);
        return result;
      });
  static auto local_AA_rot = mf::build::SI3_SO<float3, float3, float, float3>(
      "Rotate Euler by AxisAngle/Local", [](const float3 &input, const float3 &axis, float angle) {
        float input_mat[3][3];
        eul_to_mat3(input_mat, input);
        float rot_mat[3][3];
        axis_angle_to_mat3(rot_mat, axis, angle);
        float mat_res[3][3];
        mul_m3_m3m3(mat_res, input_mat, rot_mat);
        float3 result;
        mat3_to_eul(result, mat_res);
        return result;
      });
  short type = bnode.custom1;
  short space = bnode.custom2;
  if (type == FN_NODE_ROTATE_EULER_TYPE_AXIS_ANGLE) {
    return space == FN_NODE_ROTATE_EULER_SPACE_OBJECT ?
               static_cast<const mf::MultiFunction *>(&obj_AA_rot) :
               &local_AA_rot;
  }
  if (type == FN_NODE_ROTATE_EULER_TYPE_EULER) {
    return space == FN_NODE_ROTATE_EULER_SPACE_OBJECT ?
               static_cast<const mf::MultiFunction *>(&obj_euler_rot) :
               &local_euler_rot;
  }
  BLI_assert_unreachable();
  return nullptr;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const mf::MultiFunction *fn = get_multi_function(builder.node());
  builder.set_matching_fn(fn);
}

static void node_register()
{
  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_ROTATE_EULER, "Rotate Euler", NODE_CLASS_CONVERTER);
  ntype.draw_buttons = node_layout;
  ntype.declare = node_declare;
  ntype.build_multi_function = node_build_multi_function;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_rotate_euler_cc
