/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_node.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_rigid_body_motion_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Bool>("Smooth").field_source();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  Field<bool> sharp = AttributeFieldInput::Create<bool>("sharp_face");
  params.set_output("Smooth", fn::invert_boolean_field(std::move(sharp)));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_INPUT_RIGID_BODY_MOTION, "Rigid Body Motion", NODE_CLASS_INPUT);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_input_rigid_body_motion_cc
