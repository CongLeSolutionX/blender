/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_node.hh"
#include "BKE_physics_geometry.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_body_mass_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Float>("Mass").field_source();
  b.add_output<decl::Matrix>("Inertia").field_source();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  params.set_output("Mass",
                    AttributeFieldInput::Create<float>(bke::PhysicsGeometry::attribute_name(
                        bke::PhysicsGeometry::BodyAttribute::mass)));
  params.set_output("Inertia",
                    AttributeFieldInput::Create<float4x4>(bke::PhysicsGeometry::attribute_name(
                        bke::PhysicsGeometry::BodyAttribute::inertia)));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_INPUT_BODY_MASS, "Body Mass", NODE_CLASS_INPUT);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_input_body_mass_cc
