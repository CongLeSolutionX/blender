/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_collision_shape.hh"
#include "BKE_physics_geometry.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_shape_mass_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Shape");

  b.add_output<decl::Float>("Density").description("Mass density of the shape");
  b.add_output<decl::Float>("Mass").description(
      "Mass of the entire shape based on volume and density");
  b.add_output<decl::Matrix>("Inertia").description("Local inertia tensor of the shape");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const GeometrySet geometry_set = params.extract_input<bke::GeometrySet>("Shape");
  const bke::CollisionShape *shape = geometry_set.get_collision_shape();
  if (shape == nullptr) {
    params.set_default_remaining_outputs();
    return;
  }

  params.set_output("Density", shape->density());
  params.set_output("Mass", shape->mass());
  params.set_output("Inertia", shape->inertia());
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_INPUT_SHAPE_MASS, "Shape Mass", NODE_CLASS_INPUT);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_input_shape_mass_cc
