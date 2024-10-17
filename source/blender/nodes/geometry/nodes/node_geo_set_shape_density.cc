/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_collision_shape.hh"
#include "BKE_node.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_set_shape_density_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry")
      .supported_type(bke::GeometryComponent::Type::CollisionShape);
  b.add_input<decl::Float>("Density").default_value(1000.0f);
  b.add_output<decl::Geometry>("Geometry").propagate_all();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
  const float density = params.extract_input<float>("Density");

  if (geometry_set.has(bke::GeometryComponent::Type::CollisionShape)) {
    bke::CollisionShape *shape = geometry_set.get_collision_shape_for_write();
    shape->set_density(density);
  }

  params.set_output("Geometry", std::move(geometry_set));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SET_SHAPE_DENSITY, "Set Shape Density", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_set_shape_density_cc
