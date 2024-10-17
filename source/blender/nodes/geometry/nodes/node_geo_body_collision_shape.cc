/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array_utils.hh"

#include "BKE_collision_shape.hh"
#include "BKE_physics_geometry.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_body_collision_shape_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry");
  b.add_input<decl::Int>("Body Index").description("The body to retrieve data from");
  b.add_output<decl::Geometry>("Shape").description("The collision shape of the body");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
  const int body_index = params.extract_input<int>("Body Index");
  if (!geometry_set.has_physics()) {
    params.set_default_remaining_outputs();
    return;
  }

  const bke::PhysicsGeometry &physics = *geometry_set.get_physics();
  const VArraySpan<int> body_shapes =
      bke::physics_attributes::physics_attribute_lookup_or_default<int>(
          physics.attributes(), bke::PhysicsBodyAttribute::collision_shape);
  const int body_shape = body_shapes.index_range().contains(body_index) ? body_shapes[body_index] :
                                                                          -1;
  const Span<bke::InstanceReference> shapes = physics.state().shapes();
  if (!shapes.index_range().contains(body_shape)) {
    params.set_default_remaining_outputs();
    return;
  }

  const bke::CollisionShape *shape = shapes[body_shape].geometry_set().get_collision_shape();
  if (shape == nullptr) {
    params.set_default_remaining_outputs();
    return;
  }
  params.set_output("Shape", GeometrySet::from_collision_shape(new bke::CollisionShape(*shape)));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, GEO_NODE_BODY_COLLISION_SHAPE, "Body Collision Shape", NODE_CLASS_INPUT);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_body_collision_shape_cc
