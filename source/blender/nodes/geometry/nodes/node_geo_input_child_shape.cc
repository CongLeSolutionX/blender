/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_collision_shape.hh"
#include "BKE_instances.hh"
#include "BKE_physics_geometry.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_child_shape_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Shape");
  b.add_input<decl::Int>("Index").min(0).description(
      "Index of the child shape to extract from the compound shape");

  b.add_output<decl::Bool>("Valid").description(
      "True if a child shape was found at the given index");
  b.add_output<decl::Geometry>("Child Shape");
  b.add_output<decl::Matrix>("Transform")
      .description("Transform of the child shape relative to the parent shape");
}

template<typename T>
static void set_shape_parameter_output(GeoNodeExecParams params,
                                       const bke::CollisionShape &shape,
                                       const bke::PhysicsShapeParam param)
{
  if (bke::physics_shape_param_valid(shape.type(), param)) {
    params.set_output(bke::physics_shape_param_name(param),
                      bke::physics_shape_get_param<T>(shape.impl(), param));
  }
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const GeometrySet geometry_set = params.extract_input<bke::GeometrySet>("Shape");
  const int index = params.extract_input<int>("Index");

  const bke::CollisionShape *shape = geometry_set.get_collision_shape();
  if (shape == nullptr) {
    params.set_default_remaining_outputs();
    return;
  }

  const Span<bke::GeometrySet> child_geometries = shape->child_geometries();
  const VArray<float4x4> child_transforms = shape->child_transforms();
  if (!child_geometries.index_range().contains(index)) {
    params.set_default_remaining_outputs();
    return;
  }

  params.set_output("Valid", true);
  params.set_output("Child Shape", child_geometries[index]);
  params.set_output("Transform", child_transforms[index]);
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_INPUT_CHILD_SHAPE, "Child Shape", NODE_CLASS_INPUT);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_input_child_shape_cc
