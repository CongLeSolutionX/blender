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

namespace blender::nodes::node_geo_input_shape_info_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  using ShapeType = bke::CollisionShapeType;
  using ShapeParam = bke::PhysicsShapeParam;
  const bNode *node = b.node_or_null();
  if (node == nullptr) {
    return;
  }

  const ShapeType shape_type = ShapeType(node->custom1);

  auto add_param_output = [&](const ShapeParam attribute) {
    if (bke::physics_shape_param_valid(shape_type, attribute)) {
      const eCustomDataType data_type = bke::cpp_type_to_custom_data_type(
          bke::physics_shape_param_type(attribute));
      const StringRef identifier = bke::physics_shape_param_name(attribute);
      const StringRef name = bke::physics_shape_param_label(shape_type, attribute);
      b.add_output(data_type, name, identifier);
    }
  };

  b.add_input<decl::Geometry>("Collision Shape");

  b.add_output<decl::Bool>("Valid").description("The shape is of the expected type");

  add_param_output(ShapeParam::size);
  add_param_output(ShapeParam::radius);
  add_param_output(ShapeParam::radius2);
  add_param_output(ShapeParam::height);
  add_param_output(ShapeParam::point0);
  add_param_output(ShapeParam::point1);
  add_param_output(ShapeParam::point2);
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  using ShapeType = bke::CollisionShapeType;
  node->custom1 = int(ShapeType::Box);
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
  using ShapeParam = bke::PhysicsShapeParam;
  using ShapeType = bke::CollisionShapeType;
  const ShapeType shape_type = ShapeType(params.node().custom1);

  const GeometrySet geometry_set = params.extract_input<bke::GeometrySet>("Collision Shape");
  const bke::CollisionShape *shape = geometry_set.get_collision_shape();
  if (shape == nullptr || shape->type() != shape_type) {
    params.set_default_remaining_outputs();
    return;
  }

  params.set_output("Valid", true);
  set_shape_parameter_output<float3>(params, *shape, ShapeParam::size);
  set_shape_parameter_output<float>(params, *shape, ShapeParam::radius);
  set_shape_parameter_output<float>(params, *shape, ShapeParam::radius2);
  set_shape_parameter_output<float>(params, *shape, ShapeParam::height);
  set_shape_parameter_output<float3>(params, *shape, ShapeParam::point0);
  set_shape_parameter_output<float3>(params, *shape, ShapeParam::point1);
  set_shape_parameter_output<float3>(params, *shape, ShapeParam::point2);
}

static void node_rna(StructRNA *srna)
{
  using ShapeType = bke::CollisionShapeType;

  /* Make sure this matches implemented types in make_collision_shape_from_type. */
  static EnumPropertyItem type_items[] = {
      {int(ShapeType::Sphere), "SPHERE", 0, "Sphere", ""},
      {int(ShapeType::Box), "BOX", 0, "Box", ""},
      {int(ShapeType::Triangle), "TRIANGLE", 0, "Triangle", ""},
      {int(ShapeType::Capsule), "CAPSULE", 0, "Capsule", ""},
      {int(ShapeType::TaperedCapsule), "TAPERED_CAPSULE", 0, "TaperedCapsule", ""},
      {int(ShapeType::Cylinder), "CYLINDER", 0, "Cylinder", ""},
      {int(ShapeType::ConvexHull), "CONVEX_HULL", 0, "ConvexHull", ""},
      {int(ShapeType::StaticCompound), "STATIC_COMPOUND", 0, "StaticCompound", ""},
      {int(ShapeType::MutableCompound), "MUTABLE_COMPOUND", 0, "MutableCompound", ""},
      {int(ShapeType::RotatedTranslated), "ROTATED_TRANSLATED", 0, "RotatedTranslated", ""},
      {int(ShapeType::Scaled), "SCALED", 0, "Scaled", ""},
      {int(ShapeType::OffsetCenterOfMass), "OFFSET_CENTER_OF_MASS", 0, "OffsetCenterOfMass", ""},
      {int(ShapeType::Mesh), "MESH", 0, "Mesh", ""},
      {int(ShapeType::HeightField), "HEIGHT_FIELD", 0, "HeightField", ""},
      {int(ShapeType::SoftBody), "SOFT_BODY", 0, "SoftBody", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "type",
                    "Type",
                    "",
                    type_items,
                    NOD_inline_enum_accessors(custom1),
                    int(ShapeType::Box));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_INPUT_SHAPE_INFO, "Shape Info", NODE_CLASS_INPUT);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_input_shape_info_cc
