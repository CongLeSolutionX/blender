/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_rotation.hh"

#include "BKE_collision_shape.hh"
#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_instances.hh"
#include "BKE_physics_geometry.hh"

#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_collision_shape_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Vector>("Translation");
  b.add_input<decl::Rotation>("Rotation");
  b.add_input<decl::Vector>("Scale").default_value(float3(1.0f));
  b.add_input<decl::Vector>("Size", "SizeVector").default_value(float3(1.0f));
  b.add_input<decl::Float>("Radius").default_value(1.0f);
  b.add_input<decl::Float>("Radius 2").default_value(1.0f);
  b.add_input<decl::Float>("Height").default_value(1.0f);
  b.add_input<decl::Vector>("Point", "Point0");
  b.add_input<decl::Vector>("Point", "Point1");
  b.add_input<decl::Vector>("Point", "Point2");
  b.add_input<decl::Geometry>("Geometry")
      .supported_type({GeometryComponent::Type::Mesh,
                       GeometryComponent::Type::Curve,
                       GeometryComponent::Type::PointCloud});
  b.add_input<decl::Geometry>("Mesh").supported_type(GeometryComponent::Type::Mesh);
  b.add_input<decl::Geometry>("Child Shape").supported_type(GeometryComponent::Type::Physics);
  b.add_output<decl::Geometry>("Shape").propagate_all();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  using ShapeType = bke::CollisionShape::ShapeType;
  node->custom1 = int(ShapeType::Box);
}

static void node_update(bNodeTree *tree, bNode *node)
{
  using ShapeType = bke::CollisionShape::ShapeType;
  const ShapeType type = ShapeType(node->custom1);

  bNodeSocket *socket_iter = static_cast<bNodeSocket *>(node->inputs.first);
  auto next_socket = [&socket_iter]() {
    bNodeSocket *result = socket_iter;
    BLI_assert(result != nullptr);
    socket_iter = socket_iter->next;
    return result;
  };

  bNodeSocket *translation_socket = next_socket();
  bNodeSocket *rotation_socket = next_socket();
  bNodeSocket *scale_socket = next_socket();
  bNodeSocket *size_vector_socket = next_socket();
  bNodeSocket *radius_socket = next_socket();
  bNodeSocket *radius2_socket = next_socket();
  bNodeSocket *height_socket = next_socket();
  bNodeSocket *point0_socket = next_socket();
  bNodeSocket *point1_socket = next_socket();
  bNodeSocket *point2_socket = next_socket();
  bNodeSocket *geometry_socket = next_socket();
  bNodeSocket *mesh_socket = next_socket();
  bNodeSocket *child_shape_socket = next_socket();

  bke::node_set_socket_availability(
      tree,
      translation_socket,
      ELEM(type, ShapeType::RotatedTranslated, ShapeType::OffsetCenterOfMass));
  bke::node_set_socket_availability(
      tree, rotation_socket, ELEM(type, ShapeType::RotatedTranslated));
  bke::node_set_socket_availability(tree, scale_socket, ELEM(type, ShapeType::Scaled));
  bke::node_set_socket_availability(tree, size_vector_socket, ELEM(type, ShapeType::Box));
  bke::node_set_socket_availability(tree,
                                    radius_socket,
                                    ELEM(type,
                                         ShapeType::Sphere,
                                         ShapeType::Cylinder,
                                         ShapeType::Capsule,
                                         ShapeType::TaperedCapsule));
  bke::node_set_socket_availability(tree, radius2_socket, ELEM(type, ShapeType::TaperedCapsule));
  bke::node_set_socket_availability(
      tree,
      height_socket,
      ELEM(type, ShapeType::Cylinder, ShapeType::Capsule, ShapeType::TaperedCapsule));
  const bool use_points = ELEM(type, ShapeType::Triangle);
  bke::node_set_socket_availability(tree, point0_socket, use_points);
  bke::node_set_socket_availability(tree, point1_socket, use_points);
  bke::node_set_socket_availability(tree, point2_socket, use_points);
  bke::node_set_socket_availability(tree, geometry_socket, ELEM(type, ShapeType::ConvexHull));
  bke::node_set_socket_availability(tree, mesh_socket, ELEM(type, ShapeType::Mesh));
  bke::node_set_socket_availability(tree,
                                    child_shape_socket,
                                    ELEM(type,
                                         ShapeType::Scaled,
                                         ShapeType::OffsetCenterOfMass,
                                         ShapeType::RotatedTranslated,
                                         ShapeType::StaticCompound,
                                         ShapeType::MutableCompound));
}

static VArray<float3> gather_points(const GeometrySet &geometry_set)
{
  int span_count = 0;
  int count = 0;
  int total_num = 0;

  Span<float3> positions_span;

  if (const Mesh *mesh = geometry_set.get_mesh()) {
    count++;
    if (const VArray positions = *mesh->attributes().lookup<float3>("position")) {
      if (positions.is_span()) {
        span_count++;
        positions_span = positions.get_internal_span();
      }
      total_num += positions.size();
    }
  }

  if (const PointCloud *points = geometry_set.get_pointcloud()) {
    count++;
    if (const VArray positions = *points->attributes().lookup<float3>("position")) {
      if (positions.is_span()) {
        span_count++;
        positions_span = positions.get_internal_span();
      }
      total_num += positions.size();
    }
  }

  if (const Curves *curves_id = geometry_set.get_curves()) {
    count++;
    span_count++;
    const bke::CurvesGeometry &curves = curves_id->geometry.wrap();
    positions_span = curves.evaluated_positions();
    total_num += positions_span.size();
  }

  if (count == 0) {
    return nullptr;
  }

  /* If there is only one positions virtual array and it is already contiguous, avoid copying
   * all of the positions and instead pass the span directly to the convex hull function. */
  if (span_count == 1 && count == 1) {
    return VArray<float3>::ForSpan(positions_span);
  }

  Array<float3> positions(total_num);
  int offset = 0;

  if (const Mesh *mesh = geometry_set.get_mesh()) {
    if (const VArray varray = *mesh->attributes().lookup<float3>("position")) {
      varray.materialize(positions.as_mutable_span().slice(offset, varray.size()));
      offset += varray.size();
    }
  }

  if (const PointCloud *points = geometry_set.get_pointcloud()) {
    if (const VArray varray = *points->attributes().lookup<float3>("position")) {
      varray.materialize(positions.as_mutable_span().slice(offset, varray.size()));
      offset += varray.size();
    }
  }

  if (const Curves *curves_id = geometry_set.get_curves()) {
    const bke::CurvesGeometry &curves = curves_id->geometry.wrap();
    Span<float3> array = curves.evaluated_positions();
    positions.as_mutable_span().slice(offset, array.size()).copy_from(array);
    offset += array.size();
  }

  return VArray<float3>::ForContainer(positions);
}

static bke::CollisionShape::Ptr get_collision_shape(const bke::GeometrySet &geometry_set)
{
  if (!geometry_set.has_physics()) {
    return nullptr;
  }
  const bke::PhysicsGeometry *physics = geometry_set.get_physics();
  if (physics->shapes_num() == 0) {
    return nullptr;
  }
  return physics->state().shapes().first();
}

static void find_child_shapes(const GeometrySet &geometry_set,
                              Vector<bke::CollisionShapePtr> &r_child_shapes,
                              Vector<float4x4> &r_child_transforms)
{
  const int num_shapes = geometry_set.has_physics() ? geometry_set.get_physics()->shapes_num() : 0;
  const int num_instances = geometry_set.has_instances() ?
                                geometry_set.get_instances()->instances_num() :
                                0;

  r_child_shapes.reserve(num_shapes + num_instances);
  r_child_transforms.reserve(num_shapes + num_instances);
  if (geometry_set.has_physics()) {
    for (const bke::CollisionShapePtr &child_shape : geometry_set.get_physics()->state().shapes())
    {
      r_child_shapes.append(child_shape);
      r_child_transforms.append(float4x4::identity());
    }
  }
  if (geometry_set.has_instances()) {
    const bke::Instances &instances = *geometry_set.get_instances();
    const Span<bke::InstanceReference> references = instances.references();
    const Span<float4x4> transforms = instances.transforms();
    for (const int ref_index : instances.reference_handles()) {
      const GeometrySet &ref_geometry_set = references[ref_index].geometry_set();
      const float4x4 &transform = transforms[ref_index];
      if (ref_geometry_set.has_physics()) {
        for (const bke::CollisionShapePtr &child_shape :
             ref_geometry_set.get_physics()->state().shapes())
        {
          r_child_shapes.append(child_shape);
          r_child_transforms.append(transform);
        }
      }
    }
  }
}

/* Keep in sync with the type_items enum in node_rna. */
static bke::CollisionShape *make_collision_shape_from_type(
    const bke::CollisionShape::ShapeType type, GeoNodeExecParams params)
{
  using ShapeType = bke::CollisionShape::ShapeType;

  switch (type) {
    case ShapeType::Sphere: {
      const float radius = params.extract_input<float>("Radius");
      return new bke::SphereCollisionShape(radius);
    }
    case ShapeType::Box: {
      const float3 half_extent = 0.5f * params.extract_input<float3>("SizeVector");
      return new bke::BoxCollisionShape(half_extent);
    }
    case ShapeType::Triangle: {
      const float3 point0 = params.extract_input<float3>("Point0");
      const float3 point1 = params.extract_input<float3>("Point1");
      const float3 point2 = params.extract_input<float3>("Point2");
      return new bke::TriangleCollisionShape(point0, point1, point2);
    }
    case ShapeType::Capsule: {
      const float radius = params.extract_input<float>("Radius");
      const float height = params.extract_input<float>("Height");
      return new bke::CapsuleCollisionShape(radius, height);
    }
    case ShapeType::TaperedCapsule: {
      const float top_radius = params.extract_input<float>("Radius");
      const float bottom_radius = params.extract_input<float>("Radius 2");
      const float height = params.extract_input<float>("Height");
      return new bke::TaperedCapsuleCollisionShape(top_radius, bottom_radius, height);
    }
    case ShapeType::Cylinder: {
      const float radius = params.extract_input<float>("Radius");
      const float height = params.extract_input<float>("Height");
      return new bke::CylinderCollisionShape(radius, height);
    }
    case ShapeType::ConvexHull: {
      const GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
      const VArray<float3> points = gather_points(geometry_set);
      return new bke::ConvexHullCollisionShape(points);
    }
    case ShapeType::StaticCompound: {
      const GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
      Vector<bke::CollisionShapePtr> child_shapes;
      Vector<float4x4> child_transforms;
      find_child_shapes(geometry_set, child_shapes, child_transforms);
      return new bke::StaticCompoundCollisionShape(child_shapes, child_transforms);
    }
    case ShapeType::MutableCompound: {
      const GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
      Vector<bke::CollisionShapePtr> child_shapes;
      Vector<float4x4> child_transforms;
      find_child_shapes(geometry_set, child_shapes, child_transforms);
      return new bke::MutableCompoundCollisionShape(child_shapes, child_transforms);
    }
    case ShapeType::RotatedTranslated: {
      const GeometrySet geometry_set = params.extract_input<GeometrySet>("Child Shape");
      const float3 translation = params.extract_input<float3>("Translation");
      const math::Quaternion rotation = params.extract_input<math::Quaternion>("Rotation");
      const bke::CollisionShape::Ptr child_shape = get_collision_shape(geometry_set);
      return new bke::RotatedTranslatedCollisionShape(child_shape, rotation, translation);
    }
    case ShapeType::Scaled: {
      const GeometrySet geometry_set = params.extract_input<GeometrySet>("Child Shape");
      const float3 scale = params.extract_input<float3>("Size");
      const bke::CollisionShape::Ptr child_shape = get_collision_shape(geometry_set);
      if (!child_shape) {
        return new bke::SphereCollisionShape(1.0f);
      }
      return new bke::ScaledCollisionShape(child_shape, scale);
    }
    case ShapeType::OffsetCenterOfMass: {
      const GeometrySet geometry_set = params.extract_input<GeometrySet>("Child Shape");
      const float3 offset = params.extract_input<float3>("Translation");
      const bke::CollisionShape::Ptr child_shape = get_collision_shape(geometry_set);
      if (!child_shape) {
        return new bke::SphereCollisionShape(1.0f);
      }
      return new bke::OffsetCenterOfMassShape(child_shape, offset);
    }
    case ShapeType::Mesh: {
      const GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");
      if (!geometry_set.has_mesh()) {
        return new bke::SphereCollisionShape(1.0f);
      }
      return new bke::MeshCollisionShape(*geometry_set.get_mesh());
    }
    case ShapeType::HeightField: {
      // TODO
      BLI_assert_unreachable();
      return nullptr;
    }
    case ShapeType::SoftBody: {
      // TODO
      BLI_assert_unreachable();
      return nullptr;
    }
  }
  return nullptr;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const auto shape_type = bke::CollisionShape::ShapeType(params.node().custom1);

  bke::PhysicsGeometry *physics = new bke::PhysicsGeometry(0, 0, 1);
  physics->state_for_write().shapes_for_write().first() = bke::CollisionShapePtr(
      make_collision_shape_from_type(shape_type, params));

  params.set_output("Shape", GeometrySet::from_physics(physics));
}

static void node_rna(StructRNA *srna)
{
  using ShapeType = bke::CollisionShape::ShapeType;

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

  geo_node_type_base(&ntype, GEO_NODE_COLLISION_SHAPE, "Collision Shape", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.updatefunc = node_update;
  ntype.draw_buttons = node_layout;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_collision_shape_cc
