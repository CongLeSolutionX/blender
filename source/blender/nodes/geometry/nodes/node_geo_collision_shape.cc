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
  using ShapeType = bke::CollisionShape::ShapeType;
  const bNode *node = b.node_or_null();
  if (node == nullptr) {
    return;
  }
  const ShapeType type = ShapeType(node->custom1);

  if (ELEM(type, ShapeType::RotatedTranslated, ShapeType::OffsetCenterOfMass)) {
    b.add_input<decl::Vector>("Translation");
  }

  if (ELEM(type, ShapeType::RotatedTranslated)) {
    b.add_input<decl::Rotation>("Rotation");
  }
  if (ELEM(type, ShapeType::Scaled)) {
    b.add_input<decl::Vector>("Scale").default_value(float3(1.0f));
  }
  if (ELEM(type, ShapeType::Box)) {
    b.add_input<decl::Vector>("Size", "SizeVector").default_value(float3(1.0f));
  }
  if (ELEM(type,
           ShapeType::Sphere,
           ShapeType::Cylinder,
           ShapeType::Capsule,
           ShapeType::TaperedCapsule))
  {
    b.add_input<decl::Float>("Radius").default_value(1.0f);
  }
  if (ELEM(type, ShapeType::TaperedCapsule)) {
    b.add_input<decl::Float>("Radius 2").default_value(1.0f);
  }
  if (ELEM(type, ShapeType::Cylinder, ShapeType::Capsule, ShapeType::TaperedCapsule)) {
    b.add_input<decl::Float>("Height").default_value(1.0f);
  }
  if (ELEM(type, ShapeType::Triangle)) {
    b.add_input<decl::Vector>("Point", "Point0");
    b.add_input<decl::Vector>("Point", "Point1");
    b.add_input<decl::Vector>("Point", "Point2");
  }
  if (ELEM(type, ShapeType::ConvexHull)) {
    b.add_input<decl::Geometry>("Geometry")
        .supported_type({GeometryComponent::Type::Mesh,
                         GeometryComponent::Type::Curve,
                         GeometryComponent::Type::PointCloud});
  }
  if (ELEM(type, ShapeType::Mesh)) {
    b.add_input<decl::Geometry>("Mesh").supported_type(GeometryComponent::Type::Mesh);
  }
  if (ELEM(type,
           ShapeType::Scaled,
           ShapeType::OffsetCenterOfMass,
           ShapeType::RotatedTranslated,
           ShapeType::StaticCompound,
           ShapeType::MutableCompound))
  {
    b.add_input<decl::Geometry>("Child Shape")
        .supported_type(GeometryComponent::Type::CollisionShape);
  }

  b.add_output<decl::Geometry>("Shape").propagate_all();
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

static void find_child_shapes(const GeometrySet &geometry_set,
                              Vector<bke::GeometrySet> &r_child_shapes,
                              Vector<float4x4> &r_child_transforms)
{
  const bool has_shape = geometry_set.has_collision_shape();
  const int num_instances = geometry_set.has_instances() ?
                                geometry_set.get_instances()->instances_num() :
                                0;
  const int num_child_shapes = (has_shape ? 1 : 0) + num_instances;

  r_child_shapes.reserve(num_child_shapes);
  r_child_transforms.reserve(num_child_shapes);
  if (geometry_set.has_collision_shape()) {
    r_child_shapes.append(geometry_set);
    r_child_transforms.append(float4x4::identity());
  }
  if (geometry_set.has_instances()) {
    const bke::Instances &instances = *geometry_set.get_instances();
    const Span<bke::InstanceReference> references = instances.references();
    const Span<float4x4> transforms = instances.transforms();
    for (const int ref_index : instances.reference_handles()) {
      const GeometrySet &ref_geometry_set = references[ref_index].geometry_set();
      const float4x4 &transform = transforms[ref_index];
      if (ref_geometry_set.has_collision_shape()) {
        r_child_shapes.append(ref_geometry_set);
        r_child_transforms.append(transform);
      }
    }
  }
}

/* Keep in sync with the type_items enum in node_rna. */
static bke::CollisionShape make_collision_shape_from_type(const bke::CollisionShapeType type,
                                                          GeoNodeExecParams params)
{
  using ShapeType = bke::CollisionShapeType;

  switch (type) {
    case ShapeType::Empty: {
      return {};
    }
    case ShapeType::Sphere: {
      const float radius = params.extract_input<float>("Radius");
      return bke::collision_shapes::make_sphere(radius);
    }
    case ShapeType::Box: {
      const float3 half_extent = 0.5f * params.extract_input<float3>("SizeVector");
      return bke::collision_shapes::make_box(half_extent);
    }
    case ShapeType::Triangle: {
      const float3 point0 = params.extract_input<float3>("Point0");
      const float3 point1 = params.extract_input<float3>("Point1");
      const float3 point2 = params.extract_input<float3>("Point2");
      return bke::collision_shapes::make_triangle(point0, point1, point2);
    }
    case ShapeType::Capsule: {
      const float radius = params.extract_input<float>("Radius");
      const float height = params.extract_input<float>("Height");
      return bke::collision_shapes::make_capsule(radius, height);
    }
    case ShapeType::TaperedCapsule: {
      const float top_radius = params.extract_input<float>("Radius");
      const float bottom_radius = params.extract_input<float>("Radius 2");
      const float height = params.extract_input<float>("Height");
      return bke::collision_shapes::make_tapered_capsule(top_radius, bottom_radius, height);
    }
    case ShapeType::Cylinder: {
      const float radius = params.extract_input<float>("Radius");
      const float height = params.extract_input<float>("Height");
      return bke::collision_shapes::make_cylinder(radius, height);
    }
    case ShapeType::ConvexHull: {
      const GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
      const VArray<float3> points = gather_points(geometry_set);
      return bke::collision_shapes::make_convex_hull(points);
    }
    case ShapeType::StaticCompound: {
      const GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
      Vector<bke::GeometrySet> child_shapes;
      Vector<float4x4> child_transforms;
      find_child_shapes(geometry_set, child_shapes, child_transforms);
      return bke::collision_shapes::make_static_compound(child_shapes, child_transforms);
    }
    case ShapeType::MutableCompound: {
      const GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
      Vector<bke::GeometrySet> child_shapes;
      Vector<float4x4> child_transforms;
      find_child_shapes(geometry_set, child_shapes, child_transforms);
      return bke::collision_shapes::make_mutable_compound(child_shapes, child_transforms);
    }
    case ShapeType::RotatedTranslated: {
      const GeometrySet geometry_set = params.extract_input<GeometrySet>("Child Shape");
      const float3 translation = params.extract_input<float3>("Translation");
      const math::Quaternion rotation = params.extract_input<math::Quaternion>("Rotation");
      return bke::collision_shapes::make_rotated_translated(geometry_set, rotation, translation);
    }
    case ShapeType::Scaled: {
      const GeometrySet geometry_set = params.extract_input<GeometrySet>("Child Shape");
      const float3 scale = params.extract_input<float3>("Scale");
      return bke::collision_shapes::make_scaled_shape(geometry_set, scale);
    }
    case ShapeType::OffsetCenterOfMass: {
      const GeometrySet geometry_set = params.extract_input<GeometrySet>("Child Shape");
      const float3 offset = params.extract_input<float3>("Translation");
      return bke::collision_shapes::make_offset_center_of_mass_shape(geometry_set, offset);
    }
    case ShapeType::Mesh: {
      const GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");
      if (!geometry_set.has_mesh()) {
        return bke::collision_shapes::make_sphere(1.0f);
      }
      return bke::collision_shapes::make_mesh(*geometry_set.get_mesh());
    }
    case ShapeType::HeightField: {
      // TODO
      BLI_assert_unreachable();
      return {};
    }
    case ShapeType::SoftBody: {
      // TODO
      BLI_assert_unreachable();
      return {};
    }
  }
  return {};
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const auto shape_type = bke::CollisionShapeType(params.node().custom1);

  bke::CollisionShape shape = make_collision_shape_from_type(shape_type, params);

  params.set_output("Shape", GeometrySet::from_collision_shape(new bke::CollisionShape(shape)));
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

  geo_node_type_base(&ntype, GEO_NODE_COLLISION_SHAPE, "Collision Shape", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_collision_shape_cc
