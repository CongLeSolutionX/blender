/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

#include "BLI_any.hh"
#include "BLI_array.hh"
#include "BLI_bounds_types.hh"
#include "BLI_implicit_sharing_ptr.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_string_ref.hh"
#include "BLI_utility_mixins.hh"
#include "BLI_virtual_array_fwd.hh"

#include <functional>
#include <optional>

struct Mesh;

namespace JPH {
class Shape;
}

namespace blender {
class CPPType;
namespace bke {
struct GeometrySet;
}  // namespace bke
}  // namespace blender

namespace blender::bke {

enum class CollisionShapeType {
  Empty = -1,
  Sphere = 0,
  Box = 1,
  Triangle = 2,
  Capsule = 3,
  TaperedCapsule = 4,
  Cylinder = 5,
  ConvexHull = 6,
  StaticCompound = 7,
  MutableCompound = 8,
  RotatedTranslated = 9,
  Scaled = 10,
  OffsetCenterOfMass = 11,
  Mesh = 12,
  HeightField = 13,
  SoftBody = 14,
};

enum class PhysicsShapeParam {
  translation,
  rotation,
  scale,
  size,
  radius,
  radius2,
  height,
  point0,
  point1,
  point2,
};

class CollisionShape {
 public:
  using ShapeType = CollisionShapeType;

 protected:
  JPH::Shape *impl_ = nullptr;
  std::optional<std::string> error_;

 public:
  CollisionShape();
  CollisionShape(const CollisionShape &other);
  CollisionShape(JPH::Shape *impl, std::optional<std::string> error = std::nullopt);
  ~CollisionShape();

  CollisionShape &operator=(const CollisionShape &other);

  bool is_empty() const;

  JPH::Shape &impl();
  const JPH::Shape &impl() const;

  std::optional<StringRef> error() const;

  ShapeType type() const;

  static StringRef type_name(const ShapeType type);

  /** Shape type is supported for dynamic bodies.
   *  Concave shapes and some other types can only be used for static bodies.
   */
  bool supports_motion() const;

  bool is_convex() const;
  bool is_concave() const;

  float3 center_of_mass() const;
  Bounds<float3> local_bounds() const;

  float density() const;
  void set_density(const float);

  float mass() const;
  float4x4 inertia() const;

  GeometrySet create_mesh_instances() const;

  friend class PhysicsGeometry;
};

namespace collision_shapes {

CollisionShape make_empty();
CollisionShape make_box(const float3 &half_extent);
CollisionShape make_sphere(float radius);
CollisionShape make_triangle(const float3 &pt0, const float3 &pt1, const float3 &pt2);
CollisionShape make_convex_hull(const VArray<float3> &points);
CollisionShape make_capsule(float radius, float height);
CollisionShape make_tapered_capsule(float top_radius, float bottom_radius, float height);
CollisionShape make_cylinder(float radius, float height);
CollisionShape make_scaled_shape(const CollisionShape *child_shape, const float3 &scale);
CollisionShape make_offset_center_of_mass_shape(const CollisionShape *child_shape,
                                                const float3 &offset);
CollisionShape make_rotated_translated(const CollisionShape *child_shape,
                                       const math::Quaternion &rotation,
                                       const float3 &translation);
CollisionShape make_mutable_compound(Span<const CollisionShape *> child_shapes,
                                     Span<float4x4> child_transforms);
CollisionShape make_static_compound(Span<const CollisionShape *> child_shapes,
                                    Span<float4x4> child_transforms);
CollisionShape make_mesh(const Mesh &mesh);

}  // namespace collision_shapes

StringRef physics_shape_param_name(PhysicsShapeParam param);
StringRef physics_shape_param_label(const CollisionShapeType shape_type, PhysicsShapeParam param);
bool physics_shape_param_valid(const CollisionShapeType shape_type, PhysicsShapeParam param);
const CPPType &physics_shape_param_type(const PhysicsShapeParam param);
bool physics_shape_has_geometry(const CollisionShapeType shape_type);
blender::Any<> physics_shape_get_param(const JPH::Shape &shape, PhysicsShapeParam param);
template<typename T> T physics_shape_get_param(const JPH::Shape &shape, PhysicsShapeParam param)
{
  return physics_shape_get_param(shape, param).get<T>();
}

}  // namespace blender::bke
