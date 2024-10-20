/* SPDX-FileCopyrightText: 2004-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#include "BLI_array_utils.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"

#include "BKE_collision_shape.hh"
#include "BKE_material.h"
#include "BKE_mesh.hh"
#include "BKE_physics_geometry.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "physics_geometry_intern.hh"

#ifdef WITH_JOLT
#  include <Jolt/Jolt.h>

#  include <Jolt/Physics/Collision/Shape/BoxShape.h>
#  include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#  include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#  include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#  include <Jolt/Physics/Collision/Shape/MeshShape.h>
#  include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#  include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#  include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#  include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#  include <Jolt/Physics/Collision/Shape/SphereShape.h>
#  include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#  include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>
#  include <Jolt/Physics/Collision/Shape/TriangleShape.h>
#endif

namespace blender::bke {

#ifdef WITH_JOLT

static StringRef shape_type_name(const CollisionShapeType type)
{
  switch (type) {
    case CollisionShapeType::Empty:
      return "None";
    case CollisionShapeType::Sphere:
      return "Sphere";
    case CollisionShapeType::Box:
      return "Box";
    case CollisionShapeType::Triangle:
      return "Triangle";
    case CollisionShapeType::Capsule:
      return "Capsule";
    case CollisionShapeType::TaperedCapsule:
      return "TaperedCapsule";
    case CollisionShapeType::Cylinder:
      return "Cylinder";
    case CollisionShapeType::ConvexHull:
      return "ConvexHull";
    case CollisionShapeType::StaticCompound:
      return "StaticCompound";
    case CollisionShapeType::MutableCompound:
      return "MutableCompound";
    case CollisionShapeType::RotatedTranslated:
      return "RotatedTranslated";
    case CollisionShapeType::Scaled:
      return "Scaled";
    case CollisionShapeType::OffsetCenterOfMass:
      return "OffsetCenterOfMass";
    case CollisionShapeType::Mesh:
      return "Mesh";
    case CollisionShapeType::HeightField:
      return "HeightField";
    case CollisionShapeType::SoftBody:
      return "SoftBody";
  }
  BLI_assert_unreachable();
  return "";
}

#  ifdef WITH_JOLT

/* Limit for shape parameters that may never be zero. */
static constexpr float shape_size_epsilon = 0.0001f;
static constexpr float shape_convex_radius = JPH::cDefaultConvexRadius;
static constexpr JPH::PhysicsMaterial *shape_physics_material = nullptr;

/* Fallback shape that can be used when an input pointer is null but some shape is required. */
static const JPH::Shape *get_shape_or_fallback(const CollisionShape *shape)
{
  if (shape == nullptr || shape->is_empty()) {
    static const JPH::RefConst<JPH::Shape> fallback_shape = JPH::RefConst<JPH::Shape>(
        new JPH::SphereShape(1.0f));
    return fallback_shape.GetPtr();
  }
  return &shape->impl();
}

/* Fallback shape that can be used when an error occurred but some shape is required. */
static CollisionShape construct_shape(const JPH::ShapeSettings &settings)
{
  const JPH::ShapeSettings::ShapeResult result = settings.Create();
  if (result.HasError()) {
    return CollisionShape(new JPH::SphereShape(1.0f), std::string(result.GetError()));
  }
  if (result.IsEmpty()) {
    return CollisionShape(new JPH::SphereShape(1.0f), "Empty shape");
  }
  return CollisionShape(result.Get());
}

static Mesh *shape_to_mesh(const JPH::Shape &shape, const float4x4 &transform)
{
  const JPH::AABox bounds = to_jolt(Bounds<float3>(float3(-FLT_MAX), float3(FLT_MAX)));
  float3 loc;
  math::Quaternion rot;
  float3 scale;
  math::to_loc_rot_scale(transform, loc, rot, scale);

  const int max_triangles = shape.GetStats().mNumTriangles;
  /* Simple triangle soup mesh. */
  Mesh *mesh = BKE_mesh_new_nomain(
      max_triangles * 3, max_triangles * 3, max_triangles, max_triangles * 3);
  bke::mesh_smooth_set(*mesh, false);

  JPH::Shape::GetTrianglesContext context;
  shape.GetTrianglesStart(context, bounds, to_jolt(loc), to_jolt(rot), to_jolt(scale));

  MutableSpan<JPH::Float3> positions = mesh->vert_positions_for_write().cast<JPH::Float3>();
  while (!positions.is_empty()) {
    const int num_triangles = shape.GetTrianglesNext(context, positions.size(), positions.data());
    if (num_triangles == 0) {
      BLI_assert_unreachable();
      break;
    }
    positions = positions.drop_front(num_triangles * 3);
  }

  MutableSpan<int2> edges = mesh->edges_for_write();
  MutableSpan<int> face_offsets = mesh->face_offsets_for_write();
  MutableSpan<int> corner_verts = mesh->corner_verts_for_write();
  MutableSpan<int> corner_edges = mesh->corner_edges_for_write();
  for (const int i : IndexRange(max_triangles)) {
    const int v0 = i * 3;
    const int v1 = v0 + 1;
    const int v2 = v0 + 2;
    face_offsets[i] = v0;
    edges[v0] = {v0, v1};
    edges[v1] = {v1, v2};
    edges[v2] = {v2, v0};
    corner_verts[v0] = v0;
    corner_verts[v1] = v1;
    corner_verts[v2] = v2;
    corner_edges[v0] = v0;
    corner_edges[v1] = v1;
    corner_edges[v2] = v2;
  }
  face_offsets.last() = mesh->faces_num * 3;
  mesh->tag_topology_changed();
  mesh->tag_positions_changed();

  mesh->tag_loose_verts_none();
  mesh->tag_loose_edges_none();
  mesh->tag_overlapping_none();
  mesh->bounds_set_eager(to_blender(shape.GetLocalBounds()));

  BKE_id_material_eval_ensure_default_slot(&mesh->id);

  return mesh;
}

#  endif

CollisionShape::CollisionShape() {}

CollisionShape::CollisionShape(const CollisionShape &other)
{
  *this = other;
}

CollisionShape::CollisionShape(JPH::Shape *impl, std::optional<std::string> error)
    : impl_(impl), error_(error)
{
  if (impl_) {
    impl_->AddRef();
  }
}

CollisionShape::~CollisionShape()
{
  if (impl_) {
    impl_->Release();
  }
}

CollisionShape &CollisionShape::operator=(const CollisionShape &other)
{
  if (impl_) {
    impl_->Release();
  }
  impl_ = other.impl_;
  error_ = other.error_;
  if (impl_) {
    impl_->AddRef();
  }
  return *this;
}

bool CollisionShape::is_empty() const
{
  return impl_ == nullptr;
}

JPH::Shape &CollisionShape::impl()
{
  BLI_assert(impl_ != nullptr);
  return *impl_;
}

const JPH::Shape &CollisionShape::impl() const
{
  BLI_assert(impl_ != nullptr);
  return *impl_;
}

std::optional<StringRef> CollisionShape::error() const
{
  return error_.has_value() ? std::make_optional(error_.value()) : std::nullopt;
}

CollisionShape::ShapeType CollisionShape::type() const
{
  if (!impl_) {
    return CollisionShape::ShapeType::Empty;
  }
  const JPH::EShapeSubType subtype = impl_->GetSubType();
  CollisionShapeType shape_type = to_blender(subtype);
  return shape_type;
}

StringRef CollisionShape::type_name(const CollisionShape::ShapeType type)
{
  return shape_type_name(type);
}

bool CollisionShape::supports_motion() const
{
  return impl_ ? impl_->MustBeStatic() : false;
}

bool CollisionShape::is_convex() const
{
  return impl_ ? impl_->GetType() == JPH::EShapeType::Convex : true;
}

bool CollisionShape::is_concave() const
{
  return impl_ ? impl_->GetType() != JPH::EShapeType::Convex : false;
}

float3 CollisionShape::center_of_mass() const
{
  return impl_ ? to_blender(impl_->GetCenterOfMass()) : float3(0);
}

Bounds<float3> CollisionShape::local_bounds() const
{
  return impl_ ? to_blender(impl_->GetLocalBounds()) : Bounds<float3>{};
}

float CollisionShape::density() const
{
  if (!impl_ || impl_->GetType() != JPH::EShapeType::Convex) {
    return 0.0f;
  }
  const auto *convex_shape = static_cast<const JPH::ConvexShape *>(impl_);
  return to_blender(convex_shape->GetDensity());
}

void CollisionShape::set_density(const float density)
{
  if (!impl_ || impl_->GetType() != JPH::EShapeType::Convex) {
    return;
  }
  auto *convex_shape = static_cast<JPH::ConvexShape *>(impl_);
  convex_shape->SetDensity(density);
}

float CollisionShape::mass() const
{
  if (!impl_) {
    return 0.0f;
  }
  const JPH::MassProperties mass_props = impl_->GetMassProperties();
  return to_blender(mass_props.mMass);
}

float4x4 CollisionShape::inertia() const
{
  if (!impl_) {
    return float4x4::zero();
  }
  const JPH::MassProperties mass_props = impl_->GetMassProperties();
  return to_blender(mass_props.mInertia);
}

GeometrySet CollisionShape::create_mesh_instances() const
{
  if (!impl_) {
    return {};
  }
  switch (impl_->GetSubType()) {
    case JPH::EShapeSubType::ConvexHull:
    case JPH::EShapeSubType::Mesh:
    case JPH::EShapeSubType::Scaled:
    case JPH::EShapeSubType::OffsetCenterOfMass:
    case JPH::EShapeSubType::RotatedTranslated:
    case JPH::EShapeSubType::StaticCompound:
    case JPH::EShapeSubType::MutableCompound:
      return GeometrySet::from_mesh(shape_to_mesh(*impl_, float4x4::identity()));
    default:
      return {};
  }
  BLI_assert_unreachable();
  return {};
}

namespace collision_shapes {

CollisionShape make_empty()
{
  return CollisionShape();
}

CollisionShape make_box(const float3 &half_extent)
{
  JPH::BoxShapeSettings settings(
      to_jolt(math::max(half_extent, float3(shape_convex_radius + 1.0e-6f))),
      shape_convex_radius,
      shape_physics_material);
  return construct_shape(settings);
}

CollisionShape make_sphere(float radius)
{
  JPH::SphereShapeSettings settings(to_jolt(std::max(radius, shape_size_epsilon)),
                                    shape_physics_material);
  return construct_shape(settings);
}

CollisionShape make_triangle(const float3 &pt0, const float3 &pt1, const float3 &pt2)
{
  JPH::TriangleShapeSettings settings(
      to_jolt(pt0), to_jolt(pt1), to_jolt(pt2), shape_convex_radius, shape_physics_material);
  return construct_shape(settings);
}

CollisionShape make_convex_hull(const VArray<float3> &points)
{
  JPH::ConvexHullShapeSettings settings;
  settings.mPoints.reserve(points.size());
  for (const int i : points.index_range()) {
    settings.mPoints.push_back(to_jolt(points[i]));
  }
  settings.mMaxConvexRadius = shape_convex_radius;
  settings.mMaterial = shape_physics_material;
  return construct_shape(settings);
}

CollisionShape make_capsule(float radius, float height)
{
  JPH::CapsuleShapeSettings settings(to_jolt(std::max(0.5f * height, shape_size_epsilon)),
                                     to_jolt(std::max(radius, shape_size_epsilon)),
                                     shape_physics_material);
  return construct_shape(settings);
}

CollisionShape make_tapered_capsule(float top_radius, float bottom_radius, float height)
{
  JPH::TaperedCapsuleShapeSettings settings;
  settings.mTopRadius = to_jolt(std::max(top_radius, shape_size_epsilon));
  settings.mBottomRadius = to_jolt(std::max(bottom_radius, shape_size_epsilon));
  settings.mHalfHeightOfTaperedCylinder = to_jolt(std::max(0.5f * height, shape_size_epsilon));
  settings.mMaterial = shape_physics_material;
  return construct_shape(settings);
}

CollisionShape make_cylinder(float radius, float height)
{
  JPH::CylinderShapeSettings settings(to_jolt(std::max(0.5f * height, shape_convex_radius)),
                                      to_jolt(std::max(radius, shape_convex_radius)),
                                      shape_convex_radius,
                                      shape_physics_material);
  return construct_shape(settings);
}

CollisionShape make_scaled_shape(const CollisionShape *child_shape, const float3 &scale)
{
  JPH::ScaledShapeSettings settings;
  settings.mInnerShapePtr = get_shape_or_fallback(child_shape);
  settings.mScale = to_jolt(math::is_zero(scale) ? float3(1.0f) : scale);
  return construct_shape(settings);
}

CollisionShape make_offset_center_of_mass_shape(const CollisionShape *child_shape,
                                                const float3 &offset)
{
  JPH::OffsetCenterOfMassShapeSettings settings;
  settings.mInnerShapePtr = get_shape_or_fallback(child_shape);
  settings.mOffset = to_jolt(offset);
  return construct_shape(settings);
}

CollisionShape make_rotated_translated(const CollisionShape *child_shape,
                                       const math::Quaternion &rotation,
                                       const float3 &translation)
{
  JPH::RotatedTranslatedShapeSettings settings;
  settings.mInnerShapePtr = get_shape_or_fallback(child_shape);
  settings.mRotation = to_jolt(rotation);
  settings.mPosition = to_jolt(translation);
  return construct_shape(settings);
}

CollisionShape make_mutable_compound(Span<const CollisionShape *> child_shapes,
                                     Span<float4x4> child_transforms)
{
  JPH::MutableCompoundShapeSettings settings;
  settings.mSubShapes.reserve(child_shapes.size());
  for (const int i : child_shapes.index_range()) {
    if (child_shapes[i] == nullptr) {
      continue;
    }
    JPH::CompoundShapeSettings::SubShapeSettings child_settings;
    child_settings.mShapePtr = &child_shapes[i]->impl();
    child_settings.mPosition = to_jolt(child_transforms[i].location());
    child_settings.mRotation = to_jolt(math::to_quaternion(child_transforms[i]));
    settings.mSubShapes.push_back(std::move(child_settings));
  }
  return construct_shape(settings);
}

CollisionShape make_static_compound(Span<const CollisionShape *> child_shapes,
                                    Span<float4x4> child_transforms)
{
  JPH::StaticCompoundShapeSettings settings;
  settings.mSubShapes.reserve(child_shapes.size());
  for (const int i : child_shapes.index_range()) {
    if (child_shapes[i] == nullptr) {
      continue;
    }
    JPH::CompoundShapeSettings::SubShapeSettings child_settings;
    child_settings.mShapePtr = &child_shapes[i]->impl();
    child_settings.mPosition = to_jolt(child_transforms[i].location());
    child_settings.mRotation = to_jolt(math::to_quaternion(child_transforms[i]));
    settings.mSubShapes.push_back(std::move(child_settings));
  }
  return construct_shape(settings);
}

CollisionShape make_mesh(const Mesh &mesh)
{ /* This is assuming the mesh is triangulated. If a face has more than 3 vertices
   * it will create a random (but not invalid) mesh. */
  BLI_assert(mesh.corners_num == 3 * mesh.faces_num);

  const Span<float3> src_positions = mesh.vert_positions();
  JPH::VertexList vertices(src_positions.size());
  for (const int i : IndexRange(vertices.size())) {
    vertices[i] = JPH::Float3(src_positions[i].x, src_positions[i].y, src_positions[i].z);
  }

  const Span<int> src_indices = mesh.corner_verts();
  const OffsetIndices corners_by_face = mesh.face_offsets();
  JPH::IndexedTriangleList triangles(corners_by_face.size());
  for (const int i : IndexRange(triangles.size())) {
    const IndexRange corners = corners_by_face[i];
    BLI_assert(corners.size() >= 3);
    triangles[i] = {uint32_t(src_indices[corners[0]]),
                    uint32_t(src_indices[corners[1]]),
                    uint32_t(src_indices[corners[2]])};
  }

  JPH::MeshShapeSettings settings(vertices, triangles);
  return construct_shape(settings);
}

}  // namespace collision_shapes

blender::Any<> physics_shape_get_param(const JPH::Shape &shape, PhysicsShapeParam param)
{
  using ShapeParam = PhysicsShapeParam;

  switch (shape.GetSubType()) {
    case JPH::EShapeSubType::Sphere: {
      const auto &sphere_shape = static_cast<const JPH::SphereShape &>(shape);
      switch (param) {
        case ShapeParam::radius:
          return to_blender(sphere_shape.GetRadius());
        default:
          return {};
      }
    }
    case JPH::EShapeSubType::Box: {
      const auto &box_shape = static_cast<const JPH::BoxShape &>(shape);
      switch (param) {
        case ShapeParam::size:
          return to_blender(box_shape.GetHalfExtent()) * 2.0f;
        default:
          return {};
      }
    }
    case JPH::EShapeSubType::Triangle: {
      const auto &triangle_shape = static_cast<const JPH::TriangleShape &>(shape);

      JPH::TriangleShape::GetTrianglesContext context;
      triangle_shape.GetTrianglesStart(context,
                                       {},
                                       to_jolt(float3(0)),
                                       to_jolt(math::Quaternion::identity()),
                                       to_jolt(float3(1)));
      JPH::Float3 triangle[3];
      triangle_shape.GetTrianglesNext(context, 1, triangle);
      switch (param) {
        case ShapeParam::point0:
          return to_blender(triangle[0]);
        case ShapeParam::point1:
          return to_blender(triangle[1]);
        case ShapeParam::point2:
          return to_blender(triangle[2]);
        default:
          return {};
      }
    }
    case JPH::EShapeSubType::Capsule: {
      const auto &capsule_shape = static_cast<const JPH::CapsuleShape &>(shape);
      switch (param) {
        case ShapeParam::radius:
          return to_blender(capsule_shape.GetRadius());
        case ShapeParam::height:
          return to_blender(capsule_shape.GetHalfHeightOfCylinder()) * 2.0f;
        default:
          return {};
      }
    }
    case JPH::EShapeSubType::TaperedCapsule: {
      const auto &tapered_capsule_shape = static_cast<const JPH::TaperedCapsuleShape &>(shape);
      /* Reconstruct parameters from local bounds.
       * TODO It may be beneficial to store "uncooked"
       * JPH::ShapeSettings along with the "cooked" JPH::Shape.
       * For now this should work ok. */
      const Bounds<float3> local_bounds = to_blender(tapered_capsule_shape.GetLocalBounds());
      float top_radius, bottom_radius, half_height;
      if (-local_bounds.min.y < local_bounds.max.y) {
        top_radius = local_bounds.max.x;
        half_height = local_bounds.max.y - top_radius;
        bottom_radius = -local_bounds.min.y - half_height;
      }
      else {
        bottom_radius = local_bounds.max.x;
        half_height = -local_bounds.min.y - bottom_radius;
        top_radius = local_bounds.max.y - half_height;
      }
      switch (param) {
        case ShapeParam::radius:
          return bottom_radius;
        case ShapeParam::radius2:
          return top_radius;
        case ShapeParam::height:
          return 2.0f * half_height;
        default:
          return {};
      }
    }
    case JPH::EShapeSubType::Cylinder: {
      const auto &cylinder_shape = static_cast<const JPH::CylinderShape &>(shape);
      switch (param) {
        case ShapeParam::radius:
          return to_blender(cylinder_shape.GetRadius());
        case ShapeParam::height:
          return to_blender(cylinder_shape.GetHalfHeight()) * 2.0f;
        default:
          return {};
      }
    }
    case JPH::EShapeSubType::RotatedTranslated: {
      const auto &loc_rot_shape = static_cast<const JPH::RotatedTranslatedShape &>(shape);
      switch (param) {
        case ShapeParam::translation:
          return to_blender(loc_rot_shape.GetPosition());
        case ShapeParam::rotation:
          return to_blender(loc_rot_shape.GetRotation());
        default:
          return {};
      }
    }
    case JPH::EShapeSubType::Scaled: {
      const auto &scaled_shape = static_cast<const JPH::ScaledShape &>(shape);
      switch (param) {
        case ShapeParam::scale:
          return to_blender(scaled_shape.GetScale());
        default:
          return {};
      }
    }
    case JPH::EShapeSubType::OffsetCenterOfMass: {
      const auto &offset_com_shape = static_cast<const JPH::OffsetCenterOfMassShape &>(shape);
      switch (param) {
        case ShapeParam::translation:
          return to_blender(offset_com_shape.GetOffset());
        default:
          return {};
      }
    }

    default:
      return {};
  }
  BLI_assert_unreachable();
  return {};
}

StringRef physics_shape_param_name(PhysicsShapeParam param)
{
  using ShapeParam = PhysicsShapeParam;
  switch (param) {
    case ShapeParam::translation:
      return "translation";
    case ShapeParam::rotation:
      return "rotation";
    case ShapeParam::scale:
      return "scale";
    case ShapeParam::size:
      return "size";
    case ShapeParam::radius:
      return "radius";
    case ShapeParam::radius2:
      return "radius2";
    case ShapeParam::height:
      return "height";
    case ShapeParam::point0:
      return "point0";
    case ShapeParam::point1:
      return "point1";
    case ShapeParam::point2:
      return "point2";
  }
  BLI_assert_unreachable();
  return "";
}

StringRef physics_shape_param_label(const bke::CollisionShapeType shape_type,
                                    bke::PhysicsShapeParam param)
{
  using ShapeType = bke::CollisionShapeType;
  using ShapeParam = bke::PhysicsShapeParam;

  const StringRef default_label = bke::physics_shape_param_name(param);

  switch (shape_type) {
    case ShapeType::Sphere:
      switch (param) {
        case ShapeParam::radius:
          return "Radius";
        default:
          return default_label;
      }
    case ShapeType::Box:
      switch (param) {
        case ShapeParam::size:
          return "Size";
        default:
          return default_label;
      }
    case ShapeType::Triangle:
      switch (param) {
        case ShapeParam::point0:
          return "Point 0";
        case ShapeParam::point1:
          return "Point 1";
        case ShapeParam::point2:
          return "Point 2";
        default:
          return default_label;
      }
    case ShapeType::Capsule:
      switch (param) {
        case ShapeParam::radius:
          return "Radius";
        case ShapeParam::height:
          return "Height";
        default:
          return default_label;
      }
    case ShapeType::TaperedCapsule:
      switch (param) {
        case ShapeParam::radius:
          return "Bottom Radius";
        case ShapeParam::radius2:
          return "Top Radius";
        case ShapeParam::height:
          return "Height";
        default:
          return default_label;
      }
    case ShapeType::Cylinder:
      switch (param) {
        case ShapeParam::radius:
          return "Radius";
        case ShapeParam::height:
          return "Height";
        default:
          return default_label;
      }
    case ShapeType::RotatedTranslated:
      switch (param) {
        case ShapeParam::rotation:
          return "Rotation";
        case ShapeParam::translation:
          return "Translation";
        default:
          return default_label;
      }
    case ShapeType::Scaled:
      switch (param) {
        case ShapeParam::scale:
          return "Scale";
        default:
          return default_label;
      }
    case ShapeType::OffsetCenterOfMass:
      switch (param) {
        case ShapeParam::translation:
          return "Offset";
        default:
          return default_label;
      }

    case ShapeType::Empty:
    case ShapeType::ConvexHull:
    case ShapeType::StaticCompound:
    case ShapeType::MutableCompound:
    case ShapeType::Mesh:
    case ShapeType::HeightField:
    case ShapeType::SoftBody:
      return default_label;
  }
  BLI_assert_unreachable();
  return "";
}

bool physics_shape_param_valid(const CollisionShapeType shape_type, PhysicsShapeParam param)
{
  using ShapeType = CollisionShapeType;
  using ShapeParam = PhysicsShapeParam;

  switch (shape_type) {
    case ShapeType::Sphere:
      return ELEM(param, ShapeParam::radius);
    case ShapeType::Box:
      return ELEM(param, ShapeParam::size);
    case ShapeType::Triangle:
      return ELEM(param, ShapeParam::point0, ShapeParam::point1, ShapeParam::point2);
    case ShapeType::Capsule:
      return ELEM(param, ShapeParam::radius, ShapeParam::height);
    case ShapeType::TaperedCapsule:
      return ELEM(param, ShapeParam::radius, ShapeParam::radius2, ShapeParam::height);
    case ShapeType::Cylinder:
      return ELEM(param, ShapeParam::radius, ShapeParam::height);
    case ShapeType::RotatedTranslated:
      return ELEM(param, ShapeParam::translation, ShapeParam::rotation);
    case ShapeType::Scaled:
      return ELEM(param, ShapeParam::scale);
    case ShapeType::OffsetCenterOfMass:
      return ELEM(param, ShapeParam::translation);

    case ShapeType::Empty:
    case ShapeType::ConvexHull:
    case ShapeType::StaticCompound:
    case ShapeType::MutableCompound:
    case ShapeType::Mesh:
    case ShapeType::HeightField:
    case ShapeType::SoftBody:
      return false;
  }
  BLI_assert_unreachable();
  return false;
}

const CPPType &physics_shape_param_type(const PhysicsShapeParam param)
{
  using ShapeParam = PhysicsShapeParam;
  switch (param) {
    case ShapeParam::translation:
      return CPPType::get<float3>();
    case ShapeParam::rotation:
      return CPPType::get<math::Quaternion>();
    case ShapeParam::scale:
      return CPPType::get<float3>();
    case ShapeParam::size:
      return CPPType::get<float3>();
    case ShapeParam::radius:
      return CPPType::get<float>();
    case ShapeParam::radius2:
      return CPPType::get<float>();
    case ShapeParam::height:
      return CPPType::get<float>();
    case ShapeParam::point0:
      return CPPType::get<float3>();
    case ShapeParam::point1:
      return CPPType::get<float3>();
    case ShapeParam::point2:
      return CPPType::get<float3>();
  }
  BLI_assert_unreachable();
  return CPPType::get<int>();
}

bool physics_shape_has_geometry(const CollisionShapeType shape_type)
{
  using ShapeType = CollisionShapeType;

  return ELEM(shape_type,
              ShapeType::ConvexHull,
              ShapeType::Mesh,
              ShapeType::Scaled,
              ShapeType::OffsetCenterOfMass,
              ShapeType::RotatedTranslated,
              ShapeType::StaticCompound,
              ShapeType::MutableCompound);
}

#endif

}  // namespace blender::bke
