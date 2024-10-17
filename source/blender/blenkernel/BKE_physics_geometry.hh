/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

#include "BLI_generic_array.hh"
#include "BLI_index_mask_fwd.hh"
#include "BLI_index_range.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_utility_mixins.hh"
#include "BLI_virtual_array_fwd.hh"

#include "BKE_attribute.hh"
#include "BKE_geometry_fields.hh"
#include "BKE_instances.hh"

#include "DNA_customdata_types.h"

#include <functional>

namespace blender::bke {
class AttributeAccessor;
class MutableAttributeAccessor;
enum class CollisionShapeType;
}  // namespace blender::bke

namespace blender::bke {

class PhysicsWorldData;
struct CustomDataAccessInfo;
struct PhysicsWorldDataAccessInfo;

enum class PhysicsBodyAttribute {
  collision_shape,
  motion_type,
  mass,
  inertia,
  position,
  rotation,
  velocity,
  angular_velocity,
  is_active,
  allow_sleep,
  friction,
  restitution,
  linear_damping,
  angular_damping,
  total_force,
  total_torque,
};

enum class PhysicsConstraintAttribute {
  type,
  body1,
  body2,
  enabled,
  frame1,
  frame2,
  limit_min_axis,
  limit_max_axis,
  limit_min_angle,
  limit_max_angle,
  spring_stiffness_axis,
  spring_stiffness_angle,
  spring_damping_axis,
  spring_damping_angle,
  max_friction_axis,
  max_friction_angle,
  motor_spring_stiffness_axis,
  motor_spring_stiffness_angle,
  motor_spring_damping_axis,
  motor_spring_damping_angle,
  min_motor_force_axis,
  min_motor_force_angle,
  max_motor_force_axis,
  max_motor_force_angle,
};

enum class PhysicsMotionType {
  Dynamic,
  Static,
  Kinematic,
};

enum class PhysicsConstraintType {
  Fixed,
  Distance,
  Point,
  Hinge,
  Cone,
  Slider,
  SwingTwist,
  SixDOF,
  Path,
  Gear,
  RackAndPinion,
  Pulley,
  Vehicle,
};

enum class PhysicsBodyActivationState {
  AlwaysActive = 0,
  Active,
  WantsSleep,
  Sleeping,
  AlwaysSleeping,
};

/**
 * A physics world state is a snapshot of the physics world.
 * The physics state acts as a bridge between geometry nodes and the physics engine. Physics
 * world data cannot be copied easily, there can only be one owner of the physics data. One
 * "active" physics state owns the world data.
 *
 * At the same time a physics state is also implicitly shared data. This way it represents physics
 * data in the geometry nodes system of components and attributes. In order to make a physics state
 * mutable it gets copied. Importantly the physics world data is moved to the new state, making the
 * new state the active one.
 *
 * In order to adhere to the rules of copy-on-write the physics state must be able to move the
 * world data without requiring a lock. To make this possible the state always stores a copy of the
 * physics state for use by attribute readers. This cache can safely be accessed while the world
 * data pointer ownership changes.
 */
class PhysicsWorldState : public ImplicitSharingMixin {
  using BodyAttribute = PhysicsBodyAttribute;
  using ConstraintAttribute = PhysicsConstraintAttribute;
  using ConstraintType = PhysicsConstraintType;
  using BodyActivationState = PhysicsBodyActivationState;

  using OverlapFilterFn = std::function<bool(const int a, const int b)>;

  using CacheFlag = std::atomic<bool>;

  /* Cache for readers storing copies of physics data in custom data. */
  mutable CacheFlag read_cache_valid_ = false;
  /* Valid when body collision shape pointers match pointers from the
   * shapes list, as stored in the body shapes index attribute. */
  mutable CacheFlag body_collision_shapes_valid_ = false;
  /* Valid when body mass properties match the write cache. */
  mutable CacheFlag body_mass_valid_ = false;
  /* Valid when internal constraints have been updated to specified types and bodies. */
  mutable CacheFlag constraints_valid_ = false;

  int body_num_;
  int constraint_num_;
  Map<BodyAttribute, GArray<>> body_data_;
  Map<ConstraintAttribute, GArray<>> constraint_data_;
  CustomData body_custom_data_;
  CustomData constraint_custom_data_;

  /**
   * Contains the data that is used by the individual instances.
   * Actual instances store an index ("handle") into this vector.
   */
  Vector<InstanceReference> shapes_;
  /**
   * Caches how often each shape is used.
   */
  mutable SharedCache<Array<int>> shape_user_counts_;

  mutable PhysicsWorldData *world_data_ = nullptr;
  /* Protects shared read/write access to world data. */
  mutable std::mutex world_data_mutex_;

 public:
  PhysicsWorldState();
  PhysicsWorldState(int body_num, int constraint_num);
  PhysicsWorldState(const PhysicsWorldState &other);
  ~PhysicsWorldState();

  PhysicsWorldState &operator=(const PhysicsWorldState &other);

  void delete_self() override;

  int bodies_num() const;
  int constraints_num() const;
  int shapes_num() const;

  IndexRange bodies_range() const;
  IndexRange constraints_range() const;
  IndexRange shapes_range() const;

  bool has_world_data() const;

  void tag_read_cache_changed();
  void tag_body_topology_changed();
  void tag_body_collision_shape_changed();
  void tag_body_mass_changed();
  void tag_constraints_changed();
  // void tag_constraint_disable_collision_changed();
  void tag_shapes_changed();

  bool has_builtin_attribute_cache(BodyAttribute attribute) const;
  bool has_builtin_attribute_cache(ConstraintAttribute attribute) const;

  /* Make sure all world data has been copied to the custom data read cache. */
  void ensure_read_cache() const;
  /* Make sure attributes with write caches have been transferred to world data. */
  void ensure_bodies();
  void ensure_constraints();

  void create_world();
  void destroy_world();
  void move_or_copy_selection(const PhysicsWorldState &src,
                              const IndexMask &src_body_mask,
                              const IndexMask &src_constraint_mask,
                              const AttributeFilter &attribute_filter);
  bool try_move_data(const PhysicsWorldState &src,
                     int body_num,
                     int constraint_num,
                     const IndexMask &src_body_mask,
                     const IndexMask &src_constraint_mask,
                     int dst_body_offset,
                     int dst_constraint_offset);

  /**
   * Returns a handle for the given shape reference.
   * If the reference exists already, the handle of the existing reference is returned.
   * Otherwise a new handle is added.
   */
  int add_shape(const InstanceReference &reference);
  /**
   * Same as above, but does not deduplicate with existing references.
   */
  int add_new_shape(const InstanceReference &reference);
  std::optional<int> find_shape_handle(const InstanceReference &query);

  Span<InstanceReference> shapes() const;
  void remove_unused_shapes();
  /**
   * Get cached user counts for every shape.
   */
  Span<int> shape_user_counts() const;

  /**
   * If references have a collection or object type, convert them into geometry instances
   * recursively. After that, the geometry sets can be edited. There may still be instances of
   * other types of they can't be converted to geometry sets.
   */
  void ensure_shape_geometry_instances();
  /**
   * With write access to the physics component, the data in the instanced geometry sets can be
   * changed. This is a function on the component rather than each reference to ensure `const`
   * correctness for that reason.
   */
  GeometrySet &geometry_set_from_shape(int shape_index);

  void set_overlap_filter(OverlapFilterFn fn);
  void clear_overlap_filter();

  float3 gravity() const;
  void set_gravity(const float3 &gravity);
  void set_solver_iterations(int num_solver_iterations);
  void set_split_impulse(bool split_impulse);

  void step_simulation(float delta_time, int collision_steps = 1);

  void apply_force(const IndexMask &selection,
                   const VArray<float3> &forces,
                   const VArray<float3> &relative_positions = {});
  void apply_torque(const IndexMask &selection, const VArray<float3> &torques);

  void apply_impulse(const IndexMask &selection,
                     const VArray<float3> &impulses,
                     const VArray<float3> &relative_positions = {});
  void apply_angular_impulse(const IndexMask &selection, const VArray<float3> &angular_impulses);
  void clear_forces(const IndexMask &selection);

  /* Validate internal relations, for debugging and testing purposes.
   * Should only be used in tests or debug mode. */
  bool validate_world_data();

  bke::AttributeAccessor attributes() const;
  bke::MutableAttributeAccessor attributes_for_write();

  static const PhysicsWorldDataAccessInfo &world_data_access_info();
  static const CustomDataAccessInfo &body_custom_data_access_info();
  static const CustomDataAccessInfo &constraint_custom_data_access_info();

  /* Debug utility functions, should be removed eventually. */
#ifndef NDEBUG
  void debug_print_status(StringRef name) const;
  void debug_print_enter(StringRef name) const;
  void debug_print_exit(StringRef name) const;
#else
  void debug_print_status(StringRef /*name*/) const {}
  void debug_print_enter(StringRef /*name*/) const {}
  void debug_print_exit(StringRef /*name*/) const {}
#endif

 private:
  void ensure_read_cache_no_lock() const;
  void ensure_bodies_no_lock();
  void ensure_body_collision_shapes_no_lock();
  void ensure_body_mass_no_lock();
  void ensure_constraints_no_lock();
  // void ensure_constraint_disable_collision_no_lock();
  void ensure_attribute_cache(BodyAttribute attribute);
  void ensure_attribute_cache(ConstraintAttribute attribute);
  void remove_attribute_caches();

  bke::AttributeAccessor state_attributes() const;
  bke::MutableAttributeAccessor state_attributes_for_write();
  bke::AttributeAccessor world_data_attributes() const;
  bke::MutableAttributeAccessor world_data_attributes_for_write();

  friend class PhysicsStateBodyAttributeProvider;
  friend class PhysicsStateConstraintAttributeProvider;
};

class PhysicsGeometry {
 public:
  using OverlapFilterFn = std::function<bool(const int a, const int b)>;

  using BodyAttribute = PhysicsBodyAttribute;
  using ConstraintAttribute = PhysicsConstraintAttribute;
  using ConstraintType = PhysicsConstraintType;
  using BodyActivationState = PhysicsBodyActivationState;

 private:
  /* Implementation of the physics world and rigid bodies.
   * This is an implicit shared pointer, multiple users can read the physics data. Requesting write
   * access will move the data to a new component. This is somewhat different from other components
   * which do a full copy of the data, but necessary for efficiently handling physics state. Moving
   * the physics state will create a read-only cache, so older state still be accessed. */
  const PhysicsWorldState *world_state_ = nullptr;

 public:
  PhysicsGeometry();
  explicit PhysicsGeometry(int bodies_num, int constraints_num);
  PhysicsGeometry(const PhysicsGeometry &other);
  ~PhysicsGeometry();

  const PhysicsWorldState &state() const;
  PhysicsWorldState &state_for_write();

  int bodies_num() const;
  int constraints_num() const;
  int shapes_num() const;

  IndexRange bodies_range() const;
  IndexRange constraints_range() const;
  IndexRange shapes_range() const;

  VArray<int> body_shapes() const;
  AttributeWriter<int> body_shapes_for_write();

  VArray<int> body_motion_types() const;
  AttributeWriter<int> body_motion_types_for_write();

  VArray<float> body_masses() const;
  AttributeWriter<float> body_masses_for_write();

  VArray<float4x4> body_inertias() const;
  AttributeWriter<float4x4> body_inertias_for_write();

  VArray<float3> body_positions() const;
  AttributeWriter<float3> body_positions_for_write();

  VArray<math::Quaternion> body_rotations() const;
  AttributeWriter<math::Quaternion> body_rotations_for_write();

  VArray<float3> body_velocities() const;
  AttributeWriter<float3> body_velocities_for_write();

  VArray<float3> body_angular_velocities() const;
  AttributeWriter<float3> body_angular_velocities_for_write();

  VArray<float3> body_total_force() const;
  VArray<float3> body_total_torque() const;

  VArray<int> constraint_types() const;
  AttributeWriter<int> constraint_types_for_write();
  VArray<int> constraint_body1() const;
  AttributeWriter<int> constraint_body1_for_write();
  VArray<int> constraint_body2() const;
  AttributeWriter<int> constraint_body2_for_write();

  VArray<bool> constraint_enabled() const;
  AttributeWriter<bool> constraint_enabled_for_write();

  VArray<float4x4> constraint_frame1() const;
  AttributeWriter<float4x4> constraint_frame1_for_write();

  VArray<float4x4> constraint_frame2() const;
  AttributeWriter<float4x4> constraint_frame2_for_write();

  static Span<BodyAttribute> all_body_attributes();
  static Span<ConstraintAttribute> all_constraint_attributes();

  static StringRef attribute_name(BodyAttribute attribute);
  static StringRef attribute_name(ConstraintAttribute attribute);

  static Span<std::string> all_body_attribute_names();
  static Span<std::string> all_constraint_attribute_names();

  static const CPPType &attribute_type(PhysicsBodyAttribute attribute);
  static const CPPType &attribute_type(PhysicsConstraintAttribute attribute);

  static const void *attribute_default_value(PhysicsBodyAttribute attribute);
  static const void *attribute_default_value(PhysicsConstraintAttribute attribute);

  bke::AttributeAccessor attributes() const;
  bke::MutableAttributeAccessor attributes_for_write();

  /* XXX Attributes functions on components can nominally return a nullopt, but some parts of
   * the code don't check for that return value case
   * (realize_instances.cc::gather_attributes_for_propagation).
   * These dummy functions return a placeholder accessor that can be used as a fallback to
   * avoid this bug */
  static AttributeAccessor dummy_attributes();
  static MutableAttributeAccessor dummy_attributes_for_write();

  /* Validate internal world data.
   * Should only be used in tests. */
  bool validate_world_data();

 private:
  template<typename T> VArray<T> lookup_attribute(const PhysicsBodyAttribute attribute) const;
};

/* -------------------------------------------------------------------- */
/** \name Attribute Info for Physics Geometry
 * \{ */

namespace physics_attributes {

Span<PhysicsBodyAttribute> all_body_attributes();
Span<PhysicsConstraintAttribute> all_constraint_attributes();

StringRef physics_attribute_name(PhysicsBodyAttribute attribute);
StringRef physics_attribute_name(PhysicsConstraintAttribute attribute);

std::optional<PhysicsBodyAttribute> find_body_attribute(StringRef name);
std::optional<PhysicsConstraintAttribute> find_constraint_attribute(StringRef name);

Span<std::string> all_body_attribute_names();
Span<std::string> all_constraint_attribute_names();

const CPPType &physics_attribute_type(PhysicsBodyAttribute attribute);
const CPPType &physics_attribute_type(PhysicsConstraintAttribute attribute);

const void *physics_attribute_default_value(PhysicsBodyAttribute attribute);
const void *physics_attribute_default_value(PhysicsConstraintAttribute attribute);
template<typename T> const T &physics_attribute_default_value(PhysicsBodyAttribute attribute)
{
  BLI_assert(physics_attribute_type(attribute).is<T>());
  return *static_cast<const T *>(physics_attribute_default_value(attribute));
}
template<typename T> const T &physics_attribute_default_value(PhysicsConstraintAttribute attribute)
{
  BLI_assert(physics_attribute_type(attribute).is<T>());
  return *static_cast<const T *>(physics_attribute_default_value(attribute));
}

/* Writes to cache first, then updates engine data afterward.
 * This is used for attributes which do not have a direct property in engine data.
 * Example: Mass is stored as inverse mass, the "mass" attribute is stored in a cache to avoid
 * loss of accuracy through repeated conversion. */
bool physics_attribute_use_write_cache(const PhysicsBodyAttribute attribute);
bool physics_attribute_use_write_cache(const PhysicsConstraintAttribute attribute);

template<typename T>
VArray<T> physics_attribute_lookup_or_default(const AttributeAccessor attributes,
                                              PhysicsBodyAttribute physics_attribute)
{
  return *attributes.lookup_or_default<T>(physics_attribute_name(physics_attribute),
                                          AttrDomain::Point,
                                          physics_attribute_default_value<T>(physics_attribute));
}

template<typename T>
VArray<T> physics_attribute_lookup_or_default(const AttributeAccessor attributes,
                                              PhysicsConstraintAttribute physics_attribute)
{
  return *attributes.lookup_or_default<T>(physics_attribute_name(physics_attribute),
                                          AttrDomain::Edge,
                                          physics_attribute_default_value<T>(physics_attribute));
}

template<typename T>
static SpanAttributeWriter<T> physics_attribute_lookup_for_write_only_span(
    MutableAttributeAccessor attributes, PhysicsBodyAttribute physics_attribute)
{
  return attributes.lookup_or_add_for_write_only_span<T>(physics_attribute_name(physics_attribute),
                                                         AttrDomain::Point);
}

template<typename T>
static SpanAttributeWriter<T> physics_attribute_lookup_for_write_only_span(
    MutableAttributeAccessor attributes, PhysicsConstraintAttribute physics_attribute)
{
  return attributes.lookup_or_add_for_write_only_span<T>(physics_attribute_name(physics_attribute),
                                                         AttrDomain::Edge);
}

}  // namespace physics_attributes

/** \} */

}  // namespace blender::bke
