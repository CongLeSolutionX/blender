/* SPDX-FileCopyrightText: 2004-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#include <functional>

#include "BKE_attribute.hh"

#include "BLI_assert.h"
#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_utildefines.h"
#include "BLI_virtual_array.hh"

#include "attribute_access_intern.hh"
#include "physics_geometry_attributes.hh"
#include "physics_geometry_intern.hh"

namespace blender::bke {

/* -------------------------------------------------------------------- */
/** \name Physics Geometry
 * \{ */

template<PhysicsStateAttributeAccessMode access_mode>
static ComponentAttributeProviders create_attribute_providers_for_physics()
{
  using namespace physics_attributes;
  using BodyAttribute = PhysicsBodyAttribute;
  using ConstraintAttribute = PhysicsConstraintAttribute;

  static CustomDataAccessInfo body_custom_data_access =
      PhysicsWorldState::body_custom_data_access_info();
  static CustomDataAccessInfo constraint_custom_data_access =
      PhysicsWorldState::constraint_custom_data_access_info();

  const Span<BodyAttribute> body_attributes = all_body_attributes();
  const Span<ConstraintAttribute> constraint_attributes = all_constraint_attributes();
  static Vector<PhysicsStateBodyAttributeProvider> body_attribute_providers;
  static Vector<PhysicsStateConstraintAttributeProvider> constraint_attribute_providers;
  body_attribute_providers.reserve(body_attributes.size());
  constraint_attribute_providers.reserve(constraint_attributes.size());
  for (const BodyAttribute attribute : body_attributes) {
    PhysicsStateBodyAttributeProvider provider(access_mode, attribute);
    body_attribute_providers.append(std::move(provider));
  }
  for (const ConstraintAttribute attribute : constraint_attributes) {
    PhysicsStateConstraintAttributeProvider provider(access_mode, attribute);
    constraint_attribute_providers.append(std::move(provider));
  }

  static Array<const BuiltinAttributeProvider *> builtin_providers(
      body_attribute_providers.size() + constraint_attribute_providers.size());
  const IndexRange body_attribute_provider_range = body_attribute_providers.index_range();
  const IndexRange constraint_attribute_provider_range = body_attribute_provider_range.after(
      constraint_attribute_providers.size());
  for (const int i : body_attribute_providers.index_range()) {
    builtin_providers[body_attribute_provider_range[i]] = &body_attribute_providers[i];
  }
  for (const int i : constraint_attribute_providers.index_range()) {
    builtin_providers[constraint_attribute_provider_range[i]] = &constraint_attribute_providers[i];
  }

  static CustomDataAttributeProvider body_custom_data(AttrDomain::Point, body_custom_data_access);
  static CustomDataAttributeProvider constraint_custom_data(AttrDomain::Edge,
                                                            constraint_custom_data_access);

  return ComponentAttributeProviders(builtin_providers,
                                     {&body_custom_data, &constraint_custom_data});
}

static GVArray adapt_physics_attribute_domain(const PhysicsWorldState & /*state*/,
                                              const GVArray &varray,
                                              const AttrDomain from,
                                              const AttrDomain to)
{
  if (from == to) {
    return varray;
  }
  return {};
}

static AttributeAccessorFunctions get_physics_accessor_functions(const bool enable_cache,
                                                                 const bool enable_world_data)
{
  static const ComponentAttributeProviders providers =
      create_attribute_providers_for_physics<PhysicsStateAttributeAccessMode::CachedRead>();
  static const ComponentAttributeProviders cache_providers =
      create_attribute_providers_for_physics<PhysicsStateAttributeAccessMode::CachedReadWrite>();
  static const ComponentAttributeProviders world_data_providers =
      create_attribute_providers_for_physics<PhysicsStateAttributeAccessMode::DirectReadWrite>();
  BLI_assert(enable_cache || enable_world_data);
  AttributeAccessorFunctions fn =
      (enable_world_data ?
           (enable_cache ?
                attribute_accessor_functions::accessor_functions_for_providers<providers>() :
                attribute_accessor_functions::accessor_functions_for_providers<
                    world_data_providers>()) :
           attribute_accessor_functions::accessor_functions_for_providers<cache_providers>());
  fn.domain_size = [](const void *owner, const AttrDomain domain) {
    if (owner == nullptr) {
      return 0;
    }

    const PhysicsWorldState &state = *static_cast<const PhysicsWorldState *>(owner);
    switch (domain) {
      case AttrDomain::Point:
        return state.bodies_num();
      case AttrDomain::Edge:
        return state.constraints_num();
      case blender::bke::AttrDomain::Instance:
        return int(state.shapes_num());
      default:
        return 0;
    }
  };
  fn.domain_supported = [](const void * /*owner*/, const AttrDomain domain) {
    return ELEM(domain, AttrDomain::Point, AttrDomain::Edge, AttrDomain::Instance);
  };
  fn.adapt_domain = [](const void *owner,
                       const GVArray &varray,
                       const AttrDomain from_domain,
                       const AttrDomain to_domain) -> GVArray {
    if (owner == nullptr) {
      return {};
    }
    const PhysicsWorldState &state = *static_cast<const PhysicsWorldState *>(owner);
    return adapt_physics_attribute_domain(state, varray, from_domain, to_domain);
  };
  return fn;
}

const AttributeAccessorFunctions &get_physics_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_physics_accessor_functions(true, true);
  return fn;
}

const AttributeAccessorFunctions &get_physics_state_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_physics_accessor_functions(true, false);
  return fn;
}

const AttributeAccessorFunctions &get_physics_world_data_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_physics_accessor_functions(false, true);
  return fn;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Attribute Info for Physics Geometry
 * \{ */

namespace physics_attributes {

Span<PhysicsBodyAttribute> all_body_attributes()
{
  using BodyAttribute = PhysicsBodyAttribute;
  static Array<BodyAttribute> attributes = {
      BodyAttribute::collision_shape,
      BodyAttribute::motion_type,
      BodyAttribute::mass,
      BodyAttribute::inertia,
      BodyAttribute::position,
      BodyAttribute::rotation,
      BodyAttribute::velocity,
      BodyAttribute::angular_velocity,
      BodyAttribute::is_active,
      BodyAttribute::allow_sleep,
      BodyAttribute::friction,
      BodyAttribute::restitution,
      BodyAttribute::linear_damping,
      BodyAttribute::angular_damping,
      BodyAttribute::total_force,
      BodyAttribute::total_torque,
  };
  return attributes;
}

Span<PhysicsConstraintAttribute> all_constraint_attributes()
{
  using ConstraintAttribute = PhysicsConstraintAttribute;
  static Array<ConstraintAttribute> attributes = {
      ConstraintAttribute::type,
      ConstraintAttribute::body1,
      ConstraintAttribute::body2,
      ConstraintAttribute::enabled,
      ConstraintAttribute::frame1,
      ConstraintAttribute::frame2,
      ConstraintAttribute::limit_min_axis,
      ConstraintAttribute::limit_max_axis,
      ConstraintAttribute::limit_min_angle,
      ConstraintAttribute::limit_max_angle,
      ConstraintAttribute::spring_stiffness_axis,
      ConstraintAttribute::spring_stiffness_angle,
      ConstraintAttribute::spring_damping_axis,
      ConstraintAttribute::spring_damping_angle,
      ConstraintAttribute::max_friction_axis,
      ConstraintAttribute::max_friction_angle,
      ConstraintAttribute::motor_spring_stiffness_axis,
      ConstraintAttribute::motor_spring_stiffness_angle,
      ConstraintAttribute::motor_spring_damping_axis,
      ConstraintAttribute::motor_spring_damping_angle,
      ConstraintAttribute::min_motor_force_axis,
      ConstraintAttribute::min_motor_force_angle,
      ConstraintAttribute::max_motor_force_axis,
      ConstraintAttribute::max_motor_force_angle,
  };
  return attributes;
}

StringRef physics_attribute_name(PhysicsBodyAttribute attribute)
{
  using BodyAttribute = PhysicsBodyAttribute;

  switch (attribute) {
    case BodyAttribute::collision_shape:
      return "collision_shape";
    case BodyAttribute::motion_type:
      return "motion_type";
    case BodyAttribute::mass:
      return "mass";
    case BodyAttribute::inertia:
      return "inertia";
    case BodyAttribute::position:
      return "position";
    case BodyAttribute::rotation:
      return "rotation";
    case BodyAttribute::velocity:
      return "velocity";
    case BodyAttribute::angular_velocity:
      return "angular_velocity";
    case BodyAttribute::is_active:
      return "is_active";
    case BodyAttribute::allow_sleep:
      return "allow_sleep";
    case BodyAttribute::friction:
      return "friction";
    case BodyAttribute::restitution:
      return "restitution";
    case BodyAttribute::linear_damping:
      return "linear_damping";
    case BodyAttribute::angular_damping:
      return "angular_damping";
    case BodyAttribute::total_force:
      return "total_force";
    case BodyAttribute::total_torque:
      return "total_torque";
  }
  BLI_assert_unreachable();
  return "";
}

StringRef physics_attribute_name(PhysicsConstraintAttribute attribute)
{
  using ConstraintAttribute = PhysicsConstraintAttribute;

  switch (attribute) {
    case ConstraintAttribute::type:
      return "type";
    case ConstraintAttribute::body1:
      return "body1";
    case ConstraintAttribute::body2:
      return "body2";
    case ConstraintAttribute::enabled:
      return "enabled";
    case ConstraintAttribute::frame1:
      return "frame1";
    case ConstraintAttribute::frame2:
      return "frame2";
    case ConstraintAttribute::limit_min_axis:
      return "limit_min_axis";
    case ConstraintAttribute::limit_max_axis:
      return "limit_max_axis";
    case ConstraintAttribute::limit_min_angle:
      return "limit_min_angle";
    case ConstraintAttribute::limit_max_angle:
      return "limit_max_angle";
    case ConstraintAttribute::spring_stiffness_axis:
      return "spring_stiffness_axis";
    case ConstraintAttribute::spring_stiffness_angle:
      return "spring_stiffness_angle";
    case ConstraintAttribute::spring_damping_axis:
      return "spring_damping_axis";
    case ConstraintAttribute::spring_damping_angle:
      return "spring_damping_angle";
    case ConstraintAttribute::max_friction_axis:
      return "max_friction_axis";
    case ConstraintAttribute::max_friction_angle:
      return "max_friction_angle";
    case ConstraintAttribute::motor_spring_stiffness_axis:
      return "motor_spring_stiffness_axis";
    case ConstraintAttribute::motor_spring_stiffness_angle:
      return "motor_spring_stiffness_angle";
    case ConstraintAttribute::motor_spring_damping_axis:
      return "motor_spring_damping_axis";
    case ConstraintAttribute::motor_spring_damping_angle:
      return "motor_spring_damping_angle";
    case ConstraintAttribute::min_motor_force_axis:
      return "min_motor_force_axis";
    case ConstraintAttribute::min_motor_force_angle:
      return "min_motor_force_angle";
    case ConstraintAttribute::max_motor_force_axis:
      return "max_motor_force_axis";
    case ConstraintAttribute::max_motor_force_angle:
      return "max_motor_force_angle";
  }
  BLI_assert_unreachable();
  return "";
}

std::optional<PhysicsBodyAttribute> find_body_attribute(StringRef name)
{
  static const Map<StringRef, PhysicsBodyAttribute> name_map = []() {
    Map<StringRef, PhysicsBodyAttribute> name_map;
    for (const PhysicsBodyAttribute attribute : all_body_attributes()) {
      name_map.add_new(physics_attribute_name(attribute), attribute);
    }
    return name_map;
  }();

  const PhysicsBodyAttribute *attribute = name_map.lookup_ptr(name);
  return attribute ? std::make_optional(*attribute) : std::nullopt;
}

std::optional<PhysicsConstraintAttribute> find_constraint_attribute(StringRef name)
{
  static const Map<StringRef, PhysicsConstraintAttribute> name_map = []() {
    Map<StringRef, PhysicsConstraintAttribute> name_map;
    for (const PhysicsConstraintAttribute attribute : all_constraint_attributes()) {
      name_map.add_new(physics_attribute_name(attribute), attribute);
    }
    return name_map;
  }();

  const PhysicsConstraintAttribute *attribute = name_map.lookup_ptr(name);
  return attribute ? std::make_optional(*attribute) : std::nullopt;
}

Span<std::string> all_body_attribute_names()
{
  static Array<std::string> all_names = []() {
    const Span<PhysicsBodyAttribute> all_attributes = all_body_attributes();
    Array<std::string> all_names(all_attributes.size());
    for (const int i : all_names.index_range()) {
      all_names[i] = physics_attribute_name(all_attributes[i]);
    }
    return all_names;
  }();
  return all_names;
}

Span<std::string> all_constraint_attribute_names()
{
  static Array<std::string> all_names = []() {
    const Span<PhysicsConstraintAttribute> all_attributes = all_constraint_attributes();
    Array<std::string> all_names(all_attributes.size());
    for (const int i : all_names.index_range()) {
      all_names[i] = physics_attribute_name(all_attributes[i]);
    }
    return all_names;
  }();
  return all_names;
}

const CPPType &physics_attribute_type(PhysicsBodyAttribute attribute)
{
  using BodyAttribute = PhysicsBodyAttribute;

  switch (attribute) {
    case BodyAttribute::collision_shape:
      return CPPType::get<int>();
    case BodyAttribute::motion_type:
      return CPPType::get<int>();
    case BodyAttribute::mass:
      return CPPType::get<float>();
    case BodyAttribute::inertia:
      return CPPType::get<float4x4>();
    case BodyAttribute::position:
      return CPPType::get<float3>();
    case BodyAttribute::rotation:
      return CPPType::get<math::Quaternion>();
    case BodyAttribute::velocity:
      return CPPType::get<float3>();
    case BodyAttribute::angular_velocity:
      return CPPType::get<float3>();
    case BodyAttribute::is_active:
      return CPPType::get<bool>();
    case BodyAttribute::allow_sleep:
      return CPPType::get<bool>();
    case BodyAttribute::friction:
      return CPPType::get<float>();
    case BodyAttribute::restitution:
      return CPPType::get<float>();
    case BodyAttribute::linear_damping:
      return CPPType::get<float>();
    case BodyAttribute::angular_damping:
      return CPPType::get<float>();
    case BodyAttribute::total_force:
      return CPPType::get<float3>();
    case BodyAttribute::total_torque:
      return CPPType::get<float3>();
  }
  BLI_assert_unreachable();
  return CPPType::get<int>();
}

const CPPType &physics_attribute_type(PhysicsConstraintAttribute attribute)
{
  using ConstraintAttribute = PhysicsConstraintAttribute;

  switch (attribute) {
    case ConstraintAttribute::type:
      return CPPType::get<int>();
    case ConstraintAttribute::body1:
    case ConstraintAttribute::body2:
      return CPPType::get<int>();
    case ConstraintAttribute::enabled:
      return CPPType::get<bool>();
    case ConstraintAttribute::frame1:
    case ConstraintAttribute::frame2:
      return CPPType::get<float4x4>();
    case ConstraintAttribute::limit_min_axis:
    case ConstraintAttribute::limit_max_axis:
    case ConstraintAttribute::limit_min_angle:
    case ConstraintAttribute::limit_max_angle:
    case ConstraintAttribute::spring_stiffness_axis:
    case ConstraintAttribute::spring_stiffness_angle:
    case ConstraintAttribute::spring_damping_axis:
    case ConstraintAttribute::spring_damping_angle:
    case ConstraintAttribute::max_friction_axis:
    case ConstraintAttribute::max_friction_angle:
    case ConstraintAttribute::motor_spring_stiffness_axis:
    case ConstraintAttribute::motor_spring_stiffness_angle:
    case ConstraintAttribute::motor_spring_damping_axis:
    case ConstraintAttribute::motor_spring_damping_angle:
    case ConstraintAttribute::min_motor_force_axis:
    case ConstraintAttribute::min_motor_force_angle:
    case ConstraintAttribute::max_motor_force_axis:
    case ConstraintAttribute::max_motor_force_angle:
      return CPPType::get<float3>();
  }
  BLI_assert_unreachable();
  return CPPType::get<int>();
}

const void *physics_attribute_default_value(PhysicsBodyAttribute attribute)
{
  using BodyAttribute = PhysicsBodyAttribute;

  const CPPType &type = physics_attribute_type(attribute);
  switch (attribute) {
    case BodyAttribute::collision_shape: {
      static const int default_value = -1;
      return &default_value;
    }
    case BodyAttribute::mass: {
      static const float default_value = 1.0f;
      return &default_value;
    }
    case BodyAttribute::inertia: {
      static const float4x4 default_value = float4x4::identity();
      return &default_value;
    }
    case BodyAttribute::is_active: {
      static const bool default_value = false;
      return &default_value;
    }
    case BodyAttribute::allow_sleep: {
      static const bool default_value = true;
      return &default_value;
    }
    case BodyAttribute::friction: {
      static const float default_value = 0.5f;
      return &default_value;
    }

    case BodyAttribute::motion_type:
    case BodyAttribute::position:
    case BodyAttribute::rotation:
    case BodyAttribute::velocity:
    case BodyAttribute::angular_velocity:
    case BodyAttribute::restitution:
    case BodyAttribute::linear_damping:
    case BodyAttribute::angular_damping:
    case BodyAttribute::total_force:
    case BodyAttribute::total_torque:
      return type.default_value();
  }
  BLI_assert_unreachable();
  return nullptr;
}

const void *physics_attribute_default_value(PhysicsConstraintAttribute attribute)
{
  using ConstraintAttribute = PhysicsConstraintAttribute;

  const CPPType &type = physics_attribute_type(attribute);
  switch (attribute) {
    case ConstraintAttribute::type: {
      static const int default_value = int(PhysicsConstraintType::Fixed);
      return &default_value;
    }
    case ConstraintAttribute::body1:
    case ConstraintAttribute::body2: {
      static const int default_value = -1;
      return &default_value;
    }
    case ConstraintAttribute::frame1:
    case ConstraintAttribute::frame2: {
      static const float4x4 default_value = float4x4::identity();
      return &default_value;
    }
    case ConstraintAttribute::enabled: {
      static const bool default_value = true;
      return &default_value;
    }
    case ConstraintAttribute::limit_min_axis: {
      static const float3 default_value = float3(-FLT_MAX);
      return &default_value;
    }
    case ConstraintAttribute::limit_min_angle: {
      static const float3 default_value = float3(-M_PI);
      return &default_value;
    }
    case ConstraintAttribute::limit_max_axis: {
      static const float3 default_value = float3(FLT_MAX);
      return &default_value;
    }
    case ConstraintAttribute::limit_max_angle: {
      static const float3 default_value = float3(M_PI);
      return &default_value;
    }
    case ConstraintAttribute::min_motor_force_axis:
    case ConstraintAttribute::min_motor_force_angle: {
      static const float3 default_value = float3(-FLT_MAX);
      return &default_value;
    }
    case ConstraintAttribute::max_motor_force_axis:
    case ConstraintAttribute::max_motor_force_angle: {
      static const float3 default_value = float3(FLT_MAX);
      return &default_value;
    }

    case ConstraintAttribute::spring_stiffness_axis:
    case ConstraintAttribute::spring_stiffness_angle:
    case ConstraintAttribute::spring_damping_axis:
    case ConstraintAttribute::spring_damping_angle:
    case ConstraintAttribute::max_friction_axis:
    case ConstraintAttribute::max_friction_angle:
    case ConstraintAttribute::motor_spring_stiffness_axis:
    case ConstraintAttribute::motor_spring_stiffness_angle:
    case ConstraintAttribute::motor_spring_damping_axis:
    case ConstraintAttribute::motor_spring_damping_angle:
      return type.default_value();
  }
  BLI_assert_unreachable();
  return nullptr;
}

/** True if the attribute is stored in the physics state and world data is updated after writing.
 */
bool physics_attribute_use_write_cache(const PhysicsBodyAttribute attribute)
{
  using BodyAttribute = PhysicsBodyAttribute;

  switch (attribute) {
    case BodyAttribute::collision_shape:
    case BodyAttribute::mass:
    case BodyAttribute::inertia:
      return true;

    case BodyAttribute::motion_type:
    case BodyAttribute::position:
    case BodyAttribute::rotation:
    case BodyAttribute::velocity:
    case BodyAttribute::angular_velocity:
    case BodyAttribute::friction:
    case BodyAttribute::restitution:
    case BodyAttribute::is_active:
    case BodyAttribute::allow_sleep:
    case BodyAttribute::linear_damping:
    case BodyAttribute::angular_damping:
    case BodyAttribute::total_force:
    case BodyAttribute::total_torque:
      return false;
  }
  BLI_assert_unreachable();
  return true;
}

/** True if the attribute is stored in the physics state and world data is updated after writing.
 */
bool physics_attribute_use_write_cache(const PhysicsConstraintAttribute attribute)
{
  // TODO make these mostly local for Jolt physics constraints
  using ConstraintAttribute = PhysicsConstraintAttribute;

  switch (attribute) {
    case ConstraintAttribute::type:
    case ConstraintAttribute::body1:
    case ConstraintAttribute::body2:
    case ConstraintAttribute::frame1:
    case ConstraintAttribute::frame2:
    case ConstraintAttribute::limit_min_axis:
    case ConstraintAttribute::limit_max_axis:
    case ConstraintAttribute::limit_min_angle:
    case ConstraintAttribute::limit_max_angle:
    case ConstraintAttribute::spring_stiffness_axis:
    case ConstraintAttribute::spring_stiffness_angle:
    case ConstraintAttribute::spring_damping_axis:
    case ConstraintAttribute::spring_damping_angle:
    case ConstraintAttribute::max_friction_axis:
    case ConstraintAttribute::max_friction_angle:
    case ConstraintAttribute::motor_spring_stiffness_axis:
    case ConstraintAttribute::motor_spring_stiffness_angle:
    case ConstraintAttribute::motor_spring_damping_axis:
    case ConstraintAttribute::motor_spring_damping_angle:
    case ConstraintAttribute::min_motor_force_axis:
    case ConstraintAttribute::min_motor_force_angle:
    case ConstraintAttribute::max_motor_force_axis:
    case ConstraintAttribute::max_motor_force_angle:
      return true;

    case ConstraintAttribute::enabled:
      return false;
  }
  BLI_assert_unreachable();
  return true;
}

}  // namespace physics_attributes

/** \} */

using namespace physics_attributes;

using WorldStateUpdateOnChange = void (*)(void *owner);

static WorldStateUpdateOnChange world_state_update_on_change_fn(
    const PhysicsBodyAttribute attribute)
{
  using BodyAttribute = PhysicsBodyAttribute;

  switch (attribute) {
    case BodyAttribute::collision_shape: {
      static WorldStateUpdateOnChange update = [](void *owner) {
        PhysicsWorldState &state = get_physics_owner(owner);
        state.tag_body_collision_shape_changed();
      };
      return update;
    }
    case BodyAttribute::mass:
    case BodyAttribute::inertia: {
      static WorldStateUpdateOnChange update = [](void *owner) {
        PhysicsWorldState &state = get_physics_owner(owner);
        state.tag_body_mass_changed();
      };
      return update;
    }

    case BodyAttribute::motion_type:
    case BodyAttribute::position:
    case BodyAttribute::rotation:
    case BodyAttribute::velocity:
    case BodyAttribute::angular_velocity:
    case BodyAttribute::is_active:
    case BodyAttribute::allow_sleep:
    case BodyAttribute::friction:
    case BodyAttribute::restitution:
    case BodyAttribute::linear_damping:
    case BodyAttribute::angular_damping:
    case BodyAttribute::total_force:
    case BodyAttribute::total_torque: {
      BLI_assert(!physics_attribute_use_write_cache(attribute));
      static WorldStateUpdateOnChange update = [](void *owner) {
        PhysicsWorldState &state = get_physics_owner(owner);
        state.tag_read_cache_changed();
      };
      return update;
    }
  }
  BLI_assert_unreachable();
  return nullptr;
}

static WorldStateUpdateOnChange world_state_update_on_change_fn(
    const PhysicsConstraintAttribute attribute)
{
  using ConstraintAttribute = PhysicsConstraintAttribute;

  switch (attribute) {
    case ConstraintAttribute::type:
    case ConstraintAttribute::body1:
    case ConstraintAttribute::body2:
    case ConstraintAttribute::frame1:
    case ConstraintAttribute::frame2:
    case ConstraintAttribute::limit_min_axis:
    case ConstraintAttribute::limit_max_axis:
    case ConstraintAttribute::limit_min_angle:
    case ConstraintAttribute::limit_max_angle:
    case ConstraintAttribute::spring_stiffness_axis:
    case ConstraintAttribute::spring_stiffness_angle:
    case ConstraintAttribute::spring_damping_axis:
    case ConstraintAttribute::spring_damping_angle:
    case ConstraintAttribute::max_friction_axis:
    case ConstraintAttribute::max_friction_angle:
    case ConstraintAttribute::motor_spring_stiffness_axis:
    case ConstraintAttribute::motor_spring_stiffness_angle:
    case ConstraintAttribute::motor_spring_damping_axis:
    case ConstraintAttribute::motor_spring_damping_angle:
    case ConstraintAttribute::min_motor_force_axis:
    case ConstraintAttribute::min_motor_force_angle:
    case ConstraintAttribute::max_motor_force_axis:
    case ConstraintAttribute::max_motor_force_angle: {
      BLI_assert(physics_attribute_use_write_cache(attribute));
      static WorldStateUpdateOnChange update = [](void *owner) {
        PhysicsWorldState &state = get_physics_owner(owner);
        state.tag_constraints_changed();
      };
      return update;
    }

    case ConstraintAttribute::enabled: {
      BLI_assert(!physics_attribute_use_write_cache(attribute));
      static WorldStateUpdateOnChange update = [](void *owner) {
        PhysicsWorldState &state = get_physics_owner(owner);
        state.tag_read_cache_changed();
      };
      return update;
    }
  }
  BLI_assert_unreachable();
  return nullptr;
}

PhysicsStateBodyAttributeProvider::PhysicsStateBodyAttributeProvider(
    PhysicsStateAttributeAccessMode access_mode, PhysicsBodyAttribute attribute)
    : BuiltinAttributeProvider(physics_attribute_name(attribute),
                               bke::AttrDomain::Point,
                               cpp_type_to_custom_data_type(physics_attribute_type(attribute)),
                               NonDeletable,
                               {}),
      attribute_(attribute),
      world_data_provider_(attribute,
                           PhysicsWorldState::world_data_access_info(),
                           world_state_update_on_change_fn(attribute),
                           {}),
      access_mode_(access_mode)
{
}

GAttributeReader PhysicsStateBodyAttributeProvider::try_get_for_read(const void *owner) const
{
  const PhysicsWorldState &state = get_physics_owner(owner);

  /* Only read from world data in Direct mode. */
  if (access_mode_ == PhysicsStateAttributeAccessMode::DirectReadWrite) {
    return world_data_provider_.try_get_for_read(owner);
  }

  /* Update the read cache from world data.
   * In cached read+write mode this should be ignored, among other things to avoid deadlock since
   * building the cache will try reading the attribute in turn. */
  if (access_mode_ != PhysicsStateAttributeAccessMode::CachedReadWrite) {
    state.ensure_read_cache();
  }

  const GArray<> *data = state.body_data_.lookup_ptr(attribute_);
  GVArray varray;
  if (data == nullptr) {
    varray = GVArray::ForSingle(physics_attribute_type(attribute_),
                                state.bodies_num(),
                                physics_attribute_default_value(attribute_));
  }
  else {
    BLI_assert(data->size() == state.bodies_num());
    varray = GVArray::ForSpan(data->as_span());
  }
  return {std::move(varray), domain_, nullptr};
}

GAttributeWriter PhysicsStateBodyAttributeProvider::try_get_for_write(void *owner) const
{
  PhysicsWorldState &state = get_physics_owner(owner);
  const bool is_state_local = physics_attribute_use_write_cache(attribute_);

  /* Write to world data unless the attribute is stored on the state. */
  if (ELEM(access_mode_,
           PhysicsStateAttributeAccessMode::DirectReadWrite,
           PhysicsStateAttributeAccessMode::CachedRead) &&
      state.has_world_data() && !is_state_local)
  {
    return world_data_provider_.try_get_for_write(owner);
  }

  state.ensure_attribute_cache(attribute_);

  GArray<> *data = state.body_data_.lookup_ptr(attribute_);
  BLI_assert(data != nullptr);

  GVMutableArray varray = physics_attribute_cache_vmutablearray(attribute_, *data);
  return {std::move(varray), domain_, nullptr};
}

bool PhysicsStateBodyAttributeProvider::try_delete(void * /*owner*/) const
{
  return false;
}

bool PhysicsStateBodyAttributeProvider::try_create(void * /*owner*/,
                                                   const AttributeInit & /*initializer*/) const
{
  return false;
}

bool PhysicsStateBodyAttributeProvider::exists(const void * /*owner*/) const
{
  return true;
}

PhysicsStateConstraintAttributeProvider::PhysicsStateConstraintAttributeProvider(
    PhysicsStateAttributeAccessMode access_mode, PhysicsConstraintAttribute attribute)
    : BuiltinAttributeProvider(physics_attribute_name(attribute),
                               bke::AttrDomain::Edge,
                               cpp_type_to_custom_data_type(physics_attribute_type(attribute)),
                               NonDeletable,
                               {}),
      attribute_(attribute),
      world_data_provider_(attribute,
                           PhysicsWorldState::world_data_access_info(),
                           world_state_update_on_change_fn(attribute),
                           {}),
      access_mode_(access_mode)
{
}

GAttributeReader PhysicsStateConstraintAttributeProvider::try_get_for_read(const void *owner) const
{
  const PhysicsWorldState &state = get_physics_owner(owner);

  /* Only read from world data in Direct mode. */
  if (access_mode_ == PhysicsStateAttributeAccessMode::DirectReadWrite) {
    return world_data_provider_.try_get_for_read(owner);
  }

  /* Update the read cache from world data.
   * In cached read+write mode this should be ignored, among other things to avoid deadlock since
   * building the cache will try reading the attribute in turn. */
  if (access_mode_ != PhysicsStateAttributeAccessMode::CachedReadWrite) {
    state.ensure_read_cache();
  }

  const GArray<> *data = state.constraint_data_.lookup_ptr(attribute_);
  GVArray varray;
  if (data == nullptr) {
    varray = GVArray::ForSingle(physics_attribute_type(attribute_),
                                state.constraints_num(),
                                physics_attribute_default_value(attribute_));
  }
  else {
    BLI_assert(data->size() == state.constraints_num());
    varray = GVArray::ForSpan(data->as_span());
  }
  return {std::move(varray), domain_, nullptr};
}

GAttributeWriter PhysicsStateConstraintAttributeProvider::try_get_for_write(void *owner) const
{
  PhysicsWorldState &state = get_physics_owner(owner);
  const bool is_state_local = physics_attribute_use_write_cache(attribute_);

  /* Write to world data unless the attribute is stored on the state. */
  if (ELEM(access_mode_,
           PhysicsStateAttributeAccessMode::DirectReadWrite,
           PhysicsStateAttributeAccessMode::CachedRead) &&
      state.has_world_data() && !is_state_local)
  {
    return world_data_provider_.try_get_for_write(owner);
  }

  state.ensure_attribute_cache(attribute_);

  GArray<> *data = state.constraint_data_.lookup_ptr(attribute_);
  BLI_assert(data != nullptr);

  GVMutableArray varray = physics_attribute_cache_vmutablearray(attribute_, *data);
  return {std::move(varray), domain_, nullptr};
}

bool PhysicsStateConstraintAttributeProvider::try_delete(void * /*owner*/) const
{
  return false;
}

bool PhysicsStateConstraintAttributeProvider::try_create(
    void * /*owner*/, const AttributeInit & /*initializer*/) const
{
  return false;
}

bool PhysicsStateConstraintAttributeProvider::exists(const void * /*owner*/) const
{
  return true;
}

}  // namespace blender::bke
