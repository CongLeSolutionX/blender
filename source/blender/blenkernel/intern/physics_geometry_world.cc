/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include <functional>

#include "BLI_array.hh"
#include "BLI_generic_virtual_array.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_matrix_types.hh"

#include "BKE_attribute.hh"
#include "BKE_collision_shape.hh"

#include "attribute_access_intern.hh"
#include "physics_geometry_attributes.hh"
#include "physics_geometry_intern.hh"
#include "physics_geometry_world.hh"

#ifdef WITH_JOLT
#  include <Jolt/Core/TempAllocator.h>
#  include <Jolt/Physics/Body/BodyCreationSettings.h>
#  include <Jolt/Physics/Collision/Shape/SphereShape.h>
#  include <Jolt/Physics/Constraints/ConeConstraint.h>
#  include <Jolt/Physics/Constraints/Constraint.h>
#  include <Jolt/Physics/Constraints/DistanceConstraint.h>
#  include <Jolt/Physics/Constraints/FixedConstraint.h>
#  include <Jolt/Physics/Constraints/GearConstraint.h>
#  include <Jolt/Physics/Constraints/HingeConstraint.h>
#  include <Jolt/Physics/Constraints/PathConstraint.h>
#  include <Jolt/Physics/Constraints/PointConstraint.h>
#  include <Jolt/Physics/Constraints/PulleyConstraint.h>
#  include <Jolt/Physics/Constraints/RackAndPinionConstraint.h>
#  include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#  include <Jolt/Physics/Constraints/SliderConstraint.h>
#  include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#endif

namespace blender::bke {

#ifdef WITH_JOLT

static const AttributeAccessorFunctions &get_accessor_functions_ref();

static float4x4 get_constraint_frame1(const JPH::Constraint &constraint)
{
  switch (constraint.GetType()) {
    case JPH::EConstraintType::Constraint:
      return float4x4::identity();
    case JPH::EConstraintType::TwoBodyConstraint: {
      const JPH::TwoBodyConstraint &two_body_constraint =
          static_cast<const JPH::TwoBodyConstraint &>(constraint);
      return to_blender(two_body_constraint.GetConstraintToBody1Matrix());
    }
  }
  BLI_assert_unreachable();
  return float4x4::identity();
}

static void set_constraint_frame1(JPH::Constraint & /*constraint*/, float4x4 /*value*/) {}

static float4x4 get_constraint_frame2(const JPH::Constraint &constraint)
{
  switch (constraint.GetType()) {
    case JPH::EConstraintType::Constraint:
      return float4x4::identity();
    case JPH::EConstraintType::TwoBodyConstraint: {
      const JPH::TwoBodyConstraint &two_body_constraint =
          static_cast<const JPH::TwoBodyConstraint &>(constraint);
      return to_blender(two_body_constraint.GetConstraintToBody2Matrix());
    }
  }
  BLI_assert_unreachable();
  return float4x4::identity();
}

static void set_constraint_frame2(JPH::Constraint & /*constraint*/, float4x4 /*value*/) {}

static Array<JPH::BodyID> get_valid_body_ids(const Span<JPH::Body *> bodies,
                                             const IndexMask &selection)
{
  IndexMaskMemory memory;
  IndexMask selection_valid = IndexMask::from_predicate(
      selection, GrainSize(4096), memory, [&](const int index) {
        return bodies[index] != nullptr;
      });

  Array<JPH::BodyID> body_ids(selection_valid.size());
  selection_valid.foreach_index(GrainSize(4096), [&](const int src_i, const int dst_i) {
    body_ids[dst_i] = bodies[src_i]->GetID();
  });
  return body_ids;
}

static Array<JPH::BodyID> get_valid_body_ids(const Span<JPH::Body *> bodies)
{
  return get_valid_body_ids(bodies, bodies.index_range());
}

static Array<JPH::Constraint *> get_valid_constraints(const Span<JPH::Constraint *> constraints,
                                                      const IndexMask &selection)
{
  IndexMaskMemory memory;
  IndexMask selection_valid = IndexMask::from_predicate(
      selection, GrainSize(4096), memory, [&](const int index) {
        return constraints[index] != nullptr;
      });

  Array<JPH::Constraint *> valid_constraints(selection_valid.size());
  selection_valid.foreach_index(GrainSize(4096), [&](const int src_i, const int dst_i) {
    valid_constraints[dst_i] = constraints[src_i];
  });
  return valid_constraints;
}

static Array<JPH::Constraint *> get_valid_constraints(const Span<JPH::Constraint *> constraints)
{
  return get_valid_constraints(constraints, constraints.index_range());
}

static void create_default_bodies(JPH::BodyInterface &body_interface,
                                  MutableSpan<JPH::Body *> bodies,
                                  const IndexMask &mask)
{
  static const JPH::RefConst<JPH::SphereShape> default_shape = JPH::RefConst<JPH::SphereShape>(
      new JPH::SphereShape(1.0f));
  static const JPH::EActivation activation = JPH::EActivation::Activate;

  mask.foreach_index([&](const int index) {
    JPH::BodyCreationSettings settings = {default_shape,
                                          to_jolt(float3(0.0f)),
                                          to_jolt(math::Quaternion::identity()),
                                          JPH::EMotionType::Dynamic,
                                          Layers::moving};
    bodies[index] = body_interface.CreateBody(settings);
  });

  Array<JPH::BodyID> body_ids = get_valid_body_ids(bodies, mask);
  JPH::BodyInterface::AddState add_state = body_interface.AddBodiesPrepare(body_ids.data(),
                                                                           body_ids.size());
  body_interface.AddBodiesFinalize(body_ids.data(), body_ids.size(), add_state, activation);
}

static void create_default_constraints(const IndexMask &selection,
                                       MutableSpan<JPH::Constraint *> constraints)
{
  /* Transforms are defined relative to the respective body. */
  constexpr JPH::EConstraintSpace constraint_space = JPH::EConstraintSpace::LocalToBodyCOM;

  selection.foreach_index(GrainSize(256), [&](const int index) {
    JPH::Body *body1 = &JPH::Body::sFixedToWorld;
    JPH::Body *body2 = &JPH::Body::sFixedToWorld;

    JPH::FixedConstraintSettings settings;
    settings.mSpace = constraint_space;
    settings.mPoint1 = to_jolt(float3(0));
    settings.mAxisX1 = to_jolt(float3(1, 0, 0));
    settings.mAxisY1 = to_jolt(float3(0, 1, 0));
    settings.mPoint2 = to_jolt(float3(0));
    settings.mAxisX2 = to_jolt(float3(1, 0, 0));
    settings.mAxisY2 = to_jolt(float3(0, 1, 0));
    constraints[index] = settings.Create(*body1, *body2);
  });
}

/* 6-dimensional vector for combined linear and angular values. */
using SixDOFValue = VecBase<float, 6>;

static JPH::TwoBodyConstraint *make_constraint_type(const PhysicsConstraintType type,
                                                    const JPH::EConstraintSpace space,
                                                    const float4x4 &frame1,
                                                    const float4x4 &frame2,
                                                    JPH::Body &body1,
                                                    JPH::Body &body2,
                                                    const SixDOFValue &limit_min,
                                                    const SixDOFValue &limit_max,
                                                    const SixDOFValue &spring_stiffness,
                                                    const SixDOFValue &spring_damping,
                                                    const SixDOFValue &max_friction,
                                                    const SixDOFValue &motor_spring_stiffness,
                                                    const SixDOFValue &motor_spring_damping,
                                                    const SixDOFValue &min_motor_force,
                                                    const SixDOFValue &max_motor_force)
{
  using ConstraintType = PhysicsConstraintType;

  /* TODO Make this configurable?
   * Frequency mode is basically a mass-agnostic spring constraint that defines stiffness as
   * acceleration. This could also be a boolean flag. */
  constexpr JPH::ESpringMode spring_mode = JPH::ESpringMode::StiffnessAndDamping;

  auto spring_settings = [&](const int axis, JPH::SpringSettings r_spring_settings) {
    r_spring_settings = JPH::SpringSettings(
        spring_mode, spring_stiffness[axis], spring_damping[axis]);
  };
  auto limit_settings = [&](const int axis, float &r_min, float &r_max) {
    r_min = limit_min[axis];
    r_max = limit_max[axis];
  };
  auto limit_spring_settings =
      [&](const int axis, float &r_min, float &r_max, JPH::SpringSettings &r_spring_settings) {
        const bool is_angle = (axis >= 3);
        if (is_angle) {
          r_min = std::clamp(limit_min[axis], -JPH::JPH_PI, 0.0f);
          r_max = std::clamp(limit_max[axis], 0.0f, JPH::JPH_PI);
        }
        else {
          r_min = limit_min[axis];
          r_max = limit_max[axis];
        }
        spring_settings(axis, r_spring_settings);
      };
  auto limit_friction_settings =
      [&](const int axis, float &r_min, float &r_max, float &r_max_friction) {
        limit_settings(axis, r_min, r_max);
        r_max_friction = max_friction[axis];
      };
  auto limit_spring_friction_settings = [&](const int axis,
                                            float &r_min,
                                            float &r_max,
                                            float &r_max_friction,
                                            JPH::SpringSettings &r_spring_settings) {
    limit_spring_settings(axis, r_min, r_max, r_spring_settings);
    r_max_friction = max_friction[axis];
  };
  auto motor_settings = [&](const int axis, JPH::MotorSettings &r_motor_settings) {
    r_motor_settings.mSpringSettings = JPH::SpringSettings(
        spring_mode, motor_spring_stiffness[axis], motor_spring_damping[axis]);
    if (axis < 3) {
      r_motor_settings.mMinForceLimit = min_motor_force[axis];
      r_motor_settings.mMaxForceLimit = max_motor_force[axis];
    }
    else {
      r_motor_settings.mMinTorqueLimit = min_motor_force[axis];
      r_motor_settings.mMaxTorqueLimit = max_motor_force[axis];
    }
  };

  [[maybe_unused]] constexpr int axis_x = 0;
  [[maybe_unused]] constexpr int axis_y = 1;
  [[maybe_unused]] constexpr int axis_z = 2;
  [[maybe_unused]] constexpr int angle_x = 3;
  [[maybe_unused]] constexpr int angle_y = 4;
  [[maybe_unused]] constexpr int angle_z = 5;

  switch (type) {
    case ConstraintType::Fixed: {
      JPH::FixedConstraintSettings settings;
      settings.mSpace = space;
      settings.mPoint1 = to_jolt(frame1.location());
      settings.mAxisX1 = to_jolt(frame1.x_axis());
      settings.mAxisY1 = to_jolt(frame1.y_axis());
      settings.mPoint2 = to_jolt(frame2.location());
      settings.mAxisX2 = to_jolt(frame2.x_axis());
      settings.mAxisY2 = to_jolt(frame2.y_axis());
      return settings.Create(body1, body2);
    }
    case ConstraintType::Distance: {
      JPH::DistanceConstraintSettings settings;
      settings.mSpace = space;
      settings.mPoint1 = to_jolt(frame1.location());
      settings.mPoint2 = to_jolt(frame2.location());
      limit_spring_settings(
          axis_x, settings.mMinDistance, settings.mMaxDistance, settings.mLimitsSpringSettings);
      return settings.Create(body1, body2);
    }
    case ConstraintType::Point: {
      JPH::PointConstraintSettings settings;
      settings.mSpace = space;
      settings.mPoint1 = to_jolt(frame1.location());
      settings.mPoint2 = to_jolt(frame2.location());
      return settings.Create(body1, body2);
    }
    case ConstraintType::Hinge: {
      JPH::HingeConstraintSettings settings;
      settings.mSpace = space;
      settings.mPoint1 = to_jolt(frame1.location());
      settings.mHingeAxis1 = to_jolt(frame1.x_axis());
      settings.mNormalAxis1 = to_jolt(frame1.y_axis());
      settings.mPoint2 = to_jolt(frame2.location());
      settings.mHingeAxis2 = to_jolt(frame2.x_axis());
      settings.mNormalAxis2 = to_jolt(frame2.y_axis());
      limit_spring_friction_settings(angle_x,
                                     settings.mLimitsMin,
                                     settings.mLimitsMax,
                                     settings.mMaxFrictionTorque,
                                     settings.mLimitsSpringSettings);
      return settings.Create(body1, body2);
    }
    case ConstraintType::Cone: {
      JPH::ConeConstraintSettings settings;
      settings.mSpace = space;
      settings.mPoint1 = to_jolt(frame1.location());
      settings.mTwistAxis1 = to_jolt(frame1.x_axis());
      settings.mPoint2 = to_jolt(frame2.location());
      settings.mTwistAxis2 = to_jolt(frame2.x_axis());
      settings.mHalfConeAngle = limit_max[angle_x];
      return settings.Create(body1, body2);
    }
    case ConstraintType::Slider: {
      JPH::SliderConstraintSettings settings;
      settings.mSpace = space;
      settings.mAutoDetectPoint = false;
      settings.mPoint1 = to_jolt(frame1.location());
      settings.mSliderAxis1 = to_jolt(frame1.x_axis());
      settings.mNormalAxis1 = to_jolt(frame1.y_axis());
      settings.mPoint2 = to_jolt(frame2.location());
      settings.mSliderAxis2 = to_jolt(frame2.x_axis());
      settings.mNormalAxis2 = to_jolt(frame2.y_axis());
      limit_spring_friction_settings(axis_x,
                                     settings.mLimitsMin,
                                     settings.mLimitsMax,
                                     settings.mMaxFrictionForce,
                                     settings.mLimitsSpringSettings);
      motor_settings(axis_x, settings.mMotorSettings);
      return settings.Create(body1, body2);
    }
    case ConstraintType::SwingTwist: {
      constexpr JPH::ESwingType swing_type = JPH::ESwingType::Cone;

      JPH::SwingTwistConstraintSettings settings;
      settings.mSpace = space;
      settings.mPosition1 = to_jolt(frame1.location());
      settings.mTwistAxis1 = to_jolt(frame1.x_axis());
      settings.mPlaneAxis1 = to_jolt(frame1.y_axis());
      settings.mPosition2 = to_jolt(frame2.location());
      settings.mTwistAxis2 = to_jolt(frame2.x_axis());
      settings.mPlaneAxis2 = to_jolt(frame2.y_axis());
      settings.mSwingType = swing_type;
      settings.mPlaneHalfConeAngle = limit_max[angle_y];
      settings.mNormalHalfConeAngle = limit_max[angle_z];
      limit_friction_settings(
          angle_x, settings.mTwistMinAngle, settings.mTwistMaxAngle, settings.mMaxFrictionTorque);
      motor_settings(angle_x, settings.mTwistMotorSettings);
      motor_settings(angle_y, settings.mSwingMotorSettings);
      return settings.Create(body1, body2);
    }
    case ConstraintType::SixDOF: {
      constexpr JPH::ESwingType swing_type = JPH::ESwingType::Cone;

      JPH::SixDOFConstraintSettings settings;
      settings.mSpace = space;
      settings.mPosition1 = to_jolt(frame1.location());
      settings.mAxisX1 = to_jolt(frame1.x_axis());
      settings.mAxisY1 = to_jolt(frame1.y_axis());
      settings.mPosition2 = to_jolt(frame2.location());
      settings.mAxisX2 = to_jolt(frame2.x_axis());
      settings.mAxisY2 = to_jolt(frame2.y_axis());
      settings.mSwingType = swing_type;
      for (const int axis : IndexRange(6)) {
        limit_spring_friction_settings(axis,
                                       settings.mLimitMin[axis],
                                       settings.mLimitMax[axis],
                                       settings.mMaxFriction[axis],
                                       settings.mLimitsSpringSettings[axis]);
        motor_settings(axis, settings.mMotorSettings[axis]);
      }
      return settings.Create(body1, body2);
    }
    case ConstraintType::Path: {
      JPH::PathConstraintSettings settings;
      /* TODO should be able to share path data, maybe via implicit shared curves? */
      return settings.Create(body1, body2);
    }
    case ConstraintType::Gear: {
      JPH::GearConstraintSettings settings;
      settings.mSpace = space;
      settings.mHingeAxis1 = to_jolt(frame1.x_axis());
      settings.mHingeAxis2 = to_jolt(frame2.x_axis());
      /* Using spring stiffness parameter as gear ratio. */
      settings.mRatio = spring_stiffness[angle_x];
      return settings.Create(body1, body2);
    }
    case ConstraintType::RackAndPinion: {
      JPH::RackAndPinionConstraintSettings settings;
      settings.mSpace = space;
      settings.mHingeAxis = to_jolt(frame1.x_axis());
      settings.mSliderAxis = to_jolt(frame2.x_axis());
      /* Using spring stiffness parameter as rack-to-pinion ratio. */
      settings.mRatio = spring_stiffness[axis_x];
      return settings.Create(body1, body2);
    }
    case ConstraintType::Pulley: {
      JPH::PulleyConstraintSettings settings;
      settings.mSpace = space;
      settings.mBodyPoint1 = to_jolt(frame1.location());
      settings.mBodyPoint2 = to_jolt(frame2.location());
      /* TODO How to define the two fixed points? */
      // settings.mFixedPoint1 = ;
      // settings.mFixedPoint2 = ;
      /* Using spring stiffness parameter as pulley ratio. */
      settings.mRatio = spring_stiffness[axis_x];
      limit_settings(axis_x, settings.mMinLength, settings.mMaxLength);
      return settings.Create(body1, body2);
    }
    case ConstraintType::Vehicle: {
      /* TODO */
      BLI_assert_unreachable();
      return nullptr;
    }
  }
  BLI_assert_unreachable();
  return nullptr;
}

static void update_jolt_constraints(const IndexMask &selection,
                                    const AttributeAccessor attributes,
                                    const Span<JPH::Body *> bodies,
                                    MutableSpan<JPH::Constraint *> constraints)
{
  using namespace bke::physics_attributes;
  using ConstraintAttribute = PhysicsConstraintAttribute;
  using ConstraintType = PhysicsConstraintType;

  const VArraySpan<int> types = physics_attribute_lookup_or_default<int>(
      attributes, ConstraintAttribute::type);
  const VArraySpan<int> bodies1 = physics_attribute_lookup_or_default<int>(
      attributes, ConstraintAttribute::body1);
  const VArraySpan<int> bodies2 = physics_attribute_lookup_or_default<int>(
      attributes, ConstraintAttribute::body2);
  const VArraySpan<float4x4> frames1 = physics_attribute_lookup_or_default<float4x4>(
      attributes, ConstraintAttribute::frame1);
  const VArraySpan<float4x4> frames2 = physics_attribute_lookup_or_default<float4x4>(
      attributes, ConstraintAttribute::frame2);
  const VArraySpan<float3> limit_min_axis = physics_attribute_lookup_or_default<float3>(
      attributes, ConstraintAttribute::limit_min_axis);
  const VArraySpan<float3> limit_max_axis = physics_attribute_lookup_or_default<float3>(
      attributes, ConstraintAttribute::limit_max_axis);
  const VArraySpan<float3> limit_min_angle = physics_attribute_lookup_or_default<float3>(
      attributes, ConstraintAttribute::limit_min_angle);
  const VArraySpan<float3> limit_max_angle = physics_attribute_lookup_or_default<float3>(
      attributes, ConstraintAttribute::limit_max_angle);
  const VArraySpan<float3> spring_stiffness_axis = physics_attribute_lookup_or_default<float3>(
      attributes, ConstraintAttribute::spring_stiffness_axis);
  const VArraySpan<float3> spring_stiffness_angle = physics_attribute_lookup_or_default<float3>(
      attributes, ConstraintAttribute::spring_stiffness_angle);
  const VArraySpan<float3> spring_damping_axis = physics_attribute_lookup_or_default<float3>(
      attributes, ConstraintAttribute::spring_damping_axis);
  const VArraySpan<float3> spring_damping_angle = physics_attribute_lookup_or_default<float3>(
      attributes, ConstraintAttribute::spring_damping_angle);
  const VArraySpan<float3> max_friction_axis = physics_attribute_lookup_or_default<float3>(
      attributes, ConstraintAttribute::max_friction_axis);
  const VArraySpan<float3> max_friction_angle = physics_attribute_lookup_or_default<float3>(
      attributes, ConstraintAttribute::max_friction_angle);
  const VArraySpan<float3> motor_spring_stiffness_axis =
      physics_attribute_lookup_or_default<float3>(
          attributes, ConstraintAttribute::motor_spring_stiffness_axis);
  const VArraySpan<float3> motor_spring_stiffness_angle =
      physics_attribute_lookup_or_default<float3>(
          attributes, ConstraintAttribute::motor_spring_stiffness_angle);
  const VArraySpan<float3> motor_spring_damping_axis = physics_attribute_lookup_or_default<float3>(
      attributes, ConstraintAttribute::motor_spring_damping_axis);
  const VArraySpan<float3> motor_spring_damping_angle =
      physics_attribute_lookup_or_default<float3>(attributes,
                                                  ConstraintAttribute::motor_spring_damping_angle);
  const VArraySpan<float3> min_motor_force_axis = physics_attribute_lookup_or_default<float3>(
      attributes, ConstraintAttribute::min_motor_force_axis);
  const VArraySpan<float3> min_motor_force_angle = physics_attribute_lookup_or_default<float3>(
      attributes, ConstraintAttribute::min_motor_force_angle);
  const VArraySpan<float3> max_motor_force_axis = physics_attribute_lookup_or_default<float3>(
      attributes, ConstraintAttribute::max_motor_force_axis);
  const VArraySpan<float3> max_motor_force_angle = physics_attribute_lookup_or_default<float3>(
      attributes, ConstraintAttribute::max_motor_force_angle);

  /* Transforms are defined relative to the respective body. */
  constexpr JPH::EConstraintSpace constraint_space = JPH::EConstraintSpace::LocalToBodyCOM;

  auto axis_values = [](const float3 axis_values, const float3 &angle_values) {
    SixDOFValue v;
    v[0] = axis_values.x;
    v[1] = axis_values.y;
    v[2] = axis_values.z;
    v[3] = angle_values.x;
    v[4] = angle_values.y;
    v[5] = angle_values.z;
    return v;
  };

  selection.foreach_index(GrainSize(256), [&](const int index) {
    const ConstraintType type = ConstraintType(types[index]);
    const int body1_index = bodies1[index];
    const int body2_index = bodies2[index];
    JPH::Body *body1 = (bodies.index_range().contains(body1_index) ? bodies[body1_index] :
                                                                     &JPH::Body::sFixedToWorld);
    JPH::Body *body2 = (bodies.index_range().contains(body2_index) ? bodies[body2_index] :
                                                                     &JPH::Body::sFixedToWorld);
    BLI_assert(body1 != nullptr);
    BLI_assert(body2 != nullptr);

    const float4x4 &frame1 = frames1[index];
    const float4x4 &frame2 = frames2[index];

    const SixDOFValue limit_min = axis_values(limit_min_axis[index], limit_min_angle[index]);
    const SixDOFValue limit_max = axis_values(limit_max_axis[index], limit_max_angle[index]);
    const SixDOFValue spring_stiffness = axis_values(spring_stiffness_axis[index],
                                                     spring_stiffness_angle[index]);
    const SixDOFValue spring_damping = axis_values(spring_damping_axis[index],
                                                   spring_damping_angle[index]);
    const SixDOFValue max_friction = axis_values(max_friction_axis[index],
                                                 max_friction_angle[index]);
    const SixDOFValue motor_spring_stiffness = axis_values(motor_spring_stiffness_axis[index],
                                                           motor_spring_stiffness_angle[index]);
    const SixDOFValue motor_spring_damping = axis_values(motor_spring_damping_axis[index],
                                                         motor_spring_damping_angle[index]);
    const SixDOFValue min_motor_force = axis_values(min_motor_force_axis[index],
                                                    min_motor_force_angle[index]);
    const SixDOFValue max_motor_force = axis_values(max_motor_force_axis[index],
                                                    max_motor_force_angle[index]);

    constraints[index]->Release();

    constraints[index] = make_constraint_type(type,
                                              constraint_space,
                                              frame1,
                                              frame2,
                                              *body1,
                                              *body2,
                                              limit_min,
                                              limit_max,
                                              spring_stiffness,
                                              spring_damping,
                                              max_friction,
                                              motor_spring_stiffness,
                                              motor_spring_damping,
                                              min_motor_force,
                                              max_motor_force);
  });
}

/* -------------------------------------------------------------------- */
/** \name Jolt Physics World Data
 * \{ */

namespace body_flags {
}  // namespace body_flags

namespace constraint_flags {
static constexpr uint8_t in_world_offset = 0;
static constexpr uint8_t index_offset = 32;
static constexpr uint64_t in_world_mask = (1 << in_world_offset);
static constexpr uint64_t index_mask = (uint64_t(0xffffffff) << index_offset);
}  // namespace constraint_flags

// static int get_constraint_index(JPH::Constraint *constraint)
// {
//   if (constraint == nullptr) {
//     return -1;
//   }
//   const uint64_t userdata = constraint->GetUserData();
//   return int((userdata & constraint_flags::index_mask) >> constraint_flags::index_offset);
// }

static void set_constraint_index(JPH::Constraint *constraint, int constraint_index)
{
  if (constraint == nullptr) {
    return;
  }
  const uint64_t userdata = (constraint->GetUserData() & (~constraint_flags::index_mask));
  constraint->SetUserData(userdata |
                          (uint64_t(constraint_index) << constraint_flags::index_offset));
}

static bool is_constraint_in_world(JPH::Constraint *constraint)
{
  if (constraint == nullptr) {
    return false;
  }
  const uint64_t userdata = constraint->GetUserData();
  return (userdata & constraint_flags::in_world_mask) != 0;
}

static void set_constraint_in_world(JPH::Constraint *constraint, bool is_in_world)
{
  if (constraint == nullptr) {
    return;
  }
  const uint64_t userdata = (constraint->GetUserData() & (~constraint_flags::in_world_mask));
  constraint->SetUserData(userdata | (is_in_world ? constraint_flags::in_world_mask : 0));
}

constexpr size_t temp_allocator_size = 10 * 1024 * 1024;
constexpr int max_physics_jobs = 2048;
constexpr int max_physics_barriers = 8;

JoltPhysicsWorldData::JoltPhysicsWorldData()
    : temp_allocator_(temp_allocator_size),
      job_system_(max_physics_jobs, max_physics_barriers, std::thread::hardware_concurrency() - 1)
{
  this->setup();
}

JoltPhysicsWorldData::JoltPhysicsWorldData(int body_num, int constraint_num)
    : temp_allocator_(temp_allocator_size),
      job_system_(max_physics_jobs, max_physics_barriers, std::thread::hardware_concurrency() - 1)
{
  this->setup();
  this->resize(body_num, constraint_num);
}

JoltPhysicsWorldData::~JoltPhysicsWorldData()
{
  this->shutdown();

  JPH::BodyInterface &body_interface = physics_system_.GetBodyInterface();
  const Array<JPH::BodyID> body_ids = get_valid_body_ids(bodies_);
  body_interface.DestroyBodies(body_ids.data(), body_ids.size());

  for (const int i : constraints_.index_range()) {
    constraints_[i]->Release();
  }
}

void JoltPhysicsWorldData::resize(const int body_num, const int constraint_num)
{
  const IndexRange src_body_range = bodies_.index_range().take_front(body_num);
  const IndexRange src_constraint_range = constraints_.index_range().take_front(constraint_num);
  this->resize(body_num, constraint_num, src_body_range, src_constraint_range, 0, 0);
}

void JoltPhysicsWorldData::resize(const int body_num,
                                  const int constraint_num,
                                  const IndexMask &src_body_mask,
                                  const IndexMask &src_constraint_mask,
                                  const int dst_body_offset,
                                  const int dst_constraint_offset)
{
  const IndexRange dst_body_range = src_body_mask.index_range().shift(dst_body_offset);
  const IndexRange dst_constraint_range = src_constraint_mask.index_range().shift(
      dst_constraint_offset);
  BLI_assert(dst_body_range.is_empty() || dst_body_range.last() < body_num);
  BLI_assert(dst_constraint_range.is_empty() || dst_constraint_range.last() < constraint_num);

  /* Check both target range and overall size, in case the size is growing. */
  const bool bodies_changed = (body_num != bodies_.size() ||
                               dst_body_range != bodies_.index_range());
  const bool constraints_changed = (constraint_num != constraints_.size() ||
                                    dst_constraint_range != constraints_.index_range());
  /* If the full range is copied the offset can only be zero. */
  BLI_assert(bodies_changed || dst_body_offset == 0);
  BLI_assert(constraints_changed || dst_constraint_offset == 0);
  if (!bodies_changed && !constraints_changed) {
    return;
  }

  JPH::BodyInterface &body_interface = physics_system_.GetBodyInterface();

  if (bodies_changed) {
    Array<JPH::Body *> new_rigid_bodies(body_num, nullptr);
    src_body_mask.foreach_index([&](const int src_i, const int i) {
      const int dst_i = dst_body_range[i];
      new_rigid_bodies[dst_i] = bodies_[src_i];
      /* Clear to avoid deleting. */
      bodies_[src_i] = nullptr;
    });
    /* Delete unused. */
    Array<JPH::BodyID> body_ids = get_valid_body_ids(bodies_);
    body_interface.RemoveBodies(body_ids.data(), body_ids.size());
    body_interface.DestroyBodies(body_ids.data(), body_ids.size());

    /* Create new default bodies in empty places. */
    const IndexRange full_range = new_rigid_bodies.index_range();
    const IndexRange empty_head = dst_body_range.is_empty() ?
                                      full_range :
                                      full_range.take_front(dst_body_range.first());
    const IndexRange empty_tail = dst_body_range.is_empty() ?
                                      IndexRange{} :
                                      full_range.drop_front(dst_body_range.one_after_last());
    create_default_bodies(body_interface, new_rigid_bodies, empty_head);
    create_default_bodies(body_interface, new_rigid_bodies, empty_tail);
    bodies_ = std::move(new_rigid_bodies);

    body_index_cache_.tag_dirty();
  }

  if (constraints_changed) {
    Array<JPH::Constraint *> new_constraints(constraint_num, nullptr);
    src_constraint_mask.foreach_index([&](const int src_i, const int i) {
      const int dst_i = dst_constraint_range[i];
      new_constraints[dst_i] = constraints_[src_i];
      /* Clear to avoid deleting. */
      constraints_[src_i] = nullptr;
    });
    /* Delete unused. */
    Array<JPH::Constraint *> valid_constraints = get_valid_constraints(constraints_);
    physics_system_.RemoveConstraints(valid_constraints.data(), valid_constraints.size());
    constraints_ = std::move(new_constraints);

    /* Initialize new constraints. */
    IndexMaskMemory memory;
    init_constraints(IndexMask::from_union(
        IndexRange::from_begin_end(0, dst_constraint_range.start()),
        IndexRange::from_begin_end(dst_constraint_range.one_after_last(), constraint_num),
        memory));

    constraint_index_cache_.tag_dirty();
  }

  bodies_in_world_cache_.tag_dirty();
}

void JoltPhysicsWorldData::move(const IndexMask &src_body_mask,
                                const IndexMask &src_constraint_mask,
                                int dst_body_offset,
                                int dst_constraint_offset)
{
  this->resize(bodies_.size(),
               constraints_.size(),
               src_body_mask,
               src_constraint_mask,
               dst_body_offset,
               dst_constraint_offset);
}

void JoltPhysicsWorldData::tag_bodies_in_world() const
{
  bodies_in_world_cache_.tag_dirty();
}

void JoltPhysicsWorldData::tag_constraints_in_world() const
{
  constraints_in_world_cache_.tag_dirty();
}

void JoltPhysicsWorldData::ensure_body_and_constraint_indices() const
{
  /* Note: Technically the physics system is not mutable here. We're just using it as a
   * cache with exclusive write access, so it's fine. */
  body_index_cache_.ensure([&]() {
    JPH::BodyInterface &body_interface =
        const_cast<JPH::PhysicsSystem &>(physics_system_).GetBodyInterface();
    for (const int i : bodies_.index_range()) {
      body_interface.SetUserData(bodies_[i]->GetID(), i);
    }
  });
  constraint_index_cache_.ensure([&]() {
    for (const int i : constraints_.index_range()) {
      set_constraint_index(constraints_[i], i);
    }
  });
}

void JoltPhysicsWorldData::ensure_bodies_and_constraints_in_world()
{
  // TODO determine whether activation is needed and if it's the same for every body.
  static constexpr JPH::EActivation activation = JPH::EActivation ::DontActivate;

  bodies_in_world_cache_.ensure([&]() {
    JPH::BodyInterface &body_interface =
        const_cast<JPH::PhysicsSystem &>(physics_system_).GetBodyInterface();

    IndexMaskMemory memory;
    const IndexMask body_ids_to_add = IndexMask::from_predicate(
        bodies_.index_range(), GrainSize(4096), memory, [&](const int index) -> bool {
          return !bodies_[index]->IsInBroadPhase();
        });
    if (!body_ids_to_add.is_empty()) {
      Array<JPH::BodyID> body_ids = get_valid_body_ids(bodies_, body_ids_to_add);
      JPH::BodyInterface::AddState add_state = body_interface.AddBodiesPrepare(body_ids.data(),
                                                                               body_ids.size());
      body_interface.AddBodiesFinalize(body_ids.data(), body_ids.size(), add_state, activation);
    }
  });

  constraints_in_world_cache_.ensure([&]() {
    Array<JPH::Constraint *> constraints_to_add(constraints_.size());
    for (const int i : constraints_.index_range()) {
      JPH::Constraint *constraint = constraints_[i];
      if (is_constraint_in_world(constraint)) {
        constraints_to_add[i] = nullptr;
        continue;
      }
      constraints_to_add[i] = constraint;
      set_constraint_in_world(constraint, true);
    }
    physics_system_.AddConstraints(constraints_to_add.data(), constraints_to_add.size());
  });
}

void JoltPhysicsWorldData::init_constraints(const IndexMask &selection)
{
  if (selection.is_empty()) {
    return;
  }

  create_default_constraints(selection, constraints_);

  constraint_index_cache_.tag_dirty();
  tag_constraints_in_world();
}

void JoltPhysicsWorldData::update_constraints(const IndexMask &selection,
                                              const AttributeAccessor attributes)
{
  if (selection.is_empty()) {
    return;
  }

  /* XXX Jolt bug: The Add/Remove functions should support nullptrs in the array, but don't. Have
   * to use a vector here to avoid gaps if some constraints have not been added to the world yet.
   */
  Vector<JPH::Constraint *> constraints_to_remove;
  constraints_to_remove.reserve(constraints_.size());
  selection.foreach_index([&](const int index) {
    if (is_constraint_in_world(constraints_[index])) {
      constraints_to_remove.append(constraints_[index]);
      set_constraint_in_world(constraints_[index], false);
    }
  });
  physics_system_.RemoveConstraints(constraints_to_remove.data(), constraints_to_remove.size());

  update_jolt_constraints(selection, attributes, bodies_, constraints_);

  constraint_index_cache_.tag_dirty();
  tag_constraints_in_world();
}

float3 JoltPhysicsWorldData::gravity() const
{
  return to_blender(physics_system_.GetGravity());
}

void JoltPhysicsWorldData::set_gravity(const float3 &gravity)
{
  physics_system_.SetGravity(to_jolt(gravity));
}

void JoltPhysicsWorldData::step_simulation(const float delta_time, const int collision_steps)
{
  this->ensure_bodies_and_constraints_in_world();

  physics_system_.Update(
      std::max(delta_time, 0.0f), std::max(collision_steps, 1), &temp_allocator_, &job_system_);
}

void JoltPhysicsWorldData::set_body_shapes(const IndexMask &selection,
                                           const Span<InstanceReference> shapes,
                                           const Span<int> shape_handles)
{
  static const JPH::EActivation activation_mode = JPH::EActivation::DontActivate;

  JPH::BodyInterface &body_interface = physics_system_.GetBodyInterface();
  selection.foreach_index([&](const int index) {
    const int handle = shape_handles[index];
    if (!shapes.index_range().contains(handle)) {
      return;
    }
    const InstanceReference &reference = shapes[handle];
    const CollisionShape *shape = reference.geometry_set().get_collision_shape();
    if (!shape) {
      return;
    }

    const JPH::Shape &jolt_shape = shape->impl();
    JPH::Body *body = this->bodies_[index];
    /* XXX Jolt bug: triangle shapes always compute zero mass from density,
     * updating motion properties will trigger an assert. */
    const bool update_motion_props = shape->supports_motion() &&
                                     (shape->type() != CollisionShapeType::Triangle);
    const JPH::EActivation body_activation_mode = shape->supports_motion() ?
                                                      activation_mode :
                                                      JPH::EActivation::DontActivate;
    body_interface.SetShape(body->GetID(), &jolt_shape, update_motion_props, body_activation_mode);
  });
}

void JoltPhysicsWorldData::set_body_mass(const IndexMask &selection,
                                         const Span<float> masses,
                                         const Span<float4x4> inertias)
{
  selection.foreach_index([&](const int index) {
    JPH::Body *body = this->bodies_[index];
    if (body->IsStatic()) {
      return;
    }

    JPH::MassProperties mass_props;
    mass_props.mMass = to_jolt(masses[index]);
    mass_props.mInertia = to_jolt(inertias[index]);

    JPH::MotionProperties &motion_props = *body->GetMotionProperties();
    motion_props.SetMassProperties(motion_props.GetAllowedDOFs(), mass_props);
  });
}

void JoltPhysicsWorldData::apply_force(const IndexMask &selection,
                                       const VArray<float3> &forces,
                                       const VArray<float3> &positions)
{
  JPH::BodyInterface &body_interface = physics_system_.GetBodyInterface();
  const JPH::EActivation activation_mode = JPH::EActivation::Activate;

  if (!positions) {
    selection.foreach_index([&](const int index) {
      body_interface.AddForce(bodies_[index]->GetID(), to_jolt(forces[index]), activation_mode);
    });
  }

  selection.foreach_index([&](const int index) {
    body_interface.AddForce(bodies_[index]->GetID(),
                            to_jolt(forces[index]),
                            to_jolt(positions[index]),
                            activation_mode);
  });
}

void JoltPhysicsWorldData::apply_torque(const IndexMask &selection, const VArray<float3> &torques)
{
  JPH::BodyInterface &body_interface = physics_system_.GetBodyInterface();
  const JPH::EActivation activation_mode = JPH::EActivation::Activate;

  selection.foreach_index([&](const int index) {
    body_interface.AddTorque(bodies_[index]->GetID(), to_jolt(torques[index]), activation_mode);
  });
}

void JoltPhysicsWorldData::apply_impulse(const IndexMask &selection,
                                         const VArray<float3> &impulses,
                                         const VArray<float3> &positions)
{
  JPH::BodyInterface &body_interface = physics_system_.GetBodyInterface();

  if (!positions) {
    selection.foreach_index([&](const int index) {
      body_interface.AddImpulse(bodies_[index]->GetID(), to_jolt(impulses[index]));
    });
  }

  selection.foreach_index([&](const int index) {
    body_interface.AddImpulse(
        bodies_[index]->GetID(), to_jolt(impulses[index]), to_jolt(positions[index]));
  });
}

void JoltPhysicsWorldData::apply_angular_impulse(const IndexMask &selection,
                                                 const VArray<float3> &angular_impulses)
{
  JPH::BodyInterface &body_interface = physics_system_.GetBodyInterface();

  selection.foreach_index([&](const int index) {
    body_interface.AddAngularImpulse(bodies_[index]->GetID(), to_jolt(angular_impulses[index]));
  });
}

JPH::BodyInterface &JoltPhysicsWorldData::body_interface()
{
  return physics_system_.GetBodyInterface();
}

const JPH::BodyInterface &JoltPhysicsWorldData::body_interface() const
{
  return physics_system_.GetBodyInterface();
}

Span<JPH::Body *> JoltPhysicsWorldData::bodies() const
{
  return bodies_;
}

Span<JPH::Constraint *> JoltPhysicsWorldData::constraints() const
{
  return constraints_;
}

bool JoltPhysicsWorldData::validate(const AttributeAccessor /*attributes*/,
                                    const Span<InstanceReference> /*shapes*/)
{
  // TODO
  return true;
}

void JoltPhysicsWorldData::setup()
{
  // This is the max amount of rigid bodies that you can add to the physics system. If you try to
  // add more you'll get an error.
  constexpr int max_bodies = 65536;
  // This determines how many mutexes to allocate to protect rigid bodies from concurrent access.
  // Set it to 0 for the default settings.
  constexpr int num_body_mutexes = 0;
  // This is the max amount of body pairs that can be queued at any time (the broad phase will
  // detect overlapping body pairs based on their bounding boxes and will insert them into a queue
  // for the narrowphase). If you make this buffer too small the queue will fill up and the broad
  // phase jobs will start to do narrow phase work. This is slightly less efficient.
  constexpr int max_body_pairs = 65536;
  // This is the maximum size of the contact constraint buffer. If more contacts (collisions
  // between bodies) are detected than this number then these contacts will be ignored and bodies
  // will start interpenetrating / fall through the world.
  constexpr int max_contact_constraints = 10240;

  physics_system_.Init(max_bodies,
                       num_body_mutexes,
                       max_body_pairs,
                       max_contact_constraints,
                       broad_phase_layer_interface_,
                       object_vs_broadphase_layer_filter_,
                       object_vs_object_layer_filter_);
}

void JoltPhysicsWorldData::shutdown()
{
  JPH::BodyInterface &body_interface = physics_system_.GetBodyInterface();
  if (!bodies_.is_empty()) {
    Array<JPH::BodyID> body_ids = get_valid_body_ids(bodies_);
    body_interface.RemoveBodies(body_ids.data(), body_ids.size());
  }
  // XXX This requires that all constraints have been added to the world, which is not necessarily
  // the case.
  // physics_system_.RemoveConstraints(constraints_.data(), constraints_.size());
}

AttributeAccessor JoltPhysicsWorldData::attributes() const
{
  return AttributeAccessor(this, bke::get_accessor_functions_ref());
}

MutableAttributeAccessor JoltPhysicsWorldData::attributes_for_write()
{
  return MutableAttributeAccessor(this, bke::get_accessor_functions_ref());
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Concrete Attribute Functions
 * \{ */

namespace physics_world_attribute_functions {

static int motion_type_get_fn(const JPH::BodyInterface & /*body_interface*/, const JPH::Body &body)
{
  return int(to_blender(body.GetMotionType()));
}
static int motion_type_validate_fn(const int value)
{
  switch (value) {
    case int(PhysicsMotionType::Dynamic):
    case int(PhysicsMotionType::Static):
    case int(PhysicsMotionType::Kinematic):
      return value;
  }
  return int(PhysicsMotionType::Dynamic);
}
static void motion_type_set_fn(JPH::BodyInterface &body_interface, JPH::Body &body, int value)
{
  const JPH::EMotionType jolt_motion_type = to_jolt(
      PhysicsMotionType(motion_type_validate_fn(value)));
  if (jolt_motion_type == JPH::EMotionType::Static) {
    body_interface.DeactivateBody(body.GetID());
  }
  body.SetMotionType(jolt_motion_type);
}
static float3 position_get_fn(const JPH::BodyInterface & /*body_interface*/, const JPH::Body &body)
{
  return to_blender(body.GetPosition());
}
static void position_set_fn(JPH::BodyInterface &body_interface, JPH::Body &body, float3 value)
{
  static constexpr JPH::EActivation activation = JPH::EActivation::DontActivate;
  body_interface.SetPosition(body.GetID(), to_jolt(value), activation);
}
static math::Quaternion rotation_get_fn(const JPH::BodyInterface & /*body_interface*/,
                                        const JPH::Body &body)
{
  return to_blender(body.GetRotation());
}
static void rotation_set_fn(JPH::BodyInterface &body_interface,
                            JPH::Body &body,
                            math::Quaternion value)
{
  static constexpr JPH::EActivation activation = JPH::EActivation::DontActivate;
  body_interface.SetRotation(body.GetID(), to_jolt(value), activation);
}
static float3 velocity_get_fn(const JPH::BodyInterface & /*body_interface*/, const JPH::Body &body)
{
  return to_blender(body.GetLinearVelocity());
}
static void velocity_set_fn(JPH::BodyInterface &body_interface, JPH::Body &body, float3 value)
{
  body_interface.SetLinearVelocity(body.GetID(), to_jolt(value));
}
static float3 angular_velocity_get_fn(const JPH::BodyInterface & /*body_interface*/,
                                      const JPH::Body &body)
{
  return to_blender(body.GetAngularVelocity());
}
static void angular_velocity_set_fn(JPH::BodyInterface &body_interface,
                                    JPH::Body &body,
                                    float3 value)
{
  body_interface.SetAngularVelocity(body.GetID(), to_jolt(value));
}
static bool is_active_get_fn(const JPH::BodyInterface & /*body_interface*/, const JPH::Body &body)
{
  return body.IsActive();
}
static void is_active_set_fn(JPH::BodyInterface &body_interface, JPH::Body &body, bool value)
{
  // TODO should use bulk setter ActivateBodies
  if (value) {
    body_interface.ActivateBody(body.GetID());
  }
  else {
    body_interface.DeactivateBody(body.GetID());
  }
}
static bool allow_sleep_get_fn(const JPH::BodyInterface & /*body_interface*/,
                               const JPH::Body &body)
{
  return body.GetAllowSleeping();
}
static void allow_sleep_set_fn(JPH::BodyInterface & /*body_interface*/,
                               JPH::Body &body,
                               bool value)
{
  body.SetAllowSleeping(value);
}
static float friction_get_fn(const JPH::BodyInterface & /*body_interface*/, const JPH::Body &body)
{
  return body.GetFriction();
}
static void friction_set_fn(JPH::BodyInterface & /*body_interface*/, JPH::Body &body, float value)
{
  body.SetFriction(value);
}
static float restitution_get_fn(const JPH::BodyInterface & /*body_interface*/,
                                const JPH::Body &body)
{
  return body.GetRestitution();
}
static void restitution_set_fn(JPH::BodyInterface & /*body_interface*/,
                               JPH::Body &body,
                               float value)
{
  body.SetRestitution(value);
}
static float linear_damping_get_fn(const JPH::BodyInterface & /*body_interface*/,
                                   const JPH::Body &body)
{
  const JPH::MotionProperties *motion_props = body.GetMotionPropertiesUnchecked();
  if (motion_props) {
    return motion_props->GetLinearDamping();
  }
  return 0.0f;
}
static void linear_damping_set_fn(JPH::BodyInterface & /*body_interface*/,
                                  JPH::Body &body,
                                  float value)
{
  JPH::MotionProperties *motion_props = body.GetMotionPropertiesUnchecked();
  if (motion_props) {
    return motion_props->SetLinearDamping(value);
  }
}
static float angular_damping_get_fn(const JPH::BodyInterface & /*body_interface*/,
                                    const JPH::Body &body)
{
  const JPH::MotionProperties *motion_props = body.GetMotionPropertiesUnchecked();
  if (motion_props) {
    return motion_props->GetAngularDamping();
  }
  return 0.0f;
}
static void angular_damping_set_fn(JPH::BodyInterface & /*body_interface*/,
                                   JPH::Body &body,
                                   float value)
{
  JPH::MotionProperties *motion_props = body.GetMotionPropertiesUnchecked();
  if (motion_props) {
    return motion_props->SetAngularDamping(value);
  }
}

static float3 total_force_get_fn(const JPH::BodyInterface & /*body_interface*/,
                                 const JPH::Body &body)
{
  const JPH::MotionProperties *motion_props = body.GetMotionPropertiesUnchecked();
  if (motion_props) {
    return to_blender(motion_props->GetAccumulatedForce());
  }
  return float3(0.0f);
}
static float3 total_force_validate_fn(float3 /*value*/)
{
  return float3(0.0f);
}
static void total_force_set_fn(JPH::BodyInterface & /*body_interface*/,
                               JPH::Body & /*body*/,
                               float3 /*value*/)
{
}

static float3 total_torque_get_fn(const JPH::BodyInterface & /*body_interface*/,
                                  const JPH::Body &body)
{
  const JPH::MotionProperties *motion_props = body.GetMotionPropertiesUnchecked();
  if (motion_props) {
    return to_blender(motion_props->GetAccumulatedTorque());
  }
  return float3(0.0f);
}
static float3 total_torque_validate_fn(float3 /*value*/)
{
  return float3(0.0f);
}
static void total_torque_set_fn(JPH::BodyInterface & /*body_interface*/,
                                JPH::Body & /*body*/,
                                float3 /*value*/)
{
}

// static int constraint_type_get_fn(const JPH::Constraint &constraint)
//{
//   return int(bke::PhysicsConstraintType(constraint.getUserConstraintType()));
// }
// static void constraint_type_set_fn(JPH::Constraint & /*constraint*/, int /*value*/) {}
static int constraint_type_validate_fn(const int value)
{
  switch (value) {
    case int(PhysicsConstraintType::Fixed):
    case int(PhysicsConstraintType::Distance):
    case int(PhysicsConstraintType::Point):
    case int(PhysicsConstraintType::Hinge):
    case int(PhysicsConstraintType::Cone):
    case int(PhysicsConstraintType::Slider):
    case int(PhysicsConstraintType::SwingTwist):
    case int(PhysicsConstraintType::SixDOF):
    case int(PhysicsConstraintType::Path):
    case int(PhysicsConstraintType::Gear):
    case int(PhysicsConstraintType::RackAndPinion):
    case int(PhysicsConstraintType::Pulley):
    case int(PhysicsConstraintType::Vehicle):
      return value;
  }
  return int(PhysicsConstraintType::Fixed);
}
static bool constraint_enabled_get_fn(const JPH::Constraint &constraint)
{
  return constraint.GetEnabled();
}
static void constraint_enabled_set_fn(JPH::Constraint &constraint, const bool value)
{
  constraint.SetEnabled(value);
}

// static int constraint_body1_get_fn(const JPH::Constraint &constraint)
//{
//   return get_body_index(constraint.getRigidBodyA());
// }
// static void constraint_body1_set_fn(JPH::Constraint &constraint, int /*value*/) {}
//
// static int constraint_body2_get_fn(const JPH::Constraint &constraint)
//{
//   return get_body_index(constraint.getRigidBodyB());
// }
// static void constraint_body2_set_fn(JPH::Constraint & /*constraint*/, int /*value*/) {}

static float4x4 constraint_frame1_get_fn(const JPH::Constraint &constraint)
{
  return get_constraint_frame1(constraint);
}
static void constraint_frame1_set_fn(JPH::Constraint &constraint, float4x4 value)
{
  set_constraint_frame1(constraint, value);
}

static float4x4 constraint_frame2_get_fn(const JPH::Constraint &constraint)
{
  return get_constraint_frame2(constraint);
}
static void constraint_frame2_set_fn(JPH::Constraint &constraint, float4x4 value)
{
  set_constraint_frame2(constraint, value);
}

}  // namespace physics_world_attribute_functions

/** \} */

/* -------------------------------------------------------------------- */
/** \name VArrays for Physics World Data
 * \{ */

template<typename T>
using BodyGetFn = T (*)(const JPH::BodyInterface &body_interface, const JPH::Body &body);
template<typename T>
using BodySetFn = void (*)(JPH::BodyInterface &body_interface, JPH::Body &body, T value);

template<typename T> using ConstraintGetFn = T (*)(const JPH::Constraint &constraint);
template<typename T> using ConstraintSetFn = void (*)(JPH::Constraint &constraint, T value);

/* Function used for validating a value when writing to the cache. */
template<typename T> using CacheValidateFn = T (*)(T value);

template<typename ElemT, BodyGetFn<ElemT> GetFn, BodySetFn<ElemT> SetFn>
class VMutableArrayImpl_For_PhysicsBodies final : public VMutableArrayImpl<ElemT> {
 private:
  PhysicsWorldData *world_data_;

 public:
  VMutableArrayImpl_For_PhysicsBodies(PhysicsWorldData &world_data)
      : VMutableArrayImpl<ElemT>(world_data.bodies().size()), world_data_(&world_data)
  {
  }

  /* Construct from constant world data.
   * This avoids the need for a separate VArrayImpl class.
   */
  VMutableArrayImpl_For_PhysicsBodies(const PhysicsWorldData &world_data)
      : VMutableArrayImpl<ElemT>(world_data.bodies().size()),
        world_data_(const_cast<PhysicsWorldData *>(&world_data))
  {
  }

 private:
  ElemT get(const int64_t index) const override
  {
    return GetFn(world_data_->body_interface(), *world_data_->bodies()[index]);
  }

  void set(const int64_t index, ElemT value) override
  {
    SetFn(world_data_->body_interface(), *world_data_->bodies()[index], value);
  }

  void materialize(const IndexMask &mask, ElemT *dst) const override
  {
    const JPH::BodyInterface &body_interface = world_data_->body_interface();
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { dst[i] = GetFn(body_interface, *world_data_->bodies()[i]); });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    const JPH::BodyInterface &body_interface = world_data_->body_interface();
    mask.foreach_index_optimized<int64_t>([&](const int64_t i) {
      new (dst + i) ElemT(GetFn(body_interface, *world_data_->bodies()[i]));
    });
  }

  void materialize_compressed(const IndexMask &mask, ElemT *dst) const override
  {
    const JPH::BodyInterface &body_interface = world_data_->body_interface();
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      dst[pos] = GetFn(body_interface, *world_data_->bodies()[i]);
    });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    const JPH::BodyInterface &body_interface = world_data_->body_interface();
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      new (dst + pos) ElemT(GetFn(body_interface, *world_data_->bodies()[i]));
    });
  }
};

template<typename ElemT, ConstraintGetFn<ElemT> GetFn, ConstraintSetFn<ElemT> SetFn>
class VMutableArrayImpl_For_PhysicsConstraints final : public VMutableArrayImpl<ElemT> {
 private:
  PhysicsWorldData *world_data_;

 public:
  VMutableArrayImpl_For_PhysicsConstraints(PhysicsWorldData &world_data)
      : VMutableArrayImpl<ElemT>(world_data.constraints().size()), world_data_(&world_data)
  {
  }

  /* Construct from constant world data.
   * This avoids the need for a separate VArrayImpl class.
   */
  VMutableArrayImpl_For_PhysicsConstraints(const PhysicsWorldData &world_data)
      : VMutableArrayImpl<ElemT>(world_data.constraints().size()),
        world_data_(const_cast<PhysicsWorldData *>(&world_data))
  {
  }

 private:
  ElemT get(const int64_t index) const override
  {
    return GetFn(*world_data_->constraints()[index]);
  }

  void set(const int64_t index, ElemT value) override
  {
    SetFn(*world_data_->constraints()[index], value);
  }

  void materialize(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { dst[i] = GetFn(*world_data_->constraints()[i]); });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { new (dst + i) ElemT(GetFn(*world_data_->constraints()[i])); });
  }

  void materialize_compressed(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      dst[pos] = GetFn(*world_data_->constraints()[i]);
    });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      new (dst + pos) ElemT(GetFn(*world_data_->constraints()[i]));
    });
  }
};

GVMutableArray physics_attribute_vmutablearray(PhysicsBodyAttribute attribute,
                                               PhysicsWorldData &world_data)
{
  using namespace physics_attributes;
  using namespace physics_world_attribute_functions;
  using BodyAttribute = PhysicsBodyAttribute;

  switch (attribute) {
    case BodyAttribute::motion_type:
      return VMutableArray<int>::For<
          VMutableArrayImpl_For_PhysicsBodies<int, motion_type_get_fn, motion_type_set_fn>>(
          world_data);
    case BodyAttribute::position:
      return VMutableArray<float3>::For<
          VMutableArrayImpl_For_PhysicsBodies<float3, position_get_fn, position_set_fn>>(
          world_data);
    case BodyAttribute::rotation:
      return VMutableArray<math::Quaternion>::For<
          VMutableArrayImpl_For_PhysicsBodies<math::Quaternion, rotation_get_fn, rotation_set_fn>>(
          world_data);
    case BodyAttribute::velocity:
      return VMutableArray<float3>::For<
          VMutableArrayImpl_For_PhysicsBodies<float3, velocity_get_fn, velocity_set_fn>>(
          world_data);
    case BodyAttribute::angular_velocity:
      return VMutableArray<float3>::For<
          VMutableArrayImpl_For_PhysicsBodies<float3,
                                              angular_velocity_get_fn,
                                              angular_velocity_set_fn>>(world_data);
    case BodyAttribute::is_active:
      return VMutableArray<bool>::For<
          VMutableArrayImpl_For_PhysicsBodies<bool, is_active_get_fn, is_active_set_fn>>(
          world_data);
    case BodyAttribute::allow_sleep:
      return VMutableArray<bool>::For<
          VMutableArrayImpl_For_PhysicsBodies<bool, allow_sleep_get_fn, allow_sleep_set_fn>>(
          world_data);
    case BodyAttribute::friction:
      return VMutableArray<float>::For<
          VMutableArrayImpl_For_PhysicsBodies<float, friction_get_fn, friction_set_fn>>(
          world_data);
    case BodyAttribute::restitution:
      return VMutableArray<float>::For<
          VMutableArrayImpl_For_PhysicsBodies<float, restitution_get_fn, restitution_set_fn>>(
          world_data);
    case BodyAttribute::linear_damping:
      return VMutableArray<float>::For<VMutableArrayImpl_For_PhysicsBodies<float,
                                                                           linear_damping_get_fn,
                                                                           linear_damping_set_fn>>(
          world_data);
    case BodyAttribute::angular_damping:
      return VMutableArray<float>::For<
          VMutableArrayImpl_For_PhysicsBodies<float,
                                              angular_damping_get_fn,
                                              angular_damping_set_fn>>(world_data);
    case BodyAttribute::total_force:
      return VMutableArray<float3>::For<
          VMutableArrayImpl_For_PhysicsBodies<float3, total_force_get_fn, total_force_set_fn>>(
          world_data);
    case BodyAttribute::total_torque:
      return VMutableArray<float3>::For<
          VMutableArrayImpl_For_PhysicsBodies<float3, total_torque_get_fn, total_torque_set_fn>>(
          world_data);

    default:
      /* Any world data attribute (not state-local) should be handled above. */
      BLI_assert(physics_attribute_use_write_cache(attribute));
      return {};
  }
  BLI_assert_unreachable();
  return {};
}

GVMutableArray physics_attribute_vmutablearray(PhysicsConstraintAttribute attribute,
                                               PhysicsWorldData &world_data)
{
  using namespace physics_attributes;
  using namespace physics_world_attribute_functions;
  using ConstraintAttribute = PhysicsConstraintAttribute;

  switch (attribute) {
    case ConstraintAttribute::enabled:
      return VMutableArray<bool>::For<
          VMutableArrayImpl_For_PhysicsConstraints<bool,
                                                   constraint_enabled_get_fn,
                                                   constraint_enabled_set_fn>>(world_data);
    case ConstraintAttribute::frame1:
      return VMutableArray<float4x4>::For<
          VMutableArrayImpl_For_PhysicsConstraints<float4x4,
                                                   constraint_frame1_get_fn,
                                                   constraint_frame1_set_fn>>(world_data);
    case ConstraintAttribute::frame2:
      return VMutableArray<float4x4>::For<
          VMutableArrayImpl_For_PhysicsConstraints<float4x4,
                                                   constraint_frame2_get_fn,
                                                   constraint_frame2_set_fn>>(world_data);

    default:
      /* Any world data attribute (not state-local) should be handled above. */
      BLI_assert(physics_attribute_use_write_cache(attribute));
      break;
  }
  BLI_assert_unreachable();
  return {};
}

GVArray physics_attribute_varray(PhysicsBodyAttribute attribute,
                                 const PhysicsWorldData &world_data)
{
  return physics_attribute_vmutablearray(attribute, const_cast<PhysicsWorldData &>(world_data));
}

GVArray physics_attribute_varray(PhysicsConstraintAttribute attribute,
                                 const PhysicsWorldData &world_data)
{
  return physics_attribute_vmutablearray(attribute, const_cast<PhysicsWorldData &>(world_data));
}

template<typename ElemT> ElemT DefaultValidateFn(const ElemT value)
{
  return value;
}

/**
 * Technically just adds validation to the setter.
 * However, this is NOT a CommonVArrayInfo::Type::Span, because doing so will cause some code to
 * take shortcuts. In particular VArray::fill will check for internal Span type and then bypass all
 * setters and use the CPPType to directly fill the internal span. This bypasses the validation and
 * leads to unvalidated data.
 */
template<typename T, CacheValidateFn<T> ValidateFn = DefaultValidateFn<T>>
class VArrayImpl_For_ValidatedSpan : public VMutableArrayImpl<T> {
 protected:
  T *data_ = nullptr;

 public:
  VArrayImpl_For_ValidatedSpan(const MutableSpan<T> data)
      : VMutableArrayImpl<T>(data.size()), data_(data.data())
  {
  }

  VArrayImpl_For_ValidatedSpan(const GMutableSpan data)
      : VMutableArrayImpl<T>(data.size()), data_(data.typed<T>().data())
  {
  }

 protected:
  VArrayImpl_For_ValidatedSpan(const int64_t size) : VMutableArrayImpl<T>(size) {}

  T get(const int64_t index) const override
  {
    return data_[index];
  }

  void set(const int64_t index, T value) override
  {
    data_[index] = ValidateFn(value);
  }

  void materialize(const IndexMask &mask, T *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i) { dst[i] = data_[i]; });
  }

  void materialize_to_uninitialized(const IndexMask &mask, T *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i) { new (dst + i) T(data_[i]); });
  }

  void materialize_compressed(const IndexMask &mask, T *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i, const int64_t pos) { dst[pos] = data_[i]; });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, T *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i, const int64_t pos) { new (dst + pos) T(data_[i]); });
  }
};

GVMutableArray physics_attribute_cache_vmutablearray(PhysicsBodyAttribute attribute,
                                                     GArray<> &data)
{
  using BodyAttribute = PhysicsBodyAttribute;
  using namespace physics_world_attribute_functions;

  switch (attribute) {
    case BodyAttribute::collision_shape:
      return VMutableArray<int>::For<VArrayImpl_For_ValidatedSpan<int>>(data);
    case BodyAttribute::motion_type:
      return VMutableArray<int>::For<VArrayImpl_For_ValidatedSpan<int, motion_type_validate_fn>>(
          data);
    case BodyAttribute::mass:
      return VMutableArray<float>::For<VArrayImpl_For_ValidatedSpan<float>>(data);
    case BodyAttribute::inertia:
      return VMutableArray<float4x4>::For<VArrayImpl_For_ValidatedSpan<float4x4>>(data);
    case BodyAttribute::position:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case BodyAttribute::rotation:
      return VMutableArray<math::Quaternion>::For<VArrayImpl_For_ValidatedSpan<math::Quaternion>>(
          data);
    case BodyAttribute::velocity:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case BodyAttribute::angular_velocity:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case BodyAttribute::is_active:
      return VMutableArray<bool>::For<VArrayImpl_For_ValidatedSpan<bool>>(data);
    case BodyAttribute::allow_sleep:
      return VMutableArray<bool>::For<VArrayImpl_For_ValidatedSpan<bool>>(data);
    case BodyAttribute::friction:
      return VMutableArray<float>::For<VArrayImpl_For_ValidatedSpan<float>>(data);
    case BodyAttribute::restitution:
      return VMutableArray<float>::For<VArrayImpl_For_ValidatedSpan<float>>(data);
    case BodyAttribute::linear_damping:
      return VMutableArray<float>::For<VArrayImpl_For_ValidatedSpan<float>>(data);
    case BodyAttribute::angular_damping:
      return VMutableArray<float>::For<VArrayImpl_For_ValidatedSpan<float>>(data);

    case BodyAttribute::total_force:
      return VMutableArray<float3>::For<
          VArrayImpl_For_ValidatedSpan<float3, total_force_validate_fn>>(data);
    case BodyAttribute::total_torque:
      return VMutableArray<float3>::For<
          VArrayImpl_For_ValidatedSpan<float3, total_torque_validate_fn>>(data);
  }
  BLI_assert_unreachable();
  return {};
}

GVMutableArray physics_attribute_cache_vmutablearray(PhysicsConstraintAttribute attribute,
                                                     GArray<> &data)
{
  using ConstraintAttribute = PhysicsConstraintAttribute;
  using namespace physics_world_attribute_functions;

  switch (attribute) {
    case ConstraintAttribute::type:
      return VMutableArray<int>::For<
          VArrayImpl_For_ValidatedSpan<int, constraint_type_validate_fn>>(data);
    case ConstraintAttribute::body1:
      return VMutableArray<int>::For<VArrayImpl_For_ValidatedSpan<int>>(data);
    case ConstraintAttribute::body2:
      return VMutableArray<int>::For<VArrayImpl_For_ValidatedSpan<int>>(data);
    case ConstraintAttribute::enabled:
      return VMutableArray<bool>::For<VArrayImpl_For_ValidatedSpan<bool>>(data);
    case ConstraintAttribute::frame1:
      return VMutableArray<float4x4>::For<VArrayImpl_For_ValidatedSpan<float4x4>>(data);
    case ConstraintAttribute::frame2:
      return VMutableArray<float4x4>::For<VArrayImpl_For_ValidatedSpan<float4x4>>(data);
    case ConstraintAttribute::limit_min_axis:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case ConstraintAttribute::limit_max_axis:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case ConstraintAttribute::limit_min_angle:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case ConstraintAttribute::limit_max_angle:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case ConstraintAttribute::spring_stiffness_axis:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case ConstraintAttribute::spring_stiffness_angle:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case ConstraintAttribute::spring_damping_axis:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case ConstraintAttribute::spring_damping_angle:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case ConstraintAttribute::max_friction_axis:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case ConstraintAttribute::max_friction_angle:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case ConstraintAttribute::motor_spring_stiffness_axis:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case ConstraintAttribute::motor_spring_stiffness_angle:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case ConstraintAttribute::motor_spring_damping_axis:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case ConstraintAttribute::motor_spring_damping_angle:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case ConstraintAttribute::min_motor_force_axis:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case ConstraintAttribute::min_motor_force_angle:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case ConstraintAttribute::max_motor_force_axis:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
    case ConstraintAttribute::max_motor_force_angle:
      return VMutableArray<float3>::For<VArrayImpl_For_ValidatedSpan<float3>>(data);
  }
  BLI_assert_unreachable();
  return {};
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Attribute Providers for World Data
 * \{ */

using namespace physics_world_attribute_functions;

PhysicsWorldBodyAttributeProvider::PhysicsWorldBodyAttributeProvider(
    const BodyAttribute attribute,
    const PhysicsWorldDataAccessInfo access_info,
    const UpdateOnChange update_on_change,
    const AttributeValidator validator)
    : BuiltinAttributeProvider(
          physics_attributes::physics_attribute_name(attribute),
          bke::AttrDomain::Point,
          cpp_type_to_custom_data_type(physics_attributes::physics_attribute_type(attribute)),
          NonDeletable,
          validator),
      attribute_(attribute),
      access_info_(access_info),
      update_on_change_(update_on_change)
{
}

GAttributeReader PhysicsWorldBodyAttributeProvider::try_get_for_read(const void *owner) const
{
  const PhysicsWorldData &world_data = *access_info_.get_const_world_data(owner);
  GVArray varray = physics_attribute_varray(attribute_, world_data);
  return {std::move(varray), domain_, nullptr};
}

GAttributeWriter PhysicsWorldBodyAttributeProvider::try_get_for_write(void *owner) const
{
  PhysicsWorldData &world_data = *access_info_.get_world_data(owner);
  GVMutableArray varray = physics_attribute_vmutablearray(attribute_, world_data);
  std::function<void()> tag_modified_fn = [attribute = attribute_,
                                           access_info = access_info_,
                                           update_on_change = update_on_change_,
                                           owner]() {
    if (update_on_change) {
      update_on_change(owner);
    }
  };
  return {std::move(varray), domain_, std::move(tag_modified_fn)};
}

bool PhysicsWorldBodyAttributeProvider::try_delete(void * /*owner*/) const
{
  return false;
}

bool PhysicsWorldBodyAttributeProvider::try_create(void * /*owner*/,
                                                   const AttributeInit & /*initializer*/) const
{
  return false;
}

bool PhysicsWorldBodyAttributeProvider::exists(const void * /*owner*/) const
{
  return true;
}

PhysicsWorldConstraintAttributeProvider::PhysicsWorldConstraintAttributeProvider(
    const ConstraintAttribute attribute,
    const PhysicsWorldDataAccessInfo access_info,
    const UpdateOnChange update_on_change,
    const AttributeValidator validator)
    : BuiltinAttributeProvider(
          physics_attributes::physics_attribute_name(attribute),
          bke::AttrDomain::Edge,
          cpp_type_to_custom_data_type(physics_attributes::physics_attribute_type(attribute)),
          NonDeletable,
          validator),
      attribute_(attribute),
      access_info_(access_info),
      update_on_change_(update_on_change)
{
}

GAttributeReader PhysicsWorldConstraintAttributeProvider::try_get_for_read(const void *owner) const
{
  const PhysicsWorldData &world_data = *access_info_.get_const_world_data(owner);
  GVArray varray = physics_attribute_varray(attribute_, world_data);
  return {std::move(varray), domain_, nullptr};
}

GAttributeWriter PhysicsWorldConstraintAttributeProvider::try_get_for_write(void *owner) const
{
  PhysicsWorldData &world_data = *access_info_.get_world_data(owner);
  GVMutableArray varray = physics_attribute_vmutablearray(attribute_, world_data);
  std::function<void()> tag_modified_fn = [attribute = attribute_,
                                           access_info = access_info_,
                                           update_on_change = update_on_change_,
                                           owner]() {
    if (update_on_change) {
      update_on_change(owner);
    }
  };
  return {std::move(varray), domain_, std::move(tag_modified_fn)};
}

bool PhysicsWorldConstraintAttributeProvider::try_delete(void * /*owner*/) const
{
  return false;
}

bool PhysicsWorldConstraintAttributeProvider::try_create(
    void * /*owner*/, const AttributeInit & /*initializer*/) const
{
  return false;
}

bool PhysicsWorldConstraintAttributeProvider::exists(const void * /*owner*/) const
{
  return true;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name World Data Attribute Accessor Functions
 * \{ */

static ComponentAttributeProviders create_attribute_providers()
{
  using namespace physics_attributes;
  using BodyAttribute = PhysicsBodyAttribute;
  using ConstraintAttribute = PhysicsConstraintAttribute;

  static PhysicsWorldDataAccessInfo world_data_access_info = {
      [](void *owner) -> PhysicsWorldData * { return static_cast<PhysicsWorldData *>(owner); },
      [](const void *owner) -> const PhysicsWorldData * {
        return static_cast<const PhysicsWorldData *>(owner);
      },
  };

  const Span<BodyAttribute> body_attributes = all_body_attributes();
  const Span<ConstraintAttribute> constraint_attributes = all_constraint_attributes();
  static Vector<PhysicsWorldBodyAttributeProvider> body_attribute_providers_data;
  static Vector<PhysicsWorldConstraintAttributeProvider> constraint_attribute_providers_data;
  body_attribute_providers_data.reserve(body_attributes.size());
  constraint_attribute_providers_data.reserve(constraint_attributes.size());
  for (const BodyAttribute attribute : body_attributes) {
    PhysicsWorldBodyAttributeProvider provider(attribute, world_data_access_info, {});
    body_attribute_providers_data.append(std::move(provider));
  }
  for (const ConstraintAttribute attribute : constraint_attributes) {
    PhysicsWorldConstraintAttributeProvider provider(attribute, world_data_access_info, {});
    constraint_attribute_providers_data.append(std::move(provider));
  }

  Array<const BuiltinAttributeProvider *> builtin_providers(
      body_attribute_providers_data.size() + constraint_attribute_providers_data.size());
  const IndexRange body_attribute_provider_range = body_attribute_providers_data.index_range();
  const IndexRange constraint_attribute_provider_range = body_attribute_provider_range.after(
      constraint_attribute_providers_data.size());
  for (const int i : body_attribute_providers_data.index_range()) {
    builtin_providers[body_attribute_provider_range[i]] = &body_attribute_providers_data[i];
  }
  for (const int i : constraint_attribute_provider_range.index_range()) {
    builtin_providers[constraint_attribute_provider_range[i]] =
        &constraint_attribute_providers_data[i];
  }

  return ComponentAttributeProviders(builtin_providers, {});
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

static AttributeAccessorFunctions get_accessor_functions()
{
  static const ComponentAttributeProviders providers = create_attribute_providers();
  AttributeAccessorFunctions fn =
      attribute_accessor_functions::accessor_functions_for_providers<providers>();
  fn.domain_size = [](const void *owner, const AttrDomain domain) {
    if (owner == nullptr) {
      return 0;
    }
    const PhysicsWorldData &world_data = *static_cast<const PhysicsWorldData *>(owner);
    switch (domain) {
      case AttrDomain::Point:
        return int(world_data.bodies().size());
      case AttrDomain::Edge:
        return int(world_data.constraints().size());
      default:
        return 0;
    }
  };
  fn.domain_supported = [](const void * /*owner*/, const AttrDomain domain) {
    return ELEM(domain, AttrDomain::Point, AttrDomain::Edge);
  };
  fn.adapt_domain = [](const void *owner,
                       const GVArray &varray,
                       const AttrDomain from_domain,
                       const AttrDomain to_domain) -> GVArray {
    if (owner == nullptr) {
      return {};
    }
    const PhysicsWorldState &impl = *static_cast<const PhysicsWorldState *>(owner);
    return adapt_physics_attribute_domain(impl, varray, from_domain, to_domain);
  };
  return fn;
}

static const AttributeAccessorFunctions &get_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_accessor_functions();
  return fn;
}

/** \} */

#endif

}  // namespace blender::bke
