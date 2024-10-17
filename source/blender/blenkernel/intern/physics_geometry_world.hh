/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#pragma once

#include <functional>

#include "BLI_array.hh"
#include "BLI_cache_mutex.hh"
#include "BLI_index_mask_fwd.hh"
#include "BLI_utility_mixins.hh"
#include "BLI_virtual_array_fwd.hh"

#include "BKE_collision_shape.hh"
#include "BKE_physics_geometry.hh"

#ifdef WITH_JOLT
#  include <Jolt/Jolt.h>

#  include <Jolt/Core/JobSystemThreadPool.h>
#  include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#  include <Jolt/Physics/PhysicsSystem.h>
#endif

namespace blender::bke {

// Layer that objects can be in, determines which other objects it can collide with
// Typically you at least want to have 1 layer for moving bodies and 1 layer for static bodies, but
// you can have more layers if you want. E.g. you could have a layer for high detail collision
// (which is not used by the physics simulation but only if you do collision testing).
namespace Layers {
static constexpr JPH::ObjectLayer non_moving = 0;
static constexpr JPH::ObjectLayer moving = 1;
static constexpr int num_layers = 2;
};  // namespace Layers

// Each broadphase layer results in a separate bounding volume tree in the broad phase. You at
// least want to have a layer for non-moving and moving objects to avoid having to update a tree
// full of static objects every frame. You can have a 1-on-1 mapping between object layers and
// broadphase layers (like in this case) but if you have many object layers you'll be creating many
// broad phase trees, which is not efficient. If you want to fine tune your broadphase layers
// define JPH_TRACK_BROADPHASE_STATS and look at the stats reported on the TTY.
namespace BroadPhaseLayers {
static constexpr JPH::BroadPhaseLayer non_moving(0);
static constexpr JPH::BroadPhaseLayer moving(1);
static constexpr int num_layers(2);
};  // namespace BroadPhaseLayers

// BroadPhaseLayerInterface implementation
// This defines a mapping between object and broadphase layers.
class BroadPhaseLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
 public:
  BroadPhaseLayerInterfaceImpl() {}

  virtual uint GetNumBroadPhaseLayers() const override
  {
    return BroadPhaseLayers::num_layers;
  }

  virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override
  {
    switch (inLayer) {
      case Layers::non_moving:
        return BroadPhaseLayers::non_moving;
      case Layers::moving:
        return BroadPhaseLayers::moving;
    }
    BLI_assert_unreachable();
    return BroadPhaseLayers::non_moving;
  }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
  virtual const char *GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override
  {
    switch ((BroadPhaseLayer::Type)inLayer) {
      case (BroadPhaseLayer::Type)BroadPhaseLayers::non_moving:
        return "non_moving";
      case (BroadPhaseLayer::Type)BroadPhaseLayers::moving:
        return "moving";
      default:
        BLI_assert_unreachable();
        return "invalid";
    }
  }
#endif  // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED
};

/// Class that determines if an object layer can collide with a broadphase layer
class ObjectVsBroadPhaseLayerFilterImpl : public JPH::ObjectVsBroadPhaseLayerFilter {
 public:
  virtual bool ShouldCollide(JPH::ObjectLayer inLayer1,
                             JPH::BroadPhaseLayer inLayer2) const override
  {
    switch (inLayer1) {
      case Layers::non_moving:
        return inLayer2 == BroadPhaseLayers::moving;
      case Layers::moving:
        return true;
      default:
        BLI_assert_unreachable();
        return false;
    }
  }
};

/// Class that determines if two object layers can collide
class ObjectLayerPairFilterImpl : public JPH::ObjectLayerPairFilter {
 public:
  virtual bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override
  {
    switch (inObject1) {
      case Layers::non_moving:
        return inObject2 == Layers::moving;  // Non moving only collides with moving
      case Layers::moving:
        return true;  // Moving collides with everything
      default:
        BLI_assert_unreachable();
        return false;
    }
  }
};

class JoltPhysicsWorldData : NonCopyable, NonMovable {
 public:
  using BodyAttribute = PhysicsBodyAttribute;
  using ConstraintAttribute = PhysicsConstraintAttribute;
  using BodyActivationState = PhysicsBodyActivationState;

 private:
  // Create mapping table from object layer to broadphase layer
  // Note: As this is an interface, PhysicsSystem will take a reference to this so this instance
  // needs to stay alive!
  BroadPhaseLayerInterfaceImpl broad_phase_layer_interface_;

  // Create class that filters object vs broadphase layers
  // Note: As this is an interface, PhysicsSystem will take a reference to this so this instance
  // needs to stay alive!
  ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter_;

  // Create class that filters object vs object layers
  // Note: As this is an interface, PhysicsSystem will take a reference to this so this instance
  // needs to stay alive!
  ObjectLayerPairFilterImpl object_vs_object_layer_filter_;

  // We need a temp allocator for temporary allocations during the physics update. We're
  // pre-allocating 10 MB to avoid having to do allocations during the physics update.
  // B.t.w. 10 MB is way too much for this example but it is a typical value you can use.
  // If you don't want to pre-allocate you can also use TempAllocatorMalloc to fall back to
  // malloc / free.
  JPH::TempAllocatorImpl temp_allocator_;

  // We need a job system that will execute physics jobs on multiple threads. Typically
  // you would implement the JobSystem interface yourself and let Jolt Physics run on top
  // of your own job scheduler. JobSystemThreadPool is an example implementation.
  JPH::JobSystemThreadPool job_system_;

  // Now we can create the actual physics system.
  JPH::PhysicsSystem physics_system_;

  Array<JPH::Body *> bodies_;
  Array<JPH::Constraint *> constraints_;

  mutable CacheMutex body_index_cache_;
  mutable CacheMutex constraint_index_cache_;
  mutable CacheMutex bodies_in_world_cache_;
  mutable CacheMutex constraints_in_world_cache_;

 public:
  JoltPhysicsWorldData();
  JoltPhysicsWorldData(int body_num, int constraint_num);
  ~JoltPhysicsWorldData();

  void resize(int body_num, int constraint_num);
  void resize(int body_num,
              int constraint_num,
              const IndexMask &src_body_mask,
              const IndexMask &src_constraint_mask,
              int dst_body_offset,
              int dst_constraint_offset);
  void move(const IndexMask &src_body_mask,
            const IndexMask &src_constraint_mask,
            int dst_body_offset,
            int dst_constraint_offset);

  void tag_bodies_in_world() const;
  void tag_constraints_in_world() const;

  void ensure_body_and_constraint_indices() const;
  void ensure_bodies_and_constraints_in_world();

  void init_constraints(const IndexMask &selection);
  void update_constraints(const IndexMask &selection, AttributeAccessor attributes);

  float3 gravity() const;
  void set_gravity(const float3 &gravity);

  void step_simulation(float delta_time, int collision_steps);

  void set_body_shapes(const IndexMask &selection,
                       const Span<InstanceReference> shapes,
                       const Span<int> shape_handles);

  void set_body_mass(const IndexMask &selection,
                     const Span<float> masses,
                     const Span<float4x4> inertias);

  void apply_force(const IndexMask &selection,
                   const VArray<float3> &forces,
                   const VArray<float3> &positions = {});
  void apply_torque(const IndexMask &selection, const VArray<float3> &torques);

  void apply_impulse(const IndexMask &selection,
                     const VArray<float3> &impulses,
                     const VArray<float3> &positions = {});
  void apply_angular_impulse(const IndexMask &selection, const VArray<float3> &angular_impulses);

  bke::AttributeAccessor attributes() const;
  bke::MutableAttributeAccessor attributes_for_write();

  /* For VArray access. */
  JPH::BodyInterface &body_interface();
  const JPH::BodyInterface &body_interface() const;
  Span<JPH::Body *> bodies() const;
  Span<JPH::Constraint *> constraints() const;

  bool validate(AttributeAccessor attributes, Span<InstanceReference> shapes);

 private:
  void setup();
  void shutdown();
};

class PhysicsWorldData : public JoltPhysicsWorldData {
  using JoltPhysicsWorldData::JoltPhysicsWorldData;
};

GVMutableArray physics_attribute_vmutablearray(PhysicsBodyAttribute attribute,
                                               PhysicsWorldData &world_data);
GVMutableArray physics_attribute_vmutablearray(PhysicsConstraintAttribute attribute,
                                               PhysicsWorldData &world_data);
GVArray physics_attribute_varray(PhysicsBodyAttribute attribute,
                                 const PhysicsWorldData &world_data);
GVArray physics_attribute_varray(PhysicsConstraintAttribute attribute,
                                 const PhysicsWorldData &world_data);

}  // namespace blender::bke
