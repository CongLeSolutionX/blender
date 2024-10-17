/* SPDX-FileCopyrightText: 2004-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#include <functional>
#include <mutex>

#include "BKE_attribute.hh"
#include "BKE_attribute_filter.hh"
#include "BKE_collision_shape.hh"
#include "BKE_customdata.hh"
#include "BKE_geometry_set.hh"
#include "BKE_physics_geometry.hh"
#include "BKE_type_conversions.hh"

#include "BLI_array_utils.hh"
#include "BLI_assert.h"
#include "BLI_cpp_type.hh"
#include "BLI_index_mask.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_utildefines.h"
#include "BLI_virtual_array.hh"

#include "DNA_customdata_types.h"
#include "physics_geometry_attributes.hh"
#include "physics_geometry_intern.hh"
#include "physics_geometry_world.hh"

#ifndef NDEBUG
#  include <iostream>
#endif

namespace blender::bke {

using namespace physics_attributes;

/* -------------------------------------------------------------------- */
/** \name Physics Geometry
 * \{ */

/* Similar to CacheMutex, but with supplied mutex and flag. */
static void ensure_cache(std::mutex &mutex,
                         std::atomic<bool> &flag,
                         const FunctionRef<void()> compute_cache)
{
  if (flag.load(std::memory_order_acquire)) {
    return;
  }
  std::scoped_lock lock{mutex};
  /* Double checked lock. */
  if (flag.load(std::memory_order_relaxed)) {
    return;
  }
  /* Use task isolation because a mutex is locked and the cache computation might use
   * multi-threading. */
  threading::isolate_task(compute_cache);

  flag.store(true, std::memory_order_release);
}

/* Same as ensure_cache but updates if any flag is dirty. */
[[maybe_unused]] static void ensure_cache_any(std::mutex &mutex,
                                              Span<std::atomic<bool> *> flags,
                                              Span<const FunctionRef<void()>> compute_caches)
{
  bool all_valid = true;
  for (const int i : flags.index_range()) {
    if (!flags[i]->load(std::memory_order_acquire)) {
      all_valid = false;
      break;
    }
  }
  if (all_valid) {
    return;
  }
  std::scoped_lock lock{mutex};
  /* Double checked lock. */
  for (const int i : flags.index_range()) {
    if (!flags[i]->load(std::memory_order_acquire)) {
      /* Use task isolation because a mutex is locked and the cache computation might use
       * multi-threading. */
      threading::isolate_task(compute_caches[i]);

      flags[i]->store(true, std::memory_order_release);
    }
  }
}

static void tag_cache_dirty(std::atomic<bool> &flag)
{
  flag.store(false);
}

static bool is_cache_dirty(const std::atomic<bool> &flag)
{
  return !flag.load(std::memory_order_relaxed);
}

PhysicsWorldState::PhysicsWorldState()
    : body_num_(0), constraint_num_(0), body_custom_data_({}), constraint_custom_data_({})
{
  CustomData_reset(&body_custom_data_);
  CustomData_reset(&constraint_custom_data_);
  this->tag_read_cache_changed();
}

PhysicsWorldState::PhysicsWorldState(int body_num, int constraint_num)
    : body_num_(body_num),
      constraint_num_(constraint_num),
      body_custom_data_({}),
      constraint_custom_data_({})
{
  CustomData_reset(&body_custom_data_);
  CustomData_reset(&constraint_custom_data_);
  CustomData_realloc(&body_custom_data_, 0, body_num);
  CustomData_realloc(&constraint_custom_data_, 0, constraint_num);
  this->tag_read_cache_changed();
}

PhysicsWorldState::PhysicsWorldState(const PhysicsWorldState &other)
{
  *this = other;
}

PhysicsWorldState::~PhysicsWorldState()
{
  CustomData_free(&body_custom_data_, body_num_);
  CustomData_free(&constraint_custom_data_, constraint_num_);

  /* World data is owned by the geometry when it's mutable (always the case on destruction). */
  delete world_data_;
}

PhysicsWorldState &PhysicsWorldState::operator=(const PhysicsWorldState &other)
{
  body_data_ = other.body_data_;
  constraint_data_ = other.constraint_data_;
  shapes_ = other.shapes_;
  shape_user_counts_ = other.shape_user_counts_;

  CustomData_reset(&body_custom_data_);
  CustomData_reset(&constraint_custom_data_);
  body_num_ = other.body_num_;
  constraint_num_ = other.constraint_num_;
  CustomData_init_from(&other.body_custom_data_, &body_custom_data_, CD_MASK_ALL, other.body_num_);
  CustomData_init_from(&other.constraint_custom_data_,
                       &constraint_custom_data_,
                       CD_MASK_ALL,
                       other.constraint_num_);

  if (other.world_data_) {
    try_move_data(other,
                  body_num_,
                  constraint_num_,
                  IndexRange(body_num_),
                  IndexRange(constraint_num_),
                  0,
                  0);
  }

  if (is_cache_dirty(other.read_cache_valid_)) {
    tag_cache_dirty(read_cache_valid_);
  }
  if (is_cache_dirty(other.body_collision_shapes_valid_)) {
    tag_cache_dirty(body_collision_shapes_valid_);
  }
  if (is_cache_dirty(other.body_mass_valid_)) {
    tag_cache_dirty(body_mass_valid_);
  }
  if (is_cache_dirty(other.constraints_valid_)) {
    tag_cache_dirty(constraints_valid_);
  }

  return *this;
}

void PhysicsWorldState::delete_self()
{
  delete this;
}

int PhysicsWorldState::bodies_num() const
{
  return body_num_;
}

int PhysicsWorldState::constraints_num() const
{
  return constraint_num_;
}

int PhysicsWorldState::shapes_num() const
{
  return shapes_.size();
}

IndexRange PhysicsWorldState::bodies_range() const
{
  return IndexRange(body_num_);
}

IndexRange PhysicsWorldState::constraints_range() const
{
  return IndexRange(constraint_num_);
}

IndexRange PhysicsWorldState::shapes_range() const
{
  return shapes_.index_range();
}

bool PhysicsWorldState::has_world_data() const
{
  return world_data_ != nullptr;
}

void PhysicsWorldState::tag_read_cache_changed()
{
  /* Cache only becomes invalid if there is world data that can change. */
  if (world_data_ != nullptr) {
    tag_cache_dirty(read_cache_valid_);
  }
}

void PhysicsWorldState::tag_body_topology_changed()
{
  // this->tag_constraint_disable_collision_changed();
}

void PhysicsWorldState::tag_body_collision_shape_changed()
{
  tag_cache_dirty(body_collision_shapes_valid_);
}

void PhysicsWorldState::tag_body_mass_changed()
{
  tag_cache_dirty(body_mass_valid_);
}

void PhysicsWorldState::tag_constraints_changed()
{
  tag_cache_dirty(constraints_valid_);
}

// void PhysicsWorldState::tag_constraint_disable_collision_changed()
//{
//   tag_cache_dirty(constraint_disable_collision_valid_);
// }

void PhysicsWorldState::tag_shapes_changed()
{
  this->shape_user_counts_.tag_dirty();
  this->tag_body_collision_shape_changed();
}

bool PhysicsWorldState::has_builtin_attribute_cache(
    PhysicsWorldState::BodyAttribute attribute) const
{
  return this->body_data_.contains(attribute);
}

bool PhysicsWorldState::has_builtin_attribute_cache(
    PhysicsWorldState::ConstraintAttribute attribute) const
{
  return this->constraint_data_.contains(attribute);
}

void PhysicsWorldState::ensure_read_cache() const
{
  ensure_cache(world_data_mutex_, read_cache_valid_, [&]() { this->ensure_read_cache_no_lock(); });
}

void PhysicsWorldState::ensure_read_cache_no_lock() const
{
  using BodyAttribute = PhysicsBodyAttribute;
  using ConstraintAttribute = PhysicsConstraintAttribute;

  PhysicsWorldState &dst = *const_cast<PhysicsWorldState *>(this);

  /* For inactive states just create default attributes. */
  if (world_data_ == nullptr) {
    for (const BodyAttribute attribute : all_body_attributes()) {
      dst.ensure_attribute_cache(attribute);
    }
    for (const ConstraintAttribute attribute : all_constraint_attributes()) {
      dst.ensure_attribute_cache(attribute);
    }
    return;
  }

  /* Some attributes require other updates before valid world data can be read. */
  dst.ensure_bodies_no_lock();
  dst.ensure_constraints_no_lock();

  /* Write to cache attributes. */
  MutableAttributeAccessor dst_attributes = dst.state_attributes_for_write();

  /* Read from world data and ignore the cache.
   * Important! This also prevents deadlock caused by re-entering this function. */
  const AttributeAccessor src_attributes = this->world_data_attributes();
  Set<std::string> local_body_attribute_names, local_constraint_attribute_names,
      local_shape_attribute_names;
  for (const BodyAttribute attribute : all_body_attributes()) {
    if (physics_attribute_use_write_cache(attribute)) {
      local_body_attribute_names.add_new(physics_attribute_name(attribute));
    }
  }
  for (const ConstraintAttribute attribute : all_constraint_attributes()) {
    if (physics_attribute_use_write_cache(attribute)) {
      local_constraint_attribute_names.add_new(physics_attribute_name(attribute));
    }
  }
  /* Only use builtin attributes, dynamic attributes are already in custom data. */
  src_attributes.foreach_attribute([&](const AttributeIter &iter) {
    if (!src_attributes.is_builtin(iter.name)) {
      local_body_attribute_names.add_new(iter.name);
      local_constraint_attribute_names.add_new(iter.name);
    }
  });

  gather_attributes(src_attributes,
                    bke::AttrDomain::Point,
                    bke::AttrDomain::Point,
                    attribute_filter_from_skip_ref(local_body_attribute_names),
                    IndexRange(body_num_),
                    dst_attributes);
  gather_attributes(src_attributes,
                    bke::AttrDomain::Edge,
                    bke::AttrDomain::Edge,
                    attribute_filter_from_skip_ref(local_constraint_attribute_names),
                    IndexRange(constraint_num_),
                    dst_attributes);
}

void PhysicsWorldState::ensure_bodies()
{
  ensure_cache(
      world_data_mutex_, body_collision_shapes_valid_, [&]() { this->ensure_bodies_no_lock(); });
}

void PhysicsWorldState::ensure_bodies_no_lock()
{
  this->ensure_body_collision_shapes_no_lock();
  this->ensure_body_mass_no_lock();
}

void PhysicsWorldState::ensure_body_collision_shapes_no_lock()
{
  const static StringRef collision_shape_id = physics_attribute_name(
      PhysicsBodyAttribute::collision_shape);

  if (!is_cache_dirty(body_collision_shapes_valid_)) {
    return;
  }
  if (world_data_ == nullptr) {
    return;
  }

  AttributeAccessor cached_attributes = this->state_attributes();
  const VArraySpan<int> body_shapes = *cached_attributes.lookup_or_default(
      collision_shape_id, AttrDomain::Point, -1);

  const IndexMask selection = world_data_->bodies().index_range();
  world_data_->set_body_shapes(selection, shapes_, body_shapes);
}

void PhysicsWorldState::ensure_body_mass_no_lock()
{
  if (!is_cache_dirty(body_mass_valid_)) {
    return;
  }
  if (world_data_ == nullptr) {
    return;
  }

  AttributeAccessor cached_attributes = this->state_attributes();
  const VArraySpan<float> masses = physics_attributes::physics_attribute_lookup_or_default<float>(
      cached_attributes, BodyAttribute::mass);
  const VArraySpan<float4x4> inertias =
      physics_attributes::physics_attribute_lookup_or_default<float4x4>(cached_attributes,
                                                                        BodyAttribute::inertia);

  const IndexMask selection = world_data_->bodies().index_range();
  world_data_->set_body_mass(selection, masses, inertias);
}

void PhysicsWorldState::ensure_constraints()
{
  ensure_cache(
      world_data_mutex_, constraints_valid_, [&]() { this->ensure_constraints_no_lock(); });
}

void PhysicsWorldState::ensure_constraints_no_lock()
{
  if (!is_cache_dirty(constraints_valid_)) {
    return;
  }
  if (world_data_ == nullptr) {
    return;
  }

  AttributeAccessor custom_data_attributes = this->state_attributes();
  const IndexMask selection = world_data_->constraints().index_range();
  world_data_->update_constraints(selection, custom_data_attributes);
}

// void PhysicsWorldState::ensure_constraint_disable_collision()
//{
//  ensure_cache(world_data_mutex_, constraint_disable_collision_valid_, [&]() {
//    this->ensure_constraint_disable_collision_no_lock();
//  });
//}

// void PhysicsWorldState::ensure_constraint_disable_collision_no_lock()
//{
//   const static StringRef disable_collision_id = physics_attribute_name(
//       PhysicsConstraintAttribute::disable_collision);
//
//   if (!is_cache_dirty(constraint_disable_collision_valid_)) {
//     return;
//   }
//   if (world_data_ == nullptr) {
//     return;
//   }
//
//   AttributeAccessor custom_data_attributes = this->custom_data_attributes();
//   const VArray<bool> disable_collision = *custom_data_attributes.lookup_or_default<bool>(
//       disable_collision_id, AttrDomain::Edge, false);
//
//   const IndexMask selection = world_data_->constraints().index_range();
//   world_data_->set_disable_collision(selection, disable_collision);
// }

void PhysicsWorldState::ensure_attribute_cache(PhysicsWorldState::BodyAttribute attribute)
{
  if (body_data_.contains(attribute)) {
    return;
  }

  const CPPType &type = physics_attribute_type(attribute);
  GArray<> data(type, this->bodies_num());
  type.fill_construct_n(physics_attribute_default_value(attribute), data.data(), data.size());
  body_data_.add(attribute, std::move(data));
}

void PhysicsWorldState::ensure_attribute_cache(PhysicsWorldState::ConstraintAttribute attribute)
{
  if (constraint_data_.contains(attribute)) {
    return;
  }

  const CPPType &type = physics_attribute_type(attribute);
  GArray<> data(type, this->constraints_num());
  type.fill_construct_n(physics_attribute_default_value(attribute), data.data(), data.size());
  constraint_data_.add(attribute, std::move(data));
}

void PhysicsWorldState::remove_attribute_caches()
{
  /* Force use of cache for writing. */
  MutableAttributeAccessor attributes = this->state_attributes_for_write();
  attributes.foreach_attribute([&](const AttributeIter &iter) {
    CustomData *custom_data = nullptr;
    int totelem = 0;
    switch (iter.domain) {
      case AttrDomain::Point:
        custom_data = &body_custom_data_;
        totelem = body_num_;
        break;
      case AttrDomain::Edge:
        custom_data = &constraint_custom_data_;
        totelem = constraint_num_;
        break;
      default:
        BLI_assert_unreachable();
        break;
    }
    if (custom_data != nullptr) {
      CustomData_free_layer_named(custom_data, iter.name, totelem);
    }
  });
}

void PhysicsWorldState::create_world()
{
  /* Avoid locking later on. */
  this->ensure_read_cache();

  if (world_data_) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_) {
    return;
  }

  /* Read from custom data, write to world data. */
  const AttributeAccessor src_attributes = this->state_attributes();

  const IndexRange body_range = IndexRange(body_num_);
  const IndexRange constraint_range = IndexRange(constraint_num_);

  const VArraySpan<int> body_shapes = *src_attributes.lookup_or_default(
      physics_attribute_name(BodyAttribute::collision_shape), AttrDomain::Point, -1);
  world_data_ = new PhysicsWorldData(body_num_, constraint_num_);
  world_data_->set_body_shapes(body_range, shapes_, body_shapes);
  world_data_->update_constraints(constraint_range, src_attributes);
}

void PhysicsWorldState::destroy_world()
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }
  this->ensure_read_cache();
  delete world_data_;
  world_data_ = nullptr;
}

static void remap_bodies(const int src_bodies_num,
                         const IndexMask &bodies_mask,
                         const IndexMask &constraints_mask,
                         const Span<int> src_constraint_types,
                         const Span<int> src_constraint_body1,
                         const Span<int> src_constraint_body2,
                         MutableSpan<int> dst_constraint_types,
                         MutableSpan<int> dst_constraint_body1,
                         MutableSpan<int> dst_constraint_body2)
{
  Array<int> map(src_bodies_num);
  const IndexRange body_range = map.index_range();
  index_mask::build_reverse_map<int>(bodies_mask, map);
  constraints_mask.foreach_index(GrainSize(512), [&](const int64_t src_i, const int64_t dst_i) {
    dst_constraint_types[dst_i] = src_constraint_types[src_i];

    const int body1 = src_constraint_body1[src_i];
    const int body2 = src_constraint_body2[src_i];
    dst_constraint_body1[dst_i] = body_range.contains(body1) ? map[body1] : -1;
    dst_constraint_body2[dst_i] = body_range.contains(body2) ? map[body2] : -1;
  });
}

void PhysicsWorldState::move_or_copy_selection(const PhysicsWorldState &src,
                                               const IndexMask &src_body_mask,
                                               const IndexMask &src_constraint_mask,
                                               const AttributeFilter &attribute_filter)
{
  this->try_move_data(src, body_num_, constraint_num_, src_body_mask, src_constraint_mask, 0, 0);

  const AttributeAccessor src_attributes = src.attributes();
  MutableAttributeAccessor dst_attributes = this->attributes_for_write();

  Set<std::string> ignored_attributes = {};
  if (world_data_) {
    /* Don't copy builtin attributes when there is world data. */
    ignored_attributes.add_multiple(all_body_attribute_names());
    ignored_attributes.add_multiple(all_constraint_attribute_names());
  }
  else {
    ignored_attributes.add_multiple({physics_attribute_name(PhysicsConstraintAttribute::type),
                                     physics_attribute_name(PhysicsConstraintAttribute::body1),
                                     physics_attribute_name(PhysicsConstraintAttribute::body2)});

    const VArraySpan<int> src_types = physics_attribute_lookup_or_default<int>(
        src_attributes, ConstraintAttribute::type);
    ;
    const VArraySpan<int> src_body1 = physics_attribute_lookup_or_default<int>(
        src_attributes, ConstraintAttribute::body1);
    ;
    const VArraySpan<int> src_body2 = physics_attribute_lookup_or_default<int>(
        src_attributes, ConstraintAttribute::body2);
    ;

    SpanAttributeWriter<int> dst_types = physics_attribute_lookup_for_write_only_span<int>(
        dst_attributes, ConstraintAttribute::type);
    SpanAttributeWriter<int> dst_body1 = physics_attribute_lookup_for_write_only_span<int>(
        dst_attributes, ConstraintAttribute::body1);
    SpanAttributeWriter<int> dst_body2 = physics_attribute_lookup_for_write_only_span<int>(
        dst_attributes, ConstraintAttribute::body2);

    remap_bodies(src.body_num_,
                 src_body_mask,
                 src_constraint_mask,
                 src_types,
                 src_body1,
                 src_body2,
                 dst_types.span,
                 dst_body1.span,
                 dst_body2.span);
  }

  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Point,
                         bke::AttrDomain::Point,
                         bke::attribute_filter_with_skip_ref(attribute_filter, ignored_attributes),
                         src_body_mask,
                         dst_attributes);
  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Edge,
                         bke::AttrDomain::Edge,
                         bke::attribute_filter_with_skip_ref(attribute_filter, ignored_attributes),
                         src_constraint_mask,
                         dst_attributes);
}

bool PhysicsWorldState::try_move_data(const PhysicsWorldState &src,
                                      const int body_num,
                                      const int constraint_num,
                                      const IndexMask &src_body_mask,
                                      const IndexMask &src_constraint_mask,
                                      int dst_body_offset,
                                      int dst_constraint_offset)
{
  BLI_assert(this->is_mutable());

  {
    if (src.world_data_ == nullptr) {
      return false;
    }
    std::scoped_lock src_lock(src.world_data_mutex_);
    if (src.world_data_ == nullptr) {
      return false;
    }

    /* Ensure valid read cache before moving data. */
    src.ensure_read_cache_no_lock();

    std::scoped_lock dst_lock(world_data_mutex_);
    if (world_data_) {
      delete world_data_;
      world_data_ = nullptr;
    }

    world_data_ = src.world_data_;
    src.world_data_ = nullptr;

    world_data_->resize(body_num,
                        constraint_num,
                        src_body_mask,
                        src_constraint_mask,
                        dst_body_offset,
                        dst_constraint_offset);
  }

  /* Copy attributes write caches. */
  const bke::AttributeFilterFromFunc attribute_filter = bke::AttributeFilterFromFunc(
      [&](const StringRef name) {
        const std::optional<PhysicsBodyAttribute> body_attribute = find_body_attribute(name);
        const std::optional<PhysicsConstraintAttribute> constraint_attribute =
            find_constraint_attribute(name);
        const bool is_cached_body_attribute = body_attribute &&
                                              physics_attribute_use_write_cache(*body_attribute);
        const bool is_cached_constraint_attribute = constraint_attribute &&
                                                    physics_attribute_use_write_cache(
                                                        *constraint_attribute);
        return (is_cached_body_attribute || is_cached_constraint_attribute) ?
                   bke::AttributeFilter::Result::Process :
                   bke::AttributeFilter::Result::AllowSkip;
      });

  bke::gather_attributes(src.state_attributes(),
                         bke::AttrDomain::Point,
                         bke::AttrDomain::Point,
                         attribute_filter,
                         src_body_mask,
                         dst_body_offset,
                         this->state_attributes_for_write());
  bke::gather_attributes(src.state_attributes(),
                         bke::AttrDomain::Edge,
                         bke::AttrDomain::Edge,
                         attribute_filter,
                         src_constraint_mask,
                         dst_constraint_offset,
                         this->state_attributes_for_write());
  return true;
}

int PhysicsWorldState::add_shape(const InstanceReference &reference)
{
  if (std::optional<int> handle = this->find_shape_handle(reference)) {
    return *handle;
  }
  return this->add_new_shape(reference);
}

int PhysicsWorldState::add_new_shape(const InstanceReference &reference)
{
  this->tag_shapes_changed();
  return shapes_.append_and_get_index(reference);
}

std::optional<int> PhysicsWorldState::find_shape_handle(const InstanceReference &query)
{
  for (const int i : shapes_.index_range()) {
    const InstanceReference &reference = shapes_[i];
    if (reference == query) {
      return i;
    }
  }
  return std::nullopt;
}

Span<InstanceReference> PhysicsWorldState::shapes() const
{
  return shapes_;
}

void PhysicsWorldState::remove_unused_shapes()
{
  const int tot_bodies = bodies_num();
  const int tot_references_before = shapes_.size();

  if (tot_bodies == 0) {
    /* If there are no instances, no reference is needed. */
    shapes_.clear();
    return;
  }
  if (tot_references_before == 1) {
    /* There is only one reference and at least one instance. So the only existing reference is
     * used. Nothing to do here. */
    return;
  }

  const Span<int> reference_handles =
      body_data_.lookup_default(PhysicsBodyAttribute::collision_shape, {}).as_span().typed<int>();

  Array<bool> usage_by_handle(tot_references_before, false);
  std::mutex mutex;

  /* Loop over all instances to see which references are used. */
  threading::parallel_for(reference_handles.index_range(), 1024, [&](IndexRange range) {
    /* Use local counter to avoid lock contention. */
    Array<bool> local_usage_by_handle(tot_references_before, false);

    for (const int i : range) {
      const int handle = reference_handles[i];
      BLI_assert(handle >= 0 && handle < tot_references_before);
      local_usage_by_handle[handle] = true;
    }

    std::lock_guard lock{mutex};
    for (const int i : IndexRange(tot_references_before)) {
      usage_by_handle[i] |= local_usage_by_handle[i];
    }
  });

  if (!usage_by_handle.as_span().contains(false)) {
    /* All references are used. */
    return;
  }

  /* Create new references and a mapping for the handles. */
  Vector<int> handle_mapping;
  Vector<InstanceReference> new_references;
  int next_new_handle = 0;
  bool handles_have_to_be_updated = false;
  for (const int old_handle : IndexRange(tot_references_before)) {
    if (!usage_by_handle[old_handle]) {
      /* Add some dummy value. It won't be read again. */
      handle_mapping.append(-1);
    }
    else {
      const InstanceReference &reference = shapes_[old_handle];
      handle_mapping.append(next_new_handle);
      new_references.append(reference);
      if (old_handle != next_new_handle) {
        handles_have_to_be_updated = true;
      }
      next_new_handle++;
    }
  }
  shapes_ = new_references;

  if (!handles_have_to_be_updated) {
    /* All remaining handles are the same as before, so they don't have to be updated. This happens
     * when unused handles are only at the end. */
    return;
  }

  /* Update handles of instances. */
  {
    const MutableSpan<int> reference_handles = this->body_data_
                                                   .lookup_or_add(
                                                       PhysicsBodyAttribute::collision_shape, {})
                                                   .as_mutable_span()
                                                   .typed<int>();
    threading::parallel_for(reference_handles.index_range(), 1024, [&](IndexRange range) {
      for (const int i : range) {
        reference_handles[i] = handle_mapping[reference_handles[i]];
      }
    });
  }
}

Span<int> PhysicsWorldState::shape_user_counts() const
{
  shape_user_counts_.ensure([&](Array<int> &r_data) {
    const int references_num = shapes_.size();
    r_data.reinitialize(references_num);
    r_data.fill(0);

    const VArraySpan<int> handles = physics_attributes::physics_attribute_lookup_or_default<int>(
        attributes(), PhysicsBodyAttribute::collision_shape);
    for (const int handle : handles) {
      if (handle >= 0 && handle < references_num) {
        r_data[handle]++;
      }
    }
  });
  return shape_user_counts_.data();
}

void PhysicsWorldState::ensure_shape_geometry_instances()
{
  shapes_ = ensure_geometry_instances_span(shapes_);
}

GeometrySet &PhysicsWorldState::geometry_set_from_shape(const int reference_index)
{
  /* If this assert fails, it means #ensure_geometry_instances must be called first or that the
   * reference can't be converted to a geometry set. */
  BLI_assert(shapes_[reference_index].type() == InstanceReference::Type::GeometrySet);

  return shapes_[reference_index].geometry_set();
}

void PhysicsWorldState::set_overlap_filter(OverlapFilterFn /*fn*/)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }

  // world_data_->set_overlap_filter(fn);
}

void PhysicsWorldState::clear_overlap_filter()
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }

  // world_data_->clear_overlap_filter();
}

float3 PhysicsWorldState::gravity() const
{
  /* TODO add output caches for single values too, to avoid locking. */
  if (world_data_ == nullptr) {
    return float3(0.0f);
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return float3(0.0f);
  }

  return world_data_->gravity();
}

void PhysicsWorldState::set_gravity(const float3 &gravity)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }

  world_data_->set_gravity(gravity);
}

void PhysicsWorldState::set_solver_iterations(const int /*num_solver_iterations*/)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }

  // world_data_->set_solver_iterations(num_solver_iterations);
}

void PhysicsWorldState::set_split_impulse(const bool /*split_impulse*/)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }

  // world_data_->set_split_impulse(split_impulse);
}

void PhysicsWorldState::step_simulation(float delta_time, int collision_steps)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }

  this->ensure_bodies_no_lock();
  this->ensure_constraints_no_lock();
  world_data_->step_simulation(delta_time, collision_steps);

  this->tag_read_cache_changed();
}

void PhysicsWorldState::apply_force(const IndexMask &selection,
                                    const VArray<float3> &forces,
                                    const VArray<float3> &relative_positions)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }
  world_data_->apply_force(selection, forces, relative_positions);

  this->tag_read_cache_changed();
}

void PhysicsWorldState::apply_torque(const IndexMask &selection, const VArray<float3> &torques)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }
  world_data_->apply_torque(selection, torques);

  this->tag_read_cache_changed();
}

void PhysicsWorldState::apply_impulse(const IndexMask &selection,
                                      const VArray<float3> &impulses,
                                      const VArray<float3> &relative_positions)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }
  world_data_->apply_impulse(selection, impulses, relative_positions);

  this->tag_read_cache_changed();
}

void PhysicsWorldState::apply_angular_impulse(const IndexMask &selection,
                                              const VArray<float3> &angular_impulses)
{
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }
  world_data_->apply_angular_impulse(selection, angular_impulses);

  this->tag_read_cache_changed();
}

void PhysicsWorldState::clear_forces(const IndexMask & /*selection*/)
{
  BLI_assert_unreachable();
  if (world_data_ == nullptr) {
    return;
  }
  std::scoped_lock lock(world_data_mutex_);
  if (world_data_ == nullptr) {
    return;
  }
  // world_data_->clear_forces(selection);

  this->tag_read_cache_changed();
}

bool PhysicsWorldState::validate_world_data()
{
  bool ok = true;

  if (world_data_ == nullptr) {
    return ok;
  }

  this->ensure_bodies();
  this->ensure_constraints();
  this->ensure_read_cache();

  return world_data_->validate(state_attributes(), shapes_);
}

AttributeAccessor PhysicsWorldState::attributes() const
{
  return AttributeAccessor(this, bke::get_physics_accessor_functions_ref());
}

MutableAttributeAccessor PhysicsWorldState::attributes_for_write()
{
  return MutableAttributeAccessor(this, bke::get_physics_accessor_functions_ref());
}

AttributeAccessor PhysicsWorldState::state_attributes() const
{
  return AttributeAccessor(this, bke::get_physics_state_accessor_functions_ref());
}

MutableAttributeAccessor PhysicsWorldState::state_attributes_for_write()
{
  return MutableAttributeAccessor(this, bke::get_physics_state_accessor_functions_ref());
}

AttributeAccessor PhysicsWorldState::world_data_attributes() const
{
  return AttributeAccessor(this, bke::get_physics_world_data_accessor_functions_ref());
}

MutableAttributeAccessor PhysicsWorldState::world_data_attributes_for_write()
{
  return MutableAttributeAccessor(this, bke::get_physics_world_data_accessor_functions_ref());
}

const PhysicsWorldDataAccessInfo &PhysicsWorldState::world_data_access_info()
{
  static PhysicsWorldDataAccessInfo world_data_access = {
      [](void *owner) -> PhysicsWorldData * {
        auto &state = *static_cast<PhysicsWorldState *>(owner);
        return state.world_data_;
      },
      [](const void *owner) -> const PhysicsWorldData * {
        const auto &state = *static_cast<const PhysicsWorldState *>(owner);
        return state.world_data_;
      }};
  return world_data_access;
}

const CustomDataAccessInfo &PhysicsWorldState::body_custom_data_access_info()
{
  static CustomDataAccessInfo body_custom_data_access = {
      [](void *owner) -> CustomData * {
        auto &state = *static_cast<PhysicsWorldState *>(owner);
        return &state.body_custom_data_;
      },
      [](const void *owner) -> const CustomData * {
        const auto &state = static_cast<const PhysicsWorldState *>(owner);
        return &state->body_custom_data_;
      },
      [](const void *owner) -> int {
        const auto &state = static_cast<const PhysicsWorldState *>(owner);
        return state->body_num_;
      }};
  return body_custom_data_access;
}

const CustomDataAccessInfo &PhysicsWorldState::constraint_custom_data_access_info()
{
  static CustomDataAccessInfo constraint_custom_data_access = {
      [](void *owner) -> CustomData * {
        auto &state = *static_cast<PhysicsWorldState *>(owner);
        return &state.constraint_custom_data_;
      },
      [](const void *owner) -> const CustomData * {
        const auto &state = static_cast<const PhysicsWorldState *>(owner);
        return &state->constraint_custom_data_;
      },
      [](const void *owner) -> int {
        const auto &state = static_cast<const PhysicsWorldState *>(owner);
        return state->constraint_num_;
      }};
  return constraint_custom_data_access;
}

#ifndef NDEBUG
void PhysicsWorldState::debug_print_status(const StringRef name) const
{
  std::cout << name << "(" << (void *)this << ") | " << this->strong_users() << " users | "
            << (this->has_world_data() ? "WORLD " : "cached") << " | "
            << (this->read_cache_valid_ ? "valid  " : "INVALID") << std::endl;
}

void PhysicsWorldState::debug_print_enter(const StringRef name) const
{
  std::cout << ">>   ";
  debug_print_status(name);
}

void PhysicsWorldState::debug_print_exit(const StringRef name) const
{
  constexpr int max_elements = 3;

  std::cout << "<<<< ";
  debug_print_status(name);

  if (world_data_) {
    const VArray<float3> positions = physics_attribute_lookup_or_default<float3>(
        this->world_data_attributes(), PhysicsBodyAttribute::position);
    std::cout << "  world data positions [" << positions.size() << "]:" << std::endl;
    for (const int i : IndexRange(std::min(max_elements, int(positions.size())))) {
      std::cout << "    " << positions[i] << std::endl;
    }
    if (positions.size() > max_elements) {
      std::cout << "    ..." << std::endl;
    }
  }

  {
    const Span<float3> positions = body_data_
                                       .lookup_default(PhysicsBodyAttribute::position,
                                                       GArray<>(CPPType::get<float3>()))
                                       .as_span()
                                       .typed<float3>();
    std::cout << "  read cache positions [" << positions.size() << "]:" << std::endl;
    for (const int i : IndexRange(std::min(max_elements, int(positions.size())))) {
      std::cout << "    " << positions[i] << std::endl;
    }
    if (positions.size() > max_elements) {
      std::cout << "    ..." << std::endl;
    }
  }
}
#endif

template<typename T>
VArray<T> PhysicsGeometry::lookup_attribute(const PhysicsBodyAttribute attribute) const
{
  return *attributes().lookup_or_default<int>(physics_attribute_name(attribute),
                                              AttrDomain::Point,
                                              physics_attribute_default_value<int>(attribute));
}

PhysicsGeometry::PhysicsGeometry()
{
  world_state_ = new PhysicsWorldState();
}

PhysicsGeometry::PhysicsGeometry(int bodies_num, int constraints_num)
{
  world_state_ = new PhysicsWorldState(bodies_num, constraints_num);
  this->state_for_write().tag_body_topology_changed();
}

PhysicsGeometry::PhysicsGeometry(const PhysicsGeometry &other)
{
  world_state_ = other.world_state_;
  world_state_->add_user();
}

PhysicsGeometry::~PhysicsGeometry()
{
  BLI_assert(world_state_ && world_state_->strong_users() > 0);
  world_state_->remove_user_and_delete_if_last();
}

const PhysicsWorldState &PhysicsGeometry::state() const
{
  return *world_state_;
}

PhysicsWorldState &PhysicsGeometry::state_for_write()
{
  if (world_state_->is_mutable()) {
    return *const_cast<PhysicsWorldState *>(world_state_);
  }

  PhysicsWorldState *new_state = new PhysicsWorldState(*world_state_);

  world_state_->remove_user_and_delete_if_last();
  world_state_ = new_state;

  return *const_cast<PhysicsWorldState *>(world_state_);
}

int PhysicsGeometry::bodies_num() const
{
  return world_state_->bodies_num();
}

int PhysicsGeometry::constraints_num() const
{
  return world_state_->constraints_num();
}

int PhysicsGeometry::shapes_num() const
{
  return world_state_->shapes_num();
}

IndexRange PhysicsGeometry::bodies_range() const
{
  return world_state_->bodies_range();
}

IndexRange PhysicsGeometry::constraints_range() const
{
  return world_state_->constraints_range();
}

IndexRange PhysicsGeometry::shapes_range() const
{
  return world_state_->shapes_range();
}

VArray<int> PhysicsGeometry::body_shapes() const
{
  return *attributes().lookup_or_default<int>(
      physics_attribute_name(BodyAttribute::collision_shape), AttrDomain::Point, -1);
}

AttributeWriter<int> PhysicsGeometry::body_shapes_for_write()
{
  const GVArray init_varray = VArray<int>::ForSingle(-1, this->state().bodies_num());
  return attributes_for_write().lookup_or_add_for_write<int>(
      physics_attribute_name(BodyAttribute::collision_shape),
      AttrDomain::Point,
      AttributeInitVArray(init_varray));
}

VArray<int> PhysicsGeometry::body_motion_types() const
{
  return lookup_attribute<int>(BodyAttribute::motion_type);
}

AttributeWriter<int> PhysicsGeometry::body_motion_types_for_write()
{
  return attributes_for_write().lookup_or_add_for_write<int>(
      physics_attribute_name(BodyAttribute::motion_type), AttrDomain::Point);
}

VArray<float> PhysicsGeometry::body_masses() const
{
  return *attributes().lookup<float>(physics_attribute_name(BodyAttribute::mass));
}

AttributeWriter<float> PhysicsGeometry::body_masses_for_write()
{
  return attributes_for_write().lookup_or_add_for_write<float>(
      physics_attribute_name(BodyAttribute::mass), AttrDomain::Point);
}

VArray<float4x4> PhysicsGeometry::body_inertias() const
{
  return *attributes().lookup<float4x4>(physics_attribute_name(BodyAttribute::inertia));
}

AttributeWriter<float4x4> PhysicsGeometry::body_inertias_for_write()
{
  return attributes_for_write().lookup_for_write<float4x4>(
      physics_attribute_name(BodyAttribute::inertia));
}

VArray<float3> PhysicsGeometry::body_positions() const
{
  return *attributes().lookup<float3>(physics_attribute_name(BodyAttribute::position));
}

AttributeWriter<float3> PhysicsGeometry::body_positions_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(
      physics_attribute_name(BodyAttribute::position));
}

VArray<math::Quaternion> PhysicsGeometry::body_rotations() const
{
  return *attributes().lookup<math::Quaternion>(physics_attribute_name(BodyAttribute::rotation));
}

AttributeWriter<math::Quaternion> PhysicsGeometry::body_rotations_for_write()
{
  return attributes_for_write().lookup_for_write<math::Quaternion>(
      physics_attribute_name(BodyAttribute::rotation));
}

VArray<float3> PhysicsGeometry::body_velocities() const
{
  return *attributes().lookup<float3>(physics_attribute_name(BodyAttribute::velocity));
}

AttributeWriter<float3> PhysicsGeometry::body_velocities_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(
      physics_attribute_name(BodyAttribute::velocity));
}

VArray<float3> PhysicsGeometry::body_angular_velocities() const
{
  return *attributes().lookup<float3>(physics_attribute_name(BodyAttribute::angular_velocity));
}

AttributeWriter<float3> PhysicsGeometry::body_angular_velocities_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(
      physics_attribute_name(BodyAttribute::angular_velocity));
}

VArray<float3> PhysicsGeometry::body_total_force() const
{
  return *attributes().lookup<float3>(physics_attribute_name(BodyAttribute::total_force));
}

VArray<float3> PhysicsGeometry::body_total_torque() const
{
  return *attributes().lookup<float3>(physics_attribute_name(BodyAttribute::total_torque));
}

VArray<bool> PhysicsGeometry::constraint_enabled() const
{
  return *attributes().lookup<bool>(physics_attribute_name(ConstraintAttribute::enabled));
}

AttributeWriter<bool> PhysicsGeometry::constraint_enabled_for_write()
{
  return attributes_for_write().lookup_for_write<bool>(
      physics_attribute_name(ConstraintAttribute::enabled));
}

VArray<int> PhysicsGeometry::constraint_types() const
{
  return *attributes().lookup<int>(physics_attribute_name(ConstraintAttribute::type));
}

AttributeWriter<int> PhysicsGeometry::constraint_types_for_write()
{
  const VArray<int> default_varray = VArray<int>::ForSingle(
      int(PhysicsGeometry::ConstraintType::Fixed), this->constraints_num());
  return attributes_for_write().lookup_or_add_for_write<int>(
      physics_attribute_name(ConstraintAttribute::type),
      AttrDomain::Edge,
      AttributeInitVArray(default_varray));
}

VArray<int> PhysicsGeometry::constraint_body1() const
{
  return *attributes().lookup<int>(physics_attribute_name(ConstraintAttribute::body1));
}

AttributeWriter<int> PhysicsGeometry::constraint_body1_for_write()
{
  const VArray<int> default_varray = VArray<int>::ForSingle(-1, this->constraints_num());
  return attributes_for_write().lookup_or_add_for_write<int>(
      physics_attribute_name(ConstraintAttribute::body1),
      AttrDomain::Edge,
      AttributeInitVArray(default_varray));
}

VArray<int> PhysicsGeometry::constraint_body2() const
{
  return *attributes().lookup<int>(physics_attribute_name(ConstraintAttribute::body2));
}

AttributeWriter<int> PhysicsGeometry::constraint_body2_for_write()
{
  const VArray<int> default_varray = VArray<int>::ForSingle(-1, this->constraints_num());
  return attributes_for_write().lookup_or_add_for_write<int>(
      physics_attribute_name(ConstraintAttribute::body2),
      AttrDomain::Edge,
      AttributeInitVArray(default_varray));
}

VArray<float4x4> PhysicsGeometry::constraint_frame1() const
{
  return *attributes().lookup<float4x4>(physics_attribute_name(ConstraintAttribute::frame1));
}

AttributeWriter<float4x4> PhysicsGeometry::constraint_frame1_for_write()
{
  return attributes_for_write().lookup_for_write<float4x4>(
      physics_attribute_name(ConstraintAttribute::frame1));
}

VArray<float4x4> PhysicsGeometry::constraint_frame2() const
{
  return *attributes().lookup<float4x4>(physics_attribute_name(ConstraintAttribute::frame2));
}

AttributeWriter<float4x4> PhysicsGeometry::constraint_frame2_for_write()
{
  return attributes_for_write().lookup_for_write<float4x4>(
      physics_attribute_name(ConstraintAttribute::frame2));
}

Span<PhysicsGeometry::BodyAttribute> PhysicsGeometry::all_body_attributes()
{
  return bke::all_body_attributes();
}

Span<PhysicsGeometry::ConstraintAttribute> PhysicsGeometry::all_constraint_attributes()
{
  return bke::all_constraint_attributes();
}

StringRef PhysicsGeometry::attribute_name(BodyAttribute attribute)
{
  return bke::physics_attribute_name(attribute);
}

StringRef PhysicsGeometry::attribute_name(ConstraintAttribute attribute)
{
  return bke::physics_attribute_name(attribute);
}

Span<std::string> PhysicsGeometry::all_body_attribute_names()
{
  return bke::all_body_attribute_names();
}

Span<std::string> PhysicsGeometry::all_constraint_attribute_names()
{
  return bke::all_constraint_attribute_names();
}

const CPPType &PhysicsGeometry::attribute_type(PhysicsBodyAttribute attribute)
{
  return bke::physics_attribute_type(attribute);
}

const CPPType &PhysicsGeometry::attribute_type(PhysicsConstraintAttribute attribute)
{
  return bke::physics_attribute_type(attribute);
}

const void *PhysicsGeometry::attribute_default_value(PhysicsBodyAttribute attribute)
{
  return bke::physics_attribute_default_value(attribute);
}

const void *PhysicsGeometry::attribute_default_value(PhysicsConstraintAttribute attribute)
{
  return bke::physics_attribute_default_value(attribute);
}

bool PhysicsGeometry::validate_world_data()
{
  return state_for_write().validate_world_data();
}

AttributeAccessor PhysicsGeometry::attributes() const
{
  return AttributeAccessor(&this->state(), bke::get_physics_accessor_functions_ref());
}

MutableAttributeAccessor PhysicsGeometry::attributes_for_write()
{
  return MutableAttributeAccessor(&this->state_for_write(),
                                  bke::get_physics_accessor_functions_ref());
}

AttributeAccessor PhysicsGeometry::dummy_attributes()
{
  return AttributeAccessor(nullptr, bke::get_physics_accessor_functions_ref());
}

MutableAttributeAccessor PhysicsGeometry::dummy_attributes_for_write()
{
  return MutableAttributeAccessor(nullptr, bke::get_physics_accessor_functions_ref());
}

/** \} */

}  // namespace blender::bke
