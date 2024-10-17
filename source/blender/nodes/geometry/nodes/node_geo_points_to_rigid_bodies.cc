/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_pointcloud_types.h"

#include "BKE_collision_shape.hh"
#include "BKE_physics_geometry.hh"

#include "FN_field.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_points_to_rigid_bodies_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Points").supported_type(GeometryComponent::Type::PointCloud);
  b.add_input<decl::Bool>("Selection").default_value(true).field_on({0}).hide_value();
  b.add_input<decl::Float>("Mass").default_value(1.0f).field_on({0});
  b.add_input<decl::Matrix>("Inertia").field_on({0}).hide_value();
  b.add_input<decl::Geometry>("Shapes").description(
      "Collision shapes to choose from for each body");
  b.add_input<decl::Int>("Shape Index")
      .description("Index of the collision shape used for each point")
      .field_on({0});
  b.add_input<decl::Vector>("Position")
      .field_on({0})
      .implicit_field(implicit_field_inputs::position);
  b.add_input<decl::Rotation>("Rotation").field_on({0}).hide_value();
  b.add_input<decl::Vector>("Velocity").field_on({0}).hide_value();
  b.add_input<decl::Vector>("Angular Velocity").field_on({0}).hide_value();
  b.add_output<decl::Geometry>("Rigid Bodies").propagate_all();
}

static void geometry_set_points_to_rigid_bodies(GeometrySet &geometry_set,
                                                Field<bool> &selection_field,
                                                Field<float> &mass_field,
                                                Field<float4x4> &inertia_field,
                                                Field<float3> &position_field,
                                                Field<math::Quaternion> &rotation_field,
                                                Field<float3> &velocity_field,
                                                Field<float3> &angular_velocity_field,
                                                const GeometrySet shapes_geometry,
                                                Field<int> shape_index_field,
                                                const AttributeFilter & /*attribute_filter*/)
{
  const PointCloud *points = geometry_set.get_pointcloud();
  if (points == nullptr || points->totpoint == 0) {
    geometry_set.remove_geometry_during_modify();
    return;
  }

  const bke::PointCloudFieldContext field_context{*points};
  fn::FieldEvaluator field_evaluator{field_context, points->totpoint};
  field_evaluator.set_selection(selection_field);
  field_evaluator.add(mass_field);
  field_evaluator.add(inertia_field);
  field_evaluator.add(position_field);
  field_evaluator.add(rotation_field);
  field_evaluator.add(velocity_field);
  field_evaluator.add(angular_velocity_field);
  field_evaluator.add(shape_index_field);
  field_evaluator.evaluate();
  const IndexMask selection = field_evaluator.get_evaluated_selection_as_mask();

  const VArray<float> src_masses = field_evaluator.get_evaluated<float>(0);
  const VArray<float4x4> src_inertias = field_evaluator.get_evaluated<float4x4>(1);
  const VArray<float3> src_positions = field_evaluator.get_evaluated<float3>(2);
  const VArray<math::Quaternion> src_rotations = field_evaluator.get_evaluated<math::Quaternion>(
      3);
  const VArray<float3> src_velocities = field_evaluator.get_evaluated<float3>(4);
  const VArray<float3> src_angular_velocities = field_evaluator.get_evaluated<float3>(5);
  const VArray<int> src_shape_index = field_evaluator.get_evaluated<int>(6);

  const int num_bodies = selection.size();
  auto *physics = new bke::PhysicsGeometry(num_bodies, 0);

  Array<int> handle_map;
  if (shapes_geometry.has_instances()) {
    const bke::Instances *instances = shapes_geometry.get_instances();
    handle_map.reinitialize(instances->references_num());
    for (const int i : instances->references().index_range()) {
      const bke::InstanceReference &reference = instances->references()[i];
      handle_map[i] = physics->state_for_write().add_shape(reference);
    }
  }

  // Array<int> body_shape_handles(num_bodies);
  // selection.foreach_index(GrainSize(512), [&](const int index, const int pos) {
  //   const int shape_index = src_shape_index[index];
  //   body_shape_handles[pos] = shapes.index_range().contains(shape_index) ? shape_index : -1;
  // });

  AttributeWriter<int> dst_body_shapes = physics->body_shapes_for_write();
  AttributeWriter<float> dst_masses = physics->body_masses_for_write();
  AttributeWriter<float4x4> dst_inertias = physics->body_inertias_for_write();
  AttributeWriter<float3> dst_positions = physics->body_positions_for_write();
  AttributeWriter<math::Quaternion> dst_rotations = physics->body_rotations_for_write();
  AttributeWriter<float3> dst_velocities = physics->body_velocities_for_write();
  AttributeWriter<float3> dst_angular_velocities = physics->body_angular_velocities_for_write();

  selection.foreach_index(GrainSize(512), [&](const int index, const int pos) {
    const int original_index = src_shape_index[index];
    const int shape_index = mod_i(original_index, std::max(int(handle_map.size()), 1));
    dst_body_shapes.varray.set(pos,
                               handle_map.index_range().contains(shape_index) ? shape_index : -1);
    dst_masses.varray.set(pos, src_masses[index]);
    dst_inertias.varray.set(pos, src_inertias[index]);
    dst_positions.varray.set(pos, src_positions[index]);
    dst_rotations.varray.set(pos, src_rotations[index]);
    dst_velocities.varray.set(pos, src_velocities[index]);
    dst_angular_velocities.varray.set(pos, src_angular_velocities[index]);
  });

  dst_body_shapes.finish();
  dst_masses.finish();
  dst_inertias.finish();
  dst_positions.finish();
  dst_rotations.finish();
  dst_velocities.finish();
  dst_angular_velocities.finish();

  geometry_set.replace_physics(physics);
  geometry_set.keep_only_during_modify({GeometryComponent::Type::Physics});
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Points");
  Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");
  Field<float> mass_field = params.extract_input<Field<float>>("Mass");
  Field<float4x4> inertia_field = params.extract_input<Field<float4x4>>("Inertia");
  Field<float3> position_field = params.extract_input<Field<float3>>("Position");
  Field<math::Quaternion> rotation_field = params.extract_input<Field<math::Quaternion>>(
      "Rotation");
  Field<float3> velocity_field = params.extract_input<Field<float3>>("Velocity");
  Field<float3> angular_velocity_field = params.extract_input<Field<float3>>("Angular Velocity");
  GeometrySet shapes_geometry = params.extract_input<GeometrySet>("Shapes");
  Field<int> shape_index_field = params.extract_input<Field<int>>("Shape Index");

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    geometry_set_points_to_rigid_bodies(geometry_set,
                                        selection_field,
                                        mass_field,
                                        inertia_field,
                                        position_field,
                                        rotation_field,
                                        velocity_field,
                                        angular_velocity_field,
                                        shapes_geometry,
                                        shape_index_field,
                                        params.get_attribute_filter("Rigid Bodies"));
  });

  params.set_output("Rigid Bodies", std::move(geometry_set));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_POINTS_TO_RIGID_BODIES, "Points to Rigid Bodies", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_points_to_rigid_bodies_cc
