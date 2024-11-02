/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BKE_curves.hh"
#include "BKE_pointcloud.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_pdb_solver_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry");
  b.add_input<decl::Float>("Delta Time");
  b.add_input<decl::Int>("Substeps").min(1).default_value(1);
  b.add_output<decl::Geometry>("Geometry");
}

static void solve_floor_collision(MutableSpan<float3> positions, const float floor_height = 0.0f)
{
  for (const int pos_i : positions.index_range()) {
    const float3 unconstrained_pos = positions[pos_i];
    if (unconstrained_pos.z >= floor_height) {
      continue;
    }
    const float3 constrainted_pos{unconstrained_pos.x, unconstrained_pos.y, floor_height};
    positions[pos_i] = constrainted_pos;
  }
}

static void solve_floor_collision_with_friction(MutableSpan<float3> positions,
                                                const Span<float3> start_positions,
                                                const float friction_factor,
                                                const float floor_height = 0.0f)
{
  for (const int pos_i : positions.index_range()) {
    const float3 unconstrained_pos = positions[pos_i];
    if (unconstrained_pos.z >= floor_height) {
      continue;
    }

    const float3 constrainted_pos{unconstrained_pos.x, unconstrained_pos.y, floor_height};
    positions[pos_i] = constrainted_pos;

    const float3 velocity = constrainted_pos - start_positions[pos_i];
    const float3 contact_normal{0, 0, 1.0f};
    const float3 friction_tangent = velocity - velocity * contact_normal;
    positions[pos_i] -= friction_tangent * friction_factor;
  }
}

static void solve_length_constrainted(float3 &pos_a,
                                      float3 &pos_b,
                                      const float inv_weight_a,
                                      const float inv_weight_b,
                                      const float goal_length)
{
  const float inv_weight_sum = inv_weight_a + inv_weight_b;
  const float unconstrainted_length = math::distance(pos_a, pos_b);
  const float error = unconstrainted_length - goal_length;
  float3 gradient_a = math::safe_divide(pos_b - pos_a, unconstrainted_length);
  if (math::is_zero(gradient_a)) {
    gradient_a = float3(0, 0, 1);
  }
  gradient_a = math::normalize(gradient_a);
  const float3 gradient_b = -gradient_a;
  const float3 delta_a = gradient_a * error * inv_weight_a / inv_weight_sum;
  const float3 delta_b = gradient_b * error * inv_weight_b / inv_weight_sum;
  pos_a += delta_a;
  pos_b += delta_b;
}

static void solve_edges(MutableSpan<float3> positions,
                        const Span<int2> edges,
                        const Span<float> goal_lengths)
{
  for (const int edge_i : edges.index_range()) {
    const int2 edge = edges[edge_i];
    solve_length_constrainted(
        positions[edge[0]], positions[edge[1]], 1.0f, 1.0f, goal_lengths[edge_i]);
  }
}

static void solve_curve_segment_lengths(MutableSpan<float3> positions,
                                        const OffsetIndices<int> offsets,
                                        const Span<float> goal_segment_lengths)
{
  for (const int curve_i : offsets.index_range()) {
    const IndexRange points = offsets[curve_i];
    for (const int i0 : points.drop_back(1)) {
      const int i1 = i0 + 1;
      solve_length_constrainted(
          positions[i0], positions[i1], 1.0f, 1.0f, goal_segment_lengths[i1]);
    }
  }
}

static void solve_pinned(MutableSpan<float3> positions,
                         const Span<int> pin_indices,
                         const Span<float3> pin_positions)
{
  BLI_assert(pin_indices.size() == pin_positions.size());
  for (const int pin_i : pin_indices.index_range()) {
    const int pos_i = pin_indices[pin_i];
    const float3 &pin_pos = pin_positions[pin_i];
    positions[pos_i] = pin_pos;
  }
}

static void solve_point_collisions(MutableSpan<float3> positions,
                                   const float radius,
                                   const Span<float3> old_positions)
{
  const float cell_size = std::max(radius * 2, 0.00001f);
  const float inv_cell_size = 1.0f / cell_size;
  const int hash_table_size = std::max(1, power_of_2_max_i(positions.size()));
  const uint64_t mask = hash_table_size - 1;
  Array<Vector<int>> hash_table(hash_table_size);
  for (const int pos_i : positions.index_range()) {
    const float3 &pos = positions[pos_i];
    const int3 cell = int3(pos * inv_cell_size);
    const uint64_t hash = cell.hash() & mask;
    hash_table[hash].append(pos_i);
  }

  Vector<int> possible_colliders;
  for (const int pos_i : positions.index_range()) {
    possible_colliders.clear();

    float3 &pos = positions[pos_i];
    const int3 cell = int3(pos * inv_cell_size);
    for (int x = -1; x <= 1; x++) {
      for (int y = -1; y <= 1; y++) {
        for (int z = -1; z <= 1; z++) {
          const int3 other_cell = cell + int3(x, y, z);
          const uint64_t other_cell_hash = other_cell.hash() & mask;
          const Span<int> indices = hash_table[other_cell_hash];
          possible_colliders.extend(indices);
        }
      }
    }

    for (const int other_i : possible_colliders) {
      if (other_i <= pos_i) {
        continue;
      }
      float3 &other_pos = positions[other_i];
      const float distance = math::distance(pos, other_pos);
      const float goal_distance = 2 * radius;
      if (distance >= goal_distance) {
        continue;
      }
      solve_length_constrainted(pos, other_pos, 1.0f, 1.0f, goal_distance);
    }
  }
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Geometry");
  const float delta_time = params.get_input<float>("Delta Time");
  const int substeps = std::max(1, params.get_input<int>("Substeps"));
  const float substep_delta_time = delta_time / substeps;

  if (Mesh *mesh = geometry.get_mesh_for_write()) {
    MutableSpan<float3> positions = mesh->vert_positions_for_write();
    const Span<int2> edges = mesh->edges();

    const VArraySpan<float> goal_edge_lengths = *mesh->attributes().lookup_or_default<float>(
        "rest_edge_length", AttrDomain::Edge, 0.0f);

    for ([[maybe_unused]] const int substep_i : IndexRange(substeps)) {
      solve_floor_collision(positions);
      solve_edges(positions, edges, goal_edge_lengths);
    }

    mesh->tag_positions_changed();
  }
  if (Curves *curves_id = geometry.get_curves_for_write()) {
    bke::CurvesGeometry &curves = curves_id->geometry.wrap();
    MutableSpan<float3> positions = curves.positions_for_write();
    const OffsetIndices<int> offsets = curves.offsets();

    const VArraySpan<float> goal_segment_lengths = *curves.attributes().lookup_or_default<float>(
        "rest_segment_length", AttrDomain::Point, 0.0f);
    const VArraySpan<float3> pin_positions = *curves.attributes().lookup_or_default<float3>(
        "pin_positions", AttrDomain::Curve, float3(0, 0, 0));
    for ([[maybe_unused]] const int substep_i : IndexRange(substeps)) {
      solve_floor_collision(positions);
      solve_curve_segment_lengths(positions, offsets, goal_segment_lengths);
      solve_pinned(positions, offsets.data().drop_back(1), pin_positions);
    }

    curves.tag_positions_changed();
  }
  if (PointCloud *pointcloud = geometry.get_pointcloud_for_write()) {
    MutableSpan<float3> positions = pointcloud->positions_for_write();
    VArraySpan<float3> start_positions = *pointcloud->attributes().lookup_or_default<float3>(
        "start_position", AttrDomain::Point, float3(0, 0, 0));
    const VArraySpan<float> radii = *pointcloud->attributes().lookup_or_default<float>(
        "radius", AttrDomain::Point, 0.05f);
    const float uniform_radius = pointcloud->totpoint > 0 ? radii[0] : 0.0f;

    const float floor_friction = 0.1;

    for ([[maybe_unused]] const int substep_i : IndexRange(substeps)) {
      solve_floor_collision_with_friction(
          positions, start_positions, floor_friction, uniform_radius);
      solve_point_collisions(positions, uniform_radius, start_positions);
    }

    pointcloud->tag_positions_changed();
  }

  params.set_output("Geometry", geometry);
}

static void node_register()
{
  static blender::bke::bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_PDB_SOLVER, "PDB Solver", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_pdb_solver_cc
