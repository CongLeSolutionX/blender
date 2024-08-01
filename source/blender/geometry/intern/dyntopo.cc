/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <iostream>

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.hh"

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_index_range.hh"
#include "BLI_map.hh"
#include "BLI_math_geom.h"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_offset_indices.hh"
#include "BLI_ordered_edge.hh"
#include "BLI_span.hh"
#include "BLI_task.hh"
#include "BLI_vector.hh"
#include "BLI_vector_set.hh"

namespace blender::geometry::dyntopo {

template<typename T> std::ostream &operator<<(std::ostream &stream, Vector<T> data);

template<typename T, int num>
std::ostream &operator<<(std::ostream &stream, const std::array<T, num> &data)
{
  stream << "{";
  for (const int64_t i : IndexRange(num)) {
    stream << data[i] << (num - 1 == i ? "" : ", ");
  }
  stream << "}";
  return stream;
}

template<typename T> std::ostream &operator<<(std::ostream &stream, const Span<T> span)
{
  for (const int64_t i : span.index_range()) {
    stream << span[i] << (span.size() - 1 == i ? "" : ", ");
  }
  return stream;
}

template<typename T> std::ostream &operator<<(std::ostream &stream, MutableSpan<T> span)
{
  stream << span.as_span();
  return stream;
}

template<typename T> std::ostream &operator<<(std::ostream &stream, Vector<T> data)
{
  stream << data.as_span();
  return stream;
}

template<typename In, typename Out, typename Func>
static void parallel_transform(const Span<In> src,
                               const int64_t grain_size,
                               MutableSpan<Out> dst,
                               const Func func)
{
  BLI_assert(src.size() == dst.size());
  threading::parallel_for(src.index_range(), grain_size, [&](const IndexRange range) {
    const Span<In> src_slice = src.slice(range);
    MutableSpan<Out> dst_slice = dst.slice(range);
    std::transform(src_slice.begin(), src_slice.end(), dst_slice.begin(), func);
  });
}

namespace topo_set {

static bool diverse(const int2 all)
{
  return all.x != all.y;
}

static bool diverse(const int3 all)
{
  return (all.x != all.y) && (all.x != all.z) && (all.z != all.y);
}

static bool part_of(const int2 all, const int part)
{
  return part == all.x || part == all.y;
}

static bool part_of(const int3 all, const int part)
{
  return (part == all.x || part == all.y || part == all.z);
}

static bool part_of(const int3 all, const int2 part)
{
  return part_of(all, part.x) && part_of(all, part.y);
}

static bool equals(const int3 left, const int3 right)
{
  return part_of(left, right.x) && part_of(left, right.y) && part_of(left, right.z);
}

static bool equals(const int2 left, const int2 right)
{
  return part_of(left, right.x) && part_of(left, right.y);
}

static int index_of(const int3 all, const int part)
{
  BLI_assert(part_of(all, part));
  BLI_assert(diverse(all));
  return int(all.y == part) * 1 + int(all.z == part) * 2;
}

static int index_of(const int2 all, const int part)
{
  BLI_assert(part_of(all, part));
  BLI_assert(diverse(all));
  return int(all.y == part) * 1;
}

static int unordered_index_of(const int3 all, const int2 part)
{
  BLI_assert(part_of(all, part));
  BLI_assert(diverse(all));
  return int(equals(all.yz(), part)) * 1 + int(equals(int2(all.z, all.x), part)) * 2;
}

static int diff(const int3 all, const int2 part)
{
  BLI_assert(part_of(all, part));
  return (all.x - part.x) + (all.y - part.y) + all.z;
}

static int diff(const int2 all, const int part)
{
  BLI_assert(part_of(all, part));
  return (all.x - part) + all.y;
}

static int2 sample(const int3 all, const int2 indices)
{
  return int2{all[indices.x], all[indices.y]};
}

static const std::array<int2, 3> edges{int2(0, 1), int2(1, 2), int2(2, 0)};
static const std::array<int, 3> face{0, 1, 2};

static const int3 shift_front(1, 2, 0);
static const int3 shift_back(2, 0, 1);

static constexpr int vert_a = 0;
static constexpr int vert_b = 1;
static constexpr int vert_c = 2;

static constexpr int edge_ab = 0;
static constexpr int edge_bc = 1;
static constexpr int edge_ca = 2;

}  // namespace topo_set

static float len_squared_to_tris(const std::array<float2, 3> tri, const float2 pos)
{
  float3 result;
  std::array<float3, 3> tri_3d;
  tri_3d[0] = float3(tri[0], 0.0f);
  tri_3d[1] = float3(tri[1], 0.0f);
  tri_3d[2] = float3(tri[2], 0.0f);
  const float3 pos_3d(pos, 0.0f);
  closest_on_tri_to_point_v3(result, pos_3d, tri_3d[0], tri_3d[1], tri_3d[2]);
  return math::distance_squared(result, pos_3d);
}

static bool triangle_is_in_range(
    const float2 &a, const float2 &b, const float2 &c, const float2 &centre, const float range)
{
  return len_squared_to_tris({a, b, c}, centre) < range;
}

static float3 bary_weight_for_tris_point(const float2 a,
                                         const float2 b,
                                         const float2 c,
                                         const float2 p)
{
  float3 result;
  barycentric_weights_v2(a, b, c, p, result);
  return result;
}

namespace edge_state {
static constexpr const int8_t ab_is_real_edge = 1 << 0;
static constexpr const int8_t bc_is_real_edge = 1 << 1;
static constexpr const int8_t ca_is_real_edge = 1 << 2;

static constexpr const int8_t ab_is_owned_edge = 1 << 3;
static constexpr const int8_t bc_is_owned_edge = 1 << 4;
static constexpr const int8_t ca_is_owned_edge = 1 << 5;

static const std::array<int8_t, 3> edges_real = {
    ab_is_real_edge, bc_is_real_edge, ca_is_real_edge};
static const std::array<int8_t, 3> edges_owned = {
    ab_is_owned_edge, bc_is_owned_edge, ca_is_owned_edge};
}  // namespace edge_state

static void corner_verts_from_edges(const Span<int> corner_edges,
                                    const Span<int2> edges,
                                    const int faces_num,
                                    MutableSpan<int> corner_verts)
{
  threading::parallel_for(IndexRange(faces_num), 2048, [&](const IndexRange range) {
    for (const int64_t i : range) {
      const int face_i = i * 3;
      const int2 edge_a = edges[corner_edges[face_i + 0]];
      const int2 edge_b = edges[corner_edges[face_i + 1]];
      const int2 edge_c = edges[corner_edges[face_i + 2]];

      BLI_assert(topo_set::part_of(edge_b, edge_a[0]) != topo_set::part_of(edge_c, edge_a[0]));
      BLI_assert(topo_set::part_of(edge_a, edge_b[0]) != topo_set::part_of(edge_c, edge_b[0]));
      BLI_assert(topo_set::part_of(edge_b, edge_c[0]) != topo_set::part_of(edge_a, edge_c[0]));

      const int vert_a = topo_set::part_of(edge_c, edge_a[0]) ? edge_a[0] : edge_a[1];
      const int vert_c = bke::mesh::edge_other_vert(edge_c, vert_a);
      const int vert_b = bke::mesh::edge_other_vert(edge_b, vert_c);

      corner_verts[face_i + 0] = vert_a;
      corner_verts[face_i + 1] = vert_b;
      corner_verts[face_i + 2] = vert_c;
    }
  });
}

static void edge_tris_split_and_skip(const float3 a_point_3d,
                                     const float3 b_point_3d,
                                     const float2 a_point_2d,
                                     const float2 b_point_2d,
                                     const float2 centre,
                                     const float radius,
                                     const float max_length_squared,
                                     Vector<float3> &r_points_3d,
                                     Vector<float2> &r_points_2d)
{
  BLI_assert(r_points_3d.size() == r_points_2d.size());

  const IndexRange range = r_points_2d.index_range();
  for (const int point_i : range) {
    const int r_point_i = range.last(point_i);
    if (!triangle_is_in_range(a_point_2d, b_point_2d, r_points_2d[r_point_i], centre, radius)) {
      r_points_3d.remove_and_reorder(r_point_i);
      r_points_2d.remove_and_reorder(r_point_i);
      continue;
    }

    while (true) {
      const float3 c_point_3d = r_points_3d[r_point_i];
      const float2 c_point_2d = r_points_2d[r_point_i];

      const float ac_length_squared = math::distance_squared(a_point_3d, c_point_3d);
      const float bc_length_squared = math::distance_squared(b_point_3d, c_point_3d);
      if (math::max(ac_length_squared, bc_length_squared) <= max_length_squared) {
        break;
      }

      /* TODO. */
      BLI_assert(ac_length_squared != bc_length_squared);
      if (ac_length_squared < bc_length_squared) {
        r_points_3d[r_point_i] = math::midpoint(b_point_3d, c_point_3d);
        r_points_2d[r_point_i] = math::midpoint(b_point_2d, c_point_2d);
      }
      else {
        r_points_3d[r_point_i] = math::midpoint(a_point_3d, c_point_3d);
        r_points_2d[r_point_i] = math::midpoint(a_point_2d, c_point_2d);
      }

      if (!triangle_is_in_range(a_point_2d, b_point_2d, r_points_2d[r_point_i], centre, radius)) {
        // r_points_3d.remove_and_reorder(r_point_i);
        // r_points_2d.remove_and_reorder(r_point_i);
        break;
      }
    }
  }
}

static void face_subdivide(std::array<Vector<float2>, 3> connected_2d_points,
                           std::array<Vector<float3>, 3> connected_3d_points,
                           const std::array<float2, 3> tri_2d_points,
                           const std::array<float3, 3> tri_3d_points,
                           const float2 centre,
                           const float radius,
                           const float max_length,
                           int &r_total_verts,
                           int &r_total_edges,
                           int &r_total_faces)
{
  // std::cout << __func__ << ";\n";

  Vector<std::array<Vector<float2>, 3>> connected_2d_points_stack = {std::move(connected_2d_points)};
  Vector<std::array<Vector<float3>, 3>> connected_3d_points_stack = {std::move(connected_3d_points)};
  Vector<std::array<float2, 3>> tri_2d_points_stack = {tri_2d_points};
  Vector<std::array<float3, 3>> tri_3d_points_stack = {tri_3d_points};

  Vector<int8_t> edges_states = {0};

  Vector<bool> force_split_stack = {triangle_is_in_range(tri_2d_points[0], tri_2d_points[1], tri_2d_points[2], centre, radius)};

  while (!edges_states.is_empty()) {
    std::array<Vector<float2>, 3> connected_2d_points = connected_2d_points_stack.pop_last();
    std::array<Vector<float3>, 3> connected_3d_points = connected_3d_points_stack.pop_last();
    const std::array<float2, 3> tri_2d_points = tri_2d_points_stack.pop_last();
    const std::array<float3, 3> tri_3d_points = tri_3d_points_stack.pop_last();

    const int8_t edges_state = edges_states.pop_last();

    const bool force_this_split = force_split_stack.pop_last();

    // std::cout << "Pass;\n";
    // std::cout << "\t" << connected_2d_points << ";\n";
    // std::cout << "\t" << connected_3d_points << ";\n";
    // std::cout << "\t" << tri_2d_points << ";\n";
    // std::cout << "\t" << tri_3d_points << ";\n";
    // std::cout << "\t" << std::boolalpha << bool(edge_state::ab_is_real_edge & edges_state) << ",
    // " <<
    //                                        bool(edge_state::bc_is_real_edge & edges_state) << ",
    //                                        " << bool(edge_state::ca_is_real_edge & edges_state)
    //                                        << ";\n";

    std::array<float, 3> lengths_squared;
    lengths_squared[0] = math::distance_squared(tri_3d_points[0], tri_3d_points[1]);
    lengths_squared[1] = math::distance_squared(tri_3d_points[1], tri_3d_points[2]);
    lengths_squared[2] = math::distance_squared(tri_3d_points[2], tri_3d_points[0]);
    /* TODO. */
    BLI_assert(!(ELEM(lengths_squared[0], lengths_squared[1], lengths_squared[2]) || ELEM(lengths_squared[1], lengths_squared[0], lengths_squared[2])));

    std::array<int, 3> edge_indices = {0, 1, 2};
    std::sort(edge_indices.begin(), edge_indices.end(), [&](const int a, const int b) {
      return lengths_squared[a] > lengths_squared[b];
    });
    // std::cout << lengths_squared << edge_indices << "\n";

    const float squred_length_for_pass = lengths_squared[edge_indices[0]];
    if (squred_length_for_pass <= max_length) {
      r_total_faces++;
      continue;
    }

    for (const int side_i : topo_set::face) {
      edge_tris_split_and_skip(tri_3d_points[side_i],
                               tri_3d_points[topo_set::shift_front[side_i]],
                               tri_2d_points[side_i],
                               tri_2d_points[topo_set::shift_front[side_i]],
                               centre,
                               radius,
                               lengths_squared[side_i],
                               connected_3d_points[side_i],
                               connected_2d_points[side_i]);
    }

    int largest_side_to_split = edge_indices[0];
    const bool need_this_split = triangle_is_in_range(tri_2d_points[0], tri_2d_points[1], tri_2d_points[2], centre, radius);
    if (!(force_this_split || need_this_split))
    {
      const auto side_iter = std::find_if(edge_indices.begin(), edge_indices.end(), [&] (const int side_i) {
        const float side_length_squared = lengths_squared[side_i];
        const bool has_side_tris = !connected_3d_points[side_i].is_empty();
        return (side_length_squared > max_length) && has_side_tris;
      });

      if (side_iter == edge_indices.end()) {
        r_total_faces++;
        continue;
      }
      largest_side_to_split = *side_iter;
    }

    // std::cout << "\t" << "Split of:\t" << largest_side_to_split << "\n";

    const int2 split_edge = topo_set::edges[largest_side_to_split];
    const bool real_edge_split = edges_state & edge_state::edges_real[largest_side_to_split];

    r_total_edges++;
    if (real_edge_split) {
      // std::cout << "\t" << "Verts count" << ";\n";
      r_total_edges++;
      r_total_verts++;
    }

    const float2 mid_2d = math::midpoint(tri_2d_points[split_edge[0]],
                                         tri_2d_points[split_edge[1]]);
    const float3 mid_3d = math::midpoint(tri_3d_points[split_edge[0]],
                                         tri_3d_points[split_edge[1]]);

    std::array<Vector<float2>, 3> left_connected_2d_points;
    std::array<Vector<float3>, 3> left_connected_3d_points;
    std::array<float2, 3> left_tri_2d_points = tri_2d_points;
    std::array<float3, 3> left_tri_3d_points = tri_3d_points;

    left_connected_2d_points[largest_side_to_split] = connected_2d_points[largest_side_to_split];
    left_connected_3d_points[largest_side_to_split] = connected_3d_points[largest_side_to_split];

    left_connected_2d_points[topo_set::shift_front[largest_side_to_split]] = {
        tri_2d_points[topo_set::shift_front[largest_side_to_split]]};
    left_connected_3d_points[topo_set::shift_front[largest_side_to_split]] = {
        tri_3d_points[topo_set::shift_front[largest_side_to_split]]};

    left_connected_2d_points[topo_set::shift_back[largest_side_to_split]] = std::move(
        connected_2d_points[topo_set::shift_back[largest_side_to_split]]);
    left_connected_3d_points[topo_set::shift_back[largest_side_to_split]] = std::move(
        connected_3d_points[topo_set::shift_back[largest_side_to_split]]);

    left_tri_2d_points[topo_set::shift_front[largest_side_to_split]] = mid_2d;
    left_tri_3d_points[topo_set::shift_front[largest_side_to_split]] = mid_3d;

    const int8_t left_edges_state =
        edges_state | edge_state::edges_real[topo_set::shift_front[largest_side_to_split]];

    std::array<Vector<float2>, 3> right_connected_2d_points;
    std::array<Vector<float3>, 3> right_connected_3d_points;
    std::array<float2, 3> right_tri_2d_points = tri_2d_points;
    std::array<float3, 3> right_tri_3d_points = tri_3d_points;

    right_connected_2d_points[largest_side_to_split] = std::move(
        connected_2d_points[largest_side_to_split]);
    right_connected_3d_points[largest_side_to_split] = std::move(
        connected_3d_points[largest_side_to_split]);

    right_connected_2d_points[topo_set::shift_front[largest_side_to_split]] = std::move(
        connected_2d_points[topo_set::shift_front[largest_side_to_split]]);
    right_connected_3d_points[topo_set::shift_front[largest_side_to_split]] = std::move(
        connected_3d_points[topo_set::shift_front[largest_side_to_split]]);

    right_connected_2d_points[topo_set::shift_back[largest_side_to_split]] = {
        tri_2d_points[largest_side_to_split]};
    right_connected_3d_points[topo_set::shift_back[largest_side_to_split]] = {
        tri_3d_points[largest_side_to_split]};

    right_tri_2d_points[largest_side_to_split] = mid_2d;
    right_tri_3d_points[largest_side_to_split] = mid_3d;

    const int8_t right_edges_state =
        edges_state & (~edge_state::edges_real[topo_set::shift_back[largest_side_to_split]]);

    connected_2d_points_stack.append(std::move(left_connected_2d_points));
    connected_3d_points_stack.append(std::move(left_connected_3d_points));
    tri_2d_points_stack.append(std::move(left_tri_2d_points));
    tri_3d_points_stack.append(std::move(left_tri_3d_points));
    edges_states.append(left_edges_state);

    connected_2d_points_stack.append(std::move(right_connected_2d_points));
    connected_3d_points_stack.append(std::move(right_connected_3d_points));
    tri_2d_points_stack.append(std::move(right_tri_2d_points));
    tri_3d_points_stack.append(std::move(right_tri_3d_points));
    edges_states.append(right_edges_state);
    
    force_split_stack.append(need_this_split);
    force_split_stack.append(need_this_split);
  }
}

static void edge_subdivide(Vector<float2> connected_2d_points,
                           Vector<float3> connected_3d_points,
                           const std::array<float2, 2> edge_2d_points,
                           const std::array<float3, 2> edge_3d_points,
                           const float2 centre,
                           const float radius,
                           const float max_length,
                           int &r_split_count)
{
  Vector<Vector<float2>> connected_2d_points_stack = {std::move(connected_2d_points)};
  Vector<Vector<float3>> connected_3d_points_stack = {std::move(connected_3d_points)};
  Vector<std::array<float2, 2>> edge_2d_points_stack = {edge_2d_points};
  Vector<std::array<float3, 2>> edge_3d_points_stack = {edge_3d_points};

  while (!edge_2d_points_stack.is_empty()) {
    Vector<float2> connected_2d_points = connected_2d_points_stack.pop_last();
    Vector<float3> connected_3d_points = connected_3d_points_stack.pop_last();
    const std::array<float2, 2> edge_2d_points = edge_2d_points_stack.pop_last();
    const std::array<float3, 2> edge_3d_points = edge_3d_points_stack.pop_last();

    const float length_squared = math::distance_squared(edge_3d_points[0], edge_3d_points[1]);
    if (length_squared <= max_length) {
      continue;
    }

    edge_tris_split_and_skip(edge_3d_points[0],
                             edge_3d_points[1],
                             edge_2d_points[0],
                             edge_2d_points[1],
                             centre,
                             radius,
                             length_squared,
                             connected_3d_points,
                             connected_2d_points);

    if (connected_3d_points.is_empty()) {
      continue;
    }

    r_split_count++;

    const float2 a_2d_point = edge_2d_points[0];
    const float2 b_2d_point = edge_2d_points[1];

    const float2 mid_2d = math::midpoint(a_2d_point, b_2d_point);
    const float3 mid_3d = math::midpoint(edge_3d_points[0], edge_3d_points[1]);

    Vector<float2> left_connected_2d_points = connected_2d_points;
    Vector<float3> left_connected_3d_points = connected_3d_points;
    std::array<float2, 2> left_edge_2d_points = edge_2d_points;
    std::array<float3, 2> left_edge_3d_points = edge_3d_points;

    left_edge_2d_points[1] = mid_2d;
    left_edge_3d_points[1] = mid_3d;

    Vector<float2> right_connected_2d_points = std::move(connected_2d_points);
    Vector<float3> right_connected_3d_points = std::move(connected_3d_points);
    std::array<float2, 2> right_edge_2d_points = edge_2d_points;
    std::array<float3, 2> right_edge_3d_points = edge_3d_points;

    right_edge_2d_points[0] = mid_2d;
    right_edge_3d_points[0] = mid_3d;

    connected_2d_points_stack.append(std::move(left_connected_2d_points));
    connected_3d_points_stack.append(std::move(left_connected_3d_points));
    edge_2d_points_stack.append(left_edge_2d_points);
    edge_3d_points_stack.append(left_edge_3d_points);

    connected_2d_points_stack.append(std::move(right_connected_2d_points));
    connected_3d_points_stack.append(std::move(right_connected_3d_points));
    edge_2d_points_stack.append(right_edge_2d_points);
    edge_3d_points_stack.append(right_edge_3d_points);
  }
}

static void edge_subdivide(Vector<float2> connected_2d_points,
                           Vector<float3> connected_3d_points,
                           const std::array<float2, 2> edge_2d_points,
                           const std::array<float3, 2> edge_3d_points,
                           const float2 centre,
                           const float radius,
                           const float max_length,
                           const int2 ab_verts,
                           const int body_verts_offset,
                           MutableSpan<float> r_factors,
                           MutableSpan<int2> r_edges)
{
  Vector<Vector<float2>> connected_2d_points_stack = {std::move(connected_2d_points)};
  Vector<Vector<float3>> connected_3d_points_stack = {std::move(connected_3d_points)};
  Vector<std::array<float2, 2>> edge_2d_points_stack = {edge_2d_points};
  Vector<std::array<float3, 2>> edge_3d_points_stack = {edge_3d_points};

  Vector<float2> edge_factor_stack = {{0.0f, 1.0f}};
  Vector<int2> edge_verts_stack = {ab_verts};

  int vert_iter = 0;
  int edge_iter = 0;

  while (!edge_2d_points_stack.is_empty()) {
    Vector<float2> connected_2d_points = connected_2d_points_stack.pop_last();
    Vector<float3> connected_3d_points = connected_3d_points_stack.pop_last();
    const std::array<float2, 2> edge_2d_points = edge_2d_points_stack.pop_last();
    const std::array<float3, 2> edge_3d_points = edge_3d_points_stack.pop_last();

    const float2 edge_factors = edge_factor_stack.pop_last();
    const int2 edge_verts = edge_verts_stack.pop_last();

    const float length_squared = math::distance_squared(edge_3d_points[0], edge_3d_points[1]);
    if (length_squared <= max_length) {
      r_edges[edge_iter] = edge_verts;
      edge_iter++;
      continue;
    }

    edge_tris_split_and_skip(edge_3d_points[0],
                             edge_3d_points[1],
                             edge_2d_points[0],
                             edge_2d_points[1],
                             centre,
                             radius,
                             length_squared,
                             connected_3d_points,
                             connected_2d_points);

    if (connected_3d_points.is_empty()) {
      r_edges[edge_iter] = edge_verts;
      edge_iter++;
      continue;
    }

    const float2 a_2d_point = edge_2d_points[0];
    const float2 b_2d_point = edge_2d_points[1];

    r_factors[vert_iter] = math::midpoint(edge_factors.x, edge_factors.y);

    const float2 mid_2d = math::midpoint(a_2d_point, b_2d_point);
    const float3 mid_3d = math::midpoint(edge_3d_points[0], edge_3d_points[1]);

    Vector<float2> left_connected_2d_points = connected_2d_points;
    Vector<float3> left_connected_3d_points = connected_3d_points;
    std::array<float2, 2> left_edge_2d_points = edge_2d_points;
    std::array<float3, 2> left_edge_3d_points = edge_3d_points;

    left_edge_2d_points[1] = mid_2d;
    left_edge_3d_points[1] = mid_3d;

    int2 left_edge_verts = edge_verts;
    left_edge_verts[1] = body_verts_offset + vert_iter;

    float2 left_edge_factors = edge_factors;
    left_edge_factors[1] = math::midpoint(edge_factors.x, edge_factors.y);

    Vector<float2> right_connected_2d_points = std::move(connected_2d_points);
    Vector<float3> right_connected_3d_points = std::move(connected_3d_points);
    std::array<float2, 2> right_edge_2d_points = edge_2d_points;
    std::array<float3, 2> right_edge_3d_points = edge_3d_points;

    right_edge_2d_points[0] = mid_2d;
    right_edge_3d_points[0] = mid_3d;

    int2 right_edge_verts = edge_verts;
    right_edge_verts[0] = body_verts_offset + vert_iter;

    float2 right_edge_factors = edge_factors;
    right_edge_factors[0] = math::midpoint(edge_factors.x, edge_factors.y);

    connected_2d_points_stack.append(std::move(right_connected_2d_points));
    connected_3d_points_stack.append(std::move(right_connected_3d_points));
    edge_2d_points_stack.append(right_edge_2d_points);
    edge_3d_points_stack.append(right_edge_3d_points);
    edge_verts_stack.append(right_edge_verts);
    edge_factor_stack.append(right_edge_factors);

    connected_2d_points_stack.append(std::move(left_connected_2d_points));
    connected_3d_points_stack.append(std::move(left_connected_3d_points));
    edge_2d_points_stack.append(left_edge_2d_points);
    edge_3d_points_stack.append(left_edge_3d_points);
    edge_verts_stack.append(left_edge_verts);
    edge_factor_stack.append(left_edge_factors);

    vert_iter++;
  }
}

static void face_subdivide(std::array<Vector<float2>, 3> connected_2d_points,
                           std::array<Vector<float3>, 3> connected_3d_points,
                           const std::array<float2, 3> tri_2d_points,
                           const std::array<float3, 3> tri_3d_points,
                           const float2 centre,
                           const float radius,
                           const float max_length,
                           const int3 tri_verts,
                           const std::array<IndexRange, 3> side_edge_verts_ranges,
                           const std::array<IndexRange, 3> side_edge_edges_ranges,
                           const std::array<bool, 3> side_edge_dirrections,
                           const IndexRange body_verts_range,
                           const IndexRange body_edges_range,
                           MutableSpan<int2> r_edges,
                           MutableSpan<float3> r_bary_coods,
                           MutableSpan<int> r_edge_indices)
{
  // std::cout << __func__ << ";\n";

  // std::cout << side_edge_verts_ranges << ";\n";
  // std::cout << side_edge_edges_ranges << ";\n";
  // std::cout << body_verts_range << ";\n";
  // std::cout << body_edges_range << ";\n";
  // std::cout << tri_verts << ";\n";

  Vector<std::array<Vector<float2>, 3>> connected_2d_points_stack = {
      std::move(connected_2d_points)};
  Vector<std::array<Vector<float3>, 3>> connected_3d_points_stack = {
      std::move(connected_3d_points)};
  Vector<std::array<float2, 3>> tri_2d_points_stack = {tri_2d_points};
  Vector<std::array<float3, 3>> tri_3d_points_stack = {tri_3d_points};

  Vector<int3> tri_verts_stack = {tri_verts};

  VectorSet<OrderedEdge> edges_while_split;
  VectorSet<OrderedEdge> edges_in_face;

  Vector<int8_t> edges_states = {0};

  const std::array<float2, 3> &main_tri_2d_points = tri_2d_points;

  int3 edge_verts_iters(0);
  int3 edge_edges_iters(0);

  int face_iter = 0;

  Vector<bool> force_split_stack = {triangle_is_in_range(tri_2d_points[0], tri_2d_points[1], tri_2d_points[2], centre, radius)};


  while (!edges_states.is_empty()) {
    std::array<Vector<float2>, 3> connected_2d_points = connected_2d_points_stack.pop_last();
    std::array<Vector<float3>, 3> connected_3d_points = connected_3d_points_stack.pop_last();
    const std::array<float2, 3> tri_2d_points = tri_2d_points_stack.pop_last();
    const std::array<float3, 3> tri_3d_points = tri_3d_points_stack.pop_last();

    const int3 tri_verts = tri_verts_stack.pop_last();

    const int8_t edges_state = edges_states.pop_last();

    const bool force_this_split = force_split_stack.pop_last();

    // std::cout << "Pass;\n";
    // std::cout << "\t" << connected_2d_points << ";\n";
    // std::cout << "\t" << connected_3d_points << ";\n";
    // std::cout << "\t" << tri_2d_points << ";\n";
    // std::cout << "\t" << tri_3d_points << ";\n";
    // std::cout << "\t" << tri_verts << ";\n";
    // std::cout << "\t" << std::boolalpha << bool(edge_state::ab_is_real_edge & edges_state) << ",
    // " <<
    //                                        bool(edge_state::bc_is_real_edge & edges_state) << ",
    //                                        " << bool(edge_state::ca_is_real_edge & edges_state)
    //                                        << ";\n";

    std::array<float, 3> lengths_squared;
    lengths_squared[0] = math::distance_squared(tri_3d_points[0], tri_3d_points[1]);
    lengths_squared[1] = math::distance_squared(tri_3d_points[1], tri_3d_points[2]);
    lengths_squared[2] = math::distance_squared(tri_3d_points[2], tri_3d_points[0]);
    /* TODO. */
    BLI_assert(!(ELEM(lengths_squared[0], lengths_squared[1], lengths_squared[2]) ||
                 ELEM(lengths_squared[1], lengths_squared[0], lengths_squared[2])));

    std::array<int, 3> edge_indices = {0, 1, 2};
    std::sort(edge_indices.begin(), edge_indices.end(), [&](const int a, const int b) {
      return lengths_squared[a] > lengths_squared[b];
    });
    // std::cout << lengths_squared << edge_indices << "\n";

    const float squred_length_for_pass = lengths_squared[edge_indices[0]];
    if (squred_length_for_pass <= max_length) {
      int3 face_edges(-1);
      for (const int side_i : topo_set::face) {
        if (edges_state & edge_state::edges_owned[side_i]) {
          face_edges[side_i] = body_edges_range[edges_in_face.index_of_or_add(topo_set::sample(tri_verts, topo_set::edges[side_i]))];
        }
        else {
          const int edge_edge_i = edge_edges_iters[side_i];
          edge_edges_iters[side_i]++;
          const int edge_edge_index = side_edge_edges_ranges[side_i][edge_edge_i];
          face_edges[side_i] = edge_edge_index;
        }
      }
      r_edge_indices[face_iter + 0] = face_edges[0];
      r_edge_indices[face_iter + 1] = face_edges[1];
      r_edge_indices[face_iter + 2] = face_edges[2];
      face_iter += 3;
      // std::cout << "\t" << "Skip lack of length" << "\n";
      continue;
    }

    for (const int side_i : topo_set::face) {
      edge_tris_split_and_skip(tri_3d_points[side_i],
                               tri_3d_points[topo_set::shift_front[side_i]],
                               tri_2d_points[side_i],
                               tri_2d_points[topo_set::shift_front[side_i]],
                               centre,
                               radius,
                               lengths_squared[side_i],
                               connected_3d_points[side_i],
                               connected_2d_points[side_i]);
    }

    int largest_side_to_split = edge_indices[0];
    const bool need_this_split = triangle_is_in_range(tri_2d_points[0], tri_2d_points[1], tri_2d_points[2], centre, radius);
    if (!(force_this_split || need_this_split))
    {
      const auto side_iter = std::find_if(edge_indices.begin(), edge_indices.end(), [&] (const int side_i) {
        const float side_length_squared = lengths_squared[side_i];
        const bool has_side_tris = !connected_3d_points[side_i].is_empty();
        return (side_length_squared > max_length) && has_side_tris;
      });

      if (side_iter == edge_indices.end()) {
        int3 face_edges(-1);
        for (const int side_i : topo_set::face) {
          if (edges_state & edge_state::edges_owned[side_i]) {
            face_edges[side_i] = body_edges_range[edges_in_face.index_of_or_add(topo_set::sample(tri_verts, topo_set::edges[side_i]))];
          }
          else {
            const int edge_edge_i = edge_edges_iters[side_i];
            edge_edges_iters[side_i]++;
            const int edge_edge_index = side_edge_edges_ranges[side_i][edge_edge_i];
            face_edges[side_i] = edge_edge_index;
          }
        }
        r_edge_indices[face_iter + 0] = face_edges[0];
        r_edge_indices[face_iter + 1] = face_edges[1];
        r_edge_indices[face_iter + 2] = face_edges[2];
        face_iter += 3;
        continue;
      }
      largest_side_to_split = *side_iter;
    }

    // std::cout << "\t" << "Split of:\t" << largest_side_to_split << "\n";

    const int2 split_edge = topo_set::edges[largest_side_to_split];
    const int2 face_verts_to_split = topo_set::sample(tri_verts, split_edge);

    const bool real_edge_split = edges_state & edge_state::edges_real[largest_side_to_split];

    const int new_vert_i = [&]() -> int {
      const bool vert_is_in_body = edges_state & edge_state::edges_owned[largest_side_to_split];
      if (vert_is_in_body) {
        const int vert_in_body = edges_while_split.index_of_or_add(face_verts_to_split);
        return body_verts_range[vert_in_body];
      }

      const int index_in_edge = edge_verts_iters[largest_side_to_split];
      const int vert_on_edge = side_edge_verts_ranges[largest_side_to_split][index_in_edge];
      edge_verts_iters[largest_side_to_split]++;
      return vert_on_edge;
    }();

    const float2 mid_2d = math::midpoint(tri_2d_points[split_edge[0]],
                                         tri_2d_points[split_edge[1]]);
    const float3 mid_3d = math::midpoint(tri_3d_points[split_edge[0]],
                                         tri_3d_points[split_edge[1]]);

    if (real_edge_split) {
      const int body_edge_i = edges_while_split.index_of(face_verts_to_split);
      r_bary_coods[body_edge_i] = bary_weight_for_tris_point(
          main_tri_2d_points[0], main_tri_2d_points[1], main_tri_2d_points[2], mid_2d);
    }

    std::array<Vector<float2>, 3> left_connected_2d_points;
    std::array<Vector<float3>, 3> left_connected_3d_points;
    std::array<float2, 3> left_tri_2d_points = tri_2d_points;
    std::array<float3, 3> left_tri_3d_points = tri_3d_points;

    left_connected_2d_points[largest_side_to_split] = connected_2d_points[largest_side_to_split];
    left_connected_3d_points[largest_side_to_split] = connected_3d_points[largest_side_to_split];

    left_connected_2d_points[topo_set::shift_front[largest_side_to_split]] = {
        tri_2d_points[topo_set::shift_front[largest_side_to_split]]};
    left_connected_3d_points[topo_set::shift_front[largest_side_to_split]] = {
        tri_3d_points[topo_set::shift_front[largest_side_to_split]]};

    left_connected_2d_points[topo_set::shift_back[largest_side_to_split]] = std::move(
        connected_2d_points[topo_set::shift_back[largest_side_to_split]]);
    left_connected_3d_points[topo_set::shift_back[largest_side_to_split]] = std::move(
        connected_3d_points[topo_set::shift_back[largest_side_to_split]]);

    left_tri_2d_points[topo_set::shift_front[largest_side_to_split]] = mid_2d;
    left_tri_3d_points[topo_set::shift_front[largest_side_to_split]] = mid_3d;

    const int8_t left_edges_state =
        edges_state | edge_state::edges_real[topo_set::shift_front[largest_side_to_split]] |
        edge_state::edges_owned[topo_set::shift_front[largest_side_to_split]];

    int3 left_tri_verts = tri_verts;
    left_tri_verts[topo_set::shift_front[largest_side_to_split]] = new_vert_i;

    std::array<Vector<float2>, 3> right_connected_2d_points;
    std::array<Vector<float3>, 3> right_connected_3d_points;
    std::array<float2, 3> right_tri_2d_points = tri_2d_points;
    std::array<float3, 3> right_tri_3d_points = tri_3d_points;

    right_connected_2d_points[largest_side_to_split] = std::move(
        connected_2d_points[largest_side_to_split]);
    right_connected_3d_points[largest_side_to_split] = std::move(
        connected_3d_points[largest_side_to_split]);

    right_connected_2d_points[topo_set::shift_front[largest_side_to_split]] = std::move(
        connected_2d_points[topo_set::shift_front[largest_side_to_split]]);
    right_connected_3d_points[topo_set::shift_front[largest_side_to_split]] = std::move(
        connected_3d_points[topo_set::shift_front[largest_side_to_split]]);

    right_connected_2d_points[topo_set::shift_back[largest_side_to_split]] = {
        tri_2d_points[largest_side_to_split]};
    right_connected_3d_points[topo_set::shift_back[largest_side_to_split]] = {
        tri_3d_points[largest_side_to_split]};

    right_tri_2d_points[largest_side_to_split] = mid_2d;
    right_tri_3d_points[largest_side_to_split] = mid_3d;

    const int8_t right_edges_state =
        (edges_state & (~edge_state::edges_real[topo_set::shift_back[largest_side_to_split]])) |
        edge_state::edges_owned[topo_set::shift_back[largest_side_to_split]];

    int3 right_tri_verts = tri_verts;
    right_tri_verts[largest_side_to_split] = new_vert_i;

    if (side_edge_dirrections[largest_side_to_split]) {
      connected_2d_points_stack.append(std::move(right_connected_2d_points));
      connected_3d_points_stack.append(std::move(right_connected_3d_points));
      tri_2d_points_stack.append(std::move(right_tri_2d_points));
      tri_3d_points_stack.append(std::move(right_tri_3d_points));
      edges_states.append(right_edges_state);
      tri_verts_stack.append(right_tri_verts);

      connected_2d_points_stack.append(std::move(left_connected_2d_points));
      connected_3d_points_stack.append(std::move(left_connected_3d_points));
      tri_2d_points_stack.append(std::move(left_tri_2d_points));
      tri_3d_points_stack.append(std::move(left_tri_3d_points));
      edges_states.append(left_edges_state);
      tri_verts_stack.append(left_tri_verts);
      
      force_split_stack.append(need_this_split);
      force_split_stack.append(need_this_split);
    }
    else {
      connected_2d_points_stack.append(std::move(left_connected_2d_points));
      connected_3d_points_stack.append(std::move(left_connected_3d_points));
      tri_2d_points_stack.append(std::move(left_tri_2d_points));
      tri_3d_points_stack.append(std::move(left_tri_3d_points));
      edges_states.append(left_edges_state);
      tri_verts_stack.append(left_tri_verts);

      connected_2d_points_stack.append(std::move(right_connected_2d_points));
      connected_3d_points_stack.append(std::move(right_connected_3d_points));
      tri_2d_points_stack.append(std::move(right_tri_2d_points));
      tri_3d_points_stack.append(std::move(right_tri_3d_points));
      edges_states.append(right_edges_state);
      tri_verts_stack.append(right_tri_verts);
      
      force_split_stack.append(need_this_split);
      force_split_stack.append(need_this_split);
    }
  }

  r_edges.copy_from(edges_in_face.as_span().cast<int2>());
}

Mesh *subdivide(const Mesh &src_mesh,
                const Span<float2> projection,
                const float2 centre,
                const float radius,
                const float max_length)
{
  BLI_assert([&]() -> bool {
    const OffsetIndices<int> faces = src_mesh.faces();
    for (const int face_i : faces.index_range()) {
      if (faces[face_i].size() != 3) {
        return false;
      }
    }
    return true;
  }());

  const float squared_radius = radius * radius;
  const float squared_max_length = max_length * max_length;

  const OffsetIndices<int> faces = src_mesh.faces();
  const Span<int2> edges = src_mesh.edges();
  const Span<int> corner_edges = src_mesh.corner_edges();
  const Span<int> corner_verts = src_mesh.corner_verts();

  const Span<float3> src_positions = src_mesh.vert_positions();

  Array<int> edge_to_face_offset_data;
  Array<int> edge_to_face_indices;
  const GroupedSpan<int> edge_to_face_map = bke::mesh::build_edge_to_face_map(
      faces, corner_edges, src_mesh.edges_num, edge_to_face_offset_data, edge_to_face_indices);

  Array<int> edge_total_verts(src_mesh.edges_num + 1);
  Array<int> face_total_verts(src_mesh.faces_num + 1);
  Array<int> face_total_edges(src_mesh.faces_num + 1);
  Array<int> face_total_faces(src_mesh.faces_num + 1);

  for (const int edge_i : IndexRange(src_mesh.edges_num)) {
    const int2 edge = edges[edge_i];

    const Span<int> edge_faces = edge_to_face_map[edge_i];

    const float3 a_3d_vert = src_positions[edge[0]];
    const float3 b_3d_vert = src_positions[edge[1]];

    const float2 a_2d_vert = projection[edge[0]];
    const float2 b_2d_vert = projection[edge[1]];

    Vector<float3> connected_3d_points(edge_faces.size());
    std::transform(
        edge_faces.begin(), edge_faces.end(), connected_3d_points.begin(), [&](const int face_i) {
          const int a_vert = corner_verts[faces[face_i][0]];
          const int b_vert = corner_verts[faces[face_i][1]];
          const int c_vert = corner_verts[faces[face_i][2]];
          const int oposit_vert = topo_set::diff(int3(a_vert, b_vert, c_vert), edge);
          return src_positions[oposit_vert];
        });

    Vector<float2> connected_2d_points(edge_faces.size());
    std::transform(
        edge_faces.begin(), edge_faces.end(), connected_2d_points.begin(), [&](const int face_i) {
          const int a_vert = corner_verts[faces[face_i][0]];
          const int b_vert = corner_verts[faces[face_i][1]];
          const int c_vert = corner_verts[faces[face_i][2]];
          const int oposit_vert = topo_set::diff(int3(a_vert, b_vert, c_vert), edge);
          return projection[oposit_vert];
        });

    int total_verts = 0;
    edge_subdivide(std::move(connected_2d_points),
                   std::move(connected_3d_points),
                   {a_2d_vert, b_2d_vert},
                   {a_3d_vert, b_3d_vert},
                   centre,
                   squared_radius,
                   squared_max_length,
                   total_verts);
    edge_total_verts[edge_i] = total_verts;
  }

  for (const int face_i : faces.index_range()) {
    const IndexRange face = faces[face_i];

    const int a_vert = corner_verts[face[0]];
    const int b_vert = corner_verts[face[1]];
    const int c_vert = corner_verts[face[2]];

    const float3 a_3d_vert = src_positions[a_vert];
    const float3 b_3d_vert = src_positions[b_vert];
    const float3 c_3d_vert = src_positions[c_vert];

    const float2 a_2d_vert = projection[a_vert];
    const float2 b_2d_vert = projection[b_vert];
    const float2 c_2d_vert = projection[c_vert];

    const int ab_edge_i = corner_edges[face[0]];
    const int bc_edge_i = corner_edges[face[1]];
    const int cs_edge_i = corner_edges[face[2]];

    const int2 ab_edge = edges[ab_edge_i];
    const int2 bc_edge = edges[bc_edge_i];
    const int2 ca_edge = edges[cs_edge_i];

    BLI_assert(OrderedEdge(a_vert, b_vert) == OrderedEdge(ab_edge));
    BLI_assert(OrderedEdge(b_vert, c_vert) == OrderedEdge(bc_edge));
    BLI_assert(OrderedEdge(c_vert, a_vert) == OrderedEdge(ca_edge));

    std::array<Vector<float3>, 3> connected_3d_points;
    for (const int edge_i : topo_set::face) {
      const int edge_index = corner_edges[face[edge_i]];
      const int2 edge = edges[edge_index];
      Vector<float3> &points_3d = connected_3d_points[edge_i];
      const Span<int> edge_faces = edge_to_face_map[edge_index];
      points_3d.reserve(edge_faces.size() - 1);
      for (const int connected_face_i : edge_faces) {
        if (connected_face_i == face_i) {
          continue;
        }
        const int a_vert = corner_verts[faces[connected_face_i][0]];
        const int b_vert = corner_verts[faces[connected_face_i][1]];
        const int c_vert = corner_verts[faces[connected_face_i][2]];
        const int oposit_vert = topo_set::diff(int3(a_vert, b_vert, c_vert), edge);
        points_3d.append(src_positions[oposit_vert]);
      }
    }

    std::array<Vector<float2>, 3> connected_2d_points;
    for (const int edge_i : topo_set::face) {
      const int edge_index = corner_edges[face[edge_i]];
      const int2 edge = edges[edge_index];
      Vector<float2> &points_2d = connected_2d_points[edge_i];
      const Span<int> edge_faces = edge_to_face_map[edge_index];
      points_2d.reserve(edge_faces.size() - 1);
      for (const int connected_face_i : edge_faces) {
        if (connected_face_i == face_i) {
          continue;
        }
        const int a_vert = corner_verts[faces[connected_face_i][0]];
        const int b_vert = corner_verts[faces[connected_face_i][1]];
        const int c_vert = corner_verts[faces[connected_face_i][2]];
        const int oposit_vert = topo_set::diff(int3(a_vert, b_vert, c_vert), edge);
        points_2d.append(projection[oposit_vert]);
      }
    }

    int total_verts = 0;
    int total_edges = 0;
    int total_faces = 0;

    face_subdivide(std::move(connected_2d_points),
                   std::move(connected_3d_points),
                   {a_2d_vert, b_2d_vert, c_2d_vert},
                   {a_3d_vert, b_3d_vert, c_3d_vert},
                   centre,
                   squared_radius,
                   squared_max_length,
                   total_verts,
                   total_edges,
                   total_faces);

    // std::cout << " Result:\n";
    // std::cout << "\t total_verts: " << total_verts << ";\n";
    // std::cout << "\t total_edges: " << total_edges << ";\n";
    // std::cout << "\t total_faces: " << total_faces << ";\n";

    face_total_verts[face_i] = total_verts;
    face_total_edges[face_i] = total_edges;
    face_total_faces[face_i] = total_faces;
  }

  // std::cout << ">> Edges: " << edge_total_verts.as_span() << ";\n";
  // std::cout << std::endl;
  // std::cout << ">> Faces: " << face_total_verts.as_span() << ";\n";

  IndexMaskMemory memory;
  const IndexMask changed_edges = IndexMask::from_predicate(
      IndexMask(src_mesh.edges_num), GrainSize(8192), memory, [&](const int i) {
        return edge_total_verts[i] > 0;
      });

  const IndexMask changed_faces = IndexMask::from_predicate(
      IndexMask(src_mesh.faces_num), GrainSize(8192), memory, [&](const int i) {
        return face_total_edges[i] > 0;
      });

  const IndexMask keeped_edges = changed_edges.complement(IndexMask(src_mesh.edges_num), memory);
  const IndexMask keeped_faces = changed_faces.complement(IndexMask(src_mesh.faces_num), memory);

  const OffsetIndices<int> subdive_edge_verts = offset_indices::accumulate_counts_to_offsets(
      edge_total_verts);
  const OffsetIndices<int> subdive_face_verts = offset_indices::accumulate_counts_to_offsets(
      face_total_verts);
  const OffsetIndices<int> subdive_face_edges = offset_indices::accumulate_counts_to_offsets(
      face_total_edges);
  const OffsetIndices<int> subdive_face_faces = offset_indices::accumulate_counts_to_offsets(
      face_total_faces);

  const IndexRange verts_range(src_mesh.verts_num);
  const IndexRange edges_verts_range = verts_range.after(subdive_edge_verts.total_size());
  const IndexRange faces_verts_range = edges_verts_range.after(subdive_face_verts.total_size());

  const IndexRange edges_edges_range(src_mesh.edges_num + subdive_edge_verts.total_size());
  const IndexRange faces_edges_range = edges_edges_range.after(subdive_face_edges.total_size());

  const int total_verts = faces_verts_range.one_after_last();
  const int total_edges = faces_edges_range.one_after_last();
  const int total_faces = subdive_face_faces.total_size();

  Mesh *dst_mesh = BKE_mesh_new_nomain(total_verts, total_edges, total_faces, total_faces * 3);
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh->attributes_for_write();

  offset_indices::fill_constant_group_size(3, 0, dst_mesh->face_offsets_for_write());

  MutableSpan<float3> dst_positions = dst_mesh->vert_positions_for_write();
  MutableSpan<int2> dst_edges = dst_mesh->edges_for_write();
  dst_edges.fill({0, 1});

  MutableSpan<int> dst_corner_verts = dst_mesh->corner_verts_for_write();
  MutableSpan<int> dst_corner_edges = dst_mesh->corner_edges_for_write();

  // bke::gather_attributes(src_attributes, bke::AttrDomain::Point, {}, {}, verts_range,
  // dst_attributes);
  dst_positions.take_front(src_mesh.verts_num).copy_from(src_positions);

  keeped_edges.foreach_index_optimized<int>(GrainSize(8092), [&](const int i) {
    dst_edges[edge_total_verts[i] + i] = edges[i];
  });

  Array<float> edge_vertices_factor_weight(edges_verts_range.size(), 0.0f);
  Array<float3> face_vertices_bary_weight(faces_verts_range.size(), float3(0.0f));

  changed_edges.foreach_index([&](const int edge_i) {
    const int2 edge = edges[edge_i];

    const Span<int> edge_faces = edge_to_face_map[edge_i];

    const float3 a_3d_vert = src_positions[edge[0]];
    const float3 b_3d_vert = src_positions[edge[1]];

    const float2 a_2d_vert = projection[edge[0]];
    const float2 b_2d_vert = projection[edge[1]];

    Vector<float3> connected_3d_points(edge_faces.size());
    std::transform(
        edge_faces.begin(), edge_faces.end(), connected_3d_points.begin(), [&](const int face_i) {
          const int a_vert = corner_verts[faces[face_i][0]];
          const int b_vert = corner_verts[faces[face_i][1]];
          const int c_vert = corner_verts[faces[face_i][2]];
          const int oposit_vert = topo_set::diff(int3(a_vert, b_vert, c_vert), edge);
          return src_positions[oposit_vert];
        });

    Vector<float2> connected_2d_points(edge_faces.size());
    std::transform(
        edge_faces.begin(), edge_faces.end(), connected_2d_points.begin(), [&](const int face_i) {
          const int a_vert = corner_verts[faces[face_i][0]];
          const int b_vert = corner_verts[faces[face_i][1]];
          const int c_vert = corner_verts[faces[face_i][2]];
          const int oposit_vert = topo_set::diff(int3(a_vert, b_vert, c_vert), edge);
          return projection[oposit_vert];
        });

    const IndexRange edge_verts_range = subdive_edge_verts[edge_i];
    const int edge_verts_start = edges_verts_range[edge_verts_range.start()];

    MutableSpan<float> edge_verts_factors = edge_vertices_factor_weight.as_mutable_span().slice(
        edge_verts_range);
    MutableSpan<int2> edges = dst_edges.slice(edge_verts_range.start() + edge_i,
                                              edge_verts_range.size() + 1);
    edge_subdivide(std::move(connected_2d_points),
                   std::move(connected_3d_points),
                   {a_2d_vert, b_2d_vert},
                   {a_3d_vert, b_3d_vert},
                   centre,
                   squared_radius,
                   squared_max_length,
                   edge,
                   edge_verts_start,
                   edge_verts_factors,
                   edges);
  });

  changed_faces.foreach_index([&](const int face_i) {
    const IndexRange face = faces[face_i];

    const int a_vert = corner_verts[face[0]];
    const int b_vert = corner_verts[face[1]];
    const int c_vert = corner_verts[face[2]];

    const float3 a_3d_vert = src_positions[a_vert];
    const float3 b_3d_vert = src_positions[b_vert];
    const float3 c_3d_vert = src_positions[c_vert];

    const float2 a_2d_vert = projection[a_vert];
    const float2 b_2d_vert = projection[b_vert];
    const float2 c_2d_vert = projection[c_vert];

    const int ab_edge_i = corner_edges[face[0]];
    const int bc_edge_i = corner_edges[face[1]];
    const int ca_edge_i = corner_edges[face[2]];

    const int2 ab_edge = edges[ab_edge_i];
    const int2 bc_edge = edges[bc_edge_i];
    const int2 ca_edge = edges[ca_edge_i];

    BLI_assert(OrderedEdge(a_vert, b_vert) == OrderedEdge(ab_edge));
    BLI_assert(OrderedEdge(b_vert, c_vert) == OrderedEdge(bc_edge));
    BLI_assert(OrderedEdge(c_vert, a_vert) == OrderedEdge(ca_edge));

    std::array<Vector<float3>, 3> connected_3d_points;
    for (const int edge_i : topo_set::face) {
      const int edge_index = corner_edges[face[edge_i]];
      const int2 edge = edges[edge_index];
      Vector<float3> &points_3d = connected_3d_points[edge_i];
      const Span<int> edge_faces = edge_to_face_map[edge_index];
      points_3d.reserve(edge_faces.size() - 1);
      for (const int connected_face_i : edge_faces) {
        if (connected_face_i == face_i) {
          continue;
        }
        const int a_vert = corner_verts[faces[connected_face_i][0]];
        const int b_vert = corner_verts[faces[connected_face_i][1]];
        const int c_vert = corner_verts[faces[connected_face_i][2]];
        const int oposit_vert = topo_set::diff(int3(a_vert, b_vert, c_vert), edge);
        points_3d.append(src_positions[oposit_vert]);
      }
    }

    std::array<Vector<float2>, 3> connected_2d_points;
    for (const int edge_i : topo_set::face) {
      const int edge_index = corner_edges[face[edge_i]];
      const int2 edge = edges[edge_index];
      Vector<float2> &points_2d = connected_2d_points[edge_i];
      const Span<int> edge_faces = edge_to_face_map[edge_index];
      points_2d.reserve(edge_faces.size() - 1);
      for (const int connected_face_i : edge_faces) {
        if (connected_face_i == face_i) {
          continue;
        }
        const int a_vert = corner_verts[faces[connected_face_i][0]];
        const int b_vert = corner_verts[faces[connected_face_i][1]];
        const int c_vert = corner_verts[faces[connected_face_i][2]];
        const int oposit_vert = topo_set::diff(int3(a_vert, b_vert, c_vert), edge);
        points_2d.append(projection[oposit_vert]);
      }
    }

    const IndexRange ab_edge_verts = subdive_edge_verts[ab_edge_i].shift(
        edges_verts_range.start());
    const IndexRange bc_edge_verts = subdive_edge_verts[bc_edge_i].shift(
        edges_verts_range.start());
    const IndexRange ca_edge_verts = subdive_edge_verts[ca_edge_i].shift(
        edges_verts_range.start());

    const IndexRange ab_edge_edges = IndexRange(subdive_edge_verts[ab_edge_i].start() + ab_edge_i,
                                                subdive_edge_verts[ab_edge_i].size() + 1)
                                         .shift(edges_edges_range.start());
    const IndexRange bc_edge_edges = IndexRange(subdive_edge_verts[bc_edge_i].start() + bc_edge_i,
                                                subdive_edge_verts[bc_edge_i].size() + 1)
                                         .shift(edges_edges_range.start());
    const IndexRange ca_edge_edges = IndexRange(subdive_edge_verts[ca_edge_i].start() + ca_edge_i,
                                                subdive_edge_verts[ca_edge_i].size() + 1)
                                         .shift(edges_edges_range.start());

    const bool ab_edge_dirrection = ab_edge == int2(a_vert, b_vert);
    const bool bc_edge_dirrection = bc_edge == int2(b_vert, c_vert);
    const bool ca_edge_dirrection = ca_edge == int2(c_vert, a_vert);

    const IndexRange face_verts_range = subdive_face_verts[face_i].shift(
        faces_verts_range.start());
    const IndexRange face_edges_range = subdive_face_edges[face_i].shift(
        faces_edges_range.start());

    MutableSpan<int2> face_edges = dst_edges.slice(
        subdive_face_edges[face_i].shift(faces_edges_range.start()));
    MutableSpan<float3> face_verts_bary_coords = face_vertices_bary_weight.as_mutable_span().slice(
        subdive_face_verts[face_i]);
    MutableSpan<int> face_cornder_edges = dst_corner_edges.slice(
        subdive_face_faces[face_i].start() * 3, subdive_face_faces[face_i].size() * 3);

    face_subdivide(std::move(connected_2d_points),
                   std::move(connected_3d_points),
                   {a_2d_vert, b_2d_vert, c_2d_vert},
                   {a_3d_vert, b_3d_vert, c_3d_vert},
                   centre,
                   squared_radius,
                   squared_max_length,
                   {a_vert, b_vert, c_vert},
                   {ab_edge_verts, bc_edge_verts, ca_edge_verts},
                   {ab_edge_edges, bc_edge_edges, ca_edge_edges},
                   {ab_edge_dirrection, bc_edge_dirrection, ca_edge_dirrection},
                   face_verts_range,
                   face_edges_range,
                   face_edges,
                   face_verts_bary_coords,
                   face_cornder_edges);
  });

  keeped_faces.foreach_index_optimized<int>(GrainSize(4096), [&](const int face_i) {
    const int start = subdive_face_faces[face_i].start() * 3;

    const int ab_edge_i = corner_edges[faces[face_i][0]];
    const int bc_edge_i = corner_edges[faces[face_i][1]];
    const int ca_edge_i = corner_edges[faces[face_i][2]];
    dst_corner_edges[start + 0] = subdive_edge_verts[ab_edge_i].start() + ab_edge_i;
    dst_corner_edges[start + 1] = subdive_edge_verts[bc_edge_i].start() + bc_edge_i;
    dst_corner_edges[start + 2] = subdive_edge_verts[ca_edge_i].start() + ca_edge_i;
  });

  // std::cout << dst_corner_edges << ";\n";
  // std::cout << dst_edges << ";\n";

  corner_verts_from_edges(dst_corner_edges, dst_edges, total_faces, dst_corner_verts);

  changed_edges.foreach_index([&](const int edge_i) {
    const int2 edge = edges[edge_i];
    const IndexRange edge_verts_range = subdive_edge_verts[edge_i];
    const Span<float> factors = edge_vertices_factor_weight.as_span().slice(edge_verts_range);
    MutableSpan<float3> dst = dst_positions.slice(
        edge_verts_range.shift(edges_verts_range.start()));
    const float3 a = dst_positions[edge[0]];
    const float3 b = dst_positions[edge[1]];
    parallel_transform(factors, 2048, dst, [&](const float factor) {
      return bke::attribute_math::mix2(factor, a, b);
    });
  });

  changed_faces.foreach_index([&](const int face_i) {
    const IndexRange face = faces[face_i];

    const int a_vert = corner_verts[face[0]];
    const int b_vert = corner_verts[face[1]];
    const int c_vert = corner_verts[face[2]];

    const IndexRange face_verts_range = subdive_face_verts[face_i];
    const Span<float3> bary_weights = face_vertices_bary_weight.as_span().slice(face_verts_range);
    MutableSpan<float3> dst = dst_positions.slice(
        face_verts_range.shift(faces_verts_range.start()));

    const float3 a = dst_positions[a_vert];
    const float3 b = dst_positions[b_vert];
    const float3 c = dst_positions[c_vert];
    parallel_transform(bary_weights, 2048, dst, [&](const float3 weight) {
      // std::cout << ":: " << a << ", " << b << ", " << c << ": " << weight << ";\n";
      return bke::attribute_math::mix3(weight, a, b, c);
    });
  });

  BLI_assert(std::all_of(dst_edges.cast<int>().begin(),
                         dst_edges.cast<int>().end(),
                         [&](const int vert_i) { return vert_i >= 0 && vert_i < total_verts; }));

  return dst_mesh;
}

}  // namespace blender::geometry::dyntopo
