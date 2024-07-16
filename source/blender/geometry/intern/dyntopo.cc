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

static int3 gather_tri(const IndexRange range, const Span<int> indices)
{
  BLI_assert(range.size() == 3);
  return int3(indices[range[0]], indices[range[1]], indices[range[2]]);
}

static bool elem(const int2 items, const int to_check)
{
  return ELEM(to_check, items[0], items[1]);
}

static bool elem(const int3 items, const int to_check)
{
  return ELEM(to_check, items[0], items[1], items[2]);
}

static int exclusive_one(const int3 set, const int2 part)
{
  BLI_assert(elem(set, part[0]));
  BLI_assert(elem(set, part[1]));
  BLI_assert(set[0] >= 0 && set[1] >= 0 && set[2] >= 0);
  const int exclusive_elem = (set[0] - part[0]) + (set[1] - part[1]) + set[2];
  BLI_assert(elem(set, exclusive_elem));
  BLI_assert(!elem(part, exclusive_elem));
  BLI_assert(exclusive_elem >= 0);
  return exclusive_elem;
}

static int exclusive_one(const int2 set, const int part)
{
  BLI_assert(elem(set, part));
  BLI_assert(set[0] >= 0 && set[1] >= 0);
  const int exclusive_elem = (set[0] - part) + set[1];
  BLI_assert(elem(set, exclusive_elem));
  BLI_assert(exclusive_elem >= 0);
  return exclusive_elem;
}

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

static float3 bary_weight_for_tris_point(const float2 a,
                                         const float2 b,
                                         const float2 c,
                                         const float2 p)
{
  float3 result;
  barycentric_weights_v2(a, b, c, p, result);
  return result;
}

static bool triangle_is_in_range(
    const float2 &a, const float2 &b, const float2 &c, const float2 &centre, const float range)
{
  return len_squared_to_tris({a, b, c}, centre) < range;
}

static std::optional<int2> largest_side_to_split(const float2 &a,
                                                 const float2 &b,
                                                 const float2 &c,
                                                 const std::array<int, 9> &edge_indices,
                                                 const float max_length)
{
  const std::array<float, 3> to_compare = {
      math::distance_squared(a, b), math::distance_squared(b, c), math::distance_squared(c, a)};
  constexpr const IndexRange edges_range(3);
  const int max_elem_i = *std::max_element(
      edges_range.begin(), edges_range.end(), [&](const int a_i, const int b_i) -> bool {
        const float a = to_compare[a_i];
        const float b = to_compare[b_i];
        if (UNLIKELY(a == b)) {
          return edge_indices[a_i] < edge_indices[b_i];
        }
        return a < b;
      });

  if (UNLIKELY(to_compare[max_elem_i] <= max_length)) {
    return std::nullopt;
  }

  switch (max_elem_i) {
    case 0:
      return int2(0, 1);
    case 1:
      return int2(1, 2);
    case 2:
      return int2(2, 0);
    default:
      BLI_assert_unreachable();
      return {};
  }
}

namespace FaceVerts {
static constexpr const int a = 0;
static constexpr const int b = 1;
static constexpr const int c = 2;
static const int3 abc(0, 1, 2);
}  // namespace FaceVerts

namespace EdgeState {
static constexpr const int8_t ab_is_real_edge = 1 << 0;
static constexpr const int8_t bc_is_real_edge = 1 << 1;
static constexpr const int8_t ca_is_real_edge = 1 << 2;

static const std::array<int8_t, 3> edge_state_for_vert = {
    bc_is_real_edge, ca_is_real_edge, ab_is_real_edge};
}  // namespace EdgeState

static void split_edge_for_vert(const std::array<float2, 6> &verts,
                                const int2 split_edge,
                                Vector<std::array<float2, 6>> &r_list)
{
  switch (exclusive_one(int3(0, 1, 2), split_edge)) {
    case 2: {
      const float2 mid = math::midpoint(verts[0], verts[1]);
      r_list.append({verts[0], mid, verts[2], verts[3], verts[2], verts[5]});
      r_list.append({mid, verts[1], verts[2], verts[3], verts[4], verts[0]});
      break;
    }
    case 0: {
      const float2 mid = math::midpoint(verts[1], verts[2]);
      r_list.append({verts[0], verts[1], mid, verts[3], verts[4], verts[2]});
      r_list.append({verts[0], mid, verts[2], verts[2], verts[4], verts[5]});
      break;
    }
    case 1: {
      const float2 mid = math::midpoint(verts[2], verts[0]);
      r_list.append({verts[0], verts[1], mid, verts[3], verts[2], verts[5]});
      r_list.append({mid, verts[1], verts[2], verts[0], verts[4], verts[5]});
      break;
    }
  }
}

static void split_edge_for_state(const int8_t state, const int2 split_edge, Vector<int8_t> &r_list)
{
  switch (exclusive_one(FaceVerts::abc, split_edge)) {
    case FaceVerts::a: {
      r_list.append(state & ~EdgeState::ca_is_real_edge);
      r_list.append(state & ~EdgeState::ab_is_real_edge);
      break;
    }
    case FaceVerts::b: {
      r_list.append(state & ~EdgeState::bc_is_real_edge);
      r_list.append(state & ~EdgeState::ab_is_real_edge);
      break;
    }
    case FaceVerts::c: {
      r_list.append(state & ~EdgeState::bc_is_real_edge);
      r_list.append(state & ~EdgeState::ca_is_real_edge);
      break;
    }
  }
}

static void split_edge_for_indices(const int3 verts,
                                   const int new_vert,
                                   const int2 split_edge,
                                   Vector<int3> &r_list)
{
  switch (exclusive_one(FaceVerts::abc, split_edge)) {
    case FaceVerts::a: {
      r_list.append(int3(verts[0], verts[1], new_vert));
      r_list.append(int3(verts[0], new_vert, verts[2]));
      break;
    }
    case FaceVerts::b: {
      r_list.append(int3(verts[0], verts[1], new_vert));
      r_list.append(int3(new_vert, verts[1], verts[2]));
      break;
    }
    case FaceVerts::c: {
      r_list.append(int3(verts[0], new_vert, verts[2]));
      r_list.append(int3(new_vert, verts[1], verts[2]));
      break;
    }
  }
}

static void edge_subdivide(const std::array<float2, 2> &real_verts,
                           const Span<float2> linked_verts,
                           const int2 real_edge,
                           const Span<int> edge_indices,
                           const Span<float> edge_lengths,
                           const float2 centre,
                           const float radius,
                           const float max_length,
                           VectorSet<OrderedEdge> &r_all_edges,
                           VectorSet<OrderedEdge> &r_unique_edges)
{
}

static void gather_faces_to_split(const GroupedSpan<float2> verts_by_face_type,
                                  const float2 centre,
                                  const float radius,
                                  Vector<int> &r_faces_to_split)
{
  if (triangle_is_in_range(verts_by_face_type[0][0],
                           verts_by_face_type[0][1],
                           verts_by_face_type[0][2],
                           centre,
                           radius))
  {
    r_faces_to_split.append(0);
  }
  int offset = 1;
  for (const int face_i : verts_by_face_type[1].index_range()) {
    if (triangle_is_in_range(verts_by_face_type[0][0],
                             verts_by_face_type[0][1],
                             verts_by_face_type[1][face_i],
                             centre,
                             radius))
    {
      r_faces_to_split.append(offset + face_i);
    }
  }
  offset += verts_by_face_type[1].size();
  for (const int face_i : verts_by_face_type[2].index_range()) {
    if (triangle_is_in_range(verts_by_face_type[0][1],
                             verts_by_face_type[0][2],
                             verts_by_face_type[2][face_i],
                             centre,
                             radius))
    {
      r_faces_to_split.append(offset + face_i);
    }
  }
  offset += verts_by_face_type[2].size();
  for (const int face_i : verts_by_face_type[3].index_range()) {
    if (triangle_is_in_range(verts_by_face_type[0][2],
                             verts_by_face_type[0][0],
                             verts_by_face_type[3][face_i],
                             centre,
                             radius))
    {
      r_faces_to_split.append(offset + face_i);
    }
  }
}

static void gather_edges_to_split(const Span<int> faces,
                                  const GroupedSpan<float3> real_verts_by_face_type,
                                  const float max_length,
                                  Vector<int> &r_edges_to_split)
{
  float max_length_iter = std::numeric_limits<float>::lowest();

  bool ab_added = false;
  bool bc_added = false;
  bool ca_added = false;

  for (const int face_i : faces) {
    if (face_i == 0) {
      const float ab_length = math::distance_squared(real_verts_by_face_type[0][0],
                                                     real_verts_by_face_type[0][1]);
      if (max_length_iter == ab_length) {
        ab_added = true;
        r_edges_to_split.append(0);
        // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
      }
      else if (max_length_iter < ab_length) {
        ab_added = true;
        r_edges_to_split.clear();
        max_length_iter = ab_length;
        r_edges_to_split.append(0);
        // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
      }
      const float bc_length = math::distance_squared(real_verts_by_face_type[0][1],
                                                     real_verts_by_face_type[0][2]);
      if (max_length_iter == bc_length) {
        bc_added = true;
        r_edges_to_split.append(1);
        // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
      }
      else if (max_length_iter < bc_length) {
        bc_added = true;
        r_edges_to_split.clear();
        max_length_iter = bc_length;
        r_edges_to_split.append(1);
        // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
      }
      const float ca_length = math::distance_squared(real_verts_by_face_type[0][2],
                                                     real_verts_by_face_type[0][0]);
      if (max_length_iter == ca_length) {
        ca_added = true;
        r_edges_to_split.append(2);
        // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
      }
      else if (max_length_iter < ca_length) {
        ca_added = true;
        r_edges_to_split.clear();
        max_length_iter = ca_length;
        r_edges_to_split.append(2);
        // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
      }
    }

    const IndexRange ab_faces = real_verts_by_face_type[1].index_range();
    int offset = 1;
    int edge_offset = 3;
    if (ab_faces.contains(face_i - offset)) {
      const int side_edges_offset = (face_i - offset) * 2;
      const float ad_length = math::distance_squared(real_verts_by_face_type[0][0],
                                                     real_verts_by_face_type[1][face_i - offset]);
      if (max_length_iter == ad_length) {
        r_edges_to_split.append(edge_offset + side_edges_offset + 0);
        // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
      }
      else if (max_length_iter < ad_length) {
        r_edges_to_split.clear();
        max_length_iter = ad_length;
        r_edges_to_split.append(edge_offset + side_edges_offset + 0);
        // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
      }
      const float bd_length = math::distance_squared(real_verts_by_face_type[0][1],
                                                     real_verts_by_face_type[1][face_i - offset]);
      if (max_length_iter == bd_length) {
        r_edges_to_split.append(edge_offset + side_edges_offset + 1);
        // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
      }
      else if (max_length_iter < bd_length) {
        r_edges_to_split.clear();
        max_length_iter = bd_length;
        r_edges_to_split.append(edge_offset + side_edges_offset + 1);
        // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
      }

      if (!ab_added) {
        ab_added = true;
        const float ab_length = math::distance_squared(real_verts_by_face_type[0][0],
                                                       real_verts_by_face_type[0][1]);
        if (max_length_iter == ab_length) {
          r_edges_to_split.append(0);
          // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
        }
        else if (max_length_iter < ab_length) {
          r_edges_to_split.clear();
          max_length_iter = ab_length;
          r_edges_to_split.append(0);
          // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
        }
      }
    }
    offset += ab_faces.size();
    edge_offset += ab_faces.size() * 2;

    const IndexRange bc_faces = real_verts_by_face_type[2].index_range();
    if (bc_faces.contains(face_i - offset)) {
      const int side_edges_offset = (face_i - offset) * 2;
      const float de_length = math::distance_squared(real_verts_by_face_type[0][1],
                                                     real_verts_by_face_type[2][face_i - offset]);
      if (max_length_iter == de_length) {
        r_edges_to_split.append(edge_offset + side_edges_offset + 0);
        // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
      }
      else if (max_length_iter < de_length) {
        r_edges_to_split.clear();
        max_length_iter = de_length;
        r_edges_to_split.append(edge_offset + side_edges_offset + 0);
        // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
      }
      const float ce_length = math::distance_squared(real_verts_by_face_type[0][2],
                                                     real_verts_by_face_type[2][face_i - offset]);
      if (max_length_iter == ce_length) {
        r_edges_to_split.append(edge_offset + side_edges_offset + 1);
        // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
      }
      else if (max_length_iter < ce_length) {
        r_edges_to_split.clear();
        max_length_iter = ce_length;
        r_edges_to_split.append(edge_offset + side_edges_offset + 1);
        // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
      }
      if (!bc_added) {
        bc_added = true;
        const float bc_length = math::distance_squared(real_verts_by_face_type[0][1],
                                                       real_verts_by_face_type[0][2]);
        if (max_length_iter == bc_length) {
          r_edges_to_split.append(1);
          // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
        }
        else if (max_length_iter < bc_length) {
          r_edges_to_split.clear();
          max_length_iter = bc_length;
          r_edges_to_split.append(1);
          // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
        }
      }
    }
    offset += bc_faces.size();
    edge_offset += bc_faces.size() * 2;

    const IndexRange ca_faces = real_verts_by_face_type[3].index_range();
    if (ca_faces.contains(face_i - offset)) {
      const int side_edges_offset = (face_i - offset) * 2;
      const float cf_length = math::distance_squared(real_verts_by_face_type[0][2],
                                                     real_verts_by_face_type[3][face_i - offset]);
      if (max_length_iter == cf_length) {
        r_edges_to_split.append(edge_offset + side_edges_offset + 0);
        // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
      }
      else if (max_length_iter < cf_length) {
        r_edges_to_split.clear();
        max_length_iter = cf_length;
        r_edges_to_split.append(edge_offset + side_edges_offset + 0);
        // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
      }
      const float af_length = math::distance_squared(real_verts_by_face_type[0][0],
                                                     real_verts_by_face_type[3][face_i - offset]);
      if (max_length_iter == af_length) {
        r_edges_to_split.append(edge_offset + side_edges_offset + 1);
        // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
      }
      else if (max_length_iter < af_length) {
        r_edges_to_split.clear();
        max_length_iter = af_length;
        r_edges_to_split.append(edge_offset + side_edges_offset + 1);
        // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
      }

      if (!ca_added) {
        ca_added = true;
        const float ca_length = math::distance_squared(real_verts_by_face_type[0][2],
                                                       real_verts_by_face_type[0][0]);
        if (max_length_iter == ca_length) {
          r_edges_to_split.append(2);
          // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
        }
        else if (max_length_iter < ca_length) {
          r_edges_to_split.clear();
          max_length_iter = ca_length;
          r_edges_to_split.append(2);
          // std::cout << "\t\t\t << " << r_edges_to_split.as_span() << ": " << AT << ";\n";
        }
      }
    }
  }

  if (max_length_iter < max_length) {
    r_edges_to_split.clear();
  }

  // std::cout << "\t" << __func__ << ":\n";
  // std::cout << "\t\t" << max_length_iter << ";\n";
  // std::cout << "\t\t" << r_edges_to_split.as_span() << ";\n";
}

static void face_subdivide(const std::array<int, 5> &vert_offsets,
                           const Span<float2> verts,
                           const Span<float3> real_verts,
                           const float2 centre,
                           const float radius,
                           const float max_length,
                           const int3 face_verts,
                           VectorSet<OrderedEdge> &r_all_edges,
                           VectorSet<OrderedEdge> &r_unique_edges)
{
  Vector<std::array<int, 5>> offsets_stack = {vert_offsets};
  Vector<Vector<float2>> verts_stack = {verts};
  Vector<Vector<float3>> real_verts_stack = {real_verts};
  Vector<int3> tris = {face_verts};

  while (!offsets_stack.is_empty()) {
    std::cout << "Stack call." << std::endl;
    BLI_assert(offsets_stack.size() == verts_stack.size());
    BLI_assert(offsets_stack.size() == real_verts_stack.size());
    BLI_assert(offsets_stack.size() == tris.size());

    const std::array<int, 5> offset_data = offsets_stack.pop_last();
    Vector<float2> verts = verts_stack.pop_last();
    Vector<float3> real_verts = real_verts_stack.pop_last();
    const int3 face_verts = tris.pop_last();

    std::cout << " -" << offset_data << std::endl;
    std::cout << " -" << verts.as_span() << std::endl;
    std::cout << " -" << real_verts.as_span() << std::endl;
    std::cout << " -" << face_verts << std::endl;

    BLI_assert(offset_data[1] == 3);

    const OffsetIndices<int> vert_offset(offset_data);
    const GroupedSpan<float2> verts_by_face_type(vert_offset, verts);
    const GroupedSpan<float3> real_verts_by_face_type(vert_offset, real_verts);

    Vector<int> faces_to_split;
    gather_faces_to_split(verts_by_face_type, centre, radius, faces_to_split);
    std::cout << "  - " << faces_to_split.as_span() << ";\n";
    if (faces_to_split.is_empty()) {
      continue;
    }

    Vector<int> edges_to_split;
    gather_edges_to_split(faces_to_split, real_verts_by_face_type, max_length, edges_to_split);
    std::cout << "  - " << edges_to_split.as_span() << ";\n";
    if (edges_to_split.is_empty()) {
      continue;
    }

    if (edges_to_split.size() > 1) {
      /* TODO. */
      BLI_assert_unreachable();
      break;
    }

    const std::array<int2, 3> edges = {int2{0, 1}, int2{1, 2}, int2{2, 0}};

    const int edge_i = edges_to_split[0];

    const bool split_this_tris = ELEM(edge_i, 0, 1, 2);
    if (!split_this_tris) {
      const int side_index = (edge_i - 3) / 2 + 3;

      const int face_side = vert_offset[1].contains(side_index) ?
                                0 :
                                (vert_offset[2].contains(side_index) ? 1 : 2);
      BLI_assert(vert_offset[face_side + 1].contains(side_index));
      const int split_face_i = vert_offset[face_side + 1].start() - side_index;
      const int split_face_side_i = (edge_i - 3) % 2;
      BLI_assert(split_face_i >= 0);
      BLI_assert(ELEM(split_face_side_i, 0, 1));

      const int2 split_edge = edges[face_side];
      const int split_a_vert_i = split_edge[split_face_side_i];
      const int split_target_vert_i = split_face_i;

      const float2 mid = math::midpoint(verts_by_face_type[0][split_a_vert_i],
                                        verts_by_face_type[face_side + 1][split_target_vert_i]);
      const float3 real_mid = math::midpoint(
          real_verts_by_face_type[0][split_a_vert_i],
          real_verts_by_face_type[face_side + 1][split_target_vert_i]);

      const int3 next_face_verts = face_verts;
      Vector<float2> next_verts = verts;
      Vector<float3> next_real_verts = real_verts;
      std::array<int, 5> next_offset_data;
      MutableSpan(next_offset_data).copy_from(offset_data);

      next_verts[vert_offset[face_side + 1][split_face_i]] = mid;
      next_real_verts[vert_offset[face_side + 1][split_face_i]] = real_mid;

      offsets_stack.append(next_offset_data);
      verts_stack.append(next_verts);
      real_verts_stack.append(next_real_verts);
      tris.append(next_face_verts);
      continue;
    }

    const int2 edge = edges[edge_i];
    const int new_vert_i = r_all_edges.index_of_or_add({face_verts[edge[0]], face_verts[edge[1]]});

    int3 left_face_verts = face_verts;
    int3 right_face_verts = face_verts;
    left_face_verts[edge[0]] = new_vert_i;
    right_face_verts[edge[1]] = new_vert_i;

    Vector<float2> left_verts;
    Vector<float2> right_verts;

    Vector<float3> left_real_verts;
    Vector<float3> right_real_verts;

    std::array<int, 5> left_offset_data;
    std::array<int, 5> right_offset_data;
    left_offset_data[0] = 3;
    right_offset_data[0] = 3;

    const float2 mid = math::midpoint(verts_by_face_type[0][edge[0]],
                                      verts_by_face_type[0][edge[1]]);
    const float3 real_mid = math::midpoint(real_verts_by_face_type[0][edge[0]],
                                           real_verts_by_face_type[0][edge[1]]);

    switch (edge_i) {
      case 0: {
        left_verts.append(mid);
        left_verts.append(verts_by_face_type[0][1]);
        left_verts.append(verts_by_face_type[0][2]);
        left_verts.extend(verts_by_face_type[1]);
        left_verts.extend(verts_by_face_type[2]);
        left_verts.append(verts_by_face_type[0][0]);

        left_offset_data[1] = verts_by_face_type[1].size();
        left_offset_data[2] = verts_by_face_type[2].size();
        left_offset_data[3] = 1;

        right_verts.append(verts_by_face_type[0][0]);
        right_verts.append(mid);
        right_verts.append(verts_by_face_type[0][2]);
        right_verts.extend(verts_by_face_type[1]);
        right_verts.append(verts_by_face_type[0][1]);
        right_verts.extend(verts_by_face_type[3]);

        right_offset_data[1] = verts_by_face_type[1].size();
        right_offset_data[2] = 1;
        right_offset_data[3] = verts_by_face_type[3].size();

        left_real_verts.append(real_mid);
        left_real_verts.append(real_verts_by_face_type[0][1]);
        left_real_verts.append(real_verts_by_face_type[0][2]);
        left_real_verts.extend(real_verts_by_face_type[1]);
        left_real_verts.extend(real_verts_by_face_type[2]);
        left_real_verts.append(real_verts_by_face_type[0][0]);

        right_real_verts.append(real_verts_by_face_type[0][0]);
        right_real_verts.append(real_mid);
        right_real_verts.append(real_verts_by_face_type[0][2]);
        right_real_verts.extend(real_verts_by_face_type[1]);
        right_real_verts.append(real_verts_by_face_type[0][1]);
        right_real_verts.extend(real_verts_by_face_type[3]);
        break;
      }
      case 1: {
        left_verts.append(verts_by_face_type[0][0]);
        left_verts.append(verts_by_face_type[0][1]);
        left_verts.append(mid);
        left_verts.extend(verts_by_face_type[1]);
        left_verts.extend(verts_by_face_type[2]);
        left_verts.append(verts_by_face_type[0][2]);

        left_offset_data[1] = verts_by_face_type[1].size();
        left_offset_data[2] = verts_by_face_type[2].size();
        left_offset_data[3] = 1;

        right_verts.append(verts_by_face_type[0][0]);
        right_verts.append(mid);
        right_verts.append(verts_by_face_type[0][2]);
        right_verts.append(verts_by_face_type[0][1]);
        right_verts.extend(verts_by_face_type[2]);
        right_verts.extend(verts_by_face_type[3]);

        right_offset_data[1] = 1;
        right_offset_data[2] = verts_by_face_type[2].size();
        right_offset_data[3] = verts_by_face_type[3].size();

        left_real_verts.append(real_verts_by_face_type[0][0]);
        left_real_verts.append(real_verts_by_face_type[0][1]);
        left_real_verts.append(real_mid);
        left_real_verts.extend(real_verts_by_face_type[1]);
        left_real_verts.extend(real_verts_by_face_type[2]);
        left_real_verts.append(real_verts_by_face_type[0][2]);

        right_real_verts.append(real_verts_by_face_type[0][0]);
        right_real_verts.append(real_mid);
        right_real_verts.append(real_verts_by_face_type[0][2]);
        right_real_verts.append(real_verts_by_face_type[0][1]);
        right_real_verts.extend(real_verts_by_face_type[2]);
        right_real_verts.extend(real_verts_by_face_type[3]);
        break;
      }
      case 2: {
        left_verts.append(verts_by_face_type[0][0]);
        left_verts.append(verts_by_face_type[0][1]);
        left_verts.append(mid);
        left_verts.extend(verts_by_face_type[1]);
        left_verts.append(verts_by_face_type[0][2]);
        left_verts.extend(verts_by_face_type[3]);

        left_offset_data[1] = verts_by_face_type[1].size();
        left_offset_data[2] = 1;
        left_offset_data[3] = verts_by_face_type[3].size();

        right_verts.append(mid);
        right_verts.append(verts_by_face_type[0][1]);
        right_verts.append(verts_by_face_type[0][2]);
        right_verts.append(verts_by_face_type[0][0]);
        right_verts.extend(verts_by_face_type[2]);
        right_verts.extend(verts_by_face_type[3]);

        right_offset_data[1] = 1;
        right_offset_data[2] = verts_by_face_type[2].size();
        right_offset_data[3] = verts_by_face_type[3].size();

        left_real_verts.append(real_verts_by_face_type[0][0]);
        left_real_verts.append(real_verts_by_face_type[0][1]);
        left_real_verts.append(real_mid);
        left_real_verts.extend(real_verts_by_face_type[1]);
        left_real_verts.append(real_verts_by_face_type[0][2]);
        left_real_verts.extend(real_verts_by_face_type[3]);

        right_real_verts.append(real_mid);
        right_real_verts.append(real_verts_by_face_type[0][1]);
        right_real_verts.append(real_verts_by_face_type[0][2]);
        right_real_verts.append(real_verts_by_face_type[0][0]);
        right_real_verts.extend(real_verts_by_face_type[2]);
        right_real_verts.extend(real_verts_by_face_type[3]);
        break;
      }
    }

    // std::cout << "\t left_offset_data" << left_offset_data << ";\n";
    // std::cout << "\t right_offset_data" << right_offset_data << ";\n";

    offset_indices::accumulate_counts_to_offsets(left_offset_data);
    offset_indices::accumulate_counts_to_offsets(right_offset_data);

    offsets_stack.append(left_offset_data);
    offsets_stack.append(right_offset_data);

    verts_stack.append(left_verts);
    verts_stack.append(right_verts);

    real_verts_stack.append(left_real_verts);
    real_verts_stack.append(right_real_verts);

    tris.append(left_face_verts);
    tris.append(right_face_verts);
  }

  std::cout << __func__ << ":\n";
  for (const OrderedEdge e : r_unique_edges) {
    std::cout << "  " << e << ", ";
  }
  std::cout << ";\n";
  for (const OrderedEdge e : r_all_edges) {
    std::cout << "  " << e << ", ";
  }
  std::cout << ";\n";
}

static void edge_subdivide_count(const float2 a_vert,
                                 const float2 b_vert,
                                 const float2 c_vert,
                                 const float2 d_vert,
                                 const float2 centre,
                                 const float radius,
                                 const float max_length,
                                 int &r_total_verts_in)
{
  // std::cout << "Face Size:\n";
  Vector<std::array<float2, 4>> stack = {{a_vert, b_vert, c_vert, d_vert}};
  while (!stack.is_empty()) {
    const std::array<float2, 4> edge_and_tris = stack.pop_last();
    // std::cout << "  " << edge_and_tris << ";\n";

    const float ab_length = math::distance_squared(edge_and_tris[0], edge_and_tris[1]);
    if (ab_length < max_length) {
      continue;
    }
    // std::cout << "  is enought large" << ";\n";

    const bool left_is_affected = len_squared_to_tris(
                                      {edge_and_tris[0], edge_and_tris[1], edge_and_tris[2]},
                                      centre) <= radius;
    const bool right_is_affected = len_squared_to_tris(
                                       {edge_and_tris[0], edge_and_tris[1], edge_and_tris[3]},
                                       centre) <= radius;
    if (!(left_is_affected || right_is_affected)) {
      continue;
    }
    // std::cout << "  is_affected" << ";\n";

    const float ac_length = math::distance_squared(edge_and_tris[0], edge_and_tris[2]);
    const float bc_length = math::distance_squared(edge_and_tris[1], edge_and_tris[2]);
    const float abc_max = math::max(ac_length, bc_length);

    const float ad_length = math::distance_squared(edge_and_tris[0], edge_and_tris[3]);
    const float bd_length = math::distance_squared(edge_and_tris[1], edge_and_tris[3]);
    const float abd_max = math::max(ad_length, bd_length);

    if (math::max(abd_max, abc_max) < max_length) {
      continue;
    }
    // std::cout << "  Is in range" << ";\n";

    if (ab_length > abc_max && ab_length > abd_max) {
      const float2 mid = math::midpoint(edge_and_tris[0], edge_and_tris[1]);
      stack.append({edge_and_tris[0], mid, edge_and_tris[2], edge_and_tris[3]});
      stack.append({mid, edge_and_tris[1], edge_and_tris[2], edge_and_tris[3]});
      r_total_verts_in++;
      // std::cout << "  Is AB Split" << ";\n";
      continue;
    }

    if (abc_max > abd_max) {
      // std::cout << "  Is ABC split" << ";\n";
      if (ac_length > bc_length) {
        const float2 mid = math::midpoint(edge_and_tris[0], edge_and_tris[2]);
        stack.append({edge_and_tris[0], edge_and_tris[1], mid, edge_and_tris[3]});
      }
      else {
        const float2 mid = math::midpoint(edge_and_tris[1], edge_and_tris[2]);
        stack.append({edge_and_tris[0], edge_and_tris[1], mid, edge_and_tris[3]});
      }
    }
    else {
      // std::cout << "  Is ABD split" << ";\n";
      if (ad_length > bd_length) {
        const float2 mid = math::midpoint(edge_and_tris[0], edge_and_tris[3]);
        stack.append({edge_and_tris[0], edge_and_tris[1], edge_and_tris[2], mid});
      }
      else {
        const float2 mid = math::midpoint(edge_and_tris[1], edge_and_tris[3]);
        stack.append({edge_and_tris[0], edge_and_tris[1], edge_and_tris[2], mid});
      }
    }
  }
}

static void edge_subdivide_uv(const float2 a_vert,
                              const float2 b_vert,
                              const float2 c_vert,
                              const float2 d_vert,
                              const float2 centre,
                              const float radius,
                              const float max_length,
                              MutableSpan<float> r_factors)
{
  int r_total_verts_in = 0;
  // std::cout << "Face Size:\n";
  Vector<float2> factor_range_stack = {{0.0f, 1.0f}};
  Vector<std::array<float2, 4>> stack = {{a_vert, b_vert, c_vert, d_vert}};
  while (!stack.is_empty()) {
    BLI_assert(factor_range_stack.size() == stack.size());
    const std::array<float2, 4> edge_and_tris = stack.pop_last();
    const float2 edge_factor = factor_range_stack.pop_last();
    // std::cout << "  " << edge_and_tris << ";\n";

    const float ab_length = math::distance_squared(edge_and_tris[0], edge_and_tris[1]);
    if (ab_length < max_length) {
      continue;
    }
    // std::cout << "  is enought large" << ";\n";

    const bool left_is_affected = len_squared_to_tris(
                                      {edge_and_tris[0], edge_and_tris[1], edge_and_tris[2]},
                                      centre) <= radius;
    const bool right_is_affected = len_squared_to_tris(
                                       {edge_and_tris[0], edge_and_tris[1], edge_and_tris[3]},
                                       centre) <= radius;
    if (!(left_is_affected || right_is_affected)) {
      continue;
    }
    // std::cout << "  is_affected" << ";\n";

    const float ac_length = math::distance_squared(edge_and_tris[0], edge_and_tris[2]);
    const float bc_length = math::distance_squared(edge_and_tris[1], edge_and_tris[2]);
    const float abc_max = math::max(ac_length, bc_length);

    const float ad_length = math::distance_squared(edge_and_tris[0], edge_and_tris[3]);
    const float bd_length = math::distance_squared(edge_and_tris[1], edge_and_tris[3]);
    const float abd_max = math::max(ad_length, bd_length);

    if (math::max(abc_max, abd_max) < max_length) {
      continue;
    }
    // std::cout << "  Is in range" << ";\n";

    if (ab_length > abc_max && ab_length > abd_max) {
      const float2 mid = math::midpoint(edge_and_tris[0], edge_and_tris[1]);
      stack.append({edge_and_tris[0], mid, edge_and_tris[2], edge_and_tris[3]});
      stack.append({mid, edge_and_tris[1], edge_and_tris[2], edge_and_tris[3]});
      const float factor_mid = math::midpoint(edge_factor[0], edge_factor[1]);
      factor_range_stack.append({edge_factor[0], factor_mid});
      factor_range_stack.append({factor_mid, edge_factor[1]});
      r_factors[r_total_verts_in] = factor_mid;
      r_total_verts_in++;
      // std::cout << "  Is AB Split" << ";\n";
      continue;
    }

    if (abc_max > abd_max) {
      // std::cout << "  Is ABC split" << ";\n";
      if (ac_length > bc_length) {
        const float2 mid = math::midpoint(edge_and_tris[0], edge_and_tris[2]);
        stack.append({edge_and_tris[0], edge_and_tris[1], mid, edge_and_tris[3]});
        factor_range_stack.append(edge_factor);
      }
      else {
        const float2 mid = math::midpoint(edge_and_tris[1], edge_and_tris[2]);
        stack.append({edge_and_tris[0], edge_and_tris[1], mid, edge_and_tris[3]});
        factor_range_stack.append(edge_factor);
      }
    }
    else {
      // std::cout << "  Is ABD split" << ";\n";
      if (ad_length > bd_length) {
        const float2 mid = math::midpoint(edge_and_tris[0], edge_and_tris[3]);
        stack.append({edge_and_tris[0], edge_and_tris[1], edge_and_tris[2], mid});
        factor_range_stack.append(edge_factor);
      }
      else {
        const float2 mid = math::midpoint(edge_and_tris[1], edge_and_tris[3]);
        stack.append({edge_and_tris[0], edge_and_tris[1], edge_and_tris[2], mid});
        factor_range_stack.append(edge_factor);
      }
    }
  }
}

static int dominant_axis(const float3 a)
{
  return ((a.x > a.y) ? ((a.x > a.z) ? 0 : 2) : ((a.y > a.z) ? 1 : 2));
}

struct TrisEdge {
  float2 a;
  float2 b;
  float2 c;
  float2 d;

  int vert_a;
  int vert_b;
};

static void edge_subdivide_verts(const float2 a_vert,
                                 const float2 b_vert,
                                 const float2 c_vert,
                                 const float2 d_vert,
                                 const float2 centre,
                                 const float radius,
                                 const float max_length,
                                 const int2 edge,
                                 const IndexRange verts_range,
                                 MutableSpan<int2> r_verts)
{
  BLI_assert(!verts_range.is_empty());

  int r_total_verts_in = 0;
  int edge_iter = 0;
  Vector<TrisEdge> stack = {{a_vert, b_vert, c_vert, d_vert, edge[0], edge[1]}};
  while (!stack.is_empty()) {
    const TrisEdge edge_and_tris = stack.pop_last();
    // std::cout << "  " << edge_and_tris << ";\n";

    const float ab_length = math::distance_squared(edge_and_tris.a, edge_and_tris.b);
    if (ab_length < max_length) {
      r_verts[edge_iter] = int2(edge_and_tris.vert_a, edge_and_tris.vert_b);
      edge_iter++;
      continue;
    }
    // std::cout << "  is enought large" << ";\n";

    const bool left_is_affected = len_squared_to_tris(
                                      {edge_and_tris.a, edge_and_tris.b, edge_and_tris.c},
                                      centre) <= radius;
    const bool right_is_affected = len_squared_to_tris(
                                       {edge_and_tris.a, edge_and_tris.b, edge_and_tris.d},
                                       centre) <= radius;
    if (!(left_is_affected || right_is_affected)) {
      r_verts[edge_iter] = int2(edge_and_tris.vert_a, edge_and_tris.vert_b);
      edge_iter++;
      continue;
    }
    // std::cout << "  is_affected" << ";\n";

    const float ac_length = math::distance_squared(edge_and_tris.a, edge_and_tris.c);
    const float bc_length = math::distance_squared(edge_and_tris.b, edge_and_tris.c);
    const float abc_max = math::max(ac_length, bc_length);

    const float ad_length = math::distance_squared(edge_and_tris.a, edge_and_tris.d);
    const float bd_length = math::distance_squared(edge_and_tris.b, edge_and_tris.d);
    const float abd_max = math::max(ad_length, bd_length);

    if (math::max(abc_max, abd_max) < max_length) {
      r_verts[edge_iter] = int2(edge_and_tris.vert_a, edge_and_tris.vert_b);
      edge_iter++;
      continue;
    }
    // std::cout << "  Is in range" << ";\n";

    if (ab_length > abc_max && ab_length > abd_max) {
      const float2 mid = math::midpoint(edge_and_tris.a, edge_and_tris.b);
      const int mid_index = verts_range[r_total_verts_in];
      r_total_verts_in++;
      stack.append({edge_and_tris.a,
                    mid,
                    edge_and_tris.c,
                    edge_and_tris.d,
                    edge_and_tris.vert_a,
                    mid_index});
      stack.append({mid,
                    edge_and_tris.b,
                    edge_and_tris.c,
                    edge_and_tris.d,
                    mid_index,
                    edge_and_tris.vert_b});
      // std::cout << "  Is AB Split" << ";\n";
      continue;
    }

    if (abc_max > abd_max) {
      // std::cout << "  Is ABC split" << ";\n";
      if (ac_length > bc_length) {
        const float2 mid = math::midpoint(edge_and_tris.a, edge_and_tris.c);
        stack.append({edge_and_tris.a,
                      edge_and_tris.b,
                      mid,
                      edge_and_tris.d,
                      edge_and_tris.vert_a,
                      edge_and_tris.vert_b});
      }
      else {
        const float2 mid = math::midpoint(edge_and_tris.b, edge_and_tris.c);
        stack.append({edge_and_tris.a,
                      edge_and_tris.b,
                      mid,
                      edge_and_tris.d,
                      edge_and_tris.vert_a,
                      edge_and_tris.vert_b});
      }
    }
    else {
      // std::cout << "  Is ABD split" << ";\n";
      if (ad_length > bd_length) {
        const float2 mid = math::midpoint(edge_and_tris.a, edge_and_tris.d);
        stack.append({edge_and_tris.a,
                      edge_and_tris.b,
                      edge_and_tris.c,
                      mid,
                      edge_and_tris.vert_a,
                      edge_and_tris.vert_b});
      }
      else {
        const float2 mid = math::midpoint(edge_and_tris.b, edge_and_tris.d);
        stack.append({edge_and_tris.a,
                      edge_and_tris.b,
                      edge_and_tris.c,
                      mid,
                      edge_and_tris.vert_a,
                      edge_and_tris.vert_b});
      }
    }
  }
}

static void face_subdivide(const std::array<float2, 6> &verts_list,
                           const std::array<int, 9> &edge_indices,
                           const int3 face_verts,
                           const float2 centre,
                           const float radius,
                           const float max_length,
                           const IndexRange verts_range,
                           const IndexRange ab_points_range,
                           const IndexRange bc_points_range,
                           const IndexRange ca_points_range,
                           VectorSet<OrderedEdge> &r_face_edges,
                           VectorSet<OrderedEdge> &r_unique_face_edges)
{
  BLI_assert(verts_range.intersect(ab_points_range).is_empty());
  BLI_assert(verts_range.intersect(bc_points_range).is_empty());
  BLI_assert(verts_range.intersect(ca_points_range).is_empty());

  BLI_assert(ab_points_range.intersect(bc_points_range).is_empty());
  BLI_assert(ab_points_range.intersect(ca_points_range).is_empty());

  BLI_assert(bc_points_range.intersect(ab_points_range).is_empty());
  BLI_assert(bc_points_range.intersect(ca_points_range).is_empty());

  BLI_assert(ca_points_range.intersect(ab_points_range).is_empty());
  BLI_assert(ca_points_range.intersect(bc_points_range).is_empty());

  static const int3 abc_verts(0, 1, 2);

  [[maybe_unused]] VectorSet<OrderedEdge> ab_edges;
  [[maybe_unused]] VectorSet<OrderedEdge> bc_edges;
  [[maybe_unused]] VectorSet<OrderedEdge> ca_edges;

  Vector<std::array<float2, 6>> vertices = {verts_list};
  Vector<int3> tri_indices = {face_verts};
  Vector<int8_t> is_real_edges = {EdgeState::ab_is_real_edge | EdgeState::bc_is_real_edge |
                                  EdgeState::ca_is_real_edge};

  while (!vertices.is_empty()) {
    // std::cout << "  " << vertices.size() << ";\n";
    BLI_assert(vertices.size() == is_real_edges.size());
    BLI_assert(vertices.size() == tri_indices.size());
    const std::array<float2, 6> verts = vertices.pop_last();
    const int8_t edges_is_real = is_real_edges.pop_last();
    const int3 face_indices = tri_indices.pop_last();

    // std::cout << "    verts: " << verts << ";\n";
    // std::cout << "    edges_is_real: " << edges_is_real << ";\n";
    // std::cout << "    face_indices: " << face_indices << ";\n";

    const float2 &vert_a = verts[abc_verts[0]];
    const float2 &vert_b = verts[abc_verts[1]];
    const float2 &vert_c = verts[abc_verts[2]];

    const bool abc_is_affected = triangle_is_in_range(vert_a, vert_b, vert_c, centre, radius);
    if (!abc_is_affected) {
      // std::cout << "    - Is not affected;\n";
      // TODO: Handle neighboards...
      r_unique_face_edges.add({face_indices[0], face_indices[1]});
      r_unique_face_edges.add({face_indices[1], face_indices[2]});
      r_unique_face_edges.add({face_indices[2], face_indices[0]});
      continue;
    }
    // std::cout << "    - Is affected;\n";

    const std::optional<int2> edge_to_split = largest_side_to_split(
        vert_a, vert_b, vert_c, edge_indices, max_length);
    if (!edge_to_split.has_value()) {
      // std::cout << "    - No edge to split;\n";
      r_unique_face_edges.add({face_indices[0], face_indices[1]});
      r_unique_face_edges.add({face_indices[1], face_indices[2]});
      r_unique_face_edges.add({face_indices[2], face_indices[0]});
      continue;
    }
    // std::cout << "    Edge to split: " << (*edge_to_split) << "\n";

    split_edge_for_vert(verts, *edge_to_split, vertices);
    split_edge_for_state(edges_is_real, *edge_to_split, is_real_edges);

    const int2 virtual_split_edge(face_indices[(*edge_to_split)[0]],
                                  face_indices[(*edge_to_split)[1]]);

    const int other_vert_for_split = exclusive_one(FaceVerts::abc, *edge_to_split);
    const bool split_of_real_edge = EdgeState::edge_state_for_vert[other_vert_for_split] &
                                    edges_is_real;
    if (UNLIKELY(split_of_real_edge)) {
      switch ((*edge_to_split)[0]) {
        case 0: {
          const int vert_i = ab_edges.index_of_or_add(virtual_split_edge);
          // std::cout << "    ." << ab_points_range << ": " << vert_i << ";\n";
          const int edge_vert = ab_points_range[vert_i];
          split_edge_for_indices(face_indices, edge_vert, *edge_to_split, tri_indices);
          break;
        }
        case 1: {
          const int vert_i = bc_edges.index_of_or_add(virtual_split_edge);
          // std::cout << "    ." << bc_points_range << ": " << vert_i << ";\n";
          const int edge_vert = bc_points_range[vert_i];
          split_edge_for_indices(face_indices, edge_vert, *edge_to_split, tri_indices);
          break;
        }
        case 2: {
          const int vert_i = ca_edges.index_of_or_add(virtual_split_edge);
          // std::cout << "    ." << ca_points_range << ": " << vert_i << ";\n";
          const int edge_vert = ca_points_range[vert_i];
          split_edge_for_indices(face_indices, edge_vert, *edge_to_split, tri_indices);
          break;
        }
      }
    }
    else {
      const int vert_i = r_face_edges.index_of_or_add(virtual_split_edge);
      const int face_vert = verts_range[vert_i];
      split_edge_for_indices(face_indices, face_vert, *edge_to_split, tri_indices);
    }
  }
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

  for (const int edge_i : IndexRange(src_mesh.edges_num)) {
    // const int2 edge = edges[edge_i];

    // const Span<int> edge_faces = edge_to_face_map[edge_i];

    //  const int3 left_verts = gather_tri(faces[edge_faces[0]], corner_verts);
    // const int3 right_verts = gather_tri(faces[edge_faces[1]], corner_verts);

    // BLI_assert(elem(left_verts, edge[0]) && elem(left_verts, edge[1]));
    // BLI_assert(elem(right_verts, edge[0]) && elem(right_verts, edge[1]));

    // const int a_vert = exclusive_one(left_verts, edge);
    // const int b_vert = exclusive_one(right_verts, edge);

    // std::cout << "Edge: " << edge_i << ";\n";

    int total_verts_in = 0; /*
     edge_subdivide_count(projection[edge[0]],
                          projection[edge[1]],
                          projection[a_vert],
                          projection[b_vert],
                          centre,
                          squared_radius,
                          squared_max_length,
                          total_verts_in);*/
    edge_total_verts[edge_i] = total_verts_in;
  }

  for (const int face_i : faces.index_range()) {
    const IndexRange face = faces[face_i];
    const int3 face_edges = gather_tri(face, corner_edges);
    const int3 face_verts = gather_tri(face, corner_verts);

    const int2 edge_ab = edges[face_edges[0]];
    const int2 edge_bc = edges[face_edges[1]];
    const int2 edge_ca = edges[face_edges[2]];

    BLI_assert(OrderedEdge(face_verts[0], face_verts[1]) == OrderedEdge(edge_ab));
    BLI_assert(OrderedEdge(face_verts[1], face_verts[2]) == OrderedEdge(edge_bc));
    BLI_assert(OrderedEdge(face_verts[2], face_verts[0]) == OrderedEdge(edge_ca));

    const Span<int> ab_faces = edge_to_face_map[face_edges[0]];
    const Span<int> bc_faces = edge_to_face_map[face_edges[1]];
    const Span<int> ca_faces = edge_to_face_map[face_edges[2]];

    std::array<int, 5> vert_offsets;
    vert_offsets[0] = 3;
    vert_offsets[1] = ab_faces.size() - 1;
    vert_offsets[2] = bc_faces.size() - 1;
    vert_offsets[3] = ca_faces.size() - 1;
    offset_indices::accumulate_counts_to_offsets(vert_offsets);

    Vector<float2> uv_verts;
    uv_verts.append(projection[face_verts[0]]);
    uv_verts.append(projection[face_verts[1]]);
    uv_verts.append(projection[face_verts[2]]);
    Vector<float3> verts;
    verts.append(src_positions[face_verts[0]]);
    verts.append(src_positions[face_verts[1]]);
    verts.append(src_positions[face_verts[2]]);
    for (const int other_face_i : ab_faces) {
      if (other_face_i != face_i) {
        const IndexRange other_face = faces[other_face_i];
        const int3 other_face_verts = gather_tri(other_face, corner_verts);
        const int d_vert = exclusive_one(other_face_verts, edge_ab);
        BLI_assert(!elem(face_verts, d_vert));
        uv_verts.append(projection[d_vert]);
        verts.append(src_positions[d_vert]);
      }
    }
    for (const int other_face_i : bc_faces) {
      if (other_face_i != face_i) {
        const IndexRange other_face = faces[other_face_i];
        const int3 other_face_verts = gather_tri(other_face, corner_verts);
        const int e_vert = exclusive_one(other_face_verts, edge_bc);
        BLI_assert(!elem(face_verts, e_vert));
        uv_verts.append(projection[e_vert]);
        verts.append(src_positions[e_vert]);
      }
    }
    for (const int other_face_i : ca_faces) {
      if (other_face_i != face_i) {
        const IndexRange other_face = faces[other_face_i];
        const int3 other_face_verts = gather_tri(other_face, corner_verts);
        const int f_vert = exclusive_one(other_face_verts, edge_ca);
        BLI_assert(!elem(face_verts, f_vert));
        uv_verts.append(projection[f_vert]);
        verts.append(src_positions[f_vert]);
      }
    }

    VectorSet<OrderedEdge> face_edges_set;
    VectorSet<OrderedEdge> result_face_edges_set;
    face_subdivide(vert_offsets,
                   uv_verts,
                   verts,
                   centre,
                   squared_radius,
                   squared_max_length,
                   face_verts,
                   face_edges_set,
                   result_face_edges_set);

    int total_verts_in = 0;
    int total_edges_in = 0;

    face_total_verts[face_i] = total_verts_in;
    face_total_edges[face_i] = total_edges_in;
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
  const IndexMask keeped_faces = changed_faces.complement(IndexMask(src_mesh.edges_num), memory);

  const OffsetIndices<int> subdive_edge_verts = offset_indices::accumulate_counts_to_offsets(
      edge_total_verts);
  const OffsetIndices<int> subdive_face_verts = offset_indices::accumulate_counts_to_offsets(
      face_total_verts);
  const OffsetIndices<int> subdive_face_edges = offset_indices::accumulate_counts_to_offsets(
      face_total_edges);

  const IndexRange verts_range(src_mesh.verts_num);
  const IndexRange edges_verts_range = verts_range.after(subdive_edge_verts.total_size());
  const IndexRange faces_verts_range = edges_verts_range.after(subdive_face_verts.total_size());

  const IndexRange edges_range(keeped_edges.size());
  const IndexRange edges_edges_range = edges_range.after(changed_edges.size() +
                                                         subdive_edge_verts.total_size());
  const IndexRange faces_edges_range = edges_edges_range.after(subdive_face_edges.total_size());

  const int total_verts = faces_verts_range.one_after_last();
  const int total_edges = faces_edges_range.one_after_last();

  Mesh *dst_mesh = BKE_mesh_new_nomain(total_verts, total_edges, 0, 0);
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh->attributes_for_write();

  MutableSpan<float3> dst_positions = dst_mesh->vert_positions_for_write();
  MutableSpan<int2> dst_edges = dst_mesh->edges_for_write();
  dst_edges.fill({0, 1});

  // bke::gather_attributes(src_attributes, bke::AttrDomain::Point, {}, {}, verts_range,
  // dst_attributes);
  dst_positions.take_front(src_mesh.verts_num).copy_from(src_positions);

  keeped_edges.foreach_index_optimized<int>(GrainSize(8092), [&](const int i, const int pos) {
    dst_edges[edge_total_verts[i] + i] = edges[i];
  });

  Array<float> edge_vertices_factor_weight(edges_verts_range.size());
  Array<float3> face_vertices_bary_weight(faces_verts_range.size());

  changed_edges.foreach_index([&](const int edge_i) {
    const int2 edge = edges[edge_i];

    const Span<int> edge_faces = edge_to_face_map[edge_i];

    const int3 left_verts = gather_tri(faces[edge_faces[0]], corner_verts);
    const int3 right_verts = gather_tri(faces[edge_faces[1]], corner_verts);

    BLI_assert(elem(left_verts, edge[0]) && elem(left_verts, edge[1]));
    BLI_assert(elem(right_verts, edge[0]) && elem(right_verts, edge[1]));

    const int a_vert = exclusive_one(left_verts, edge);
    const int b_vert = exclusive_one(right_verts, edge);

    // std::cout << "Edge: " << edge_i << ";\n";
    edge_subdivide_uv(
        projection[edge[0]],
        projection[edge[1]],
        projection[a_vert],
        projection[b_vert],
        centre,
        squared_radius,
        squared_max_length,
        edge_vertices_factor_weight.as_mutable_span().slice(subdive_edge_verts[edge_i]));
  });

  changed_edges.foreach_index([&](const int edge_i) {
    const int2 edge = edges[edge_i];
    const Span<int> edge_faces = edge_to_face_map[edge_i];

    const int3 left_verts = gather_tri(faces[edge_faces[0]], corner_verts);
    const int3 right_verts = gather_tri(faces[edge_faces[1]], corner_verts);

    BLI_assert(elem(left_verts, edge[0]) && elem(left_verts, edge[1]));
    BLI_assert(elem(right_verts, edge[0]) && elem(right_verts, edge[1]));

    const int a_vert = exclusive_one(left_verts, edge);
    const int b_vert = exclusive_one(right_verts, edge);

    const IndexRange edge_verts_range = subdive_edge_verts[edge_i];
    const IndexRange edge_verts_indices_range = edge_verts_range.shift(edges_verts_range.start());
    const IndexRange edge_edges_range = IndexRange::from_begin_size(
        edge_verts_range.start() + edge_i, edge_verts_range.size() + 1);

    edge_subdivide_verts(projection[edge[0]],
                         projection[edge[1]],
                         projection[a_vert],
                         projection[b_vert],
                         centre,
                         squared_radius,
                         squared_max_length,
                         edge,
                         edge_verts_indices_range,
                         dst_edges.slice(edge_edges_range));
  });

  changed_faces.foreach_index([&](const int face_i) {
    const IndexRange face = faces[face_i];
    const int3 face_edges = gather_tri(face, corner_edges);
    const int3 face_verts = gather_tri(face, corner_verts);

    const int2 edge_ab = edges[face_edges[0]];
    const int2 edge_bc = edges[face_edges[1]];
    const int2 edge_ca = edges[face_edges[2]];

    const Span<int> ab_faces = edge_to_face_map[face_edges[0]];
    const Span<int> bc_faces = edge_to_face_map[face_edges[1]];
    const Span<int> ca_faces = edge_to_face_map[face_edges[2]];

    const int face_ab = ab_faces[0] == face_i ? ab_faces[1] : ab_faces[0];
    const int face_bc = bc_faces[0] == face_i ? bc_faces[1] : bc_faces[0];
    const int face_ca = ca_faces[0] == face_i ? ca_faces[1] : ca_faces[0];

    const int3 abd_verts = gather_tri(faces[face_ab], corner_verts);
    const int3 bce_verts = gather_tri(faces[face_bc], corner_verts);
    const int3 caf_verts = gather_tri(faces[face_ca], corner_verts);

    BLI_assert(elem(abd_verts, edge_ab[0]) && elem(abd_verts, edge_ab[1]));
    BLI_assert(elem(bce_verts, edge_bc[0]) && elem(bce_verts, edge_bc[1]));
    BLI_assert(elem(caf_verts, edge_ca[0]) && elem(caf_verts, edge_ca[1]));

    const int a_vert = exclusive_one(face_verts, edge_bc);
    const int b_vert = exclusive_one(face_verts, edge_ca);
    const int c_vert = exclusive_one(face_verts, edge_ab);

    const int d_vert = exclusive_one(abd_verts, edge_ab);
    const int e_vert = exclusive_one(bce_verts, edge_bc);
    const int f_vert = exclusive_one(caf_verts, edge_ca);

    BLI_assert(!elem(face_verts, d_vert));
    BLI_assert(!elem(face_verts, e_vert));
    BLI_assert(!elem(face_verts, f_vert));

    const IndexRange verts_range = subdive_face_verts[face_i].shift(faces_verts_range.start());
    const IndexRange edges_range = subdive_face_edges[face_i].shift(faces_edges_range.start());

    const IndexRange ab_points_range = subdive_edge_verts[face_edges[0]].shift(
        edges_verts_range.start());
    const IndexRange bc_points_range = subdive_edge_verts[face_edges[1]].shift(
        edges_verts_range.start());
    const IndexRange ca_points_range = subdive_edge_verts[face_edges[2]].shift(
        edges_verts_range.start());

    const int3 face_abd_edge = gather_tri(faces[face_ab], corner_edges);
    const int3 face_bce_edge = gather_tri(faces[face_bc], corner_edges);
    const int3 face_caf_edge = gather_tri(faces[face_ca], corner_edges);

    Map<OrderedEdge, int, 9> edge_indices;
    edge_indices.add(edges[face_edges[0]], face_edges[0]);
    edge_indices.add(edges[face_edges[1]], face_edges[1]);
    edge_indices.add(edges[face_edges[2]], face_edges[2]);

    edge_indices.add(edges[face_abd_edge[0]], face_abd_edge[0]);
    edge_indices.add(edges[face_abd_edge[1]], face_abd_edge[1]);
    edge_indices.add(edges[face_abd_edge[2]], face_abd_edge[2]);

    edge_indices.add(edges[face_bce_edge[0]], face_bce_edge[0]);
    edge_indices.add(edges[face_bce_edge[1]], face_bce_edge[1]);
    edge_indices.add(edges[face_bce_edge[2]], face_bce_edge[2]);

    edge_indices.add(edges[face_caf_edge[0]], face_caf_edge[0]);
    edge_indices.add(edges[face_caf_edge[1]], face_caf_edge[1]);
    edge_indices.add(edges[face_caf_edge[2]], face_caf_edge[2]);

    const int2 edge_ad(face_verts[0], d_vert);
    const int2 edge_bd(face_verts[1], d_vert);

    const int2 edge_be(face_verts[1], e_vert);
    const int2 edge_ce(face_verts[2], e_vert);

    const int2 edge_cf(face_verts[2], f_vert);
    const int2 edge_af(face_verts[0], f_vert);

    const std::array<float2, 6> verts_list = {projection[a_vert],
                                              projection[b_vert],
                                              projection[c_vert],
                                              projection[d_vert],
                                              projection[e_vert],
                                              projection[f_vert]};
    const std::array<int, 9> edge_indices_list = {edge_indices.lookup(edge_ab),
                                                  edge_indices.lookup(edge_bc),
                                                  edge_indices.lookup(edge_ca),
                                                  edge_indices.lookup(edge_ad),
                                                  edge_indices.lookup(edge_bd),
                                                  edge_indices.lookup(edge_be),
                                                  edge_indices.lookup(edge_ce),
                                                  edge_indices.lookup(edge_cf),
                                                  edge_indices.lookup(edge_af)};

    const IndexRange points_range = subdive_face_verts[face_i].shift(
        src_mesh.verts_num + subdive_edge_verts.total_size());
    MutableSpan<float3> face_verts_bary_weight = face_vertices_bary_weight.as_mutable_span().slice(
        subdive_face_verts[face_i]);
  });

  changed_faces.foreach_index([&](const int face_i) {
    const IndexRange face = faces[face_i];
    const int3 face_edges = gather_tri(face, corner_edges);
    const int3 face_verts = gather_tri(face, corner_verts);

    const int2 edge_ab = edges[face_edges[0]];
    const int2 edge_bc = edges[face_edges[1]];
    const int2 edge_ca = edges[face_edges[2]];

    const Span<int> ab_faces = edge_to_face_map[face_edges[0]];
    const Span<int> bc_faces = edge_to_face_map[face_edges[1]];
    const Span<int> ca_faces = edge_to_face_map[face_edges[2]];

    const int face_ab = ab_faces[0] == face_i ? ab_faces[1] : ab_faces[0];
    const int face_bc = bc_faces[0] == face_i ? bc_faces[1] : bc_faces[0];
    const int face_ca = ca_faces[0] == face_i ? ca_faces[1] : ca_faces[0];

    const int3 abd_verts = gather_tri(faces[face_ab], corner_verts);
    const int3 bce_verts = gather_tri(faces[face_bc], corner_verts);
    const int3 caf_verts = gather_tri(faces[face_ca], corner_verts);

    BLI_assert(elem(abd_verts, edge_ab[0]) && elem(abd_verts, edge_ab[1]));
    BLI_assert(elem(bce_verts, edge_bc[0]) && elem(bce_verts, edge_bc[1]));
    BLI_assert(elem(caf_verts, edge_ca[0]) && elem(caf_verts, edge_ca[1]));

    const int a_vert = exclusive_one(face_verts, edge_bc);
    const int b_vert = exclusive_one(face_verts, edge_ca);
    const int c_vert = exclusive_one(face_verts, edge_ab);

    const int d_vert = exclusive_one(abd_verts, edge_ab);
    const int e_vert = exclusive_one(bce_verts, edge_bc);
    const int f_vert = exclusive_one(caf_verts, edge_ca);

    BLI_assert(!elem(face_verts, d_vert));
    BLI_assert(!elem(face_verts, e_vert));
    BLI_assert(!elem(face_verts, f_vert));

    const IndexRange verts_range = subdive_face_verts[face_i].shift(faces_verts_range.start());
    const IndexRange edges_range = subdive_face_edges[face_i].shift(faces_edges_range.start());

    const IndexRange ab_points_range = subdive_edge_verts[face_edges[0]].shift(
        edges_verts_range.start());
    const IndexRange bc_points_range = subdive_edge_verts[face_edges[1]].shift(
        edges_verts_range.start());
    const IndexRange ca_points_range = subdive_edge_verts[face_edges[2]].shift(
        edges_verts_range.start());

    const int3 face_abd_edge = gather_tri(faces[face_ab], corner_edges);
    const int3 face_bce_edge = gather_tri(faces[face_bc], corner_edges);
    const int3 face_caf_edge = gather_tri(faces[face_ca], corner_edges);

    Map<OrderedEdge, int, 9> edge_indices;
    edge_indices.add(edges[face_edges[0]], face_edges[0]);
    edge_indices.add(edges[face_edges[1]], face_edges[1]);
    edge_indices.add(edges[face_edges[2]], face_edges[2]);

    edge_indices.add(edges[face_abd_edge[0]], face_abd_edge[0]);
    edge_indices.add(edges[face_abd_edge[1]], face_abd_edge[1]);
    edge_indices.add(edges[face_abd_edge[2]], face_abd_edge[2]);

    edge_indices.add(edges[face_bce_edge[0]], face_bce_edge[0]);
    edge_indices.add(edges[face_bce_edge[1]], face_bce_edge[1]);
    edge_indices.add(edges[face_bce_edge[2]], face_bce_edge[2]);

    edge_indices.add(edges[face_caf_edge[0]], face_caf_edge[0]);
    edge_indices.add(edges[face_caf_edge[1]], face_caf_edge[1]);
    edge_indices.add(edges[face_caf_edge[2]], face_caf_edge[2]);

    const int2 edge_ad(face_verts[0], d_vert);
    const int2 edge_bd(face_verts[1], d_vert);

    const int2 edge_be(face_verts[1], e_vert);
    const int2 edge_ce(face_verts[2], e_vert);

    const int2 edge_cf(face_verts[2], f_vert);
    const int2 edge_af(face_verts[0], f_vert);

    const std::array<float2, 6> verts_list = {projection[a_vert],
                                              projection[b_vert],
                                              projection[c_vert],
                                              projection[d_vert],
                                              projection[e_vert],
                                              projection[f_vert]};
    const std::array<int, 9> edge_indices_list = {edge_indices.lookup(edge_ab),
                                                  edge_indices.lookup(edge_bc),
                                                  edge_indices.lookup(edge_ca),
                                                  edge_indices.lookup(edge_ad),
                                                  edge_indices.lookup(edge_bd),
                                                  edge_indices.lookup(edge_be),
                                                  edge_indices.lookup(edge_ce),
                                                  edge_indices.lookup(edge_cf),
                                                  edge_indices.lookup(edge_af)};
  });

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
    const int3 face_verts = gather_tri(face, corner_verts);

    const IndexRange face_verts_range = subdive_face_verts[face_i];
    const Span<float3> bary_weights = face_vertices_bary_weight.as_span().slice(face_verts_range);
    MutableSpan<float3> dst = dst_positions.slice(
        face_verts_range.shift(faces_verts_range.start()));
    const float3 a = dst_positions[face_verts[0]];
    const float3 b = dst_positions[face_verts[1]];
    const float3 c = dst_positions[face_verts[2]];
    parallel_transform(bary_weights, 2048, dst, [&](const float3 weight) {
      return bke::attribute_math::mix3(weight, a, b, c);
    });
  });

  return dst_mesh;
}

}  // namespace blender::geometry::dyntopo
