/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BLI_array_utils.hh"
#include "BLI_bit_vector.hh"
#include "BLI_math_geom.h"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BKE_customdata.hh"
#include "BKE_mesh.hh"

#include "GEO_mesh_bisect.hh"

namespace blender::geometry {

Mesh *bisect_mesh(const Mesh &mesh,
                  const BisectArgs &args,
                  const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const int src_num_vert = mesh.verts_num;
  const int src_num_edges = mesh.edges_num;
  const int src_num_polys = mesh.faces_num;

  /* Compute per-vert distance. */
  const Span<float3> src_positions = mesh.vert_positions();
  IndexRange src_vert_range = src_positions.index_range();
  BLI_assert(src_num_vert == src_positions.size());

  BitVector<> outer_vertex(src_num_vert, false);
  Array<float, 12> dist_buffer(src_num_vert);
  threading::parallel_for(src_vert_range, 512, [&](const IndexRange range) {
    for (const int64_t i : range) {
      dist_buffer[i] = dist_signed_to_plane_v3(src_positions[0], args.plane);
      const bool is_outer = dist_buffer[i] >= 0.0f;
      outer_vertex[i].set(is_outer);
    }
  });

  /* Compute edge intersections. */
  const Span<int2> src_edges = mesh.edges();
  BLI_assert(src_num_vert == src_edges.size());

  Array<float, 12> edge_insertion_factor(src_num_edges);
  BitVector<> intersects(src_num_edges, false);
  const int num_edge_intersect = threading::parallel_reduce(
      src_edges.index_range(),
      512,
      0,
      [&](const IndexRange range, int identity) {
        int intersect_counter = identity;
        for (const int64_t i : range) {
          const bool v1_outer = outer_vertex[src_edges[i].x];
          const bool v2_outer = outer_vertex[src_edges[i].y];
          if (v1_outer != v2_outer) {
            /* Intersects plane */
            const float abs_d1 = abs(dist_buffer[src_edges[i].x]);
            const float tot_dist = abs_d1 + abs(dist_buffer[src_edges[i].y]);
            edge_insertion_factor[i] = abs_d1 / tot_dist;
            intersects[i].set();
            intersect_counter++;
          }
        }
        return intersect_counter;
      },
      [](int a, int b) { return a + b; });

  /* Handle keep/discard all. */
  if (num_edge_intersect == 0) {
  }

  const OffsetIndices src_polys = mesh.faces();
  const Span<int> src_corner_verts = mesh.corner_verts();
  const Span<int> src_corner_edges = mesh.corner_edges();

  auto keep = [&](int vert_index) {
    return (outer_vertex[vert_index] && !args.clear_outer) ||
           (!outer_vertex[vert_index] && !args.clear_inner);
  };

  /* Create copy masks */
  /* Vertices */
  Vector<int> old_to_new_vertex_map(src_num_vert);
  Vector<int> new_to_old_vertex_map;
  new_to_old_vertex_map.reserve(src_num_vert);
  for (const int64_t i : src_vert_range) {
    if (keep(i)) {
      old_to_new_vertex_map[i] = new_to_old_vertex_map.size();
      new_to_old_vertex_map.append(i);
    }
  }

  /* Edges */
  Vector<int> new_to_old_edge_map;
  new_to_old_edge_map.reserve(src_num_edges);
  for (const int64_t i : src_edges.index_range()) {
    if (keep(src_edges[i].x) && keep(src_edges[i].y)) {
      new_to_old_edge_map.append(i);
    }
  }

  /* Polygons */
  Vector<int> new_to_old_poly_map, new_poly_offsets, intersected_poly;
  new_to_old_poly_map.reserve(src_num_polys);
  new_poly_offsets.reserve(src_num_polys);

  new_poly_offsets.append(0);
  for (const int64_t index_poly : src_polys.index_range()) {

    int keep_flag = 0;
    for (const int64_t index_vert : src_corner_verts.slice(src_polys[index_poly])) {
      keep_flag |= 1 << keep(index_vert);
    }

    if (keep_flag == 3) {
      intersected_poly.append(index_poly);
    }
    else if (keep_flag & 2) {
      new_to_old_poly_map.append(index_poly);
      new_poly_offsets.append(new_poly_offsets.last() + src_polys[index_poly].size());
    }
  }

  /* Create new mesh */
  // TODO: Does not account for copied counts
  const int result_nverts = new_to_old_vertex_map.size();
  const int result_nedges = new_to_old_edge_map.size();
  const int result_nloops = new_poly_offsets.last();
  const int result_npolys = new_to_old_poly_map.size();

  Mesh *result = BKE_mesh_new_nomain_from_template(
      &mesh, result_nverts, result_nedges, result_nloops, result_npolys);

  /* Copy kept data */

  // TODO

  /* Handle intersected data */

  // TODO

  return result;
}

}  // namespace blender::geometry
