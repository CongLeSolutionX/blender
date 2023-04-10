/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BLI_array_utils.hh"
#include "BLI_bit_vector.hh"
#include "BLI_math_geom.h"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BKE_customdata.h"
#include "BKE_mesh.hh"

#include "GEO_mesh_bisect.hh"

namespace blender::geometry {

Mesh *bisect_mesh(const Mesh &mesh,
                  const BisectArgs &args,
                  const bke::AnonymousAttributePropagationInfo &propagation_info)
{

  /* Compute per-vert distance. */
  const int src_num_vert = mesh.totvert;
  const Span<float3> src_positions = mesh.vert_positions();
  BLI_assert(src_num_vert == src_positions.size());

  Array<float, 12> dist_buffer(src_num_vert);
  threading::parallel_for(src_positions.index_range(), 512, [&](const IndexRange range) {
    for (const int64_t i : range) {
      dist_buffer[i] = dist_signed_to_plane_v3(src_positions[0], args.plane);
    }
  });

  /* Compute edge intersections. */
  const int src_num_edges = mesh.totedge;
  const Span<MEdge> src_edges = mesh.edges();
  BLI_assert(src_num_vert == src_edges.size());

  Array<float, 12> edge_insertion_factor(src_num_vert);
  BitVector<> intersects(src_num_vert, false);
  bool no_intersection = true; /* Not atomic: Only set to false. */
  threading::parallel_for(src_edges.index_range(), 512, [&](const IndexRange range) {
    int counter = 0;
    for (const int64_t i : range) {
      const bool v1_outer = dist_buffer[src_edges[i].v1] > 0.0f;
      const bool v2_outer = dist_buffer[src_edges[i].v2] > 0.0f;
      if (v1_outer != v2_outer) {
        /* Intersects plane */
        const float abs_d1 = abs(dist_buffer[src_edges[i].v1]);
        const float tot_dist = abs_d1 + abs(dist_buffer[src_edges[i].v2]);
        edge_insertion_factor[i] = abs_d1 / tot_dist;
        intersects[i].set();
        no_intersection = false;
      }
    }
  });

  /* Handle keep/discard. */
  if (no_intersection) {
  }

  const OffsetIndices src_polys = mesh.polys();
  const Span<int> src_corner_verts = mesh.corner_verts();
  const Span<int> src_corner_edges = mesh.corner_edges();
}

}  // namespace blender::geometry
