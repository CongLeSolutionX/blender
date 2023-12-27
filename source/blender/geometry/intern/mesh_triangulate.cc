/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "atomic_ops.h"

#include "BLI_array_utils.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_index_mask.hh"
#include "BLI_math_geom.h"
#include "BLI_math_matrix.h"
#include "BLI_ordered_edge.hh"
#include "BLI_polyfill_2d.h"
#include "BLI_polyfill_2d_beautify.h"
#include "BLI_vector_set.hh"

#include "BLI_heap.h"
#include "BLI_memarena.h"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_customdata.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.hh"

#include "GEO_mesh_triangulate.hh"

namespace blender::geometry {

static void gather(const Span<int> src, const Span<int16_t> indices, MutableSpan<int> dst)
{
  for (const int i : indices.index_range()) {
    dst[i] = src[indices[i]];
  }
}

static Span<int> gather_or_reference(const Span<int> src,
                                     const Span<int16_t> indices,
                                     Vector<int> &dst)
{
  if (unique_sorted_indices::non_empty_is_range(indices)) {
    return src.slice(indices[0], indices.size());
  }
  dst.reinitialize(indices.size());
  gather(src, indices, dst);
  return dst.as_span();
}

static Span<int> gather_or_reference(const Span<int> src,
                                     const IndexMaskSegment mask,
                                     Vector<int> &dst)
{
  return gather_or_reference(src.drop_front(mask.offset()), mask.base_span(), dst);
}

/**
 * If a significant number of Ngons are selected (> 25% of the faces), then use the
 * face normals cache, in case the cache is persistent (or already calculated).
 */
static Span<float3> face_normals_if_worthwhile(const Mesh &src_mesh, const int selection_size)
{
  if (src_mesh.runtime->face_normals_cache.is_cached()) {
    return src_mesh.face_normals();
  }
  if (selection_size > src_mesh.faces_num / 4) {
    return src_mesh.face_normals();
  }
  return {};
}

static void copy_loose_vert_hint(const Mesh &src, Mesh &dst)
{
  const auto &src_cache = src.runtime->loose_verts_cache;
  if (src_cache.is_cached() && src_cache.data().count == 0) {
    dst.tag_loose_verts_none();
  }
}

static void copy_loose_edge_hint(const Mesh &src, Mesh &dst)
{
  const auto &src_cache = src.runtime->loose_edges_cache;
  if (src_cache.is_cached() && src_cache.data().count == 0) {
    dst.tag_loose_edges_none();
  }
}

static Mesh *create_mesh_no_attributes(const Mesh &params_mesh,
                                       const int verts_num,
                                       const int edges_num,
                                       const int faces_num,
                                       const int corners_num)
{
  Mesh *mesh = BKE_mesh_new_nomain(0, 0, faces_num, 0);
  BKE_mesh_copy_parameters_for_eval(mesh, &params_mesh);
  CustomData_free_layer_named(&mesh->vert_data, "position", 0);
  CustomData_free_layer_named(&mesh->edge_data, ".edge_verts", 0);
  CustomData_free_layer_named(&mesh->corner_data, ".corner_vert", 0);
  CustomData_free_layer_named(&mesh->corner_data, ".corner_edge", 0);
  mesh->verts_num = verts_num;
  mesh->edges_num = edges_num;
  mesh->corners_num = corners_num;
  return mesh;
}

static OffsetIndices<int> calc_faces(const OffsetIndices<int> src_faces,
                                     const IndexMask &unselected,
                                     MutableSpan<int> offsets)
{
  MutableSpan<int> new_tri_offsets = offsets.drop_back(unselected.size());
  offset_indices::fill_constant_group_size(3, new_tri_offsets.first(), new_tri_offsets);
  offset_indices::gather_selected_offsets(
      src_faces, unselected, new_tri_offsets.last(), offsets.take_back(unselected.size() + 1));
  return OffsetIndices<int>(offsets);
}

namespace quad {

/**
 *  #Edge_0_2       #Edge_1_3
 * 3 ------- 2     3 ------- 2
 * | 1     / |     | \     1 |
 * |     /   |     |   \     |
 * |   /     |     |     \   |
 * | /     0 |     | 0     \ |
 * 0 ------- 1     0 ------- 1
 */
enum class QuadDirection : int8_t {
  Edge_0_2 = 0,
  Edge_1_3 = 1,
};

/**
 * \note This behavior is meant to be the same as #BM_verts_calc_rotate_beauty.
 * The order of vertices requires special attention.
 */
static QuadDirection calc_quad_direction_beauty(const float3 &v0,
                                                const float3 &v1,
                                                const float3 &v2,
                                                const float3 &v3)
{
  const int flip_flag = is_quad_flip_v3(v1, v2, v3, v0);
  if (UNLIKELY(flip_flag & (1 << 0))) {
    return QuadDirection::Edge_0_2;
  }
  if (UNLIKELY(flip_flag & (1 << 1))) {
    return QuadDirection::Edge_1_3;
  }
  return BLI_polyfill_edge_calc_rotate_beauty__area(v1, v2, v3, v0, false) > 0.0f ?
             QuadDirection::Edge_0_2 :
             QuadDirection::Edge_1_3;
}

static void calc_quad_directions(const Span<float3> positions,
                                 const Span<int> face_offsets,
                                 const Span<int> corner_verts,
                                 const TriangulateQuadMode quad_mode,
                                 MutableSpan<QuadDirection> directions)
{
  if (quad_mode == TriangulateQuadMode::Fixed) {
    directions.fill(QuadDirection::Edge_0_2);
  }
  else if (quad_mode == TriangulateQuadMode::Alternate) {
    directions.fill(QuadDirection::Edge_1_3);
  }
  else if (quad_mode == TriangulateQuadMode::Beauty) {
    for (const int i : face_offsets.index_range()) {
      const Span<int> verts = corner_verts.slice(face_offsets[i], 4);
      directions[i] = calc_quad_direction_beauty(
          positions[verts[0]], positions[verts[1]], positions[verts[2]], positions[verts[3]]);
    }
  }
  else {
    const QuadDirection long_dir = quad_mode == TriangulateQuadMode::ShortEdge ?
                                       QuadDirection::Edge_0_2 :
                                       QuadDirection::Edge_1_3;
    const QuadDirection short_dir = quad_mode == TriangulateQuadMode::ShortEdge ?
                                        QuadDirection::Edge_1_3 :
                                        QuadDirection::Edge_0_2;
    for (const int i : face_offsets.index_range()) {
      const Span<int> verts = corner_verts.slice(face_offsets[i], 4);
      const float dist_0_2 = math::distance_squared(positions[verts[0]], positions[verts[2]]);
      const float dist_1_3 = math::distance_squared(positions[verts[1]], positions[verts[3]]);
      directions[i] = dist_0_2 < dist_1_3 ? long_dir : short_dir;
    }
  }
}

static void calc_corner_maps(const Span<int> face_offsets,
                             const Span<QuadDirection> directions,
                             MutableSpan<int> corner_map)
{
  for (const int i : face_offsets.index_range()) {
    MutableSpan<int> quad_map = corner_map.slice(6 * i, 6);
    /* These corner orders give new edges based on the first vertex of each triangle. */
    switch (directions[i]) {
      case QuadDirection::Edge_0_2:
        quad_map.copy_from({2, 0, 1, 0, 2, 3});
        break;
      case QuadDirection::Edge_1_3:
        quad_map.copy_from({1, 3, 0, 3, 1, 2});
        break;
    }
    const int face_start = face_offsets[i];
    for (int &i : quad_map) {
      i += face_start;
    }
  }
}

static void calc_corner_maps(const Span<float3> positions,
                             const OffsetIndices<int> src_faces,
                             const Span<int> src_corner_verts,
                             const IndexMask &quads,
                             const TriangulateQuadMode quad_mode,
                             MutableSpan<int> corner_map)
{
  struct TLS {
    Vector<int> offsets;
    Vector<QuadDirection> directions;
  };
  threading::EnumerableThreadSpecific<TLS> tls;

  quads.foreach_segment(GrainSize(1024), [&](const IndexMaskSegment quads, const int64_t pos) {
    TLS &data = tls.local();
    data.directions.reinitialize(quads.size());

    /* Find the offsets of each face in the local selection. We can gather them together even if
     * they aren't contiguous because we only need to know the start of each face; the size is
     * just four. */
    const Span<int> offsets = gather_or_reference(src_faces.data(), quads, data.offsets);
    calc_quad_directions(positions, offsets, src_corner_verts, quad_mode, data.directions);
    const IndexRange corners(pos * 6, offsets.size() * 6);
    quad::calc_corner_maps(offsets, data.directions, corner_map.slice(corners));
  });
}

/**
 * Each triangulated quad creates one additional edge in the result mesh, between the two
 * triangles. The corner_verts are just the corners of the quads, and the edges are just the new
 * edges for these quads.
 */
static void calc_edges(const Span<int> quad_corner_verts, MutableSpan<int2> edges)
{
  const int quads_num = quad_corner_verts.size() / 6;
  for (const int quad : IndexRange(quads_num)) {
    const Span<int> verts = quad_corner_verts.slice(6 * quad, 6);
    /* Use the first vertex of each triangle. */
    edges[quad] = int2(verts[0], verts[1]);
  }
}

static void calc_quad_corner_edges(const Span<int> src_corner_edges,
                                   const Span<int> corner_map,
                                   const int edges_start,
                                   MutableSpan<int> corner_edges)
{
  /* Each triangle starts at the new edge and winds in the same order as corner vertices
   * described by the corner map. */
  for (const int tri : IndexRange(corner_map.size() / 3)) {
    const int tri_start = 3 * tri;
    corner_edges[tri_start] = edges_start + tri / 2;
    corner_edges[tri_start + 1] = src_corner_edges[corner_map[tri_start + 1]];
    corner_edges[tri_start + 2] = src_corner_edges[corner_map[tri_start + 2]];
  }
}

static void calc_edges(const Span<int> src_corner_edges,
                       const Span<int> corner_map,
                       const Span<int> corner_verts,
                       const int edges_start,
                       MutableSpan<int2> edges,
                       MutableSpan<int> quad_corner_edges)
{
  const int quads_num = corner_map.size() / 6;
  threading::parallel_for(IndexRange(quads_num), 1024, [&](const IndexRange quads) {
    const IndexRange corners(quads.start() * 6, quads.size() * 6);
    calc_edges(corner_verts.slice(corners), edges.slice(quads));
    calc_quad_corner_edges(src_corner_edges,
                           corner_map.slice(corners),
                           edges_start + quads.start(),
                           quad_corner_edges.slice(corners));
  });
}

template<typename T>
static void copy_face_data_to_tris(const Span<T> src, const IndexMask &quads, MutableSpan<T> dst)
{
  quads.foreach_index_optimized<int>([&](const int src_i, const int dst_i) {
    dst[2 * dst_i + 0] = src[src_i];
    dst[2 * dst_i + 1] = src[src_i];
  });
}

static void copy_face_data_to_tris(const GSpan src, const IndexMask &quads, GMutableSpan dst)
{
  bke::attribute_math::convert_to_static_type(src.type(), [&](auto dummy) {
    using T = decltype(dummy);
    copy_face_data_to_tris(src.typed<T>(), quads, dst.typed<T>());
  });
}

}  // namespace quad

namespace ngon {

static OffsetIndices<int> calc_tris_by_ngon(const OffsetIndices<int> src_faces,
                                            const IndexMask &ngons,
                                            MutableSpan<int> face_offset_data)
{
  ngons.foreach_index(GrainSize(2048), [&](const int face, const int mask) {
    face_offset_data[mask] = bke::mesh::face_triangles_num(src_faces[face].size());
  });
  return offset_indices::accumulate_counts_to_offsets(face_offset_data);
}

static OffsetIndices<int> calc_edges_by_ngon(const OffsetIndices<int> src_faces,
                                             const IndexMask &selection,
                                             MutableSpan<int> edge_offset_data)
{
  selection.foreach_index(GrainSize(2048), [&](const int face, const int mask) {
    /* The number of new inner edges for each face is the number of corners - 3. */
    edge_offset_data[mask] = src_faces[face].size() - 3;
  });
  return offset_indices::accumulate_counts_to_offsets(edge_offset_data);
}

static void calc_corner_maps(const Span<float3> positions,
                             const OffsetIndices<int> src_faces,
                             const Span<int> src_corner_verts,
                             const Span<float3> face_normals,
                             const IndexMask &ngons,
                             const OffsetIndices<int> tris_by_ngon,
                             const TriangulateNGonMode ngon_mode,
                             MutableSpan<int> corner_map)
{
  struct TLS {
    Vector<float3x3> projections;
    Array<int> offset_data;
    Vector<float2> projected_positions;

    /* Only used for the "Beauty" method. */
    MemArena *arena = nullptr;
    Heap *heap = nullptr;

    ~TLS()
    {
      if (arena) {
        BLI_memarena_free(arena);
      }
      if (heap) {
        BLI_heap_free(heap, nullptr);
      }
    }
  };
  threading::EnumerableThreadSpecific<TLS> tls;

  ngons.foreach_segment(GrainSize(128), [&](const IndexMaskSegment ngons, const int pos) {
    TLS &data = tls.local();

    /* In order to simplify and "parallelize" the next loops, gather offsets used to group an array
     * large enough for all the local face corners. */
    data.offset_data.reinitialize(ngons.size() + 1);
    const OffsetIndices local_corner_offsets = offset_indices::gather_selected_offsets(
        src_faces, ngons, data.offset_data);

    /* Use face normals to build projection matrices to make the face positions 2D. */
    data.projections.reinitialize(ngons.size());
    MutableSpan<float3x3> projections = data.projections;
    if (face_normals.is_empty()) {
      for (const int i : ngons.index_range()) {
        const IndexRange src_face = src_faces[ngons[i]];
        const Span<int> face_verts = src_corner_verts.slice(src_face);
        const float3 normal = bke::mesh::face_normal_calc(positions, face_verts);
        axis_dominant_v3_to_m3_negate(projections[i].ptr(), normal);
      }
    }
    else {
      for (const int i : ngons.index_range()) {
        axis_dominant_v3_to_m3_negate(projections[i].ptr(), face_normals[ngons[i]]);
      }
    }

    /* Project the face positions into 2D using the matrices calculated above. */
    data.projected_positions.reinitialize(local_corner_offsets.total_size());
    MutableSpan<float2> projected_positions = data.projected_positions;
    for (const int i : ngons.index_range()) {
      const IndexRange src_face = src_faces[ngons[i]];
      const Span<int> face_verts = src_corner_verts.slice(src_face);
      const float3x3 &matrix = projections[i];

      MutableSpan<float2> positions_2d = projected_positions.slice(local_corner_offsets[i]);
      for (const int i : face_verts.index_range()) {
        mul_v2_m3v3(positions_2d[i], matrix.ptr(), positions[face_verts[i]]);
      }
    }

    if (ngon_mode == TriangulateNGonMode::Beauty) {
      if (!data.arena) {
        data.arena = BLI_memarena_new(BLI_POLYFILL_ARENA_SIZE, __func__);
      }
      if (!data.heap) {
        data.heap = BLI_heap_new_ex(BLI_POLYFILL_ALLOC_NGON_RESERVE);
      }
    }

    /* Calculate the triangulation of corners indices local to each face. */
    for (const int i : ngons.index_range()) {
      const Span<float2> positions_2d = projected_positions.slice(local_corner_offsets[i]);
      const IndexRange tris_range = tris_by_ngon[pos + i];
      MutableSpan<int> map = corner_map.slice(tris_range.start() * 3, tris_range.size() * 3);
      BLI_polyfill_calc(reinterpret_cast<const float(*)[2]>(positions_2d.data()),
                        positions_2d.size(),
                        1,
                        reinterpret_cast<uint(*)[3]>(map.data()));
      if (ngon_mode == TriangulateNGonMode::Beauty) {
        BLI_polyfill_beautify(reinterpret_cast<const float(*)[2]>(positions_2d.data()),
                              positions_2d.size(),
                              reinterpret_cast<uint(*)[3]>(map.data()),
                              data.arena,
                              data.heap);
        BLI_memarena_clear(data.arena);
      }
    }

    /* "Globalize" the triangulation created above so the map source indices reference _all_ of the
     * source vertices, not just within the source face. */
    for (const int i : ngons.index_range()) {
      const IndexRange src_face = src_faces[ngons[i]];
      const IndexRange tris_range = tris_by_ngon[pos + i];
      MutableSpan<int> map = corner_map.slice(tris_range.start() * 3, tris_range.size() * 3);
      for (int &vert : map) {
        vert += src_face.start();
      }
    }
  });
}

static void calc_inner_tri_edges(const IndexRange src_face,
                                 const Span<int> src_corner_verts,
                                 const Span<int> src_corner_edges,
                                 const Span<int3> corner_tris,
                                 const int edges_start,
                                 MutableSpan<int> corner_edges,
                                 VectorSet<OrderedEdge> &deduplication)
{
  const OrderedEdge last_edge(int(src_face.first()), int(src_face.last()));
  auto add_edge = [&](const OrderedEdge corner_edge) -> int {
    if (corner_edge == last_edge) {
      return src_corner_edges[src_face.last()];
    }
    if (corner_edge.v_high == corner_edge.v_low + 1) {
      return src_corner_edges[corner_edge.v_low];
    }
    const OrderedEdge vert_edge(src_corner_verts[corner_edge.v_low],
                                src_corner_verts[corner_edge.v_high]);
    return edges_start + deduplication.index_of_or_add(vert_edge);
  };

  for (const int i : corner_tris.index_range()) {
    const int3 tri = corner_tris[i];
    corner_edges[3 * i + 0] = add_edge({tri[0], tri[1]});
    corner_edges[3 * i + 1] = add_edge({tri[1], tri[2]});
    corner_edges[3 * i + 2] = add_edge({tri[2], tri[0]});
  }
}

static void calc_edges(const OffsetIndices<int> src_faces,
                       const Span<int> src_corner_verts,
                       const Span<int> src_corner_edges,
                       const IndexMask &ngons,
                       const OffsetIndices<int> tris_by_ngon,
                       const OffsetIndices<int> edges_by_ngon,
                       const OffsetIndices<int> faces,
                       const int edges_start,
                       const Span<int> corner_verts,
                       MutableSpan<int2> inner_edges,
                       MutableSpan<int> corner_edges)
{
  threading::EnumerableThreadSpecific<VectorSet<OrderedEdge>> tls;
  ngons.foreach_segment(GrainSize(128), [&](const IndexMaskSegment ngons, const int pos) {
    VectorSet<OrderedEdge> &deduplication = tls.local();
    for (const int i : ngons.index_range()) {
      const IndexRange edges = edges_by_ngon[pos + i];
      const IndexRange corners = faces[tris_by_ngon[pos + i]];
      deduplication.clear();
      calc_inner_tri_edges(src_faces[ngons[i]],
                           src_corner_verts,
                           src_corner_edges,
                           corner_verts.slice(corners).cast<int3>(),
                           edges_start + edges.start(),
                           corner_edges.slice(corners),
                           deduplication);
      inner_edges.slice(edges).copy_from(deduplication.as_span().cast<int2>());
    }
  });
}

}  // namespace ngon

namespace deduplication {

struct TriangleRef {
  const int *verts;
  uint64_t hash() const
  {
    return get_default_hash_3(verts[0], verts[1], verts[2]);
  }
  friend bool operator==(const TriangleRef &a, const TriangleRef &b)
  {
    return a.verts[0] == b.verts[0] && a.verts[1] == b.verts[1] && a.verts[2] == b.verts[2];
  }
};

static GroupedSpan<int> build_vert_to_tri_map(const int verts_num,
                                              const Span<int3> vert_tris,
                                              Array<int> &r_offsets,
                                              Array<int> &r_indices)
{
  r_offsets = Array<int>(verts_num + 1, 0);
  offset_indices::build_reverse_offsets(vert_tris.cast<int>(), r_offsets);
  const OffsetIndices offsets(r_offsets.as_span());

  r_indices.reinitialize(offsets.total_size());
  int *counts = MEM_cnew_array<int>(size_t(offsets.size()), __func__);
  BLI_SCOPED_DEFER([&]() { MEM_freeN(counts); })
  threading::parallel_for(vert_tris.index_range(), 1024, [&](const IndexRange range) {
    for (const int tri : range) {
      for (const int vert : {vert_tris[tri][0], vert_tris[tri][1], vert_tris[tri][2]}) {
        const int index_in_group = atomic_fetch_and_add_int32(&counts[vert], 1);
        r_indices[offsets[vert][index_in_group]] = tri;
      }
    }
  });

  return {r_offsets.as_span(), r_indices.as_span()};
}

/**
 * To avoid adding duplicate faces to the mesh without complicating the triangulation code to
 * support that unlikely case, check if triangles (which are all unselected) have an equivalent
 * newly created triangle, and don't copy them to the result mesh if so.
 */
static IndexMask calc_unselected_faces(const Mesh &mesh,
                                       const OffsetIndices<int> src_faces,
                                       const Span<int> src_corner_verts,
                                       const IndexMask &selection,
                                       const Span<int> corner_map,
                                       IndexMaskMemory &memory)
{
  const IndexMask unselected = selection.complement(src_faces.index_range(), memory);
  if (mesh.no_overlapping_topology()) {
    return unselected;
  }
  const bool has_tris = threading::parallel_reduce(
      unselected.index_range(),
      4096,
      false,
      [&](const IndexRange range, const bool init) {
        if (init) {
          return init;
        }
        const IndexMask sliced_mask = unselected.slice(range);
        for (const int64_t segment_i : IndexRange(sliced_mask.segments_num())) {
          const IndexMaskSegment segment = sliced_mask.segment(segment_i);
          for (const int i : segment) {
            if (src_faces[i].size() == 3) {
              return true;
            }
          }
        }
        return false;
      },
      std::logical_or());
  if (!has_tris) {
    return unselected;
  }
  Array<int3> vert_tris(corner_map.size() / 3);
  bke::attribute_math::gather(
      src_corner_verts, corner_map, vert_tris.as_mutable_span().cast<int>());

  Array<int> vert_to_tri_offsets;
  Array<int> vert_to_tri_indices;
  const GroupedSpan<int> vert_to_tri_map = build_vert_to_tri_map(
      mesh.verts_num, corner_map.cast<int3>(), vert_to_tri_offsets, vert_to_tri_indices);
  return IndexMask::from_predicate(unselected, GrainSize(1024), memory, [&](const int i) {
    const Span<int> face_verts = src_corner_verts.slice(src_faces[i]);
    if (face_verts.size() != 3) {
      return true;
    }
    for (const int vert : face_verts) {
      for (const int tri : vert_to_tri_map[vert]) {
        // TODO: Better performance check.
        if (Set<int, 3>(Span(&vert_tris[tri].x, 3)) == Set<int, 3>(face_verts)) {
          return false;
        }
      }
    }
    return true;
  });
}

}  // namespace deduplication

std::optional<Mesh *> mesh_triangulate(
    const Mesh &src_mesh,
    const IndexMask &selection_with_tris,
    const TriangulateNGonMode ngon_mode,
    const TriangulateQuadMode quad_mode,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const Span<float3> positions = src_mesh.vert_positions();
  const Span<int2> src_edges = src_mesh.edges();
  const OffsetIndices src_faces = src_mesh.faces();
  const Span<int> src_corner_verts = src_mesh.corner_verts();
  const Span<int> src_corner_edges = src_mesh.corner_edges();
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();

  /* Divide the input selection into separate selections for each face type. This isn't necessary
   * for correctness, but considering groups of each face type separately simplifies optimizing
   * for each type. For example, quad triangulation is much simpler than Ngon triangulation. */
  IndexMaskMemory memory;
  const IndexMask selection = IndexMask::from_predicate(
      selection_with_tris, GrainSize(4096), memory, [&](const int i) {
        return src_faces[i].size() > 3;
      });
  const IndexMask quads = IndexMask::from_predicate(
      selection, GrainSize(4096), memory, [&](const int i) { return src_faces[i].size() == 4; });
  const IndexMask ngons = IndexMask::from_predicate(
      selection, GrainSize(4096), memory, [&](const int i) { return src_faces[i].size() > 4; });
  if (quads.is_empty() && ngons.is_empty()) {
    /* All selected faces are already triangles. */
    return std::nullopt;
  }

  /* Calculate group of triangle indices for each selected Ngon to facilitate calculating them in
   * parallel later. */
  Array<int> tri_offsets(ngons.size() + 1);
  const OffsetIndices tris_by_ngon = ngon::calc_tris_by_ngon(src_faces, ngons, tri_offsets);
  const int ngon_tris_num = tris_by_ngon.total_size();
  const int quad_tris_num = quads.size() * 2;
  const IndexRange tris_range(ngon_tris_num + quad_tris_num);
  const IndexRange ngon_tris_range = tris_range.take_front(ngon_tris_num);
  const IndexRange quad_tris_range = tris_range.take_front(quad_tris_num);

  const int ngon_corners_num = tris_by_ngon.total_size() * 3;
  const int quad_corners_num = quads.size() * 6;
  const IndexRange tri_corners_range(quad_corners_num + ngon_corners_num);
  const IndexRange ngon_corners_range = tri_corners_range.take_front(ngon_corners_num);
  const IndexRange quad_corners_range = tri_corners_range.take_back(quad_corners_num);

  /* Calculate groups of new inner edges for each selected Ngon so they can be filled in parallel
   * later. */
  Array<int> edge_offset_data(ngons.size() + 1);
  const OffsetIndices edges_by_ngon = ngon::calc_edges_by_ngon(src_faces, ngons, edge_offset_data);
  const int ngon_edges_num = edges_by_ngon.total_size();
  const int quad_edges_num = quads.size();
  const IndexRange src_edges_range(0, src_edges.size());
  const IndexRange tri_edges_range(src_edges_range.one_after_last(),
                                   ngon_edges_num + quad_edges_num);
  const IndexRange ngon_edges_range = tri_edges_range.take_front(ngon_edges_num);
  const IndexRange quad_edges_range = tri_edges_range.take_front(quad_edges_num);

  /* An index map that maps from newly created corners in `tri_corners_range` to original corner
   * indices. This is used to interpolate `corner_vert` indices and face corner attributes. If
   * there are no face corner attributes, theoretically the map could be skipped and corner
   * vertex indices could be interpolated immediately, but that isn't done for simplicity. */
  Array<int> corner_map(tri_corners_range.size());

  if (!ngons.is_empty()) {
    ngon::calc_corner_maps(positions,
                           src_faces,
                           src_corner_verts,
                           face_normals_if_worthwhile(src_mesh, ngons.size()),
                           ngons,
                           tris_by_ngon,
                           ngon_mode,
                           corner_map.as_mutable_span().slice(ngon_corners_range));
  }
  if (!quads.is_empty()) {
    quad::calc_corner_maps(positions,
                           src_faces,
                           src_corner_verts,
                           quads,
                           quad_mode,
                           corner_map.as_mutable_span().slice(quad_corners_range));
  }

  const IndexMask unselected = deduplication::calc_unselected_faces(
      src_mesh, src_faces, src_corner_verts, selection, corner_map, memory);
  const IndexRange unselected_range(tris_range.one_after_last(), unselected.size());

  /* Create a mesh with no face corners. We don't know the number of corners from unselected faces.
   * We have to create the face offsets anyway, this will give us the number of corners. */
  Mesh *mesh = create_mesh_no_attributes(src_mesh,
                                         src_mesh.verts_num,
                                         tri_edges_range.size() + src_edges.size(),
                                         tris_range.size() + unselected.size(),
                                         0);

  /* Find the face corner ranges using the offsets array from the new mesh. That gives us the
   * final number of face corners. */
  const OffsetIndices faces = calc_faces(src_faces, unselected, mesh->face_offsets_for_write());
  mesh->corners_num = faces.total_size();
  const OffsetIndices faces_unselected = faces.slice(unselected_range);

  bke::MutableAttributeAccessor attributes = mesh->attributes_for_write();
  attributes.add<int2>(".edge_verts", bke::AttrDomain::Edge, bke::AttributeInitConstruct());
  attributes.add<int>(".corner_vert", bke::AttrDomain::Corner, bke::AttributeInitConstruct());
  attributes.add<int>(".corner_edge", bke::AttrDomain::Corner, bke::AttributeInitConstruct());

  MutableSpan<int2> edges = mesh->edges_for_write();
  MutableSpan<int> corner_verts = mesh->corner_verts_for_write();
  MutableSpan<int> corner_edges = mesh->corner_edges_for_write();

  bke::attribute_math::gather(src_corner_verts, corner_map, corner_verts.slice(tri_corners_range));

  if (!ngons.is_empty()) {
    ngon::calc_edges(src_faces,
                     src_corner_verts,
                     src_corner_edges,
                     ngons,
                     tris_by_ngon,
                     edges_by_ngon,
                     faces.slice(ngon_tris_range),
                     ngon_edges_range.start(),
                     corner_map.as_mutable_span().slice(ngon_corners_range),
                     edges.slice(ngon_edges_range),
                     corner_edges);
  }

  if (!quads.is_empty()) {
    quad::calc_edges(src_corner_edges,
                     corner_map.as_mutable_span().slice(quad_corners_range),
                     corner_verts,
                     quad_edges_range.start(),
                     edges.slice(quad_edges_range),
                     corner_edges.slice(quad_corners_range));
  }

  /* Vertex attributes are totally unnaffected and can be shared with implicit sharing.
   * Use the #CustomData API for better support for vertex groups. */
  CustomData_merge(&src_mesh.vert_data, &mesh->vert_data, CD_MASK_MESH.vmask, mesh->verts_num);

  array_utils::copy(src_edges, edges.take_front(src_edges.size()));
  for (auto &attribute : bke::retrieve_attributes_for_transfer(
           src_attributes, attributes, ATTR_DOMAIN_MASK_EDGE, propagation_info, {".edge_verts"}))
  {
    attribute.dst.span.slice(src_edges_range).copy_from(attribute.src);
    GMutableSpan new_data = attribute.dst.span.slice(tri_edges_range);
    /* It would be reasonable interpolate data from connected edges within each face.
     * Currently the data from new edges is just set to the type's default value. */
    const void *default_value = new_data.type().default_value();
    new_data.type().fill_assign_n(default_value, new_data.data(), new_data.size());
    attribute.dst.finish();
  }

  if (CustomData_has_layer(&src_mesh.edge_data, CD_ORIGINDEX)) {
    const Span src(
        static_cast<const int *>(CustomData_get_layer(&src_mesh.edge_data, CD_ORIGINDEX)),
        src_mesh.edges_num);
    MutableSpan dst(static_cast<int *>(CustomData_add_layer(
                        &mesh->edge_data, CD_ORIGINDEX, CD_CONSTRUCT, mesh->edges_num)),
                    mesh->edges_num);
    dst.slice(tri_edges_range).fill(ORIGINDEX_NONE);
    array_utils::copy(src, dst.slice(src_edges_range));
  }

  for (auto &attribute : bke::retrieve_attributes_for_transfer(
           src_attributes, attributes, ATTR_DOMAIN_MASK_FACE, propagation_info, {}))
  {
    bke::attribute_math::gather_to_groups(
        tris_by_ngon, ngons, attribute.src, attribute.dst.span.slice(ngon_tris_range));
    quad::copy_face_data_to_tris(attribute.src, quads, attribute.dst.span.slice(quad_tris_range));
    array_utils::gather(attribute.src, unselected, attribute.dst.span.slice(unselected_range));
    attribute.dst.finish();
  }

  if (CustomData_has_layer(&src_mesh.face_data, CD_ORIGINDEX)) {
    const Span src(
        static_cast<const int *>(CustomData_get_layer(&src_mesh.face_data, CD_ORIGINDEX)),
        src_mesh.faces_num);
    MutableSpan dst(static_cast<int *>(CustomData_add_layer(
                        &mesh->face_data, CD_ORIGINDEX, CD_CONSTRUCT, mesh->faces_num)),
                    mesh->faces_num);
    bke::attribute_math::gather_to_groups(tris_by_ngon, ngons, src, dst.slice(ngon_tris_range));
    quad::copy_face_data_to_tris(src, quads, dst.slice(quad_tris_range));
    array_utils::gather(src, unselected, dst.slice(unselected_range));
  }

  array_utils::gather_group_to_group(
      src_faces, faces_unselected, unselected, src_corner_verts, corner_verts);
  array_utils::gather_group_to_group(
      src_faces, faces_unselected, unselected, src_corner_edges, corner_edges);
  for (auto &attribute : bke::retrieve_attributes_for_transfer(src_attributes,
                                                               attributes,
                                                               ATTR_DOMAIN_MASK_CORNER,
                                                               propagation_info,
                                                               {".corner_vert", ".corner_edge"}))
  {
    bke::attribute_math::gather_group_to_group(
        src_faces, faces_unselected, unselected, attribute.src, attribute.dst.span);
    bke::attribute_math::gather(
        attribute.src, corner_map.as_span(), attribute.dst.span.slice(tri_corners_range));
    attribute.dst.finish();
  }

  mesh->runtime->bounds_cache = src_mesh.runtime->bounds_cache;
  copy_loose_vert_hint(src_mesh, *mesh);
  copy_loose_edge_hint(src_mesh, *mesh);
  if (src_mesh.no_overlapping_topology()) {
    mesh->tag_overlapping_none();
  }
  return mesh;
}

}  // namespace blender::geometry
