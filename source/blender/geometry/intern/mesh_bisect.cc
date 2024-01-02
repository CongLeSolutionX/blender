/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BLI_array_utils.hh"
#include "BLI_bit_vector.hh"
#include "BLI_math_base.hh"
#include "BLI_math_geom.h"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_customdata.hh"
#include "BKE_mesh.hh"

#include "GEO_mesh_bisect.hh"
#include <variant>

namespace blender::geometry {

enum EdgeIntersectType { Discarded = 0, Kept = 1, Intersect = 2 };

/*
 * Vertex generated from linear interpolation between two
 */
struct MeshVertexGroupCopyMask {
  IndexMask src_indices;
};
/*
 * Vertex generated from linear interpolation between two
 */
struct MeshVertexGroupLinear {
  Span<int2> src_indices;
  Span<float> weights;
};

struct MeshEdgeGroupCopyMask {
  /*
   * Mask for the edges to copy from the data source.
   */
  IndexMask src_indices;
  /*
   * Optional: Table for remapping the vertex indices for the edge. Leave empty if vertices are
   * left unchanged.
   */
  Span<int> mapping_table;
};
/*
 * Edge generated between the given vertices.
 */
struct MeshEdgeGroupPair {
  /*
   * Indices to the edge connects in the target mesh.
   */
  Span<int2> vertex_indices;
  /*
   * Optional: Edges to inherit values from, support 0, 1, 2 edges to inherit from. Takes the
   * average when possible. Leave invalid entries to -1, or leave it empty if no values are set.
   */
  Span<int2> src_edge_indices;
};

/*
 * Polygons generated between the given vertices.
 */
struct MeshPolygonGroup {
  /*
   * Indices to the polygon attributes should be copied from.
   */
  Span<int> src_polygon_indices;
  /*
   * Edge index sets, indices point to edges in the target mesh.
   */
  Span<Vector<int, 4>> edge_indices;
};

/*
 * Why variants and not inheritance?
 *
 * Pros:
 * - Stack constructible (clean to use, promotes Span<T> over owning data structure).
 * - Restricted usage (enforces standardization, reduce maintainance).
 *
 * Cons:
 * - Unsafe (if allowing 'custom' variants compiler would not complain, mitigatable?).
 *    - Can provide inheritance based impl. as 'custom' through pointer...
 * - Verbose with if / else switches, increased maintainance and error prone (compiler..).
 * - Restricted usage complicates deprication of old / modified variants.
 */

/*
 * Descriptor variants for mesh vertex groups/sets.
 *
 * TODO:
 * Barycentric
 * N-ary
 * Custom
 */
using VariantVertexGroup = std::variant<MeshVertexGroupCopyMask, MeshVertexGroupLinear>;
/*
 * Descriptor variants for mesh edge groups/sets.
 */
using VariantEdgeGroup = std::variant<MeshEdgeGroupCopyMask, MeshEdgeGroupPair>;
/*
 * Descriptor variants for mesh polygon groups/sets.
 */
using VariantPolygonGroup = std::variant<MeshPolygonGroup>;

IndexRange vertex_range(VariantVertexGroup group)
{
  if (auto item = std::get_if<MeshVertexGroupCopyMask>(&group)) {
    return item->src_indices.index_range();
  }
  else if (auto item = std::get_if<MeshVertexGroupLinear>(&group)) {
    return item->src_indices.index_range();
  }
  BLI_assert_msg(false, "Unreachable: Invalid variant implementation");
}

IndexRange edge_range(VariantEdgeGroup group)
{
  if (auto item = std::get_if<MeshEdgeGroupCopyMask>(&group)) {
    return item->src_indices.index_range();
  }
  else if (auto item = std::get_if<MeshEdgeGroupPair>(&group)) {
    return item->vertex_indices.index_range();
  }
  BLI_assert_msg(false, "Unreachable: Invalid variant implementation");
}

IndexRange polygon_corner_range(VariantPolygonGroup group)
{
  if (auto item = std::get_if<MeshPolygonGroup>(&group)) {
    return item->src_polygon_indices.index_range();
  }
  BLI_assert_msg(false, "Unreachable: Invalid variant implementation");
}

/*
 * Pre-compute corner offsets indices.
 */
Array<int, 12> polygon_corner_offsets(VariantPolygonGroup group, int64_t source_offset)
{
  const IndexRange range = polygon_corner_range(group);
  Array<int, 12> offsets(range.size() + 1);
  offsets[0] = source_offset;

  if (auto item = std::get_if<MeshPolygonGroup>(&group)) {
    for (const int64_t group_index : range) {
      offsets[group_index + 1] = offsets[group_index] + item->edge_indices[group_index].size();
    }
  }
  else {
    BLI_assert_msg(false, "Unreachable: Invalid variant implementation");
  }
  return offsets;
}

int count_num_vertices(const Span<VariantVertexGroup> vertex_groups)
{
  int vertex_count = 0;
  for (int64_t i = 0; i < vertex_groups.size(); i++) {
    vertex_count += vertex_range(vertex_groups[i]).size();
  }
  return vertex_count;
}

int count_num_edges(const Span<VariantEdgeGroup> edge_groups)
{
  int edge_count = 0;
  for (int64_t i = 0; i < edge_groups.size(); i++) {
    edge_count += edge_range(edge_groups[i]).size();
  }
  return edge_count;
}

int2 count_num_polygons(const Span<VariantPolygonGroup> poly_groups)
{
  int face_count = 0;
  int offset = 0;
  for (int64_t i = 0; i < poly_groups.size(); i++) {
    auto range = polygon_corner_offsets(poly_groups[i], offset);
    face_count += range.size() - 1;
    offset = range.last();
  }
  return {face_count, offset};
}

void transfer_vertex_data(const Mesh &src_mesh,
                          Mesh &dst_mesh,
                          const Span<VariantVertexGroup> vertex_groups,
                          const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh.attributes_for_write();

  Set<std::string> copy_point_skip;
  // copy_point_skip.add("nurbs_weight");

  /* Copy point domain. */
  for (bke::AttributeTransferData &attribute :
       bke::retrieve_attributes_for_transfer(src_attributes,
                                             dst_attributes,
                                             ATTR_DOMAIN_MASK_POINT,
                                             propagation_info,
                                             copy_point_skip))
  {
    bke::attribute_math::convert_to_static_type(attribute.meta_data.data_type, [&](auto dummy) {
      using T = decltype(dummy);

      Span<T> src_data = attribute.src.template typed<T>();
      MutableSpan<T> dst_data = attribute.dst.span.typed<T>();

      int64_t group_offset = 0;
      for (int64_t i = 0; i < vertex_groups.size(); i++) {
        IndexRange group_range = vertex_range(vertex_groups[i]);

        if (auto item = std::get_if<MeshVertexGroupCopyMask>(&vertex_groups[i])) {
          /* Copy */
          IndexRange dst_range = group_range.shift(group_offset);
          array_utils::gather(src_data, item->src_indices, dst_data.slice(dst_range));
        }
        else {
          if (auto item = std::get_if<MeshVertexGroupLinear>(&vertex_groups[i])) {
            threading::parallel_for(group_range, 512, [&](IndexRange group_slice) {
              IndexRange dst_range = group_range.shift(group_offset);

              /* Linear interpolate */
              for (const int64_t i : group_slice.index_range()) {
                const int64_t sample_index = group_slice[i];
                int2 vert_indices = item->src_indices[sample_index];
                dst_data[dst_range[i]] = bke::attribute_math::mix2(item->weights[sample_index],
                                                                   src_data[vert_indices.x],
                                                                   src_data[vert_indices.y]);
              }
            });
          }
        }
        group_offset += group_range.size();
      }
    });
  }
}

void transfer_edge_data(const Mesh &src_mesh,
                        Mesh &dst_mesh,
                        const Span<VariantEdgeGroup> edge_groups,
                        const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh.attributes_for_write();

  Set<std::string> copy_edge_skip;
  copy_edge_skip.add(".edge_verts");

  /* Assign edge indices */
  {
    Span<int2> src_edges = src_mesh.edges();
    MutableSpan<int2> dst_edges = dst_mesh.edges_for_write();

    int64_t group_offset = 0;
    for (int64_t i = 0; i < edge_groups.size(); i++) {
      const IndexRange dst_range = edge_range(edge_groups[i]).shift(group_offset);
      MutableSpan<int2> dst_edge_slice = dst_edges.slice(dst_range);

      if (auto item = std::get_if<MeshEdgeGroupCopyMask>(&edge_groups[i])) {
        /* Copy using updated mapping table */
        item->src_indices.foreach_index([&](int64_t i, int64_t pos) {
          int2 src_index_pair = src_edges[i];
          int2 mapped_pair = int2(item->mapping_table[src_index_pair.x],
                                  item->mapping_table[src_index_pair.y]);
          dst_edge_slice[pos] = mapped_pair;
        });
      }
      else if (auto item = std::get_if<MeshEdgeGroupPair>(&edge_groups[i])) {
        /* Copy indices */
        dst_edge_slice.copy_from(item->vertex_indices);
      }
      else {
        BLI_assert_msg(false, "Unreachable: Invalid variant implementation");
      }
      group_offset += dst_range.size();
    }
  }

  /* Copy edge domain. */
  for (bke::AttributeTransferData &attribute :
       bke::retrieve_attributes_for_transfer(src_attributes,
                                             dst_attributes,
                                             ATTR_DOMAIN_MASK_EDGE,
                                             propagation_info,
                                             copy_edge_skip))
  {
    const CPPType &cpp_type = *bke::custom_data_type_to_cpp_type(attribute.meta_data.data_type);
    bke::attribute_math::convert_to_static_type(cpp_type, [&](auto dummy) {
      using T = decltype(dummy);

      Span<T> src_data = attribute.src.template typed<T>();
      MutableSpan<T> dst_data = attribute.dst.span.typed<T>();

      int64_t group_offset = 0;
      for (int64_t i = 0; i < edge_groups.size(); i++) {
        const IndexRange group_range = edge_range(edge_groups[i]);
        const IndexRange dst_range = group_range.shift(group_offset);

        if (auto item = std::get_if<MeshEdgeGroupCopyMask>(&edge_groups[i])) {
          /* Copy */
          array_utils::gather(src_data, item->src_indices, dst_data.slice(dst_range));
        }
        else if (auto item = std::get_if<MeshEdgeGroupPair>(&edge_groups[i])) {
          /* Mix edge data IFF source edges are specified. */
          if (item->src_edge_indices.size() == group_range.size()) {
            threading::parallel_for(group_range, 512, [&](IndexRange group_slice) {
              for (const int64_t i : group_slice.index_range()) {
                const int64_t sample_index = group_slice[i];
                int2 vert_indices = item->src_edge_indices[sample_index];

                T &value = dst_data[dst_range[i]];
                if (vert_indices.x == -1) {
                  cpp_type.default_construct(&value);
                }
                else if (vert_indices.y == -1) {
                  value = src_data[vert_indices.x];
                }
                else {
                  value = bke::attribute_math::mix2(
                      0.5f, src_data[vert_indices.x], src_data[vert_indices.y]);
                }
              }
            });
          }
          else {
            /* Defualt fill. */
            cpp_type.default_construct_n(dst_data.slice(dst_range).data(), dst_range.size());
          }
        }
        else {
          BLI_assert_msg(false, "Unreachable: Invalid variant implementation");
        }
        group_offset += group_range.size();
      }
    });
  }
}

/* Find the common value (assumed the shared value exist).
 */
int shared_index(int2 a, int2 b)
{
  if (a.x == b.x || a.x == b.y) {
    return a.x;
  }
  // BLI_assert(a.y == b.x || a.y == b.y);
  return a.y;
}

int shared_edge_vert(Span<int2> edge_indices, int a, int b)
{
  return shared_index(edge_indices[a], edge_indices[b]);
}

void transfer_polygon_data(const Mesh &src_mesh,
                           Mesh &dst_mesh,
                           const Span<VariantPolygonGroup> poly_groups,
                           const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  if (dst_mesh.faces_num == 0) {
    return;
  }

  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh.attributes_for_write();

  Set<std::string> copy_poly_skip;
  copy_poly_skip.remove(".corner_vert");
  copy_poly_skip.remove(".corner_edge");

  /* Assign polygon indices */
  {
    Span<int> src_vert_corners = src_mesh.corner_verts();
    Span<int> src_edge_corners = src_mesh.corner_edges();
    MutableSpan<int> dst_vert_corners = dst_mesh.corner_verts_for_write();
    MutableSpan<int> dst_edge_corners = dst_mesh.corner_edges_for_write();
    MutableSpan<int> dst_face_offsets = dst_mesh.face_offsets_for_write();
    Span<int2> dst_edges = dst_mesh.edges();

    int64_t tot_offset = 0;
    int64_t tot_face_count = 0;
    dst_face_offsets[0] = 0;
    for (int64_t group_index = 0; group_index < poly_groups.size(); group_index++) {
      const auto dst_offsets = polygon_corner_offsets(poly_groups[group_index], tot_offset);
      const IndexRange dst_offset_range(tot_face_count, dst_offsets.size());
      BLI_assert(dst_face_offsets[dst_offset_range.first()] == dst_offsets.first());
      dst_face_offsets.slice(dst_offset_range).copy_from(dst_offsets);

      if (auto item = std::get_if<MeshPolygonGroup>(&poly_groups[group_index])) {
        BLI_assert(dst_offsets.size() - 1 == item->edge_indices.size());

        threading::parallel_for(item->edge_indices.index_range(), 512, [&](IndexRange subrange) {
          for (const int64_t index : subrange) {
            MutableSpan<int> dst_edgec = dst_edge_corners.slice(
                IndexRange(dst_offsets[index], dst_offsets[index + 1] - dst_offsets[index]));
            MutableSpan<int> dst_vertc = dst_vert_corners.slice(
                IndexRange(dst_offsets[index], dst_offsets[index + 1] - dst_offsets[index]));

            /* Transfer corners */
            dst_edgec[0] = item->edge_indices[index][0];
            dst_vertc[0] = shared_edge_vert(
                dst_edges,
                dst_edgec[0],
                item->edge_indices[index][dst_edgec.index_range().last()]);
            for (const int64_t i : dst_edgec.index_range().drop_front(1)) {
              dst_edgec[i] = item->edge_indices[index][i];
              int shared_vert = shared_edge_vert(dst_edges, dst_edgec[i], dst_edgec[i - 1]);
              dst_vertc[i] = shared_vert;
            }
          }
        });
      }

      tot_offset += dst_offsets.last();
      tot_face_count += dst_offsets.size() - 1;
    }
  }
}

Mesh *new_mesh_from_groups(const Mesh &src_mesh,
                           const Span<VariantVertexGroup> vertex_groups,
                           const Span<VariantEdgeGroup> edge_groups,
                           const Span<VariantPolygonGroup> poly_groups)
{
  const int num_vertices = count_num_vertices(vertex_groups);
  const int num_edges = count_num_edges(edge_groups);
  int2 num_poly_corner = count_num_polygons(poly_groups);

  Mesh *result = BKE_mesh_new_nomain_from_template(
      &src_mesh, num_vertices, num_edges, num_poly_corner.x, num_poly_corner.y);
  return result;
}

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

  BitVector<> is_kept_vertex(src_num_vert, false);
  Array<float, 12> dist_buffer(src_num_vert);

  auto fn_is_outside = [](float signed_distance) { return signed_distance > 0; };
  auto fn_is_keep = [](const BisectArgs &args, bool is_outer_vertex) {
    return (is_outer_vertex && !args.clear_outer) || (!is_outer_vertex && !args.clear_inner);
  };

  IndexMaskMemory kept_memory;
  IndexMask kept_vertices = IndexMask::from_predicate(
      IndexMask(src_vert_range), GrainSize(512), kept_memory, [&](const int64_t i) {
        dist_buffer[i] = dist_signed_to_plane_v3(src_positions[i], args.plane);
        /* If 'outside': Logic MUST match edge_split gen. */
        const bool is_outside = fn_is_outside(dist_buffer[i]);
        const bool is_kept = fn_is_keep(args, is_outside);
        is_kept_vertex[i].set(is_kept);
        return is_kept;
      });

  /* Compute edge intersections. */
  const Span<int2> src_edges = mesh.edges();
  // BLI_assert(src_num_vert == src_edges.size());
  auto fn_intersected_edge = [&](int64_t i) {
    const bool v1_kept = is_kept_vertex[src_edges[i].x];
    const bool v2_kept = is_kept_vertex[src_edges[i].y];
    if (v1_kept != v2_kept) {
      return int(EdgeIntersectType::Intersect);
    }
    /* No intersection and keep one => both vertices kept => edge kept => 1 otherwise 0 */
    return int(v1_kept);
  };

  Array<float, 12> edge_insertion_factor(src_num_edges);
  IndexMaskMemory intersected_memory;

  std::array<IndexMask, 3> edge_type_selections;
  IndexMask::from_groups<int64_t>(IndexMask(src_edges.index_range()),
                                  intersected_memory,
                                  fn_intersected_edge,
                                  MutableSpan<IndexMask>(edge_type_selections));

  /* Handle keep/discard all. */
  const int64_t num_kept_edges = edge_type_selections[EdgeIntersectType::Kept].size();
  const int64_t num_inter_edges = edge_type_selections[EdgeIntersectType::Intersect].size();
  if (num_inter_edges == 0) {
    if (edge_type_selections[EdgeIntersectType::Discarded].size() == 0) {
      return nullptr;
    }
    else if (num_kept_edges == 0) {
      // TODO: Return empty
    }
    /* else: process normally, at least 1 disjoint mesh is discarded */
  }

  const OffsetIndices src_polys = mesh.faces();
  const Span<int> src_corner_verts = mesh.corner_verts();
  const Span<int> src_corner_edges = mesh.corner_edges();

  /* Create vertex index map */
  Array<int, 12> old_to_new_vertex_map(src_num_vert);
  kept_vertices.foreach_index([&](const int64_t index, const int64_t index_pos) {
    old_to_new_vertex_map[index] = index_pos;
  });

  /* Edges */
  auto fn_intersect_vertex_index = [&](const int intersect_edge_index) {
    return intersect_edge_index + int(kept_vertices.size());
  };

  const int keep_count = !args.clear_inner + !args.clear_outer;
  const int keep_both = keep_count == 2;
  Array<float, 12> split_lerp_weights(edge_type_selections[EdgeIntersectType::Intersect].size());
  Array<int2, 12> split_index_pairs(edge_type_selections[EdgeIntersectType::Intersect].size());
  Array<int2, 12> new_split_edge_verts(edge_type_selections[EdgeIntersectType::Intersect].size() *
                                       keep_count);
  Array<int2, 12> new_split_edge_map(new_split_edge_verts.size());

  edge_type_selections[EdgeIntersectType::Intersect].foreach_index(
      GrainSize(512), [&](const int64_t src_index, const int64_t index_pos) {
        const int2 vert_indices = src_edges[src_index];
        const float signed_dist_x = dist_buffer[vert_indices.x];
        const float signed_dist_y = dist_buffer[vert_indices.y];
        const float abs_d1 = abs(signed_dist_x);
        const float tot_dist = abs_d1 + abs(signed_dist_y);
        split_lerp_weights[index_pos] = math::safe_divide<float>(abs_d1, tot_dist);
        split_index_pairs[index_pos] = vert_indices;

        /* Generate split edges IFF any side is kept */
        if (keep_count > 0) {
          const int new_vert_index = fn_intersect_vertex_index(index_pos);
          const int new_edge_index = index_pos * keep_count;
          /* Inner/Outer logic must match 'is_kept_vertex'*/
          const bool is_ouside_x = fn_is_outside(signed_dist_x);
          const bool is_outside_y = fn_is_outside(signed_dist_y);

          /* Ordered vertex indices: x = outer, y = inner */
          int2 order = is_ouside_x ? vert_indices : int2{vert_indices.y, vert_indices.x};
          int2 insert_index = {new_edge_index, new_edge_index + keep_both};

          if (!args.clear_outer) {
            /* Keep outside */
            new_split_edge_verts[insert_index.x] = int2(old_to_new_vertex_map[order.x],
                                                        new_vert_index);
            new_split_edge_map[insert_index.x] = {int(src_index), -1};
          }
          if (!args.clear_inner) {
            /* Keep inside */
            new_split_edge_verts[insert_index.y] = int2(old_to_new_vertex_map[order.y],
                                                        new_vert_index);
            new_split_edge_map[insert_index.y] = {int(src_index), -1};
          }
        }
      });

  /* Create edge index map */
  Array<int, 12> old_to_new_edge_map(src_num_edges);
  old_to_new_edge_map.fill(-1);
  edge_type_selections[EdgeIntersectType::Kept].foreach_index(
      [&](const int64_t index, const int64_t index_pos) {
        old_to_new_edge_map[index] = index_pos;
      });

  /* Polygons */
  Vector<int> new_to_old_poly_map, new_poly_offsets, intersected_poly;
  new_to_old_poly_map.reserve(src_num_polys);
  new_poly_offsets.reserve(src_num_polys);
  new_poly_offsets.append(0);

  Array<Vector<Vector<int, 4>, 2>, 12> new_split_polygons(src_num_polys);
  Array<Vector<int, 2>, 12> new_inter_edge_offsets(new_split_polygons.size());
  Array<Vector<int2, 2>, 12> new_inter_edge_verts(new_split_polygons.size());
  Array<Vector<int2, 2>, 12> new_inter_edge_map(new_split_polygons.size());

  std::atomic<int> new_inter_edge_index = 0;
  const int new_inter_edge_offset = num_kept_edges + new_split_edge_verts.size();
  const int new_total_face_count = threading::parallel_reduce<int>(
      src_polys.index_range(),
      512,
      0,
      [&](IndexRange src_poly_subrange, const int &identity) {
        int count = identity;
        for (const int64_t index_poly : src_poly_subrange) {
          Vector<int, 12> intersected;

          const Span<int> corners = src_corner_verts.slice(src_polys[index_poly]);
          const Span<int> corner_edges = src_corner_edges.slice(src_polys[index_poly]);

          const bool first_kept = is_kept_vertex[corners[0]];
          bool is_kept = first_kept;
          int kept_count = first_kept;
          for (const int64_t index : corners.index_range().drop_front(1)) {
            bool next_kept = is_kept_vertex[corners[index]];
            if (is_kept != next_kept) {
              intersected.append(index - 1);
            }
            kept_count += next_kept;
            is_kept = next_kept;
          }
          if (kept_count == 0) {
            /* Discard */
            continue;
          }
          if (is_kept != first_kept) {
            /* Last edge case */
            intersected.append(corners.size() - 1);
          }

          if (kept_count == corners.size()) {
            /* All kept */
            Vector<int> mapped_corner_edges(corner_edges.size());
            for (const int64_t index : mapped_corner_edges.index_range()) {
              const int64_t new_index = old_to_new_edge_map[corner_edges[index]];
              mapped_corner_edges[index] = new_index;
            }
            new_split_polygons[index_poly].append(std::move(mapped_corner_edges));
            count++;
          }
          else {
            BLI_assert(intersected.size() % 2 == 0);
            /* Split face(s). Iterate corner loop and form edges between adjacent intersection
             * pairs */

            /* Find the index for the edge in the intersected edge group.
             */
            auto fetch_split_edge = [&](const int64_t index, bool outside) {
              const int edge_index = corner_edges[index];

              IndexMask &intersect_mask = edge_type_selections[EdgeIntersectType::Intersect];
              auto new_edge_offset = intersect_mask.find(edge_index);
              if (!new_edge_offset.has_value()) {
                BLI_assert(false);
                return int2(0, 0); /* Panic. */
              }

              const int intersect_index = intersect_mask.iterator_to_index(
                  new_edge_offset.value());
              const int split_edge_sided_index = intersect_index * keep_count + !outside +
                                                 num_kept_edges;
              return int2{intersect_index, split_edge_sided_index};
            };
            /* Add an edge spanning two intersected edges from the intersected edge group indices.
             */
            auto add_intersection_edge = [&](const int2 inter_edge_a, const int2 inter_edge_b) {
              int new_index = new_inter_edge_index++;

              new_inter_edge_offsets[index_poly].append(new_index);
              /* Fetch vertex indices for the inserted vertices in the split */
              new_inter_edge_verts[index_poly].append(
                  int2{fn_intersect_vertex_index(inter_edge_a.x),
                       fn_intersect_vertex_index(inter_edge_b.x)});
              /* Find source edges to map data from */
              int2 src_edges {new_split_edge_map[inter_edge_a.x].x,
                                    new_split_edge_map[inter_edge_b.x].x};
              new_inter_edge_map[index_poly].append(src_edges);
              return new_index + new_inter_edge_offset;
            };
            auto increment = [&](const int64_t index) { return (index + 1) % corners.size(); };

            /* Perform a voting scheme to determine order for the intersection pairs since N-gons
             * may not lie in the plane and near parallel edges will cause robustness issues
             * if only a single intersection ís used to determine the ordering.
             *
             * Shift vote options:
             *  1: do shift
             *  0: no vote
             *  -1: no shift
             */
            int start_shift_vote = 0;
            // TODO: Vote shift
            const bool do_shift_start = start_shift_vote > 0;

            /* Construct corner loops */
            int next_intersect = intersected.size() - 2 + do_shift_start;
            while (next_intersect < intersected.size()) {
              const int2 start_pair = {intersected[next_intersect],
                                       intersected[(next_intersect + 1) % intersected.size()]};
              next_intersect = (next_intersect + 2) % intersected.size();
              int64_t index = increment(start_pair.y);
              bool outside = fn_is_outside(dist_buffer[corners[index]]);

              Vector<int, 12> poly_ecorners;
              int2 edge_offset_a = fetch_split_edge(start_pair.x, outside);
              int2 edge_offset_b = fetch_split_edge(start_pair.y, outside);
              poly_ecorners.append(edge_offset_a.y);
              poly_ecorners.append(add_intersection_edge(edge_offset_a, edge_offset_b));
              poly_ecorners.append(edge_offset_b.y);

              for (; index != start_pair.x; index = increment(index)) {
                if (index == intersected[next_intersect]) {
                  /* Traversed to next bisected edge */
                  edge_offset_a = fetch_split_edge(intersected[next_intersect++], outside);
                  edge_offset_b = fetch_split_edge(intersected[next_intersect], outside);
                  poly_ecorners.append(edge_offset_a.y);
                  poly_ecorners.append(add_intersection_edge(edge_offset_a, edge_offset_b));
                  poly_ecorners.append(edge_offset_b.y);
                  index = increment(intersected[next_intersect++]);
                }
                else {
                  /* Asserts edge is a 'kept' edge. */
                  int mapped_edge = old_to_new_edge_map[corner_edges[index]];
                  if (mapped_edge == -1) {
                    /* Not possible unless voting is wrong */
                    poly_ecorners.clear();
                    next_intersect = intersected.size();
                    break;
                  }
                  poly_ecorners.append(mapped_edge);
                }
              }
              /* Move to next loop (if any) */
              next_intersect += 2;
              /* Make poly */
              if (poly_ecorners.size() < 3) {
                /* Not possible unless voting is wrong */
                continue;
              }
              new_split_polygons[index_poly].append(std::move(poly_ecorners));
              count++;
            }
          }
        }
        return count;
      },
      [](int a, int b) { return a + b; });

  /* Condense arrays */
  Array<int, 12> new_split_polygon_indices(new_total_face_count);
  Array<Vector<int, 4>, 12> new_split_polygons_dense(new_total_face_count);

  Array<int2, 12> new_inter_edge_verts_dense(new_inter_edge_index);
  Array<int2, 12> new_inter_edge_map_dense(new_inter_edge_index);

  int poly_counter = 0;
  for (const int64_t index_poly : new_split_polygons.index_range()) {
    for (const int64_t i : new_inter_edge_offsets[index_poly].index_range()) {
      const int edge_offset = new_inter_edge_offsets[index_poly][i];
      new_inter_edge_verts_dense[edge_offset] = new_inter_edge_verts[index_poly][i];
      new_inter_edge_map_dense[edge_offset] = new_inter_edge_map[index_poly][i];
    }
    if (new_split_polygons[index_poly].is_empty()) {
      continue;
    }

    /* Iterate and make dense! */
    for (const int64_t i : new_split_polygons[index_poly].index_range()) {
      new_split_polygon_indices[poly_counter] = index_poly;
      new_split_polygons_dense[poly_counter] = std::move(new_split_polygons[index_poly][i]);
      ++poly_counter;
    }
  }

  /* Group Descriptors */
  std::array<VariantVertexGroup, 2> vertex_groups = {
      MeshVertexGroupCopyMask{kept_vertices},
      MeshVertexGroupLinear{split_index_pairs, split_lerp_weights}};

  std::array<VariantEdgeGroup, 3> edge_groups = {
      MeshEdgeGroupCopyMask{edge_type_selections[EdgeIntersectType::Kept],
                            old_to_new_vertex_map.as_span()},
      MeshEdgeGroupPair{new_split_edge_verts, new_split_edge_map},
      MeshEdgeGroupPair{new_inter_edge_verts_dense, new_inter_edge_map_dense}};
  std::array<VariantPolygonGroup, 2> poly_groups = {
      MeshPolygonGroup{new_split_polygon_indices, new_split_polygons_dense}};

  /* Create new mesh */
  Mesh *result = new_mesh_from_groups(mesh, vertex_groups, edge_groups, poly_groups);
  transfer_vertex_data(mesh, *result, vertex_groups, propagation_info);
  transfer_edge_data(mesh, *result, edge_groups, propagation_info);
  transfer_polygon_data(mesh, *result, poly_groups, propagation_info);

  /* Copy kept data */

  // TODO

  /* Handle intersected data */

  // TODO

  return result;
}  // namespace blender::geometry

}  // namespace blender::geometry
