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

using VariantVertexGroup = std::variant<MeshVertexGroupCopyMask, MeshVertexGroupLinear>;
using VariantEdgeGroup = std::variant<MeshEdgeGroupCopyMask, MeshEdgeGroupPair>;

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
          dst_edge_slice[pos] = int2(item->mapping_table[src_index_pair.x],
                                     item->mapping_table[src_index_pair.y]);
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

Mesh *new_mesh_from_groups(const Mesh &src_mesh,
                           const Span<VariantVertexGroup> vertex_groups,
                           const Span<VariantEdgeGroup> edge_groups)
{
  const int num_vertices = count_num_vertices(vertex_groups);
  const int num_edges = count_num_edges(edge_groups);

  Mesh *result = BKE_mesh_new_nomain_from_template(&src_mesh, num_vertices, num_edges, 0, 0);
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

  auto is_keep = [](const BisectArgs &args, bool is_outer_vertex) {
    return (is_outer_vertex && !args.clear_outer) || (!is_outer_vertex && !args.clear_inner);
  };

  IndexMaskMemory kept_memory;
  IndexMask kept_vertices = IndexMask::from_predicate(
      IndexMask(src_vert_range), GrainSize(512), kept_memory, [&](const int64_t i) {
        dist_buffer[i] = dist_signed_to_plane_v3(src_positions[i], args.plane);
        const bool is_outer = dist_buffer[i] >= 0.0f;
        const bool is_kept = is_keep(args, is_outer);
        is_kept_vertex[i].set(is_kept);
        return is_kept;
      });

  /* Compute edge intersections. */
  const Span<int2> src_edges = mesh.edges();
  // BLI_assert(src_num_vert == src_edges.size());

  Array<float, 12> edge_insertion_factor(src_num_edges);
  IndexMaskMemory intersected_memory;

  std::array<IndexMask, 3> edge_type_selections;
  IndexMask::from_groups<int64_t>(
      IndexMask(src_edges.index_range()),
      intersected_memory,
      [&](int64_t i) {
        const bool v1_kept = is_kept_vertex[src_edges[i].x];
        const bool v2_kept = is_kept_vertex[src_edges[i].y];
        if (v1_kept != v2_kept) {
          return int(EdgeIntersectType::Intersect);
        }
        /* No intersection and keep one => both vertices kept => edge kept => 1 otherwise 0 */
        return int(v1_kept);
      },
      MutableSpan<IndexMask>(edge_type_selections));

  /* Handle keep/discard all. */
  const int64_t num_edge_intersect = edge_type_selections[2].size() == 0;
  if (num_edge_intersect == 0) {
    if (edge_type_selections[EdgeIntersectType::Discarded].size() == 0) {
      return nullptr;
    }
    else if (edge_type_selections[EdgeIntersectType::Kept].size() == 0) {
      // TODO: Return empty
    }
    /* else: process normally, at least 1 disjoint mesh is discarded */
  }

  const OffsetIndices src_polys = mesh.faces();
  const Span<int> src_corner_verts = mesh.corner_verts();
  const Span<int> src_corner_edges = mesh.corner_edges();

  /* Create index mappings */
  /* Vertices */
  Array<int, 12> old_to_new_vertex_map(src_num_vert);
  kept_vertices.foreach_index([&](const int64_t index, const int64_t index_pos) {
    old_to_new_vertex_map[index] = index_pos;
  });

  /* Edges */
  /* Number of new edges in the split mesh can at most be: total + intersected * 1.5 (0.5 for
   * every other edge split due to newly formed edge inside the split face) */
  auto intersect_vertex_index = [&](const int64_t intersect_edge_index) {
    return intersect_edge_index + kept_vertices.size();
  };

  const int keep_count = !args.clear_inner + !args.clear_outer;
  const int keep_both = keep_count == 2;
  Array<float, 12> split_lerp_weights(edge_type_selections[EdgeIntersectType::Intersect].size());
  Array<int2, 12> split_index_pairs(edge_type_selections[EdgeIntersectType::Intersect].size());
  Array<int2, 12> new_split_edges(edge_type_selections[EdgeIntersectType::Intersect].size() *
                              keep_count);

  edge_type_selections[EdgeIntersectType::Intersect].foreach_index(
      GrainSize(512), [&](const int64_t src_index, const int64_t index_pos) {
        const float signed_dist_x = dist_buffer[src_edges[src_index].x];
        const float signed_dist_y = dist_buffer[src_edges[src_index].y];
        const float abs_d1 = abs(signed_dist_x);
        const float tot_dist = abs_d1 + abs(signed_dist_y);
        split_lerp_weights[index_pos] = math::safe_divide<float>(abs_d1, tot_dist);
        split_index_pairs[index_pos] = src_edges[src_index];

        /* Generate split edges IFF any side is kept */
        if (keep_count > 0) {
          const int new_vert_index = intersect_vertex_index(index_pos);
          const int new_edge_index = index_pos * keep_count;
          if (!args.clear_inner) {
            if (signed_dist_x < 0.0f) {
              new_split_edges[new_edge_index] = int2(old_to_new_vertex_map[src_edges[src_index].x],
                                            new_vert_index);
            }
            else if (signed_dist_y < 0.0f) {
              new_split_edges[new_edge_index] = int2(
                  old_to_new_vertex_map[src_edges[src_index].y], new_vert_index);
            }
          }
          if (!args.clear_outer) {
            if (signed_dist_x > 0.0f) {
              new_split_edges[new_edge_index + keep_both] = int2(
                  old_to_new_vertex_map[src_edges[src_index].x],
                                            new_vert_index);
            }
            else if (signed_dist_y > 0.0f) {
              new_split_edges[new_edge_index + keep_both] = int2(
                  old_to_new_vertex_map[src_edges[src_index].y], new_vert_index);
            }
          }
        }
      });

  std::array<VariantVertexGroup, 2> vertex_groups = {
      MeshVertexGroupCopyMask{kept_vertices},
      MeshVertexGroupLinear{split_index_pairs, split_lerp_weights}};

  std::array<VariantEdgeGroup, 2> edge_groups = {
      MeshEdgeGroupCopyMask{edge_type_selections[EdgeIntersectType::Kept],
                            old_to_new_vertex_map.as_span()},
      MeshEdgeGroupPair{new_split_edges, Span<int2>()}};

  /* Polygons */

  /* Vector<int> new_to_old_poly_map, new_poly_offsets, intersected_poly;
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
  */

  /* Create new mesh */
  Mesh *result = new_mesh_from_groups(mesh, vertex_groups, edge_groups);
  transfer_vertex_data(mesh, *result, vertex_groups, propagation_info);
  transfer_edge_data(mesh, *result, edge_groups, propagation_info);

  /* Copy kept data */

  // TODO

  /* Handle intersected data */

  // TODO

  return result;
}  // namespace blender::geometry

}  // namespace blender::geometry
