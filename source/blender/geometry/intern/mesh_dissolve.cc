/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <iostream>

#include "atomic_ops.h"

#include "DNA_mesh_types.h"

#include "BKE_attribute.hh"
#include "BLI_array_utils.hh"
#include "BLI_atomic_disjoint_set.hh"
#include "BLI_offset_indices.hh"
#include "BLI_ordered_edge.hh"
#include "BLI_sort.hh"
#include "BLI_struct_equality_utils.hh"
#include "BLI_task.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_filter.hh"
#include "BKE_attribute_math.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.hh"

#include "GEO_randomize.hh"

namespace blender::geometry {

static Array<int> create_reverse_offsets(const Span<int> indices, const int items_num)
{
  Array<int> offsets(items_num + 1, 0);
  offset_indices::build_reverse_offsets(indices, offsets);
  return offsets;
}

static Span<int> front_indices_to_same_value(const Span<int> indices, const Span<int> values)
{
  const int value = values[indices.first()];
  const int &first_other = *std::find_if(
      indices.begin(), indices.end(), [&](const int index) { return values[index] != value; });
  return indices.take_front(&first_other - indices.begin());
}

static void from_indices_large_groups(const Span<int> group_indices,
                                      MutableSpan<int> r_counts_to_offset,
                                      MutableSpan<int> r_indices)
{
  constexpr const int segment_size = 1024;
  constexpr const IndexRange segment(segment_size);
  const bool last_small_segmet = bool(group_indices.size() % segment_size);
  const int total_segments = group_indices.size() / segment_size + int(last_small_segmet);

  Array<int> src_indices(group_indices.size());
  threading::parallel_for_each(IndexRange(total_segments), [&](const int segment_index) {
    const IndexRange range = segment.shift(segment_size * segment_index);
    MutableSpan<int> segment_indices = src_indices.as_mutable_span().slice_safe(range);
    std::iota(segment_indices.begin(), segment_indices.end(), segment_size * segment_index);
    parallel_sort(segment_indices.begin(), segment_indices.end(), [&](const int a, const int b) {
      return group_indices[a] < group_indices[b];
    });

    for (Span<int> indices = segment_indices; !indices.is_empty();) {
      const int group = group_indices[indices.first()];
      const int step_size = front_indices_to_same_value(indices, group_indices).size();
      atomic_add_and_fetch_int32(&r_counts_to_offset[group], step_size);
      indices = indices.drop_front(step_size);
    }
  });

  const OffsetIndices<int> offset = offset_indices::accumulate_counts_to_offsets(
      r_counts_to_offset);
  Array<int> counts(offset.size(), 0);
  threading::parallel_for_each(IndexRange(total_segments), [&](const int segment_index) {
    const IndexRange range = segment.shift(segment_size * segment_index);
    const Span<int> segment_indices = src_indices.as_span().slice_safe(range);
    for (Span<int> indices = segment_indices; !indices.is_empty();) {
      const Span<int> indices_of_current_group = front_indices_to_same_value(indices,
                                                                             group_indices);
      const int step_size = indices_of_current_group.size();
      const int group = group_indices[indices.first()];
      const int start = atomic_add_and_fetch_int32(&counts[group], step_size) - step_size;
      const IndexRange dst_range = offset[group].slice(start, step_size);
      array_utils::copy(indices_of_current_group, r_indices.slice(dst_range));
      indices = indices.drop_front(step_size);
    }
  });
}

static Array<int> reverse_indices_in_groups(const Span<int> group_indices,
                                            const OffsetIndices<int> offsets)
{
  if (group_indices.is_empty()) {
    return {};
  }
  BLI_assert(*std::max_element(group_indices.begin(), group_indices.end()) < offsets.size());
  BLI_assert(*std::min_element(group_indices.begin(), group_indices.end()) >= 0);

  /* `counts` keeps track of how many elements have been added to each group, and is incremented
   * atomically by many threads in parallel. `calloc` can be measurably faster than a parallel fill
   * of zero. Alternatively the offsets could be copied and incremented directly, but the cost of
   * the copy is slightly higher than the cost of `calloc`. */
  int *counts = MEM_cnew_array<int>(size_t(offsets.size()), __func__);
  BLI_SCOPED_DEFER([&]() { MEM_freeN(counts); })
  Array<int> results(group_indices.size());
  threading::parallel_for(group_indices.index_range(), 1024, [&](const IndexRange range) {
    for (const int64_t i : range) {
      const int group_index = group_indices[i];
      const int index_in_group = atomic_fetch_and_add_int32(&counts[group_index], 1);
      results[offsets[group_index][index_in_group]] = int(i);
    }
  });
  return results;
}

static GroupedSpan<int> gather_groups(const Span<int> group_indices,
                                      const int groups_num,
                                      Array<int> &r_offsets,
                                      Array<int> &r_indices)
{
  if (group_indices.size() / groups_num > 1000) {
    r_offsets.reinitialize(groups_num + 1);
    r_offsets.as_mutable_span().fill(0);
    r_indices.reinitialize(group_indices.size());
    from_indices_large_groups(group_indices, r_offsets, r_indices);
  }
  else {
    r_offsets = create_reverse_offsets(group_indices, groups_num);
    r_indices = reverse_indices_in_groups(group_indices, r_offsets.as_span());
  }
  return {OffsetIndices<int>(r_offsets), r_indices};
}

static void mix_attributes(const bke::AttributeAccessor src_attributes,
                           const bke::AttrDomain domain,
                           const bke::AttributeFilter &attribute_filter,
                           const Span<int> src_to_dst_mapping,
                           bke::MutableAttributeAccessor dst_attributes)
{
  const int dst_domain_size = src_attributes.domain_size(domain);

  Array<int> mix_groups_offsets;
  Array<int> mix_groups_indices;
  const GroupedSpan<int> groups = gather_groups(
      src_to_dst_mapping, dst_domain_size, mix_groups_offsets, mix_groups_indices);

  src_attributes.foreach_attribute([&](const bke::AttributeIter &iter) {
    if (attribute_filter.allow_skip(iter.name)) {
      return;
    }
    if (iter.data_type == CD_PROP_STRING) {
      return;
    }
    const bke::GAttributeReader src = iter.get();

    bke::GSpanAttributeWriter dst = dst_attributes.lookup_or_add_for_write_only_span(
        iter.name, domain, iter.data_type);
    if (!dst) {
      return;
    }

    bke::attribute_math::convert_to_static_type(iter.data_type, [&](auto dymmu) {
      using T = decltype(dymmu);
      blender::bke::attribute_math::DefaultMixer<T> values(dst.span.typed<T>(), IndexMask(0));
      const VArraySpan<T> src_span(src.varray.typed<T>());
      threading::parallel_for(
          groups.index_range(),
          4098,
          [&](const IndexRange range) {
            for (const int group_i : range) {
              const Span<int> indices_to_mix = groups[group_i];
              for (const int src_i : indices_to_mix) {
                values.mix_in(group_i, src_span[src_i], 1.0f);
              }
            }
          },
          threading::accumulated_task_sizes(
              [&](const IndexRange range) { return groups.offsets[range].size(); }));
      values.finalize();
    });

    dst.finish();
  });
}

static void ensure_min_face_sizes(const GroupedSpan<int> corner_verts,
                                  const int faces_num,
                                  MutableSpan<int> face_sizes,
                                  MutableSpan<bool> verts_to_dissolve_selection)
{
  IndexMaskMemory memory;
  const IndexMask invalid_faces = IndexMask::from_predicate(
      IndexMask(faces_num), GrainSize(4096), memory, [&](const int face_i) {
        return face_sizes[face_i] < 3;
      });

  static constexpr int min_requered_verts = 3;

  invalid_faces.foreach_index_optimized<int>(GrainSize(4096), [&](const int face_i) {
    const Span<int> face_verts = corner_verts[face_i];
    const Span<int> front_corner_selection = face_verts.take_front(min_requered_verts);
    const int forced_face_vertices_num = std::count_if(
        front_corner_selection.begin(), front_corner_selection.end(), [&](const int v) {
          return verts_to_dissolve_selection[v];
        });
    face_sizes[face_i] += forced_face_vertices_num;
  });

  invalid_faces.foreach_index_optimized<int>(GrainSize(4096), [&](const int face_i) {
    const Span<int> face_verts = corner_verts[face_i];
    const Span<int> front_corner_selection = face_verts.take_front(min_requered_verts);
    verts_to_dissolve_selection.fill_indices(front_corner_selection, false);
  });
}

static VectorSet<OrderedEdge> dissolved_edges_for_verts(const Span<int2> src_edges,
                                                        const GroupedSpan<int> vert_to_edge_map,
                                                        const IndexMask &verts_to_dissolve_mask,
                                                        const IndexMask &keeped_verts_mask,
                                                        MutableSpan<int> old_to_new_edges_map,
                                                        Array<int2> &r_new_edges)
{
  AtomicDisjointSet edges_sets(src_edges.size());
  verts_to_dissolve_mask.foreach_index(GrainSize(4096), [&](const int vert_i) {
    const Span<int> edges = vert_to_edge_map[vert_i];
    BLI_assert(edges.size() == 2);
    edges_sets.join(edges[0], edges[1]);
  });

  const int total_edges = edges_sets.calc_reduced_ids(old_to_new_edges_map);

  r_new_edges.reinitialize(total_edges);
  r_new_edges.as_mutable_span().fill(int2(-1));

  keeped_verts_mask.foreach_index([&](const int vert_i) {
    for (const int edge_i : vert_to_edge_map[vert_i]) {
      const int edge_index = old_to_new_edges_map[edge_i];
      if (r_new_edges[edge_index][0] == -1) {
        r_new_edges[edge_index][0] = vert_i;
      }
      else {
        BLI_assert(r_new_edges[edge_index][1] == -1);
        r_new_edges[edge_index][1] = vert_i;
      }
    }
  });
  BLI_assert(!r_new_edges.as_span().cast<int>().contains(-1));

  VectorSet<OrderedEdge> unique_edges;
  unique_edges.reserve(r_new_edges.size());
  for (const int2 edge : r_new_edges) {
    unique_edges.add(edge);
  }

  threading::parallel_for(IndexRange(src_edges.size()), 2048, [&](const IndexRange range) {
    for (const int edge_i : range) {
      old_to_new_edges_map[edge_i] = unique_edges.index_of(
          r_new_edges[old_to_new_edges_map[edge_i]]);
    }
  });

  return unique_edges;
}

struct FaceKey {
  int face_i;
  int first_corner_vert;
  BLI_STRUCT_EQUALITY_OPERATORS_2(FaceKey, face_i, first_corner_vert);

  FaceKey(const int face_index, const GroupedSpan<int> faces)
      : face_i(face_index), first_corner_vert(faces[face_index][0])
  {
  }
};

struct FaceHash {
  uint64_t operator()(const FaceKey value) const
  {
    return uint64_t(value.first_corner_vert);
  }
};

struct FacesEquality {
  GroupedSpan<int> sorted_corners;
  bool operator()(const FaceKey a, const FaceKey b) const
  {
    return sorted_corners[a.face_i] == sorted_corners[b.face_i];
  }
};

static Array<int> deduplicate_faces_for_verts(const GroupedSpan<int> dst_corner_verts,
                                              MutableSpan<int> old_to_new_faces_map,
                                              MutableSpan<int> old_to_new_corners_map)
{
  Array<int> sorted_corners(dst_corner_verts.data.size());
  Array<int> reverse_corners(dst_corner_verts.data.size());
  Array<int> sorted_corner_verts(dst_corner_verts.data.size());

  threading::parallel_for(
      dst_corner_verts.index_range(),
      8192,
      [&](const IndexRange range) {
        for (const int face_i : range) {
          const IndexRange face = dst_corner_verts.offsets[face_i];
          const Span<int> unordered_verts = dst_corner_verts.data.slice(face);
          MutableSpan<int> corners = sorted_corners.as_mutable_span().slice(face);
          array_utils::fill_index_range(corners, 0);
          parallel_sort(
              corners.begin(), corners.end(), [&](const int corner_a, const int corner_b) {
                return unordered_verts[corner_a] < unordered_verts[corner_b];
              });
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return dst_corner_verts.offsets[range].size(); }));

  threading::parallel_for(dst_corner_verts.index_range(), 8192, [&](const IndexRange range) {
    for (const int face_i : range) {
      const IndexRange face = dst_corner_verts.offsets[face_i];
      const Span<int> unordered_verts = dst_corner_verts.data.slice(face);
      const Span<int> corners = sorted_corners.as_span().slice(face);
      MutableSpan<int> sorted_verts = sorted_corner_verts.as_mutable_span().slice(face);
      array_utils::gather(unordered_verts, corners, sorted_verts);
    }
  });

  threading::parallel_for(dst_corner_verts.index_range(), 8192, [&](const IndexRange range) {
    for (const int face_i : range) {
      const IndexRange face = dst_corner_verts.offsets[face_i];
      const Span<int> corners = sorted_corners.as_span().slice(face);
      MutableSpan<int> reverse = reverse_corners.as_mutable_span().slice(face);
      for (const int corner_i : face.index_range()) {
        reverse[corners[corner_i]] = corner_i;
      }
    }
  });

  const GroupedSpan<int> sorted_face_verts(dst_corner_verts.offsets, sorted_corner_verts);
  VectorSet<FaceKey, DefaultProbingStrategy, FaceHash, FacesEquality> faces(
      FaceHash{}, FacesEquality{sorted_face_verts});
  for (const int face_i : dst_corner_verts.index_range()) {
    const FaceKey face_key(face_i, sorted_face_verts);
    faces.add(face_key);
  }

  threading::parallel_for(dst_corner_verts.index_range(), 8192, [&](const IndexRange range) {
    for (const int face_i : range) {
      const FaceKey face_key(face_i, sorted_face_verts);
      old_to_new_faces_map[face_i] = faces.index_of(face_key);
    }
  });

  threading::parallel_for(dst_corner_verts.index_range(), 8192, [&](const IndexRange range) {
    for (const int face_i : range) {
      const FaceKey face_key(face_i, sorted_face_verts);
      const int result_face_i = faces.lookup_key(face_key).face_i;

      const IndexRange face = dst_corner_verts.offsets[face_i];
      if (face_i == result_face_i) {
        array_utils::fill_index_range<int>(old_to_new_corners_map.slice(face), face.start());
        continue;
      }
      const IndexRange result_face = dst_corner_verts.offsets[result_face_i];
      BLI_assert(face.size() == result_face.size());
      const Span<int> result_face_reverse_corners = reverse_corners.as_span().slice(result_face);
      const Span<int> corners = sorted_corners.as_span().slice(face);
      MutableSpan<int> mapped_corners = old_to_new_corners_map.slice(face);
      std::transform(
          corners.begin(), corners.end(), mapped_corners.begin(), [&](const int ordered_i) {
            return result_face[result_face_reverse_corners[ordered_i]];
          });
    }
  });

  Array<int> unique_faces(faces.size());
  threading::parallel_for(faces.index_range(), 8192, [&](const IndexRange range) {
    for (const int face_i : range) {
      unique_faces[face_i] = faces[face_i].face_i;
    }
  });

  return unique_faces;
}

Mesh *dissolve_boundary_verts(const Mesh &src_mesh,
                              const IndexMask &verts_mask,
                              const bke::AttributeFilter &attribute_filter)
{
  const Span<int2> src_edges = src_mesh.edges();
  const OffsetIndices<int> src_faces = src_mesh.faces();
  const Span<int> src_corner_verts = src_mesh.corner_verts();

  Array<int> vert_to_edge_offsets;
  Array<int> vert_to_edge_indices;
  const GroupedSpan<int> vert_to_edge_map = bke::mesh::build_vert_to_edge_map(
      src_edges, src_mesh.verts_num, vert_to_edge_offsets, vert_to_edge_indices);

  IndexMaskMemory memory;
  const IndexMask boundary_verts_to_dissolve_mask = IndexMask::from_predicate(
      verts_mask, GrainSize(4096), memory, [&](const int vert_i) {
        return vert_to_edge_map[vert_i].size() == 2;
      });

  Array<bool> verts_to_dissolve_selection(src_mesh.verts_num);
  boundary_verts_to_dissolve_mask.to_bools(verts_to_dissolve_selection);

  Array<int> new_faces_sizes(src_mesh.faces_num + 1);
  threading::parallel_for(
      IndexRange(src_mesh.faces_num),
      4096,
      [&](const IndexRange range) {
        for (const int face_i : range) {
          const IndexRange face = src_faces[face_i];
          const Span<int> corner_verts = src_corner_verts.slice(face);
          const int dissolved_verts_num = std::count_if(
              corner_verts.begin(), corner_verts.end(), [&](const int v) {
                return verts_to_dissolve_selection[v];
              });
          new_faces_sizes[face_i] = face.size() - dissolved_verts_num;
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return src_faces[range].size(); }));

  ensure_min_face_sizes({src_faces, src_corner_verts},
                        src_mesh.faces_num,
                        new_faces_sizes,
                        verts_to_dissolve_selection);

  const IndexMask verts_to_dissolve_mask = IndexMask::from_bools(
      boundary_verts_to_dissolve_mask, verts_to_dissolve_selection, memory);
  const IndexMask keeped_verts_mask = verts_to_dissolve_mask.complement(
      IndexMask(src_mesh.verts_num), memory);
  const IndexMask keeped_corners_mask = IndexMask::from_predicate(
      IndexMask(src_mesh.corners_num), GrainSize(4096), memory, [&](const int corner_i) {
        return !verts_to_dissolve_selection[src_corner_verts[corner_i]];
      });

  Array<int> new_verts_remap(keeped_verts_mask.min_array_size(), -1);
  index_mask::build_reverse_map(keeped_verts_mask, new_verts_remap.as_mutable_span());

  const OffsetIndices<int> dst_faces = offset_indices::accumulate_counts_to_offsets(
      new_faces_sizes);

  Array<int> corner_verts(dst_faces.total_size());
  array_utils::gather(src_corner_verts, keeped_corners_mask, corner_verts.as_mutable_span());

  Array<int2> new_edges;
  Array<int> old_to_new_edges_map(src_mesh.edges_num);
  VectorSet<OrderedEdge> unique_edges;

  Array<int> old_to_new_faces_map(src_mesh.faces_num);
  Array<int> old_to_new_corners_map(keeped_corners_mask.size());
  Array<int> unique_faces;

  threading::parallel_invoke(
      src_mesh.edges_num + src_mesh.faces_num > 1000,
      [&]() {
        unique_edges = dissolved_edges_for_verts(src_edges,
                                                 vert_to_edge_map,
                                                 verts_to_dissolve_mask,
                                                 keeped_verts_mask,
                                                 old_to_new_edges_map,
                                                 new_edges);
      },
      [&]() {
        unique_faces = deduplicate_faces_for_verts(
            {dst_faces, corner_verts}, old_to_new_faces_map, old_to_new_corners_map);
      });

  const int total_unique_edges = unique_edges.size();
  const int total_unique_faces = unique_faces.size();

  Array<int> unique_face_sizes(total_unique_faces + 1);
  threading::parallel_for(IndexRange(total_unique_faces), 4096, [&](const IndexRange range) {
    for (const int face_i : range) {
      unique_face_sizes[face_i] = dst_faces[unique_faces[face_i]].size();
    }
  });
  const OffsetIndices<int> unique_dst_faces = offset_indices::accumulate_counts_to_offsets(
      unique_face_sizes);

  Mesh *dst_mesh = BKE_mesh_new_nomain(keeped_verts_mask.size(),
                                       total_unique_edges,
                                       total_unique_faces,
                                       unique_dst_faces.total_size());
  BKE_mesh_copy_parameters_for_eval(dst_mesh, &src_mesh);

  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh->attributes_for_write();

  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Point,
                         bke::AttrDomain::Point,
                         attribute_filter,
                         keeped_verts_mask,
                         dst_attributes);

  MutableSpan<int2> dst_edges = dst_mesh->edges_for_write();
  array_utils::copy(unique_edges.as_span().cast<int2>(), dst_edges);
  array_utils::gather(
      new_verts_remap.as_span(), dst_edges.as_span().cast<int>(), dst_edges.cast<int>());
  BLI_assert(!dst_edges.cast<int>().as_span().contains(-1));

  mix_attributes(src_attributes,
                 bke::AttrDomain::Edge,
                 bke::attribute_filter_with_skip_ref(attribute_filter, {".edge_verts"}),
                 old_to_new_edges_map,
                 dst_attributes);

  array_utils::copy(unique_dst_faces.data(), dst_mesh->face_offsets_for_write());
  mix_attributes(
      src_attributes,
      bke::AttrDomain::Face,
      bke::attribute_filter_with_skip_ref(attribute_filter, {/*"ID", "material_index"*/}),
      old_to_new_faces_map,
      dst_attributes);
  // bke::gather_attributes(src_attributes, bke::AttrDomain::Face, attribute_filter,
  // unique_faces.as_span(), dst_attributes);

  mix_attributes(
      src_attributes,
      bke::AttrDomain::Corner,
      bke::attribute_filter_with_skip_ref(attribute_filter, {".corner_vert", ".corner_edge"}),
      old_to_new_corners_map,
      dst_attributes);

  MutableSpan<int> dst_corner_verts = dst_mesh->corner_verts_for_write();
  threading::parallel_for(unique_faces.index_range(), 4096, [&](const IndexRange range) {
    for (const int face_i : range) {
      const IndexRange face = unique_dst_faces[face_i];
      const IndexRange dst_face = dst_faces[unique_faces[face_i]];
      dst_corner_verts.slice(face).copy_from(corner_verts.as_span().slice(dst_face));
    }
  });
  // array_utils::gather(src_corner_verts, keeped_corners_mask, dst_corner_verts);

  MutableSpan<int> dst_corner_edges = dst_mesh->corner_edges_for_write();
  threading::parallel_for(
      IndexRange(total_unique_faces),
      2048,
      [&](const IndexRange range) {
        for (const int face_i : range) {
          const IndexRange face = unique_dst_faces[face_i];
          const Span<int> corner_verts = dst_corner_verts.slice(face);
          MutableSpan<int> corner_edges = dst_corner_edges.slice(face);
          for (const int corner_i : face.index_range().drop_back(1)) {
            corner_edges[corner_i] = unique_edges.index_of(
                OrderedEdge(corner_verts[corner_i], corner_verts[corner_i + 1]));
          }
          corner_edges.last() = unique_edges.index_of(
              OrderedEdge(corner_verts.last(), corner_verts.first()));
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return dst_faces[range].size(); }));

  array_utils::gather(new_verts_remap.as_span(), dst_corner_verts.as_span(), dst_corner_verts);
  BLI_assert(!dst_corner_verts.as_span().contains(-1));

  dst_mesh->tag_positions_changed();
  dst_mesh->tag_topology_changed();

  BLI_assert(BKE_mesh_is_valid(dst_mesh));

  debug_randomize_vert_order(dst_mesh);
  debug_randomize_edge_order(dst_mesh);
  debug_randomize_face_order(dst_mesh);

  return dst_mesh;
}

}  // namespace blender::geometry
