/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_curves.hh"

#include "DNA_mesh_types.h"

#include "GEO_mesh_to_curve.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_edge_paths_to_curves_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Mesh").supported_type(GeometryComponent::Type::Mesh);
  b.add_input<decl::Bool>("Start Vertices").default_value(true).hide_value().field_on_all();
  b.add_input<decl::Int>("Next Vertex Index").default_value(-1).hide_value().field_on_all();
  b.add_output<decl::Geometry>("Curves").propagate_all();
}

static Curves *edge_paths_to_curves_convert(
    const Mesh &mesh,
    const IndexMask &start_verts_mask,
    const AnonymousAttributePropagationInfo &propagation_info,
    MutableSpan<int> next_indices)
{
  const IndexRange vert_range(mesh.totvert);

  /* Do not create curves from first point with incorrect index. */
  IndexMaskMemory memory;
  IndexMask valid_start_verts = IndexMask::from_predicate(
      start_verts_mask, GrainSize(4096), memory, [&](const int vert_i) {
        return vert_range.contains(next_indices[vert_i]);
      });

  /* All the next points can pointing to itself. */
  threading::parallel_for(next_indices.index_range(), 4096, [&](const IndexRange range) {
    MutableSpan<int> next_indices_range = next_indices.slice(range);
    for (const int vert_i : range) {
      int &next_vert = next_indices[vert_i];
      if (UNLIKELY(!vert_range.contains(next_vert))) {
        next_vert = vert_i;
      }
    }
  });

  /* Make sure it is okay to subtract `in_progress` value as nothing. */
  const constexpr int in_progress = 0;
  const constexpr int non_checked = -1;

  Array<int> rank(mesh.totvert, non_checked);
  const auto rank_for_vertex = [&](const int vertex) -> int {
    if (rank[vertex] != non_checked) {
      return rank[vertex];
    }

    int total_rank = 0;
    int last_index = vertex;
    for (; rank[last_index] == non_checked; last_index = next_indices[last_index]) {
      rank[last_index] = in_progress;
      total_rank++;
    }

    /* End of list is a loop that is larger than single point. Count rank of the whole loop as a
     * single one. */
    if (UNLIKELY(last_index != next_indices[last_index] && rank[last_index] == in_progress)) {
      /* Size of loop can be deduced in loop above, but this is not really important to be
       * supported in general. This shouldn't impact regular use case. */
      int rank_of_loop = 1;
      for (int current_vert = next_indices[last_index]; current_vert != last_index;
           current_vert = next_indices[current_vert])
      {
        rank_of_loop++;
      }
      rank[last_index] = rank_of_loop;
      for (int current_vert = next_indices[last_index]; current_vert != last_index;
           current_vert = next_indices[current_vert])
      {
        /* For any start point, whole loops have to be gathered. */
        rank[current_vert] = rank_of_loop;
      }
      total_rank -= rank_of_loop;
    }
    total_rank += rank[last_index];

    for (int current_vert = vertex; rank[current_vert] == in_progress;
         current_vert = next_indices[current_vert])
    {
      rank[current_vert] = total_rank;
      total_rank--;
    }
    return rank[vertex];
  };

  Array<int> curve_offsets(valid_start_verts.size() + 1);
  valid_start_verts.foreach_index([&](const int first_vert, const int vert_pos) {
    curve_offsets[vert_pos] = rank_for_vertex(first_vert);
  });

  OffsetIndices<int> curves = offset_indices::accumulate_counts_to_offsets(curve_offsets);
  if (curves.is_empty()) {
    return nullptr;
  }

  Array<int> indices(curves.total_size());
  valid_start_verts.foreach_index(GrainSize(1024), [&](const int first_vert, const int pos) {
    MutableSpan<int> curve_indices = indices.as_mutable_span().slice(curves[pos]);
    int current_vert = first_vert;
    for (const int i : curve_indices.index_range()) {
      curve_indices[i] = current_vert;
      current_vert = next_indices[current_vert];
    }
  });

  Curves *curves_id = bke::curves_new_nomain(
      geometry::create_curve_from_vert_indices(mesh.attributes(),
                                               indices,
                                               curve_offsets.as_span().drop_back(1),
                                               IndexRange(0),
                                               propagation_info));
  return curves_id;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    const Mesh *mesh = geometry_set.get_mesh();
    if (mesh == nullptr) {
      geometry_set.keep_only({GeometryComponent::Type::Instance});
      return;
    }

    const bke::MeshFieldContext context{*mesh, ATTR_DOMAIN_POINT};
    fn::FieldEvaluator evaluator{context, mesh->totvert};
    Array<int> next_vert(mesh->totvert);
    evaluator.add_with_destination(params.get_input<Field<int>>("Next Vertex Index"),
                                   next_vert.as_mutable_span());
    evaluator.add(params.get_input<Field<bool>>("Start Vertices"));
    evaluator.evaluate();
    IndexMask start_verts = evaluator.get_evaluated_as_mask(1);

    if (start_verts.is_empty()) {
      geometry_set.keep_only({GeometryComponent::Type::Instance});
      return;
    }

    geometry_set.replace_curves(edge_paths_to_curves_convert(
        *mesh, start_verts, params.get_output_propagation_info("Curves"), next_vert));
    geometry_set.keep_only({GeometryComponent::Type::Curve, GeometryComponent::Type::Instance});
  });

  params.set_output("Curves", std::move(geometry_set));
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_EDGE_PATHS_TO_CURVES, "Edge Paths to Curves", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_edge_paths_to_curves_cc
