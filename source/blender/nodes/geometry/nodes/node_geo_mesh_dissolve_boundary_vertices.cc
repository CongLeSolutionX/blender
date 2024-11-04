/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute_filter.hh"

#include "GEO_mesh_dissolve.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_mesh_dissolve_boundary_vertices_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Mesh").supported_type({GeometryComponent::Type::Mesh});
  b.add_input<decl::Bool>("Selection").hide_value().field_on_all();

  b.add_output<decl::Geometry>("Mesh").propagate_all();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");
  const Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");

  GeometryComponentEditData::remember_deformed_positions_if_necessary(geometry_set);

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    const Mesh *src_mesh = geometry_set.get_mesh();
    if (src_mesh == nullptr) {
      return;
    }
    const bke::MeshFieldContext context(*src_mesh, AttrDomain::Point);
    FieldEvaluator evaluator(context, src_mesh->verts_num);
    evaluator.set_selection(selection_field);

    evaluator.evaluate();
    const IndexMask &mask = evaluator.get_evaluated_selection_as_mask();
    if (mask.is_empty()) {
      return;
    }

    geometry_set.replace_mesh(
        geometry::dissolve_boundary_verts(*src_mesh, mask, params.get_attribute_filter("Mesh")));
  });

  params.set_output("Mesh", std::move(geometry_set));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype,
                     GEO_NODE_DISSOLVE_BOUNDARY_VERTICES,
                     "Dissolve Boundary Vertices",
                     NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_mesh_dissolve_boundary_vertices_cc
