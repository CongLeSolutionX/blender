/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_mesh_types.h"

#include "BLI_virtual_array.hh"

#include "GEO_mesh_resample_topology.hh"

#include "BKE_attribute.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_resample_topology_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(N_("Mesh")).supported_type(GEO_COMPONENT_TYPE_MESH);
  b.add_input<decl::Int>(N_("Count"))
      .description(N_("Number of points to resample edges"))
      .hide_value()
      .field_on_all();
  b.add_output<decl::Geometry>(N_("Mesh")).propagate_all();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "try_to_fill_by_grid", 0, nullptr, ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = int(false);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");

  const Field<int> count_field = params.extract_input<Field<int>>("Count");

  const bNode &node = params.node();
  const bool try_to_fill_by_grid = bool(node.custom1);

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    if (!geometry_set.has_mesh()) {
      return;
    }

    const Mesh &mesh = *geometry_set.get_mesh_for_read();

    bke::MeshFieldContext field_context{mesh, ATTR_DOMAIN_EDGE};
    fn::FieldEvaluator evaluator{field_context, mesh.totedge};
    Array<int> count(mesh.totedge);
    evaluator.add_with_destination(count_field, count.as_mutable_span());
    evaluator.evaluate();

    Map<bke::AttributeIDRef, bke::AttributeKind> attributes;
    geometry_set.gather_attributes_for_propagation({GEO_COMPONENT_TYPE_MESH},
                                                   GEO_COMPONENT_TYPE_MESH,
                                                   false,
                                                   params.get_output_propagation_info("Mesh"),
                                                   attributes);

    Mesh &result = geometry::resample_topology(
        mesh, count.as_span(), try_to_fill_by_grid, std::move(attributes));
    geometry_set.replace_mesh(&result);
  });

  params.set_output("Mesh", std::move(geometry_set));
}

}  // namespace blender::nodes::node_geo_resample_topology_cc

void register_node_type_geo_resample_topology()
{
  namespace file_ns = blender::nodes::node_geo_resample_topology_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_RESAMPLE_TOPOLOGY, "Resample Topology", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.declare = file_ns::node_declare;
  ntype.initfunc = file_ns::node_init;
  nodeRegisterType(&ntype);
}
