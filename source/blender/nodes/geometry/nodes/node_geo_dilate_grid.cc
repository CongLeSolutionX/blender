/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_openvdb_util.hh"
#include "node_geometry_util.hh"

#include "RNA_enum_types.hh"

#include "NOD_rna_define.hh"
#include "NOD_socket_search_link.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "BKE_volume_grid.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/tools/Morphology.h>
#endif

namespace blender::nodes::node_geo_dilate_grid_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }
  const eCustomDataType data_type = eCustomDataType(node->custom1);
  b.add_input(data_type, "Grid");
  b.add_input<decl::Int>("Iterations").default_value(1).min(0);
  b.add_output(data_type, "Grid");
}

static void search_link_ops(GatherLinkSearchOpParams &params)
{
  if (U.experimental.use_new_volume_nodes) {
    nodes::search_link_ops_for_basic_node(params);
  }
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
  uiItemR(layout, ptr, "neighbors_mode", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = CD_PROP_FLOAT;
  node->custom2 = GEO_NODE_GRID_NEIGHBOR_FACE;
}

template<typename T>
static void try_dilate_grid(GeoNodeExecParams params,
                            const int iterations,
                            const GeometryNodeGridNeighborTopology neighbors_mode)
{
  bke::VolumeGrid<T> volume_grid = params.extract_input<bke::VolumeGrid<T>>("Grid");
  if (!volume_grid) {
    return;
  }

  bke::VolumeTreeAccessToken access_token = volume_grid->tree_access_token();
  bke::OpenvdbGridType<T> &grid = volume_grid.grid_for_write(access_token);

  openvdb::tools::dilateActiveValues(
      grid.tree(), iterations, get_vdb_neighbors_mode(neighbors_mode));

  params.set_output("Grid", std::move(volume_grid));
}

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  const eCustomDataType data_type = eCustomDataType(params.node().custom1);
  const GeometryNodeGridNeighborTopology neighbors_mode = GeometryNodeGridNeighborTopology(
      params.node().custom2);
  BLI_assert(grid_type_supported(data_type));
  const int iterations = params.extract_input<int>("Iterations");

  switch (data_type) {
    case CD_PROP_FLOAT:
      try_dilate_grid<float>(params, iterations, neighbors_mode);
      break;
    case CD_PROP_FLOAT3:
      try_dilate_grid<float3>(params, iterations, neighbors_mode);
      break;
    default:
      BLI_assert_unreachable();
      break;
  }

  params.set_default_remaining_outputs();
#else
  params.set_default_remaining_outputs();
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without OpenVDB"));
#endif
}

static void node_rna(StructRNA *srna)
{
  RNA_def_node_enum(srna,
                    "data_type",
                    "Data Type",
                    "Type of grid data",
                    rna_enum_attribute_type_items,
                    NOD_inline_enum_accessors(custom1),
                    CD_PROP_FLOAT,
                    grid_type_items_fn);

  RNA_def_node_enum(srna,
                    "neighbors_mode",
                    "Neighbors Mode",
                    "Which voxel neighbors to use",
                    rna_enum_grid_neighbors_topology_items,
                    NOD_inline_enum_accessors(custom2),
                    GEO_NODE_GRID_NEIGHBOR_FACE);
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_DILATE_GRID, "Dilate Grid", NODE_CLASS_CONVERTER);

  ntype.declare = node_declare;
  ntype.gather_link_search_ops = search_link_ops;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_dilate_grid_cc
