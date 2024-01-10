/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BKE_volume_grid.hh"
#include "BKE_volume_openvdb.hh"

#include "NOD_rna_define.hh"

#include "RNA_enum_types.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/tools/Morphology.h>
#endif

namespace blender::nodes::node_geo_combine_grid_topology_cc {

static const eCustomDataType DummyMaskGridType = CD_PROP_FLOAT;

static void node_declare(NodeDeclarationBuilder &b)
{
  /* XXX Grid values are ignored, float socket is a placeholder for type-less grid socket. */

  grids::declare_grid_type_output(b, DummyMaskGridType, "Grid");

  grids::declare_grid_type_input(b, DummyMaskGridType, "Grid");
  /* XXX should be a multi-input for parity with the mesh boolean node, but CPPType is not defined
   * for Vector<ValueOrField<T>>. */
  grids::declare_grid_type_input(b, DummyMaskGridType, "Grids") /*.multi_input()*/;
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "operation", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = GEO_NODE_BOOLEAN_INTERSECT;
}

#ifdef WITH_OPENVDB
static bke::GVolumeGrid try_combine_grids(GeoNodeExecParams params)
{
  const GeometryNodeBooleanOperation operation = GeometryNodeBooleanOperation(
      params.node().custom1);

  bke::GVolumeGrid primary_grid = params.extract_input<bke::GVolumeGrid>("Grid");
  if (!primary_grid) {
    return {};
  }

  // const Vector<bke::VolumeGrid *> secondary_grids = grids::extract_grid_multi_input(
  //     params, "Grids", DummyMaskGridType);
  const Vector<bke::GVolumeGrid> secondary_grids = {
      params.extract_input<bke::GVolumeGrid>("Grids")};

  bke::VolumeTreeAccessToken primary_tree_token;
  primary_grid.get_for_write()
      .grid_for_write(primary_tree_token)
      .apply<SupportedVDBGridTypes>([&](auto &primary_grid) {
        for (const bke::GVolumeGrid &secondary_grid : secondary_grids) {
          bke::VolumeTreeAccessToken secondary_tree_token;
          secondary_grid->grid(secondary_tree_token)
              .apply<SupportedVDBGridTypes>([&](const auto &secondary_grid) {
                switch (operation) {
                  case GEO_NODE_BOOLEAN_INTERSECT: {
                    primary_grid.topologyIntersection(secondary_grid);
                    break;
                  }
                  case GEO_NODE_BOOLEAN_UNION: {
                    primary_grid.topologyUnion(secondary_grid);
                    break;
                  }
                  case GEO_NODE_BOOLEAN_DIFFERENCE: {
                    primary_grid.topologyDifference(secondary_grid);
                    break;
                  }
                }
              });
        }
      });

  return primary_grid;
}
#endif

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  bke::GVolumeGrid output_grid = try_combine_grids(params);
  params.set_output("Grid", output_grid);
#else
  node_geo_exec_with_missing_openvdb(params);
#endif
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem rna_node_geometry_boolean_method_items[] = {
      {GEO_NODE_BOOLEAN_INTERSECT,
       "INTERSECT",
       0,
       "Intersect",
       "Keep voxels that are active in all grids"},
      {GEO_NODE_BOOLEAN_UNION, "UNION", 0, "Union", "Activate voxels that are active in any grid"},
      {GEO_NODE_BOOLEAN_DIFFERENCE,
       "DIFFERENCE",
       0,
       "Difference",
       "Deactivate voxels that are active in other grids"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "operation",
                    "Operation",
                    "",
                    rna_node_geometry_boolean_method_items,
                    NOD_inline_enum_accessors(custom1),
                    GEO_NODE_BOOLEAN_INTERSECT);
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_COMBINE_GRID_TOPOLOGY, "Combine Grid Topology", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_combine_grid_topology_cc
