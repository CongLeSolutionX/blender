/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BKE_volume_grid.hh"
#include "BKE_volume_openvdb.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "RNA_enum_types.hh"

namespace blender::nodes::node_geo_grid_deactivate_voxels_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }

  eCustomDataType data_type = eCustomDataType(node->custom1);

  grids::declare_grid_type_input(b, data_type, "Grid");
  b.add_input(data_type, "Value");
  b.add_input(data_type, "Tolerance");

  grids::declare_grid_type_output(b, data_type, "Grid");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = CD_PROP_FLOAT;
}

struct DeactivateVoxelsOp {
  GeoNodeExecParams params;

  template<typename T> bke::GVolumeGrid operator()()
  {
    using Converter = bke::grids::Converter<T>;

    bke::VolumeGrid<T> grid = this->params.extract_input<bke::VolumeGrid<T>>("Grid");
    if (!grid) {
      return {};
    }
    const T value = params.extract_input<T>("Value");
    const T tolerance = params.extract_input<T>("Tolerance");

    bke::VolumeGrid<T> output_grid(&grid.get_for_write());

    openvdb::tools::deactivate(
        output_grid.grid_for_write(output_grid.get_for_write().tree_access_token()),
        Converter::to_openvdb(value),
        Converter::to_openvdb(tolerance));

    return output_grid;
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  const eCustomDataType data_type = eCustomDataType(params.node().custom1);
  BLI_assert(grid_type_supported(data_type));

  DeactivateVoxelsOp deactivate_voxels_op = {params};
  bke::GVolumeGrid grid = grids::apply(data_type, deactivate_voxels_op);

  params.set_output("Grid", grid);
#else
  node_geo_exec_with_missing_openvdb(params);
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
                    grid_custom_data_type_items_filter_fn);
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_DEACTIVATE_VOXELS, "Deactivate Voxels", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_grid_deactivate_voxels_cc
