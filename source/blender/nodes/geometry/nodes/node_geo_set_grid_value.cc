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

namespace blender::nodes::node_geo_set_grid_value_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }

  eCustomDataType data_type = eCustomDataType(node->custom1);

  b.add_input<decl::MaskGrid>("Grid");
  b.add_input(data_type, "Value").supports_field();
  b.add_input(data_type, "Background");

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

struct CaptureGridOp {
  GeoNodeExecParams params;

  template<typename T> bke::GVolumeGrid operator()()
  {
    const eCustomDataType data_type = eCustomDataType(params.node().custom1);
    const auto topo_grid = this->params.extract_input<bke::GVolumeGrid>("Grid");
    const fn::Field<T> value_field = this->params.extract_input<fn::Field<T>>("Value");
    const T background = this->params.extract_input<T>("Background");

    return grids::try_capture_field_as_grid(data_type, topo_grid, value_field, &background);
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  const eCustomDataType data_type = eCustomDataType(params.node().custom1);

  CaptureGridOp capture_op = {params};
  bke::GVolumeGrid grid = grids::apply(data_type, capture_op);

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

  geo_node_type_base(&ntype, GEO_NODE_SET_GRID_VALUE, "Set Grid Value", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_set_grid_value_cc
