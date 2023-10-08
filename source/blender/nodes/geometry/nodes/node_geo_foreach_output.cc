/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_compute_contexts.hh"
#include "BKE_scene.h"

#include "DEG_depsgraph_query.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_geometry.hh"
#include "NOD_socket.hh"
#include "NOD_zone_socket_items.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_foreach_output_cc {

NODE_STORAGE_FUNCS(NodeGeometryForEachOutput);

static void node_declare_dynamic(const bNodeTree & /*node_tree*/,
                                 const bNode &node,
                                 NodeDeclarationBuilder &b)
{
  const NodeGeometryForEachOutput &storage = node_storage(node);
  for (const int i : IndexRange(storage.output_items_num)) {
    const NodeForEachOutputItem &item = storage.output_items[i];
    const eNodeSocketDatatype socket_type = eNodeSocketDatatype(item.socket_type);
    const std::string identifier = ForEachOutputItemsAccessor::socket_identifier_for_item(item);
    const StringRef name = item.name;

    auto &input_decl = b.add_input(socket_type, name, identifier);
    auto &output_decl = b.add_output(socket_type, name, identifier);

    if (socket_type_supports_fields(socket_type)) {
      input_decl.supports_field();
      output_decl.field_on_all();
    }
  }
  b.add_input<decl::Extend>("", "__extend__");
  b.add_output<decl::Extend>("", "__extend__");
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryForEachOutput *data = MEM_cnew<NodeGeometryForEachOutput>(__func__);

  data->output_items = MEM_cnew_array<NodeForEachOutputItem>(1, __func__);
  data->output_items_num = 1;

  NodeForEachOutputItem &item = data->output_items[0];
  item.name = BLI_strdup(DATA_("Geometry"));
  item.socket_type = SOCK_GEOMETRY;
  item.identifier = data->output_next_identifier++;

  node->storage = data;
}

static void node_free_storage(bNode *node)
{
  socket_items::destruct_array<ForEachInputItemsAccessor>(*node);
  socket_items::destruct_array<ForEachOutputItemsAccessor>(*node);
  MEM_freeN(node->storage);
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeGeometryForEachOutput &src_storage = node_storage(*src_node);
  auto *dst_storage = MEM_new<NodeGeometryForEachOutput>(__func__, src_storage);
  dst_node->storage = dst_storage;

  socket_items::copy_array<ForEachInputItemsAccessor>(*src_node, *dst_node);
  socket_items::copy_array<ForEachOutputItemsAccessor>(*src_node, *dst_node);
}

static bool node_insert_link(bNodeTree *ntree, bNode *node, bNodeLink *link)
{
  return socket_items::try_add_item_via_any_extend_socket<ForEachOutputItemsAccessor>(
      *ntree, *node, *node, *link);
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_FOR_EACH_OUTPUT, "For-Each Output", NODE_CLASS_INTERFACE);
  ntype.initfunc = node_init;
  ntype.declare_dynamic = node_declare_dynamic;
  ntype.gather_link_search_ops = nullptr;
  ntype.insert_link = node_insert_link;
  node_type_storage(&ntype, "NodeGeometryForEachOutput", node_free_storage, node_copy_storage);
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_foreach_output_cc
