/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

// #include "BLI_hash.h"
#include "BLI_noise.hh"

#include "node_function_util.hh"

#include "NOD_socket_search_link.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

namespace blender::nodes::node_fn_random_value_cc {

NODE_STORAGE_FUNCS(NodeRandomValue)

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();

  if (node != nullptr) {
    const eCustomDataType data_type = eCustomDataType(node_storage(*node).data_type);
    switch (data_type) {
      case CD_PROP_FLOAT3:
        b.add_input<decl::Vector>("Min").supports_field();
        b.add_input<decl::Vector>("Max").default_value({1.0f, 1.0f, 1.0f}).supports_field();
        break;
      case CD_PROP_FLOAT:
        b.add_input<decl::Float>("Min").supports_field();
        b.add_input<decl::Float>("Max").default_value(1.0f).supports_field();
        break;
      case CD_PROP_INT32:
        b.add_input<decl::Int>("Min").supports_field();
        b.add_input<decl::Int>("Max").default_value(100).supports_field();
        break;
      case CD_PROP_BOOL:
        b.add_input<decl::Float>("Probability")
            .min(0.0f)
            .max(1.0f)
            .default_value(0.5f)
            .subtype(PROP_FACTOR)
            .supports_field()
            .make_available([](bNode &node) { node_storage(node).data_type = CD_PROP_BOOL; });
        break;
      default:
        BLI_assert_unreachable();
        break;
    }
  }

  b.add_input<decl::Int>("ID").implicit_field(implicit_field_inputs::id_or_index);
  b.add_input<decl::Int>("Seed").supports_field();

  if (node != nullptr) {
    const eCustomDataType data_type = eCustomDataType(node_storage(*node).data_type);
    b.add_output(data_type, "Value").dependent_field();
  }
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void fn_node_random_value_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeRandomValue *data = MEM_cnew<NodeRandomValue>(__func__);
  data->data_type = CD_PROP_FLOAT;
  node->storage = data;
}

static std::optional<eCustomDataType> node_type_from_other_socket(const bNodeSocket &socket)
{
  switch (socket.type) {
    case SOCK_FLOAT:
      return CD_PROP_FLOAT;
    case SOCK_BOOLEAN:
      return CD_PROP_BOOL;
    case SOCK_INT:
      return CD_PROP_INT32;
    case SOCK_VECTOR:
    case SOCK_RGBA:
      return CD_PROP_FLOAT3;
    default:
      return {};
  }
}

static void node_gather_link_search_ops(GatherLinkSearchOpParams &params)
{
  const NodeDeclaration &declaration = *params.node_type().static_declaration;
  const std::optional<eCustomDataType> type = node_type_from_other_socket(params.other_socket());
  if (!type) {
    return;
  }
  if (params.in_out() == SOCK_IN) {
    if (ELEM(*type, CD_PROP_INT32, CD_PROP_FLOAT3, CD_PROP_FLOAT)) {
      params.add_item(IFACE_("Min"), [type](LinkSearchOpParams &params) {
        bNode &node = params.add_node("FunctionNodeRandomValue");
        node_storage(node).data_type = *type;
        params.update_and_connect_available_socket(node, "Min");
      });
      params.add_item(IFACE_("Max"), [type](LinkSearchOpParams &params) {
        bNode &node = params.add_node("FunctionNodeRandomValue");
        node_storage(node).data_type = *type;
        params.update_and_connect_available_socket(node, "Max");
      });
    }
    search_link_ops_for_declarations(params, declaration.inputs.as_span().take_back(3));
  }
  else {
    params.add_item(IFACE_("Value"), [type](LinkSearchOpParams &params) {
      bNode &node = params.add_node("FunctionNodeRandomValue");
      node_storage(node).data_type = *type;
      params.update_and_connect_available_socket(node, "Value");
    });
  }
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const NodeRandomValue &storage = node_storage(builder.node());
  const eCustomDataType data_type = static_cast<eCustomDataType>(storage.data_type);

  switch (data_type) {
    case CD_PROP_FLOAT3: {
      static auto fn = mf::build::SI4_SO<float3, float3, int, int, float3>(
          "Random Vector",
          [](float3 min_value, float3 max_value, int id, int seed) -> float3 {
            const float x = noise::hash_to_float(seed, id, 0);
            const float y = noise::hash_to_float(seed, id, 1);
            const float z = noise::hash_to_float(seed, id, 2);
            return float3(x, y, z) * (max_value - min_value) + min_value;
          },
          mf::build::exec_presets::SomeSpanOrSingle<2>());
      builder.set_matching_fn(fn);
      break;
    }
    case CD_PROP_FLOAT: {
      static auto fn = mf::build::SI4_SO<float, float, int, int, float>(
          "Random Float",
          [](float min_value, float max_value, int id, int seed) -> float {
            const float value = noise::hash_to_float(seed, id);
            return value * (max_value - min_value) + min_value;
          },
          mf::build::exec_presets::SomeSpanOrSingle<2>());
      builder.set_matching_fn(fn);
      break;
    }
    case CD_PROP_INT32: {
      static auto fn = mf::build::SI4_SO<int, int, int, int, int>(
          "Random Int",
          [](int min_value, int max_value, int id, int seed) -> int {
            const float value = noise::hash_to_float(id, seed);
            /* Add one to the maximum and use floor to produce an even
             * distribution for the first and last values (See #93591). */
            return floor(value * (max_value + 1 - min_value) + min_value);
          },
          mf::build::exec_presets::SomeSpanOrSingle<2>());
      builder.set_matching_fn(fn);
      break;
    }
    case CD_PROP_BOOL: {
      static auto fn = mf::build::SI3_SO<float, int, int, bool>(
          "Random Bool",
          [](float probability, int id, int seed) -> bool {
            return noise::hash_to_float(id, seed) <= probability;
          },
          mf::build::exec_presets::SomeSpanOrSingle<1>());
      builder.set_matching_fn(fn);
      break;
    }
    default: {
      BLI_assert_unreachable();
      break;
    }
  }
}

static void node_register()
{
  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_RANDOM_VALUE, "Random Value", NODE_CLASS_CONVERTER);
  ntype.initfunc = fn_node_random_value_init;
  ntype.draw_buttons = node_layout;
  ntype.declare = node_declare;
  ntype.build_multi_function = node_build_multi_function;
  ntype.gather_link_search_ops = node_gather_link_search_ops;
  node_type_storage(
      &ntype, "NodeRandomValue", node_free_standard_storage, node_copy_standard_storage);
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_random_value_cc
