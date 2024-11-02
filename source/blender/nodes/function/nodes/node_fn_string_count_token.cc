/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_string_utf8.h"

#include <iomanip>

#include "node_function_util.hh"

namespace blender::nodes::node_fn_string_count_token_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::String>("String").hide_label();
  b.add_input<decl::String>("Token");
  b.add_output<decl::Int>("Count");
}
static int string_count_token(const std::string_view a, const std::string_view b)
{
  if (a.empty() || b.empty()) {
    return 0;
  }
  int count = 0;
  size_t pos = 0;
  while ((pos = a.find(b, pos)) != std::string::npos) {
    count++;
    pos += b.length();
  }
  return count;
}
static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static auto count = mf::build::SI2_SO<std::string, std::string, int>(
      "String Count Token",
      [](const std::string_view a, const std::string_view b) { return string_count_token(a, b); });

  builder.set_matching_fn(&count);
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  fn_node_type_base(
      &ntype, FN_NODE_STRING_COUNT_TOKEN, "String Count Token", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.build_multi_function = node_build_multi_function;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_string_count_token_cc
