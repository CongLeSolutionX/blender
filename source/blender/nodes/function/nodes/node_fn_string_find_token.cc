/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_string_utf8.h"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_string_find_token_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::String>("String").hide_label();
  b.add_input<decl::String>("Token");
  b.add_input<decl::Int>("Start Char").min(0);
  b.add_input<decl::Int>("Next Find").min(0).default_value(1);
  b.add_output<decl::Int>("Token Position");
}
static size_t string_find_token(const std::string_view s,
                                const std::string_view k,
                                const int *start,
                                const int *next)
{
  if (s.empty() || k.empty() || *start < 0 || *start > s.length()) {
    return -1;
  }
  if (*next == 0) {
    return 0;
  }
  size_t pos = *start;
  for (int i = 0; i < *next; ++i) {
    pos = s.find(k, pos);
    if (pos == std::string::npos) {
      return -1;
    }
    pos += k.length();
  }
  return pos - k.length();
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static auto find_fn = mf::build::SI4_SO<std::string, std::string, int, int, int>(
      "String Find Token",
      [](const std::string_view string, const std::string_view k, const int &start, const int &next) {
        return string_find_token(string, k, &start, &next);
      }); 

  builder.set_matching_fn(&find_fn);
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_STRING_FIND_TOKEN, "String Find Token", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.build_multi_function = node_build_multi_function;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_string_find_token_cc
