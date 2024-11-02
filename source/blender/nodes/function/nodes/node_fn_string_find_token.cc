/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_string_utf8.h"

#include "node_function_util.hh"
#include <codecvt>
#include <iomanip>

namespace blender::nodes::node_fn_string_find_token_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::String>("String").hide_label();
  b.add_input<decl::String>("Token");
  b.add_input<decl::Int>("Start Char").min(0);
  b.add_input<decl::Int>("Next Find").min(0).default_value(1);
  b.add_output<decl::Int>("Token Position");
}

static int string_find_token(const std::string_view a,
                             const std::string_view b,
                             const int *start,
                             const int *next)
{
  if (a.empty() || b.empty() || *start < 0 || *start > a.size()) {
    return -1;
  }
  if (*next == 0) {
    return 0;
  }
  std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> converter;
  std::u32string a_u32 = converter.from_bytes(a.data(), a.data() + a.size());
  std::u32string b_u32 = converter.from_bytes(b.data(), b.data() + b.size());

  size_t pos = *start;
  int count = 0;
  while ((pos = a_u32.find(b_u32, pos)) != std::u32string_view::npos) {
    count++;
    if (count == *next) {
      return pos;
    }
    pos += b_u32.size();
  }
  return -1;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static auto find_fn = mf::build::SI4_SO<std::string, std::string, int, int, int>(
      "String Find Token",
      [](const std::string_view s, const std::string_view t, const int &start, const int &next) {
        return string_find_token(s, t, &start, &next);
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
