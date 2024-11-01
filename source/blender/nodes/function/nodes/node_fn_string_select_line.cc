/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_string_utf8.h"

#include <iomanip>

#include "node_function_util.hh"

namespace blender::nodes::node_fn_string_select_line_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::String>("String").hide_label();
  b.add_input<decl::String>("Break");
  b.add_input<decl::Int>("Line Index").min(0);
  b.add_output<decl::String>("Count");
}
static std::string string_Select_Line(const StringRef a, const StringRef b, const int *i)
{
  if (a.is_empty() || b.is_empty()) {
    return "";
  }
  std::string out_line = "";
  size_t pos = 0;
  if (*i == 0) {
    size_t next_pos = a.find(b, pos);
    if (next_pos == std::string::npos) {
      out_line = a.substr(pos);
    }
    else {
      out_line = a.substr(pos, next_pos - pos);
    }
    return out_line;
  }
  int count = 0;
  while ((pos = a.find(b, pos)) != std::string::npos) {
    count++;
    if (count == *i) {
      size_t next_pos = a.find(b, pos + b.size());
      if (next_pos == std::string::npos) {
        out_line = a.substr(pos + b.size());
      }
      else {
        out_line = a.substr(pos + b.size(), next_pos - (pos + b.size()));
      }
      break;
    }
    pos += b.size();
  }
  if (count < *i) {
    return "";
  }
  return out_line;
}
static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static auto count = mf::build::SI3_SO<std::string, std::string, int, std::string>(
      "String Select Line", [](const StringRef a, const StringRef b, const int &i) {
        return string_Select_Line(a, b, &i);
      });  // static auto count

  builder.set_matching_fn(&count);
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  fn_node_type_base(
      &ntype, FN_NODE_STRING_SELECT_LINE, "String select line", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.build_multi_function = node_build_multi_function;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_string_select_line_cc
