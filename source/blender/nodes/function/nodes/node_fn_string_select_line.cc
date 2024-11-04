/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_string_ref.hh"
#include "BLI_string_utf8.h"
#include <iomanip>

#include "node_function_util.hh"

namespace blender::nodes::node_fn_string_select_line_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::String>("String").hide_label();
  b.add_input<decl::String>("Break");
  b.add_input<decl::Int>("Line Index").min(0);
  b.add_output<decl::String>("Out Line");
}

static std::string string_select_line(const StringRef text, const StringRef my_break, const int *i)
{
  std::string out_line = "";
  int pos = 0;
  if (*i == 0) {
    int next_pos = text.find(my_break, pos);
    if (next_pos == std::string::npos) {
      out_line = text.substr(pos);
    }
    else {
      out_line = text.substr(pos, next_pos - pos);
    }
    return out_line;
  }
  int count = 0;
  while ((pos = text.find(my_break, pos)) != std::string::npos) {
    count++;
    if (count == *i) {
      int next_pos = text.find(my_break, pos + my_break.size());
      if (next_pos == std::string::npos) {
        out_line = text.substr(pos + my_break.size());
      }
      else {
        out_line = text.substr(pos + my_break.size(), next_pos - (pos + my_break.size()));
      }
      break;
    }
    pos += my_break.size();
  }
  if (count < *i) {
    return "";
  }
  return out_line;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static auto count = mf::build::SI3_SO<std::string, std::string, int, std::string>(
      "String Select Line", [](const StringRef text, const StringRef my_break, const int &i) {
        if (text.is_empty() || my_break.is_empty()) {
          static std::string out_line = "";
          return out_line;
        }
        return string_select_line(text, my_break, &i);
      });

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
