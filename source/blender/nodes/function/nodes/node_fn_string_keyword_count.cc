/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_string_utf8.h"

#include <iomanip>

#include "node_function_util.hh"

namespace blender::nodes::node_fn_string_keyword_count_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::String>("String").hide_label();
  b.add_input<decl::String>("Keyword");
  b.add_output<decl::Int>("Count");
}
static int string_keyword_count(const std::string *a,const std::string *b){
  if (a == nullptr || b == nullptr || a->empty() || b->empty()) {return 0;}
  int count = 0;
  size_t pos = 0;
  while ((pos = a->find(*b, pos)) != std::string::npos) {
      count++;
      pos += b->length();
  }
  return count;
}
static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static auto count = mf::build::SI2_SO<std::string,std::string, int>(
    "Keyword Count", [](const std::string &a , const std::string &b) 
  { return string_keyword_count(&a,&b); });//static auto count

  builder.set_matching_fn(&count);
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_STRING_KEYWORD_COUNT, "String Keyword Count", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.build_multi_function = node_build_multi_function;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_string_length_cc
