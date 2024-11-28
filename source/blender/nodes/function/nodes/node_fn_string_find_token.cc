/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_string_utf8.h"
#include "node_function_util.hh"
#include <charconv>
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

std::u32string bli_str_utf8_as_u32string(const std::string &src_c)
{
  const size_t src_c_len = src_c.size();
  std::u32string result;
  result.reserve(src_c_len); 

  const char *src_c_end = src_c.data() + src_c_len;
  size_t index = 0;
  while (index < src_c_len) {
    const uint unicode = BLI_str_utf8_as_unicode_step_or_error(src_c.data(), src_c_len, &index);
    if (unicode != BLI_UTF8_ERR) {
      result.push_back(unicode);
    } else {
      result.push_back('?');
      const char *src_c_next = BLI_str_find_next_char_utf8(src_c.data() + index, src_c_end);
      index = size_t(src_c_next - src_c.data());
    }
  }

  return result;
}

static int string_find_token(const StringRef text,
                             const StringRef token,
                             const int start,
                             const int next)
{
  std::u32string a_u32 = bli_str_utf8_as_u32string(text);
  std::u32string b_u32 = bli_str_utf8_as_u32string(token);

  if (start < 0 || start > a_u32.size()) {
    return -1;
  }

  int pos = start;
  int count = 0;
  while ((pos = a_u32.find(b_u32, pos)) != std::u32string_view::npos) {
    count++;
    if (count == next) {
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
      [](const std::string &text, const std::string &token, const int &start, const int &next) {
        if (text == nullptr || token == nullptr || text.empty() || token.empty()) {
          return 0;
        }
        else if (next == 0) {
          return 0;
        }
        return string_find_token(text, token, start, next);
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
