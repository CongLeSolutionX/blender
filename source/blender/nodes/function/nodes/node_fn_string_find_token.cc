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
class char_length {
 public:
  static constexpr int array[5] = {0, 0x7F, 0xDF, 0xEF, 0xF7};
  static int to_int(int length)
  {
    return array[length];
  }
  static int to_int(char c)
  {
    unsigned char uc;
    if (uc <= array[1]) {
      return 1;
    }
    else if (uc <= array[2]) {
      return 2;
    }
    else if (uc <= array[3]) {
      return 3;
    }
    else if (uc <= array[4]) {
      return 4;
    }
    else {
      return -1;
    }
  }
  static bool equal(char c, int length)
  {
    return to_int(c) == length;
  }
};

std::u32string u32_from_utf8(const StringRef &utf8)
{
  std::u32string u32;
  u32.reserve(utf8.size());

  const char *from = utf8.data();
  const char *from_end = from + utf8.size();

  while (from < from_end) {
    const unsigned char lead = static_cast<unsigned char>(*from);
    const int lead_int = static_cast<int>(static_cast<unsigned char>(*from));
    char32_t code_point = 0;
    if (lead_int <= char_length::to_int(1)) {
      code_point = lead;
      from++;
    }
    else if (lead_int <= char_length::to_int(2) && from + 1 < from_end) {
      const unsigned char c1 = static_cast<unsigned char>(from[1]);
      if ((c1 & 0xC0) != 0x80) {
        from++;
        continue;
      }
      code_point = ((lead & 0x1F) << 6) | (c1 & 0x3F);
      from += 2;
    }
    else if (lead_int <= char_length::to_int(3) && from + 2 < from_end) {
      const unsigned char c1 = static_cast<unsigned char>(from[1]);
      const unsigned char c2 = static_cast<unsigned char>(from[2]);
      if ((c1 & 0xC0) != 0x80 && (c2 & 0xC0) != 0x80) {
        from++;
        continue;
      }
      code_point = ((lead & 0x0F) << 12) | ((c1 & 0x3F) << 6) | (c2 & 0x3F);
      from += 3;
    }
    else if (lead_int <= char_length::to_int(4) && from + 3 < from_end) {
      const unsigned char c1 = static_cast<unsigned char>(from[1]);
      const unsigned char c2 = static_cast<unsigned char>(from[2]);
      const unsigned char c3 = static_cast<unsigned char>(from[3]);
      if ((c1 & 0xC0) != 0x80 && (c2 & 0xC0) != 0x80 && (c3 & 0xC0) != 0x80) {
        from++;
        continue;
      }
      code_point = ((lead & 0x07) << 18) | ((c1 & 0x3F) << 12) | ((c2 & 0x3F) << 6) | (c3 & 0x3F);
      from += 4;
    }
    else {
      from++;
      continue;
    }
    u32.push_back(code_point);
  }
  return u32;
}
static int string_find_token(const StringRef text,
                             const StringRef token,
                             const int start,
                             const int next)
{
  std::u32string a_u32 = u32_from_utf8(text);
  std::u32string b_u32 = u32_from_utf8(token);

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
