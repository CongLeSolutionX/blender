/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "dna_lexer.hh"

#include <cctype>
#include <charconv>
#include <fmt/format.h>

namespace blender::dna::lex {

void TokenIterator::eval_space(const char *&itr, const char *end)
{
  while (itr < end && itr[0] != '\n' && std::isspace(itr[0])) {
    itr++;
  }
}

void TokenIterator::eval_break_line(const char *&itr, const char * /*end*/)
{
  if (itr[0] != '\n') {
    return;
  }
  this->append(BreakLineToken{StringRef(itr, itr + 1)});
  itr++;
}

void TokenIterator::eval_identifier(const char *&itr, const char *end)
{
  if (!(std::isalpha(itr[0]) || itr[0] == '_')) {
    return;
  }
  const char *start = itr++;
  while (itr < end && (std::isalnum(itr[0]) || itr[0] == '_')) {
    itr++;
  }

  StringRef str = StringRef(start, itr);
  if (Span(keywords, ARRAY_SIZE(keywords)).contains(str)) {
    this->append(KeywordToken{str});
    return;
  }
  this->append(IdentifierToken{str});
}

void TokenIterator::eval_line_comment(const char *&itr, const char *end)
{
  if (end - itr < 2) {
    return;
  }
  if (!(itr[0] == '/' && itr[1] == '/')) {
    return;
  }
  while (itr != end && itr[0] != '\n') {
    itr++;
  }
}

void TokenIterator::eval_int_literal(const char *&itr, const char *end)
{
  const char *start = itr;
  while (itr < end && std::isdigit(itr[0])) {
    itr++;
  }
  if (itr == start) {
    return;
  }
  int64_t value = 0;
  std::from_chars(start, itr, value);
  this->append(IntLiteralToken{StringRef(start, itr), value});
}

void TokenIterator::eval_multiline_comment(const char *&itr, const char *end)
{
  if (end - itr < 2) {
    return;
  }
  if (!(itr[0] == '/' && itr[1] == '*')) {
    return;
  }
  char carry = itr[0];
  itr += 2;
  while (itr < end && !(carry == '*' && itr[0] == '/')) {
    carry = itr[0];
    itr++;
  }
  if (itr < end) {
    itr++;
  }
}

void TokenIterator::eval_symbol(const char *&itr, const char * /* end */)
{
  if (symbols.find(itr[0]) != symbols.not_found) {
    this->append(SymbolToken{StringRef(itr, itr + 1)});
    itr++;
  }
}

void TokenIterator::eval_string_literal(const char *&itr, const char *end)
{
  const char opening = itr[0];
  if (!(opening == '"' || opening == '\'')) {
    return;
  }
  const char *start = itr++;
  bool scape = false;
  while (itr < end && !(!scape && itr[0] == opening)) {
    scape = itr[0] == '\\' && !scape;
    itr++;
  }
  if (!(itr < end)) {
    itr = start;
    return;
  }
  itr++;
  this->append(StringLiteralToken{StringRef(start, itr)});
}

void TokenIterator::print_unkown_token(StringRef filepath, StringRef text, const char *where)
{
  size_t line = 1;
  const char *itr = text.begin();
  while (itr < where) {
    if (itr[0] == '\n') {
      line++;
    }
    itr++;
  }
  fmt::print("{}({}) Unknown token: ({})\n", filepath, line, where[0]);
}

void TokenIterator::skip_break_lines()
{
  while (next_ < token_stream_.end() && std::holds_alternative<BreakLineToken>(*next_)) {
    next_++;
  }
}

void TokenIterator::process_text(StringRef filepath, StringRef text)
{
  const char *itr = text.begin();
  const char *end = text.end();
  const auto eval_token_type =
      [this, &itr, end](void (TokenIterator::*fn)(const char *&, const char *)) -> bool {
    const char *current = itr;
    (this->*fn)(itr, end);
    return current != itr;
  };

  while (itr != text.end()) {
    const char *current = itr;

    if (eval_token_type(&TokenIterator::eval_space) ||
        eval_token_type(&TokenIterator::eval_line_comment) ||
        eval_token_type(&TokenIterator::eval_multiline_comment) ||
        eval_token_type(&TokenIterator::eval_identifier) ||
        eval_token_type(&TokenIterator::eval_int_literal) ||
        eval_token_type(&TokenIterator::eval_string_literal) ||
        eval_token_type(&TokenIterator::eval_symbol) ||
        eval_token_type(&TokenIterator::eval_break_line))
    {
      continue;
    }
    /* Unknown token found. */
    if (current == itr) {
      print_unkown_token(filepath, text, itr);
      token_stream_.clear();
      break;
    }
  }
  next_ = token_stream_.begin();
};

TokenVariant *TokenIterator::next_variant()
{
  if (next_ < token_stream_.end()) {
    return next_++;
  }
  return nullptr;
}

bool TokenIterator::has_finish()
{
  return !(next_ < token_stream_.end());
}

void TokenIterator::push_waypoint()
{
  waypoints_.append(next_);
}

void TokenIterator::end_waypoint(bool success)
{
  if (!success) {
    if (last_unmatched < next_) {
      last_unmatched = next_;
    }
    next_ = waypoints_.last();
  }
  waypoints_.remove_last();
}

}  // namespace blender::dna::lex
