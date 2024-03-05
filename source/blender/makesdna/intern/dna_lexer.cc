#include "dna_lexer.hh"
#include <charconv>
#include <iostream>

namespace blender::dna::lex {

static std::string_view range_string_view(const std::string_view::iterator first,
                                          const std::string_view::iterator last)
{
  return std::string_view{first._Unwrapped(), size_t(last - first)};
}

/* Match any withe space except break lines. */
static void eval_space(std::string_view::iterator &itr,
                       std::string_view::iterator last,
                       TokenIterator & /*cont*/)
{
  while (itr < last && itr[0] != '\n' && std::isspace(itr[0])) {
    itr++;
  }
}

/* Match break lines, added as tokens since are needed for `#define` blocks. */
static void eval_break_line(std::string_view::iterator &itr,
                            std::string_view::iterator /*last*/,
                            TokenIterator &cont)
{
  if (itr[0] != '\n') {
    return;
  }
  cont.append(BreakLineToken{range_string_view(itr, itr + 1)});
  itr++;
}

/* Match any identifier substring, also matches c++ keywords. */
static void eval_identifier(std::string_view::iterator &itr,
                            std::string_view::iterator last,
                            TokenIterator &cont)
{
  if (!(std::isalpha(itr[0]) || itr[0] == '_')) {
    return;
  }
  std::string_view::iterator start{itr++};
  while (itr < last && (std::isalnum(itr[0]) || itr[0] == '_')) {
    itr++;
  }
  struct KeywordItem {
    std::string_view word;
    KeywordType type;
  };
  using namespace std::string_view_literals;
  static constexpr KeywordItem keywords[]{
      {"BLI_STATIC_ASSERT_ALIGN"sv, KeywordType::BLI_STATIC_ASSERT_ALIGN},
      {"DNA_DEFINE_CXX_METHODS"sv, KeywordType::DNA_DEFINE_CXX_METHODS},
      {"DNA_DEPRECATED"sv, KeywordType::DNA_DEPRECATED},
      {"ENUM_OPERATORS"sv, KeywordType::ENUM_OPERATORS},
      {"extern"sv, KeywordType::EXTERN},
      {"char"sv, KeywordType::CHAR},
      {"char16_t"sv, KeywordType::CHAR16_T},
      {"char32_t"sv, KeywordType::CHAR32_T},
      {"class"sv, KeywordType::CLASS},
      {"const"sv, KeywordType::CONST},
      {"define"sv, KeywordType::DEFINE},
      {"double"sv, KeywordType::DOUBLE},
      {"endif"sv, KeywordType::ENDIF},
      {"enum"sv, KeywordType::ENUM},
      {"float"sv, KeywordType::FLOAT},
      {"if"sv, KeywordType::IF},
      {"ifdef"sv, KeywordType::IFDEF},
      {"ifndef"sv, KeywordType::IFNDEF},
      {"include"sv, KeywordType::INCLUDE},
      {"int"sv, KeywordType::INT},
      {"int16_t"sv, KeywordType::INT16_T},
      {"int32_t"sv, KeywordType::INT32_T},
      {"int64_t"sv, KeywordType::INT64_T},
      {"int8_t"sv, KeywordType::INT8_T},
      {"long"sv, KeywordType::LONG},
      {"once"sv, KeywordType::ONCE},
      {"pragma"sv, KeywordType::PRAGMA},
      {"private"sv, KeywordType::PRIVATE},
      {"public"sv, KeywordType::PUBLIC},
      {"short"sv, KeywordType::SHORT},
      {"signed"sv, KeywordType::SIGNED},
      {"struct"sv, KeywordType::STRUCT},
      {"typedef"sv, KeywordType::TYPEDEF},
      {"uint16_t"sv, KeywordType::UINT16_T},
      {"uint32_t"sv, KeywordType::UINT32_T},
      {"uint64_t"sv, KeywordType::UINT64_T},
      {"uint8_t"sv, KeywordType::UINT8_T},
      {"unsigned"sv, KeywordType::UNSIGNED},
      {"void"sv, KeywordType::VOID},
  };

  std::string_view str = range_string_view(start, itr);
  auto test_keyword_fn = [str](const KeywordItem &val) -> bool { return val.word == str; };
  const KeywordItem *keyword_itr = std::find_if(
      std::begin(keywords), std::end(keywords), test_keyword_fn);
  if (keyword_itr != std::end(keywords)) {
    cont.append(KeywordToken{str, keyword_itr->type});
    return;
  }
  cont.append(IdentifierToken{str});
}

/* Match a line comment until break line. */
static void eval_line_comment(std::string_view::iterator &itr,
                              std::string_view::iterator last,
                              TokenIterator & /*cont*/)
{
  if (last - itr < 2) {
    return;
  }
  if (!(itr[0] == '/' && itr[1] == '/')) {
    return;
  }
  std::string_view::iterator start{itr++};
  while (itr != last && itr[0] != '\n') {
    itr++;
  }
}

/* Match a int literal. */
static void eval_int_literal(std::string_view::iterator &itr,
                             std::string_view::iterator last,
                             TokenIterator &cont)
{
  const std::string_view::iterator start{itr};
  while (itr < last && std::isdigit(itr[0])) {
    itr++;
  }
  if (itr == start) {
    return;
  }
  int val{};
  auto transform_Result = std::from_chars(start._Unwrapped(), itr._Unwrapped(), val);
  cont.append(IntLiteralToken{range_string_view(start, itr), val});
}

/* Match a c-style comment. */
static void eval_multiline_comment(std::string_view::iterator &itr,
                                   std::string_view::iterator last,
                                   TokenIterator & /*cont*/)
{
  if (last - itr < +2) {
    return;
  }
  if (!(itr[0] == '/' && itr[1] == '*')) {
    return;
  }
  char carry = itr[0];
  itr += 2;
  while (itr < last && !(carry == '*' && itr[0] == '/')) {
    carry = itr[0];
    itr++;
  }
  if (itr < last) {
    itr++;
  }
}

/* Match a symbol. */
static void eval_symbol(std::string_view::iterator &itr,
                        std::string_view::iterator /*last*/,
                        TokenIterator &cont)
{
  struct SymbolItem {
    char value;
    SymbolType type;
  };
  static constexpr SymbolItem symbols[]{
      {'!', SymbolType::EXCLAMATION}, {'#', SymbolType::HASH},       {'%', SymbolType::PERCENT},
      {'&', SymbolType::BIT_AND},     {'(', SymbolType::LPAREN},     {')', SymbolType::RPAREN},
      {'*', SymbolType::STAR},        {'+', SymbolType::PLUS},       {',', SymbolType::COMMA},
      {'-', SymbolType::MINUS},       {'.', SymbolType::DOT},        {'/', SymbolType::SLASH},
      {':', SymbolType::COLON},       {';', SymbolType::SEMICOLON},  {'<', SymbolType::LESS},
      {'=', SymbolType::ASSIGN},      {'>', SymbolType::GREATER},    {'?', SymbolType::QUESTION},
      {'[', SymbolType::LBRACKET},    {'\\', SymbolType::BACKSLASH}, {']', SymbolType::RBRACKET},
      {'^', SymbolType::CARET},       {'{', SymbolType::LBRACE},     {'|', SymbolType::BIT_OR},
      {'}', SymbolType::RBRACE},      {'~', SymbolType::TILDE},
  };
  const char value = itr[0];
  auto test_symbol = [value](const SymbolItem &item) -> bool { return item.value == value; };
  const SymbolItem *symbol_itr = std::find_if(std::begin(symbols), std::end(symbols), test_symbol);
  if (symbol_itr != std::end(symbols)) {
    cont.append(SymbolToken{range_string_view(itr, itr + 1), symbol_itr->type});
    itr++;
  }
}

/* Match a string or char literal. */
static void eval_string_literal(std::string_view::iterator &itr,
                                std::string_view::iterator last,
                                TokenIterator &cont)
{
  const char opening = itr[0];
  if (!(opening == '"' || opening == '\'')) {
    return;
  }
  const std::string_view::iterator start{itr++};
  bool scape{false};
  while (itr < last && !(!scape && itr[0] == opening)) {
    scape = itr[0] == '\\' && !scape;
    itr++;
  }
  if (!(itr < last)) {
    itr = start;
    return;
  }
  itr++;
  cont.append(StringLiteralToken{range_string_view(start, itr)});
}

void TokenIterator::print_unkown_token(std::string_view filepath,
                                       std::string_view::iterator start,
                                       std::string_view::iterator where)
{
  size_t line = 1;
  while (start < where) {
    if (start[0] == '\n') {
      line++;
    }
    start++;
  }
  printf("%.*s(%zd) Unknown token: (%c)\n",
         uint32_t(filepath.size()),
         filepath.data(),
         line,
         where[0]);
}

void TokenIterator::skip_break_lines()
{
  while (next_ < token_stream_.end() && std::holds_alternative<BreakLineToken>(*next_)) {
    next_++;
  }
}

void TokenIterator::process_text(std::string_view filepath, std::string_view text)
{
  std::string_view::iterator itr = text.begin();
  const std::string_view::iterator end = text.end();

  auto eval_token = [this](std::string_view::iterator &itr,
                           std::string_view::iterator end,
                           auto &&eval_fn) -> bool {
    std::string_view::iterator current = itr;
    eval_fn(itr, end, *this);
    return current != itr;
  };

  while (itr != text.end()) {
    const std::string_view::iterator current = itr;

    if (eval_token(itr, end, eval_space) || eval_token(itr, end, eval_line_comment) ||
        eval_token(itr, end, eval_multiline_comment) || eval_token(itr, end, eval_identifier) ||
        eval_token(itr, end, eval_int_literal) || eval_token(itr, end, eval_string_literal) ||
        eval_token(itr, end, eval_symbol) || eval_token(itr, end, eval_break_line))
    {
      continue;
    }
    /* Unkown token found. */
    if (current == itr) {
      print_unkown_token(filepath, text.begin(), itr);
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
    if (last < next_) {
      last = next_;
    }
    next_ = waypoints_.last();
  }
  waypoints_.remove_last();
}

KeywordToken *TokenIterator::next_keyword(KeywordType type)
{
  TokenVariant *tmp = next_;
  if (KeywordToken *keyword = next<KeywordToken>(); keyword && keyword->type == type) {
    return keyword;
  }
  next_ = tmp;
  return nullptr;
}

SymbolToken *TokenIterator::next_symbol(SymbolType type)
{
  TokenVariant *tmp = next_;
  if (SymbolToken *symbol = next<SymbolToken>(); symbol && symbol->type == type) {
    return symbol;
  }
  next_ = tmp;
  return nullptr;
}
}  // namespace blender::dna::lex
