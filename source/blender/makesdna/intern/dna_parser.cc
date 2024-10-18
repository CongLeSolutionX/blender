/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "dna_parser.hh"
#include "dna_lexer.hh"

#include <fmt/format.h>

#include <fstream>
#include <iostream>
#include <sstream>

namespace blender::dna::parser {

static void to_string(std::stringstream &ss,
                      const ast::VariableDeclaration &var_decl,
                      size_t /*padding*/)
{
  ss << fmt::format("{}{} ", var_decl.const_tag ? "const " : "", var_decl.type);
  bool first = true;
  for (auto &item : var_decl.items) {
    if (!first) {
      ss << ',';
    }
    first = false;
    ss << fmt::format("{}{}", item.ptr.value_or(""), item.name);
    for (auto &size : item.array_size) {
      std::visit([&ss](auto &&size) { ss << fmt::format("[{}]", size); }, size);
    }
  }
}

static void to_string(std::stringstream &ss,
                      const ast::FunctionPtrDeclaration &fn_ptr_decl,
                      size_t /*padding*/)
{
  const std::string const_tag = fn_ptr_decl.const_tag ? "const " : "";
  ss << fmt::format("{}{} (*{})(...)", const_tag, fn_ptr_decl.type, fn_ptr_decl.name);
}

static void to_string(std::stringstream &ss,
                      const ast::ArrayPtrDeclaration &array_ptr_decl,
                      size_t /*padding*/)
{
  ss << fmt::format("{} (*{})[{}]", array_ptr_decl.type, array_ptr_decl.name, array_ptr_decl.size);
}

static void to_string(std::stringstream &ss,
                      const ast::IntDeclaration &int_decl,
                      size_t /*padding*/)
{
  ss << fmt::format("#define {} {}", int_decl.name, int_decl.value);
}

static void to_string(std::stringstream &ss,
                      const ast::EnumDeclaration &enum_decl,
                      size_t /*padding*/)
{
  ss << fmt::format("enum {}", enum_decl.name.value_or("unnamed"));
  if (enum_decl.type) {
    ss << fmt::format(": {}", enum_decl.type.value());
  }
  ss << " {...}";
}

static void to_string(std::stringstream &ss,
                      const ast::StructDeclaration &struct_decl,
                      size_t padding)
{
  const auto add_padding = [&ss](size_t padding) {
    ss << fmt::format("{: >{}}", "", padding * 4);
  };
  ss << fmt::format("struct {} {{\n", struct_decl.name);
  for (auto &item : struct_decl.items) {
    add_padding(padding + 1);
    std::visit([&ss, padding](auto &&item) { to_string(ss, item, padding + 1); }, item);
    ss << ";\n";
  }
  add_padding(padding);
  ss << '}';
}

std::string to_string(const CppFile &cpp_file)
{
  std::stringstream ss;
  for (auto &cpp_def : cpp_file.cpp_defs) {
    std::visit([&ss](auto &&cpp_def) { to_string(ss, cpp_def, 0); }, cpp_def);
    ss << ";\n";
  }
  return ss.str();
}

}  // namespace blender::dna::parser

namespace blender::dna::parser::ast {

using namespace lex;

struct ParseFailedResult {};

static constexpr ParseFailedResult parse_failed{};

template<typename T> class ParseResult {
 public:
  using value_type = T;

 private:
  std::optional<T> data_;

 public:
  ParseResult() = delete;
  ParseResult(ParseFailedResult) : data_{std::nullopt} {};
  ParseResult(T &&value) : data_{std::move(value)} {};

  bool success() const
  {
    return data_.has_value();
  }

  bool fail() const
  {
    return !data_.has_value();
  }

  T &value() &
  {
    BLI_assert(success());
    return data_.value();
  }

  const T &value() const &
  {
    BLI_assert(success());
    return data_.value();
  }

  T &&value() &&
  {
    BLI_assert(success());
    return std::move(data_.value());
  }
};

template<typename T> struct Parser {
  static ParseResult<T> parse()
  {
    BLI_assert_unreachable();
    return parse_failed;
  }
};

/* Helper function that manages parsing state. */
template<typename T> ParseResult<T> parse_t(TokenIterator &token_iterator)
{
  token_iterator.push_waypoint();
  ParseResult<T> parse_Result = Parser<T>::parse(token_iterator);
  token_iterator.end_waypoint(parse_Result.success());
  return parse_Result;
};

/**
 * Parser that matches a sequence of elements to parse, fails if any `Args` in `Args...` fails to
 * parse.
 * The sequence: `Sequence<Symbol<'#'>,  Keyword<pragma_str>, Keyword<once_str>>` parses
 * when the text contains `#pragma once`.
 */
template<class... Args> using Sequence = std::tuple<Args...>;
template<class... Args> struct Parser<Sequence<Args...>> {

 private:
  template<std::size_t I>
  static inline bool parse_idx(Sequence<Args...> &sequence, TokenIterator &token_iterator)
  {
    using T = std::tuple_element_t<I, std::tuple<Args...>>;
    ParseResult<T> parse_Result = Parser<T>::parse(token_iterator);
    if (parse_Result.success()) {
      std::get<I>(sequence) = std::move(parse_Result.value());
    }
    return parse_Result.success();
  };

  template<std::size_t... I>
  static inline bool parse_impl(Sequence<Args...> &sequence,
                                std::index_sequence<I...> /*indices*/,
                                TokenIterator &token_iterator)
  {
    return (parse_idx<I>(sequence, token_iterator) && ...);
  };

 public:
  static ParseResult<Sequence<Args...>> parse(TokenIterator &token_iterator)
  {
    Sequence<Args...> sequence{};
    if (parse_impl(sequence, std::index_sequence_for<Args...>{}, token_iterator)) {
      return sequence;
    }
    return parse_failed;
  }
};

/**
 * Parser that don't fails if `T` can't be parsed.
 * The sequence `Sequence<Optional<Keyword<const_str>>, Keyword<int_str>, Identifier, Symbol<';'>>`
 * success either if text is `const int num;` or `int num;`
 */
template<typename T> using Optional = std::optional<T>;
template<typename T> struct Parser<Optional<T>> {
 public:
  static ParseResult<Optional<T>> parse(TokenIterator &token_iterator)
  {
    ParseResult<T> result = parse_t<T>(token_iterator);
    if (result.success()) {
      return Optional<T>{std::move(result.value())};
    }
    return Optional<T>{std::nullopt};
  }
};

/**
 * Parser that tries to match any `Arg` in `Args...`
 * The sequence `Sequence<Variant<Keyword<int_str>,  Keyword<float_str>>, Identifier, Symbol<';'>>`
 * success either if text is `int num;` or `float num;`
 */
template<class... Args> using Variant = std::variant<Args...>;
template<class... Args> struct Parser<Variant<Args...>> {

 private:
  template<typename T>
  static bool parse_variant(Variant<Args...> &variant, TokenIterator &token_iterator)
  {
    ParseResult<T> val = parse_t<T>(token_iterator);
    if (val.success()) {
      variant.template emplace<T>(std::move(val.value()));
    }
    return val.success();
  };

 public:
  static ParseResult<Variant<Args...>> parse(TokenIterator &token_iterator)
  {
    Variant<Args...> variant{};
    if ((parse_variant<Args>(variant, token_iterator) || ...)) {
      return variant;
    }
    return parse_failed;
  }
};

/** Keyword parser. */
template<const char *word> struct Keyword {
  static_assert(Span(keywords, ARRAY_SIZE(keywords)).contains(StringRef(word)),
                "Template string parameter is not a Keyword Type");
  StringRef str;
};

template<const char *Type> struct Parser<Keyword<Type>> {
  static ParseResult<Keyword<Type>> parse(TokenIterator &token_iterator)
  {
    if (KeywordToken *keyword = token_iterator.next<KeywordToken>();
        keyword && keyword->where == Type)
    {
      return Keyword<Type>{keyword->where};
    }
    return parse_failed;
  }
};

/** Symbol parser. */
template<char type> struct Symbol {
  static_assert(symbols.find(type) != symbols.not_found,
                "Template char parameter is not a Symbol Type");
  StringRef str;
};
template<char Type> struct Parser<Symbol<Type>> {
  static ParseResult<Symbol<Type>> parse(TokenIterator &token_iterator)
  {
    if (SymbolToken *symbol = token_iterator.next<SymbolToken>();
        symbol && symbol->where[0] == Type)
    {
      return Symbol<Type>{symbol->where};
    }
    return parse_failed;
  }
};

static void skip_until_match_paired_symbols(char left, char right, TokenIterator &token_iterator);

/**
 * Parses a macro call, `MacroCall<Keyword<DNA_DEFINE_CXX_METHODS_str>>` parses
 * `DNA_DEFINE_CXX_METHODS(...)`.
 */
template<const char *Type> struct MacroCall {};
template<const char *Type> struct Parser<MacroCall<Type>> {
  static ParseResult<MacroCall<Type>> parse(TokenIterator &token_iterator)
  {
    if (parse_t<Sequence<Keyword<Type>, Symbol<'('>>>(token_iterator).success()) {
      skip_until_match_paired_symbols('(', ')', token_iterator);
      parse_t<Symbol<';'>>(token_iterator);
      return MacroCall<Type>{};
    }
    return parse_failed;
  }
};

/** Parses a string literal. */
struct StringLiteral {
  StringRef value;
};
template<> struct Parser<StringLiteral> {
  static ParseResult<StringLiteral> parse(TokenIterator &token_iterator)
  {
    if (StringLiteralToken *literal = token_iterator.next<StringLiteralToken>(); literal) {
      return StringLiteral{literal->where};
    }
    return parse_failed;
  }
};

/** Parses a int literal. */
struct IntLiteral {
  int64_t value;
};
template<> struct Parser<IntLiteral> {
  static ParseResult<IntLiteral> parse(TokenIterator &token_iterator)
  {
    if (IntLiteralToken *value = token_iterator.next<IntLiteralToken>(); value) {
      return IntLiteral{value->value};
    }
    return parse_failed;
  }
};

/** Parses a identifier. */
struct Identifier {
  StringRef str;
};
template<> struct Parser<Identifier> {
  static ParseResult<Identifier> parse(TokenIterator &token_iterator)
  {
    if (IdentifierToken *identifier = token_iterator.next<IdentifierToken>(); identifier) {
      return Identifier{identifier->where};
    }
    return parse_failed;
  }
};

/** Parses a include, either `#include "include_name.hh"` or `#include <path/to/include.hh>`. */
struct Include {};
template<> struct Parser<Include> {
  static ParseResult<Include> parse(TokenIterator &token_iterator)
  {
    if (parse_t<Sequence<Symbol<'#'>, Keyword<include_str>>>(token_iterator).success()) {
      TokenVariant *token = token_iterator.next_variant();
      while (token && !std::holds_alternative<BreakLineToken>(*token)) {
        token = token_iterator.next_variant();
      }
      return Include{};
    }
    return parse_failed;
  }
};

/** Check if a token is a symbol and has a type. */
static bool inline is_symbol_type(const TokenVariant &token, const char type)
{
  return std::holds_alternative<SymbolToken>(token) &&
         std::get<SymbolToken>(token).where[0] == type;
}

/** Parses `#define` directives except to const int defines. */
struct Define {};
template<> struct Parser<Define> {
  static ParseResult<Define> parse(TokenIterator &token_iterator)
  {
    if (!parse_t<Sequence<Symbol<'#'>, Keyword<define_str>>>(token_iterator).success()) {
      return parse_failed;
    }
    bool scape_bl = false;
    for (TokenVariant *token = token_iterator.next_variant(); token;
         token = token_iterator.next_variant())
    {
      if (std::holds_alternative<BreakLineToken>(*token) && !scape_bl) {
        break;
      }
      scape_bl = is_symbol_type(*token, '\\');
    }
    return Define{};
  }
};

/**
 * Parses constant int named values, currently only defines like `#define FILE_MAX 1024`.
 * Eventually could also parse `constexpr` values.
 */
template<> struct Parser<IntDeclaration> {
  static ParseResult<IntDeclaration> parse(TokenIterator &token_iterator)
  {
    using DefineConstIntSeq = Sequence<Symbol<'#'>, Keyword<define_str>, Identifier, IntLiteral>;
    ParseResult<DefineConstIntSeq> def_int_seq = parse_t<DefineConstIntSeq>(token_iterator);
    if (!def_int_seq.success() || !token_iterator.next<BreakLineToken>()) {
      return parse_failed;
    }
    return IntDeclaration{std::get<2>(def_int_seq.value()).str,
                          std::get<3>(def_int_seq.value()).value};
  }
};

/** Parses most c++ primitive types. */
struct PrimitiveType {
  StringRef str;
};
template<> struct Parser<PrimitiveType> {
  static ParseResult<PrimitiveType> parse(TokenIterator &token_iterator)
  {
    /* TODO: Add all primitive types. */
    const bool is_unsigned = parse_t<Keyword<unsigned_str>>(token_iterator).success();
    using PrimitiveTypeVariants = Variant<Keyword<int_str>,
                                          Keyword<char_str>,
                                          Keyword<short_str>,
                                          Keyword<float_str>,
                                          Keyword<double_str>,
                                          Keyword<void_str>,
                                          Keyword<int8_t_str>,
                                          Keyword<int16_t_str>,
                                          Keyword<int32_t_str>,
                                          Keyword<int64_t_str>,
                                          Keyword<uint8_t_str>,
                                          Keyword<uint16_t_str>,
                                          Keyword<uint32_t_str>,
                                          Keyword<uint64_t_str>,
                                          Keyword<long_str>,
                                          Keyword<ulong_str>>;
    ParseResult<PrimitiveTypeVariants> type = parse_t<PrimitiveTypeVariants>(token_iterator);
    /* Only int, char or short could be unsigned. */
    if (type.fail() || (is_unsigned && type.value().index() > 2)) {
      return parse_failed;
    }
    return PrimitiveType{std::visit([](auto &&type) { return type.str; }, type.value())};
  }
};

/**
 * Parses the type in variable declarations or function return value, either a primitive type or
 * custom type.
 */
struct Type {
  bool const_tag = false;
  StringRef str;
};
template<> struct Parser<Type> {
  static ParseResult<Type> parse(TokenIterator &token_iterator)
  {
    using TypeVariant =
        Variant<PrimitiveType, Sequence<Optional<Keyword<struct_str>>, Identifier>>;
    using TypeSequence = Sequence<Optional<Keyword<const_str>>, TypeVariant>;

    ParseResult<TypeSequence> type_seq = parse_t<TypeSequence>(token_iterator);
    if (!type_seq.success()) {
      return parse_failed;
    }
    const bool const_tag = std::get<0>(type_seq.value()).has_value();
    TypeVariant &type_variant = std::get<1>(type_seq.value());
    if (std::holds_alternative<PrimitiveType>(type_variant)) {
      return Type{const_tag, std::get<0>(type_variant).str};
    }
    return Type{const_tag, std::move(std::get<1>(std::get<1>(type_variant)).str)};
  }
};

/**
 * Parses variable array size declarations: in `int num[3][4][FILE_MAX];`
 * parses `[3][4][FILE_MAX]`.
 * */
static Vector<std::variant<StringRef, int64_t>> variable_array_size_parse(
    TokenIterator &token_iterator)
{
  Vector<std::variant<StringRef, int64_t>> result;
  /* Dynamic array. */
  if (parse_t<Sequence<Symbol<'['>, Symbol<']'>>>(token_iterator).success()) {
    result.append("");
  }
  while (true) {
    using ArraySize = Sequence<Symbol<'['>, Variant<IntLiteral, Identifier>, Symbol<']'>>;
    ParseResult<ArraySize> size_seq = parse_t<ArraySize>(token_iterator);
    if (!size_seq.success()) {
      break;
    }
    auto &item_size = std::get<1>(size_seq.value());
    if (std::holds_alternative<IntLiteral>(item_size)) {
      result.append(std::get<IntLiteral>(item_size).value);
    }
    else {
      result.append(std::move(std::get<Identifier>(item_size).str));
    }
  }
  return result;
}

/**
 * Variable parser, parses multiple inline declarations, like:
 * `int value;`
 * `const int value[256][DEFINE_VALUE];`
 * `float *value1,value2[256][256];`
 */
template<> struct Parser<VariableDeclaration> {
  static ParseResult<VariableDeclaration> parse(TokenIterator &token_iterator)
  {
    ParseResult<Type> type = parse_t<Type>(token_iterator);
    if (!type.success()) {
      return parse_failed;
    }
    VariableDeclaration var_decl{};
    var_decl.const_tag = type.value().const_tag;
    var_decl.type = type.value().str;

    while (true) {
      std::string ptr;
      for (; parse_t<Symbol<'*'>>(token_iterator).success();) {
        ptr += '*';
      }
      ParseResult<Identifier> name = parse_t<Identifier>(token_iterator);
      if (!name.success()) {
        return parse_failed;
      }
      VariableDeclaration::Item item{};
      item.ptr = !ptr.empty() ? std::optional{ptr} : std::nullopt;
      item.name = name.value().str;
      item.array_size = variable_array_size_parse(token_iterator);
      var_decl.items.append(std::move(item));
      parse_t<Keyword<DNA_DEPRECATED_str>>(token_iterator);
      if (parse_t<Symbol<';'>>(token_iterator).success()) {
        break;
      }
      if (!parse_t<Symbol<','>>(token_iterator).success()) {
        return parse_failed;
      }
    }
    return var_decl;
  }
};

/* Skips tokens until match the closing right symbol, like function body braces `{...}`. */
static void skip_until_match_paired_symbols(char left, char right, TokenIterator &token_iterator)
{
  int left_count = 1;
  for (TokenVariant *token = token_iterator.next_variant(); token;
       token = token_iterator.next_variant())
  {
    if (is_symbol_type(*token, right)) {
      left_count--;
      if (left_count == 0) {
        break;
      }
    }
    else if (is_symbol_type(*token, left)) {
      left_count++;
    }
  }
};

/**
 * Parses function pointer variables, like `bool (*poll)(struct bContext *);`
 */
template<> struct Parser<FunctionPtrDeclaration> {
  static ParseResult<FunctionPtrDeclaration> parse(TokenIterator &token_iterator)
  {
    using FunctionPtrBegin = Sequence<Type,
                                      Optional<Symbol<'*'>>,
                                      Symbol<'('>,
                                      Symbol<'*'>,
                                      Identifier,
                                      Symbol<')'>,
                                      Symbol<'('>>;
    const ParseResult<FunctionPtrBegin> fn_ptr_seq = parse_t<FunctionPtrBegin>(token_iterator);
    if (!fn_ptr_seq.success()) {
      return parse_failed;
    }
    FunctionPtrDeclaration fn_ptr_decl{};
    fn_ptr_decl.const_tag = std::get<0>(fn_ptr_seq.value()).const_tag;
    fn_ptr_decl.type = std::get<0>(fn_ptr_seq.value()).str;
    fn_ptr_decl.name = std::get<4>(fn_ptr_seq.value()).str;
    /* Skip Function params. */
    skip_until_match_paired_symbols('(', ')', token_iterator);

    /* Closing sequence. */
    if (!parse_t<Symbol<';'>>(token_iterator).success()) {
      return parse_failed;
    }
    return fn_ptr_decl;
  }
};

/**
 * Parses array pointer variables, like `float (*vert_coords_prev)[3];`
 */
template<> struct Parser<ArrayPtrDeclaration> {
  static ParseResult<ArrayPtrDeclaration> parse(TokenIterator &token_iterator)
  {
    using PointerToArraySequence = Sequence<Type,
                                            Symbol<'('>,
                                            Symbol<'*'>,
                                            Identifier,
                                            Symbol<')'>,
                                            Symbol<'['>,
                                            IntLiteral,
                                            Symbol<']'>,
                                            Symbol<';'>>;
    ParseResult<PointerToArraySequence> array_ptr_seq = parse_t<PointerToArraySequence>(
        token_iterator);
    if (!array_ptr_seq.success()) {
      return parse_failed;
    }
    ArrayPtrDeclaration array_ptr_decl{};
    array_ptr_decl.type = std::get<0>(array_ptr_seq.value()).str;
    array_ptr_decl.name = std::get<3>(array_ptr_seq.value()).str;
    array_ptr_decl.size = std::get<6>(array_ptr_seq.value()).value;
    return array_ptr_decl;
  }
};

/**
 * Parses `#if....#endif` code blocks.
 */
struct IfDef {};
template<> struct Parser<IfDef> {
  static ParseResult<IfDef> parse(TokenIterator &token_iterator)
  {
    using IfDefBeginSequence =
        Sequence<Symbol<'#'>, Variant<Keyword<ifdef_str>, Keyword<if_str>, Keyword<ifndef_str>>>;
    const ParseResult<IfDefBeginSequence> ifdef_seq = parse_t<IfDefBeginSequence>(token_iterator);
    if (!ifdef_seq.success()) {
      return parse_failed;
    };
    int ifdef_deep = 1;
    bool hash_carried = false;
    for (TokenVariant *token = token_iterator.next_variant(); token;
         token = token_iterator.next_variant())
    {
      if (std::holds_alternative<KeywordToken>(*token)) {
        KeywordToken &keyword = std::get<KeywordToken>(*token);
        ifdef_deep += (hash_carried && ELEM(keyword.where, if_str, ifdef_str, ifndef_str));
        ifdef_deep -= hash_carried && keyword.where == endif_str;
      }
      if (ifdef_deep == 0) {
        break;
      }
      hash_carried = is_symbol_type(*token, '#');
    }
    /* Not matching #endif. */
    if (ifdef_deep != 0) {
      return parse_failed;
    }
    return IfDef{};
  }
};

/**
 * Parses struct declarations.
 */
template<> struct Parser<StructDeclaration> {
  static ParseResult<StructDeclaration> parse(TokenIterator &token_iterator)
  {
    using StructBeginSequence = Sequence<Optional<Keyword<typedef_str>>,
                                         Keyword<struct_str>,
                                         Optional<Identifier>,
                                         Symbol<'{'>>;
    ParseResult<StructBeginSequence> struct_seq = parse_t<StructBeginSequence>(token_iterator);
    if (!struct_seq.success()) {
      return parse_failed;
    }
    StructDeclaration struct_decl{};
    if (std::get<2>(struct_seq.value()).has_value()) {
      struct_decl.name = std::get<2>(struct_seq.value()).value().str;
    }
    while (true) {
      using DNA_DEF_CCX_Macro = MacroCall<DNA_DEFINE_CXX_METHODS_str>;
      using MemberVariant = Variant<VariableDeclaration,
                                    FunctionPtrDeclaration,
                                    ArrayPtrDeclaration,
                                    StructDeclaration>;
      if (auto member = parse_t<MemberVariant>(token_iterator); member.success()) {
        struct_decl.items.append(std::move(member.value()));
      }
      else if (parse_t<Variant<DNA_DEF_CCX_Macro, IfDef>>(token_iterator).success()) {
      }
      else {
        break;
      }
    }
    using StructEndSequence = Sequence<Symbol<'}'>, Optional<Identifier>, Symbol<';'>>;
    ParseResult<StructEndSequence> struct_end = parse_t<StructEndSequence>(token_iterator);
    if (!struct_end.success()) {
      return parse_failed;
    }
    if (std::get<1>(struct_end.value()).has_value()) {
      struct_decl.member_name = std::get<1>(struct_end.value()).value().str;
    }
    if (struct_decl.member_name == struct_decl.name && struct_decl.name.is_empty()) {
      return parse_failed;
    }
    return struct_decl;
  }
};

/** Parses unused declarations by makesdna. */
struct UnusedDeclaration {};
template<> struct Parser<UnusedDeclaration> {
  static ParseResult<UnusedDeclaration> parse(TokenIterator &token_iterator)
  {
    using UnusedDeclarations = Variant<
        Define,
        Include,
        IfDef,
        Sequence<Symbol<'#'>, Keyword<pragma_str>, Keyword<once_str>>,
        Sequence<Symbol<'#'>, Symbol<'#'>, StructDeclaration>,
        Sequence<Keyword<extern_str>, VariableDeclaration>,
        MacroCall<BLI_STATIC_ASSERT_ALIGN_str>,
        MacroCall<ENUM_OPERATORS_str>,
        Sequence<Keyword<typedef_str>, Keyword<struct_str>, Identifier, Identifier, Symbol<';'>>>;
    if (parse_t<UnusedDeclarations>(token_iterator).success()) {
      return UnusedDeclaration{};
    }
    /* Forward declarations. */
    if (parse_t<Sequence<Keyword<struct_str>, Identifier>>(token_iterator).success()) {
      for (; parse_t<Sequence<Symbol<','>, Identifier>>(token_iterator).success();) {
      }
      if (parse_t<Symbol<';'>>(token_iterator).success()) {
        return UnusedDeclaration{};
      }
    }
    else if (token_iterator.next<BreakLineToken>()) {
      return UnusedDeclaration{};
    }
    return parse_failed;
  }
};

/** Parse enums, with a name or not and with a fixed type or not. */
template<> struct Parser<EnumDeclaration> {
  static ParseResult<EnumDeclaration> parse(TokenIterator &token_iterator)
  {
    using EnumBeginSequence = Sequence<Optional<Keyword<typedef_str>>,
                                       Keyword<enum_str>,
                                       Optional<Keyword<class_str>>,
                                       Optional<Identifier>,
                                       Optional<Sequence<Symbol<':'>, PrimitiveType>>,
                                       Symbol<'{'>>;
    ParseResult<EnumBeginSequence> enum_begin = parse_t<EnumBeginSequence>(token_iterator);
    if (!enum_begin.success()) {
      return parse_failed;
    }
    EnumDeclaration enum_decl;
    if (std::get<3>(enum_begin.value()).has_value()) {
      enum_decl.name = std::get<3>(enum_begin.value()).value().str;
    }
    if (std::get<4>(enum_begin.value()).has_value()) {
      enum_decl.type = std::get<1>(std::get<4>(enum_begin.value()).value()).str;
    }
    /* Skip enum body. */
    skip_until_match_paired_symbols('{', '}', token_iterator);

    using EnumEndSeq =
        Sequence<Optional<Identifier>, Optional<Keyword<DNA_DEPRECATED_str>>, Symbol<';'>>;
    if (!parse_t<EnumEndSeq>(token_iterator).success()) {
      return parse_failed;
    }
    return enum_decl;
  }
};

}  // namespace blender::dna::parser::ast

namespace blender::dna::parser {

static void print_unhandled_token_error(StringRef filepath,
                                        StringRef text,
                                        lex::TokenVariant *what)
{
  const auto visit_fn = [text, filepath](auto &&token) {
    const char *itr = text.begin();
    size_t line = 1;
    while (itr < token.where.begin()) {
      if (itr[0] == '\n') {
        line++;
      }
      itr++;
    }
    fmt::print("{}{} Unhandled token: \"{}\"\n", filepath, line, token.where);
  };
  std::visit(visit_fn, *what);
}

std::string read_file(StringRef filepath)
{
  std::ifstream file(filepath.data());
  if (!file.is_open()) {
    fprintf(stderr, "Can't read file %s\n", filepath.data());
    return "";
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

std::optional<CppFile> parse_file(StringRef filepath)
{
  using namespace ast;
  CppFile cpp_file;
  cpp_file.text = read_file(filepath);
  /* Generate tokens. */
  lex::TokenIterator token_iterator;
  token_iterator.process_text(filepath, cpp_file.text);

  int dna_deprecated_allow_count = 0;
  using DNADeprecatedAllowSeq =
      Sequence<Symbol<'#'>, Keyword<ifdef_str>, Keyword<DNA_DEPRECATED_ALLOW_str>>;
  using EndIfSeq = Sequence<Symbol<'#'>, Keyword<endif_str>>;

  while (!token_iterator.has_finish()) {
    using CPPTypeVariant =
        Variant<Sequence<Optional<Keyword<typedef_str>>, FunctionPtrDeclaration>,
                VariableDeclaration,
                DNADeprecatedAllowSeq,
                EndIfSeq,
                UnusedDeclaration>;

    if (auto struct_decl = parse_t<StructDeclaration>(token_iterator); struct_decl.success()) {
      cpp_file.cpp_defs.append(std::move(struct_decl.value()));
    }
    else if (auto enum_decl = parse_t<EnumDeclaration>(token_iterator); enum_decl.success()) {
      if (!enum_decl.value().name.has_value() || !enum_decl.value().type.has_value()) {
        continue;
      }
      cpp_file.cpp_defs.append(std::move(enum_decl.value()));
    }
    else if (auto int_decl = parse_t<IntDeclaration>(token_iterator); int_decl.success()) {
      cpp_file.cpp_defs.append(int_decl.value());
    }
    else if (auto cpp_variant = parse_t<CPPTypeVariant>(token_iterator); cpp_variant.success()) {
      if (std::holds_alternative<DNADeprecatedAllowSeq>(cpp_variant.value())) {
        dna_deprecated_allow_count++;
      }
      else if (std::holds_alternative<EndIfSeq>(cpp_variant.value())) {
        dna_deprecated_allow_count--;
        BLI_assert(dna_deprecated_allow_count >= 0);
      }
    }
    else {
      print_unhandled_token_error(filepath, cpp_file.text, token_iterator.last_unmatched);
      return std::nullopt;
    }
  }
#ifdef DNA_DEBUG_PRINT
  constexpr StringRef debug_file = "DNA_action_types.h";
  if (!debug_file.is_empty() && filepath.find(debug_file) != filepath.not_found) {
    printf("%s", to_string(cpp_file).c_str());
  }
#endif
  return cpp_file;
}

}  // namespace blender::dna::parser
