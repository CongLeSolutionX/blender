/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "BLI_string_ref.hh"
#include "BLI_vector.hh"
#include <variant>

namespace blender::dna::lex {

static constexpr StringRef symbols = ":;()[]{}=#.,*<>|&+-!%^?~\\/";

static constexpr char include_str[] = "include";
static constexpr char struct_str[] = "struct";
static constexpr char typedef_str[] = "typedef";
static constexpr char class_str[] = "class";
static constexpr char enum_str[] = "enum";
static constexpr char define_str[] = "define";
static constexpr char public_str[] = "public";
static constexpr char private_str[] = "private";
static constexpr char const_str[] = "const";
static constexpr char void_str[] = "void";
static constexpr char char_str[] = "char";
static constexpr char char16_t_str[] = "char16_t";
static constexpr char char32_t_str[] = "char32_t";
static constexpr char unsigned_str[] = "unsigned";
static constexpr char signed_str[] = "signed";
static constexpr char short_str[] = "short";
static constexpr char long_str[] = "long";
static constexpr char ulong_str[] = "ulong";
static constexpr char int_str[] = "int";
static constexpr char int8_t_str[] = "int8_t";
static constexpr char int16_t_str[] = "int16_t";
static constexpr char int32_t_str[] = "int32_t";
static constexpr char int64_t_str[] = "int64_t";
static constexpr char uint8_t_str[] = "uint8_t";
static constexpr char uint16_t_str[] = "uint16_t";
static constexpr char uint32_t_str[] = "uint32_t";
static constexpr char uint64_t_str[] = "uint64_t";
static constexpr char float_str[] = "float";
static constexpr char double_str[] = "double";
static constexpr char if_str[] = "if";
static constexpr char ifdef_str[] = "ifdef";
static constexpr char ifndef_str[] = "ifndef";
static constexpr char endif_str[] = "endif";
static constexpr char extern_str[] = "extern";
static constexpr char pragma_str[] = "pragma";
static constexpr char once_str[] = "once";

/* Common Blender macros in DNA. */
static constexpr char BLI_STATIC_ASSERT_ALIGN_str[] = "BLI_STATIC_ASSERT_ALIGN";
static constexpr char DNA_DEFINE_CXX_METHODS_str[] = "DNA_DEFINE_CXX_METHODS";
static constexpr char DNA_DEPRECATED_str[] = "DNA_DEPRECATED";
static constexpr char DNA_DEPRECATED_ALLOW_str[] = "DNA_DEPRECATED_ALLOW";
static constexpr char ENUM_OPERATORS_str[] = "ENUM_OPERATORS";

static constexpr StringRef keywords[]{
    include_str,
    struct_str,
    typedef_str,
    class_str,
    enum_str,
    define_str,
    public_str,
    private_str,
    const_str,
    void_str,
    char_str,
    char16_t_str,
    char32_t_str,
    unsigned_str,
    signed_str,
    short_str,
    long_str,
    ulong_str,
    int_str,
    int8_t_str,
    int16_t_str,
    int32_t_str,
    int64_t_str,
    uint8_t_str,
    uint16_t_str,
    uint32_t_str,
    uint64_t_str,
    float_str,
    double_str,
    if_str,
    ifdef_str,
    ifndef_str,
    endif_str,
    extern_str,
    pragma_str,
    once_str,
    BLI_STATIC_ASSERT_ALIGN_str,
    DNA_DEFINE_CXX_METHODS_str,
    DNA_DEPRECATED_str,
    DNA_DEPRECATED_ALLOW_str,
    ENUM_OPERATORS_str,
};

struct Token {
  StringRef where;
};

struct BreakLineToken : public Token {};

struct IdentifierToken : public Token {};

struct StringLiteralToken : public Token {};

struct IntLiteralToken : public Token {
  int64_t value;
};

struct SymbolToken : public Token {};

struct KeywordToken : public Token {};

using TokenVariant = std::variant<BreakLineToken,
                                  IdentifierToken,
                                  IntLiteralToken,
                                  SymbolToken,
                                  KeywordToken,
                                  StringLiteralToken>;

struct TokenIterator {
  /** Last token that fails to match a token request. */
  TokenVariant *last_unmatched = nullptr;

 private:
  /** Token stream. */
  Vector<TokenVariant> token_stream_;
  /** Return points to use for roll back when parser fails to parse tokens. */
  Vector<TokenVariant *> waypoints_;
  /** Pointer to next token to iterate. */
  TokenVariant *next_ = nullptr;

  /** Print the line where an unkown token was found. */
  void print_unkown_token(StringRef filepath, StringRef text, const char *where);

  void skip_break_lines();

 public:
  /** Iterates over the input text looking for tokens. */
  void process_text(StringRef filepath, StringRef text);

  /**
   * Add the current token as waypoint, in case the token parser needs the iterator to roll
   * back.
   */
  void push_waypoint();

  /**
   * Removes the last waypoint, if `success==false` the iterator rolls back to this last
   * waypoint.
   */
  void end_waypoint(bool success);

  /** Return the pointer to the next token, and advances the iterator. */
  TokenVariant *next_variant();

  /** Checks if the token iterator has reach the last token. */
  bool has_finish();

  /**
   * Return the next token if it type matches to `Type`.
   * Break lines are skipped for non break line requested tokens.
   */
  template<class Type> Type *next()
  {
    TokenVariant *current_next = next_;
    if constexpr (!std::is_same_v<Type, BreakLineToken>) {
      skip_break_lines();
    }
    if (next_ < token_stream_.end() && std::holds_alternative<Type>(*next_)) {
      return &std::get<Type>(*next_++);
    }
    if (last_unmatched < next_) {
      last_unmatched = next_;
    }
    next_ = current_next;
    return nullptr;
  }

 private:
  /** Match any whitespace except break lines. */
  void eval_space(const char *&itr, const char *end);
  /** Match break lines. */
  void eval_break_line(const char *&itr, const char *end);
  /** Match identifiers and `C++` keywords. */
  void eval_identifier(const char *&itr, const char *end);
  /** Match single-line comment. */
  void eval_line_comment(const char *&itr, const char *end);
  /** Match a int literal. */
  void eval_int_literal(const char *&itr, const char *end);
  /** Match a multi-line comment. */
  void eval_multiline_comment(const char *&itr, const char *end);
  /** Match a symbol. */
  void eval_symbol(const char *&itr, const char *end);
  /** Match a string or char literal. */
  void eval_string_literal(const char *&itr, const char *end);

  /** Appends a token. */
  template<class TokenType> void append(TokenType &&token)
  {
    token_stream_.append(std::move(token));
  }
};

}  // namespace blender::dna::lex
