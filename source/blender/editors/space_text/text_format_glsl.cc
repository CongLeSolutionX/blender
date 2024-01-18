/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sptext
 */

#include <cstring>

#include "BLI_blenlib.h"

#include "DNA_space_types.h"
#include "DNA_text_types.h"

#include "BKE_text.h"

#include "text_format.hh"

/* -------------------------------------------------------------------- */
/** \name Local Literal Definitions
 * \{ */

static const char *text_format_glsl_literals_builtinfunc_data[] = {
    /* clang-format off */
    "attribute"
    "bool"
    "break"
    "buffer"
    "bvec2"
    "bvec3"
    "bvec4"
    "case"
    "continue"
    "default"
    "discard"
    "do"
    "double"
    "dmat2"
    "dmat2x2"
    "dmat2x3"
    "dmat2x4"
    "dmat3"
    "dmat3x2"
    "dmat3x3"
    "dmat3x4"
    "dmat4"
    "dmat4x2"
    "dmat4x3"
    "dmat4x4"
    "else"
    "flat"
    "float"
    "for"
    "if"
    "gvec4"
    "highp"
    "in"
    "int"
    "inout"
    "ivec2"
    "ivec3"
    "ivec4"
    "layout"
    "mat2"
    "mat2x2"
    "mat2x3"
    "mat2x4"
    "mat3"
    "mat3x2"
    "mat3x3"
    "mat3x4"
    "mat4"
    "mat4x2"
    "mat4x3"
    "mat4x4"
    "out"
    "precision"
    "return"
    "sampler"
    "sampler1D"
    "sampler1DArray"
    "sampler1DArrayShadow"
    "sampler1DShadow"
    "sampler2D"
    "sampler2DArrayShadow"
    "sampler2DArray"
    "sampler2DMSArray"
    "sampler2DMS"
    "sampler2DRect"
    "sampler2DShadow"
    "sampler3D"
    "samplerBuffer"
    "samplerCube" 
    "samplerCubeArray"
    "samplerCubeArrayShadow"
    "samplerCubeShadow"
    "sizeof"
    "smooth"
    "struct"
    "switch"
    "texel" 
    "uniform"
    "using"
    "uvec2"
    "uvec3"
    "uvec4" 
    "varying"
    "vec2"
    "vec3"
    "vec4"
    "void"
    "volatile"
    "while"
    /* clang-format on */
};

static const Span<const char *> text_format_glsl_literals_builtinfunc(
    text_format_glsl_literals_builtinfunc_data, ARRAY_SIZE(text_format_glsl_literals_builtinfunc_data));

static const char *text_format_glsl_literals_reserved_data[] = {
    /* Force single column, sorted list. */
    /* clang-format off */
    "abs"
    "acos"
    "acosh"
    "all"
    "any"
    "asin"
    "asinh"
    "atan"
    "atanh"
    "atomicAdd"
    "atomicAnd"
    "atomicCompSwap"
    "atomicCounter"
    "atomicCounterDecrement"
    "atomicCounterIncrement"
    "atomicExchange"
    "atomicMax"
    "atomicMin"
    "atomicOr"
    "atomicXor"
    "barrier"
    "bitCount"
    "bitfieldExtract"
    "bitfieldInsert"
    "bitfieldReverse"
    "ceil"
    "clamp"
    "cos"
    "cosh"
    "cross"
    "degrees"
    "determinant"
    "dFdx"
    "dFdy"
    "distance" 
    "dot"
    "EmitStreamVertex"
    "EmitVertex"
    "EndPrimitive"
    "EndStreamPrimitive"
    "equal"
    "exp"
    "exp2"
    "faceforward"
    "false"
    "findLSB"  
    "findMSB"
    "floatBitsToInt"
    "floor" 
    "fma"
    "fract"
    "frexp"
    "ftransform"   
    "fwidth"
    "greaterThan"
    "greaterThanEqual"
    "groupMemoryBarrier"
    "imageAtomicAdd"
    "imageAtomicAnd"
    "imageAtomicCompSwap"
    "imageAtomicExchange"
    "imageAtomicMax"
    "imageAtomicMin"
    "imageAtomicOr"
    "imageAtomicXor"
    "imageLoad"
    "imageSamples"
    "imageSize"
    "imageStore"
    "interpolateAtCentriod"
    "interpolateAtOffset"
    "interpolateAtSample"
    "inverse"
    "inversesqrt"
    "intBitsToFloat"
    "isnan"
    "isinf"
    "ldexp"
    "length"
    "lessThan"
    "lessThanEqual"
    "log"
    "log2"
    "matrixCompMult"
    "max"
    "memoryBarrier"
    "memoryBarrierAtomicCounter"
    "memoryBarrierBuffer"
    "memoryBarrierImage"
    "memoryBarrierShared"
    "min"
    "mix"
    "mod"
    "modf"
    "noise"
    "normalize"
    "not"
    "notEqual"
    "outerProduct"
    "packDouble2x32"
    "packHalf2x16"
    "packUnorm"
    "pow"
    "radians"
    "reflect"
    "refract"
    "round"
    "roundEven"
    "sign"    
    "sin"
    "sinh"
    "smoothstep"
    "sqrt"
    "step"
    "tan"
    "tanh"
    "texelFetch"
    "texelFetchOffset"
    "texture"
    "texture1D"
    "texture2D"
    "texture3D"
    "textureGather"
    "textureGatherOffset"
    "textureGatherOffsets"
    "textureGrad"
    "textureGradOffset"
    "textureLod"
    "textureLodOffset"
    "textureOffset"
    "textureProj"
    "textureProjGrad"
    "textureProjGradOffset"
    "textureProjLod"
    "textureProjLodOffset"
    "textureProjOffset"
    "textureQueryLevels"
    "textureQueryLod"
    "textureSize"
    "transpose"
    "true"
    "trunc"
    "uaddCarry"
    "umulExtended"
    "unpackDouble2x32"
    "unpackHalf2x16"
    "unpackUnorm2x16"
    "unpackUnorm4x8"
    "usubBorrow"
    
  /* Add in futur verion when GLSL 4.5 support in Blender */
  /* "textureSamples" */

  /* Built-In Variable */

    "gl_ClipDistance"
    "gl_FragCoord"
    "gl_FragDepth"
    "gl_FrontFacing"
    "gl_GlobalInvocationID"
    "gl_InstanceID"
    "gl_InvocationID"
    "gl_Layer"
    "gl_LocalInvocationID"
    "gl_LocalInvocationIndex"
    "gl_NumSamples"
    "gl_NumWorkGroups"
    "gl_PatchVerticesIn"
    "gl_PointCoord"
    "gl_PointSize"    
    "gl_Position"
    "gl_PrimitiveID"
    "gl_PrimitiveIDIn"
    "gl_SampleID"    
    "gl_SampleMask"
    "gl_SampleMaskIn"
    "gl_SamplePosition"
    "gl_TessCoord"
    "gl_TessLevelInner"
    "gl_TessLevelOuter"
    "gl_VertexID"
    "gl_ViewportIndex"
    "gl_WorkGroupID"
    "gl_WorkGroupSize"
    /* clang-format on */
};
static const Span<const char * > text_format_glsl_literals_reserved(
    text_format_glsl_literals_reserved_data, ARRAY_SIZE(text_format_glsl_literals_reserved_data));

/* GLSL shader types */
static const char *text_format_glsl_literals_specialvar_data[] = {

    /* Force single column , sorted list */
    /* clang-format off */
    "displacement",
    "shader",
    "surface",
    "volume",
    /* clang-format on */
};
static const Span<const char*> text_format_glsl_literals_specialvar(
    text_format_glsl_literals_specialvar_data, ARRAY_SIZE(*text_format_glsl_literals_specialvar_data));

static const char *text_format_glsl_literals_preprocessor[] = {

    "define"
    "defined"
    "elif"
    "else"
    "endif"
    "if"
    "ifdef"
    "ifndef"
    "pragma"
    "version"
};
static const Span<const char *>text_format_glsl_literals_preprocessor(
    text_format_glsl_literals_preprocessor, ARRAY_SIZE(text_format_glsl_literals_preprocessor));

/** \} */

/* -------------------------------------------------------------------- */
/** \name Local Functions (for #TextFormatType::format_line)
 * \{ */

static int txtfmt_glsl_find_builtinfunc(const char *string)
{
    const int i = text_format_string_literal_find(text_format_glsl_literals_builtinfunc, string);

    if (i == 0 || text_check_identifier(string[i])){
        return -1;
    }
    return i;
}

static int txtfmt_glsl_find_reserved(const char *string) 
{
    const int i = text_format_string_literal_find(text_format_glsl_literals_reserved , string);

    if (i == 0 || text_check_identifier(string[i])){
        return -1;
    }
    return i;
}
static int txtfmt_glsl_find_specialvar(const char *string)
{
    const int i = text_format_string_literal_find(text_format_glsl_literals_specialvar, string);

    if (i == 0 || text_check_identifier(string[i])){
        return -1;
    }
    return i;
}
static int txtfmt_glsl_find_preprocessor(const char *string)
{
    const int i = text_format_string_literal_find(text_format_glsl_literals_preprocessor);

    if (string[0] == '#'){
        int i = 1;
        while (text_check_whitespace(string[i])){
            i++;
        }
        while (text_check_identifier(string[i])){
            i++;
        }
        return i;
    }
    return -1;
}
static char txtfmt_glsl_format_identifier(const char *str)
{
    char fmt;

    /* clang-format off */

   if        (txtfmt_glsl_find_specialvar(str)    != -1) {fmt = FMT_TYPE_SPECIAL;
   } else if (txtfmt_glsl_find_builtinfunc(str)   != -1) {fmt = FMT_TYPE_KEYWORD;
   } else if (txtfmt_glsl_find_reserved(str)      != -1) {fmt = FMT_TYPE_RESERVED;
   } else if (txtfmt_glsl_find_preprocessor(str)  != -1) {fmt = FMT_TYPE_DIRECTIVE;
   } else                                                {fmt = FMT_TYPE_DEFAULT;
   }
    /* clang-format on */

    return fmt;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Format Line Implementation (#TextFormatType::format_line)
 * \{ */
static void txtfmt_glsl_format_line(SpaceText *st, TextLine * line, const bool do_next)
{
  FlattenString fs;
  const char *str;
  char *fmt;
  char cont_orig, cont, find, prev = ' ';
  int len, i;

  if (line->prev && line->prev->format != nullptr) {
    fmt = line->prev->format;
    cont = fmt[strlen(fmt) + 1]; /* Just after the null-terminator. */
    BLI_assert((FMT_CONT_ALL & cont) == cont);
  }
  else {
    cont = FMT_CONT_NOP;
  }

  /* Get original continuation from this line. */
  if (line->format != nullptr) {
    fmt = line->format;
    cont_orig = fmt[strlen(fmt) + 1]; /* Just after the null-terminator. */
    BLI_assert((FMT_CONT_ALL & cont_orig) == cont_orig);
  }
  else {
    cont_orig = 0xFF;
  }

  len = flatten_string(st, &fs, line->line);
  str = fs.buf;
  if (!text_check_format_len(line, len)) {
    flatten_string_free(&fs);
    return;
  }
  fmt = line->format;

  while (*str) {
    /* Handle escape sequences by skipping both \ and next char. */
    if (*str == '\\') {
      *fmt = prev;
      fmt++;
      str++;
      if (*str == '\0') {
        break;
      }
      *fmt = prev;
      fmt++;
      str += BLI_str_utf8_size_safe(str);
      continue;
    }
    /* Handle continuations */
    if (cont) {
      /* C-Style comments */
      if (cont & FMT_CONT_COMMENT_C) {
        if (*str == '*' && *(str + 1) == '/') {
          *fmt = FMT_TYPE_COMMENT;
          fmt++;
          str++;
          *fmt = FMT_TYPE_COMMENT;
          cont = FMT_CONT_NOP;
        }
        else {
          *fmt = FMT_TYPE_COMMENT;
        }
        /* Handle other comments */
      }
      else {
        find = (cont & FMT_CONT_QUOTEDOUBLE) ? '"' : '\'';
        if (*str == find) {
          cont = 0;
        }
        *fmt = FMT_TYPE_STRING;
      }

      str += BLI_str_utf8_size_safe(str) - 1;
    }
    /* Not in a string... */
    else {
      /* Deal with comments first */
      if (*str == '/' && *(str + 1) == '/') {
        /* fill the remaining line */
        text_format_fill(&str, &fmt, FMT_TYPE_COMMENT, len - int(fmt - line->format));
      }
      /* C-Style (multi-line) comments */
      else if (*str == '/' && *(str + 1) == '*') {
        cont = FMT_CONT_COMMENT_C;
        *fmt = FMT_TYPE_COMMENT;
        fmt++;
        str++;
        *fmt = FMT_TYPE_COMMENT;
      }
      else if (ELEM(*str, '"', '\'')) {
        /* Strings */
        find = *str;
        cont = (*str == '"') ? FMT_CONT_QUOTEDOUBLE : FMT_CONT_QUOTESINGLE;
        *fmt = FMT_TYPE_STRING;
      }
      /* White-space (all white-space has been converted to spaces). */
      else if (*str == ' ') {
        *fmt = FMT_TYPE_WHITESPACE;
      }
      /* Numbers (digits not part of an identifier and periods followed by digits). */
      else if ((prev != FMT_TYPE_DEFAULT && text_check_digit(*str)) ||
               (*str == '.' && text_check_digit(*(str + 1))))
      {
        *fmt = FMT_TYPE_NUMERAL;
      }
      /* Punctuation */
      else if ((*str != '#') && text_check_delim(*str)) {
        *fmt = FMT_TYPE_SYMBOL;
      }
      /* Identifiers and other text (no previous white-space or delimiters. so text continues). */
      else if (prev == FMT_TYPE_DEFAULT) {
        str += BLI_str_utf8_size_safe(str) - 1;
        *fmt = FMT_TYPE_DEFAULT;
      }
      /* Not white-space, a digit, punctuation, or continuing text.
       * Must be new, check for special words. */
      else {
        /* Keep aligned arguments for readability. */
        /* clang-format off */

        /* Special vars(v) or built-in keywords(b) */
        /* keep in sync with `txtfmt_glsl_format_identifier()`. */
        if        ((i = txtfmt_glsl_find_specialvar(str))   != -1) { prev = FMT_TYPE_SPECIAL;
        } else if ((i = txtfmt_glsl_find_builtinfunc(str))  != -1) { prev = FMT_TYPE_KEYWORD;
        } else if ((i = txtfmt_glsl_find_reserved(str))     != -1) { prev = FMT_TYPE_RESERVED;
        } else if ((i = txtfmt_glsl_find_preprocessor(str)) != -1) { prev = FMT_TYPE_DIRECTIVE;
        }
        /* clang-format on */

        if (i > 0) {
          if (prev == FMT_TYPE_DIRECTIVE) { /* can contain utf8 */
            text_format_fill(&str, &fmt, prev, i);
          }
          else {
            text_format_fill_ascii(&str, &fmt, prev, i);
          }
        }
        else {
          str += BLI_str_utf8_size_safe(str) - 1;
          *fmt = FMT_TYPE_DEFAULT;
        }
      }
    }
    prev = *fmt;
    fmt++;
    str++;
  }

  /* Terminate and add continuation char. */
  *fmt = '\0';
  fmt++;
  *fmt = cont;

  /* If continuation has changed and we're allowed, process the next line. */
  if (cont != cont_orig && do_next && line->next) {
    txtfmt_glsl_format_line(st, line->next, do_next);
  }

  flatten_string_free(&fs);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Registration
 * \{ */

void ED_text_format_register_glsl()
{
  static TextFormatType tft = {nullptr};
  static const char *ext[] = {"glsl", nullptr};

  tft.format_identifier = txtfmt_glsl_format_identifier;
  tft.format_line = txtfmt_glsl_format_line;
  tft.ext = ext;
  tft.comment_line = "/**/";

  ED_text_format_register(&tft);

  BLI_assert(text_format_string_literals_check_sorted_array(text_format_glsl_literals_builtinfunc));
  BLI_assert(text_format_string_literals_check_sorted_array(text_format_glsl_literals_reserved));
  BLI_assert(text_format_string_literals_check_sorted_array(text_format_glsl_literals_specialvar));
  BLI_assert(text_format_string_literals_check_sorted_array(text_format_glsl_literals_preprocessor));
}

/** \} */
