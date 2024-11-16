/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup glsl_preprocess
 */

#pragma once

#include <cstdint>
#include <functional>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace blender::gpu::shader {

/**
 * Shader source preprocessor that allow to mutate GLSL into cross API source that can be
 * interpreted by the different GPU backends. Some syntax are mutated or reported as incompatible.
 *
 * Implementation speed is not a huge concern as we only apply this at compile time or on python
 * shaders source.
 */
class Preprocessor {
  using uint64_t = std::uint64_t;
  using report_callback = std::function<void(const std::smatch &, const char *)>;
  struct SharedVar {
    std::string type;
    std::string name;
    std::string array;
  };

  std::vector<SharedVar> shared_vars_;
  std::unordered_set<std::string> static_strings_;
  std::unordered_set<std::string> gpu_builtins_;
  /* Note: Could be a set, but for now the order matters. */
  std::vector<std::string> dependencies_;
  std::stringstream gpu_functions_;
  std::stringstream generated_header_;

 public:
  /* Compile-time hashing function which converts string to a 64bit hash. */
  constexpr static uint64_t hash(const char *name)
  {
    uint64_t hash = 2166136261u;
    while (*name) {
      hash = hash * 16777619u;
      hash = hash ^ *name;
      ++name;
    }
    return hash;
  }

  static uint64_t hash(const std::string &name)
  {
    return hash(name.c_str());
  }

  /* String hash are outputted inside GLSL and needs to fit 32 bits. */
  static uint64_t hash_32(const std::string &str)
  {
    uint64_t hash_64 = hash(str);
    uint32_t hash_32 = uint32_t(hash_64 ^ (hash_64 >> 32));
    return hash_32;
  }

  /* Takes a whole source file and output processed source. */
  std::string process(std::string str,
                      const std::string &filename,
                      bool do_linting,
                      bool do_parse_function,
                      bool do_string_mutation,
                      bool do_include_parsing,
                      bool do_small_type_linting,
                      bool do_resource_argument,
                      report_callback report_error)
  {
    str = remove_comments(str, report_error);
    threadgroup_variables_parsing(str);
    parse_builtins(str);
    if (do_parse_function) {
      parse_library_functions(str);
    }
    if (do_include_parsing) {
      include_parse(str);
    }
    str = preprocessor_directive_mutation(str);
    if (do_string_mutation) {
      static_strings_parsing(str);
      str = static_strings_mutation(str);
      str = printf_processing(str, report_error);
      quote_linting(str, report_error);
    }
    if (do_linting) {
      global_scope_constant_linting(str, report_error);
      matrix_constructor_linting(str, report_error);
      array_constructor_linting(str, report_error);
    }
    if (do_small_type_linting) {
      small_type_linting(str, report_error);
    }
    str = remove_quotes(str);
    str = enum_macro_injection(str);
    if (do_resource_argument) {
      str = resource_arguments_mutation(str);
    }
    str = argument_decorator_macro_injection(str);
    str = array_constructor_macro_injection(str);
    return generated_header_.str() + line_directive_prefix(filename) + str +
           threadgroup_variables_suffix() + "//__blender_metadata_sta\n" + gpu_functions_.str() +
           static_strings_suffix() + gpu_builtins_suffix(filename) + dependency_suffix() +
           "//__blender_metadata_end\n";
  }

  /* Variant use for python shaders. */
  std::string process(const std::string &str)
  {
    auto no_err_report = [](std::smatch, const char *) {};
    return process(str, "", false, false, false, false, false, false, no_err_report);
  }

 private:
  using regex_callback = std::function<void(const std::smatch &)>;

  /* Helper to make the code more readable in parsing functions. */
  void regex_global_search(const std::string &str,
                           const std::regex &regex,
                           regex_callback callback)
  {
    using namespace std;
    string::const_iterator it = str.begin();
    for (smatch match; regex_search(it, str.end(), match, regex); it = match.suffix().first) {
      callback(match);
    }
  }

  template<typename ReportErrorF>
  std::string remove_comments(const std::string &str, const ReportErrorF &report_error)
  {
    std::string out_str = str;
    {
      /* Multi-line comments. */
      size_t start, end = 0;
      while ((start = out_str.find("/*", end)) != std::string::npos) {
        end = out_str.find("*/", start + 2);
        if (end == std::string::npos) {
          break;
        }
        for (size_t i = start; i < end + 2; ++i) {
          if (out_str[i] != '\n') {
            out_str[i] = ' ';
          }
        }
      }

      if (end == std::string::npos) {
        /* TODO(fclem): Add line / char position to report. */
        report_error(std::smatch(), "Malformed multi-line comment.");
        return out_str;
      }
    }
    {
      /* Single-line comments. */
      size_t start, end = 0;
      while ((start = out_str.find("//", end)) != std::string::npos) {
        end = out_str.find('\n', start + 2);
        if (end == std::string::npos) {
          break;
        }
        for (size_t i = start; i < end; ++i) {
          out_str[i] = ' ';
        }
      }

      if (end == std::string::npos) {
        /* TODO(fclem): Add line / char position to report. */
        report_error(std::smatch(), "Malformed single line comment, missing newline.");
        return out_str;
      }
    }
    /* Remove trailing white space as they make the subsequent regex much slower. */
    std::regex regex(R"((\ )*?\n)");
    return std::regex_replace(out_str, regex, "\n");
  }

  std::string remove_quotes(const std::string &str)
  {
    return std::regex_replace(str, std::regex(R"(["'])"), " ");
  }

  void include_parse(const std::string &str)
  {
    /* Parse include directive before removing them. */
    std::regex regex(R"(#\s*include\s*\"(\w+\.\w+)\")");

    regex_global_search(str, regex, [&](const std::smatch &match) {
      std::string dependency_name = match[1].str();
      if (dependency_name == "gpu_glsl_cpp_stubs.hh") {
        /* Skip GLSL-C++ stubs. They are only for IDE linting. */
        return;
      }
      dependencies_.emplace_back(dependency_name);
    });
  }

  std::string preprocessor_directive_mutation(const std::string &str)
  {
    /* Remove unsupported directives.` */
    std::regex regex(R"(#\s*(?:include|pragma once)[^\n]*)");
    return std::regex_replace(str, regex, "");
  }

  std::string dependency_suffix()
  {
    if (dependencies_.empty()) {
      return "";
    }
    std::stringstream suffix;
    for (const std::string &filename : dependencies_) {
      suffix << "// " << std::to_string(hash("dependency")) << " " << filename << "\n";
    }
    return suffix.str();
  }

  void threadgroup_variables_parsing(const std::string &str)
  {
    std::regex regex(R"(shared\s+(\w+)\s+(\w+)([^;]*);)");
    regex_global_search(str, regex, [&](const std::smatch &match) {
      shared_vars_.push_back({match[1].str(), match[2].str(), match[3].str()});
    });
  }

  void parse_library_functions(const std::string &str)
  {
    std::regex regex_func(R"(void\s+(\w+)\s*\(([^)]+\))\s*\{)");
    regex_global_search(str, regex_func, [&](const std::smatch &match) {
      std::string name = match[1].str();
      std::string args = match[2].str();
      gpu_functions_ << "// " << hash("function") << " " << name;

      std::regex regex_arg(R"((?:(const|in|out|inout)\s)?(\w+)\s([\w\[\]]+)(?:,|\)))");
      regex_global_search(args, regex_arg, [&](const std::smatch &arg) {
        std::string qualifier = arg[1].str();
        std::string type = arg[2].str();
        if (qualifier.empty() || qualifier == "const") {
          qualifier = "in";
        }
        gpu_functions_ << ' ' << hash(qualifier) << ' ' << hash(type);
      });
      gpu_functions_ << "\n";
    });
  }

  void parse_builtins(const std::string &str)
  {
    /* TODO: This can trigger false positive caused by disabled #if blocks. */
    std::regex regex(
        "("
        "gl_FragCoord|"
        "gl_FrontFacing|"
        "gl_GlobalInvocationID|"
        "gl_InstanceID|"
        "gl_LocalInvocationID|"
        "gl_LocalInvocationIndex|"
        "gl_NumWorkGroup|"
        "gl_PointCoord|"
        "gl_PointSize|"
        "gl_PrimitiveID|"
        "gl_VertexID|"
        "gl_WorkGroupID|"
        "gl_WorkGroupSize|"
        "drw_debug_|"
        "printf"
        ")");
    regex_global_search(
        str, regex, [&](const std::smatch &match) { gpu_builtins_.insert(match[0].str()); });
  }

  std::string gpu_builtins_suffix(const std::string &filename)
  {
    if (gpu_builtins_.empty()) {
      return "";
    }

    const bool skip_drw_debug = filename.find("common_debug_draw_lib.glsl") != std::string::npos ||
                                filename.find("draw_debug_draw_display_vert.glsl") !=
                                    std::string::npos;

    std::stringstream suffix;
    for (const std::string &str_var : gpu_builtins_) {
      if (str_var == "drw_debug_" && skip_drw_debug) {
        continue;
      }
      suffix << "// " << hash("builtin") << " " << hash(str_var) << "\n";
    }
    return suffix.str();
  }

  template<typename ReportErrorF>
  std::string printf_processing(const std::string &str, const ReportErrorF &report_error)
  {
    std::string out_str = str;
    {
      /* Example: `printf(2, b, f(c, d));` > `printf(2@ b@ f(c@ d))$` */
      size_t start, end = 0;
      while ((start = out_str.find("printf(", end)) != std::string::npos) {
        end = out_str.find(';', start);
        if (end == std::string::npos) {
          break;
        }
        out_str[end] = '$';
        int bracket_depth = 0;
        int arg_len = 0;
        for (size_t i = start; i < end; ++i) {
          if (out_str[i] == '(') {
            bracket_depth++;
          }
          else if (out_str[i] == ')') {
            bracket_depth--;
          }
          else if (bracket_depth == 1 && out_str[i] == ',') {
            out_str[i] = '@';
            arg_len++;
          }
        }
        if (arg_len > 99) {
          report_error(std::smatch(), "Too many parameters in printf. Max is 99.");
          break;
        }
        /* Encode number of arg in the `ntf` of `printf`. */
        out_str[start + sizeof("printf") - 4] = '$';
        out_str[start + sizeof("printf") - 3] = ((arg_len / 10) > 0) ? ('0' + arg_len / 10) : '$';
        out_str[start + sizeof("printf") - 2] = '0' + arg_len % 10;
      }
      if (end == 0) {
        /* No printf in source. */
        return str;
      }
    }
    /* Example: `pri$$1(2@ b)$` > `{int c_ = print_header(1, 2); c_ = print_data(c_, b); }` */
    {
      std::regex regex(R"(pri\$\$?(\d{1,2})\()");
      out_str = std::regex_replace(out_str, regex, "{uint c_ = print_header($1u, ");
    }
    {
      std::regex regex(R"(\@)");
      out_str = std::regex_replace(out_str, regex, "); c_ = print_data(c_,");
    }
    {
      std::regex regex(R"(\$)");
      out_str = std::regex_replace(out_str, regex, "; }");
    }
    return out_str;
  }

  void static_strings_parsing(const std::string &str)
  {
    /* Matches any character inside a pair of un-escaped quote. */
    std::regex regex(R"("(?:[^"])*")");
    regex_global_search(
        str, regex, [&](const std::smatch &match) { static_strings_.insert(match[0].str()); });
  }

  std::string static_strings_mutation(std::string str)
  {
    /* Replaces all matches by the respective string hash. */
    for (const std::string &str_var : static_strings_) {
      std::regex escape_regex(R"([\\\.\^\$\+\(\)\[\]\{\}\|\?\*])");
      std::string str_regex = std::regex_replace(str_var, escape_regex, "\\$&");

      std::regex regex(str_regex);
      str = std::regex_replace(str, regex, std::to_string(hash_32(str_var)) + 'u');
    }
    return str;
  }

  std::string static_strings_suffix()
  {
    if (static_strings_.empty()) {
      return "";
    }
    std::stringstream suffix;
    for (const std::string &str_var : static_strings_) {
      std::string no_quote = str_var.substr(1, str_var.size() - 2);
      suffix << "// " << hash("string") << " " << hash_32(str_var) << " " << no_quote << "\n";
    }
    return suffix.str();
  }

  std::string resource_arguments_mutation(std::string str)
  {
    std::stringstream generated_functions;
    std::stringstream generated_rename;
    std::stringstream generated_prototypes;

    /* Early out if no resource keyword is found. Avoid false positive that can degenerate. */
    if (!regex_search(str, std::regex(R"(\b(buffer|uniform|image)\b)"))) {
      return str;
    }

/* Workaround a processor bug in GLSLang which prevent using CONCAT in #if directives.
 * Eventually remove this if GLSLang get patched. */
#define GLSL_WORKAROUND

    auto create_string = [](std::function<void(std::stringstream &)> callback) {
      std::stringstream ss;
      callback(ss);
      return ss.str();
    };

    struct Replacement {
      size_t position, length;
      std::string content;
    };
    std::vector<Replacement> fn_replace;

    /* Note: Keep the regex simple to avoid stack overflow on MSVC. */
    std::regex regex_func(R"(\n(\w+))"  /* Return type. */
                          R"(\s+(\w+))" /* Function name. */
                          R"(\s*\()"    /* Ensure we capture functions. */
    );

    /* Line count from the beginning of the input string. */
    size_t line = 0;
    /* Offset in char from the beginning of the input string. */
    size_t offset = 0;
    regex_global_search(str, regex_func, [&](const std::smatch &match) {
      std::string prefix = match.prefix().str();
      std::string content = match[0].str();
      std::string suffix = match.suffix().str();

      offset += match.position(0) + match.length(0);
      line += std::count(prefix.begin(), prefix.end(), '\n') +
              std::count(content.begin(), content.end(), '\n');

      if (!is_function_declaration(suffix)) {
        return;
      }

      /* Recover the rest of the function */
      content += split_on_first(suffix, "\n}", true).front();

      std::string fn_return = match[1].str();
      std::string fn_name = match[2].str();
      std::string fn_args = get_content_between_balanced_pair(content, '(', ')');
      std::string fn_body = get_content_between_balanced_pair(content, '{', '}');

      /* Early out if no resource keyword is found. Avoid false positive that can degenerate. */
      if (!regex_search(fn_args, std::regex(R"(\b(buffer|uniform|image)\b)"))) {
        return;
      }

      struct Argument {
        std::string qualifier;
        std::string name;
        std::string type;
        std::string array;
        std::string prefix;
        std::vector<std::string> mem_qualifiers;
        std::vector<std::string> slots;
      };

      std::vector<Argument> resource_args;
      std::vector<Argument> regular_args;
      std::vector<Argument> all_args;

      size_t permutation_len = 1;
      for (const std::string &argument : split(fn_args, ',')) {
        Argument arg;

        /* Parse the memory qualifiers and remove them from the argument string to simplify the
         * subsequent regex. */
        std::regex regex_qual(R"(\s*\b(coherent|readonly|restrict|volatile|writeonly)\b)");
        regex_global_search(argument, regex_qual, [&](const std::smatch &qualifier_match) {
          arg.mem_qualifiers.emplace_back(qualifier_match[1].str());
        });
        std::string argument_no_qualifiers = std::regex_replace(argument, regex_qual, " ");

        /* Note: Keep the regex simple to avoid stack overflow on MSVC. */
        static std::regex regex_arg(
            R"(\s*(binding\((\w+)\))?)"         /* Resource binding slot (optional). */
            R"(\s*\b(image|uniform|buffer)?\b)" /* Resource qualifier (optional). */
            R"(\s*\b(const|inout|in|out|)?\b)"  /* Parameter qualifier (optional). */
            R"(\s*(\w+))"                       /* Type. */
            R"(\s*(\w+))"                       /* Name. */
            R"(\s*(\[\w*\])?)"                  /* Array (optional). */
        );

        std::smatch arg_match;
        if (!std::regex_match(argument_no_qualifiers, arg_match, regex_arg)) {
          std::cerr << "Argument parsing failed in \"" << fn_name << "\". " << std::endl;
        }

        arg.qualifier = arg_match[4].str();
        arg.type = arg_match[5].str();
        arg.name = arg_match[6].str();
        arg.array = arg_match[7].str();
        if (arg_match[3].matched) {
          switch (hash(arg_match[3].str())) {
            case hash("uniform"):
              arg.prefix = "UNI_";
              break;
            case hash("image"):
              arg.prefix = "IMG_";
              break;
            case hash("buffer"):
              arg.prefix = "BUF_";
              break;
          }

          if (arg_match[1].matched) {
            arg.slots.emplace_back(arg_match[2].str());
          }
          else {
            /* Match with 8 slots by default.*/
            for (int i = 0; i < 8; i++) {
              arg.slots.emplace_back(std::to_string(i));
            }
          }
          permutation_len *= arg.slots.size();
          resource_args.emplace_back(arg);
        }
        else {
          regular_args.emplace_back(arg);
        }
        all_args.emplace_back(arg);
      }

      if (resource_args.empty()) {
        return;
      }

      if (permutation_len > 8) {
        std::cerr << "Too many resource permutations in \"" << fn_name << "\". "
                  << "Use specific layout qualifier to reduce permutations." << std::endl;
        return;
      }

      size_t lines_in_body = std::count(fn_body.begin(), fn_body.end(), '\n');

      /* `#if 0` out the original declaration. Allows correct error report.  */
      Replacement fn;
      fn.position = offset - match.length(0);
      fn.length = content.length();
      fn.content = content;
      fn.content.replace(fn.length - 1, 1, "#endif //");
      fn.content.replace(1, 0, "#if 0 //");

      /* One slot per resource argument for each slot the resource is declared for. */
      using Permutation = std::vector<std::string>;

      /* Generate all permutations. */
      std::vector<Permutation> permutations;
      for (auto &res : resource_args) {
        std::vector<Permutation> permutations_copy = permutations;
        permutations.clear();
        for (const std::string &slot : res.slots) {
          /* Iterate with copy on purpose. */
          for (Permutation perm : permutations_copy) {
            perm.emplace_back(slot);
            permutations.emplace_back(perm);
          }
          /* First iteration. Create the permutation. */
          if (permutations_copy.empty()) {
            permutations.emplace_back(Permutation({slot}));
          }
        }
      }

#ifdef GLSL_WORKAROUND
      for (auto &res : resource_args) {
        std::string rename = create_string([&](std::stringstream &ss) {
          for (const std::string &slot : res.slots) {
            if (std::isdigit(slot[0])) {
              continue;
            }
            std::string &res_type = res.prefix;
            ss << "#ifndef IRRADIANCE_GRID_BUF_SLOT\n";
            ss << "#define " << res_type << slot << "_TYPE 0\n";
            for (int i = 0; i < 8; i++) {
              ss << "#elif " << slot << "==" << i << "\n";
              ss << "#define " << res_type << slot << "_TYPE " << res_type << i << "_TYPE\n";
              ss << "#define " << res_type << slot << "_READ " << res_type << i << "_READ\n";
              ss << "#define " << res_type << slot << "_WRITE " << res_type << i << "_WRITE\n";
            }
            ss << "#endif\n";
          }
        });
        generated_rename << rename;
      }
#endif

      for (const Permutation &permutation : permutations) {
        std::string perm_name = create_string([&](std::stringstream &ss) {
          ss << fn_name;
          for (int i = 0; i < resource_args.size(); i++) {
            ss << "_" << resource_args[i].prefix << permutation[i];
          }
        });

        std::string perm_proto_name = " prototype_" + perm_name + " ";

        /* Prepend prototype at the original function position to allow in argument type from
         * inside the file. The prototype is defined as a macro at the top of the file to not mess
         * with error line. */
        fn.content = perm_proto_name + fn.content;

        std::string perm_cond_start = create_string([&](std::stringstream &ss) {
          auto contains = [](const std::vector<std::string> &vec, const std::string item) {
            return (std::find(vec.begin(), vec.end(), item) != vec.end());
          };

          ss << "#if 0";
          for (int i = 0; i < resource_args.size(); i++) {
            const Argument &arg = resource_args[i];
            const std::string &buf_slot = permutation[i];
            /* Preprocessor checks. */
            const char *array_suffix = (arg.array.empty() ? "" : "_array");
            std::string type_hash = std::to_string(hash_32(arg.type + array_suffix));
            const std::string &res_type = arg.prefix;

#ifndef GLSL_WORKAROUND
            ss << " || CONCAT3(" << res_type << "," << buf_slot << ",_TYPE) != " << type_hash;
#else
            ss << " || " << res_type << buf_slot << "_TYPE != " << type_hash;
#endif
          }
#ifdef __APPLE__
          /* Warning directives only available on MSL. GLSL only have errors. */
          ss << "\n#warning " << perm_name << " disabled because of resource type mismatch\n";
#else
          /* Can be used for debugging. */
          // ss << "\n#error " << perm_name << " disabled because of resource type mismatch\n";
          ss << "\n";
#endif

          ss << "#elif 0";
          for (int i = 0; i < resource_args.size(); i++) {
            const Argument &arg = resource_args[i];
            const std::string &buf_slot = permutation[i];
            const std::string &res_type = arg.prefix;
#ifndef GLSL_WORKAROUND
            if (contains(arg.mem_qualifiers, "readonly") || arg.mem_qualifiers.empty()) {
              ss << " || CONCAT3(" << res_type << "," << buf_slot << ",_READ) != 1";
            }
            if (contains(arg.mem_qualifiers, "writeonly") || arg.mem_qualifiers.empty()) {
              ss << " || CONCAT3(" << res_type << "," << buf_slot << ",_WRITE) != 1";
            }
#else
            if (contains(arg.mem_qualifiers, "readonly") || arg.mem_qualifiers.empty()) {
              ss << " || " << res_type << buf_slot << "_READ != 1";
            }
            if (contains(arg.mem_qualifiers, "writeonly") || arg.mem_qualifiers.empty()) {
              ss << " || " << res_type << buf_slot << "_WRITE != 1";
            }
#endif
          }
#ifdef __APPLE__
          /* Warning directives only available on MSL. GLSL only have errors. */
          ss << "\n#warning " << perm_name << " disabled because of memory qualifier mismatch\n";
#else
          /* Can be used for debugging. */
          // ss << "\n#error " << perm_name << " disabled because of memory qualifier mismatch\n";
          ss << "\n";
#endif
          ss << "#else\n";
        });
        std::string perm_cond_end = "\n#endif\n";

        std::string perm_fn_name = create_string([&](std::stringstream &ss) {
          for (int i = 0; i < resource_args.size(); i++) {
            ss << " CONCAT(";
          }
          ss << fn_name << "_perm_";
          for (int i = 0; i < resource_args.size(); i++) {
            ss << ", CONCAT3(" << resource_args[i].prefix << "," << permutation[i] << ",_NAME))";
          }
        });

        std::string perm_fn_args = create_string([&](std::stringstream &ss) {
          ss << "(";
          char separator = ' ';
          for (auto &arg : regular_args) {
            ss << separator << arg.qualifier << " " << arg.type << " " << arg.name << arg.array;
            separator = ',';
          }
          ss << ")";
        });

        std::string perm_body = create_string([&](std::stringstream &ss) {
          std::string body = fn_body;
          for (int i = 0; i < resource_args.size(); i++) {
            const Argument &arg = resource_args[i];
            /* Only replace fullword match. */
            std::regex regex("\\b" + arg.name + "\\b");
            /* Replace argument name by buffer name macro. */
            std::string macro_name = "CONCAT3(" + arg.prefix + "," + permutation[i] + ",_RES)";
            body = std::regex_replace(body, regex, macro_name);
          }
          ss << "\n{" << body << "\n}";
        });

#undef GLSL_WORKAROUND

        generated_functions << "\n";
        generated_functions << "// Generated function from " << fn_name << "\n";
        generated_functions << perm_cond_start;
        generated_functions << fn_return << perm_fn_name << perm_fn_args;
        generated_functions << "\n#line " << std::to_string(line - lines_in_body);
        generated_functions << perm_body;
        generated_functions << perm_cond_end;

        generated_prototypes << "#define" << perm_proto_name << "\n";
        generated_prototypes << perm_cond_start;
        generated_prototypes << "#ifndef GPU_METAL\n";
        generated_prototypes << "#undef" << perm_proto_name << "\n";
        generated_prototypes << "#define" << perm_proto_name;
        generated_prototypes << fn_return << perm_fn_name << perm_fn_args << ";\n";
        generated_prototypes << "#endif\n";
        generated_prototypes << perm_cond_end;
      }

      /* Add function macro to redirect to correct buffer implementation. */
      std::string function_macro = create_string([&](std::stringstream &ss) {
        /* Modify the arg name to avoid referencing another macro by accident. */
        auto macro_arg_name = [](const std::string &arg_name) { return "___" + arg_name; };

        /* Function macro. Same number of argument. */
        ss << "#define " << fn_name << "(";
        for (int i = 0, sep = ' '; i < all_args.size(); i++, sep = ',') {
          ss << char(sep) << macro_arg_name(all_args[i].name);
        }
        ss << ") ";
        /* Concatenate resource argument into function name. */
        for (int i = 0; i < resource_args.size(); i++) {
          ss << "CONCAT(";
        }
        ss << fn_name << "_perm_";
        for (int i = 0; i < resource_args.size(); i++) {
#ifdef __APPLE__ /* Apple shader compiler will expand macro too much. */
          ss << ", res_##" << macro_arg_name(resource_args[i].name) << ")";
#else
          ss << ", CONCAT(res, " << macro_arg_name(resource_args[i].name) << "))";
#endif
        }
        /* Only regular arguments inside real function call. */
        ss << "(";
        for (int i = 0, sep = ' '; i < regular_args.size(); i++) {
          ss << char(sep) << macro_arg_name(regular_args[i].name);
          sep = ',';
        }
        ss << ")\n";
      });

      generated_header_ << function_macro << "\n";

      fn_replace.emplace_back(fn);
    });

    if (!fn_replace.empty()) {
      generated_header_ << generated_rename.str();

      generated_header_ << generated_prototypes.str();
    }

    {
      /* Mute original declaration for correct error reports.*/
      size_t offset = 0;
      for (auto &fn : fn_replace) {
        str.replace(fn.position + offset, fn.length, fn.content);
        offset += fn.content.size() - fn.length;
      }
    }
    return str += generated_functions.str();
  }

  std::string enum_macro_injection(std::string str)
  {
    /**
     * Transform C,C++ enum declaration into GLSL compatible defines and constants:
     *
     * \code{.cpp}
     * enum eMyEnum : uint32_t {
     *   ENUM_1 = 0u,
     *   ENUM_2 = 1u,
     *   ENUM_3 = 2u,
     * };
     * \endcode
     *
     * becomes
     *
     * \code{.glsl}
     * _enum_decl(_eMyEnum)
     *   ENUM_1 = 0u,
     *   ENUM_2 = 1u,
     *   ENUM_3 = 2u, _enum_end
     * #define eMyEnum _enum_type(_eMyEnum)
     * \endcode
     *
     * It is made like so to avoid messing with error lines, allowing to point at the exact
     * location inside the source file.
     *
     * IMPORTANT: This has some requirements:
     * - Enums needs to have underlying types set to uint32_t to make them usable in UBO and SSBO.
     * - All values needs to be specified using constant literals to avoid compiler differences.
     * - All values needs to have the 'u' suffix to avoid GLSL compiler errors.
     */
    {
      /* Replaces all matches by the respective string hash. */
      std::regex regex(R"(enum\s+((\w+)\s*(?:\:\s*\w+\s*)?)\{(\n[^}]+)\n\};)");
      str = std::regex_replace(str,
                               regex,
                               "_enum_decl(_$1)$3 _enum_end\n"
                               "#define $2 _enum_type(_$2)");
    }
    {
      /* Remove trailing comma if any. */
      std::regex regex(R"(,(\s*_enum_end))");
      str = std::regex_replace(str, regex, "$1");
    }
    return str;
  }

  std::string argument_decorator_macro_injection(const std::string &str)
  {
    /* Example: `out float var[2]` > `out float _out_sta var _out_end[2]` */
    std::regex regex(R"((out|inout|in|shared)\s+(\w+)\s+(\w+))");
    return std::regex_replace(str, regex, "$1 $2 _$1_sta $3 _$1_end");
  }

  std::string array_constructor_macro_injection(const std::string &str)
  {
    /* Example: `= float[2](0.0, 0.0)` > `= ARRAY_T(float) ARRAY_V(0.0, 0.0)` */
    std::regex regex(R"(=\s*(\w+)\s*\[[^\]]*\]\s*\()");
    return std::regex_replace(str, regex, "= ARRAY_T($1) ARRAY_V(");
  }

  /* TODO(fclem): Too many false positive and false negative to be applied to python shaders. */
  void matrix_constructor_linting(const std::string &str, report_callback report_error)
  {
    /* Example: `mat4(other_mat)`. */
    std::regex regex(R"(\s+(mat(\d|\dx\d)|float\dx\d)\([^,\s\d]+\))");
    regex_global_search(str, regex, [&](const std::smatch &match) {
      /* This only catches some invalid usage. For the rest, the CI will catch them. */
      const char *msg =
          "Matrix constructor is not cross API compatible. "
          "Use to_floatNxM to reshape the matrix or use other constructors instead.";
      report_error(match, msg);
    });
  }

  /* Assume formatted source with our code style. Cannot be applied to python shaders. */
  template<typename ReportErrorF>
  void global_scope_constant_linting(std::string str, const ReportErrorF &report_error)
  {
    /* Example: `const uint global_var = 1u;`. Matches if not indented (i.e. inside a scope). */
    std::regex regex(R"(const \w+ \w+ =)");
    regex_global_search(str, regex, [&](const std::smatch &match) {
      /* Positive look-behind is not supported in #std::regex. Do it manually. */
      if (match.prefix().str().back() == '\n') {
        const char *msg =
            "Global scope constant expression found. These get allocated per-thread in MSL. "
            "Use Macro's or uniforms instead.";
        report_error(match, msg);
      }
    });
  }

  void quote_linting(const std::string &str, report_callback report_error)
  {
    std::regex regex(R"(["'])");
    regex_global_search(str, regex, [&](const std::smatch &match) {
      /* This only catches some invalid usage. For the rest, the CI will catch them. */
      const char *msg = "Quotes are forbidden in GLSL.";
      report_error(match, msg);
    });
  }

  template<typename ReportErrorF>
  void array_constructor_linting(const std::string &str, const ReportErrorF &report_error)
  {
    std::regex regex(R"(=\s*(\w+)\s*\[[^\]]*\]\s*\()");
    regex_global_search(str, regex, [&](const std::smatch &match) {
      /* This only catches some invalid usage. For the rest, the CI will catch them. */
      const char *msg =
          "Array constructor is not cross API compatible. Use type_array instead of type[].";
      report_error(match, msg);
    });
  }

  template<typename ReportErrorF>
  void small_type_linting(const std::string &str, const ReportErrorF &report_error)
  {
    std::regex regex(R"(\su?(char|short|half)(2|3|4)?\s)");
    regex_global_search(str, regex, [&](const std::smatch &match) {
      report_error(match, "Small types are forbidden in shader interfaces.");
    });
  }

  std::string threadgroup_variables_suffix()
  {
    if (shared_vars_.empty()) {
      return "";
    }

    std::stringstream suffix;
    /**
     * For Metal shaders to compile, shared (threadgroup) variable cannot be declared globally.
     * They must reside within a function scope. Hence, we need to extract these declarations and
     * generate shared memory blocks within the entry point function. These shared memory blocks
     * can then be passed as references to the remaining shader via the class function scope.
     *
     * The shared variable definitions from the source file are replaced with references to
     * threadgroup memory blocks (using _shared_sta and _shared_end macros), but kept in-line in
     * case external macros are used to declare the dimensions.
     *
     * Each part of the codegen is stored inside macros so that we don't have to do string
     * replacement at runtime.
     */
    suffix << "\n";
    /* Arguments of the wrapper class constructor. */
    suffix << "#undef MSL_SHARED_VARS_ARGS\n";
    /* References assignment inside wrapper class constructor. */
    suffix << "#undef MSL_SHARED_VARS_ASSIGN\n";
    /* Declaration of threadgroup variables in entry point function. */
    suffix << "#undef MSL_SHARED_VARS_DECLARE\n";
    /* Arguments for wrapper class constructor call. */
    suffix << "#undef MSL_SHARED_VARS_PASS\n";

    /**
     * Example replacement:
     *
     * `
     * // Source
     * shared float bar[10];                                    // Source declaration.
     * shared float foo;                                        // Source declaration.
     * // Rest of the source ...
     * // End of Source
     *
     * // Backend Output
     * class Wrapper {                                          // Added at runtime by backend.
     *
     * threadgroup float (&foo);                                // Replaced by regex and macros.
     * threadgroup float (&bar)[10];                            // Replaced by regex and macros.
     * // Rest of the source ...
     *
     * Wrapper (                                                // Added at runtime by backend.
     * threadgroup float (&_foo), threadgroup float (&_bar)[10] // MSL_SHARED_VARS_ARGS
     * )                                                        // Added at runtime by backend.
     * : foo(_foo), bar(_bar)                                   // MSL_SHARED_VARS_ASSIGN
     * {}                                                       // Added at runtime by backend.
     *
     * }; // End of Wrapper                                     // Added at runtime by backend.
     *
     * kernel entry_point() {                                   // Added at runtime by backend.
     *
     * threadgroup float foo;                                   // MSL_SHARED_VARS_DECLARE
     * threadgroup float bar[10]                                // MSL_SHARED_VARS_DECLARE
     *
     * Wrapper wrapper                                          // Added at runtime by backend.
     * (foo, bar)                                               // MSL_SHARED_VARS_PASS
     * ;                                                        // Added at runtime by backend.
     *
     * }                                                        // Added at runtime by backend.
     * // End of Backend Output
     * `
     */
    std::stringstream args, assign, declare, pass;

    bool first = true;
    for (SharedVar &var : shared_vars_) {
      char sep = first ? ' ' : ',';
      /*  */
      args << sep << "threadgroup " << var.type << "(&_" << var.name << ")" << var.array;
      assign << (first ? ':' : ',') << var.name << "(_" << var.name << ")";
      declare << "threadgroup " << var.type << ' ' << var.name << var.array << ";";
      pass << sep << var.name;
      first = false;
    }

    suffix << "#define MSL_SHARED_VARS_ARGS " << args.str() << "\n";
    suffix << "#define MSL_SHARED_VARS_ASSIGN " << assign.str() << "\n";
    suffix << "#define MSL_SHARED_VARS_DECLARE " << declare.str() << "\n";
    suffix << "#define MSL_SHARED_VARS_PASS (" << pass.str() << ")\n";
    suffix << "\n";

    return suffix.str();
  }

  std::string line_directive_prefix(const std::string &filepath)
  {
    std::string filename = std::regex_replace(filepath, std::regex(R"((?:.*)\/(.*))"), "$1");

    std::stringstream suffix;
    suffix << "#line 1 ";
#ifdef __APPLE__
    /* For now, only Metal supports filename in line directive.
     * There is no way to know the actual backend, so we assume Apple uses Metal. */
    /* TODO(fclem): We could make it work using a macro to choose between the filename and the hash
     * at runtime. i.e.: `FILENAME_MACRO(12546546541, 'filename.glsl')` This should work for both
     * MSL and GLSL. */
    if (!filename.empty()) {
      suffix << "\"" << filename << "\"";
    }
#else
    uint64_t hash_value = hash(filename);
    /* Fold the value so it fits the GLSL spec. */
    hash_value = (hash_value ^ (hash_value >> 32)) & (~uint64_t(0) >> 33);
    suffix << std::to_string(uint64_t(hash_value));
#endif
    suffix << "\n";
    return suffix.str();
  }

  /* Returns true if str starts inside a function definition.
   * That is, if a semicolon doesn't exists before the function body.
   * Assumes strings already starts in either a function definition or declaration. */
  bool is_function_declaration(const std::string &str)
  {
    /* Assumes formatting and that function is in global scope. */
    size_t scope_pos = str.find("\n{");
    if (scope_pos == std::string::npos) {
      /* No function scope. */
      return false;
    }
    size_t semicol_pos = str.find(';');
    if (semicol_pos == std::string::npos) {
      /* Weird. But maybe the function is empty or uses macros. */
      return true;
    }
    /* Make sure this is a function definition and not a prototype. */
    return scope_pos < semicol_pos;
  }

  /* Split input string into many substrings based on a separator.
   * The returned substring doesn't include the separator.
   * Returns only the input string if no separator exists. */
  std::vector<std::string> split(const std::string &input, const char separator)
  {
    size_t start = 0;
    size_t end = 0;
    std::vector<std::string> result;
    while ((end = input.find(separator, start)) != std::string::npos) {
      result.push_back(input.substr(start, end - start));
      start = end + 1;
    }
    result.push_back(input.substr(start));
    return result;
  }

  /* Split input string into many substrings based on a separator.
   * The returned substring doesn't include the separator, except if include_separator is true.
   * Returns only the input string if no separator exists. */
  std::array<std::string, 2> split_on_first(const std::string &input,
                                            const std::string &separator,
                                            bool include_separator = false)
  {
    size_t pos = input.find(separator);
    if (pos == std::string::npos) {
      return {input, ""};
    }
    size_t content_end = pos + (include_separator ? separator.length() : 0);
    size_t suffix_start = pos + separator.length();
    return {input.substr(0, content_end), input.substr(suffix_start)};
  }

  std::string get_content_between_balanced_pair(const std::string &input,
                                                const char start_delimiter,
                                                const char end_delimiter)
  {
    int balance = 0;
    size_t start = std::string::npos;
    size_t end = std::string::npos;

    for (size_t i = 0; i < input.length(); ++i) {
      if (input[i] == start_delimiter) {
        if (balance == 0) {
          start = i;
        }
        balance++;
      }
      else if (input[i] == end_delimiter) {
        balance--;
        if (balance == 0 && start != std::string::npos) {
          end = i;
          return input.substr(start + 1, end - start - 1);
        }
      }
    }
    return "";
  }
};

/* Enum values of metadata that the preprocessor can append at the end of a source file.
 * Eventually, remove the need for these and output the metadata inside header files. */
namespace metadata {

enum Builtin : uint64_t {
  FragCoord = Preprocessor::hash("gl_FragCoord"),
  FrontFacing = Preprocessor::hash("gl_FrontFacing"),
  GlobalInvocationID = Preprocessor::hash("gl_GlobalInvocationID"),
  InstanceID = Preprocessor::hash("gl_InstanceID"),
  LocalInvocationID = Preprocessor::hash("gl_LocalInvocationID"),
  LocalInvocationIndex = Preprocessor::hash("gl_LocalInvocationIndex"),
  NumWorkGroup = Preprocessor::hash("gl_NumWorkGroup"),
  PointCoord = Preprocessor::hash("gl_PointCoord"),
  PointSize = Preprocessor::hash("gl_PointSize"),
  PrimitiveID = Preprocessor::hash("gl_PrimitiveID"),
  VertexID = Preprocessor::hash("gl_VertexID"),
  WorkGroupID = Preprocessor::hash("gl_WorkGroupID"),
  WorkGroupSize = Preprocessor::hash("gl_WorkGroupSize"),
  drw_debug = Preprocessor::hash("drw_debug_"),
  printf = Preprocessor::hash("printf"),
};

enum Qualifier : uint64_t {
  in = Preprocessor::hash("in"),
  out = Preprocessor::hash("out"),
  inout = Preprocessor::hash("inout"),
};

enum Type : uint64_t {
  vec1 = Preprocessor::hash("float"),
  vec2 = Preprocessor::hash("vec2"),
  vec3 = Preprocessor::hash("vec3"),
  vec4 = Preprocessor::hash("vec4"),
  mat3 = Preprocessor::hash("mat3"),
  mat4 = Preprocessor::hash("mat4"),
  sampler1DArray = Preprocessor::hash("sampler1DArray"),
  sampler2DArray = Preprocessor::hash("sampler2DArray"),
  sampler2D = Preprocessor::hash("sampler2D"),
  sampler3D = Preprocessor::hash("sampler3D"),
  Closure = Preprocessor::hash("Closure"),
};

}  // namespace metadata

}  // namespace blender::gpu::shader
