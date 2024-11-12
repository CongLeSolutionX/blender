# SPDX-FileCopyrightText: 2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC_GLSL
  .
  ..

  ../../../gpu
  ../../../gpu/intern
  ../../../gpu/shaders
  ../../../gpu/shaders/common
)

set(SRC_GLSL_VERT
  draw_debug_draw_display_vert.glsl
)

set(SRC_GLSL_FRAG
  draw_debug_draw_display_frag.glsl
)

set(SRC_GLSL_COMP
  common_hair_refine_comp.glsl
  draw_command_generate_comp.glsl
  draw_resource_finalize_comp.glsl
  draw_view_finalize_comp.glsl
  draw_visibility_comp.glsl
)

set(SRC_GLSL_LIB
#   draw_command_generate_comp.glsl
#   draw_debug_info.hh
#   draw_fullscreen_info.hh
#   draw_hair_refine_info.hh
#   draw_intersect_lib.glsl
#   draw_math_geom_lib.glsl
#   draw_model_lib.glsl
#   draw_object_infos_info.hh
#   draw_resource_finalize_comp.glsl
#   draw_view_finalize_comp.glsl
#   draw_view_info.hh
#   draw_view_lib.glsl
#   draw_view_reconstruction_lib.glsl
)

add_definitions(-DGPU_SHADER)
# TODO Remove
add_definitions(-DUSE_GPU_SHADER_CREATE_INFO)

function(compile_sources_as_cpp
  executable
  sources
  define
  )

  foreach(glsl_file ${sources})
    set_source_files_properties(${glsl_file} PROPERTIES LANGUAGE CXX)
  endforeach()

  add_library(${executable} OBJECT ${sources})
  set_target_properties(${executable} PROPERTIES LINKER_LANGUAGE CXX)
  target_include_directories(${executable} PUBLIC ${INC_GLSL})
  target_compile_definitions(${executable} PRIVATE ${define})
endfunction()


# Compile shaders with shader code.
if (WITH_GPU_SHADER_CPP_COMPILATION)
  compile_sources_as_cpp(draw_cpp_shaders_vert ${SRC_GLSL_VERT} "GPU_VERTEX_SHADER")
  compile_sources_as_cpp(draw_cpp_shaders_frag ${SRC_GLSL_FRAG} "GPU_FRAGMENT_SHADER")
  compile_sources_as_cpp(draw_cpp_shaders_comp ${SRC_GLSL_COMP} "GPU_COMPUTE_SHADER")
#   compile_sources_as_cpp(draw_cpp_shaders_lib ${SRC_GLSL_LIB} "GPU_FRAGMENT_SHADER")
endif()
