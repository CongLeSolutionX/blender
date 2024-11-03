/* SPDX-FileCopyrightText: 2024 Tenkai Raiko
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "gpu_shader_common_math_utils.glsl"

/* Define macro flags for code translation. */
/* No macro flags necessary, as code is translated to GLSL by default. */

/* The actual rounded polygon functions are in rounded_polygon_generic.glsl. */
#include "rounded_polygon_generic.glsl"

/* Undefine macro flags used for code translation. */
/* No macro flags necessary, as code is translated to GLSL by default. */

void node_tex_rounded_polygon(vec3 coord,
                              float scale,
                              float r_gon_sides,
                              float r_gon_roundness,
                              float normalize_r_gon_parameter,
                              float elliptical_corners,
                              float calculate_r_gon_parameter_field,
                              float calculate_max_unit_parameter,
                              out float out_r_gon_field,
                              out vec3 out_segment_coordinates,
                              out float out_max_unit_parameter,
                              out float out_x_axis_A_angle_bisector)
{
  vec4 out_variables = calculate_out_fields(bool(calculate_r_gon_parameter_field),
                                            bool(calculate_max_unit_parameter),
                                            bool(normalize_r_gon_parameter),
                                            bool(elliptical_corners),
                                            max(r_gon_sides, 2.0),
                                            clamp(r_gon_roundness, 0.0, 1.0),
                                            scale * vec2(coord.x, coord.y));

  out_r_gon_field = out_variables.x;
  out_segment_coordinates = vec3(out_variables.y, out_variables.x - 1.0, 0.0);
  out_max_unit_parameter = out_variables.z;
  out_x_axis_A_angle_bisector = out_variables.w;
}
