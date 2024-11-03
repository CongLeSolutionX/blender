/* SPDX-FileCopyrightText: 2024 Tenkai Raiko
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

CCL_NAMESPACE_BEGIN

struct RoundedPolygonStackOffsets {
  uint vector;
  uint scale;
  uint r_gon_sides;
  uint r_gon_roundness;
  uint r_gon_field;
  uint segment_coordinates;
  uint max_unit_parameter;
  uint x_axis_A_angle_bisector;
};

/* Define macro flags for code translation. */
#define TRANSLATE_TO_SVM

/* The actual rounded polygon functions are in rounded_polygon_generic.glsl. */
#include "../../../../source/blender/gpu/shaders/material/rounded_polygon_generic.glsl"

/* Undefine macro flags used for code translation. */
#undef TRANSLATE_TO_SVM

template<uint node_feature_mask>
ccl_device_noinline int svm_node_tex_rounded_polygon(
    KernelGlobals kg, ccl_private ShaderData *sd, ccl_private float *stack, uint4 node, int offset)
{
  RoundedPolygonStackOffsets so;

  uint normalize_r_gon_parameter, elliptical_corners;

  svm_unpack_node_uchar4(
      node.y, &(normalize_r_gon_parameter), &(elliptical_corners), &(so.vector), &(so.scale));
  svm_unpack_node_uchar4(node.z,
                         &(so.r_gon_sides),
                         &(so.r_gon_roundness),
                         &(so.r_gon_field),
                         &(so.segment_coordinates));
  svm_unpack_node_uchar2(node.w, &(so.max_unit_parameter), &(so.x_axis_A_angle_bisector));

  bool calculate_r_gon_parameter_field = stack_valid(so.segment_coordinates);
  bool calculate_max_unit_parameter = stack_valid(so.max_unit_parameter);

  float3 coord = stack_load_float3(stack, so.vector);
  uint4 defaults = read_node(kg, &offset);
  float scale = stack_load_float_default(stack, so.scale, defaults.x);
  float r_gon_sides = stack_load_float_default(stack, so.r_gon_sides, defaults.y);
  float r_gon_roundness = stack_load_float_default(stack, so.r_gon_roundness, defaults.z);

  float4 out_variables = calculate_out_fields(calculate_r_gon_parameter_field,
                                              calculate_max_unit_parameter,
                                              normalize_r_gon_parameter,
                                              elliptical_corners,
                                              float_max(r_gon_sides, 2.0f),
                                              clamp(r_gon_roundness, 0.0f, 1.0f),
                                              scale * make_float2(coord.x, coord.y));

  if (stack_valid(so.r_gon_field)) {
    stack_store_float(stack, so.r_gon_field, out_variables.x);
  }
  if (stack_valid(so.segment_coordinates)) {
    stack_store_float3(
        stack, so.segment_coordinates, make_float3(out_variables.y, out_variables.x - 1.0f, 0.0));
  }
  if (stack_valid(so.max_unit_parameter)) {
    stack_store_float(stack, so.max_unit_parameter, out_variables.z);
  }
  if (stack_valid(so.x_axis_A_angle_bisector)) {
    stack_store_float(stack, so.x_axis_A_angle_bisector, out_variables.w);
  }

  return offset;
}

CCL_NAMESPACE_END
