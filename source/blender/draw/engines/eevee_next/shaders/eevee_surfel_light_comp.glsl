/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Apply lights contribution to scene surfel representation.
 */

#pragma BLENDER_REQUIRE(eevee_light_eval_lib.glsl)

#ifndef LIGHT_ITER_FORCE_NO_CULLING
#  error light_eval_reflection argument assumes this is defined
#endif

void main()
{
  int index = int(gl_GlobalInvocationID.x);
  if (index >= int(capture_info_buf.surfel_len)) {
    return;
  }

  Surfel surfel = surfel_buf[index];

  /* There is no view dependent effect as we evaluate everything using diffuse. */
  vec3 V = surfel.normal;
  vec3 Ng = surfel.normal;
  vec3 P = surfel.position;

  ClosureUndetermined cl;
  cl.N = surfel.normal;
  cl.type = CLOSURE_BSDF_DIFFUSE_ID;

  ClosureLightStack stack;
  stack.cl[0] = closure_light_new(cl, V);

  light_eval_reflection(stack, P, Ng, V, 0.0);

  if (capture_info_buf.capture_indirect) {
    surfel_buf[index].radiance_direct.front.rgb += stack.cl[0].light_shadowed *
                                                   surfel.albedo_front;
  }

  V = -surfel.normal;
  Ng = -surfel.normal;
  stack.cl[0].N = -surfel.normal;
  light_eval_reflection(stack, P, Ng, V, 0.0);

  if (capture_info_buf.capture_indirect) {
    surfel_buf[index].radiance_direct.back.rgb += stack.cl[0].light_shadowed * surfel.albedo_back;
  }
}
