/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/camera/projection.h"
#include "kernel/integrator/displacement_shader.h"
#include "kernel/integrator/surface_shader.h"

#include "kernel/geom/geom.h"

#include "kernel/util/color.h"

CCL_NAMESPACE_BEGIN

ccl_device void kernel_displace_evaluate(KernelGlobals kg,
                                         ccl_global const KernelShaderEvalInput *input,
                                         ccl_global float *output,
                                         const int offset)
{
  /* Setup shader data. */
  const KernelShaderEvalInput in = input[offset];

  ShaderData sd;
  shader_setup_from_displace(kg, &sd, in.object, in.prim, in.u, in.v);

  /* Evaluate displacement shader. */
  const float3 P = sd.P;
  displacement_shader_eval(kg, INTEGRATOR_STATE_NULL, &sd);
  float3 D = sd.P - P;

  object_inverse_dir_transform(kg, &sd, &D);

#ifdef __KERNEL_DEBUG_NAN__
  if (!isfinite_safe(D)) {
    kernel_assert(!"Cycles displacement with non-finite value detected");
  }
#endif

  /* Ensure finite displacement, preventing BVH from becoming degenerate and avoiding possible
   * traversal issues caused by non-finite math. */
  D = ensure_finite(D);

  /* Write output. */
  output[offset * 3 + 0] += D.x;
  output[offset * 3 + 1] += D.y;
  output[offset * 3 + 2] += D.z;
}

ccl_device void kernel_background_evaluate(KernelGlobals kg,
                                           ccl_global const KernelShaderEvalInput *input,
                                           ccl_global float *output,
                                           const int offset)
{
  /* Setup ray */
  const KernelShaderEvalInput in = input[offset];
  const float3 ray_P = zero_float3();
  const float3 ray_D = equirectangular_to_direction(in.u, in.v);
  const float ray_time = 0.5f;

  /* Setup shader data. */
  ShaderData sd;
  shader_setup_from_background(kg, &sd, ray_P, ray_D, ray_time);

  /* Evaluate shader.
   * This is being evaluated for all BSDFs, so path flag does not contain a specific type.
   * However, we want to flag the ray visibility to ignore the sun in the background map. */
  const uint32_t path_flag = PATH_RAY_EMISSION | PATH_RAY_IMPORTANCE_BAKE;
  surface_shader_eval<KERNEL_FEATURE_NODE_MASK_SURFACE_LIGHT &
                      ~(KERNEL_FEATURE_NODE_RAYTRACE | KERNEL_FEATURE_NODE_LIGHT_PATH)>(
      kg, INTEGRATOR_STATE_NULL, &sd, NULL, path_flag);
  Spectrum color = surface_shader_background(&sd);

#ifdef __KERNEL_DEBUG_NAN__
  if (!isfinite_safe(color)) {
    kernel_assert(!"Cycles background with non-finite value detected");
  }
#endif

  /* Ensure finite color, avoiding possible numerical instabilities in the path tracing kernels. */
  color = ensure_finite(color);

  float3 color_rgb = spectrum_to_rgb(color);

  /* Write output. */
  output[offset * 3 + 0] += color_rgb.x;
  output[offset * 3 + 1] += color_rgb.y;
  output[offset * 3 + 2] += color_rgb.z;
}

ccl_device void kernel_curve_shadow_transparency_evaluate(
    KernelGlobals kg,
    ccl_global const KernelShaderEvalInput *input,
    ccl_global float *output,
    const int offset)
{
#ifdef __HAIR__
  /* Setup shader data. */
  const KernelShaderEvalInput in = input[offset];

  ShaderData sd;
  shader_setup_from_curve(kg, &sd, in.object, in.prim, __float_as_int(in.v), in.u);

  /* Evaluate transparency. */
  surface_shader_eval<KERNEL_FEATURE_NODE_MASK_SURFACE_SHADOW &
                      ~(KERNEL_FEATURE_NODE_RAYTRACE | KERNEL_FEATURE_NODE_LIGHT_PATH)>(
      kg, INTEGRATOR_STATE_NULL, &sd, NULL, PATH_RAY_SHADOW);

  /* Write output. */
  output[offset] = clamp(average(surface_shader_transparency(kg, &sd)), 0.0f, 1.0f);
#endif
}

ccl_device void kernel_volume_density_evaluate(KernelGlobals kg,
                                               ccl_global const KernelShaderEvalInput *input,
                                               ccl_global float *output,
                                               const int offset)
{
#ifdef __VOLUME__
  /* Setup shader data. */
  KernelShaderEvalInput in = input[offset * 2 + 0];

  if (in.object == OBJECT_NONE) {
    return;
  }

  ShaderData sd;
  Ray ray;
  ray.P = make_float3(__int_as_float(in.prim), in.u, in.v);
  ray.D = zero_float3();
  /* TODO(weizhen): What fields need to be written? */
  shader_setup_from_volume(kg, &sd, &ray, in.object);
  sd.flag = SD_IS_VOLUME_SHADER_EVAL;
  /* No need for closure when evaluating extinction. */
  sd.num_closure_left = 0;

  in = input[offset * 2 + 1];
  const int shader = in.object;
  const VolumeStack entry = {sd.object, shader};
  const float3 voxel_size = make_float3(__int_as_float(in.prim), in.u, in.v);

  Extrema<float> extrema = {FLT_MAX, 0.0f};
  for (int sample = 0; sample < 10; sample++) {
    /* TODO(weizhen): fix the blue noise sample number problem. */
    const float3 rand_p = path_rng_3D(kg, offset, 0, sample);
    sd.P = ray.P + rand_p * voxel_size;
    sd.closure_transparent_extinction = zero_float3();

    /* Evaluate volume extinction coefficients. */
    volume_shader_eval_entry<false,
                             KERNEL_FEATURE_NODE_MASK_VOLUME & ~KERNEL_FEATURE_NODE_LIGHT_PATH>(
        kg, INTEGRATOR_STATE_NULL, &sd, entry, PATH_RAY_SHADOW);

    const float sigma = (sd.flag & SD_EXTINCTION) ? reduce_max(sd.closure_transparent_extinction) :
                                                    0.0f;

    extrema = join(extrema, sigma);
  }

  /* Write output. */
  output[offset * 2 + 0] = extrema.min;
  output[offset * 2 + 1] = extrema.max;
#endif
}

CCL_NAMESPACE_END
