/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/sample/mapping.h"

CCL_NAMESPACE_BEGIN

/* Light Sample Result */

typedef struct LightSample {
  float3 P;            /* position on light, or direction for distant light */
  packed_float3 Ng;    /* normal on light */
  float t;             /* distance to light (FLT_MAX for distant light) */
  float3 D;            /* direction from shading point to light */
  float u, v;          /* parametric coordinate on primitive */
  float pdf;           /* pdf for selecting light and point on light */
  float pdf_selection; /* pdf for selecting light */
  float eval_fac;      /* intensity multiplier */
  int object;          /* object id for triangle/curve lights */
  int prim;            /* primitive id for triangle/curve lights */
  int shader;          /* shader id */
  int lamp;            /* lamp id */
  int group;           /* lightgroup */
  LightType type;      /* type of light */
  int emitter_id;      /* index in the emitter array */
} LightSample;

/* Utilities */

ccl_device_inline float3 ellipse_sample(float3 ru, float3 rv, float2 rand)
{
  const float2 uv = sample_uniform_disk(rand);
  return ru * uv.x + rv * uv.y;
}

ccl_device_inline float3 rectangle_sample(float3 ru, float3 rv, float2 rand)
{
  return ru * (2.0f * rand.x - 1.0f) + rv * (2.0f * rand.y - 1.0f);
}

ccl_device float3 disk_light_sample(float3 n, float2 rand)
{
  float3 ru, rv;

  make_orthonormals(n, &ru, &rv);

  return ellipse_sample(ru, rv, rand);
}

ccl_device float light_pdf_area_to_solid_angle(const float3 Ng, const float3 I, float t)
{
  float cos_pi = dot(Ng, I);

  if (cos_pi <= 0.0f)
    return 0.0f;

  return t * t / cos_pi;
}

/* Visibility flag om the light shader. */
ccl_device_inline bool is_light_shader_visible_to_path(const int shader, const uint32_t path_flag)
{
  if ((shader & SHADER_EXCLUDE_ANY) == 0) {
    return true;
  }

  if (((shader & SHADER_EXCLUDE_DIFFUSE) && (path_flag & PATH_RAY_DIFFUSE)) ||
      ((shader & SHADER_EXCLUDE_GLOSSY) && ((path_flag & (PATH_RAY_GLOSSY | PATH_RAY_REFLECT)) ==
                                            (PATH_RAY_GLOSSY | PATH_RAY_REFLECT))) ||
      ((shader & SHADER_EXCLUDE_TRANSMIT) && (path_flag & PATH_RAY_TRANSMIT)) ||
      ((shader & SHADER_EXCLUDE_CAMERA) && (path_flag & PATH_RAY_CAMERA)) ||
      ((shader & SHADER_EXCLUDE_SCATTER) && (path_flag & PATH_RAY_VOLUME_SCATTER)))
  {
    return false;
  }

  return true;
}

/* Compute vector v as in "Importance Sampling of Many Lights with Adaptive Tree Splitting" Fig. 8.
 * P_v is the corresponding point along the ray. */
ccl_device float3 light_tree_v(
    const float3 centroid, const float3 P, const float3 D, const float3 bcone_axis, const float t)
{
  const float3 unnormalized_v0 = P - centroid;
  const float3 unnormalized_v1 = unnormalized_v0 + D * fminf(t, 1e12f);
  const float3 v0 = normalize(unnormalized_v0);
  const float3 v1 = normalize(unnormalized_v1);

  const float3 o0 = v0;
  float3 o1, o2;
  make_orthonormals_tangent(o0, v1, &o1, &o2);

  const float dot_o0_a = dot(o0, bcone_axis);
  const float dot_o1_a = dot(o1, bcone_axis);
  const float inv_len = inversesqrtf(sqr(dot_o0_a) + sqr(dot_o1_a));
  const float cos_phi0 = dot_o0_a * inv_len;

  return (dot_o1_a < 0 || dot(v0, v1) > cos_phi0) ? (dot_o0_a > dot(v1, bcone_axis) ? v0 : v1) :
                                                    cos_phi0 * o0 + dot_o1_a * inv_len * o1;
}

CCL_NAMESPACE_END
