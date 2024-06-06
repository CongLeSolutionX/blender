/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/light/common.h"

CCL_NAMESPACE_BEGIN

ccl_device_inline bool point_light_sample(const ccl_global KernelLight *klight,
                                          const float2 rand,
                                          const float3 P,
                                          const float3 N,
                                          const int shader_flags,
                                          ccl_private LightSample *ls)
{
  const float r_sq = sqr(klight->spot.radius);

  float3 lightN = P - klight->co;
  const float d_sq = len_squared(lightN);
  const float d = sqrtf(d_sq);

  ls->eval_fac = klight->spot.eval_fac;

  if (klight->spot.is_sphere) {
    /* Spherical light geometry. */
    if (d_sq > r_sq) {
      /* Outside sphere. */
      float cos_theta;
      const float one_minus_cos = sin_sqr_to_one_minus_cos(r_sq / d_sq);
      ls->D = sample_uniform_cone(-lightN / d, one_minus_cos, rand, &cos_theta, &ls->pdf);
    }
    else {
      /* Inside sphere. */
      const bool has_transmission = (shader_flags & SD_BSDF_HAS_TRANSMISSION);
      if (has_transmission) {
        ls->D = sample_uniform_sphere(rand);
        ls->pdf = M_1_2PI_F * 0.5f;
      }
      else {
        sample_cos_hemisphere(N, rand, &ls->D, &ls->pdf);
      }
    }

    /* Law of cosines. */
    const float d_cos_theta = -dot(lightN, ls->D);
    const float d_sin_theta_sq = len_squared(lightN + d_cos_theta * ls->D);
    ls->t = d_cos_theta - copysignf(safe_sqrtf(r_sq - d_sin_theta_sq), d_sq - r_sq);

    ls->P = P + ls->D * ls->t;
    ls->Ng = (ls->P - klight->co) / klight->spot.radius;
  }
  else {
    lightN /= d;
    /* Point light with ad-hoc radius based on oriented disk. */
    ls->P = klight->co;
    if (r_sq > 0.0f) {
      ls->P += disk_light_sample(lightN, rand) * klight->spot.radius;
    }

    ls->D = normalize_len(ls->P - P, &ls->t);
    ls->Ng = -ls->D;

    /* PDF. */
    const float invarea = (r_sq > 0.0f) ? 1.0f / (r_sq * M_PI_F) : 1.0f;
    ls->pdf = invarea * light_pdf_area_to_solid_angle(lightN, -ls->D, ls->t);
  }

  /* Texture coordinates. */
  const Transform itfm = klight->itfm;
  const float2 uv = map_to_sphere(transform_direction(&itfm, ls->Ng));
  /* NOTE: Return barycentric coordinates in the same notation as Embree and OptiX. */
  ls->u = uv.y;
  ls->v = 1.0f - uv.x - uv.y;

  return true;
}

ccl_device_forceinline float sphere_light_pdf(
    const float d_sq, const float r_sq, const float3 N, const float3 D, const uint32_t path_flag)
{
  if (d_sq > r_sq) {
    return M_1_2PI_F / sin_sqr_to_one_minus_cos(r_sq / d_sq);
  }

  const bool has_transmission = (path_flag & PATH_RAY_MIS_HAD_TRANSMISSION);
  return has_transmission ? M_1_2PI_F * 0.5f : pdf_cos_hemisphere(N, D);
}

ccl_device_forceinline void point_light_mnee_sample_update(const ccl_global KernelLight *klight,
                                                           ccl_private LightSample *ls,
                                                           const float3 P,
                                                           const float3 N,
                                                           const uint32_t path_flag)
{
  ls->D = normalize_len(ls->P - P, &ls->t);

  const float radius = klight->spot.radius;

  if (klight->spot.is_sphere) {
    const float d_sq = len_squared(P - klight->co);
    const float r_sq = sqr(radius);
    const float t_sq = sqr(ls->t);

    /* NOTE : preserve pdf in area measure. */
    const float jacobian_solid_angle_to_area = 0.5f * fabsf(d_sq - r_sq - t_sq) /
                                               (radius * ls->t * t_sq);
    ls->pdf = sphere_light_pdf(d_sq, r_sq, N, ls->D, path_flag) * jacobian_solid_angle_to_area;

    ls->Ng = normalize(ls->P - klight->co);
  }
  else {
    /* NOTE : preserve pdf in area measure. */
    ls->pdf = ls->eval_fac * 4.0f * M_PI_F;

    ls->Ng = -ls->D;
  }

  /* Texture coordinates. */
  const Transform itfm = klight->itfm;
  const float2 uv = map_to_sphere(transform_direction(&itfm, ls->Ng));
  /* NOTE: Return barycentric coordinates in the same notation as Embree and OptiX. */
  ls->u = uv.y;
  ls->v = 1.0f - uv.x - uv.y;
}

ccl_device_inline bool point_light_intersect(const ccl_global KernelLight *klight,
                                             const ccl_private Ray *ccl_restrict ray,
                                             ccl_private float *t)
{
  const float radius = klight->spot.radius;
  if (radius == 0.0f) {
    return false;
  }

  if (klight->spot.is_sphere) {
    float3 P;
    return ray_sphere_intersect(ray->P, ray->D, ray->tmin, ray->tmax, klight->co, radius, &P, t);
  }
  else {
    float3 P;
    const float3 diskN = normalize(ray->P - klight->co);
    return ray_disk_intersect(
        ray->P, ray->D, ray->tmin, ray->tmax, klight->co, diskN, radius, &P, t);
  }
}

/* TODO(weizhen): `pdf` is solely used as an indicator if the sample is successful. Check if the
 * exact value is actually needed. */
ccl_device_inline void point_light_sample_from_uv(const ccl_global KernelLight *klight,
                                                  const float3 ray_P,
                                                  ccl_private LightSample *ccl_restrict ls)
{
  ls->eval_fac = klight->spot.eval_fac;
  const float radius = klight->spot.radius;
  if (radius == 0.0f) {
    ls->P = klight->co;
    ls->D = normalize_len(ls->P - ray_P, &ls->t);
    ls->Ng = -ls->D;
    return;
  }

  const Transform tfm = klight->tfm;
  const float2 uv = make_float2(1.0f - ls->u - ls->v, ls->u);
  ls->Ng = transform_direction(&tfm, map_to_sphere(uv));

  if (klight->spot.is_sphere) {
    ls->P = klight->co + radius * ls->Ng;
    ls->D = normalize_len(ls->P - ray_P, &ls->t);
    /* TODO(weizhen): what if uv is not visible from the new point? */
  }
  else {
    ls->D = -ls->Ng;
    if (!ray_aligned_disk_intersect(
            ray_P, ls->D, 0.0f, FLT_MAX, klight->co, radius, &ls->P, &ls->t))
    {
      ls->pdf = 0.0f;
    }
  }
}

ccl_device_inline bool point_light_sample_from_intersection(const ccl_global KernelLight *klight,
                                                            const float3 ray_P,
                                                            const float3 ray_D,
                                                            const float3 N,
                                                            const uint32_t path_flag,
                                                            ccl_private LightSample *ccl_restrict
                                                                ls)
{
  const float r_sq = sqr(klight->spot.radius);

  ls->eval_fac = klight->spot.eval_fac;

  if (klight->spot.is_sphere) {
    const float d_sq = len_squared(ray_P - klight->co);
    ls->pdf = sphere_light_pdf(d_sq, r_sq, N, ray_D, path_flag);
    ls->Ng = normalize(ls->P - klight->co);
  }
  else {
    if (ls->t != FLT_MAX) {
      const float3 lightN = normalize(ray_P - klight->co);
      const float invarea = (r_sq > 0.0f) ? 1.0f / (r_sq * M_PI_F) : 1.0f;
      ls->pdf = invarea * light_pdf_area_to_solid_angle(lightN, -ray_D, ls->t);
    }
    else {
      ls->pdf = 0.0f;
    }
    ls->Ng = -ray_D;
  }

  /* Texture coordinates. */
  const Transform itfm = klight->itfm;
  const float2 uv = map_to_sphere(transform_direction(&itfm, ls->Ng));
  /* NOTE: Return barycentric coordinates in the same notation as Embree and OptiX. */
  ls->u = uv.y;
  ls->v = 1.0f - uv.x - uv.y;

  return true;
}

template<bool in_volume_segment>
ccl_device_forceinline bool point_light_tree_parameters(const ccl_global KernelLight *klight,
                                                        const float3 centroid,
                                                        const float3 P,
                                                        ccl_private float &cos_theta_u,
                                                        ccl_private float2 &distance,
                                                        ccl_private float3 &point_to_centroid)
{
  float min_distance;
  point_to_centroid = safe_normalize_len(centroid - P, &min_distance);
  distance = min_distance * one_float2();

  if (in_volume_segment) {
    cos_theta_u = 1.0f; /* Any value in [-1, 1], irrelevant since theta = 0 */
    return true;
  }

  const float radius = klight->spot.radius;

  if (klight->spot.is_sphere) {
    if (min_distance > radius) {
      /* Equivalent to a disk light with the same angular span. */
      cos_theta_u = cos_from_sin(radius / min_distance);
      distance.x = min_distance / cos_theta_u;
    }
    else {
      /* Similar to background light. */
      cos_theta_u = -1.0f;
      /* HACK: pack radiance scaling in the distance. */
      distance = one_float2() * radius / M_SQRT2_F;
    }
  }
  else {
    const float hypotenus = sqrtf(sqr(radius) + sqr(min_distance));
    cos_theta_u = min_distance / hypotenus;

    distance.x = hypotenus;
  }

  return true;
}

CCL_NAMESPACE_END
