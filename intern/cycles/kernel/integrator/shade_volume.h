/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/film/data_passes.h"
#include "kernel/film/denoising_passes.h"
#include "kernel/film/light_passes.h"

#include "kernel/integrator/guiding.h"
#include "kernel/integrator/intersect_closest.h"
#include "kernel/integrator/path_state.h"
#include "kernel/integrator/shadow_linking.h"
#include "kernel/integrator/volume_shader.h"
#include "kernel/integrator/volume_stack.h"

#include "kernel/light/light.h"
#include "kernel/light/sample.h"
#include "kernel/sample/lcg.h"

CCL_NAMESPACE_BEGIN

#ifdef __VOLUME__

/* Events for probabilistic scattering. */

typedef enum VolumeIntegrateEvent {
  VOLUME_PATH_SCATTERED = 0,
  VOLUME_PATH_ATTENUATED = 1,
  VOLUME_PATH_MISSED = 2
} VolumeIntegrateEvent;

typedef struct VolumeIntegrateResult {
  /* Throughput and offset for direct light scattering. */
  bool direct_scatter;
  Spectrum direct_throughput;
  float direct_t;
  ShaderVolumePhases direct_phases;
#  ifdef __PATH_GUIDING__
  VolumeSampleMethod direct_sample_method;
#  endif

  /* Throughput and offset for indirect light scattering. */
  bool indirect_scatter;
  Spectrum indirect_throughput;
  float indirect_t;
  ShaderVolumePhases indirect_phases;

  Spectrum emission;
} VolumeIntegrateResult;

/* Ignore paths that have volume throughput below this value, to avoid unnecessary work
 * and precision issues.
 * todo: this value could be tweaked or turned into a probability to avoid unnecessary
 * work in volumes and subsurface scattering. */
#  define VOLUME_THROUGHPUT_LOG_EPSILON -13.815510558f /* log(1e-6) */

/* Volume shader properties
 *
 * extinction coefficient = absorption coefficient + scattering coefficient
 * sigma_t = sigma_a + sigma_s */

typedef struct VolumeShaderCoefficients {
  Spectrum sigma_t;
  Spectrum sigma_s;
  Spectrum emission;
} VolumeShaderCoefficients;

typedef struct EquiangularCoefficients {
  float3 P;
  Interval<float> t_range;
} EquiangularCoefficients;

/* Evaluate shader to get extinction coefficient at P. */
template<const bool shadow, typename ConstIntegratorGenericState>
ccl_device_inline Spectrum volume_shader_eval_extinction(KernelGlobals kg,
                                                         ConstIntegratorGenericState state,
                                                         const uint32_t path_flag,
                                                         ccl_private ShaderData *ccl_restrict sd)
{
  if constexpr (shadow) {
    VOLUME_READ_LAMBDA(integrator_state_read_shadow_volume_stack(state, i))
    volume_shader_eval<shadow>(kg, state, sd, path_flag, volume_read_lambda_pass);
  }
  else {
    VOLUME_READ_LAMBDA(integrator_state_read_volume_stack(state, i))
    volume_shader_eval<shadow>(kg, state, sd, path_flag, volume_read_lambda_pass);
  }

  if (!(sd->flag & (SD_EXTINCTION))) {
    return zero_spectrum();
  }

  return sd->closure_transparent_extinction;
}

/* Evaluate shader to get absorption, scattering and emission at P. */
ccl_device_inline bool volume_shader_sample(KernelGlobals kg,
                                            IntegratorState state,
                                            ccl_private ShaderData *ccl_restrict sd,
                                            ccl_private VolumeShaderCoefficients *coeff)
{
  const uint32_t path_flag = INTEGRATOR_STATE(state, path, flag);
  VOLUME_READ_LAMBDA(integrator_state_read_volume_stack(state, i))
  volume_shader_eval<false>(kg, state, sd, path_flag, volume_read_lambda_pass);

  if (!(sd->flag & (SD_EXTINCTION | SD_SCATTER | SD_EMISSION))) {
    return false;
  }

  coeff->sigma_s = zero_spectrum();
  coeff->sigma_t = (sd->flag & SD_EXTINCTION) ? sd->closure_transparent_extinction :
                                                zero_spectrum();
  coeff->emission = (sd->flag & SD_EMISSION) ? sd->closure_emission_background : zero_spectrum();

  if (sd->flag & SD_SCATTER) {
    for (int i = 0; i < sd->num_closure; i++) {
      ccl_private const ShaderClosure *sc = &sd->closure[i];

      if (CLOSURE_IS_VOLUME(sc->type)) {
        coeff->sigma_s += sc->weight;
      }
    }
  }

  return true;
}

ccl_device_forceinline void volume_step_init(KernelGlobals kg,
                                             ccl_private const RNGState *rng_state,
                                             const float object_step_size,
                                             const float tmin,
                                             const float tmax,
                                             ccl_private float *step_size,
                                             ccl_private float *step_shade_offset,
                                             ccl_private float *steps_offset,
                                             ccl_private int *max_steps)
{
  if (object_step_size == FLT_MAX) {
    /* Homogeneous volume. */
    *step_size = tmax - tmin;
    *step_shade_offset = 0.0f;
    *steps_offset = 1.0f;
    *max_steps = 1;
  }
  else {
    /* Heterogeneous volume. */
    const float t = tmax - tmin;

    *max_steps = kernel_data.integrator.volume_max_steps;
    const float max_step_size = min(object_step_size, t);
    if (t <= *max_steps * max_step_size) {
      /* Reduce number of steps if possible. */
      /* TODO(weizhen): for some reason t can be zero. Check why this happens. */
      *max_steps = max(1, int(ceilf(t / max_step_size)));
    }

    *step_size = t / (float)*max_steps;

    /* Perform shading at this offset within a step, to integrate over
     * over the entire step segment. */
    *step_shade_offset = path_state_rng_1D(kg, rng_state, PRNG_VOLUME_SHADE_OFFSET);

    /* Shift starting point of all segment by this random amount to avoid
     * banding artifacts from the volume bounding shape. */
    *steps_offset = path_state_rng_1D(kg, rng_state, PRNG_VOLUME_OFFSET);
  }
}

/* Volume Shadows
 *
 * These functions are used to attenuate shadow rays to lights. Both absorption
 * and scattering will block light, represented by the extinction coefficient. */

#  if 0
/* homogeneous volume: assume shader evaluation at the starts gives
 * the extinction coefficient for the entire line segment */
ccl_device void volume_shadow_homogeneous(KernelGlobals kg, IntegratorState state,
                                          ccl_private Ray *ccl_restrict ray,
                                          ccl_private ShaderData *ccl_restrict sd,
                                          ccl_global Spectrum *ccl_restrict throughput)
{
  Spectrum sigma_t = zero_spectrum();

  if (shadow_volume_shader_sample(kg, state, sd, &sigma_t)) {
    *throughput *= volume_color_transmittance(sigma_t, ray->tmax - ray->tmin);
  }
}
#  endif

/* --------------------------------------------------------------------
 * Volume transmittance estimation based on "An unbiased ray-marching transmittance estimator" by
 * Kettunen et al.
 * https://research.nvidia.com/publication/2021-06_unbiased-ray-marching-transmittance-estimator
 */

/* TODO(weizhen): find a reasonable threshold for cut off, or use the recursive formulation from
 * Georgiev et al. 2019 instead. */
#  ifdef __KERNEL_GPU__
/* For each ray, the probability of evaluating at least 5 orders is 0.1 * 2^4 / 5! = 0.01333. GPU
 * seems to have problems with too many recursions. */
#    define VOLUME_EXPANSION_ORDER_CUTOFF 5
#  else
/* For each ray, the probability of evaluating at least 7 orders is 0.1* 2^6 / 7! = 0.00127. */
#    define VOLUME_EXPANSION_ORDER_CUTOFF 7
#  endif

/* TODO(weizhen): compute volume majorant and minorant. If the value is too off it doesn't seem to
 * converge. Maybe use Spectrum instead of float. */
#  define MAJORANT 31.0f
#  define MINORANT 0.0f

/* For delta-tracking it seems benefitial to increase majorant. */
#  define DELTA_TRACKING_SCALE 1.0f

/**
 * Compute elementary symmetric means from X[0,...,N] - X[i], skipping X[i].
 *
 * \param N: expansion order
 * \param i: index of the expansion pivot (X[i])
 * \param X: samples to compute means from
 * \return elementary symmetric means m0, ..., mN
 *
 * See [Kettunen et al. 2021] Algorithm 1.
 */
ccl_device void volume_elementary_mean(const int N,
                                       const int i,
                                       const ccl_private Spectrum *X,
                                       ccl_private Spectrum *m)
{
  /* Initialization. */
  m[0] = one_spectrum();
  for (int n = 1; n <= N; n++) {
    m[n] = zero_spectrum();
  }

  /* Recursively compute elementary means. */
  for (int n = 1; n <= N; n++) {
    const Spectrum x = X[(n + i) % (N + 1)] - X[i];
    for (int k = n; k >= 1; k--) {
      m[k] += float(k) / float(n) * (m[k - 1] * x - m[k]);
    }
  }
}

/* Randomly sample expansion order N. See [Kettunen et al. 2021] Algorithm 2. */
/* TODO(weizhen): detect homogeneous media? */
ccl_device int volume_aggressive_BK_roulette(KernelGlobals kg, float rand)
{
  float continuation_probability = 0.1f;
  if (rand > continuation_probability) {
    /* Stop at the zeroth-order term. */
    return 0;
  }

  /* Rescale random number for reusing. */
  rand /= continuation_probability;

  /* Bhanot & Kennedy roulette with K = c = 2. */
  /* TODO(weizhen): maybe manual cut off to prevent infinite loop. */
  for (int k = 2 + 1;; k++) {
    /* Update the probability of sampling at least order k. */
    continuation_probability *= 2.0f / k;
    if (rand > continuation_probability || k > VOLUME_EXPANSION_ORDER_CUTOFF) {
      return k - 1;
    }

    /* Rescale random number for reusing. */
    rand /= continuation_probability;
  }
}

/**
 * Automatically compute tuple size based on the volume density.
 *
 * \param tau: control optical thickness
 * \return number of ray marching sections along the ray
 *
 * See [Kettunen et al. 2021] Algorithm 4.
 */
/* TODO(weizhen): this formula is purely for matching the cost of [Georgiev et al. 2019]. Probably
 * it's not needed, and we can use the expectation of the number of events `tau`? */
ccl_device int volume_tuple_size(const float tau)
{
  const float N_CMF = ceilf(cbrtf((0.015f + tau) * (0.65f + tau) * (60.3f + tau)));
  return max(1, int(floorf(N_CMF / 1.31945f + 0.5f)));
}

/**
 * Compute transmittance along the ray between [tmin, tmax]
 *
 * /param sigma: extinction control. Recommend using the difference between the majorant and
 * minorant extinction when a minorant is available
 * /param rand: random number \in [0, 1] for sampling expansion order
 * /param lcg_state: random number generator for sampling shading offset
 * /return transmittance along the ray
 *
 * See [Kettunen et al. 2021] Algorithm 5.
 */
template<const bool shadow, typename ConstIntegratorGenericState>
ccl_device Spectrum volume_unbiased_ray_marching(KernelGlobals kg,
                                                 ConstIntegratorGenericState state,
                                                 const ccl_private Ray *ccl_restrict ray,
                                                 ccl_private ShaderData *ccl_restrict sd,
                                                 const uint32_t path_flag,
                                                 const float tmin,
                                                 const float tmax,
                                                 const float sigma,
                                                 const float rand,
                                                 ccl_private uint &lcg_state,
                                                 ccl_private Spectrum *biased_tau = nullptr)
{
  /* Compute tuple size and expansion order. */
  const float ray_length = tmax - tmin;
  /* TODO(weizhen): check if `ray->tmax == FLT_MAX` is correctly handled. */
  const int M = min(volume_tuple_size(sigma * ray_length),
                    kernel_data.integrator.volume_max_steps);
  const float step_size = ray_length / M;
  /* TODO(weizhen): this implementation does not properly balance the workloads of the
   * transmittance estimates inside the thread groups and causes slowdowns on GPU. */
  const int N = volume_aggressive_BK_roulette(kg, rand);

  /* Combed estimators of the optical thickness. */
  Spectrum X[VOLUME_EXPANSION_ORDER_CUTOFF + 1];
  int i = 0;
  if (biased_tau) {
    X[0] = *biased_tau;
    i = 1;
  }
  for (; i <= N; i++) {
    X[i] = zero_spectrum();
    const float step_shade_offset = lcg_step_float(&lcg_state);
    for (int j = 0; j < M; j++) {
      /* Advance to new position. */
      const float t = min(tmax, tmin + (step_shade_offset + j) * step_size);
      sd->P = ray->P + ray->D * t;

      /* TODO(weizhen): early cut off? */
      X[i] += volume_shader_eval_extinction<shadow>(kg, state, path_flag, sd);
    }
    X[i] = -step_size * X[i];
  }

  Spectrum transmittance = zero_spectrum();
  for (int i = 0; i <= N; i++) {
    /* TODO(weizhen): is this zero initialization? */
    Spectrum elementary_mean[VOLUME_EXPANSION_ORDER_CUTOFF + 1] = {};
    volume_elementary_mean(N, i, X, elementary_mean);

    /* Compute Taylor expansion of T = e^x at X[i]. */
    Spectrum fN = one_spectrum();
    float weight = 20.0f;
    for (int k = 1; k <= N; k++) {
      /* weight = 1 / (k!Pk) = 1 / (k! * (0.1 * 2^(k - 1) / k!)) = 10 / 2^(k - 1). */
      weight /= 2.0f;
      fN += weight * elementary_mean[k];
    }
    transmittance += exp(X[i]) * fN;
  }

  return transmittance / (N + 1);
}

/* heterogeneous volume: integrate stepping through the volume until we
 * reach the end, get absorbed entirely, or run out of iterations */
ccl_device void volume_shadow_heterogeneous(KernelGlobals kg,
                                            IntegratorShadowState state,
                                            ccl_private Ray *ccl_restrict ray,
                                            ccl_private ShaderData *ccl_restrict sd,
                                            ccl_private Spectrum *ccl_restrict throughput,
                                            const float object_step_size)
{
  /* Load random number state. */
  RNGState rng_state;
  shadow_path_state_rng_load(state, &rng_state);

#  if 0
  /* Prepare for stepping.
   * For shadows we do not offset all segments, since the starting point is
   * already a random distance inside the volume. It also appears to create
   * banding artifacts for unknown reasons. */
  int M;
  float step_size, unused;
  volume_step_init(
      kg, &rng_state, object_step_size, ray->tmin, ray->tmax, &step_size, &unused, &unused, &M);
#  endif

  const float sigma = MAJORANT - MINORANT;
  const float rand = path_state_rng_3D(kg, &rng_state, PRNG_VOLUME_TAYLOR_EXPANSION).z;

  /* TODO(volume): use stratified samples, at least for the first few orders. */
  uint lcg_state = lcg_state_init(INTEGRATOR_STATE(state, shadow_path, rng_pixel),
                                  INTEGRATOR_STATE(state, shadow_path, rng_offset),
                                  INTEGRATOR_STATE(state, shadow_path, sample),
                                  0x8647ace4);

  *throughput *= volume_unbiased_ray_marching<true>(
      kg, state, ray, sd, PATH_RAY_SHADOW, ray->tmin, ray->tmax, sigma, rand, lcg_state);
}

/* Equi-angular sampling as in:
 * "Importance Sampling Techniques for Path Tracing in Participating Media" */

/* Below this pdf we ignore samples, as they tend to lead to very long distances.
 * This can cause performance issues with BVH traversal in OptiX, leading it to
 * traverse many nodes. Since these contribute very little to the image, just ignore
 * those samples. */
#  define VOLUME_SAMPLE_PDF_CUTOFF 1e-8f

ccl_device float volume_equiangular_sample(ccl_private const Ray *ccl_restrict ray,
                                           ccl_private const EquiangularCoefficients &coeffs,
                                           const float xi,
                                           ccl_private float *pdf)
{
  const float delta = dot((coeffs.P - ray->P), ray->D);
  const float D = safe_sqrtf(len_squared(coeffs.P - ray->P) - delta * delta);
  if (UNLIKELY(D == 0.0f)) {
    *pdf = 0.0f;
    return 0.0f;
  }
  const float tmin = coeffs.t_range.min;
  const float tmax = coeffs.t_range.max;
  const float theta_a = atan2f(tmin - delta, D);
  const float theta_b = atan2f(tmax - delta, D);
  const float t_ = D * tanf((xi * theta_b) + (1 - xi) * theta_a);
  if (UNLIKELY(theta_b == theta_a)) {
    *pdf = 0.0f;
    return 0.0f;
  }
  *pdf = D / ((theta_b - theta_a) * (D * D + t_ * t_));

  return clamp(delta + t_, tmin, tmax); /* clamp is only for float precision errors */
}

ccl_device float volume_equiangular_pdf(ccl_private const Ray *ccl_restrict ray,
                                        ccl_private const EquiangularCoefficients &coeffs,
                                        const float sample_t)
{
  const float delta = dot((coeffs.P - ray->P), ray->D);
  const float D = safe_sqrtf(len_squared(coeffs.P - ray->P) - delta * delta);
  if (UNLIKELY(D == 0.0f)) {
    return 0.0f;
  }

  const float tmin = coeffs.t_range.min;
  const float tmax = coeffs.t_range.max;
  const float t_ = sample_t - delta;

  const float theta_a = atan2f(tmin - delta, D);
  const float theta_b = atan2f(tmax - delta, D);
  if (UNLIKELY(theta_b == theta_a)) {
    return 0.0f;
  }

  const float pdf = D / ((theta_b - theta_a) * (D * D + t_ * t_));

  return pdf;
}

ccl_device_inline bool volume_equiangular_valid_ray_segment(KernelGlobals kg,
                                                            const float3 ray_P,
                                                            const float3 ray_D,
                                                            ccl_private Interval<float> *t_range,
                                                            const ccl_private LightSample *ls)
{
  if (ls->type == LIGHT_SPOT) {
    ccl_global const KernelLight *klight = &kernel_data_fetch(lights, ls->lamp);
    return spot_light_valid_ray_segment(klight, ray_P, ray_D, t_range);
  }
  if (ls->type == LIGHT_AREA) {
    ccl_global const KernelLight *klight = &kernel_data_fetch(lights, ls->lamp);
    return area_light_valid_ray_segment(&klight->area, ray_P - klight->co, ray_D, t_range);
  }
  if (ls->type == LIGHT_TRIANGLE) {
    return triangle_light_valid_ray_segment(kg, ray_P - ls->P, ray_D, t_range, ls);
  }

  /* Point light, the whole range of the ray is visible. */
  kernel_assert(ls->type == LIGHT_POINT);
  return true;
}

/* --------------------------------------------------------------------
 * Tracking-based distance sampling
 */

/**
 * Sample distance along the ray based on weighted delta tracking
 *
 * /param rand: random number \in [0, 1] for sampling absorption, scatter or continue (null
 * scatter) events
 * /param lcg_state: random number generator for sampling distance
 * /return true if successfully sampled a scatter event
 */
ccl_device bool volume_distance_sample(KernelGlobals kg,
                                       const IntegratorState state,
                                       const ccl_private Ray *ccl_restrict ray,
                                       ccl_private ShaderData *ccl_restrict sd,
                                       float rand,
                                       ccl_private uint &lcg_state,
                                       ccl_private VolumeIntegrateResult &result,
                                       ccl_private Spectrum &distance_pdf,
                                       ccl_private Spectrum &tau)
{
  /* Initialization */
  const Spectrum sigma_maj = make_float3(DELTA_TRACKING_SCALE * MAJORANT);
  const float inv_maj = 1.0f / (MAJORANT * DELTA_TRACKING_SCALE);
  float t = ray->tmin + sample_exponential_distribution(lcg_step_float(&lcg_state), inv_maj);
  int num_samples = 0;
  result.emission = zero_spectrum();
  Spectrum transmittance = one_spectrum();
  while (t < ray->tmax) {
    sd->P = ray->P + ray->D * t;
    num_samples++;

    VolumeShaderCoefficients coeff ccl_optional_struct_init;

    if (volume_shader_sample(kg, state, sd, &coeff)) {
      tau -= coeff.sigma_t;
      const Spectrum sigma_n = sigma_maj - coeff.sigma_t;
      /* For procedure volumes `sigma_maj` might not be the strict upper bound. Use absolute value
       * to handle negative null scattering coefficient as suggested in "Monte Carlo Methods for
       * Volumetric Light Transport Simulation". */
      const Spectrum abs_sigma_n = fabs(sigma_n);
      /* We do not sample absorption event, because it always returns zero and has high variance.
       * Instead, we adjust the sampling weight. */
      Spectrum sigma_c = coeff.sigma_s + abs_sigma_n;

      if (sd->flag & SD_EMISSION) {
        result.emission += transmittance * coeff.emission * inv_maj;
      }

      if (reduce_add(sigma_c) == 0.0f) {
        /* Absorption only. */
        /* TODO(weizhen): this has high variance. Can we use next-flight emstimation? */
        result.indirect_throughput = zero_spectrum();
        return false;
      }

      /* Pick random color channel, we use the Veach one-sample model with balance heuristic for
       * the channels. */
      const Spectrum albedo = safe_divide_color(coeff.sigma_s, coeff.sigma_t);
      Spectrum channel_pdf;
      const int channel = volume_sample_channel(
          albedo, result.indirect_throughput * transmittance, &rand, &channel_pdf);

      rand *= sigma_c[channel];
      if (rand < coeff.sigma_s[channel]) {
        /* Sampled scatter event. */
        result.indirect_t = t;

        const Spectrum pdf_s = coeff.sigma_s / sigma_c;
        transmittance *= coeff.sigma_s * inv_maj / dot(channel_pdf, pdf_s);
        result.indirect_throughput *= transmittance;

        volume_shader_copy_phases(&result.indirect_phases, sd);

        distance_pdf = coeff.sigma_s;
        tau *= (t - ray->tmin) / num_samples;
        return true;
      }

      /* Null scattering. Accumulate weight and continue. */
      const Spectrum pdf_n = abs_sigma_n / sigma_c;
      transmittance *= sigma_n * inv_maj / dot(channel_pdf, pdf_n);

      /* Rescale random number for reusing. */
      rand = (rand - coeff.sigma_s[channel]) / abs_sigma_n[channel];
    }

    /* Generate the next distance using random walk. */
    /* TODO(weizhen): this has variance even with monochromatic volume, seems like ratio tracking
     * needs higher majorant. Use combed estimation ([Kettunen et al. 2021] page 16 bottom), or
     * reuse the estimation from equiangular transmittance, or reduce lookups. */
    t += sample_exponential_distribution(lcg_step_float(&lcg_state), inv_maj);
  }

  /* No scatter event sampled along the ray. */
  result.indirect_throughput *= transmittance;
  return false;
}

/* Emission */

ccl_device Spectrum volume_emission_integrate(ccl_private VolumeShaderCoefficients *coeff,
                                              int closure_flag,
                                              Spectrum transmittance,
                                              float t)
{
  /* integral E * exp(-sigma_t * t) from 0 to t = E * (1 - exp(-sigma_t * t))/sigma_t
   * this goes to E * t as sigma_t goes to zero
   *
   * todo: we should use an epsilon to avoid precision issues near zero sigma_t */
  Spectrum emission = coeff->emission;

  if (closure_flag & SD_EXTINCTION) {
    Spectrum sigma_t = coeff->sigma_t;

    FOREACH_SPECTRUM_CHANNEL (i) {
      GET_SPECTRUM_CHANNEL(emission, i) *= (GET_SPECTRUM_CHANNEL(sigma_t, i) > 0.0f) ?
                                               (1.0f - GET_SPECTRUM_CHANNEL(transmittance, i)) /
                                                   GET_SPECTRUM_CHANNEL(sigma_t, i) :
                                               t;
    }
  }
  else {
    emission *= t;
  }

  return emission;
}

/* Volume Integration */

typedef struct VolumeIntegrateState {
  /* Current active segment. */
  Interval<float> t;

  /* If volume is absorption-only up to this point, and no probabilistic
   * scattering or termination has been used yet. */
  /* TODO(weizhen): doesn't seem to be used. */
  bool absorption_only;

  /* Random numbers for scattering. */
  float rscatter;
  float rchannel;

  /* Multiple importance sampling. */
  VolumeSampleMethod direct_sample_method;
  bool use_mis;
  Spectrum distance_pdf;
  float equiangular_pdf;
} VolumeIntegrateState;

/* heterogeneous volume distance sampling: integrate stepping through the
 * volume until we reach the end, get absorbed entirely, or run out of
 * iterations. this does probabilistically scatter or get transmitted through
 * for path tracing where we don't want to branch. */
ccl_device_forceinline void volume_integrate_heterogeneous(
    KernelGlobals kg,
    IntegratorState state,
    ccl_private Ray *ccl_restrict ray,
    ccl_private ShaderData *ccl_restrict sd,
    ccl_private const RNGState *rng_state,
    ccl_global float *ccl_restrict render_buffer,
    const float object_step_size,
    const VolumeSampleMethod direct_sample_method,
    ccl_private const EquiangularCoefficients &equiangular_coeffs,
    ccl_private VolumeIntegrateResult &result)
{
  PROFILING_INIT(kg, PROFILING_SHADE_VOLUME_INTEGRATE);

  /* Initialize volume integration state. */
  VolumeIntegrateState vstate ccl_optional_struct_init;
  {
    const float rand = path_state_rng_3D(kg, rng_state, PRNG_VOLUME_SCATTER_DISTANCE).x;
    /* Multiple importance sampling: pick between equiangular and distance sampling strategy. */
    vstate.direct_sample_method = direct_sample_method;
    vstate.use_mis = (direct_sample_method == VOLUME_SAMPLE_MIS);
    if (vstate.use_mis) {
      vstate.direct_sample_method = (rand < 0.5f) ? VOLUME_SAMPLE_DISTANCE :
                                                    VOLUME_SAMPLE_EQUIANGULAR;
    }
    vstate.equiangular_pdf = 0.0f;
    vstate.distance_pdf = one_spectrum();
  }

  const float rand_equiangular = path_state_rng_3D(kg, rng_state, PRNG_VOLUME_SCATTER_DISTANCE).y;

  /* Equiangular sampling: compute distance and PDF in advance. */
  result.direct_t = volume_equiangular_sample(
      ray, equiangular_coeffs, rand_equiangular, &vstate.equiangular_pdf);

  /* Initialize volume integration result. */
  const Spectrum throughput = INTEGRATOR_STATE(state, path, throughput);
  result.direct_throughput = throughput;
  result.indirect_throughput = throughput;

  /* TODO(volume): use stratified samples by scrambing `rng_offset`. See subsurface scattering. */
  uint lcg_state = lcg_state_init(INTEGRATOR_STATE(state, path, rng_pixel),
                                  INTEGRATOR_STATE(state, path, rng_offset),
                                  INTEGRATOR_STATE(state, path, sample),
                                  0xe35fad82);

  /* Biased estimation of optical thickness until indirect scatter point. */
  Spectrum tau = zero_spectrum();
  {
    /* Indirect scatter. */
    const float rand = path_state_rng_3D(kg, rng_state, PRNG_VOLUME_SCATTER_DISTANCE).z;
    result.indirect_scatter = volume_distance_sample(
        kg, state, ray, sd, rand, lcg_state, result, vstate.distance_pdf, tau);
  }

  /* Direct_scatter. */
  if (vstate.direct_sample_method == VOLUME_SAMPLE_DISTANCE && result.indirect_scatter) {
    /* If using distance sampling for direct light, just copy parameters of indirect light since we
     * scatter at the same point. */
    result.direct_scatter = true;
    result.direct_t = result.indirect_t;
    result.direct_throughput = result.indirect_throughput;

    volume_shader_copy_phases(&result.direct_phases, sd);

    if (vstate.use_mis) {
      const uint32_t path_flag = INTEGRATOR_STATE(state, path, flag);
      const float sigma = MAJORANT - MINORANT;
      const float rand = path_state_rng_3D(kg, rng_state, PRNG_VOLUME_TAYLOR_EXPANSION).y;
      vstate.distance_pdf *= volume_unbiased_ray_marching<false>(
          kg, state, ray, sd, path_flag, ray->tmin, result.direct_t, sigma, rand, lcg_state, &tau);

      const float equiangular_pdf = volume_equiangular_pdf(
          ray, equiangular_coeffs, result.direct_t);

      const Spectrum mis_weight = power_heuristic(vstate.distance_pdf, equiangular_pdf);
      result.direct_throughput *= 2.0f * mis_weight;
    }
  }

  /* TODO(weizhen): indirect transmission is computed in `volume_distance_sample()`, but maybe the
   * variance is high? Try to estimate with unbiased ray marching. */

  /* Direct scatter. */
  if (vstate.direct_sample_method == VOLUME_SAMPLE_EQUIANGULAR && vstate.equiangular_pdf != 0.0f) {
    sd->P = ray->P + ray->D * result.direct_t;
    VolumeShaderCoefficients coeff ccl_optional_struct_init;
    if (volume_shader_sample(kg, state, sd, &coeff) && (sd->flag & SD_SCATTER)) {
      result.direct_scatter = true;
      result.direct_throughput *= coeff.sigma_s / vstate.equiangular_pdf;

      volume_shader_copy_phases(&result.direct_phases, sd);

      /* Compute transmission until direct scatter position. */
      const uint32_t path_flag = INTEGRATOR_STATE(state, path, flag);
      const float sigma = MAJORANT - MINORANT;
      const float rand = path_state_rng_3D(kg, rng_state, PRNG_VOLUME_TAYLOR_EXPANSION).x;
      const Spectrum transmittance = volume_unbiased_ray_marching<false>(
          kg, state, ray, sd, path_flag, ray->tmin, result.direct_t, sigma, rand, lcg_state);
      result.direct_throughput *= transmittance;

      if (vstate.use_mis) {
        vstate.distance_pdf = coeff.sigma_s * transmittance;
        const Spectrum mis_weight = power_heuristic(vstate.equiangular_pdf, vstate.distance_pdf);
        result.direct_throughput *= 2.0f * mis_weight;
      }
    }
  }

  /* Write accumulated emission. */
  if (!is_zero(result.emission)) {
    if (light_link_object_match(kg, light_link_receiver_forward(kg, state), sd->object)) {
      result.emission *= throughput;
      film_write_volume_emission(
          kg, state, result.emission, render_buffer, object_lightgroup(kg, sd->object));
    }
  }

  /* TODO(weizhen): Write emission for path guiding. */
  /* TODO(weizhen): Accumulate albedo for denoising features. */
}

/* Path tracing: sample point on light for equiangular sampling. */
ccl_device_forceinline bool integrate_volume_equiangular_sample_light(
    KernelGlobals kg,
    IntegratorState state,
    ccl_private const Ray *ccl_restrict ray,
    ccl_private const ShaderData *ccl_restrict sd,
    ccl_private const RNGState *ccl_restrict rng_state,
    ccl_private EquiangularCoefficients *ccl_restrict equiangular_coeffs,
    ccl_private LightSample &ccl_restrict ls)
{
  /* Test if there is a light or BSDF that needs direct light. */
  if (!kernel_data.integrator.use_direct_light) {
    return false;
  }

  /* Sample position on a light. */
  const uint32_t path_flag = INTEGRATOR_STATE(state, path, flag);
  const uint bounce = INTEGRATOR_STATE(state, path, bounce);
  const float3 rand_light = path_state_rng_3D(kg, rng_state, PRNG_LIGHT);

  if (!light_sample_from_volume_segment(kg,
                                        rand_light,
                                        sd->time,
                                        sd->P,
                                        ray->D,
                                        ray->tmax - ray->tmin,
                                        light_link_receiver_nee(kg, sd),
                                        bounce,
                                        path_flag,
                                        &ls))
  {
    ls.emitter_id = EMITTER_NONE;
    return false;
  }

  if (ls.shader & SHADER_EXCLUDE_SCATTER) {
    ls.emitter_id = EMITTER_NONE;
    return false;
  }

  if (ls.t == FLT_MAX) {
    /* Sampled distant/background light is valid in volume segment, but we are going to sample the
     * light position with distance sampling instead of equiangular. */
    return false;
  }

  equiangular_coeffs->P = ls.P;

  return volume_equiangular_valid_ray_segment(
      kg, ray->P, ray->D, &equiangular_coeffs->t_range, &ls);
}

/* Path tracing: sample point on light and evaluate light shader, then
 * queue shadow ray to be traced. */
ccl_device_forceinline void integrate_volume_direct_light(
    KernelGlobals kg,
    IntegratorState state,
    ccl_private const ShaderData *ccl_restrict sd,
    ccl_private const RNGState *ccl_restrict rng_state,
    const float3 P,
    ccl_private const ShaderVolumePhases *ccl_restrict phases,
#  ifdef __PATH_GUIDING__
    ccl_private const Spectrum unlit_throughput,
#  endif
    ccl_private const Spectrum throughput,
    ccl_private LightSample &ccl_restrict ls)
{
  PROFILING_INIT(kg, PROFILING_SHADE_VOLUME_DIRECT_LIGHT);

  if (!kernel_data.integrator.use_direct_light || ls.emitter_id == EMITTER_NONE) {
    return;
  }

  /* Sample position on the same light again, now from the shading point where we scattered. */
  {
    const uint32_t path_flag = INTEGRATOR_STATE(state, path, flag);
    const uint bounce = INTEGRATOR_STATE(state, path, bounce);
    const float3 rand_light = path_state_rng_3D(kg, rng_state, PRNG_LIGHT);
    const float3 N = zero_float3();
    const int object_receiver = light_link_receiver_nee(kg, sd);
    const int shader_flags = SD_BSDF_HAS_TRANSMISSION;

    if (!light_sample<false>(
            kg, rand_light, sd->time, P, N, object_receiver, shader_flags, bounce, path_flag, &ls))
    {
      return;
    }
  }

  if (ls.shader & SHADER_EXCLUDE_SCATTER) {
    return;
  }

  /* Evaluate light shader.
   *
   * TODO: can we reuse sd memory? In theory we can move this after
   * integrate_surface_bounce, evaluate the BSDF, and only then evaluate
   * the light shader. This could also move to its own kernel, for
   * non-constant light sources. */
  ShaderDataTinyStorage emission_sd_storage;
  ccl_private ShaderData *emission_sd = AS_SHADER_DATA(&emission_sd_storage);
  const Spectrum light_eval = light_sample_shader_eval(kg, state, emission_sd, &ls, sd->time);
  if (is_zero(light_eval)) {
    return;
  }

  /* Evaluate BSDF. */
  BsdfEval phase_eval ccl_optional_struct_init;
  float phase_pdf = volume_shader_phase_eval(kg, state, sd, phases, ls.D, &phase_eval, ls.shader);
  const float mis_weight = light_sample_mis_weight_nee(kg, ls.pdf, phase_pdf);
  bsdf_eval_mul(&phase_eval, light_eval / ls.pdf * mis_weight);

  /* Path termination. */
  const float terminate = path_state_rng_light_termination(kg, rng_state);
  if (light_sample_terminate(kg, &phase_eval, terminate)) {
    return;
  }

  /* Create shadow ray. */
  Ray ray ccl_optional_struct_init;
  light_sample_to_volume_shadow_ray(kg, sd, &ls, P, &ray);

  /* Branch off shadow kernel. */
  IntegratorShadowState shadow_state = integrator_shadow_path_init(
      kg, state, DEVICE_KERNEL_INTEGRATOR_INTERSECT_SHADOW, false);

  /* Write shadow ray and associated state to global memory. */
  integrator_state_write_shadow_ray(shadow_state, &ray);
  integrator_state_write_shadow_ray_self(kg, shadow_state, &ray);

  /* Copy state from main path to shadow path. */
  const uint16_t bounce = INTEGRATOR_STATE(state, path, bounce);
  const uint16_t transparent_bounce = INTEGRATOR_STATE(state, path, transparent_bounce);
  uint32_t shadow_flag = INTEGRATOR_STATE(state, path, flag);
  const Spectrum throughput_phase = throughput * bsdf_eval_sum(&phase_eval);

  if (kernel_data.kernel_features & KERNEL_FEATURE_LIGHT_PASSES) {
    PackedSpectrum pass_diffuse_weight;
    PackedSpectrum pass_glossy_weight;

    if (shadow_flag & PATH_RAY_ANY_PASS) {
      /* Indirect bounce, use weights from earlier surface or volume bounce. */
      pass_diffuse_weight = INTEGRATOR_STATE(state, path, pass_diffuse_weight);
      pass_glossy_weight = INTEGRATOR_STATE(state, path, pass_glossy_weight);
    }
    else {
      /* Direct light, no diffuse/glossy distinction needed for volumes. */
      shadow_flag |= PATH_RAY_VOLUME_PASS;
      pass_diffuse_weight = one_spectrum();
      pass_glossy_weight = zero_spectrum();
    }

    INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, pass_diffuse_weight) = pass_diffuse_weight;
    INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, pass_glossy_weight) = pass_glossy_weight;
  }

  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, render_pixel_index) = INTEGRATOR_STATE(
      state, path, render_pixel_index);
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, rng_offset) = INTEGRATOR_STATE(
      state, path, rng_offset);
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, rng_pixel) = INTEGRATOR_STATE(
      state, path, rng_pixel);
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, sample) = INTEGRATOR_STATE(
      state, path, sample);
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, flag) = shadow_flag;
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, bounce) = bounce;
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, transparent_bounce) = transparent_bounce;
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, diffuse_bounce) = INTEGRATOR_STATE(
      state, path, diffuse_bounce);
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, glossy_bounce) = INTEGRATOR_STATE(
      state, path, glossy_bounce);
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, transmission_bounce) = INTEGRATOR_STATE(
      state, path, transmission_bounce);
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, throughput) = throughput_phase;

  /* Write Light-group, +1 as light-group is int but we need to encode into a uint8_t. */
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, lightgroup) = ls.group + 1;

#  ifdef __PATH_GUIDING__
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, unlit_throughput) = unlit_throughput;
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, path_segment) = INTEGRATOR_STATE(
      state, guiding, path_segment);
  INTEGRATOR_STATE(shadow_state, shadow_path, guiding_mis_weight) = 0.0f;
#  endif

  integrator_state_copy_volume_stack_to_shadow(kg, shadow_state, state);
}

/* Path tracing: scatter in new direction using phase function */
ccl_device_forceinline bool integrate_volume_phase_scatter(
    KernelGlobals kg,
    IntegratorState state,
    ccl_private ShaderData *sd,
    ccl_private const Ray *ray,
    ccl_private const RNGState *rng_state,
    ccl_private const ShaderVolumePhases *phases)
{
  PROFILING_INIT(kg, PROFILING_SHADE_VOLUME_INDIRECT_LIGHT);

  float2 rand_phase = path_state_rng_2D(kg, rng_state, PRNG_VOLUME_PHASE);

  ccl_private const ShaderVolumeClosure *svc = volume_shader_phase_pick(phases, &rand_phase);

  /* Phase closure, sample direction. */
  float phase_pdf = 0.0f, unguided_phase_pdf = 0.0f;
  BsdfEval phase_eval ccl_optional_struct_init;
  float3 phase_wo ccl_optional_struct_init;
  float sampled_roughness = 1.0f;
  int label;

#  if defined(__PATH_GUIDING__) && PATH_GUIDING_LEVEL >= 4
  if (kernel_data.integrator.use_guiding) {
    label = volume_shader_phase_guided_sample(kg,
                                              state,
                                              sd,
                                              svc,
                                              rand_phase,
                                              &phase_eval,
                                              &phase_wo,
                                              &phase_pdf,
                                              &unguided_phase_pdf,
                                              &sampled_roughness);

    if (phase_pdf == 0.0f || bsdf_eval_is_zero(&phase_eval)) {
      return false;
    }

    INTEGRATOR_STATE_WRITE(state, path, unguided_throughput) *= phase_pdf / unguided_phase_pdf;
  }
  else
#  endif
  {
    label = volume_shader_phase_sample(
        kg, sd, phases, svc, rand_phase, &phase_eval, &phase_wo, &phase_pdf, &sampled_roughness);

    if (phase_pdf == 0.0f || bsdf_eval_is_zero(&phase_eval)) {
      return false;
    }

    unguided_phase_pdf = phase_pdf;
  }

  /* Setup ray. */
  INTEGRATOR_STATE_WRITE(state, ray, P) = sd->P;
  INTEGRATOR_STATE_WRITE(state, ray, D) = normalize(phase_wo);
  INTEGRATOR_STATE_WRITE(state, ray, tmin) = 0.0f;
#  ifdef __LIGHT_TREE__
  if (kernel_data.integrator.use_light_tree) {
    INTEGRATOR_STATE_WRITE(state, ray, previous_dt) = ray->tmax - ray->tmin;
  }
#  endif
  INTEGRATOR_STATE_WRITE(state, ray, tmax) = FLT_MAX;
#  ifdef __RAY_DIFFERENTIALS__
  INTEGRATOR_STATE_WRITE(state, ray, dP) = differential_make_compact(sd->dP);
#  endif
  // Save memory by storing last hit prim and object in isect
  INTEGRATOR_STATE_WRITE(state, isect, prim) = sd->prim;
  INTEGRATOR_STATE_WRITE(state, isect, object) = sd->object;

  const Spectrum phase_weight = bsdf_eval_sum(&phase_eval) / phase_pdf;

  /* Add phase function sampling data to the path segment. */
  guiding_record_volume_bounce(
      kg, state, sd, phase_weight, phase_pdf, normalize(phase_wo), sampled_roughness);

  /* Update throughput. */
  const Spectrum throughput = INTEGRATOR_STATE(state, path, throughput);
  const Spectrum throughput_phase = throughput * phase_weight;
  INTEGRATOR_STATE_WRITE(state, path, throughput) = throughput_phase;

  if (kernel_data.kernel_features & KERNEL_FEATURE_LIGHT_PASSES) {
    if (INTEGRATOR_STATE(state, path, bounce) == 0) {
      INTEGRATOR_STATE_WRITE(state, path, pass_diffuse_weight) = one_spectrum();
      INTEGRATOR_STATE_WRITE(state, path, pass_glossy_weight) = zero_spectrum();
    }
  }

  /* Update path state */
  INTEGRATOR_STATE_WRITE(state, path, mis_ray_pdf) = phase_pdf;
  const float3 previous_P = ray->P + ray->D * ray->tmin;
  INTEGRATOR_STATE_WRITE(state, path, mis_origin_n) = sd->P - previous_P;
  INTEGRATOR_STATE_WRITE(state, path, min_ray_pdf) = fminf(
      unguided_phase_pdf, INTEGRATOR_STATE(state, path, min_ray_pdf));

#  ifdef __LIGHT_LINKING__
  if (kernel_data.kernel_features & KERNEL_FEATURE_LIGHT_LINKING) {
    INTEGRATOR_STATE_WRITE(state, path, mis_ray_object) = sd->object;
  }
#  endif

  path_state_next(kg, state, label, sd->flag);
  return true;
}

/* get the volume attenuation and emission over line segment defined by
 * ray, with the assumption that there are no surfaces blocking light
 * between the endpoints. distance sampling is used to decide if we will
 * scatter or not. */
ccl_device VolumeIntegrateEvent volume_integrate(KernelGlobals kg,
                                                 IntegratorState state,
                                                 ccl_private Ray *ccl_restrict ray,
                                                 ccl_global float *ccl_restrict render_buffer)
{
  ShaderData sd;
  /* FIXME: `object` is used for light linking. We read the bottom of the stack for simplicity, but
   * this does not work for overlapping volumes. */
  shader_setup_from_volume(kg, &sd, ray, INTEGRATOR_STATE_ARRAY(state, volume_stack, 0, object));

  /* Load random number state. */
  RNGState rng_state;
  path_state_rng_load(state, &rng_state);

  /* Sample light ahead of volume stepping, for equiangular sampling. */
  /* TODO: distant lights are ignored now, but could instead use even distribution. */
  LightSample ls ccl_optional_struct_init;
  const bool need_light_sample = !(INTEGRATOR_STATE(state, path, flag) & PATH_RAY_TERMINATE);

  EquiangularCoefficients equiangular_coeffs = {zero_float3(), {ray->tmin, ray->tmax}};

  const bool have_equiangular_sample =
      need_light_sample && integrate_volume_equiangular_sample_light(
                               kg, state, ray, &sd, &rng_state, &equiangular_coeffs, ls);

  VolumeSampleMethod direct_sample_method = (have_equiangular_sample) ?
                                                volume_stack_sample_method(kg, state) :
                                                VOLUME_SAMPLE_DISTANCE;

  /* Step through volume. */
  VOLUME_READ_LAMBDA(integrator_state_read_volume_stack(state, i))
  const float step_size = volume_stack_step_size(kg, volume_read_lambda_pass);

#  if defined(__PATH_GUIDING__) && PATH_GUIDING_LEVEL >= 1
  /* The current path throughput which is used later to calculate per-segment throughput. */
  const float3 initial_throughput = INTEGRATOR_STATE(state, path, throughput);
  /* The path throughput used to calculate the throughput for direct light. */
  float3 unlit_throughput = initial_throughput;
  /* If a new path segment is generated at the direct scatter position. */
  bool guiding_generated_new_segment = false;
  float rand_phase_guiding = 0.5f;
#  endif

  /* TODO: expensive to zero closures? */
  VolumeIntegrateResult result = {};
  volume_integrate_heterogeneous(kg,
                                 state,
                                 ray,
                                 &sd,
                                 &rng_state,
                                 render_buffer,
                                 step_size,
                                 direct_sample_method,
                                 equiangular_coeffs,
                                 result);

  /* Perform path termination. The intersect_closest will have already marked this path
   * to be terminated. That will shading evaluating to leave out any scattering closures,
   * but emission and absorption are still handled for multiple importance sampling. */
  const uint32_t path_flag = INTEGRATOR_STATE(state, path, flag);
  const float continuation_probability = (path_flag & PATH_RAY_TERMINATE_IN_NEXT_VOLUME) ?
                                             0.0f :
                                             INTEGRATOR_STATE(
                                                 state, path, continuation_probability);
  if (continuation_probability == 0.0f) {
    return VOLUME_PATH_MISSED;
  }

  /* Direct light. */
  if (result.direct_scatter) {
    const float3 direct_P = ray->P + result.direct_t * ray->D;

#  ifdef __PATH_GUIDING__
    if (kernel_data.integrator.use_guiding) {
#    if PATH_GUIDING_LEVEL >= 1
      if (result.direct_sample_method == VOLUME_SAMPLE_DISTANCE) {
        /* If the direct scatter event is generated using VOLUME_SAMPLE_DISTANCE the direct event
         * will happen at the same position as the indirect event and the direct light contribution
         * will contribute to the position of the next path segment. */
        float3 transmittance_weight = spectrum_to_rgb(
            safe_divide_color(result.indirect_throughput, initial_throughput));
        guiding_record_volume_transmission(kg, state, transmittance_weight);
        guiding_record_volume_segment(kg, state, direct_P, sd.wi);
        guiding_generated_new_segment = true;
        unlit_throughput = result.indirect_throughput / continuation_probability;
        rand_phase_guiding = path_state_rng_1D(kg, &rng_state, PRNG_VOLUME_PHASE_GUIDING_DISTANCE);
      }
      else {
        /* If the direct scatter event is generated using VOLUME_SAMPLE_EQUIANGULAR the direct
         * event will happen at a separate position as the indirect event and the direct light
         * contribution will contribute to the position of the current/previous path segment. The
         * unlit_throughput has to be adjusted to include the scattering at the previous segment.
         */
        float3 scatterEval = one_float3();
        if (state->guiding.path_segment) {
          pgl_vec3f scatteringWeight = state->guiding.path_segment->scatteringWeight;
          scatterEval = make_float3(scatteringWeight.x, scatteringWeight.y, scatteringWeight.z);
        }
        unlit_throughput /= scatterEval;
        unlit_throughput *= continuation_probability;
        rand_phase_guiding = path_state_rng_1D(
            kg, &rng_state, PRNG_VOLUME_PHASE_GUIDING_EQUIANGULAR);
      }
#    endif
#    if PATH_GUIDING_LEVEL >= 4
      volume_shader_prepare_guiding(
          kg, state, &sd, rand_phase_guiding, direct_P, ray->D, &result.direct_phases);
#    endif
    }
#  endif

    result.direct_throughput /= continuation_probability;
    integrate_volume_direct_light(kg,
                                  state,
                                  &sd,
                                  &rng_state,
                                  direct_P,
                                  &result.direct_phases,
#  ifdef __PATH_GUIDING__
                                  unlit_throughput,
#  endif
                                  result.direct_throughput,
                                  ls);
  }

  /* Indirect light.
   *
   * Only divide throughput by continuation_probability if we scatter. For the attenuation
   * case the next surface will already do this division. */
  if (result.indirect_scatter) {
#  if defined(__PATH_GUIDING__) && PATH_GUIDING_LEVEL >= 1
    if (!guiding_generated_new_segment) {
      float3 transmittance_weight = spectrum_to_rgb(
          safe_divide_color(result.indirect_throughput, initial_throughput));
      guiding_record_volume_transmission(kg, state, transmittance_weight);
    }
#  endif
    result.indirect_throughput /= continuation_probability;
  }
  INTEGRATOR_STATE_WRITE(state, path, throughput) = result.indirect_throughput;

  if (result.indirect_scatter) {
    sd.P = ray->P + result.indirect_t * ray->D;

#  if defined(__PATH_GUIDING__)
#    if PATH_GUIDING_LEVEL >= 1
    if (!guiding_generated_new_segment) {
      guiding_record_volume_segment(kg, state, sd.P, sd.wi);
    }
#    endif
#    if PATH_GUIDING_LEVEL >= 4
    /* If the direct scatter event was generated using VOLUME_SAMPLE_EQUIANGULAR we need to
     * initialize the guiding distribution at the indirect scatter position. */
    if (result.direct_sample_method == VOLUME_SAMPLE_EQUIANGULAR) {
      rand_phase_guiding = path_state_rng_1D(kg, &rng_state, PRNG_VOLUME_PHASE_GUIDING_DISTANCE);
      volume_shader_prepare_guiding(
          kg, state, &sd, rand_phase_guiding, sd.P, ray->D, &result.indirect_phases);
    }
#    endif
#  endif

    if (integrate_volume_phase_scatter(kg, state, &sd, ray, &rng_state, &result.indirect_phases)) {
      return VOLUME_PATH_SCATTERED;
    }
    else {
      return VOLUME_PATH_MISSED;
    }
  }
  else {
#  if defined(__PATH_GUIDING__)
    /* No guiding if we don't scatter. */
    state->guiding.use_volume_guiding = false;
#  endif
    return VOLUME_PATH_ATTENUATED;
  }
}

#endif

ccl_device void integrator_shade_volume(KernelGlobals kg,
                                        IntegratorState state,
                                        ccl_global float *ccl_restrict render_buffer)
{
  PROFILING_INIT(kg, PROFILING_SHADE_VOLUME_SETUP);

#ifdef __VOLUME__
  /* Setup shader data. */
  Ray ray ccl_optional_struct_init;
  integrator_state_read_ray(state, &ray);

  Intersection isect ccl_optional_struct_init;
  integrator_state_read_isect(state, &isect);

  /* Set ray length to current segment. */
  ray.tmax = (isect.prim != PRIM_NONE) ? isect.t : FLT_MAX;

  /* Clean volume stack for background rays. */
  if (isect.prim == PRIM_NONE) {
    volume_stack_clean(kg, state);
  }

  const VolumeIntegrateEvent event = volume_integrate(kg, state, &ray, render_buffer);
  if (event == VOLUME_PATH_MISSED) {
    /* End path. */
    integrator_path_terminate(kg, state, DEVICE_KERNEL_INTEGRATOR_SHADE_VOLUME);
    return;
  }

  if (event == VOLUME_PATH_ATTENUATED) {
    /* Continue to background, light or surface. */
    integrator_intersect_next_kernel_after_volume<DEVICE_KERNEL_INTEGRATOR_SHADE_VOLUME>(
        kg, state, &isect, render_buffer);
    return;
  }

#  ifdef __SHADOW_LINKING__
  if (shadow_linking_schedule_intersection_kernel<DEVICE_KERNEL_INTEGRATOR_SHADE_VOLUME>(kg,
                                                                                         state))
  {
    return;
  }
#  endif /* __SHADOW_LINKING__ */

  /* Queue intersect_closest kernel. */
  integrator_path_next(kg,
                       state,
                       DEVICE_KERNEL_INTEGRATOR_SHADE_VOLUME,
                       DEVICE_KERNEL_INTEGRATOR_INTERSECT_CLOSEST);
#endif /* __VOLUME__ */
}

CCL_NAMESPACE_END
