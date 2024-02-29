/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(draw_view_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_horizon_scan_eval_lib.glsl)

void main()
{
  const uint tile_size = RAYTRACE_GROUP_SIZE;
  uvec2 tile_coord = unpackUvec2x16(tiles_coord_buf[gl_WorkGroupID.x]);
  ivec2 texel = ivec2(gl_LocalInvocationID.xy + tile_coord * tile_size);

  ivec2 texel_fullres = texel * uniform_buf.raytrace.resolution_scale +
                        uniform_buf.raytrace.resolution_bias;

  /* Avoid tracing the outside border if dispatch is too big. */
  ivec2 extent = textureSize(gbuf_header_tx, 0).xy;
  if (any(greaterThanEqual(texel * uniform_buf.raytrace.resolution_scale, extent))) {
    return;
  }

  /* Avoid loading texels outside texture range.
   * This can happen even after the check above in non-power-of-2 textures. */
  texel_fullres = min(texel_fullres, extent - 1);

  /* Do not trace where nothing was rendered. */
  if (texelFetch(gbuf_header_tx, texel_fullres, 0).r == 0u) {
    imageStore(horizon_radiance_img, ivec3(texel, 0), vec4(FLT_11_11_10_MAX, 0.0));
    return;
  }

  vec2 uv = (vec2(texel_fullres) + 0.5) * uniform_buf.raytrace.full_resolution_inv;
  float depth = texelFetch(hiz_tx, texel_fullres, 0).r;
  vec3 vP = drw_point_screen_to_view(vec3(uv, depth));

  HorizonScanContext ctx;
  ctx.closure.N = horizon_scan_sample_normal(uv);

  vec2 noise = utility_tx_fetch(utility_tx, vec2(texel), UTIL_BLUE_NOISE_LAYER).rg;
  noise = fract(noise + sampling_rng_2D_get(SAMPLING_AO_U));

  horizon_scan_eval(vP,
                    ctx,
                    noise,
                    uniform_buf.ao.pixel_size,
                    1.0e16,
                    uniform_buf.ao.thickness,
                    uniform_buf.ao.angle_bias,
                    8,
                    false);

  imageStore(horizon_radiance_img, ivec3(texel, 0), ctx.sh_result.L0.M0);
  imageStore(horizon_radiance_img, ivec3(texel, 1), ctx.sh_result.L1.Mn1);
  imageStore(horizon_radiance_img, ivec3(texel, 2), ctx.sh_result.L1.M0);
  imageStore(horizon_radiance_img, ivec3(texel, 3), ctx.sh_result.L1.Mp1);
}
