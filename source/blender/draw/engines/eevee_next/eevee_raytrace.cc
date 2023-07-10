/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation.
 */

/** \file
 * \ingroup eevee
 *
 * The raytracing module class handles ray generation, scheduling, tracing and denoising.
 */

#include <fstream>
#include <iostream>

#include "BKE_global.h"

#include "eevee_instance.hh"

#include "eevee_raytrace.hh"

namespace blender::eevee {

/* -------------------------------------------------------------------- */
/** \name Raytracing
 *
 * \{ */

void RayTraceModule::init()
{
  enabled_ = true;

  const int flag = inst_.scene->eevee.flag;

  use_spatial_denoise_ = (flag & SCE_EEVEE_RAYTRACE_DENOISE);
  use_temporal_denoise_ = use_spatial_denoise_ && (flag & SCE_EEVEE_RAYTRACE_DENOISE_TEMPORAL);
  use_bilateral_denoise_ = use_temporal_denoise_ && (flag & SCE_EEVEE_RAYTRACE_DENOISE_BILATERAL);

  if (!U.experimental.use_eevee_debug) {
    /* These cannot be turned off separately when not in debug mode. */
    use_temporal_denoise_ = use_bilateral_denoise_ = use_spatial_denoise_;
  }

  data_.valid_history_reflection = 1;
  data_.resolution_scale = 1;
  data_.resolution_bias = int2(0);
  data_.thickness = 1.0f;
  data_.quality = 1.0f;
  data_.brightness_clamp = 1.0f;
  data_.max_roughness = 1.0f;
  data_.pool_offset = 0;
}

void RayTraceModule::sync()
{
  /* Setup. */
  {
    PassSimple &pass = tile_classify_ps_;
    pass.init();
    pass.shader_set(inst_.shaders.static_shader_get(RAY_TILE_CLASSIFY));
    pass.bind_texture("gbuffer_closure_tx", &inst_.gbuffer.closure_tx);
    pass.bind_texture("stencil_tx", &renderbuf_stencil_view_);
    pass.bind_image("tile_mask_img", &tile_mask_tx_);
    pass.bind_ssbo("ray_dispatch_buf", &ray_dispatch_buf_);
    pass.bind_ssbo("ray_tiles_buf", &ray_tiles_buf_);
    pass.bind_ubo("raytrace_buf", &data_);
    pass.dispatch(&tile_dispatch_size_);
    pass.barrier(GPU_BARRIER_TEXTURE_FETCH);
  }
  for (auto type : IndexRange(2)) {
    PassSimple &pass = (type == 0) ? generate_reflect_ps_ : generate_refract_ps_;
    pass.init();
    pass.shader_set(inst_.shaders.static_shader_get((type == 0) ? RAY_GENERATE_REFLECT :
                                                                  RAY_GENERATE_REFRACT));
    pass.bind_texture(RBUFS_UTILITY_TEX_SLOT, inst_.pipelines.utility_tx);
    pass.bind_texture("stencil_tx", &renderbuf_stencil_view_);
    pass.bind_texture("gbuffer_closure_tx", &inst_.gbuffer.closure_tx);
    pass.bind_texture("gbuffer_color_tx", &inst_.gbuffer.color_tx);
    pass.bind_image("out_ray_data_img", &ray_data_tx_);
    pass.bind_ssbo("tiles_coord_buf", &ray_tiles_buf_);
    inst_.sampling.bind_resources(&pass);
    pass.dispatch(ray_dispatch_buf_);
    pass.barrier(GPU_BARRIER_SHADER_STORAGE | GPU_BARRIER_TEXTURE_FETCH |
                 GPU_BARRIER_SHADER_IMAGE_ACCESS);
  }
  /* Tracing. */
  for (auto type : IndexRange(2)) {
    PassSimple &pass = (type == 0) ? trace_reflect_ps_ : trace_refract_ps_;
    pass.init();
    pass.shader_set(inst_.shaders.static_shader_get((type == 0) ? RAY_TRACE_SCREEN_REFLECT :
                                                                  RAY_TRACE_SCREEN_REFRACT));
    pass.bind_ssbo("tiles_coord_buf", &ray_tiles_buf_);
    pass.bind_image("ray_data_img", &ray_data_tx_);
    pass.bind_image("ray_time_img", &ray_time_tx_);
    pass.bind_image("ray_radiance_img", &ray_radiance_tx_);
    pass.bind_ubo("raytrace_buf", &data_);
    inst_.hiz_buffer.bind_resources(&pass);
    inst_.sampling.bind_resources(&pass);
    inst_.reflection_probes.bind_resources(&pass);
    pass.dispatch(ray_dispatch_buf_);
    pass.barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
  }
  /* Denoise. */
  for (auto type : IndexRange(2)) {
    PassSimple &pass = (type == 0) ? denoise_spatial_reflect_ps_ : denoise_spatial_refract_ps_;
    pass.init();
    pass.shader_set(inst_.shaders.static_shader_get((type == 0) ? RAY_DENOISE_SPATIAL_REFLECT :
                                                                  RAY_DENOISE_SPATIAL_REFRACT));
    pass.bind_ssbo("tiles_coord_buf", &ray_tiles_buf_);
    pass.bind_texture("gbuffer_closure_tx", &inst_.gbuffer.closure_tx);
    pass.bind_image("ray_data_img", &ray_data_tx_);
    pass.bind_image("ray_time_img", &ray_time_tx_);
    pass.bind_image("ray_radiance_img", &ray_radiance_tx_);
    pass.bind_image("out_radiance_img", &denoised_spatial_tx_);
    pass.bind_image("out_variance_img", &hit_variance_tx_);
    pass.bind_image("out_hit_depth_img", &hit_depth_tx_);
    pass.bind_image("tile_mask_img", &tile_mask_tx_);
    pass.bind_ubo("raytrace_buf", &data_);
    inst_.sampling.bind_resources(&pass);
    inst_.hiz_buffer.bind_resources(&pass);
    pass.dispatch(ray_dispatch_buf_);
    pass.barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
  }
  {
    PassSimple &pass = denoise_temporal_ps_;
    pass.init();
    pass.shader_set(inst_.shaders.static_shader_get(RAY_DENOISE_TEMPORAL));
    pass.bind_ubo("raytrace_buf", &data_);
    pass.bind_texture("radiance_history_tx", &radiance_history_tx_);
    pass.bind_texture("variance_history_tx", &variance_history_tx_);
    pass.bind_texture("tilemask_history_tx", &tilemask_history_tx_);
    pass.bind_image("hit_depth_img", &hit_depth_tx_);
    pass.bind_image("in_radiance_img", &denoised_spatial_tx_);
    pass.bind_image("out_radiance_img", &denoised_temporal_tx_);
    pass.bind_image("in_variance_img", &hit_variance_tx_);
    pass.bind_image("out_variance_img", &denoise_variance_tx_);
    pass.bind_ssbo("tiles_coord_buf", &ray_tiles_buf_);
    inst_.sampling.bind_resources(&pass);
    inst_.hiz_buffer.bind_resources(&pass);
    pass.dispatch(ray_dispatch_buf_);
    pass.barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
  }
  for (auto type : IndexRange(2)) {
    PassSimple &pass = (type == 0) ? denoise_bilateral_reflect_ps_ : denoise_bilateral_refract_ps_;
    pass.init();
    pass.shader_set(inst_.shaders.static_shader_get((type == 0) ? RAY_DENOISE_BILATERAL_REFLECT :
                                                                  RAY_DENOISE_BILATERAL_REFRACT));
    pass.bind_texture("gbuffer_closure_tx", &inst_.gbuffer.closure_tx);
    pass.bind_image("in_radiance_img", &denoised_temporal_tx_);
    pass.bind_image("out_radiance_img", &denoised_bilateral_tx_);
    pass.bind_image("in_variance_img", &denoise_variance_tx_);
    pass.bind_image("tile_mask_img", &tile_mask_tx_);
    pass.bind_ssbo("tiles_coord_buf", &ray_tiles_buf_);
    pass.bind_ubo("raytrace_buf", &data_);
    inst_.sampling.bind_resources(&pass);
    inst_.hiz_buffer.bind_resources(&pass);
    pass.dispatch(ray_dispatch_buf_);
    pass.barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
  }
}

void RayTraceModule::debug_pass_sync() {}

void RayTraceModule::debug_draw(View & /* view */, GPUFrameBuffer * /* view_fb */) {}

RayTraceResult RayTraceModule::trace(RayTraceBuffer &rt_buffer,
                                     eClosureBits active_closures,
                                     eClosureBits raytrace_closure,
                                     View &view)
{
  BLI_assert_msg(count_bits_i(raytrace_closure) == 1,
                 "Only one closure type can be raytraced at a time.");
  BLI_assert_msg(raytrace_closure ==
                     (raytrace_closure & (CLOSURE_REFLECTION | CLOSURE_REFRACTION)),
                 "Only reflection and refraction are implemented.");

  PassSimple *generate_ray_ps = nullptr;
  PassSimple *trace_ray_ps = nullptr;
  PassSimple *denoise_spatial_ps = nullptr;
  PassSimple *denoise_bilateral_ps = nullptr;
  RayTraceBuffer::DenoiseBuffer *denoise_buf = nullptr;

  if (raytrace_closure == CLOSURE_REFLECTION) {
    generate_ray_ps = &generate_reflect_ps_;
    trace_ray_ps = &trace_reflect_ps_;
    denoise_spatial_ps = &denoise_spatial_reflect_ps_;
    denoise_bilateral_ps = &denoise_bilateral_reflect_ps_;
    denoise_buf = &rt_buffer.reflection;
  }
  else if (raytrace_closure == CLOSURE_REFRACTION) {
    generate_ray_ps = &generate_refract_ps_;
    trace_ray_ps = &trace_refract_ps_;
    denoise_spatial_ps = &denoise_spatial_refract_ps_;
    denoise_bilateral_ps = &denoise_bilateral_refract_ps_;
    denoise_buf = &rt_buffer.refraction;
  }

  if ((active_closures & raytrace_closure) == 0) {
    /* Early out. Release persistent buffers. */
    denoise_buf->denoised_spatial_tx.acquire(int2(1), RAYTRACE_RADIANCE_FORMAT);
    denoise_buf->radiance_history_tx.free();
    denoise_buf->variance_history_tx.free();
    denoise_buf->tilemask_history_tx.free();
    return {denoise_buf->denoised_spatial_tx};
  }

  int2 extent = inst_.film.render_extent_get();
  int2 dummy_extent(1, 1);

  tile_dispatch_size_ = int3(math::divide_ceil(extent, int2(RAYTRACE_GROUP_SIZE)), 1);
  const int tile_count = tile_dispatch_size_.x * tile_dispatch_size_.y;

  renderbuf_stencil_view_ = inst_.render_buffers.depth_tx.stencil_view();
  renderbuf_depth_view_ = inst_.render_buffers.depth_tx;

  /* TODO(fclem): Half-Res tracing. */
  int2 tracing_res = extent;

  DRW_stats_group_start("Raytracing");

  data_.closure_active = raytrace_closure;
  data_.history_persmat = denoise_buf->history_persmat;
  data_.full_resolution = extent;
  data_.full_resolution_inv = 1.0f / float2(extent);
  data_.skip_denoise = !use_spatial_denoise_;
  data_.push_update();

  tile_mask_tx_.acquire(tile_dispatch_size_.xy(), RAYTRACE_TILEMASK_FORMAT);
  ray_tiles_buf_.resize(ceil_to_multiple_u(tile_count, 512));

  /* Ray setup. */
  GPU_storagebuf_clear_to_zero(ray_dispatch_buf_);
  inst_.manager->submit(tile_classify_ps_, view);

  {
    /* Tracing rays. */
    ray_data_tx_.acquire(tracing_res, GPU_RGBA16F);
    ray_time_tx_.acquire(tracing_res, GPU_R32F);
    ray_radiance_tx_.acquire(tracing_res, RAYTRACE_RADIANCE_FORMAT);

    inst_.manager->submit(*generate_ray_ps, view);
    inst_.manager->submit(*trace_ray_ps, view);
  }

  RayTraceResult result;

  /* Spatial denoise pass is required to resolve at least one ray per pixel. */
  {
    denoise_buf->denoised_spatial_tx.acquire(extent, RAYTRACE_RADIANCE_FORMAT);
    hit_variance_tx_.acquire(use_temporal_denoise_ ? extent : dummy_extent,
                             RAYTRACE_VARIANCE_FORMAT);
    hit_depth_tx_.acquire(use_temporal_denoise_ ? extent : dummy_extent, GPU_R32F);
    denoised_spatial_tx_ = denoise_buf->denoised_spatial_tx;

    inst_.manager->submit(*denoise_spatial_ps, view);

    result = {denoise_buf->denoised_spatial_tx};
  }

  ray_data_tx_.release();
  ray_time_tx_.release();
  ray_radiance_tx_.release();

  if (use_temporal_denoise_) {
    denoise_buf->denoised_temporal_tx.acquire(extent, RAYTRACE_RADIANCE_FORMAT);
    denoise_variance_tx_.acquire(use_bilateral_denoise_ ? extent : dummy_extent,
                                 RAYTRACE_VARIANCE_FORMAT);
    denoise_buf->radiance_history_tx.ensure_2d(RAYTRACE_RADIANCE_FORMAT, extent);
    denoise_buf->variance_history_tx.ensure_2d(RAYTRACE_VARIANCE_FORMAT,
                                               use_bilateral_denoise_ ? extent : dummy_extent);
    if (denoise_buf->tilemask_history_tx.ensure_2d(RAYTRACE_TILEMASK_FORMAT,
                                                   tile_dispatch_size_.xy()))
    {
      denoise_buf->tilemask_history_tx.clear(uint4(0u));
    }

    radiance_history_tx_ = denoise_buf->radiance_history_tx;
    variance_history_tx_ = denoise_buf->variance_history_tx;
    tilemask_history_tx_ = denoise_buf->tilemask_history_tx;
    denoised_temporal_tx_ = denoise_buf->denoised_temporal_tx;

    inst_.manager->submit(denoise_temporal_ps_, view);

    /* Swap after last use. */
    TextureFromPool::swap(tile_mask_tx_, denoise_buf->tilemask_history_tx);
    /* Save view-projection matrix for next reprojection. */
    denoise_buf->history_persmat = view.persmat();
    /* Radiance will be swapped with history in RayTraceResult::release().
     * Variance is swapped with history after bilateral denoise.
     * It keeps dataflow easier to follow. */
    result = {denoise_buf->denoised_temporal_tx, denoise_buf->radiance_history_tx};
    /* Not referenced by result anymore. */
    denoise_buf->denoised_spatial_tx.release();
  }

  hit_variance_tx_.release();
  hit_depth_tx_.release();

  if (use_bilateral_denoise_) {
    denoise_buf->denoised_bilateral_tx.acquire(extent, RAYTRACE_RADIANCE_FORMAT);
    denoised_bilateral_tx_ = denoise_buf->denoised_bilateral_tx;

    inst_.manager->submit(*denoise_bilateral_ps, view);

    /* Swap after last use. */
    TextureFromPool::swap(denoise_buf->denoised_temporal_tx, denoise_buf->radiance_history_tx);
    TextureFromPool::swap(denoise_variance_tx_, denoise_buf->variance_history_tx);

    result = {denoise_buf->denoised_bilateral_tx};
    /* Not referenced by result anymore. */
    denoise_buf->denoised_temporal_tx.release();
  }

  tile_mask_tx_.release();
  denoise_variance_tx_.release();

  DRW_stats_group_end();

  return result;
}

/** \} */

}  // namespace blender::eevee
