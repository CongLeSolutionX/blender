/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation.
 */

/** \file
 * \ingroup eevee
 *
 * The ray-tracing module class handles ray generation, scheduling, tracing and denoising.
 */

#pragma once

#include "DRW_render.h"

#include "eevee_shader_shared.hh"

namespace blender::eevee {

class Instance;

/* -------------------------------------------------------------------- */
/** \name Raytracing Buffers
 *
 * Contain persistent data used for temporal denoising. Similar to \class GBuffer but only contains
 * persistent data.
 * \{ */

/**
 * Contain persistent buffer that need to be stored per view.
 */
struct RayTraceBuffer {
  /** Set of buffers that need to be allocated for each ray type. */
  struct DenoiseBuffer {
    /* Persistent history buffers. */
    Texture radiance_history_tx = {"radiance_tx"};
    Texture variance_history_tx = {"variance_tx"};
    /* Map of tiles that were processed inside the history buffer. */
    Texture tilemask_history_tx = {"tilemask_tx"};
    /** Perspective matrix for which the history buffers were recorded. */
    float4x4 history_persmat;
    /**
     * Textures containing the ray hit radiance denoised (full-res). One of them is result_tx.
     * One might become result buffer so it need instantiation by closure type to avoid reuse.
     */
    TextureFromPool denoised_spatial_tx = {"denoised_spatial_tx"};
    TextureFromPool denoised_temporal_tx = {"denoised_temporal_tx"};
    TextureFromPool denoised_bilateral_tx = {"denoised_bilateral_tx"};
  };
  /**
   * One for each closure type. Not to be mistaken with deferred layer type.
   * For instance the opaque deferred layer will only used the reflection history buffer.
   */
  DenoiseBuffer reflection, refraction;
};

/**
 * Contains the result texture.
 * The result buffer is usually short lived and is kept in a TextureFromPool managed by the mode.
 * This structure contains a reference to it so that it can be freed after use by the caller.
 */
class RayTraceResult {
 private:
  /** Result is in a temporary texture that needs to be released. */
  TextureFromPool *result_ = nullptr;
  /** History buffer to swap the tmp texture that does not need to be released. */
  Texture *history_ = nullptr;

 public:
  RayTraceResult() = default;
  RayTraceResult(TextureFromPool &result) : result_(result.ptr()){};
  RayTraceResult(TextureFromPool &result, Texture &history)
      : result_(result.ptr()), history_(history.ptr()){};

  GPUTexture *get()
  {
    return *result_;
  }

  void release()
  {
    if (history_) {
      /* Swap after last use. */
      TextureFromPool::swap(*result_, *history_);
    }
    /* NOTE: This releases the previous history. */
    result_->release();
  }
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Raytracing
 * \{ */

class RayTraceModule {
 private:
  Instance &inst_;

  draw::PassSimple tile_classify_ps_ = {"TileClassify"};
  draw::PassSimple generate_reflect_ps_ = {"RayGenerate.Reflection"};
  draw::PassSimple generate_refract_ps_ = {"RayGenerate.Refraction"};
  draw::PassSimple trace_reflect_ps_ = {"Trace.Reflection"};
  draw::PassSimple trace_refract_ps_ = {"Trace.Refraction"};
  draw::PassSimple denoise_spatial_reflect_ps_ = {"DenoiseSpatial.Reflection"};
  draw::PassSimple denoise_spatial_refract_ps_ = {"DenoiseSpatial.Refraction"};
  draw::PassSimple denoise_temporal_ps_ = {"DenoiseTemporal"};
  draw::PassSimple denoise_bilateral_ps_ = {"DenoiseBilateral"};

  /** Dispatch with enough tiles for the whole screen. */
  int3 tile_dispatch_size_ = int3(1);
  /** 2D tile mask to check which unused adjacent tile we need to clear. */
  TextureFromPool tile_mask_tx_ = {"tile_mask_tx"};
  /** Indirect dispatch rays. Avoid dispatching workgroups that ultimately won't do any tracing. */
  DispatchIndirectBuf ray_dispatch_buf_ = {"ray_dispatch_buf_"};
  /** Tile buffer that contains tile coordinates. */
  RayTraceTileBuf ray_tiles_buf_ = {"ray_tiles_buf_"};
  /** Texture containing the ray direction and pdf. */
  TextureFromPool ray_data_tx_ = {"ray_data_tx"};
  /** Texture containing the ray hit time. */
  TextureFromPool ray_time_tx_ = {"ray_data_tx"};
  /** Texture containing the ray hit radiance (tracing-res). */
  TextureFromPool ray_radiance_tx_ = {"ray_radiance_tx"};
  /** Textures containing the ray hit radiance denoised (full-res). One of them is result_tx. */
  GPUTexture *denoised_spatial_tx_ = nullptr;
  GPUTexture *denoised_temporal_tx_ = nullptr;
  GPUTexture *denoised_bilateral_tx_ = nullptr;
  /** Ray hit depth for temporal denoising. Output of spatial denoise. */
  TextureFromPool hit_depth_tx_ = {"hit_depth_tx_"};
  /** Ray hit variance for temporal denoising. Output of spatial denoise. */
  TextureFromPool hit_variance_tx_ = {"hit_variance_tx_"};
  /** Temporally stable variance for temporal denoising. Output of temporal denoise. */
  TextureFromPool denoise_variance_tx_ = {"denoise_variance_tx_"};
  /** Persistent texture reference for temporal denoising input. */
  GPUTexture *radiance_history_tx_ = nullptr;
  GPUTexture *variance_history_tx_ = nullptr;
  GPUTexture *tilemask_history_tx_ = nullptr;

  /** Dummy texture when the tracing is disabled. */
  TextureFromPool dummy_result_tx_ = {"dummy_result_tx"};
  /** Pointer to inst_.render_buffers.depth_tx.stencil_view() updated before submission. */
  GPUTexture *renderbuf_stencil_view_ = nullptr;
  /** Pointer to inst_.render_buffers.depth_tx updated before submission. */
  GPUTexture *renderbuf_depth_view_ = nullptr;
  /** Closure being ray-traced. (Is #eClosureBits but is being used as push_constant). */
  int closure_active_;

  bool enabled_ = false;
  bool use_spatial_denoise_ = true;
  bool use_temporal_denoise_ = true;
  bool use_bilateral_denoise_ = true;

  RayTraceDataBuf data_;

 public:
  RayTraceModule(Instance &inst) : inst_(inst){};

  void init();

  void sync();

  /**
   * RayTrace the scene and resolve a radiance buffer for the corresponding `closure_bit` into the
   * given `out_radiance_tx`.
   *
   * Should not be conditionally executed as it manages the RayTraceResult.
   *
   * \arg active_closures is a mask of all active closures in a deferred layer.
   * \arg raytrace_closure is type of closure the rays are to be casted for.
   */
  RayTraceResult trace(RayTraceBuffer &rt_buffer,
                       eClosureBits active_closures,
                       eClosureBits raytrace_closure,
                       View &view);

  void debug_pass_sync();
  void debug_draw(View &view, GPUFrameBuffer *view_fb);
};

/** \} */

}  // namespace blender::eevee
