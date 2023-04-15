/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 */

#pragma once

#include "DNA_lightprobe_types.h"

#include "BLI_math_quaternion_types.hh"

#include "eevee_lightprobe.hh"
#include "eevee_shader_shared.hh"

namespace blender::eevee {

class Instance;
class CapturePipeline;
class ShadowModule;
class Camera;

/**
 * Baking related pass and data. Not used at runtime.
 */
class IrradianceBake {
  friend CapturePipeline;
  friend ShadowModule;
  friend Camera;

 private:
  Instance &inst_;

  /** Light cache being baked. */
  LightCache *light_cache_ = nullptr;
  /** Surface elements that represent the scene. */
  SurfelBuf surfels_buf_;
  /** Capture state. */
  CaptureInfoBuf capture_info_buf_;
  /** Framebuffer. */
  Framebuffer empty_raster_fb_ = {"empty_raster_fb_"};
  /** Evaluate light object contribution and store result to surfel. */
  PassSimple surfel_light_eval_ps_ = {"LightEval"};
  /** Propagate light from surfel to surfel. */
  PassSimple surfel_light_propagate_ps_ = {"LightPropagate"};
  /** Start of a light bounce. Accumulate light from previous propagation. */
  PassSimple surfel_light_bounce_ps_ = {"LightBounce"};
  /** Capture surfel lighting to irradiance samples. */
  PassSimple irradiance_capture_ps_ = {"IrradianceCapture"};

  /**
   * Basis orientation for each baking projection.
   * Note that this is the view orientation. The projection matrix will take the negative Z axis
   * as forward and Y as up. */
  math::CartesianBasis basis_x_ = {
      math::AxisSigned::Y_POS, math::AxisSigned::Z_POS, math::AxisSigned::X_POS};
  math::CartesianBasis basis_y_ = {
      math::AxisSigned::X_POS, math::AxisSigned::Z_POS, math::AxisSigned::Y_NEG};
  math::CartesianBasis basis_z_ = {
      math::AxisSigned::X_POS, math::AxisSigned::Y_POS, math::AxisSigned::Z_POS};
  /** Views for each baking projection. */
  View view_x_ = {"BakingViewX"};
  View view_y_ = {"BakingViewY"};
  View view_z_ = {"BakingViewZ"};
  /** Pixel resolution in each of the projection axes. Match the target surfel density. */
  int3 grid_pixel_extent_ = int3(0);
  /** Information for surfel list building. */
  SurfelListInfoBuf list_info_buf_ = {"list_info_buf_"};
  /** List array containing list start surfel index. Cleared to -1. */
  StorageArrayBuffer<int, 16, true> list_start_buf_ = {"list_start_buf_"};

  /* Dispatch size for per surfel workload. */
  int3 dispatch_per_surfel_ = int3(1);
  /* Dispatch size for per surfel list workload. */
  int3 dispatch_per_list_ = int3(1);
  /* Dispatch size for per grid sample workload. */
  int3 dispatch_per_grid_sample_ = int3(1);

  /** Irradiance textures for baking. Only represents one grid in there. */
  Texture irradiance_L0_tx_ = {"irradiance_L0_tx_"};
  Texture irradiance_L1_a_tx_ = {"irradiance_L1_a_tx_"};
  Texture irradiance_L1_b_tx_ = {"irradiance_L1_b_tx_"};
  Texture irradiance_L1_c_tx_ = {"irradiance_L1_c_tx_"};

  /* Orientation of the irradiance grid being baked. */
  math::Quaternion grid_orientation_;
  /* Object center of the irradiance grid being baked. */
  float3 grid_location_;
  /* Bounding box vertices of the irradiance grid being baked. In world space. */
  Vector<float3> grid_bbox_vertices;
  /* Surfel per unit distance. */
  float surfel_density_ = 2.0f;

 public:
  IrradianceBake(Instance &inst) : inst_(inst){};

  void sync();

  /** Create the views used to rasterize the scene into surfel representation. */
  void surfel_raster_views_sync(const Object &probe_object);
  /** Create a surfel representation of the scene from the probe using the capture pipeline. */
  void surfels_create(const Object &probe_object);
  /** Evaluate direct lighting (and also clear the surfels radiance). */
  void surfels_lights_eval();
  /** Propagate light from surfel to surfel in a random direction over the sphere. */
  void propagate_light_sample();
  /** Accumulate light inside `surfel.radiance_bounce` to `surfel.radiance`. */
  void accumulate_bounce();

  /** Read grid unpacked irradiance back to CPU and returns as a #LightProbeGridCacheFrame. */
  LightProbeGridCacheFrame *read_result_unpacked();
  /** Read grid packed irradiance back to CPU and returns as a #LightProbeGridCacheFrame. */
  LightProbeGridCacheFrame *read_result_packed();

 private:
  /** Read surfel data back to CPU into \a cache_frame . */
  void read_surfels(LightProbeGridCacheFrame *cache_frame);
};

/**
 * Runtime container of diffuse indirect lighting.
 * Also have debug and baking components.
 */
class IrradianceCache {
 public:
  IrradianceBake bake;

 private:
  Instance &inst_;

  /** Display surfel debug data. */
  PassSimple debug_surfels_ps_ = {"IrradianceCache.Debug"};
  /** Debug surfel elements copied from the light cache. */
  draw::StorageArrayBuffer<Surfel> debug_surfels_buf_;

  /** Display grid cache data. */
  bool display_grids_enabled_ = false;
  PassSimple display_grids_ps_ = {"IrradianceCache.Display Grids"};

 public:
  IrradianceCache(Instance &inst) : bake(inst), inst_(inst){};
  ~IrradianceCache(){};

  void init();
  void sync();
  void viewport_draw(View &view, GPUFrameBuffer *view_fb);

 private:
  void debug_pass_draw(View &view, GPUFrameBuffer *view_fb);
  void display_pass_draw(View &view, GPUFrameBuffer *view_fb);
};

}  // namespace blender::eevee
