/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_lightprobe_types.h"

#include "BKE_lightprobe.h"

#include "GPU_capabilities.h"
#include "GPU_debug.h"

#include "BLI_math_rotation.hh"

#include "eevee_instance.hh"

#include "eevee_irradiance_cache.hh"

namespace blender::eevee {

/* -------------------------------------------------------------------- */
/** \name Interface
 * \{ */

void IrradianceCache::init()
{
  display_grids_enabled_ = DRW_state_draw_support() &&
                           inst_.scene->eevee.flag & SCE_EEVEE_SHOW_IRRADIANCE;
}

void IrradianceCache::sync()
{
  if (inst_.is_baking()) {
    bake.sync();
  }
}

void IrradianceCache::viewport_draw(View &view, GPUFrameBuffer *view_fb)
{
  if (!inst_.is_baking()) {
    debug_pass_draw(view, view_fb);
    display_pass_draw(view, view_fb);
  }
}

void IrradianceCache::debug_pass_draw(View &view, GPUFrameBuffer *view_fb)
{
  switch (inst_.debug_mode) {
    case eDebugMode::DEBUG_IRRADIANCE_CACHE_SURFELS_NORMAL:
      inst_.info = "Debug Mode: Surfels Normal";
      break;
    case eDebugMode::DEBUG_IRRADIANCE_CACHE_SURFELS_IRRADIANCE:
      inst_.info = "Debug Mode: Surfels Irradiance";
      break;
    default:
      /* Nothing to display. */
      return;
  }

  for (const IrradianceGrid &grid : inst_.light_probes.grid_map_.values()) {
    if (grid.cache == nullptr) {
      continue;
    }

    LightProbeGridCacheFrame *cache = grid.cache->grid_static_cache;

    if (cache->surfels == nullptr || cache->surfels_len == 0) {
      continue;
    }

    debug_surfels_ps_.init();
    debug_surfels_ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH |
                                DRW_STATE_DEPTH_LESS_EQUAL);
    display_grids_ps_.framebuffer_set(&view_fb);
    debug_surfels_ps_.shader_set(inst_.shaders.static_shader_get(DEBUG_SURFELS));
    debug_surfels_ps_.push_constant("surfel_radius", 1.5f / 4.0f);
    debug_surfels_ps_.push_constant("debug_mode", static_cast<int>(inst_.debug_mode));

    debug_surfels_buf_.resize(cache->surfels_len);
    /* TODO(fclem): Cleanup: Could have a function in draw::StorageArrayBuffer that takes an input
     * data. */
    Span<Surfel> grid_surfels(static_cast<Surfel *>(cache->surfels), cache->surfels_len);
    MutableSpan<Surfel>(debug_surfels_buf_.data(), cache->surfels_len).copy_from(grid_surfels);
    debug_surfels_buf_.push_update();

    debug_surfels_ps_.bind_ssbo("surfels_buf", debug_surfels_buf_);
    debug_surfels_ps_.draw_procedural(GPU_PRIM_TRI_STRIP, cache->surfels_len, 4);

    inst_.manager->submit(debug_surfels_ps_, view);
  }
}

void IrradianceCache::display_pass_draw(View &view, GPUFrameBuffer *view_fb)
{
  if (!display_grids_enabled_) {
    return;
  }

  for (const IrradianceGrid &grid : inst_.light_probes.grid_map_.values()) {
    if (grid.cache == nullptr) {
      continue;
    }

    LightProbeGridCacheFrame *cache = grid.cache->grid_static_cache;

    if (cache == nullptr) {
      continue;
    }

    /* Display texture. Updated for each individual light grid to avoid increasing VRAM usage. */
    draw::Texture irradiance_a_tx_ = {"irradiance_a_tx"};
    draw::Texture irradiance_b_tx_ = {"irradiance_b_tx"};
    draw::Texture irradiance_c_tx_ = {"irradiance_c_tx"};
    draw::Texture irradiance_d_tx_ = {"irradiance_d_tx"};

    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ;
    int3 grid_size = int3(cache->size);
    if (cache->baking.L0) {
      irradiance_a_tx_.ensure_3d(GPU_RGBA16F, grid_size, usage, (float *)cache->baking.L0);
      irradiance_b_tx_.ensure_3d(GPU_RGBA16F, grid_size, usage, (float *)cache->baking.L1_a);
      irradiance_c_tx_.ensure_3d(GPU_RGBA16F, grid_size, usage, (float *)cache->baking.L1_b);
      irradiance_d_tx_.ensure_3d(GPU_RGBA16F, grid_size, usage, (float *)cache->baking.L1_c);
    }
    else if (cache->irradiance.L0) {
      irradiance_a_tx_.ensure_3d(GPU_RGB16F, grid_size, usage, (float *)cache->irradiance.L0);
      irradiance_b_tx_.ensure_3d(GPU_RGB16F, grid_size, usage, (float *)cache->irradiance.L1_a);
      irradiance_c_tx_.ensure_3d(GPU_RGB16F, grid_size, usage, (float *)cache->irradiance.L1_b);
      irradiance_d_tx_.ensure_3d(GPU_RGB16F, grid_size, usage, (float *)cache->irradiance.L1_c);
    }
    else {
      continue;
    }

    display_grids_ps_.init();
    display_grids_ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH |
                                DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_CULL_BACK);
    display_grids_ps_.framebuffer_set(&view_fb);
    display_grids_ps_.shader_set(inst_.shaders.static_shader_get(DISPLAY_PROBE_GRID));

    display_grids_ps_.push_constant("sphere_radius", 0.3f); /* TODO property */
    display_grids_ps_.push_constant("grid_resolution", grid_size);
    display_grids_ps_.push_constant("grid_to_world", grid.object_to_world);
    display_grids_ps_.push_constant("world_to_grid", grid.world_to_object);

    display_grids_ps_.bind_texture("irradiance_a_tx", &irradiance_a_tx_);
    display_grids_ps_.bind_texture("irradiance_b_tx", &irradiance_b_tx_);
    display_grids_ps_.bind_texture("irradiance_c_tx", &irradiance_c_tx_);
    display_grids_ps_.bind_texture("irradiance_d_tx", &irradiance_d_tx_);

    int sample_count = static_cast<int>(BKE_lightprobe_grid_cache_frame_sample_count(cache));
    int triangle_count = sample_count * 2;
    display_grids_ps_.draw_procedural(GPU_PRIM_TRIS, 1, triangle_count * 3);

    inst_.manager->submit(display_grids_ps_, view);

    irradiance_a_tx_.free();
    irradiance_b_tx_.free();
    irradiance_c_tx_.free();
    irradiance_d_tx_.free();
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Baking
 * \{ */

void IrradianceBake::sync()
{
  {
    PassSimple &pass = surfel_light_eval_ps_;
    pass.init();
    /* Apply lights contribution to scene surfel representation. */
    pass.shader_set(inst_.shaders.static_shader_get(SURFEL_LIGHT));
    pass.bind_ssbo(SURFEL_BUF_SLOT, &surfels_buf_);
    pass.bind_ssbo(CAPTURE_BUF_SLOT, &capture_info_buf_);
    pass.bind_texture(RBUFS_UTILITY_TEX_SLOT, inst_.pipelines.utility_tx);
    inst_.lights.bind_resources(&pass);
    inst_.shadows.bind_resources(&pass);
    /* Sync with the surfel creation stage. */
    pass.barrier(GPU_BARRIER_SHADER_STORAGE);
    pass.barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
    pass.barrier(GPU_BARRIER_TEXTURE_FETCH);
    pass.dispatch(&dispatch_per_surfel_);
  }
  {
    PassSimple &pass = surfel_light_propagate_ps_;
    pass.init();
    {
      PassSimple::Sub &sub = pass.sub("ListBuild");
      sub.shader_set(inst_.shaders.static_shader_get(SURFEL_LIST_BUILD));
      sub.bind_ssbo(SURFEL_BUF_SLOT, &surfels_buf_);
      sub.bind_ssbo(CAPTURE_BUF_SLOT, &capture_info_buf_);
      sub.bind_ssbo("list_start_buf", &list_start_buf_);
      sub.bind_ssbo("list_info_buf", &list_info_buf_);
      sub.barrier(GPU_BARRIER_SHADER_STORAGE);
      sub.dispatch(&dispatch_per_surfel_);
    }
    {
      PassSimple::Sub &sub = pass.sub("ListSort");
      sub.shader_set(inst_.shaders.static_shader_get(SURFEL_LIST_SORT));
      sub.bind_ssbo(SURFEL_BUF_SLOT, &surfels_buf_);
      sub.bind_ssbo(CAPTURE_BUF_SLOT, &capture_info_buf_);
      sub.bind_ssbo("list_start_buf", &list_start_buf_);
      sub.bind_ssbo("list_info_buf", &list_info_buf_);
      sub.barrier(GPU_BARRIER_SHADER_STORAGE);
      sub.dispatch(&dispatch_per_list_);
    }
    {
      PassSimple::Sub &sub = pass.sub("RayEval");
      sub.shader_set(inst_.shaders.static_shader_get(SURFEL_RAY));
      sub.bind_ssbo(SURFEL_BUF_SLOT, &surfels_buf_);
      sub.bind_ssbo(CAPTURE_BUF_SLOT, &capture_info_buf_);
      sub.barrier(GPU_BARRIER_SHADER_STORAGE);
      sub.dispatch(&dispatch_per_surfel_);
    }
  }
  {
    PassSimple &pass = irradiance_capture_ps_;
    pass.init();
    pass.shader_set(inst_.shaders.static_shader_get(LIGHTPROBE_IRRADIANCE_RAY));
    pass.bind_ssbo(SURFEL_BUF_SLOT, &surfels_buf_);
    pass.bind_ssbo(CAPTURE_BUF_SLOT, &capture_info_buf_);
    pass.bind_ssbo("list_start_buf", &list_start_buf_);
    pass.bind_ssbo("list_info_buf", &list_info_buf_);
    pass.bind_image("irradiance_L0_img", &irradiance_L0_tx_);
    pass.bind_image("irradiance_L1_a_img", &irradiance_L1_a_tx_);
    pass.bind_image("irradiance_L1_b_img", &irradiance_L1_b_tx_);
    pass.bind_image("irradiance_L1_c_img", &irradiance_L1_c_tx_);
    pass.barrier(GPU_BARRIER_SHADER_STORAGE | GPU_BARRIER_SHADER_IMAGE_ACCESS);
    pass.dispatch(&dispatch_per_grid_sample_);
  }
  {
    PassSimple &pass = surfel_light_bounce_ps_;
    pass.init();
    pass.shader_set(inst_.shaders.static_shader_get(SURFEL_BOUNCE));
    pass.bind_ssbo(SURFEL_BUF_SLOT, &surfels_buf_);
    pass.bind_ssbo(CAPTURE_BUF_SLOT, &capture_info_buf_);
    pass.barrier(GPU_BARRIER_SHADER_STORAGE);
    pass.dispatch(&dispatch_per_surfel_);
  }
}

void IrradianceBake::surfel_raster_views_sync(const Object &probe_object)
{
  using namespace blender::math;
  const float4x4 transform(probe_object.object_to_world);

  float3 scale;
  math::to_loc_rot_scale(transform, grid_location_, grid_orientation_, scale);

  grid_pixel_extent_ = max(int3(1), int3(surfel_density_ * 2.0f * scale));

  /* We could use multi-view rendering here to avoid multiple submissions but it is unlikely to
   * make any difference. The bottleneck is still the light propagation loop. */
  auto sync_view = [&](View &view, CartesianBasis basis) {
    float3 extent = transform_point(invert(basis), scale);
    float4x4 winmat = projection::orthographic(
        -extent.x, extent.x, -extent.y, extent.y, -extent.z, extent.z);
    float4x4 viewinv = math::from_loc_rot<float4x4>(
        grid_location_, grid_orientation_ * to_quaternion<float>(basis));
    view.sync(invert(viewinv), winmat);
  };

  sync_view(view_x_, basis_x_);
  sync_view(view_y_, basis_y_);
  sync_view(view_z_, basis_z_);
}

void IrradianceBake::surfels_create(const Object &probe_object)
{
  /**
   * We rasterize the scene along the 3 axes. Each generated fragment will write a surface element
   * so raster grid density need to match the desired surfel density. We do a first pass to know
   * how much surfel to allocate then render again to create the surfels.
   */
  using namespace blender::math;

  const ::LightProbe *lightprobe = static_cast<::LightProbe *>(probe_object.data);

  int3 grid_resolution = int3(&lightprobe->grid_resolution_x);
  float4x4 grid_local_to_world = invert(float4x4(probe_object.world_to_object));

  dispatch_per_grid_sample_ = math::divide_ceil(grid_resolution, int3(IRRADIANCE_GRID_GROUP_SIZE));
  capture_info_buf_.irradiance_grid_size = grid_resolution;
  capture_info_buf_.irradiance_grid_local_to_world = grid_local_to_world;
  capture_info_buf_.irradiance_grid_world_to_local_rotation = float4x4(
      (invert(normalize(float3x3(grid_local_to_world)))));
  capture_info_buf_.irradiance_accum_solid_angle = 0.0f;
  /* Divide by twice the sample count because each ray is evaluated in both directions. */
  capture_info_buf_.irradiance_sample_solid_angle = 4.0f * float(M_PI) /
                                                    (2 * inst_.sampling.sample_count());

  eGPUTextureUsage texture_usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_SHADER_WRITE |
                                   GPU_TEXTURE_USAGE_HOST_READ;

  /* 32bit float is needed here otherwise we loose too much energy from rounding error during the
   * accumulation when the sample count is above 500. */
  irradiance_L0_tx_.ensure_3d(GPU_RGBA32F, grid_resolution, texture_usage);
  irradiance_L1_a_tx_.ensure_3d(GPU_RGBA32F, grid_resolution, texture_usage);
  irradiance_L1_b_tx_.ensure_3d(GPU_RGBA32F, grid_resolution, texture_usage);
  irradiance_L1_c_tx_.ensure_3d(GPU_RGBA32F, grid_resolution, texture_usage);
  irradiance_L0_tx_.clear(float4(0.0f));
  irradiance_L1_a_tx_.clear(float4(0.0f));
  irradiance_L1_b_tx_.clear(float4(0.0f));
  irradiance_L1_c_tx_.clear(float4(0.0f));

  /* Extract bounding box. Order is arbitrary as it is not important for our usage. */
  const std::array<float3, 8> bbox_corners({float3{+1, +1, +1},
                                            float3{-1, +1, +1},
                                            float3{+1, -1, +1},
                                            float3{-1, -1, +1},
                                            float3{+1, +1, -1},
                                            float3{-1, +1, -1},
                                            float3{+1, -1, -1},
                                            float3{-1, -1, -1}});
  grid_bbox_vertices.clear();
  for (const float3 &point : bbox_corners) {
    grid_bbox_vertices.append(transform_point(grid_local_to_world, point));
  }

  DRW_stats_group_start("IrradianceBake.SurfelsCount");

  /* Raster the scene to query the number of surfel needed. */
  capture_info_buf_.do_surfel_count = true;
  capture_info_buf_.do_surfel_output = false;
  capture_info_buf_.surfel_len = 0u;
  capture_info_buf_.push_update();

  empty_raster_fb_.ensure(transform_point(invert(basis_x_), grid_pixel_extent_).xy());
  inst_.pipelines.capture.render(view_x_);
  empty_raster_fb_.ensure(transform_point(invert(basis_y_), grid_pixel_extent_).xy());
  inst_.pipelines.capture.render(view_y_);
  empty_raster_fb_.ensure(transform_point(invert(basis_z_), grid_pixel_extent_).xy());
  inst_.pipelines.capture.render(view_z_);

  DRW_stats_group_end();

  /* Allocate surfel pool. */
  GPU_memory_barrier(GPU_BARRIER_BUFFER_UPDATE);
  capture_info_buf_.read();
  if (capture_info_buf_.surfel_len == 0) {
    /* No surfel to allocated. */
    return;
  }

  /* TODO(fclem): Check for GL limit and abort if the surfel cache doesn't fit the GPU memory. */
  surfels_buf_.resize(capture_info_buf_.surfel_len);

  dispatch_per_surfel_.x = divide_ceil_u(surfels_buf_.size(), SURFEL_GROUP_SIZE);

  DRW_stats_group_start("IrradianceBake.SurfelsCreate");

  /* Raster the scene to generate the surfels. */
  capture_info_buf_.do_surfel_count = true;
  capture_info_buf_.do_surfel_output = true;
  capture_info_buf_.surfel_len = 0u;
  capture_info_buf_.push_update();

  empty_raster_fb_.ensure(transform_point(invert(basis_x_), grid_pixel_extent_).xy());
  inst_.pipelines.capture.render(view_x_);
  empty_raster_fb_.ensure(transform_point(invert(basis_y_), grid_pixel_extent_).xy());
  inst_.pipelines.capture.render(view_y_);
  empty_raster_fb_.ensure(transform_point(invert(basis_z_), grid_pixel_extent_).xy());
  inst_.pipelines.capture.render(view_z_);

  /* Sync with any other following pass using the surfel buffer. */
  GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);

  DRW_stats_group_end();
}

void IrradianceBake::surfels_lights_eval()
{
  /* Use the last setup view. This should work since the view is orthographic. */
  /* TODO(fclem): Remove this. It is only present to avoid crash inside `shadows.set_view` */
  inst_.render_buffers.acquire(int2(1));
  inst_.lights.set_view(view_z_, grid_pixel_extent_.xy());
  /* TODO: Instead of using the volume tagging we should tag using the surfels. */
  inst_.shadows.set_view(view_z_);
  inst_.render_buffers.release();

  inst_.manager->submit(surfel_light_eval_ps_, view_z_);
}

void IrradianceBake::propagate_light_sample()
{
  using namespace blender::math;

  float2 rand_uv = inst_.sampling.rng_2d_get(eSamplingDimension::SAMPLING_FILTER_U);
  const float3 ray_direction = inst_.sampling.sample_hemisphere(rand_uv);
  const float3 up = ray_direction;
  /* Find the closest axis. */
  const float3 grid_local_ray_direction = transform_point(grid_orientation_, ray_direction);
  Axis closest_grid_axis = Axis::from_int(dominant_axis(grid_local_ray_direction));
  /* Use one of the other 2 grid axes to get a reference right vector. */
  Axis right_axis = AxisSigned(closest_grid_axis).next_after().axis();
  const float3 grid_right = from_rotation<float3x3>(grid_orientation_)[right_axis.as_int()];
  /* Create a view around the grid position with the ray direction as up axis.
   * The other axes are aligned to the grid local axes to avoid to allocate too many list start. */
  const float4x4 viewmat = invert(
      from_orthonormal_axes<float4x4>(grid_location_, normalize(cross(up, grid_right)), up));

  /* Compute projection bounds. */
  float2 min, max;
  INIT_MINMAX2(min, max);
  for (const float3 &point : grid_bbox_vertices) {
    min_max(transform_point(viewmat, point).xy(), min, max);
  }

  /* NOTE: Z values do not really matter since we are not doing any rasterization. */
  const float4x4 winmat = projection::orthographic<float>(min.x, max.x, min.y, max.y, 0, 1);

  View ray_view = {"RayProjectionView"};
  ray_view.sync(viewmat, winmat);

  /* This avoid light leaking by making sure that for one surface there will always be at least 1
   * surfel capture inside a ray list. Since the surface with the maximum distance (after
   * projection) between adjacent surfels is a slope that goes through 3 corners of a cube,
   * the distance the grid needs to cover is the diagonal of a cube face.
   *
   * The lower the number the more surfels it clumps together in the same surfel-list.
   * Biasing the grid_density like that will create many invalid link between coplanar surfels.
   * These are dealt with during the list sorting pass.
   *
   * We add an extra epsilon just in case. We really need this step to be leak free. */
  const float max_distance_between_neighbor_surfels_inv = M_SQRT1_2 - 1e-4;
  const float ray_grid_density = surfel_density_ * max_distance_between_neighbor_surfels_inv;
  list_info_buf_.ray_grid_size = math::max(int2(1), int2(ray_grid_density * (max - min)));
  list_info_buf_.list_max = list_info_buf_.ray_grid_size.x * list_info_buf_.ray_grid_size.y;
  list_info_buf_.push_update();

  dispatch_per_list_.x = divide_ceil_u(list_info_buf_.list_max, SURFEL_LIST_GROUP_SIZE);

  list_start_buf_.resize(ceil_to_multiple_u(list_info_buf_.list_max, 4));

  GPU_storagebuf_clear(list_start_buf_, -1);
  inst_.manager->submit(surfel_light_propagate_ps_, ray_view);
  inst_.manager->submit(irradiance_capture_ps_, ray_view);
}

void IrradianceBake::accumulate_bounce()
{
  inst_.manager->submit(surfel_light_bounce_ps_);
}

void IrradianceBake::read_surfels(LightProbeGridCacheFrame *cache_frame)
{
  if (!ELEM(inst_.debug_mode,
            eDebugMode::DEBUG_IRRADIANCE_CACHE_SURFELS_NORMAL,
            eDebugMode::DEBUG_IRRADIANCE_CACHE_SURFELS_IRRADIANCE)) {
    return;
  }

  GPU_memory_barrier(GPU_BARRIER_BUFFER_UPDATE);
  capture_info_buf_.read();
  surfels_buf_.read();

  cache_frame->surfels_len = capture_info_buf_.surfel_len;
  cache_frame->surfels = MEM_malloc_arrayN(cache_frame->surfels_len, sizeof(Surfel), __func__);

  MutableSpan<Surfel> surfels_dst((Surfel *)cache_frame->surfels, cache_frame->surfels_len);
  Span<Surfel> surfels_src(surfels_buf_.data(), cache_frame->surfels_len);
  surfels_dst.copy_from(surfels_src);
}

LightProbeGridCacheFrame *IrradianceBake::read_result_unpacked()
{
  LightProbeGridCacheFrame *cache_frame = BKE_lightprobe_grid_cache_frame_create();

  read_surfels(cache_frame);

  cache_frame->size[0] = irradiance_L0_tx_.width();
  cache_frame->size[1] = irradiance_L0_tx_.height();
  cache_frame->size[2] = irradiance_L0_tx_.depth();

  GPU_memory_barrier(GPU_BARRIER_TEXTURE_UPDATE);

  cache_frame->baking.L0 = (float(*)[4])irradiance_L0_tx_.read<float4>(GPU_DATA_FLOAT);
  cache_frame->baking.L1_a = (float(*)[4])irradiance_L1_a_tx_.read<float4>(GPU_DATA_FLOAT);
  cache_frame->baking.L1_b = (float(*)[4])irradiance_L1_b_tx_.read<float4>(GPU_DATA_FLOAT);
  cache_frame->baking.L1_c = (float(*)[4])irradiance_L1_c_tx_.read<float4>(GPU_DATA_FLOAT);

  /* TODO(fclem): Connectivity. */
  // cache_frame->connectivity.bitmask = connectivity_tx_.read<uint8_t>(GPU_DATA_FLOAT);

  return cache_frame;
}

LightProbeGridCacheFrame *IrradianceBake::read_result_packed()
{
  LightProbeGridCacheFrame *cache_frame = BKE_lightprobe_grid_cache_frame_create();

  read_surfels(cache_frame);

  cache_frame->size[0] = irradiance_L0_tx_.width();
  cache_frame->size[1] = irradiance_L0_tx_.height();
  cache_frame->size[2] = irradiance_L0_tx_.depth();

  GPU_memory_barrier(GPU_BARRIER_TEXTURE_UPDATE);

  /* TODO(fclem): Temp. */
  cache_frame->baking.L0 = (float(*)[4])irradiance_L0_tx_.read<float4>(GPU_DATA_FLOAT);
  cache_frame->baking.L1_a = (float(*)[4])irradiance_L1_a_tx_.read<float4>(GPU_DATA_FLOAT);
  cache_frame->baking.L1_b = (float(*)[4])irradiance_L1_b_tx_.read<float4>(GPU_DATA_FLOAT);
  cache_frame->baking.L1_c = (float(*)[4])irradiance_L1_c_tx_.read<float4>(GPU_DATA_FLOAT);

  /* TODO(fclem): Create packed format texture and swizzle on gpu. */
  // cache_frame->irradiance.L0 = (float(*)[4])irradiance_only_L0_tx_.read<float4>(GPU_DATA_FLOAT);
  // cache_frame->irradiance.L1_a =
  // (float(*)[4])irradiance_only_L1_a_tx_.read<float4>(GPU_DATA_FLOAT);
  // cache_frame->irradiance.L1_b =
  // (float(*)[4])irradiance_only_L1_b_tx_.read<float4>(GPU_DATA_FLOAT);
  // cache_frame->irradiance.L1_c =
  // (float(*)[4])irradiance_only_L1_c_tx_.read<float4>(GPU_DATA_FLOAT);

  // cache_frame->visibility.L0 = irradiance_only_L0_tx_.read<uint8_t>(GPU_DATA_UBYTE);
  // cache_frame->visibility.L1_a = irradiance_only_L1_a_tx_.read<uint8_t>(GPU_DATA_UBYTE);
  // cache_frame->visibility.L1_b = irradiance_only_L1_b_tx_.read<uint8_t>(GPU_DATA_UBYTE);
  // cache_frame->visibility.L1_c = irradiance_only_L1_c_tx_.read<uint8_t>(GPU_DATA_UBYTE);

  /* TODO(fclem): Connectivity. */
  // cache_frame->connectivity.bitmask = connectivity_tx_.read<uint8_t>(GPU_DATA_FLOAT);

  return cache_frame;
}

/** \} */

}  // namespace blender::eevee
