/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 *
 * Volumetric effects rendering using Frostbite's Physically-based & Unified Volumetric Rendering
 * approach.
 * https://www.ea.com/frostbite/news/physically-based-unified-volumetric-rendering-in-frostbite
 *
 */

#include "DRW_render.h"

#include "BLI_listbase.h"
#include "BLI_rand.h"
#include "BLI_string_utils.h"

#include "DNA_fluid_types.h"
#include "DNA_object_force_types.h"
#include "DNA_volume_types.h"
#include "DNA_world_types.h"

#include "BKE_attribute_math.hh"
#include "BKE_fluid.h"
#include "BKE_global.h"
#include "BKE_mesh.h"
#include "BKE_modifier.h"
#include "BKE_volume.h"
#include "BKE_volume_render.h"

#include "ED_screen.h"

#include "DEG_depsgraph_query.h"

#include "GPU_capabilities.h"
#include "GPU_context.h"
#include "GPU_material.h"
#include "GPU_texture.h"

#include "draw_common.hh"

#include "eevee_instance.hh"
#include "eevee_pipeline.hh"
#include "eevee_shader.hh"

#include "eevee_volume.hh"

#define LOOK_DEV_STUDIO_LIGHT_ENABLED(v3d) \
  ((v3d) && (((v3d->shading.type == OB_MATERIAL) && \
              ((v3d->shading.flag & V3D_SHADING_SCENE_WORLD) == 0)) || \
             ((v3d->shading.type == OB_RENDER) && \
              ((v3d->shading.flag & V3D_SHADING_SCENE_WORLD_RENDER) == 0))))

namespace blender::eevee {

bool VolumeModule::GridAABB::init(Object *ob, const Camera &camera, const VolumesInfoDataBuf &data)
{
  /* Returns the unified volume grid cell index of a world space coordinate. */
  auto to_global_grid_coords = [&](float3 wP) -> int3 {
    const float4x4 &view_matrix = camera.data_get().viewmat;
    const float4x4 &perspective_matrix = camera.data_get().winmat * view_matrix;

    /** NOTE: Keep in sync with ndc_to_volume. */
    float view_z = math::transform_point(view_matrix, wP).z;

    float volume_z;
    if (camera.is_orthographic()) {
      volume_z = (view_z - data.depth_near) * data.depth_distribution;
    }
    else {
      volume_z = data.depth_distribution * log2(view_z * data.depth_far + data.depth_near);
    }

    float3 grid_coords = math::project_point(perspective_matrix, wP);
    grid_coords = (grid_coords * 0.5f) + float3(0.5f);

    grid_coords.x *= data.coord_scale.x;
    grid_coords.y *= data.coord_scale.y;
    grid_coords.z = volume_z;

    return int3(grid_coords * float3(data.tex_size));
  };

  const BoundBox &bbox = *BKE_object_boundbox_get(ob);
  min = int3(INT32_MAX);
  max = int3(INT32_MIN);

  for (float3 corner : bbox.vec) {
    corner = math::transform_point(float4x4(ob->object_to_world), corner);
    int3 grid_coord = to_global_grid_coords(corner);
    min = math::min(min, grid_coord);
    max = math::max(max, grid_coord);
  }

  bool is_visible = false;
  for (int i : IndexRange(3)) {
    is_visible = is_visible || (min[i] >= 0 && min[i] < data.tex_size[i]);
    is_visible = is_visible || (max[i] >= 0 && max[i] < data.tex_size[i]);
  }

  min = math::clamp(min, int3(0), data.tex_size);
  max = math::clamp(max, int3(0), data.tex_size);

  return is_visible;
}

bool VolumeModule::GridAABB::overlaps(const GridAABB &aabb)
{
  for (int i : IndexRange(3)) {
    if (min[i] > aabb.max[i] || max[i] < aabb.min[i]) {
      return false;
    }
  }
  return true;
}

void VolumeModule::init()
{
  enabled_ = false;
  subpass_aabbs_.clear();

  const Scene *scene_eval = inst_.scene;

  const float2 viewport_size = float2(inst_.film.render_extent_get());
  const int tile_size = scene_eval->eevee.volumetric_tile_size;

  /* Find Froxel Texture resolution. */
  int3 tex_size = int3(math::ceil(math::max(float2(1.0f), viewport_size / float(tile_size))), 0);
  tex_size.z = std::max(1, scene_eval->eevee.volumetric_samples);

  /* Clamp 3D texture size based on device maximum. */
  int3 max_size = int3(GPU_max_texture_3d_size());
  BLI_assert(tex_size == math::min(tex_size, max_size));
  tex_size = math::min(tex_size, max_size);

  data_.coord_scale = viewport_size / float2(tile_size * tex_size);
  data_.viewport_size_inv = 1.0f / viewport_size;

  /* TODO: compute snap to maxZBuffer for clustered rendering. */
  if (data_.tex_size != tex_size) {
    data_.tex_size = tex_size;
    data_.inv_tex_size = 1.0f / float3(tex_size);
  }

  data_.jitter = inst_.sampling.rng_3d_get(eSamplingDimension::SAMPLING_TRANSPARENCY);

  if ((scene_eval->eevee.flag & SCE_EEVEE_VOLUMETRIC_SHADOWS) == 0) {
    data_.shadow_steps = 0;
  }
  else {
    data_.shadow_steps = float(scene_eval->eevee.volumetric_shadow_samples);
  }

  data_.use_lights = (scene_eval->eevee.flag & SCE_EEVEE_VOLUMETRIC_LIGHTS) != 0;
  data_.use_soft_shadows = (scene_eval->eevee.flag & SCE_EEVEE_SHADOW_SOFT) != 0;

  data_.light_clamp = scene_eval->eevee.volumetric_light_clamp;
}

void VolumeModule::begin_sync()
{
  const Scene *scene_eval = inst_.scene;

  /* Negate clip values (View matrix forward vector is -Z). */
  const float clip_start = -inst_.camera.data_get().clip_near;
  const float clip_end = -inst_.camera.data_get().clip_far;
  float integration_start = scene_eval->eevee.volumetric_start;
  float integration_end = scene_eval->eevee.volumetric_end;

  if (inst_.camera.is_perspective()) {
    float sample_distribution = scene_eval->eevee.volumetric_sample_distribution;
    sample_distribution = 4.0f * std::max(1.0f - sample_distribution, 1e-2f);

    float near = integration_start = std::min(-integration_start, clip_start - 1e-4f);
    float far = integration_end = std::min(-integration_end, near - 1e-4f);

    data_.depth_near = (far - near * exp2(1.0f / sample_distribution)) / (far - near);
    data_.depth_far = (1.0f - data_.depth_near) / near;
    data_.depth_distribution = sample_distribution;
  }
  else {
    integration_start = std::min(integration_end, clip_start);
    integration_end = std::max(-integration_end, clip_end);

    data_.depth_near = integration_start;
    data_.depth_far = integration_end;
    data_.depth_distribution = 1.0f / (integration_end - integration_start);
  }

  data_.push_update();

  GPUMaterial *material = nullptr;

  ::World *world = inst_.scene->world;
  if (world && world->use_nodes && world->nodetree && !inst_.use_studio_light()) {
    material = inst_.shaders.world_shader_get(world, world->nodetree, MAT_PIPE_VOLUME);

    if (!GPU_material_has_volume_output(material)) {
      material = nullptr;
    }
  }

  enabled_ = material != nullptr;
  inst_.pipelines.world_volume.sync(material);
}

void VolumeModule::sync_object(Object *ob, ObjectHandle & /*ob_handle*/, ResourceHandle res_handle)
{
  ::Material *material = BKE_object_material_get(ob, VOLUME_MATERIAL_NR);
  if (material == nullptr) {
    if (ob->type == OB_VOLUME) {
      material = BKE_material_default_volume();
    }
    else {
      return;
    }
  }

  float3x3 world_matrix = float3x3(float4x4(ob->object_to_world));
  float3 size = math::to_scale(world_matrix);
  /* Check if any of the axes have 0 length. (see #69070) */
  const float epsilon = 1e-8f;
  if (size.x < epsilon || size.y < epsilon || size.z < epsilon) {
    return;
  }

  MaterialPass material_pass = inst_.materials.material_pass_get(
      ob, material, MAT_PIPE_VOLUME, MAT_GEOM_VOLUME_OBJECT);

  /* If shader failed to compile or is currently compiling. */
  if (material_pass.gpumat == nullptr) {
    return;
  }

  GPUShader *shader = GPU_material_get_shader(material_pass.gpumat);
  if (shader == nullptr) {
    return;
  }

  GridAABB aabb;
  if (!aabb.init(ob, inst_.camera, data_)) {
    return;
  }

  PassMain::Sub &ps = *material_pass.sub_pass;
  if (volume_sub_pass(ps, inst_.scene, ob, material_pass.gpumat)) {
    enabled_ = true;

    /* Add a barrier at the start of a subpass or when 2 volumes overlaps. */
    if (!subpass_aabbs_.contains_as(shader)) {
      ps.barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
      subpass_aabbs_.add(shader, {aabb});
    }
    else {
      Vector<GridAABB> &aabbs = subpass_aabbs_.lookup(shader);
      for (GridAABB &_aabb : aabbs) {
        if (aabb.overlaps(_aabb)) {
          ps.barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
          aabbs.clear();
          break;
        }
      }
      aabbs.append(aabb);
    }

    int3 grid_size = aabb.max - aabb.min + int3(1);

    ps.push_constant("drw_ResourceID", int(res_handle.resource_index()));
    ps.push_constant("grid_coords_min", aabb.min);
    ps.dispatch(math::divide_ceil(grid_size, int3(VOLUME_GROUP_SIZE)));
  }
}

void VolumeModule::end_sync()
{
  if (!enabled_) {
    prop_scattering_tx_.free();
    prop_extinction_tx_.free();
    prop_emission_tx_.free();
    prop_phase_tx_.free();
    scatter_tx_.free();
    extinction_tx_.free();
    integrated_scatter_tx_.free();
    integrated_transmit_tx_.free();

    transparent_pass_scatter_tx_ = dummy_scatter_tx_;
    transparent_pass_transmit_tx_ = dummy_transmit_tx_;

    return;
  }

  eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_SHADER_WRITE |
                           GPU_TEXTURE_USAGE_ATTACHMENT;

  prop_scattering_tx_.ensure_3d(GPU_R11F_G11F_B10F, data_.tex_size, usage);
  prop_extinction_tx_.ensure_3d(GPU_R11F_G11F_B10F, data_.tex_size, usage);
  prop_emission_tx_.ensure_3d(GPU_R11F_G11F_B10F, data_.tex_size, usage);
  prop_phase_tx_.ensure_3d(GPU_RG16F, data_.tex_size, usage);

  scatter_tx_.ensure_3d(GPU_R11F_G11F_B10F, data_.tex_size, usage);
  extinction_tx_.ensure_3d(GPU_R11F_G11F_B10F, data_.tex_size, usage);

  integrated_scatter_tx_.ensure_3d(GPU_R11F_G11F_B10F, data_.tex_size, usage);
  integrated_transmit_tx_.ensure_3d(GPU_R11F_G11F_B10F, data_.tex_size, usage);

  transparent_pass_scatter_tx_ = integrated_scatter_tx_;
  transparent_pass_transmit_tx_ = integrated_transmit_tx_;

  scatter_ps_.init();
  scatter_ps_.shader_set(inst_.shaders.static_shader_get(
      data_.use_lights ? VOLUME_SCATTER_WITH_LIGHTS : VOLUME_SCATTER));
  inst_.lights.bind_resources(&scatter_ps_);
  inst_.shadows.bind_resources(&scatter_ps_);
  scatter_ps_.bind_image("in_scattering_img", &prop_scattering_tx_);
  scatter_ps_.bind_image("in_extinction_img", &prop_extinction_tx_);
  scatter_ps_.bind_image("in_emission_img", &prop_emission_tx_);
  scatter_ps_.bind_image("in_phase_img", &prop_phase_tx_);
  scatter_ps_.bind_image("out_scattering_img", &scatter_tx_);
  scatter_ps_.bind_image("out_extinction_img", &extinction_tx_);
  /* Sync with the property pass. */
  scatter_ps_.barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
  scatter_ps_.dispatch(math::divide_ceil(data_.tex_size, int3(VOLUME_GROUP_SIZE)));

  integration_ps_.init();
  integration_ps_.shader_set(inst_.shaders.static_shader_get(VOLUME_INTEGRATION));
  integration_ps_.bind_ubo(VOLUMES_INFO_BUF_SLOT, data_);
  integration_ps_.bind_texture("in_scattering_tx", &scatter_tx_);
  integration_ps_.bind_texture("in_extinction_tx", &extinction_tx_);
  integration_ps_.bind_image("out_scattering_img", &integrated_scatter_tx_);
  integration_ps_.bind_image("out_transmittance_img", &integrated_transmit_tx_);
  /* Sync with the scatter pass. */
  integration_ps_.barrier(GPU_BARRIER_TEXTURE_FETCH);
  integration_ps_.dispatch(
      math::divide_ceil(int2(data_.tex_size), int2(VOLUME_INTEGRATION_GROUP_SIZE)));

  resolve_ps_.init();
  resolve_ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_CUSTOM);
  resolve_ps_.shader_set(inst_.shaders.static_shader_get(VOLUME_RESOLVE));
  this->bind_resources(resolve_ps_);
  resolve_ps_.bind_texture("depth_tx", &inst_.render_buffers.depth_tx);
  /* Sync with the integration pass. */
  resolve_ps_.barrier(GPU_BARRIER_TEXTURE_FETCH);
  resolve_ps_.draw_procedural(GPU_PRIM_TRIS, 1, 3);
}

void VolumeModule::draw_compute(View &view)
{
  if (!enabled_) {
    return;
  }

  DRW_stats_group_start("Volumes");

  inst_.pipelines.world_volume.render(view);
  inst_.pipelines.volume.render(view);

  inst_.manager->submit(scatter_ps_, view);

  inst_.manager->submit(integration_ps_, view);

  DRW_stats_group_end();
}

void VolumeModule::draw_resolve(View &view)
{
  if (!enabled_) {
    return;
  }

  resolve_fb_.ensure(GPU_ATTACHMENT_NONE,
                     GPU_ATTACHMENT_TEXTURE(inst_.render_buffers.combined_tx));
  resolve_fb_.bind();
  inst_.manager->submit(resolve_ps_, view);
}

}  // namespace blender::eevee

/** \} */
