/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "gpu_shader_create_info.hh"

#pragma once

/* Used for shaders that need the final accumulated volume transmittance and scaterring. */
GPU_SHADER_CREATE_INFO(eevee_volume_lib)
    .additional_info("eevee_shared")
    .additional_info("draw_view")
    .uniform_buf(VOLUMES_INFO_BUF_SLOT, "VolumesInfoData", "volumes_info_buf")
    .sampler(VOLUME_SCATTERING_TEX_SLOT, ImageType::FLOAT_3D, "volume_scattering_tx")
    .sampler(VOLUME_TRANSMITTANCE_TEX_SLOT, ImageType::FLOAT_3D, "volume_transmittance_tx");

GPU_SHADER_CREATE_INFO(eevee_volume_scatter)
    .additional_info("eevee_shared")
    .additional_info("draw_resource_id_varying")
    .additional_info("draw_view")
    .additional_info("eevee_light_data")
    .additional_info("eevee_shadow_data")
    .compute_source("eevee_volume_scatter_comp.glsl")
    .local_group_size(VOLUME_GROUP_SIZE, VOLUME_GROUP_SIZE, VOLUME_GROUP_SIZE)
    .define("VOLUME_SHADOW")
    .uniform_buf(VOLUMES_INFO_BUF_SLOT, "VolumesInfoData", "volumes_info_buf")
    /* Inputs. */
    .image(0, GPU_R11F_G11F_B10F, Qualifier::READ, ImageType::FLOAT_3D, "in_scattering_img")
    .image(1, GPU_R11F_G11F_B10F, Qualifier::READ, ImageType::FLOAT_3D, "in_extinction_img")
    .image(2, GPU_R11F_G11F_B10F, Qualifier::READ, ImageType::FLOAT_3D, "in_emission_img")
    .image(3, GPU_RG16F, Qualifier::READ, ImageType::FLOAT_3D, "in_phase_img")
    /* Outputs. */
    .image(4, GPU_R11F_G11F_B10F, Qualifier::WRITE, ImageType::FLOAT_3D, "out_scattering_img")
    .image(5, GPU_R11F_G11F_B10F, Qualifier::WRITE, ImageType::FLOAT_3D, "out_extinction_img")
    .do_static_compilation(true);

GPU_SHADER_CREATE_INFO(eevee_volume_scatter_with_lights)
    .additional_info("eevee_volume_scatter")
    .define("VOLUME_LIGHTING")
    .do_static_compilation(true);

GPU_SHADER_CREATE_INFO(eevee_volume_integration)
    .additional_info("eevee_shared")
    .additional_info("draw_view")
    .compute_source("eevee_volume_integration_comp.glsl")
    .local_group_size(VOLUME_INTEGRATION_GROUP_SIZE, VOLUME_INTEGRATION_GROUP_SIZE, 1)
    .uniform_buf(VOLUMES_INFO_BUF_SLOT, "VolumesInfoData", "volumes_info_buf")
    /* Inputs. */
    .sampler(0, ImageType::FLOAT_3D, "in_scattering_tx")
    .sampler(1, ImageType::FLOAT_3D, "in_extinction_tx")
    /* Outputs. */
    .image(0, GPU_R11F_G11F_B10F, Qualifier::WRITE, ImageType::FLOAT_3D, "out_scattering_img")
    .image(1, GPU_R11F_G11F_B10F, Qualifier::WRITE, ImageType::FLOAT_3D, "out_transmittance_img")
    .do_static_compilation(true);

GPU_SHADER_CREATE_INFO(eevee_volume_resolve_opaque)
    .additional_info("eevee_shared")
    .additional_info("eevee_volume_lib")
    .additional_info("draw_fullscreen")
    .fragment_source("eevee_volume_resolve_frag.glsl")
    .sampler(0, ImageType::DEPTH_2D, "depth_tx")
    .fragment_out(0, Type::VEC4, "out_radiance", DualBlend::SRC_0)
    .fragment_out(0, Type::VEC4, "out_transmittance", DualBlend::SRC_1)
    .do_static_compilation(true);
