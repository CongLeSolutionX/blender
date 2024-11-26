/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef GPU_SHADER
#  pragma once
#  include "gpu_glsl_cpp_stubs.hh"

#  include "draw_object_infos_info.hh"
#  include "draw_view_info.hh"
#  include "eevee_shader_shared.hh"
#endif

#include "eevee_defines.hh"
#include "gpu_shader_create_info.hh"

/* -------------------------------------------------------------------- */
/** \name Common
 * \{ */

/* TODO(@fclem): This is a bit out of place at the moment. */
GPU_SHADER_CREATE_INFO(eevee_shared)
TYPEDEF_SOURCE("eevee_defines.hh")
TYPEDEF_SOURCE("eevee_shader_shared.hh")
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(eevee_global_ubo)
UNIFORM_BUF(UNIFORM_BUF_SLOT, UniformData, uniform_buf)
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(eevee_hiz_data)
SAMPLER(HIZ_TEX_SLOT, FLOAT_2D, hiz_tx)
ADDITIONAL_INFO(eevee_global_ubo)
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(eevee_sampling_data)
DEFINE("EEVEE_SAMPLING_DATA")
ADDITIONAL_INFO(eevee_shared)
STORAGE_BUF(SAMPLING_BUF_SLOT, READ, SamplingData, sampling_buf)
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(eevee_utility_texture)
DEFINE("EEVEE_UTILITY_TX")
SAMPLER(RBUFS_UTILITY_TEX_SLOT, FLOAT_2D_ARRAY, utility_tx)
GPU_SHADER_CREATE_END()

GPU_SHADER_NAMED_INTERFACE_INFO(eevee_clip_plane_iface, clip_interp)
SMOOTH(FLOAT, clip_distance)
GPU_SHADER_NAMED_INTERFACE_END(clip_interp)

GPU_SHADER_CREATE_INFO(eevee_clip_plane)
VERTEX_OUT(eevee_clip_plane_iface)
UNIFORM_BUF(CLIP_PLANE_BUF, ClipPlaneData, clip_plane)
DEFINE("MAT_CLIP_PLANE")
GPU_SHADER_CREATE_END()

/** \} */
