#include "eevee_defines.hh"
#include "gpu_shader_create_info.hh"

/* -------------------------------------------------------------------- */
/** \name Shared
 * \{ */

GPU_SHADER_CREATE_INFO(eevee_reflection_probe_world_data)
    .sampler(REFLECTION_PROBE_TEX_SLOT, ImageType::FLOAT_CUBE_ARRAY, "reflectionProbes");

GPU_SHADER_CREATE_INFO(eevee_reflection_probe_data)
    .additional_info("eevee_reflection_probe_world_data")
    .storage_buf(REFLECTION_PROBE_BUF_SLOT,
                 Qualifier::READ,
                 "ReflectionProbeData",
                 "reflection_probe_buf[]");

/** \} */
