/* SPDX-FileCopyrightText: 2009-2010 Sony Pictures Imageworks Inc., et al. All Rights Reserved.
 * SPDX-FileCopyrightText: 2011-2024 Blender Foundation
 *
 * SPDX-License-Identifier: BSD-3-Clause */

#pragma once

#include "kernel/osl/types.h"

CCL_NAMESPACE_BEGIN

ccl_device_inline void cameradata_to_shaderglobals(KernelGlobals kg,
                                                   const packed_float3 sensor,
                                                   const float2 rand_lens,
                                                   ccl_private ShaderGlobals *globals)
{
  // TODO
  memset(globals, 0, sizeof(ShaderGlobals));

  globals->P = sensor;
  globals->N = float2_to_float3(rand_lens);
}

#ifndef __KERNEL_GPU__

float osl_eval_camera(KernelGlobals kg,
                      const packed_float3 sensor,
                      const float2 rand_lens,
                      packed_float3 &P,
                      packed_float3 &D);

#else

ccl_device_inline float osl_eval_camera(KernelGlobals kg,
                                        const packed_float3 sensor,
                                        const float2 rand_lens,
                                        packed_float3 &P,
                                        packed_float3 &D)
{
  ShaderGlobals globals;
  cameradata_to_shaderglobals(kg, sensor, rand_lens, &globals);

  float output[7] = {0.0f};
#  ifdef __KERNEL_OPTIX__
  optixDirectCall<void>(/* NUM_CALLABLE_PROGRAM_GROUPS */ 2,
                        /* shaderglobals_ptr = */ &globals,
                        /* groupdata_ptr = */ (void *)nullptr,
                        /* userdata_base_ptr = */ (void *)nullptr,
                        /* output_base_ptr = */ (void *)output,
                        /* shadeindex = */ 0,
                        /* interactive_params_ptr */ (void *)nullptr);
#  endif

  P = make_float3(output[0], output[1], output[2]);
  D = make_float3(output[3], output[4], output[5]);
  return output[6];
}

#endif

CCL_NAMESPACE_END
