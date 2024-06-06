/* SPDX-FileCopyrightText: 2009-2010 Sony Pictures Imageworks Inc., et al. All Rights Reserved.
 * SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Adapted code from Open Shading Language. */

#pragma once

CCL_NAMESPACE_BEGIN

/* Multiple importance sampling utilities. */

ccl_device float balance_heuristic(float a, float b)
{
  return (a) / (a + b);
}

ccl_device float balance_heuristic_3(float a, float b, float c)
{
  return (a) / (a + b + c);
}

ccl_device float power_heuristic(float a, float b)
{
  return (a * a) / (a * a + b * b);
}

ccl_device float power_heuristic(int num_a, float pdf_a, int num_b, float pdf_b)
{
  return (pdf_a * pdf_a) / (pdf_a * pdf_a * (float)num_a + pdf_b * pdf_b * (float)num_b);
}

ccl_device float power_heuristic_3(float a, float b, float c)
{
  return (a * a) / (a * a + b * b + c * c);
}

ccl_device float max_heuristic(float a, float b)
{
  return (a > b) ? 1.0f : 0.0f;
}

CCL_NAMESPACE_END
