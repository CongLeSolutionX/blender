/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#ifdef WITH_POTRACE

#  include <cstring>

#  include "BLI_index_mask_fwd.hh"
#  include "BLI_math_vector_types.hh"
#  include "BLI_span.hh"

struct potrace_state_s;
typedef struct potrace_state_s potrace_state_t;

struct Curves;

namespace blender::geometry::potrace {

struct Params {
  /* Original resolution, not aligned. */
  int2 resolution;

  static constexpr const float max_smooth_threshold = 4.0f / 3.0f;
  float smooth_threshold = 1.0f;

  float optimization_tolerance = 0.2f;
};

int2 aligned_resolution(int2 resolution);

/* \param mask: have to fit into image of aligned_resolution size in structure as: lines of
 * columns. */
potrace_state_t *image_from_mask(Params params, const IndexMask &mask);

void free_image(potrace_state_t *image);

Curves *image_to_curve(const potrace_state_t *image,
                       const std::optional<std::string> &parent_index_id);

float4x4 to_plane(int2 resolution, float2 min_point, float2 max_point);

}  // namespace blender::geometry::potrace

#endif
