/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "FN_field.hh"

#include "BKE_attribute.hh"
#include "BKE_curves.hh"

namespace blender::geometry {

/**
 * Create new curves that are interpolated between "from" and "to" curves.
 * \param dst_curve_mask: Set of curves in \a dst_curves that are being filled.
 */
void interpolate_curves(const bke::CurvesGeometry &from_curves,
                        const bke::CurvesGeometry &to_curves,
                        Span<int> from_curve_indices,
                        Span<int> to_curve_indices,
                        const IndexMask &dst_curve_mask,
                        Span<bool> dst_curve_flip_direction,
                        const float mix_factor,
                        bke::CurvesGeometry &dst_curves);

/* Customized callback */
using CurveSamplingFunc = FunctionRef<void(const Span<float> segment_lengths,
                                           const bool cyclic,
                                           const bool reverse,
                                           MutableSpan<int> r_segment_indices,
                                           MutableSpan<float> r_factors)>;

void interpolate_curves_with_sampling(const CurvesGeometry &from_curves,
                                      const CurvesGeometry &to_curves,
                                      const Span<int> from_curve_indices,
                                      const Span<int> to_curve_indices,
                                      const IndexMask &dst_curve_mask,
                                      const Span<bool> dst_curve_flip_direction,
                                      const float mix_factor,
                                      CurveSamplingFunc sampling_fn,
                                      CurvesGeometry &dst_curves);

}  // namespace blender::geometry
