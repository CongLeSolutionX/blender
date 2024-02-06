/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once
#include "BKE_anonymous_attribute_id.hh"
#include "BKE_geometry_set.hh"
#include "BLI_math_matrix_types.hh"

namespace blender::geometry {

bke::GeometrySet join_geometries(Span<bke::GeometrySet> geometries,
                                 const bke::AnonymousAttributePropagationInfo &propagation_info);

void join_attributes(const Span<const bke::GeometryComponent *> src_components,
                     bke::GeometryComponent &result,
                     const Span<StringRef> ignored_attributes);
}  // namespace blender::geometry
