/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_index_mask.hh"

#include "BKE_attribute_filter.hh"

struct Mesh;

namespace blender::geometry {

Mesh *dissolve_boundary_verts(const Mesh &src_mesh,
                              const IndexMask &vers_mask,
                              const bke::AttributeFilter &attribute_filter = {});

}  // namespace blender::geometry
