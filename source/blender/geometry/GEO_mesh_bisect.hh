/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "DNA_node_types.h"

struct Mesh;

#include "BKE_anonymous_attribute_id.hh"

namespace blender::geometry {

struct BisectArgs {

  float plane[4];
  float3 plane_co;
  float3 plane_no;
  bool clear_inner;
  bool clear_outer;
};

enum BisectResult {
  /* Mesh data is discarded, returning empty output.
   */
  Discard,
  /* Entirity of the data is kept, returning the input mesh.
   */
  Keep,
  /* Mesh is split, returning a modified copy.
   */
  Bisect
};

std::pair<Mesh *, BisectResult> bisect_mesh(
    const Mesh &mesh,
    const BisectArgs &args,
    const bke::AnonymousAttributePropagationInfo &propagation_info);

}  // namespace blender::geometry
