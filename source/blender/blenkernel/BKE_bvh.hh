/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_math_vector_types.hh"

struct Mesh;
struct RTCDeviceTy;
struct RTCSceneTy;

namespace blender {

struct BVHRay {
  static constexpr unsigned int MASK_FULL = 0xffffffff;
  static constexpr unsigned int FLAGS_NONE = 0;

  /* Coordinate of ray origin */
  float3 origin;
  /* Start of ray segment relative to ray length */
  float dist_min;

  /* Ray direction */
  float3 direction;

  /* Time of this ray for motion blur */
  float time;

  /* End of ray segment relative to ray length (set to hit distance) */
  float dist_max;

  /* Ray mask */
  unsigned int mask;
  /* Ray ID */
  unsigned int id;
  /* Ray flags */
  unsigned int flags;
};

struct BVHHit {
  static constexpr unsigned int INVALID_INSTANCE_ID = 0xffffffff;
  static constexpr int MAX_INSTANCE_LEVEL = 8;

  /* Geometry normal */
  float3 normal;
  /* Barycentric UV of hit */
  float2 uv;

  /* Primitive ID */
  unsigned int primitive_id;
  /* Geometry ID */
  unsigned int geometry_id;
  /* Instance ID */
  unsigned int instance_id[MAX_INSTANCE_LEVEL];
};

struct BVHRayHit {
  BVHRay ray;
  BVHHit hit;
};

class BVHTree {
 private:
  RTCDeviceTy *rtc_device = nullptr;
  RTCSceneTy *rtc_scene = nullptr;

 public:
  BVHTree();
  BVHTree(const BVHTree &) = delete;
  ~BVHTree();

  BVHTree &operator=(const BVHTree &) = delete;

  void free();

  void build_single_mesh(const Mesh &mesh);

  bool ray_intersect1(const BVHRay &ray, BVHRayHit &r_hit);
};

}  // namespace blender
