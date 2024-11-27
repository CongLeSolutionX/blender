/* SPDX-FileCopyrightText: 2024 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#ifndef __OCTREE_H__
#define __OCTREE_H__

#include "util/boundbox.h"
#include "util/map.h"
#include "util/task.h"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

#include <atomic>

CCL_NAMESPACE_BEGIN

class BoundBox;
class Device;
class Geometry;
class Mesh;
class Progress;
class Scene;
class Shader;
class Object;
struct KernelOctreeNode;
struct Node;
class VolumeManager;

struct OctreeNode {
  BoundBox bbox;
  int depth;

  /* TODO(weizhen): we need visibility for shadow, camera, and indirect. */
  Extrema<float> sigma = {0.0f, 0.0f};

  OctreeNode() : bbox(BoundBox::empty), depth(0) {}
  OctreeNode(BoundBox bbox_, int depth_) : bbox(bbox_), depth(depth_) {}
  virtual ~OctreeNode() = default;
};

struct OctreeInternalNode : public OctreeNode {
  OctreeInternalNode(OctreeNode &node) : children_(8)
  {
    bbox = node.bbox;
    depth = node.depth;
  }

  vector<std::shared_ptr<OctreeNode>> children_;
};

class Octree {
  friend struct OctreeNode;

 public:
  void build(Device *, Progress &, const Object *, const Shader *, openvdb::BoolGrid::ConstPtr &);
  Octree(const BoundBox &bbox);
  ~Octree() = default;
  /* Breadth-first flatten, so that children are stored in consecutive indices. */
  void flatten(KernelOctreeNode *, const int, const std::shared_ptr<OctreeNode> &, int &) const;
  int get_num_nodes() const;

  float3 object_to_index(float3 p) const;
  int3 object_to_floor_index(float3 p) const;
  int3 object_to_ceil_index(float3 p) const;
  /* Convert from index to the position of the lower left corner of the cell. */
  float3 index_to_object(int x, int y, int z) const;
  float3 voxel_size() const;
  int flatten_index(int x, int y, int z) const;

  /* Represent octree nodes as empty boxes with Blender Python API. */
  void visualize(const KernelOctreeNode *knodes, const int root, std::ofstream &file) const;
  bool is_built() const;
  std::shared_ptr<OctreeNode> get_root() const;

 private:
  bool is_built_;
  std::shared_ptr<OctreeInternalNode> make_internal(std::shared_ptr<OctreeNode> &node);
  /* Scale the node size so that Octree has the same shape in viewport and final render. */
  float volume_scale_(const Object *object) const;
  void recursive_build_(std::shared_ptr<OctreeNode> &, const float, const bool);
  void evaluate_volume_density_(
      Device *, Progress &, const Object *, const Shader *, openvdb::BoolGrid::ConstPtr &);
  Extrema<float> get_extrema(const vector<Extrema<float>> &values,
                             const int3 index_min,
                             const int3 index_max) const;
  bool should_split(std::shared_ptr<OctreeNode> &, const float, const bool) const;

  /* Root node. */
  std::shared_ptr<OctreeNode> root_;
  /* TODO(weizhen): Remove atomic? */
  std::atomic<int> num_nodes = 1;

  TaskPool task_pool;

  int width;
  float3 object_to_index_scale_;
  float3 index_to_object_scale_;
  vector<Extrema<float>> sigmas;

  /* World volume. */
  // Extrema<float> background_density;
};

CCL_NAMESPACE_END

#endif /* __OCTREE_H__ */
