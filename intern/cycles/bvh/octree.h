/* SPDX-FileCopyrightText: 2024 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#ifndef __OCTREE_H__
#define __OCTREE_H__

#include "util/boundbox.h"
#include "util/task.h"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

#include <map>

#include <atomic>

CCL_NAMESPACE_BEGIN

class BoundBox;
class Device;
class Geometry;
class Mesh;
class Object;
class Progress;
class Scene;
class Shader;
struct KernelOctreeNode;

struct OctreeNode {
  BoundBox bbox;
  vector<Object *> objects;
  int level;

  /* TODO(weizhen): we need visibility for shadow, camera, and indirect. */
  Extrema<float> sigma = {0.0f, 0.0f};

  OctreeNode() : bbox(BoundBox::empty), level(0) {}
  OctreeNode(BoundBox bbox_, int level_) : bbox(bbox_), level(level_) {}
  virtual ~OctreeNode() = default;
};

struct OctreeInternalNode : public OctreeNode {
  OctreeInternalNode(OctreeNode &node) : children_(8)
  {
    bbox = node.bbox;
    level = node.level;
    objects = std::move(node.objects);
  }

  vector<std::shared_ptr<OctreeNode>> children_;
};

class Octree {
  friend struct OctreeNode;

 public:
  void build(Device *device, Progress &progress);
  Octree(const Scene *scene);
  ~Octree();

  void flatten(KernelOctreeNode *knodes);
  bool is_empty() const;
  int get_num_nodes() const;
  bool has_world_volume() const;

  float3 world_to_index(float3 p) const;
  int3 world_to_floor_index(float3 p) const;
  int3 world_to_ceil_index(float3 p) const;
  /* Convert from index to the position of the lower left corner of the cell. */
  float3 index_to_world(int x, int y, int z) const;

  /* Represent octree nodes as empty boxes with Blender Python API. */
  void visualize(KernelOctreeNode *knodes, const char *filename) const;
  void visualize_fast(KernelOctreeNode *knodes, const char *filename) const;

 private:
  std::shared_ptr<OctreeInternalNode> make_internal(std::shared_ptr<OctreeNode> &node);
  void recursive_build_(std::shared_ptr<OctreeNode> &node);
  int flatten_(KernelOctreeNode *knodes, std::shared_ptr<OctreeNode> &node, int &index);
  void evaluate_volume_density_(Device *device, Progress &progress);
  int flatten_index(int x, int y, int z) const;
  int flatten_index(int x, int y, int z, int3 size) const;
  bool should_split(std::shared_ptr<OctreeNode> &node);

  /* Root node. */
  std::shared_ptr<OctreeNode> root_;
  std::atomic<int> num_nodes = 1;

  TaskPool task_pool;
#ifdef WITH_OPENVDB
  openvdb::BoolGrid::ConstPtr mesh_to_sdf_grid(const Mesh *mesh,
                                               const float voxel_size,
                                               const float half_width);
  std::map<const Geometry *, openvdb::BoolGrid::ConstPtr> vdb_map;
#endif

  /* Set the maximal resolution to be 128 to reduce traversing overhead. */
  /* TODO(weizhen): tweak this threshold. 128 is a reference from PBRT. */
  const int max_level = 7;
  int width;
  float3 world_to_index_scale_;
  float3 index_to_world_scale_;
  vector<Extrema<float>> sigmas;

  /* World volume. */
  /* TODO(weizhen): we only need the max density after properly evaluating volume shaders. */
  float background_density_max;
  float background_density_min;
};

CCL_NAMESPACE_END

#endif /* __OCTREE_H__ */
