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
struct KernelOctreeNode;
struct Node;

struct OctreeNode {
  BoundBox bbox;
  vector<Node *> objects;
  int depth;

  /* TODO(weizhen): we need visibility for shadow, camera, and indirect. */
  Extrema<float> sigma = {0.0f, 0.0f};

  OctreeNode() : bbox(BoundBox::empty), depth(0) {}
  OctreeNode(BoundBox bbox_, int depth_) : bbox(bbox_), depth(depth_) {}
  virtual ~OctreeNode() = default;

  bool contains_homogeneous_volume(const Scene *scene) const;
};

struct OctreeInternalNode : public OctreeNode {
  OctreeInternalNode(OctreeNode &node) : children_(8)
  {
    bbox = node.bbox;
    depth = node.depth;
    objects = std::move(node.objects);
  }

  vector<std::shared_ptr<OctreeNode>> children_;
};

class Octree {
  friend struct OctreeNode;

 public:
  void build(Device *device, Progress &progress, Scene *scene);
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
  float3 voxel_size() const;
  int flatten_index(int x, int y, int z) const;

  /* Represent octree nodes as empty boxes with Blender Python API. */
  void visualize(KernelOctreeNode *knodes, const char *filename) const;
#ifdef WITH_OPENVDB
  openvdb::BoolGrid::ConstPtr get_vdb(const std::pair<const Geometry *, const Shader *> &) const;
#endif
  int get_width() const;

 private:
  std::shared_ptr<OctreeInternalNode> make_internal(std::shared_ptr<OctreeNode> &node);
  void recursive_build_(const Scene *scene, std::shared_ptr<OctreeNode> &node);
  /* Breadth-first flatten, so that children are stored in consecutive indices. */
  void flatten_(KernelOctreeNode *knodes,
                const int current_index,
                std::shared_ptr<OctreeNode> &node,
                int &child_index);
  void evaluate_volume_density_(Device *device, Progress &progress, Scene *scene);
  Extrema<float> get_extrema(const vector<Extrema<float>> &values,
                             const int3 index_min,
                             const int3 index_max) const;
  bool should_split(const Scene *scene, std::shared_ptr<OctreeNode> &node) const;

  /* Root node. */
  std::shared_ptr<OctreeNode> root_;
  std::atomic<int> num_nodes = 1;

  TaskPool task_pool;
#ifdef WITH_OPENVDB
  openvdb::BoolGrid::ConstPtr mesh_to_sdf_grid(const Mesh *mesh,
                                               const Shader *shader,
                                               const float half_width);
  std::map<std::pair<const Geometry *, const Shader *>, openvdb::BoolGrid::ConstPtr> vdb_map;
#endif

  int width;
  float3 world_to_index_scale_;
  float3 index_to_world_scale_;
  vector<Extrema<float>> sigmas;

  /* World volume. */
  Extrema<float> background_density;
};

CCL_NAMESPACE_END

#endif /* __OCTREE_H__ */
