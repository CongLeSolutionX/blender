/* SPDX-FileCopyrightText: 2024 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#ifndef __OCTREE_H__
#define __OCTREE_H__

#include "util/boundbox.h"
#include "util/task.h"

CCL_NAMESPACE_BEGIN

class BoundBox;
class Object;
class Progress;
class Scene;

/* TODO(weizhen): belong to util.h */
struct Extrema {
  float min, max;
};

struct OctreeNode {
  BoundBox bbox;
  vector<Object *> objects;

  OctreeNode() : bbox(BoundBox::empty) {}
  virtual ~OctreeNode() = default;

  bool should_split();
};

struct OctreeInternalNode : public OctreeNode {
  OctreeInternalNode() : children_(8) {}

  vector<std::shared_ptr<OctreeNode>> children_;
};

class Octree {
 public:
  void build(Progress &progress);
  void visualize();
  Octree(const Scene *scene);
  ~Octree();

 private:
  std::shared_ptr<OctreeNode> root_;
  /* Thread. */
  TaskPool task_pool;

  std::shared_ptr<OctreeInternalNode> make_internal(std::shared_ptr<OctreeNode> &node);

  void recursive_build(std::shared_ptr<OctreeNode> &node);

  /* TODO(weizhen): helper functions to delete. */
  int num_leaf = 0;
  int num_internal = 0;
};

CCL_NAMESPACE_END

#endif /* __OCTREE_H__ */
