/* SPDX-FileCopyrightText: 2024 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "bvh/octree.h"

#include "scene/geometry.h"
#include "scene/image_vdb.h"
#include "scene/object.h"
#include "scene/volume.h"

#include "util/progress.h"
#include "util/stats.h"

#include <memory>

#ifdef WITH_NANOVDB
#  include <nanovdb/util/GridStats.h>
#endif

CCL_NAMESPACE_BEGIN

bool OctreeNode::should_split()
{
  if (objects.empty()) {
    return false;
  }

  float global_min = FLT_MAX;
  float global_max = 0.0f;

  for (Object *object : objects) {
    /* NOTE: test scene with known density */
    float min = 0.4f;
    float max = 0.4f;

    Geometry *geom = object->get_geometry();
    if (geom->geometry_type == Geometry::VOLUME) {
      Volume *volume = static_cast<Volume *>(geom);
#ifdef WITH_OPENVDB
      for (Attribute &attr : volume->attributes.attributes) {
        if (attr.element != ATTR_ELEMENT_VOXEL || attr.name != "density") {
          continue;
        }

        /* TODO(weizhen): bbox is an approximation, a more appropriate way would be to evaluate
         * volume shader at certain resolution. See `kernel/bake/bake.h`. */
        /* TODO(weizhen): potentially remove the object when max = 0. */
#  ifdef WITH_NANOVDB
        ImageHandle &handle = attr.data_voxel();
        if (handle.metadata().type == IMAGE_DATA_TYPE_NANOVDB_FP16) {

          auto const *grid = handle.vdb_loader()->nanogrid.grid<nanovdb::Fp16>();

          if (grid) {
            nanovdb::CoordBBox coord_bbox;
            const Transform itfm = transform_inverse(object->get_tfm());
            for (int i = 0; i < 8; i++) {
              const float3 t = make_float3(i & 1, (i >> 1) & 1, (i >> 2) & 1);
              const float3 v = transform_point(&itfm, mix(bbox.min, bbox.max, t));
              coord_bbox.expand(grid->worldToIndexF(nanovdb::Coord(v.x, v.y, v.z)));
            }
            auto ex = nanovdb::getExtrema(*grid, coord_bbox);
            min = ex.min();
            max = ex.max();
          }
        }
#  endif
      }
#else
      /* TODO */
#endif
    }
    global_min = fminf(min, global_min);
    global_max += max;
  }

  /* From "Volume Rendering for Pixar’s Elemental". */
  if ((global_max - global_min) * len(bbox.size()) < 1.442f) {
    return false;
  }

  return true;
}

shared_ptr<OctreeInternalNode> Octree::make_internal(shared_ptr<OctreeNode> &node)
{
  auto internal = std::make_shared<OctreeInternalNode>();
  internal->bbox = node->bbox;
  internal->objects = std::move(node->objects);
  return internal;
}

void Octree::recursive_build(shared_ptr<OctreeNode> &node)
{
  if (!node->should_split()) {
    num_leaf++;
    return;
  }

  num_internal++;

  /* Make the current node an internal node. */
  auto internal = make_internal(node);

  /* Create bounding boxes for children. */
  const float3 center = internal->bbox.center();
  for (int i = 0; i < 8; i++) {
    const float3 t = make_float3(i & 1, (i >> 1) & 1, (i >> 2) & 1);
    internal->children_[i] = std::make_shared<OctreeNode>();
    internal->children_[i]->bbox.min = mix(internal->bbox.min, center, t);
    internal->children_[i]->bbox.max = mix(center, internal->bbox.max, t);
  }

  for (auto &child : internal->children_) {
    for (Object *object : internal->objects) {
      /* TODO(weizhen): more granular than object bounding box is to use the geometry bvh. */
      if (object->bounds.intersects(child->bbox)) {
        child->objects.push_back(object);
      }
    }
    recursive_build(child);
  }

  /* TODO(weizhen): visualize Octree by creating empty mesh. */

  node = internal;
}

Octree::Octree(const Scene *scene)
{
  root_ = std::make_shared<OctreeNode>();

  for (Object *object : scene->objects) {
    if (object->get_geometry()->has_volume) {
      const float3 size = object->bounds.size();
      /* Don't push zero-sized volume. */
      if (size.x > 0.0f && size.y > 0.0f && size.z > 0.0f) {
        root_->bbox.grow(object->bounds);
        root_->objects.push_back(object);
      }
    }
  }

  /* TODO(weizhen): world volume. */

  if (!root_->bbox.valid()) {
    //    root_.reset();
    return;
  }

  VLOG_INFO << "Total " << root_->objects.size()
            << " volume objects with bounding box min = " << root_->bbox.min
            << ", max = " << root_->bbox.max << ".";
}

void Octree::visualize()
{
  /* TODO(weizhen): draw leaf node boxes. */
}

Octree::~Octree()
{
  /* TODO(weizhen): quite weird workaround to delay releasing nanogrid, but probably don't need
   * this later. */
  /* TODO(weizhen): if the scene has instanced objects vdb was already cleared, this doesn't work.
   */
#ifdef WITH_NANOVDB
  for (Object *object : root_->objects) {
    Geometry *geom = object->get_geometry();
    if (geom->geometry_type == Geometry::VOLUME) {
      Volume *volume = static_cast<Volume *>(geom);
      for (Attribute &attr : volume->attributes.attributes) {
        if (attr.element == ATTR_ELEMENT_VOXEL) {
          attr.data_voxel().vdb_loader()->nanogrid.reset();
        }
      }
    }
  }
#endif
}

void Octree::build(Progress &progress)
{
  // if (!root_) {
  //   return;
  // }
  if (!root_->bbox.valid()) {
    return;
  }

  progress.set_substatus("Building Octree for volumes");

  recursive_build(root_);

  // std::cout << "Built volume Octree with " << num_internal << " internal nodes and " << num_leaf
  //           << " leaf nodes." << std::endl;
  VLOG_INFO << "Built volume Octree with " << num_internal << " internal nodes and " << num_leaf
            << " leaf nodes.";
}

CCL_NAMESPACE_END
