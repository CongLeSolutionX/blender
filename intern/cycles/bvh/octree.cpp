/* SPDX-FileCopyrightText: 2024 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "bvh/octree.h"

#include "scene/background.h"
#include "scene/geometry.h"
#include "scene/image_vdb.h"
#include "scene/object.h"
#include "scene/shader_nodes.h"
#include "scene/volume.h"

#include "util/progress.h"
#include "util/stats.h"

#include <fstream>
#include <memory>

#ifdef WITH_OPENVDB
#  include <openvdb/tools/LevelSetUtil.h>
#endif
#ifdef WITH_NANOVDB
#  define NANOVDB_USE_OPENVDB
#  include <nanovdb/util/CreateNanoGrid.h>
#  include <nanovdb/util/GridStats.h>
#endif

CCL_NAMESPACE_BEGIN

/* TODO(weizhen): this is only for testing. Need to support procedural shaders and multiple
 * shaders. */
float volume_density_scale(const Shader *shader)
{
  if (!shader->has_volume) {
    return 0.0f;
  }

  VolumeNode *volume_node = dynamic_cast<VolumeNode *>(
      shader->graph->output()->input("Volume")->link->parent);
  if (!volume_node) {
    return 0.0f;
  }

  float3 color = volume_node->get_color();

  if (auto *absorption_volume_node = dynamic_cast<AbsorptionVolumeNode *>(volume_node)) {
    color = one_spectrum() - color;
  }

  if (auto *principled_volume_node = dynamic_cast<PrincipledVolumeNode *>(volume_node)) {
    Spectrum zero = zero_spectrum();
    Spectrum one = one_spectrum();
    Spectrum absorption = max(one - color, zero) *
                          max(one - principled_volume_node->get_absorption_color(), zero);
    color += absorption;
  }
  return reduce_max(volume_node->get_density() * color);
}

float volume_density_scale(const Object *object)
{
  Geometry *geom = object->get_geometry();

  for (Node *node : geom->get_used_shaders()) {
    Shader *shader = static_cast<Shader *>(node);
    if (shader->has_volume) {
      return volume_density_scale(shader) *
             ObjectManager::object_volume_density(object->get_tfm(), geom);
    }
  }
  return 0.0f;
}

template<typename T>
nanovdb::Extrema<typename nanovdb::NanoGrid<T>::ValueType> OctreeNode::get_extrema(
    const nanovdb::NanoGrid<T> *grid, const Transform *itfm)
{
  nanovdb::BBox<nanovdb::Vec3f> vdb_bbox;

  for (int i = 0; i < 8; i++) {
    const float3 t = make_float3(i & 1, (i >> 1) & 1, (i >> 2) & 1);
    const float3 v = transform_point(itfm, mix(bbox.min, bbox.max, t));
    vdb_bbox.expand(nanovdb::Vec3(v.x, v.y, v.z));
  }
  const nanovdb::CoordBBox coord_bbox(grid->worldToIndexF(vdb_bbox.min()).floor(),
                                      grid->worldToIndexF(vdb_bbox.max()).ceil());

  return nanovdb::getExtrema(*grid, coord_bbox);
}

bool Octree::should_split(std::shared_ptr<OctreeNode> &node)
{
  if (node->objects.empty()) {
    return false;
  }

  if (node->objects.size() == 1) {
    const Object *object = node->objects[0];
    if (object->is_homogeneous_volume()) {
      /* Do not split homogeneous volume. Volume stack already skips the zero-density regions. */
      node->sigma_min = node->sigma_max = volume_density_scale(object);
      return false;
    }
  }

  node->sigma_min = background_density_min;
  node->sigma_max = background_density_max;

  for (Object *object : node->objects) {
    float min = 1.0f;
    float max = 1.0f;

    /* TODO(weizhen): objects might have multiple shaders. */
    Geometry *geom = object->get_geometry();
    if (geom->is_volume()) {
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
            const Transform itfm = transform_inverse(object->get_tfm());
            auto extrema = node->get_extrema(grid, &itfm);
            min = extrema.min();
            max = extrema.max();
          }
        }
#  endif
      }
#else
      /* TODO */
#endif
    }
    else if (geom->is_mesh()) {
      const auto *grid = vdb_map.at(geom).grid<bool>();
      if (grid) {
        /* TODO(weizhen): evaluate objects with various shaders separately.  */
        /* TODO(weizhen): probably dense grid is the proper way. */
        const Mesh *mesh = static_cast<const Mesh *>(geom);
        Transform itfm;
        if (!mesh->transform_applied) {
          itfm = transform_inverse(object->get_tfm());
        }
        nanovdb::BBox<nanovdb::Vec3f> vdb_bbox;

        for (int i = 0; i < 8; i++) {
          const float3 t = make_float3(i & 1, (i >> 1) & 1, (i >> 2) & 1);
          float3 v = mix(node->bbox.min, node->bbox.max, t);
          if (!mesh->transform_applied) {
            v = transform_point(&itfm, v);
          }
          vdb_bbox.expand(nanovdb::Vec3(v.x, v.y, v.z));
        }
        const nanovdb::CoordBBox coord_bbox(grid->worldToIndexF(vdb_bbox.min()).floor(),
                                            grid->worldToIndexF(vdb_bbox.max()).ceil());

        /* TODO(weizhen): workaround of bool grid not supporting `getExtrema()`. Shouldn't be
         * necessary later. */
        min = 1.0f;
        max = 0.0f;
        bool has_min = false;
        bool has_max = false;
        auto acc = grid->getAccessor();
        [&] {
          for (int x = coord_bbox.min().x(); x <= coord_bbox.max().x(); x++) {
            for (int y = coord_bbox.min().y(); y <= coord_bbox.max().y(); y++) {
              for (int z = coord_bbox.min().z(); z <= coord_bbox.max().z(); z++) {
                if (acc.getValue(nanovdb::Coord(x, y, z))) {
                  max = 1.0f;
                  if (has_min) {
                    return;
                  }
                  has_max = true;
                }
                else if (!acc.getValue(nanovdb::Coord(x, y, z))) {
                  min = 0.0f;
                  if (has_max) {
                    return;
                  }
                  has_min = true;
                }
              }
            }
          }
        }();
      }
    }

    const float scale = volume_density_scale(object);
    min *= scale;
    max *= scale;

    node->sigma_min = fminf(min, node->sigma_min);
    node->sigma_max += max;
  }

  /* TODO(weizhen): force subdivision of aggregate nodes that are larger than the volume contained,
   * regardless of the volume’s majorant extinction. */

  /* From "Volume Rendering for Pixar’s Elemental". */
  if ((node->sigma_max - node->sigma_min) * len(node->bbox.size()) < 1.442f ||
      node->level == max_level)
  {
    return false;
  }

  return true;
}

shared_ptr<OctreeInternalNode> Octree::make_internal(shared_ptr<OctreeNode> &node)
{
  num_nodes += 8;
  auto internal = std::make_shared<OctreeInternalNode>(*node);

  /* Create bounding boxes for children. */
  const float3 center = internal->bbox.center();
  for (int i = 0; i < 8; i++) {
    const float3 t = make_float3(i & 1, (i >> 1) & 1, (i >> 2) & 1);
    const BoundBox bbox(mix(internal->bbox.min, center, t), mix(center, internal->bbox.max, t));
    internal->children_[i] = std::make_shared<OctreeNode>(bbox, internal->level + 1);
  }

  return internal;
}

void Octree::recursive_build_(shared_ptr<OctreeNode> &node)
{
  if (!should_split(node)) {
    return;
  }

  /* Make the current node an internal node. */
  auto internal = make_internal(node);

  for (auto &child : internal->children_) {
    child->objects.reserve(internal->objects.size());
    for (Object *object : internal->objects) {
      /* TODO(weizhen): more granular than object bounding box is to use the geometry bvh. */
      if (object->bounds.intersects(child->bbox)) {
        child->objects.push_back(object);
      }
    }
    /* TODO(weizhen): check the performance. */
    task_pool.push([&] { recursive_build_(child); });
  }

  node = internal;
}

nanovdb::GridHandle<> Octree::mesh_to_sdf_grid(const Mesh *mesh,
                                               const float voxel_size,
                                               const float half_width)
{
  const int num_verts = mesh->get_verts().size();
  std::vector<openvdb::Vec3s> points(num_verts);
  parallel_for(0, num_verts, [&](int i) {
    const float3 &vert = mesh->get_verts()[i];
    points[i] = openvdb::Vec3s(vert.x, vert.y, vert.z);
  });

  const int num_triangles = mesh->num_triangles();
  std::vector<openvdb::Vec3I> triangles(num_triangles);
  parallel_for(0, num_triangles, [&](int i) {
    triangles[i] = openvdb::Vec3I(mesh->get_triangles()[i * 3],
                                  mesh->get_triangles()[i * 3 + 1],
                                  mesh->get_triangles()[i * 3 + 2]);
  });

  auto xform = openvdb::math::Transform::createLinearTransform(voxel_size);
  auto sdf_grid = openvdb::tools::meshToLevelSet<openvdb::FloatGrid>(
      *xform, points, triangles, half_width);

  auto interior_mask_grid = openvdb::tools::sdfInteriorMask(*sdf_grid, 0.0f);

  /* TODO(weizhen): do I need to reset OpenVDB grids? */
  /* TODO(weizhen): mind vdb version, following `image_vdb.cpp`? */
  return nanovdb::openToNanoVDB(interior_mask_grid);
}

int Octree::flatten_(KernelOctreeNode *knodes, shared_ptr<OctreeNode> &node, int &node_index)
{
  const int current_index = node_index++;

  KernelOctreeNode &knode = knodes[current_index];
  knode.bbox.max = node->bbox.max;
  knode.bbox.min = node->bbox.min;
  if (auto internal_ptr = std::dynamic_pointer_cast<OctreeInternalNode>(node)) {
    knode.is_leaf = false;
    /* Loop through all the children. */
    for (int i = 0; i < 8; i++) {
      knode.children[i] = flatten_(knodes, internal_ptr->children_[i], node_index);
    }
  }
  else {
    knode.is_leaf = true;
    knode.sigma_max = node->sigma_max;
    knode.sigma_min = node->sigma_min;
  }

  return current_index;
}

void Octree::flatten(KernelOctreeNode *knodes)
{
  int node_index = 0;

  /* World volume. */
  /* TODO(weizhen): is there a better way than putting world volume in the octree array? */
  KernelOctreeNode &knode = knodes[node_index++];
  knode.is_leaf = false;
  knode.sigma_max = background_density_max;
  knode.sigma_min = has_world_volume() ? background_density_min : 0.0f;
  knode.bbox.max = make_float3(FLT_MAX);
  knode.bbox.min = -make_float3(FLT_MAX);

  flatten_(knodes, root_, node_index);
  /* TODO(weizhen): rescale the bounding box to match its resolution, for more robust traversing.
   */
}

void Octree::build(Progress &progress)
{
  // if (!root_) {
  //   return;
  // }

  progress.set_substatus("Building Octree for volumes");
  const double start_time = time_dt();

  recursive_build_(root_);

  task_pool.wait_work();

  std::cout << "Built volume Octree with " << num_nodes << " nodes in " << time_dt() - start_time
            << " seconds." << std::endl;
  VLOG_INFO << "Built volume Octree with " << num_nodes << " nodes in " << time_dt() - start_time
            << " seconds.";
}

Octree::Octree(const Scene *scene)
{
  root_ = std::make_shared<OctreeNode>();

  /* Loop through the volume objects to initialize the root node. */
  for (Object *object : scene->objects) {
    const Geometry *geom = object->get_geometry();
    if (geom->has_volume) {
      const float3 size = object->bounds.size();
      /* Don't push zero-sized volume. */
      if (size.x == 0.0f || size.y == 0.0f || size.z == 0.0f) {
        /* TODO(weizhen): how about this swimming pool mesh? */
        continue;
      }

      root_->bbox.grow(object->bounds);
      root_->objects.push_back(object);

      /* Create SDF grid for mesh volumes, to determine whether a certain point is in the
       * interior of the mesh. */
      if (geom->is_mesh() && vdb_map.find(geom) == vdb_map.end()) {
        const Mesh *mesh = static_cast<const Mesh *>(geom);
        vdb_map[geom] = mesh_to_sdf_grid(mesh, 0.1f, 1.0f);
      }
    }
  }

  /* Evaluate world volume density. */
  Shader *bg_shader = scene->background->get_shader(scene);
  background_density_max = volume_density_scale(bg_shader);
  if (background_density_max > 0.0f) {
    if (!bg_shader->has_volume_spatial_varying) {
      background_density_min = background_density_max;
    }
    else {
      /* TODO(weizhen): can we be more sure about the minimal density. */
      background_density_min = 0.0f;
    }
  }
  else {
    /* So that fminf(a, FLT_MAX) always returns a. */
    background_density_min = FLT_MAX;
  }

  // if (!root_->bbox.valid()) {
  //   root_.reset();
  //   return;
  // }

  VLOG_INFO << "Total " << root_->objects.size()
            << " volume objects with bounding box min = " << root_->bbox.min
            << ", max = " << root_->bbox.max << ".";
}

bool Octree::is_empty() const
{
  /* TODO(weizhen): zero size? */
  return !root_->bbox.valid();
}

int Octree::get_num_nodes() const
{
  /* Plus one to account for world volume. */
  return num_nodes + 1;
}

bool Octree::has_world_volume() const
{
  return background_density_max > 0.0f;
}

void Octree::visualize(KernelOctreeNode *knodes, const char *filename) const
{
  std::ofstream file(filename);
  if (file.is_open()) {
    file << "# Visualize volume octree. This script is slow when there are more than 10000 "
            "nodes, but helps with visualizing bounding boxes.\n\n";
    file << "import bpy\n\n";
    file << "octree = bpy.data.collections.new(name='Octree')\n";
    file << "bpy.context.scene.collection.children.link(octree)\n";
    file << "bpy.ops.mesh.primitive_cube_add(location = (0, 0, 0), scale=(1, 1, 1))\n";
    file << "cube = bpy.context.object\n";
    for (int i = 1; i < get_num_nodes(); i++) {
      if (!knodes[i].is_leaf) {
        /* Only draw leaf nodes. */
        continue;
      }
      file << "\n";
      const float3 center = knodes[i].bbox.center();
      file << "ob = bpy.data.objects.new(name = '" << i << " sigma_max = " << knodes[i].sigma_max
           << " sigma_min = " << knodes[i].sigma_min << "' , object_data = cube.data)\n";
      file << "ob.location = (" << center.x << ", " << center.y << ", " << center.z << ")\n";
      const float3 scale = knodes[i].bbox.size() * 0.5f;
      file << "ob.scale = (" << scale.x << ", " << scale.y << ", " << scale.z << ")\n";
      file << "octree.objects.link(ob)\n";
    }
    file << "\nbpy.ops.object.delete()\n\n";
    file << "for obj in octree.objects:\n";
    file << "    obj.select_set(True)\n\n";

    file << "bpy.context.view_layer.objects.active = octree.objects[0]\n";
    file << "bpy.ops.object.join()\n";
    file << "bpy.ops.object.mode_set(mode='EDIT')\n";
    file << "bpy.ops.mesh.delete(type='ONLY_FACE')\n";
    file << "bpy.ops.object.mode_set(mode='OBJECT')\n";

    file.close();
  }
}

void Octree::visualize_fast(KernelOctreeNode *knodes, const char *filename) const
{
  std::ofstream file(filename);
  if (file.is_open()) {
    file << "# Visualize volume octree.\n\n";
    file << "import bpy\n\n";
    file << "octree = bpy.data.collections.new(name='Octree')\n";
    file << "bpy.context.scene.collection.children.link(octree)\n\n";
    float3 center = knodes[1].bbox.center();
    float3 size = knodes[1].bbox.size() * 0.5f;
    file << "bpy.ops.mesh.primitive_cube_add(location = (" << center.x << ", " << center.y << ", "
         << center.z << "), scale = (" << size.x << ", " << size.y << ", " << size.z
         << "), enter_editmode = True)\n";
    file << "bpy.ops.mesh.delete(type='ONLY_FACE')\n";
    file << "bpy.ops.object.mode_set(mode='OBJECT')\n";
    file << "octree.objects.link(bpy.context.object)\n";
    file << "bpy.context.scene.collection.objects.unlink(bpy.context.object)\n\n";
    /* TODO(weizhen): there is a bug when scene already contains an object called `Cube`. */

    file << "vertices = [";
    for (int i = 1; i < get_num_nodes(); i++) {
      if (knodes[i].is_leaf) {
        continue;
      }
      center = knodes[i].bbox.center();
      size = knodes[i].bbox.size() * 0.5f;
      /* Create three orthogonal faces. */
      file << "(" << center.x << "," << center.y << "," << center.z - size.z << "), (" << center.x
           << "," << center.y << "," << center.z + size.z << "), (" << center.x << ","
           << center.y + size.y << "," << center.z + size.z << "), (" << center.x << ","
           << center.y + size.y << "," << center.z - size.z << "), (" << center.x << ","
           << center.y - size.y << "," << center.z - size.z << "), (" << center.x << ","
           << center.y - size.y << "," << center.z + size.z << "), (";
      file << center.x - size.x << "," << center.y << "," << center.z << "), ("
           << center.x + size.x << "," << center.y << "," << center.z << "), ("
           << center.x + size.x << "," << center.y << "," << center.z + size.z << "), ("
           << center.x - size.x << "," << center.y << "," << center.z + size.z << "), ("
           << center.x - size.x << "," << center.y << "," << center.z - size.z << "), ("
           << center.x + size.x << "," << center.y << "," << center.z - size.z << "), (";
      file << center.x << "," << center.y - size.y << "," << center.z << "), (" << center.x << ","
           << center.y + size.y << "," << center.z << "), (" << center.x + size.x << ","
           << center.y + size.y << "," << center.z << "), (" << center.x + size.x << ","
           << center.y - size.y << "," << center.z << "), (" << center.x - size.x << ","
           << center.y - size.y << "," << center.z << "), (" << center.x - size.x << ","
           << center.y + size.y << "," << center.z << "), ";
    }
    file << "]\nr = range(len(vertices))\n";
    file << "edges = [(i, i+1 if i%6<5 else i-4) for i in r]\n";
    file << "mesh = bpy.data.meshes.new('Octree')\n";
    file << "mesh.from_pydata(vertices, edges, [])\n";
    file << "mesh.update()\n";
    file << "obj = bpy.data.objects.new('obj', mesh)\n";
    file << "octree.objects.link(obj)\n";
    file << "obj.select_set(True)\n\n";

    file << "bpy.ops.object.join()\n";
    file.close();
  }
}

Octree::~Octree()
{
  /* TODO(weizhen): quite weird workaround to delay releasing nanogrid, but probably don't need
   * this later. */
  /* TODO(weizhen): if the scene has instanced objects vdb was already cleared, this doesn't work.
   */
#ifdef WITH_NANOVDB
  for (auto &it : vdb_map) {
    it.second.reset();
  }
  for (Object *object : root_->objects) {
    Geometry *geom = object->get_geometry();
    if (geom->geometry_type == Geometry::VOLUME) {
      Volume *volume = static_cast<Volume *>(geom);
      for (Attribute &attr : volume->attributes.attributes) {
        if (attr.element == ATTR_ELEMENT_VOXEL) {
          if (attr.data_voxel().vdb_loader()) {
            attr.data_voxel().vdb_loader()->nanogrid.reset();
          }
        }
      }
    }
  }
#endif
}

CCL_NAMESPACE_END
