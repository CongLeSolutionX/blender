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

#include "integrator/shader_eval.h"

#include "util/progress.h"
#include "util/stats.h"

#include <fstream>
#include <memory>

#ifdef WITH_OPENVDB
#  include <openvdb/tools/LevelSetUtil.h>
#endif

CCL_NAMESPACE_BEGIN

__forceinline static int flatten_index(int x, int y, int z, int3 size)
{
  return x + size.x * (y + z * size.y);
}

__forceinline int Octree::flatten_index(int x, int y, int z) const
{
  return x + width * (y + z * width);
}

bool OctreeNode::contains_homogeneous_volume(const Scene *scene) const
{
  if (objects.size() > 1) {
    /* If node contains multiple objects, do not consider it as homogeneous. */
    return false;
  }

  const Object *object = dynamic_cast<const Object *>(objects[0]);
  if (object) {
    return object->is_homogeneous_volume();
  }

  const Background *background = dynamic_cast<const Background *>(objects[0]);
  return !background->get_shader(scene)->has_volume_spatial_varying;
}

Extrema<float> Octree::get_extrema(const vector<Extrema<float>> &values,
                                   const int3 index_min,
                                   const int3 index_max) const
{
  const blocked_range3d<int> range(
      index_min.x, index_max.x, 32, index_min.y, index_max.y, 32, index_min.z, index_max.z, 32);
  const Extrema<float> identity = {FLT_MAX, 0.0f};

  auto reduction_func = [&](const blocked_range3d<int> &r, Extrema<float> init) -> Extrema<float> {
    for (int z = r.cols().begin(); z < r.cols().end(); ++z) {
      for (int y = r.rows().begin(); y < r.rows().end(); ++y) {
        for (int x = r.pages().begin(); x < r.pages().end(); ++x) {
          init = join(init, values[flatten_index(x, y, z)]);
        }
      }
    }
    return init;
  };

  auto join_func = [](Extrema<float> a, Extrema<float> b) -> Extrema<float> { return join(a, b); };

  return parallel_reduce(range, identity, reduction_func, join_func);
}

bool Octree::should_split(const Scene *scene, std::shared_ptr<OctreeNode> &node) const
{
  if (node->objects.empty() || !node->bbox.valid()) {
    /* If no local volume exists, do not split. */
    return false;
  }

  const int3 index_min = world_to_floor_index(node->bbox.min);
  const int3 index_max = world_to_ceil_index(node->bbox.max);
  const Extrema<float> sigma_extrema = get_extrema(sigmas, index_min, index_max);

  /* Do not split homogeneous volume. Volume stack already skips the zero-density regions. */
  node->sigma.max = sigma_extrema.max;
  node->sigma.min = node->contains_homogeneous_volume(scene) ? sigma_extrema.max :
                                                               sigma_extrema.min;

  /* TODO(weizhen): force subdivision of aggregate nodes that are larger than the volume contained,
   * regardless of the volume's majorant extinction. */

  /* From "Volume Rendering for Pixar's Elemental". */
  if ((node->sigma.max - node->sigma.min) * len(node->bbox.size()) < 1.442f ||
      node->depth == VOLUME_OCTREE_MAX_DEPTH)
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
    internal->children_[i] = std::make_shared<OctreeNode>(bbox, internal->depth + 1);
  }

  return internal;
}

void Octree::recursive_build_(const Scene *scene, shared_ptr<OctreeNode> &octree_node)
{
  if (!should_split(scene, octree_node)) {
    return;
  }

  /* Make the current node an internal node. */
  auto internal = make_internal(octree_node);

  for (auto &child : internal->children_) {
    child->objects.reserve(internal->objects.size());
    for (Node *node : internal->objects) {
      /* TODO(weizhen): more granular than object bounding box is to use the geometry bvh. */
      Object *object = dynamic_cast<Object *>(node);
      if (!object || object->bounds.intersects(child->bbox)) {
        /* If world volume or if object bounding box intersect the node bounding box. */
        child->objects.push_back(object);
      }
    }
    /* TODO(weizhen): check the performance. */
    task_pool.push([&] { recursive_build_(scene, child); });
  }

  octree_node = internal;
}

#ifdef WITH_OPENVDB
openvdb::BoolGrid::ConstPtr Octree::mesh_to_sdf_grid(const Mesh *mesh, const float half_width)
{
  const int num_verts = mesh->get_verts().size();
  std::vector<openvdb::Vec3f> points(num_verts);
  parallel_for(0, num_verts, [&](int i) {
    const float3 &vert = mesh->get_verts()[i];
    points[i] = openvdb::Vec3f(vert.x, vert.y, vert.z);
  });

  const int num_triangles = mesh->num_triangles();
  std::vector<openvdb::Vec3I> triangles(num_triangles);
  parallel_for(0, num_triangles, [&](int i) {
    triangles[i] = openvdb::Vec3I(mesh->get_triangles()[i * 3],
                                  mesh->get_triangles()[i * 3 + 1],
                                  mesh->get_triangles()[i * 3 + 2]);
  });

  /* TODO(weizhen): Should consider object instead of mesh size. */
  const float3 mesh_size = mesh->bounds.size();
  const auto vdb_voxel_size = openvdb::Vec3f(mesh_size.x, mesh_size.y, mesh_size.z) / float(width);

  auto xform = openvdb::math::Transform::createLinearTransform(1.0);
  xform->postScale(vdb_voxel_size);

  auto sdf_grid = openvdb::tools::meshToLevelSet<openvdb::FloatGrid>(
      *xform, points, triangles, half_width);

  return openvdb::tools::sdfInteriorMask(*sdf_grid, vdb_voxel_size.length());
}
#endif

void Octree::flatten_(KernelOctreeNode *knodes,
                      KernelOctreeNode &knode,
                      shared_ptr<OctreeNode> &node,
                      int &node_index)
{
  knode.bbox.max = node->bbox.max;
  knode.bbox.min = node->bbox.min;

  if (auto internal_ptr = std::dynamic_pointer_cast<OctreeInternalNode>(node)) {
    knode.is_leaf = false;
    knode.first_child = node_index;
    node_index += 8;
    /* Loop through all the children. */
    for (int i = 0; i < 8; i++) {
      flatten_(knodes, knodes[knode.first_child + i], internal_ptr->children_[i], node_index);
    }
  }
  else {
    knode.is_leaf = true;
    knode.sigma = node->sigma;
  }
}

void Octree::flatten(KernelOctreeNode *knodes)
{
  int node_index = 0;

  /* World volume. */
  /* TODO(weizhen): is there a better way than putting world volume in the octree array? */
  KernelOctreeNode &knode = knodes[node_index++];
  knode.is_leaf = true;
  knode.sigma = background_density;
  knode.bbox.max = make_float3(FLT_MAX);
  knode.bbox.min = -make_float3(FLT_MAX);

  const int root_index = node_index++;
  flatten_(knodes, knodes[root_index], root_, node_index);
  /* TODO(weizhen): rescale the bounding box to match its resolution, for more robust traversing.
   */
}

__forceinline float3 Octree::world_to_index(float3 p) const
{
  return (p - root_->bbox.min) * world_to_index_scale_;
}

int3 Octree::world_to_floor_index(float3 p) const
{
  const float3 index = floor(world_to_index(p));
  return make_int3(int(index.x), int(index.y), int(index.z));
}

int3 Octree::world_to_ceil_index(float3 p) const
{
  const float3 index = ceil(world_to_index(p));
  return make_int3(int(index.x), int(index.y), int(index.z));
}

__forceinline float3 Octree::index_to_world(int x, int y, int z) const
{
  return root_->bbox.min + make_float3(x, y, z) * index_to_world_scale_;
}

__forceinline float3 Octree::voxel_size() const
{
  return index_to_world_scale_;
}

openvdb::BoolGrid::ConstPtr Octree::get_vdb(const Geometry *geom) const
{
  if (vdb_map.count(geom) > 0) {
    return vdb_map.at(geom);
  }
  /* Create empty grid. */
  return openvdb::BoolGrid::create();
}

#ifdef WITH_OPENVDB
static bool vdb_voxel_intersect(const float3 p_min,
                                const float3 p_max,
                                const Transform itfm,
                                const bool transform_applied,
                                openvdb::BoolGrid::ConstPtr &grid,
                                openvdb::BoolGrid::ConstAccessor &acc)
{
  if (grid->empty()) {
    /* Non-mesh volume. */
    return true;
  }

  /* Transform bounding box from world space to vdb space. */
  openvdb::math::BBox<openvdb::Vec3f> vdb_bbox;
  for (int i = 0; i < 8; i++) {
    const float3 t = make_float3(i & 1, (i >> 1) & 1, (i >> 2) & 1);
    float3 v = mix(p_min, p_max, t);
    if (!transform_applied) {
      v = transform_point(&itfm, v);
    }
    vdb_bbox.expand(openvdb::Vec3f(v.x, v.y, v.z));
  }

  const openvdb::math::CoordBBox coord_bbox(
      openvdb::Coord::floor(grid->worldToIndex(vdb_bbox.min())),
      openvdb::Coord::ceil(grid->worldToIndex(vdb_bbox.max())));

  /* Check if any point in the bounding box lies inside the mesh. */
  for (auto coord = coord_bbox.begin(); coord != coord_bbox.end(); ++coord) {
    if (acc.getValue(*coord)) {
      return true;
    }
  }

  return false;
}
#endif

/* Fill in coordinates for shading the volume density. */
static void fill_shader_input(device_vector<KernelShaderEvalInput> &d_input,
                              const Scene *scene,
                              const Octree *octree,
                              const Node *node,
                              const int3 &index_min,
                              const int3 &index_range)
{
  /* Get object id. */
  const Object *object = dynamic_cast<const Object *>(node);
  const int object_id = object ? object->get_device_index() : OBJECT_NONE;

  /* Get shader id. */
  uint shader_id;
  const Geometry *geom = object ? object->get_geometry() : nullptr;
  if (object) {
    for (Node *node : geom->get_used_shaders()) {
      Shader *shader = static_cast<Shader *>(node);
      if (shader->has_volume) {
        /* TODO(weizhen): support multiple shaders. */
        shader_id = shader->id;
        break;
      }
    }
  }
  else {
    /* World volume. */
    const Background *background = dynamic_cast<const Background *>(node);
    shader_id = background->get_shader(scene)->id;
  }

  /* Get object boundary VDB. */
  openvdb::BoolGrid::ConstPtr grid = octree->get_vdb(geom);

  /* Get object transform. */
  Transform itfm;
  bool transform_applied = true;
  if (!grid->empty()) {
    const Mesh *mesh = static_cast<const Mesh *>(geom);
    if (!mesh->transform_applied) {
      transform_applied = false;
      itfm = transform_inverse(object->get_tfm());
    }
  }

  const float3 voxel_size = octree->voxel_size();

  KernelShaderEvalInput *d_input_data = d_input.data();
  const blocked_range3d<int> range(
      0, index_range.x, 32, 0, index_range.y, 32, 0, index_range.z, 32);

  parallel_for(range, [&](const blocked_range3d<int> &r) {
    /* One accessor per thread is importance for cached access. */
    openvdb::BoolGrid::ConstAccessor acc = grid->getConstAccessor();

    for (int z = r.cols().begin(); z < r.cols().end(); ++z) {
      for (int y = r.rows().begin(); y < r.rows().end(); ++y) {
        for (int x = r.pages().begin(); x < r.pages().end(); ++x) {
          const int offset = flatten_index(x, y, z, index_range);
          const float3 p = octree->index_to_world(
              x + index_min.x, y + index_min.y, z + index_min.z);

#ifdef WITH_OPENVDB
          /* Zero density for cells outside of the mesh. */
          if (!vdb_voxel_intersect(p, p + voxel_size, itfm, transform_applied, grid, acc)) {
            d_input_data[offset * 2 + 0].object = OBJECT_NONE;
            d_input_data[offset * 2 + 1].object = SHADER_NONE;
            continue;
          }
#endif

          KernelShaderEvalInput in;
          in.object = object_id;
          in.prim = __float_as_int(p.x);
          in.u = p.y;
          in.v = p.z;
          d_input_data[offset * 2 + 0] = in;

          in.object = shader_id;
          in.prim = __float_as_int(voxel_size.x);
          in.u = voxel_size.y;
          in.v = voxel_size.z;
          d_input_data[offset * 2 + 1] = in;
        }
      }
    }
  });
}

/* Read back the volume densty. */
static void read_shader_output(const device_vector<float> &d_output,
                               const Octree *octree,
                               const int3 &index_min,
                               const int3 &index_range,
                               const int num_channels,
                               vector<Extrema<float>> &sigmas)
{
  const float *d_output_data = d_output.data();
  const blocked_range3d<int> range(
      0, index_range.x, 32, 0, index_range.y, 32, 0, index_range.z, 32);

  parallel_for(range, [&](const blocked_range3d<int> &r) {
    for (int z = r.cols().begin(); z < r.cols().end(); ++z) {
      for (int y = r.rows().begin(); y < r.rows().end(); ++y) {
        for (int x = r.pages().begin(); x < r.pages().end(); ++x) {
          const int local_index = flatten_index(x, y, z, index_range);
          const int global_index = octree->flatten_index(
              x + index_min.x, y + index_min.y, z + index_min.z);
          sigmas[global_index].min += d_output_data[local_index * num_channels + 0];
          sigmas[global_index].max += d_output_data[local_index * num_channels + 1];
        }
      }
    }
  });
}

void Octree::evaluate_volume_density_(Device *device, Progress &progress, Scene *scene)
{
  /* Initialize density field. */
  const int size = width * width * width;
  sigmas.resize(size);
  parallel_for(0, size, [&](int i) { sigmas[i] = {0.0f, 0.0f}; });

  /* Min and max. */
  const int num_channels = 2;

  auto eval_density = [&](const Node *object,
                          vector<Extrema<float>> &dest,
                          const int3 index_min,
                          const int3 index_range) {
    const int size = index_range.x * index_range.y * index_range.z;

    /* Evaluate shader on device. */
    ShaderEval shader_eval(device, progress);
    shader_eval.eval(
        SHADER_EVAL_VOLUME_DENSITY,
        size * 2,
        num_channels,
        [&](device_vector<KernelShaderEvalInput> &d_input) {
          fill_shader_input(d_input, scene, this, object, index_min, index_range);
          return size;
        },
        [&](device_vector<float> &d_output) {
          read_shader_output(d_output, this, index_min, index_range, num_channels, dest);
        });
  };

  for (const Node *node : root_->objects) {
    const Object *object = dynamic_cast<const Object *>(node);

    /* Evaluate density inside object bounds. */
    const int3 index_min = object ? world_to_floor_index(object->bounds.min) : make_int3(0);
    const int3 index_max = object ? world_to_ceil_index(object->bounds.max) : make_int3(width);

    const int3 index_range = index_max - index_min;

#ifdef WITH_OPENVDB
    /* Create SDF grid for mesh volumes, to determine whether a certain point is in the
     * interior of the mesh. */
    const Geometry *geom = object ? object->get_geometry() : nullptr;
    if (geom && geom->is_mesh() && !vdb_map.count(geom)) {
      const Mesh *mesh = static_cast<const Mesh *>(geom);
      vdb_map[geom] = mesh_to_sdf_grid(mesh, 2.0f);
    }
#endif

    /* TODO(weizhen): specialize homogeneous volume and evaluate less points? */

    /* Evaluate shader on device. */
    eval_density(node, sigmas, index_min, index_range);

    if (!object) {
      /* NOTE: world volume is the first in the list, so until now `sigma_s` only contains density
       * evaluation of the world. */
      background_density = get_extrema(sigmas, index_min, index_max);
    }
  }

  /* Check if we should refine the background density. */
  if (has_world_volume()) {
    if (!root_->bbox.valid()) {
      /* Scene has only world volume, the density was already evaluated in the previous loop. */
      return;
    }
    const Background *background = static_cast<const Background *>(root_->objects[0]);
    const Shader *shader = background->get_shader(scene);
    if (shader->has_volume_spatial_varying) {
      /* For spatial varying volume, the density inside the octree bound box are not representative
       * enough for the whole world, so we evaluate density again in a larger scale. */

      /* Tempararily change the scale for calculating the postions. */
      const float3 previous_index_to_world_scale = index_to_world_scale_;
      index_to_world_scale_ = make_float3(10000.0f) / float(width);

      /* Initialize density field. */
      vector<Extrema<float>> world_density;
      world_density.resize(size);
      parallel_for(0, size, [&](int i) { world_density[i] = {0.0f, 0.0f}; });

      const int3 index_min = make_int3(0);
      const int3 index_max = make_int3(width);
      const int3 index_range = index_max;

      eval_density(background, world_density, index_min, index_range);

      background_density = join(background_density,
                                get_extrema(world_density, index_min, index_max));

      /* Change scale back. */
      index_to_world_scale_ = previous_index_to_world_scale;
    }
  }
}

void Octree::build(Device *device, Progress &progress, Scene *scene)
{
  // if (!root_) {
  //   return;
  // }

  progress.set_substatus("Evaluate volume density");
  double start_time = time_dt();

  evaluate_volume_density_(device, progress, scene);

  std::cout << "Volume density evaluated in " << time_dt() - start_time << " seconds."
            << std::endl;
  VLOG_INFO << "Volume density evaluated in " << time_dt() - start_time << " seconds.";

  progress.set_substatus("Building Octree for volumes");
  start_time = time_dt();

  recursive_build_(scene, root_);

  task_pool.wait_work();

  std::cout << "Built volume Octree with " << num_nodes << " nodes in " << time_dt() - start_time
            << " seconds." << std::endl;
  VLOG_INFO << "Built volume Octree with " << num_nodes << " nodes in " << time_dt() - start_time
            << " seconds.";
}

Octree::Octree(const Scene *scene)
{
  root_ = std::make_shared<OctreeNode>();

  /* Push world volume at the begining of the list. */
  Shader *bg_shader = scene->background->get_shader(scene);
  if (bg_shader->has_volume) {
    root_->objects.push_back(scene->background);
  }
  else {
    background_density = {0.0f, 0.0f};
  }

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
    }
  }

  /* 2^max_depth. */
  width = 1 << VOLUME_OCTREE_MAX_DEPTH;
  /* Some large number if only world volume exists. */
  world_to_index_scale_ = float(width) /
                          (root_->bbox.valid() ? root_->bbox.size() : make_float3(10000.0f));
  index_to_world_scale_ = 1.0f / world_to_index_scale_;

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
  return !(root_->bbox.valid() || has_world_volume());
}

int Octree::get_num_nodes() const
{
  /* Plus one to account for world volume. */
  return num_nodes + 1;
}

bool Octree::has_world_volume() const
{
  return !root_->objects.empty() && dynamic_cast<const Background *>(root_->objects[0]);
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
      file << "ob = bpy.data.objects.new(name = '" << i << " sigma_max = " << knodes[i].sigma.max
           << " sigma_min = " << knodes[i].sigma.min << "' , object_data = cube.data)\n";
      file << "ob.location = (" << center.x << ", " << center.y << ", " << center.z << ")\n";
      const float3 scale = knodes[i].bbox.size() * 0.5f;
      file << "ob.scale = (" << scale.x << ", " << scale.y << ", " << scale.z << ")\n";
      file << "octree.objects.link(ob)\n";
    }
    file << "\nbpy.ops.object.delete()\n\n";
    file << "for obj in octree.objects:\n";
    file << "    obj.select_set(True)\n\n";

    file << "bpy.context.view_layer.objects.active = octree.objects[0]\n";
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
#ifdef WITH_OPENVDB
  for (auto &it : vdb_map) {
    it.second.reset();
  }
#endif
}

CCL_NAMESPACE_END
