/* SPDX-FileCopyrightText: 2024 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "bvh/octree.h"

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
#  include <openvdb/tools/FindActiveValues.h>
#endif

CCL_NAMESPACE_BEGIN

__forceinline int Octree::flatten_index(int x, int y, int z) const
{
  return x + width * (y + z * width);
}

Extrema<float> Octree::get_extrema(const vector<Extrema<float>> &values,
                                   const int3 index_min,
                                   const int3 index_max) const
{
  const blocked_range3d<int> range(
      index_min.x, index_max.x, 32, index_min.y, index_max.y, 32, index_min.z, index_max.z, 32);
  const Extrema<float> identity;

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

bool Octree::should_split(std::shared_ptr<OctreeNode> &node,
                          const float scale,
                          const bool is_homogeneous_volume) const
{
  const int3 index_min = object_to_floor_index(node->bbox.min);
  const int3 index_max = object_to_ceil_index(node->bbox.max);
  const Extrema<float> sigma_extrema = get_extrema(sigmas, index_min, index_max);

  /* Do not split homogeneous volume. Volume stack already skips the zero-density regions. */
  node->sigma.max = sigma_extrema.max;
  node->sigma.min = is_homogeneous_volume ? sigma_extrema.max : sigma_extrema.min;

  /* TODO(weizhen): force subdivision of aggregate nodes that are larger than the volume contained,
   * regardless of the volume's majorant extinction. */

  /* From "Volume Rendering for Pixar's Elemental". */
  if ((node->sigma.max - node->sigma.min) * len(node->bbox.size()) * scale < 1.442f ||
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

void Octree::recursive_build_(shared_ptr<OctreeNode> &octree_node,
                              const float scale,
                              const bool is_homogeneous_volume = false)
{
  if (!should_split(octree_node, scale, is_homogeneous_volume)) {
    return;
  }

  /* Make the current node an internal node. */
  auto internal = make_internal(octree_node);

  for (auto &child : internal->children_) {
    /* TODO(weizhen): check the performance. */
    task_pool.push([&] { recursive_build_(child, scale); });
  }

  octree_node = internal;
}

void Octree::flatten(KernelOctreeNode *knodes,
                     const int current_index,
                     const shared_ptr<OctreeNode> &node,
                     int &child_index) const
{
  KernelOctreeNode &knode = knodes[current_index];
  knode.bbox.max = node->bbox.max;
  knode.bbox.min = node->bbox.min;
  knode.sigma = node->sigma;

  if (auto internal_ptr = std::dynamic_pointer_cast<OctreeInternalNode>(node)) {
    knode.first_child = child_index;
    child_index += 8;
    /* Loop through all the children. */
    for (int i = 0; i < 8; i++) {
      knodes[knode.first_child + i].parent = current_index;
      flatten(knodes, knode.first_child + i, internal_ptr->children_[i], child_index);
    }
  }
  else {
    knode.first_child = -1;
  }
}

/* Convert from position in object space to object. */
__forceinline float3 Octree::object_to_index(float3 p) const
{
  return (p - root_->bbox.min) * object_to_index_scale_;
}

int3 Octree::object_to_floor_index(float3 p) const
{
  const float3 index = floor(object_to_index(p));
  return make_int3(int(index.x), int(index.y), int(index.z));
}

int3 Octree::object_to_ceil_index(float3 p) const
{
  const float3 index = ceil(object_to_index(p));
  return make_int3(int(index.x), int(index.y), int(index.z));
}

/* Convert from index to position in object space. */
__forceinline float3 Octree::index_to_object(int x, int y, int z) const
{
  return root_->bbox.min + make_float3(x, y, z) * index_to_object_scale_;
}

__forceinline float3 Octree::voxel_size() const
{
  return index_to_object_scale_;
}

#ifdef WITH_OPENVDB
static bool vdb_voxel_intersect(const float3 p_min,
                                const float3 p_max,
                                openvdb::BoolGrid::ConstPtr &grid,
                                const openvdb::tools::FindActiveValues<openvdb::BoolTree> &find)
{
  if (grid->empty()) {
    /* Non-mesh volume. */
    return true;
  }

  const openvdb::math::CoordBBox coord_bbox(
      openvdb::Coord::floor(grid->worldToIndex({p_min.x, p_min.y, p_min.z})),
      openvdb::Coord::ceil(grid->worldToIndex({p_max.x, p_max.y, p_max.z})));

  /* Check if the bounding box lies inside or partially overlaps the mesh.
   * For interior mask grids, all the interior voxels are active. */
  return find.anyActiveValues(coord_bbox, true);
}
#endif

/* Fill in coordinates for shading the volume density. */
static void fill_shader_input(device_vector<KernelShaderEvalInput> &d_input,
                              const Octree *octree,
                              const Object *object,
                              const Shader *shader,
                              const int width,
                              openvdb::BoolGrid::ConstPtr &interior_mask)
{
  /* Get object id. */
  const int object_id = object ? object->get_device_index() : OBJECT_NONE;

  /* Get shader id. */
  const uint shader_id = shader->id;

  const int num_samples = VolumeManager::is_homogeneous_volume(object, shader) ? 1 : 16;

  KernelShaderEvalInput *d_input_data = d_input.data();
  d_input_data[0].object = num_samples;

  const float3 voxel_size = octree->voxel_size();
  const blocked_range3d<int> range(0, width, 8, 0, width, 8, 0, width, 8);
  parallel_for(range, [&](const blocked_range3d<int> &r) {
    /* One accessor per thread is important for cached access. */
    const auto find = openvdb::tools::FindActiveValues(interior_mask->tree());

    for (int z = r.cols().begin(); z < r.cols().end(); ++z) {
      for (int y = r.rows().begin(); y < r.rows().end(); ++y) {
        for (int x = r.pages().begin(); x < r.pages().end(); ++x) {
          const int offset = octree->flatten_index(x, y, z);
          /* TODO(weizhen): check if we can use index directly instead of position for mesh
           * interior. */
          const float3 p = octree->index_to_object(x, y, z);

#ifdef WITH_OPENVDB
          /* Zero density for cells outside of the mesh. */
          if (!vdb_voxel_intersect(p, p + voxel_size, interior_mask, find)) {
            d_input_data[offset * 2 + 1].object = OBJECT_NONE;
            d_input_data[offset * 2 + 2].object = SHADER_NONE;
            continue;
          }
#endif

          KernelShaderEvalInput in;
          in.object = object_id;
          in.prim = __float_as_int(p.x);
          in.u = p.y;
          in.v = p.z;
          d_input_data[offset * 2 + 1] = in;

          in.object = shader_id;
          in.prim = __float_as_int(voxel_size.x);
          in.u = voxel_size.y;
          in.v = voxel_size.z;
          d_input_data[offset * 2 + 2] = in;
        }
      }
    }
  });
}

/* Read back the volume densty. */
static void read_shader_output(const device_vector<float> &d_output,
                               const Octree *octree,
                               const int num_channels,
                               const int width,
                               vector<Extrema<float>> &sigmas)
{
  const float *d_output_data = d_output.data();
  const blocked_range3d<int> range(0, width, 32, 0, width, 32, 0, width, 32);

  parallel_for(range, [&](const blocked_range3d<int> &r) {
    for (int z = r.cols().begin(); z < r.cols().end(); ++z) {
      for (int y = r.rows().begin(); y < r.rows().end(); ++y) {
        for (int x = r.pages().begin(); x < r.pages().end(); ++x) {
          const int index = octree->flatten_index(x, y, z);
          sigmas[index].min += d_output_data[index * num_channels + 0];
          sigmas[index].max += d_output_data[index * num_channels + 1];
        }
      }
    }
  });
}

void Octree::evaluate_volume_density_(Device *device,
                                      Progress &progress,
                                      const Object *object,
                                      const Shader *shader,
                                      openvdb::BoolGrid::ConstPtr &interior_mask)
{
  width = VolumeManager::is_homogeneous_volume(object, shader) ? 1 : 1 << VOLUME_OCTREE_MAX_DEPTH;
  object_to_index_scale_ = float(width) / root_->bbox.size();
  index_to_object_scale_ = 1.0f / object_to_index_scale_;

  /* Initialize density field. */
  /* TODO(weizhen): maybe lower the resolution. */
  const int size = width * width * width;
  sigmas.resize(size);
  parallel_for(0, size, [&](int i) { sigmas[i] = {0.0f, 0.0f}; });

  /* Min and max. */
  const int num_channels = 2;

  /* Evaluate shader on device. */
  ShaderEval shader_eval(device, progress);
  shader_eval.eval(
      SHADER_EVAL_VOLUME_DENSITY,
      size * 2 + 1,
      num_channels,
      [&](device_vector<KernelShaderEvalInput> &d_input) {
        fill_shader_input(d_input, this, object, shader, width, interior_mask);
        return size;
      },
      [&](device_vector<float> &d_output) {
        read_shader_output(d_output, this, num_channels, width, sigmas);
      });
}

float Octree::volume_scale_(const Object *object) const
{
  if (object) {
    const Geometry *geom = object->get_geometry();
    if (geom->is_volume()) {
      const Volume *volume = static_cast<const Volume *>(geom);
      if (volume->get_object_space()) {
        if (volume->transform_applied) {
          const float3 unit = normalize(one_float3());
          return 1.0f / len(transform_direction(&object->get_tfm(), unit));
        }
      }
      else {
        if (!volume->transform_applied) {
          const float3 unit = normalize(one_float3());
          return len(transform_direction(&object->get_tfm(), unit));
        }
      }
    }
    else {
      /* TODO(weizhen): use the maximal scale of all instances. */
    }
  }

  return 1.0f;
}

void Octree::build(Device *device,
                   Progress &progress,
                   const Object *object,
                   const Shader *shader,
                   openvdb::BoolGrid::ConstPtr &interior_mask)
{
  progress.set_substatus("Evaluate volume density");

  evaluate_volume_density_(device, progress, object, shader, interior_mask);
  if (progress.get_cancel()) {
    return;
  }

  progress.set_substatus("Building Octree for volumes");

  const float scale = volume_scale_(object);
  const bool is_homogeneus = VolumeManager::is_homogeneous_volume(object, shader);
  recursive_build_(root_, scale, is_homogeneus);

  task_pool.wait_work();

  is_built_ = true;
  sigmas.clear();
}

Octree::Octree(const BoundBox &bbox)
{
  root_ = std::make_shared<OctreeNode>(bbox, 0);
  is_built_ = false;
}

bool Octree::is_built() const
{
  return is_built_;
}

int Octree::get_num_nodes() const
{
  return num_nodes;
}

std::shared_ptr<OctreeNode> Octree::get_root() const
{
  return root_;
}

void Octree::visualize(const KernelOctreeNode *knodes, const int root, std::ofstream &file) const
{
  std::string str = "vertices = [";
  for (int i = root; i < get_num_nodes() + root; i++) {
    if (knodes[i].is_leaf()) {
      continue;
    }
    const float3 mid = knodes[i].bbox.center();
    const float3 max = knodes[i].bbox.max;
    const float3 min = knodes[i].bbox.min;
    const std::string mid_x = to_string(mid.x), mid_y = to_string(mid.y), mid_z = to_string(mid.z),
                      min_x = to_string(min.x), min_y = to_string(min.y), min_z = to_string(min.z),
                      max_x = to_string(max.x), max_y = to_string(max.y), max_z = to_string(max.z);
    /* Create three orthogonal faces. */
    str += "(" + mid_x + "," + mid_y + "," + min_z + "), (" + mid_x + "," + mid_y + "," + max_z +
           "), (" + mid_x + "," + max_y + "," + max_z + "), (" + mid_x + "," + max_y + "," +
           min_z + "), (" + mid_x + "," + min_y + "," + min_z + "), (" + mid_x + "," + min_y +
           "," + max_z + "), ";
    str += "(" + min_x + "," + mid_y + "," + mid_z + "), (" + max_x + "," + mid_y + "," + mid_z +
           "), (" + max_x + "," + mid_y + "," + max_z + "), (" + min_x + "," + mid_y + "," +
           max_z + "), (" + min_x + "," + mid_y + "," + min_z + "), (" + max_x + "," + mid_y +
           "," + min_z + "), ";
    str += "(" + mid_x + "," + min_y + "," + mid_z + "), (" + mid_x + "," + max_y + "," + mid_z +
           "), (" + max_x + "," + max_y + "," + mid_z + "), (" + max_x + "," + min_y + "," +
           mid_z + "), (" + min_x + "," + min_y + "," + mid_z + "), (" + min_x + "," + max_y +
           "," + mid_z + "), ";
  }
  str +=
      "]\nr = range(len(vertices))\n"
      "edges = [(i, i+1 if i%6<5 else i-4) for i in r]\n"
      "mesh = bpy.data.meshes.new('Octree')\n"
      "mesh.from_pydata(vertices, edges, [])\n"
      "mesh.update()\n"
      "obj = bpy.data.objects.new('Octree', mesh)\n"
      "octree.objects.link(obj)\n"
      "bpy.context.view_layer.objects.active = obj\n"
      "bpy.ops.object.mode_set(mode='EDIT')\n";
  file << str;

  const float3 center = knodes[root].bbox.center();
  const float3 size = knodes[root].bbox.size() * 0.5f;
  file << "bpy.ops.mesh.primitive_cube_add(location = " << center << ", scale = " << size << ")\n";
  file << "bpy.ops.mesh.delete(type='ONLY_FACE')\n"
          "bpy.ops.object.mode_set(mode='OBJECT')\n"
          "obj.select_set(True)\n";
}

CCL_NAMESPACE_END
