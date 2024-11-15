/* SPDX-FileCopyrightText: 2020-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "graph/node.h"

#include "scene/mesh.h"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

CCL_NAMESPACE_BEGIN

class Device;
class DeviceScene;
class Progress;
class Scene;
class Octree;
class Object;

class Volume : public Mesh {
 public:
  NODE_DECLARE

  Volume();

  NODE_SOCKET_API(float, clipping)
  NODE_SOCKET_API(bool, object_space)
  NODE_SOCKET_API(float, velocity_scale)

  virtual void clear(bool preserve_shaders = false) override;
};

class VolumeManager {
 public:
  VolumeManager();
  ~VolumeManager();

  void device_update(Device *device, DeviceScene *dscene, Scene *scene, Progress &progress);
  void device_free(DeviceScene *dscene);
  bool need_update() const;
  /* TODO(weizhen): check if `shader->has_volume_spatial_varying` is reliable in these cases. */
  /* TODO(weizhen): check if all cases are covered. */
  void tag_update();
  void tag_update(const Shader *shader);
  void tag_update(const Object *object, const uint32_t flag);
  void tag_update(const Geometry *geometry);
  /* Check whether the shader is a homogeneous volume. */
  static bool is_homogeneous_volume(const Object *object, const Shader *shader);

 private:
  void initialize_octree_(const Scene *scene);
  void build_octree_(Device *device, Progress &progress);
  void flatten_octree_(DeviceScene *, const Scene *) const;
  /* When running Blender with `--debug-cycles`, an Octree visualization is written to `filename`,
   * which is a Python script that can be run inside Blender. */
  std::string visualize_octree_(const DeviceScene *, const char *filename) const;
  bool need_rebuild_;
  std::map<std::pair<const Object *, const Shader *>, std::shared_ptr<Octree>> object_octrees_;
#ifdef WITH_OPENVDB
  std::map<std::pair<const Geometry *, const Shader *>, openvdb::BoolGrid::ConstPtr> vdb_map_;
  openvdb::BoolGrid::ConstPtr get_vdb_(const Geometry *, const Shader *) const;
  openvdb::BoolGrid::ConstPtr mesh_to_sdf_grid_(const Mesh *mesh,
                                                const Shader *shader,
                                                const float half_width);
#endif
  bool update_visualization_ = false;
};

CCL_NAMESPACE_END
