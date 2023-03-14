/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include "graph/node.h"

#include "bvh/params.h"

#include "scene/attribute.h"

#include "util/boundbox.h"
#include "util/set.h"
#include "util/transform.h"
#include "util/types.h"
#include "util/vector.h"

CCL_NAMESPACE_BEGIN

class BVH;
class Device;
class DeviceScene;
class Mesh;
class Progress;
class RenderStats;
class Scene;
class SceneParams;
class Shader;
class Volume;
class Object;
struct PackedBVH;

/* Geometry
 *
 * Base class for geometric types like Mesh and Hair. */

class Geometry : public Node {
 public:
  NODE_ABSTRACT_DECLARE

  enum Type {
    MESH,
    HAIR,
    VOLUME,
    POINTCLOUD,
  };

  Type geometry_type;

  /* Attributes */
  AttributeSet attributes;

  /* Shaders */
  NODE_SOCKET_API_ARRAY(array<Node *>, used_shaders)

  /* Transform */
  BoundBox bounds;
  bool transform_applied;
  bool transform_negative_scaled;
  Transform transform_normal;

  /* Motion Blur */
  NODE_SOCKET_API(uint, motion_steps)
  NODE_SOCKET_API(bool, use_motion_blur)

  /* Maximum number of motion steps supported (due to Embree). */
  static const uint MAX_MOTION_STEPS = 129;

  /* BVH */
  BVH *bvh;
  size_t attr_map_offset;
  size_t prim_offset;
  size_t motion_key_offset;

  /* Shader Properties */
  bool has_volume;         /* Set in the device_update_flags(). */
  bool has_surface_bssrdf; /* Set in the device_update_flags(). */

  /* Update Flags */
  bool need_update_rebuild;
  bool need_update_bvh_for_offset;

  /* Index into scene->geometry (only valid during update) */
  size_t index;

  /* Constructor/Destructor */
  explicit Geometry(const NodeType *node_type, const Type type);
  virtual ~Geometry();

  /* Geometry */
  virtual void clear(bool preserve_shaders = false);
  virtual void compute_bounds() = 0;
  virtual void apply_transform(const Transform &tfm, const bool apply_to_motion) = 0;

  /* Attribute Requests */
  bool need_attribute(Scene *scene, AttributeStandard std);
  bool need_attribute(Scene *scene, ustring name);

  AttributeRequestSet needed_attributes();

  /* UDIM */
  virtual void get_uv_tiles(ustring map, unordered_set<int> &tiles) = 0;

  /* Convert between normalized -1..1 motion time and index in the
   * VERTEX_MOTION attribute. */
  float motion_time(int step) const;
  int motion_step(float time) const;

  /* BVH */
  void create_new_bvh_if_needed(Object *object,
                                Device *device,
                                DeviceScene *dscene,
                                SceneParams *params);
  void compute_bvh(Device *device,
                   DeviceScene *dscene,
                   SceneParams *params,
                   Progress *progress,
                   size_t n,
                   size_t total);

  virtual PrimitiveType primitive_type() const = 0;

  /* Check whether the geometry should have own BVH built separately. Briefly,
   * own BVH is needed for geometry, if:
   *
   * - It is instanced multiple times, so each instance object should share the
   *   same BVH tree.
   * - Special ray intersection is needed, for example to limit subsurface rays
   *   to only the geometry itself.
   * - The BVH layout requires the top level to only contain instances.
   */
  bool need_build_bvh(BVHLayout layout) const;

  /* Test if the geometry should be treated as instanced. */
  bool is_instanced() const;

  bool has_true_displacement() const;
  bool has_motion_blur() const;
  bool has_voxel_attributes() const;

  bool is_mesh() const
  {
    return geometry_type == MESH;
  }

  bool is_hair() const
  {
    return geometry_type == HAIR;
  }

  bool is_pointcloud() const
  {
    return geometry_type == POINTCLOUD;
  }

  bool is_volume() const
  {
    return geometry_type == VOLUME;
  }

  /* Updates */
  void tag_update(Scene *scene, bool rebuild);

  void tag_bvh_update(bool rebuild);
};

// FRL_CGR BEGIN
/* Geometry Sizes */
struct GeometrySizes {
  size_t vert_size;
  size_t tri_size;

  size_t curve_size;
  size_t curve_key_size;
  size_t curve_segment_size;

  size_t point_size;

  size_t patch_size;
  size_t face_size;
  size_t corner_size;

  size_t *num_geometries;
  size_t *vert_offsets;
  size_t *motion_vert_offsets;
};

struct AttributeSizes {
  size_t attr_float_size;
  size_t attr_float2_size;
  size_t attr_float3_size;
  size_t attr_float4_size;
  size_t attr_uchar4_size;
};

// FRL_CGR END

/* Geometry Manager */

class GeometryManager {
  uint32_t update_flags;

 public:
  enum : uint32_t {
    UV_PASS_NEEDED = (1 << 0),
    MOTION_PASS_NEEDED = (1 << 1),
    GEOMETRY_MODIFIED = (1 << 2),
    OBJECT_MANAGER = (1 << 3),
    MESH_ADDED = (1 << 4),
    MESH_REMOVED = (1 << 5),
    HAIR_ADDED = (1 << 6),
    HAIR_REMOVED = (1 << 7),
    POINT_ADDED = (1 << 12),
    POINT_REMOVED = (1 << 13),

    SHADER_ATTRIBUTE_MODIFIED = (1 << 8),
    SHADER_DISPLACEMENT_MODIFIED = (1 << 9),

    GEOMETRY_ADDED = MESH_ADDED | HAIR_ADDED | POINT_ADDED,
    GEOMETRY_REMOVED = MESH_REMOVED | HAIR_REMOVED | POINT_REMOVED,

    TRANSFORM_MODIFIED = (1 << 10),

    VISIBILITY_MODIFIED = (1 << 11),

    /* tag everything in the manager for an update */
    UPDATE_ALL = ~0u,

    UPDATE_NONE = 0u,
  };

  /* Update Flags */
  bool need_flags_update;

  /* Constructor/Destructor */
  GeometryManager();
  ~GeometryManager();

  /* Device Updates */
  void device_update_preprocess(Device *device, Scene *scene, Progress &progress);
  void device_update(Device *device, DeviceScene *dscene, Scene *scene, Progress &progress);
  void device_free(Device *device, DeviceScene *dscene, bool force_free);

  /* Updates */
  void tag_update(Scene *scene, uint32_t flag);

  bool need_update() const;
  void device_scene_clear_modified(DeviceScene *dscene);

  /* Statistics */
  void collect_statistics(const Scene *scene, RenderStats *stats);

  size_t createObjectBVHs(Device *device,
                          DeviceScene *dscene,
                          Scene *scene,
                          const BVHLayout bvh_layout,
                          bool &need_update_scene_bvh);
  void updateSceneBVHs(Device *device, DeviceScene *dscene, Scene *scene, Progress &progress);
  void clearShaderUpdateTags(Scene *scene);
  void clearGeometryUpdateAndModifiedTags(Scene *scene);
  void deviceDataXferAndBVHUpdate(int idx,
                                  Scene *scene,
                                  DeviceScene *dscene,
                                  GeometrySizes &sizes,
                                  AttributeSizes &attrib_sizes,
                                  const BVHLayout bvh_layout,
                                  size_t num_bvh,
				  bool can_refit,
				  bool need_update_scene_bvh,
                                  Progress &progress);
  void updateObjectBounds(Scene *scene);
  void tesselate(Scene *scene, size_t total_tess_needed, Progress &progress);
  void preTessDispNormalAndVerticesSetup(Device *device,
                                         Scene *scene,
                                         bool &true_displacement_used,
                                         bool &curve_shadow_transparency_used,
                                         size_t &total_tess_needed);
  static void device_update_sub_bvh(Device *device,
                                    DeviceScene *dscene,
                                    BVH *bvh,
                                    BVH *sub_bvh,
                                    bool can_refit,
                                    size_t n,
                                    size_t total,
                                    Progress *progress);
 protected:
  bool displace(Device *device, Scene *scene, Mesh *mesh, Progress &progress);

  void create_volume_mesh(const Scene *scene, Volume *volume, Progress &progress);

  /* Attributes */
  void update_osl_globals(Device *device, Scene *scene);
  void update_svm_attributes(Device *device,
                             DeviceScene *dscene,
                             Scene *scene,
                             vector<AttributeRequestSet> &geom_attributes,
                             vector<AttributeRequestSet> &object_attributes);

  /* Compute verts/triangles/curves offsets in global arrays. */
  void geom_calc_offset(Scene *scene, GeometrySizes *sizes);
  void attrib_calc_sizes(Scene *scene,
                         AttributeSizes *p_sizes,
                         vector<AttributeRequestSet> &geom_attributes,
                         vector<AttributeRequestSet> &object_attributes,
                         vector<AttributeSet> &object_attribute_values);

  void device_update_object(Device *device, DeviceScene *dscene, Scene *scene, Progress &progress);

  void device_update_mesh_preprocess(
      Device *device, DeviceScene *dscene, Scene *scene, GeometrySizes *sizes, Progress &progress);
  void device_update_mesh(Device *device,
                          DeviceScene *dscene,
                          /*Scene *scene,*/ const GeometrySizes *sizes,
                          Progress &progress);
  void device_update_host_pointers(Device *device,
                                   DeviceScene *dscene,
                                   DeviceScene *sub_dscene,
                                   GeometrySizes *p_sizes);
  bool displacement_and_curve_shadow_transparency(Scene *scene,
                                                  Device *device,
                                                  DeviceScene *dscene,
                                                  GeometrySizes *sizes,
                                                  AttributeSizes *attrib_sizes,
                                                  vector<AttributeRequestSet> &geom_attributes,
                                                  vector<AttributeRequestSet> &object_attributes,
                                                  vector<AttributeSet> &object_attribute_values,
                                                  Progress &progress);

  void gather_attributes(Scene *scene,
                         vector<AttributeRequestSet> &geom_attributes,
                         vector<AttributeRequestSet> &object_attributes,
                         vector<AttributeSet> &object_attribute_values,
                         AttributeSizes *sizes);
  bool device_update_attributes_preprocess(Device *device,
                                           DeviceScene *dscene,
                                           Scene *scene,
                                           vector<AttributeRequestSet> &geom_attributes,
                                           vector<AttributeRequestSet> &object_attributes,
                                           vector<AttributeSet> &object_attribute_values,
                                           AttributeSizes *sizes,
                                           Progress &progress);
  void device_update_attributes(Device *device,
                                DeviceScene *dscene,
                                const AttributeSizes *sizes,
                                Progress &progress);

  bool device_update_bvh_preprocess(Device *device,
                                    DeviceScene *dscene,
                                    Scene *scene,
                                    Progress &progress);
  void device_update_bvh(Device *device,
                         DeviceScene *dscene,
                         Scene *scene,
                         bool can_refit,
                         size_t n,
                         size_t total,
                         Progress &progress);
  void device_update_bvh_postprocess(Device *device,
                                     DeviceScene *dscene,
                                     Scene *scene,
                                     Progress &progress);

  void device_update_displacement_images(Device *device, Scene *scene, Progress &progress);

  void device_update_volume_images(Device *device, Scene *scene, Progress &progress);

 private:
  vector<Object> object_pool;
  static void update_attribute_element_offset(Geometry *geom,
                                              device_vector<float> &attr_float,
                                              size_t &attr_float_offset,
                                              device_vector<float2> &attr_float2,
                                              size_t &attr_float2_offset,
                                              device_vector<packed_float3> &attr_float3,
                                              size_t &attr_float3_offset,
                                              device_vector<float4> &attr_float4,
                                              size_t &attr_float4_offset,
                                              device_vector<uchar4> &attr_uchar4,
                                              size_t &attr_uchar4_offset,
                                              Attribute *mattr,
                                              AttributePrimitive prim,
                                              TypeDesc &type,
                                              AttributeDescriptor &desc);
};

CCL_NAMESPACE_END

#endif /* __GEOMETRY_H__ */
