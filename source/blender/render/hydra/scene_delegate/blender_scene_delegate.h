/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/base/gf/vec2f.h>
#include <pxr/imaging/hd/sceneDelegate.h>

#include "BKE_context.h"
#include "DEG_depsgraph.h"

#include "CLG_log.h"

#include "instancer.h"
#include "light.h"
#include "mesh.h"
#include "object.h"
#include "world.h"

namespace blender::render::hydra {

extern struct CLG_LogRef *LOG_RENDER_HYDRA_SCENE;

class BlenderSceneDelegate : public pxr::HdSceneDelegate {
  friend ObjectData;   /* has access to instances */
  friend MeshData;     /* has access to materials */
  friend MaterialData; /* has access to objects and instancers */

 public:
  enum class EngineType { VIEWPORT = 1, FINAL, PREVIEW };

  BlenderSceneDelegate(pxr::HdRenderIndex *parent_index,
                       pxr::SdfPath const &delegate_id,
                       BlenderSceneDelegate::EngineType engine_type,
                       const std::string &render_delegate_name);
  ~BlenderSceneDelegate() override = default;

  /* Delegate methods */
  pxr::HdMeshTopology GetMeshTopology(pxr::SdfPath const &id) override;
  pxr::GfMatrix4d GetTransform(pxr::SdfPath const &id) override;
  pxr::VtValue Get(pxr::SdfPath const &id, pxr::TfToken const &key) override;
  pxr::VtValue GetLightParamValue(pxr::SdfPath const &id, pxr::TfToken const &key) override;
  pxr::HdPrimvarDescriptorVector GetPrimvarDescriptors(
      pxr::SdfPath const &id, pxr::HdInterpolation interpolation) override;
  pxr::SdfPath GetMaterialId(pxr::SdfPath const &rprim_id) override;
  pxr::VtValue GetMaterialResource(pxr::SdfPath const &material_id) override;
  bool GetVisible(pxr::SdfPath const &id) override;
  bool GetDoubleSided(pxr::SdfPath const &id) override;
  pxr::HdCullStyle GetCullStyle(pxr::SdfPath const &id) override;
  pxr::SdfPath GetInstancerId(pxr::SdfPath const &prim_id) override;
  pxr::SdfPathVector GetInstancerPrototypes(pxr::SdfPath const &instancer_id) override;
  pxr::VtIntArray GetInstanceIndices(pxr::SdfPath const &instancer_id,
                                     pxr::SdfPath const &prototype_id) override;
  pxr::GfMatrix4d GetInstancerTransform(pxr::SdfPath const &instancer_id) override;

  void populate(Depsgraph *depsgraph, bContext *context);
  void clear();

  EngineType engine_type;
  Depsgraph *depsgraph = nullptr;
  bContext *context = nullptr;
  View3D *view3d = nullptr;
  Scene *scene = nullptr;

  std::string render_delegate_name;

 private:
  pxr::SdfPath prim_id(ID *id, const char *prefix) const;
  pxr::SdfPath object_prim_id(Object *object) const;
  pxr::SdfPath material_prim_id(Material *mat) const;
  pxr::SdfPath instancer_prim_id(Object *object) const;
  pxr::SdfPath world_prim_id() const;

  ObjectData *object_data(pxr::SdfPath const &id) const;
  MeshData *mesh_data(pxr::SdfPath const &id) const;
  LightData *light_data(pxr::SdfPath const &id) const;
  MaterialData *material_data(pxr::SdfPath const &id) const;
  InstancerData *instancer_data(pxr::SdfPath const &id, bool child_id = false) const;

  void update_objects(Object *object);
  void update_instancers(Object *object);
  void update_world();
  void check_updates();
  void add_new_objects();
  void remove_unused_objects();
  void update_visibility();

  ObjectDataMap objects_;
  MaterialDataMap materials_;
  InstancerDataMap instancers_;
  std::unique_ptr<WorldData> world_data_;
};

}  // namespace blender::render::hydra
