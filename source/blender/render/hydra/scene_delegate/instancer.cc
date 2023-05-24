/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/base/gf/vec2f.h>
#include <pxr/imaging/hd/light.h>

#include "BKE_object.h"
#include "DEG_depsgraph_query.h"

#include "blender_scene_delegate.h"
#include "instancer.h"

namespace blender::render::hydra {

InstancerData::InstancerData(BlenderSceneDelegate *scene_delegate,
                             Object *object,
                             pxr::SdfPath const &prim_id)
    : ObjectData(scene_delegate, object, prim_id)
{
}

bool InstancerData::is_supported(Object *object)
{
  switch (object->type) {
    case OB_MESH:
    case OB_SURF:
    case OB_FONT:
    case OB_CURVES_LEGACY:
    case OB_MBALL:
    case OB_LAMP:
      return true;

    default:
      break;
  }
  return false;
}

bool InstancerData::is_visible(BlenderSceneDelegate *scene_delegate, Object *object)
{
  eEvaluationMode deg_mode = DEG_get_mode(scene_delegate->depsgraph);
  int vis = BKE_object_visibility(object, deg_mode);
  bool ret = vis & OB_VISIBLE_INSTANCES;
  if (deg_mode == DAG_EVAL_VIEWPORT) {
    ret &= BKE_object_is_visible_in_viewport(scene_delegate->view3d, object);
  }
  else {
    if (ret) {
      /* If some of parent object is instancer, then currenct object as instancer
       * is invisible in Final render */
      for (Object *ob = object->parent; ob != nullptr; ob = ob->parent) {
        if (ob->transflag & OB_DUPLI) {
          ret = false;
          break;
        }
      }
    }
  }
  return ret;
}

void InstancerData::init()
{
  ID_LOG(2, "");
  write_instances();
}

void InstancerData::insert()
{
  ID_LOG(2, "");
  scene_delegate_->GetRenderIndex().InsertInstancer(scene_delegate_, prim_id);
}

void InstancerData::remove()
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 2, "%s", prim_id.GetText());
  for (auto &it : mesh_instances_) {
    it.second.data->remove();
  }
  scene_delegate_->GetRenderIndex().RemoveInstancer(prim_id);

  for (auto &it : light_instances_) {
    it.second.transforms.clear();
    update_light_instance(it.second);
  }
}

void InstancerData::update()
{
  ID_LOG(2, "");
  Object *object = (Object *)id;
  if (id->recalc & ID_RECALC_GEOMETRY ||
      (object->data && ((ID *)object->data)->recalc & ID_RECALC_GEOMETRY) ||
      id->recalc & ID_RECALC_TRANSFORM)
  {
    write_instances();
    scene_delegate_->GetRenderIndex().GetChangeTracker().MarkInstancerDirty(
        prim_id, pxr::HdChangeTracker::AllDirty);
  }
}

pxr::VtValue InstancerData::get_data(pxr::TfToken const &key) const
{
  ID_LOG(3, "%s", key.GetText());
  pxr::VtValue ret;
  if (key == pxr::HdInstancerTokens->instanceTransform) {
    ret = mesh_transforms_;
  }
  return ret;
}

bool InstancerData::update_visibility()
{
  bool prev_visible = visible;
  visible = is_visible(scene_delegate_, (Object *)id);
  bool ret = visible != prev_visible;

  if (ret) {
    auto &change_tracker = scene_delegate_->GetRenderIndex().GetChangeTracker();
    change_tracker.MarkInstancerDirty(prim_id, pxr::HdChangeTracker::DirtyVisibility);
    for (auto &it : mesh_instances_) {
      it.second.data->visible = visible;
      for (auto &p : it.second.data->submesh_paths()) {
        change_tracker.MarkRprimDirty(p, pxr::HdChangeTracker::DirtyVisibility);
      }
    }
    char name[16];
    for (auto &it : light_instances_) {
      for (int i = 0; i < it.second.count; ++i) {
        snprintf(name, 16, "L_%08x", i);
        change_tracker.MarkRprimDirty(it.second.data->prim_id.AppendElementString(name),
                                      pxr::HdChangeTracker::DirtyVisibility);
      }
    }
  }
  return ret;
}

pxr::GfMatrix4d InstancerData::get_transform(pxr::SdfPath const &id) const
{
  LightInstance *l_inst = light_instance(id);
  if (l_inst) {
    return l_inst->transforms[light_prim_id_index(id)];
  }

  /* Mesh instance transform must be identity */
  return pxr::GfMatrix4d(1.0);
}

pxr::HdPrimvarDescriptorVector InstancerData::primvar_descriptors(
    pxr::HdInterpolation interpolation) const
{
  pxr::HdPrimvarDescriptorVector primvars;
  if (interpolation == pxr::HdInterpolationInstance) {
    primvars.emplace_back(
        pxr::HdInstancerTokens->instanceTransform, interpolation, pxr::HdPrimvarRoleTokens->none);
  }
  return primvars;
}

pxr::VtIntArray InstancerData::indices(pxr::SdfPath const &id) const
{
  return mesh_instance(id)->indices;
}

ObjectData *InstancerData::object_data(pxr::SdfPath const &id) const
{
  MeshInstance *m_inst = mesh_instance(id);
  if (m_inst) {
    return m_inst->data.get();
  }
  LightInstance *l_inst = light_instance(id);
  if (l_inst) {
    return l_inst->data.get();
  }
  return nullptr;
}

pxr::SdfPathVector InstancerData::prototypes() const
{
  pxr::SdfPathVector paths;
  for (auto &it : mesh_instances_) {
    for (auto &p : it.second.data->submesh_paths()) {
      paths.push_back(p);
    }
  }
  return paths;
}

void InstancerData::check_update(Object *object)
{
  pxr::SdfPath path = object_prim_id(object);
  MeshInstance *m_inst = mesh_instance(path);
  if (m_inst) {
    if (!is_instance_visible(object)) {
      m_inst->data->remove();
      mesh_instances_.erase(path);
      scene_delegate_->GetRenderIndex().GetChangeTracker().MarkInstancerDirty(
          prim_id, pxr::HdChangeTracker::AllDirty);
      return;
    }

    m_inst->data->update();

    if (m_inst->data->id->recalc & ID_RECALC_TRANSFORM) {
      write_instances();
      scene_delegate_->GetRenderIndex().GetChangeTracker().MarkInstancerDirty(
          prim_id, pxr::HdChangeTracker::AllDirty);
    }
    return;
  }

  LightInstance *l_inst = light_instance(path);
  if (l_inst) {
    if (!is_instance_visible(object)) {
      l_inst->transforms.clear();
      update_light_instance(*l_inst);
      light_instances_.erase(path);
      return;
    }

    Object *obj = (Object *)l_inst->data->id;
    if (obj->id.recalc & (ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY) ||
        ((ID *)obj->data)->recalc & ID_RECALC_GEOMETRY)
    {
      write_instances();
    }
    return;
  }

  /* Checking if object wasn't added to instances before */
  if (is_supported(object) && is_instance_visible(object)) {
    bool do_write_instances = false;
    ListBase *lb = object_duplilist(
        scene_delegate_->depsgraph, scene_delegate_->scene, (Object *)id);
    LISTBASE_FOREACH (DupliObject *, dupli, lb) {
      if (dupli->ob == object) {
        do_write_instances = true;
        break;
      }
    }
    free_object_duplilist(lb);

    if (do_write_instances) {
      write_instances();
      if (!mesh_instances_.empty()) {
        scene_delegate_->GetRenderIndex().GetChangeTracker().MarkInstancerDirty(
            prim_id, pxr::HdChangeTracker::AllDirty);
      }
    }
  }
}

void InstancerData::check_remove(std::set<std::string> &available_objects)
{
  bool ret = false;
  for (auto it = mesh_instances_.begin(); it != mesh_instances_.end(); ++it) {
    if (available_objects.find(it->first.GetName()) != available_objects.end()) {
      continue;
    }
    it->second.data->remove();
    mesh_instances_.erase(it);
    it = mesh_instances_.begin();
    ret = true;
  }
  if (ret) {
    write_instances();
    scene_delegate_->GetRenderIndex().GetChangeTracker().MarkInstancerDirty(
        prim_id, pxr::HdChangeTracker::AllDirty);
  }

  for (auto it = light_instances_.begin(); it != light_instances_.end(); ++it) {
    if (available_objects.find(it->first.GetName()) != available_objects.end()) {
      continue;
    }
    it->second.transforms.clear();
    update_light_instance(it->second);

    light_instances_.erase(it);
    it = light_instances_.begin();
  }
}

void InstancerData::available_materials(std::set<pxr::SdfPath> &paths) const
{
  for (auto &it : mesh_instances_) {
    ((MeshData *)it.second.data.get())->available_materials(paths);
  }
}

void InstancerData::update_as_parent()
{
  write_instances();
  scene_delegate_->GetRenderIndex().GetChangeTracker().MarkInstancerDirty(
      prim_id, pxr::HdChangeTracker::AllDirty);
}

void InstancerData::update_double_sided(MaterialData *mat_data)
{
  for (auto &it : mesh_instances_) {
    it.second.data->update_double_sided(mat_data);
  }
}

bool InstancerData::is_instance_visible(Object *object)
{
  eEvaluationMode deg_mode = DEG_get_mode(scene_delegate_->depsgraph);
  int vis = BKE_object_visibility(object, deg_mode);
  bool ret = vis & OB_VISIBLE_SELF;
  if (deg_mode == DAG_EVAL_VIEWPORT) {
    if (!ret && ((object->transflag & OB_DUPLI) == 0 ||
                 (object->transflag & OB_DUPLI &&
                  object->duplicator_visibility_flag & OB_DUPLI_FLAG_VIEWPORT)))
    {
      ret = true;
    }
  }
  return ret;
}

pxr::SdfPath InstancerData::object_prim_id(Object *object) const
{
  /* Making id of object in form like <prefix>_<pointer in 16 hex digits format> */
  char str[32];
  snprintf(str, 32, "O_%016llx", (uint64_t)object);
  return prim_id.AppendElementString(str);
}

pxr::SdfPath InstancerData::light_prim_id(LightInstance const &inst, int index) const
{
  char name[16];
  snprintf(name, 16, "L_%08x", index);
  return inst.data->prim_id.AppendElementString(name);
}

int InstancerData::light_prim_id_index(pxr::SdfPath const &id) const
{
  int index;
  sscanf(id.GetName().c_str(), "L_%x", &index);
  return index;
}

void InstancerData::write_instances()
{
  mesh_transforms_.clear();
  for (auto &it : mesh_instances_) {
    it.second.indices.clear();
  }
  for (auto &it : light_instances_) {
    it.second.transforms.clear();
  }

  ListBase *lb = object_duplilist(
      scene_delegate_->depsgraph, scene_delegate_->scene, (Object *)id);
  LISTBASE_FOREACH (DupliObject *, dupli, lb) {
    Object *ob = dupli->ob;
    if (!is_supported(ob) || !is_instance_visible(ob)) {
      continue;
    }

    pxr::SdfPath p_id = object_prim_id(ob);
    if (ob->type == OB_LAMP) {
      LightInstance *inst = light_instance(p_id);
      if (!inst) {
        inst = &light_instances_[p_id];
        inst->data = std::make_unique<LightData>(scene_delegate_, ob, p_id);
        inst->data->init();
      }
      ID_LOG(2, "Light %s %d", inst->data->id->name, inst->transforms.size());
      inst->transforms.push_back(gf_matrix_from_transform(dupli->mat));
    }
    else {
      MeshInstance *inst = mesh_instance(p_id);
      if (!inst) {
        inst = &mesh_instances_[p_id];
        inst->data = std::make_unique<MeshData>(scene_delegate_, ob, p_id);
        inst->data->init();
        inst->data->insert();
      }
      ID_LOG(2, "Mesh %s %d", inst->data->id->name, mesh_transforms_.size());
      inst->indices.push_back(mesh_transforms_.size());
      mesh_transforms_.push_back(gf_matrix_from_transform(dupli->mat));
    }
  }
  free_object_duplilist(lb);

  /* Remove mesh intances without indices */
  for (auto it = mesh_instances_.begin(); it != mesh_instances_.end(); ++it) {
    if (!it->second.indices.empty()) {
      continue;
    }
    it->second.data->remove();
    mesh_instances_.erase(it);
    it = mesh_instances_.begin();
  }

  /* Update light intances and remove instances without transforms */
  for (auto it = light_instances_.begin(); it != light_instances_.end(); ++it) {
    update_light_instance(it->second);
    if (it->second.transforms.empty()) {
      light_instances_.erase(it);
      it = light_instances_.begin();
    }
  }
}

void InstancerData::update_light_instance(LightInstance &inst)
{
  auto &render_index = scene_delegate_->GetRenderIndex();
  LightData &l_data = *inst.data;

  int i;
  pxr::SdfPath p;

  /* Remove old light instances */
  while (inst.count > inst.transforms.size()) {
    --inst.count;
    p = light_prim_id(inst, inst.count);
    render_index.RemoveSprim(l_data.prim_type_, p);
    ID_LOG(2, "Remove %s", p.GetText());
  }

  /* Update current light instances */
  if (inst.data->prim_type((Light *)((Object *)l_data.id)->data) != l_data.prim_type_) {
    /* Recreate instances when prim_type was changed */
    for (i = 0; i < inst.count; ++i) {
      p = light_prim_id(inst, i);
      render_index.RemoveSprim(l_data.prim_type_, p);
      ID_LOG(2, "Remove %s", p.GetText());
    }
    inst.data->init();
    for (i = 0; i < inst.count; ++i) {
      p = light_prim_id(inst, i);
      render_index.InsertSprim(l_data.prim_type_, scene_delegate_, p);
      ID_LOG(2, "Insert %s (%s)", p.GetText(), l_data.id->name);
    }
  }
  else {
    /* Update light instances*/
    pxr::HdDirtyBits bits = pxr::HdLight::DirtyTransform;
    Object *obj = (Object *)inst.data->id;
    if (obj->id.recalc & ID_RECALC_GEOMETRY || ((ID *)obj->data)->recalc & ID_RECALC_GEOMETRY) {
      l_data.init();
      bits = pxr::HdLight::AllDirty;
    }
    for (i = 0; i < inst.count; ++i) {
      p = light_prim_id(inst, i);
      render_index.GetChangeTracker().MarkSprimDirty(p, bits);
      ID_LOG(2, "Update %s (%s)", p.GetText(), l_data.id->name);
    }
  }

  /* Add new light instances */
  while (inst.count < inst.transforms.size()) {
    p = light_prim_id(inst, inst.count);
    render_index.InsertSprim(l_data.prim_type_, scene_delegate_, p);
    ID_LOG(2, "Insert %s (%s)", p.GetText(), l_data.id->name);
    ++inst.count;
  }
}

InstancerData::MeshInstance *InstancerData::mesh_instance(pxr::SdfPath const &id) const
{
  auto it = mesh_instances_.find(id.GetPathElementCount() == 4 ? id.GetParentPath() : id);
  if (it == mesh_instances_.end()) {
    return nullptr;
  }
  return const_cast<MeshInstance *>(&it->second);
}

InstancerData::LightInstance *InstancerData::light_instance(pxr::SdfPath const &id) const
{
  auto it = light_instances_.find(id.GetPathElementCount() == 4 ? id.GetParentPath() : id);
  if (it == light_instances_.end()) {
    return nullptr;
  }
  return const_cast<LightInstance *>(&it->second);
}

}  // namespace blender::render::hydra
