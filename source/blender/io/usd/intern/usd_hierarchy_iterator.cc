/* SPDX-FileCopyrightText: 2019 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */
#include "usd.hh"

#include "usd_armature_utils.hh"
#include "usd_blend_shape_utils.hh"
#include "usd_hierarchy_iterator.hh"
#include "usd_skel_convert.hh"
#include "usd_skel_root_utils.hh"
#include "usd_writer_abstract.hh"
#include "usd_writer_armature.hh"
#include "usd_writer_camera.hh"
#include "usd_writer_curves.hh"
#include "usd_writer_hair.hh"
#include "usd_writer_light.hh"
#include "usd_writer_mesh.hh"
#include "usd_writer_metaball.hh"
#include "usd_writer_transform.hh"
#include "usd_writer_volume.hh"

#include <string>

#include <pxr/base/tf/stringUtils.h>

#include "BKE_idtype.hh"
#include "BKE_main.hh"
#include "BKE_material.h"

#include "BLI_assert.h"
#include "BLI_string.h"
#include "BLI_string_utf8.h"

#include "DEG_depsgraph_query.hh"

#include "DNA_layer_types.h"
#include "DNA_object_types.h"

static const Material **get_materials_from_data(const Object *object)
{
  Material ***mat_ar_p = BKE_object_material_array_p(const_cast<Object *>(object));
  return mat_ar_p ? const_cast<const Material **>(*mat_ar_p) : nullptr;
}

static short get_materials_count_from_data(const Object *object)
{
  short *len_p = BKE_object_material_len_p(const_cast<Object *>(object));
  return len_p ? *len_p : 0;
}

static bool has_unicode_chars(const char *name)
{
  size_t length_in_bytes = 0;
  const size_t length_in_characters = BLI_strlen_utf8_ex(name, &length_in_bytes);
  return length_in_bytes != length_in_characters;
}

namespace blender::io::usd {

USDHierarchyIterator::USDHierarchyIterator(Main *bmain,
                                           Depsgraph *depsgraph,
                                           pxr::UsdStageRefPtr stage,
                                           const USDExportParams &params)
    : AbstractHierarchyIterator(bmain, depsgraph), stage_(stage), params_(params)
{
  DEGObjectIterSettings deg_iter_settings{};
  deg_iter_settings.depsgraph = depsgraph_;
  deg_iter_settings.flags = DEG_ITER_OBJECT_FLAG_LINKED_DIRECTLY |
                            DEG_ITER_OBJECT_FLAG_LINKED_VIA_SET;

  /* First, cache existing names to help avoid name collisions
   * when computing USD prim names. */
  DEG_OBJECT_ITER_BEGIN (&deg_iter_settings, object) {
    cache_names_for_object(object);
  }
  DEG_OBJECT_ITER_END;

  DEG_OBJECT_ITER_BEGIN (&deg_iter_settings, object) {
    process_names_for_object(object);
  }
  DEG_OBJECT_ITER_END;
}

bool USDHierarchyIterator::id_needs_display_name(const ID *id) const
{
  return has_unicode_chars(id->name + 2);
}

bool USDHierarchyIterator::object_needs_display_name(const Object *object) const
{
  return id_needs_display_name(reinterpret_cast<const ID *>(object));
}

bool USDHierarchyIterator::object_data_needs_display_name(const Object *object) const
{
  if (!object->data) {
    return false;
  }

  return id_needs_display_name(reinterpret_cast<const ID *>(object->data));
}

std::string USDHierarchyIterator::generate_unique_name(const std::string &token)
{
  char name[64];
  int count = 0;
  BLI_snprintf(name, 64, "%s", token.c_str());

  while (prim_names_.contains(name)) {
    count += 1;
    BLI_snprintf(name, 64, "%s_%03d", token.c_str(), count);
  }

  return std::string(name);
}

void USDHierarchyIterator::store_name(const ID *id, const std::string &name)
{
  prim_names_map_.add({id->name}, name);
  prim_names_.add({name});
}

std::string USDHierarchyIterator::find_name(const ID *id) const
{
  return prim_names_map_.lookup_default({id->name}, std::string());
}

void USDHierarchyIterator::cache_names_for_object(const Object *object)
{
  const char *name = object->id.name + 2;
  if (!has_unicode_chars(name)) {
    prim_names_.add(make_valid_name(name));
  }
  cache_material_names(const_cast<const Material **>(object->mat), object->totcol);

  if (object->data) {
    ID *id = static_cast<ID *>(object->data);
    name = id->name + 2;
    if (!has_unicode_chars(name)) {
      prim_names_.add(make_valid_name(name));
    }

    const Material **data_mats = get_materials_from_data(object);
    size_t data_count = get_materials_count_from_data(object);
    cache_material_names(data_mats, data_count);
  }
}

void USDHierarchyIterator::process_names_for_object(const Object *object)
{
  const short id_code = (object->data ? GS(reinterpret_cast<const ID *>(object->data)->name) :
                                        GS(object->id.name));
  const std::string token(BKE_idtype_idcode_to_name(id_code));

  if (object_needs_display_name(object)) {
    const std::string obj_name = generate_unique_name(token);
    store_name(&object->id, obj_name);

    if (object->totcol) {
      process_materials(const_cast<const Material **>(object->mat), object->totcol);
    }
  }

  if (object->data) {
    if (object_data_needs_display_name(object)) {
      ID *object_data = static_cast<ID *>(object->data);
      const std::string data_name = generate_unique_name(token + "_Data");
      store_name(object_data, data_name);
    }

    const Material **data_mats = get_materials_from_data(object);
    size_t data_count = get_materials_count_from_data(object);
    process_materials(data_mats, data_count);
  }
}

bool USDHierarchyIterator::mark_as_weak_export(const Object *object) const
{
  if (params_.selected_objects_only && (object->base_flag & BASE_SELECTED) == 0) {
    return true;
  }
  return false;
}

void USDHierarchyIterator::release_writer(AbstractHierarchyWriter *writer)
{
  delete static_cast<USDAbstractWriter *>(writer);
}

std::string USDHierarchyIterator::make_valid_name(const std::string &name) const
{
  return pxr::TfMakeValidIdentifier(name);
}

void USDHierarchyIterator::process_usd_skel() const
{
  skel_export_chaser(stage_,
                     armature_export_map_,
                     skinned_mesh_export_map_,
                     shape_key_mesh_export_map_,
                     depsgraph_);

  create_skel_roots(stage_, params_);
}

void USDHierarchyIterator::set_export_frame(float frame_nr)
{
  /* The USD stage is already set up to have FPS time-codes per frame. */
  export_time_ = pxr::UsdTimeCode(frame_nr);
}

USDExporterContext USDHierarchyIterator::create_usd_export_context(const HierarchyContext *context)
{
  pxr::SdfPath path;
  if (params_.root_prim_path[0] != '\0') {
    path = pxr::SdfPath(params_.root_prim_path + context->export_path);
  }
  else {
    path = pxr::SdfPath(context->export_path);
  }

  /* Returns the same path that was passed to `stage_` object during it's creation (via
   * `pxr::UsdStage::CreateNew` function). */
  const pxr::SdfLayerHandle root_layer = stage_->GetRootLayer();
  const std::string export_file_path = root_layer->GetRealPath();
  auto get_time_code = [this]() { return this->export_time_; };

  return USDExporterContext{
      bmain_, depsgraph_, stage_, path, get_time_code, params_, export_file_path};
}

AbstractHierarchyWriter *USDHierarchyIterator::create_transform_writer(
    const HierarchyContext *context)
{
  USDTransformWriter* ret = new USDTransformWriter(create_usd_export_context(context));
  ret->set_iterator(this);
  return ret;
}

AbstractHierarchyWriter *USDHierarchyIterator::create_data_writer(const HierarchyContext *context)
{
  USDExporterContext usd_export_context = create_usd_export_context(context);
  USDAbstractWriter *data_writer = nullptr;

  switch (context->object->type) {
    case OB_MESH:
      data_writer = new USDMeshWriter(usd_export_context);
      break;
    case OB_CAMERA:
      data_writer = new USDCameraWriter(usd_export_context);
      break;
    case OB_LAMP:
      data_writer = new USDLightWriter(usd_export_context);
      break;
    case OB_MBALL:
      data_writer = new USDMetaballWriter(usd_export_context);
      break;
    case OB_CURVES_LEGACY:
    case OB_CURVES:
      data_writer = new USDCurvesWriter(usd_export_context);
      break;
    case OB_VOLUME:
      data_writer = new USDVolumeWriter(usd_export_context);
      break;
    case OB_ARMATURE:
      if (usd_export_context.export_params.export_armatures) {
        data_writer = new USDArmatureWriter(usd_export_context);
      }
      else {
        return nullptr;
      }
      break;
    case OB_EMPTY:
    case OB_SURF:
    case OB_FONT:
    case OB_SPEAKER:
    case OB_LIGHTPROBE:
    case OB_LATTICE:
    case OB_GPENCIL_LEGACY:
    case OB_GREASE_PENCIL:
    case OB_POINTCLOUD:
      return nullptr;
    case OB_TYPE_MAX:
      BLI_assert_msg(0, "OB_TYPE_MAX should not be used");
      return nullptr;
    default:
      BLI_assert_unreachable();
      return nullptr;
  }

  if (!data_writer->is_supported(context)) {
    delete data_writer;
    return nullptr;
  }

  if (data_writer && (params_.export_armatures || params_.export_shapekeys)) {
    add_usd_skel_export_mapping(context->object, data_writer->usd_path());
  }

	data_writer->set_iterator(this);
	return data_writer;
}

AbstractHierarchyWriter *USDHierarchyIterator::create_hair_writer(const HierarchyContext *context)
{
  if (!params_.export_hair) {
    return nullptr;
  }

	USDHairWriter *ret = new USDHairWriter(create_usd_export_context(context));
	ret->set_iterator(this);
	return ret;
}

AbstractHierarchyWriter *USDHierarchyIterator::create_particle_writer(
    const HierarchyContext * /*context*/)
{
  return nullptr;
}

void USDHierarchyIterator::add_usd_skel_export_mapping(const Object *obj, const pxr::SdfPath &path)
{
  if (params_.export_shapekeys && is_mesh_with_shape_keys(obj)) {
    shape_key_mesh_export_map_.add(obj, path);
  }

  if (params_.export_armatures && obj->type == OB_ARMATURE) {
    armature_export_map_.add(obj, path);
  }

  if (params_.export_armatures && obj->type == OB_MESH &&
      can_export_skinned_mesh(*obj, depsgraph_))
  {
    skinned_mesh_export_map_.add(obj, path);
  }
}

void USDHierarchyIterator::cache_material_names(const Material **materials, size_t count)
{
  for (int m = 0; m < count; m++) {
    const Material *mat = materials[m];
    if (!mat) {
      continue;
    }
    const char *name = mat->id.name + 2;
    if (!has_unicode_chars(name)) {
      prim_names_.add(make_valid_name(name));
    }
  }
}

void USDHierarchyIterator::process_materials(const Material **materials, const size_t count)
{
  for (int m = 0; m < count; m++) {
    const Material *mat = materials[m];
    if (!mat) {
      continue;
    }

    if (id_needs_display_name(&mat->id)) {
      const std::string data_name = generate_unique_name("Material");
      store_name(&mat->id, data_name);
    }
  }
}

std::string USDHierarchyIterator::get_object_computed_name(const Object *object) const
{
  if (!object) {
    return "";
  }
  return find_name(&object->id);
}

std::string USDHierarchyIterator::get_object_data_computed_name(const Object *object) const
{
  if (!(object && object->data)) {
    return "";
  }

  ID *object_data = static_cast<ID *>(object->data);
  return find_name(object_data);
}

}  // namespace blender::io::usd
