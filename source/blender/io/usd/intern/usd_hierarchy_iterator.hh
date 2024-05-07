/* SPDX-FileCopyrightText: 2019 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

#include "BLI_set.hh"

#include "IO_abstract_hierarchy_iterator.h"
#include "usd.hh"
#include "usd_exporter_context.hh"
#include "usd_skel_convert.hh"

#include <string>

#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/timeCode.h>

struct Depsgraph;
struct Main;
struct Object;

namespace blender::io::usd {

using blender::io::AbstractHierarchyIterator;
using blender::io::AbstractHierarchyWriter;
using blender::io::HierarchyContext;

class USDHierarchyIterator : public AbstractHierarchyIterator {
 private:
  const pxr::UsdStageRefPtr stage_;
  pxr::UsdTimeCode export_time_;
  const USDExportParams &params_;

  ObjExportMap armature_export_map_;
  ObjExportMap skinned_mesh_export_map_;
  ObjExportMap shape_key_mesh_export_map_;

  /* Map an ID name to a prim name. */
  Map<const std::string, const std::string> prim_names_map_;
  Set<const std::string> prim_names_;

 public:
  USDHierarchyIterator(Main *bmain,
                       Depsgraph *depsgraph,
                       pxr::UsdStageRefPtr stage,
                       const USDExportParams &params);

  void set_export_frame(float frame_nr);

  virtual std::string make_valid_name(const std::string &name) const override;

  void process_usd_skel() const;

  bool id_needs_display_name(const ID *id) const;
  bool object_needs_display_name(const Object *object) const;
  bool object_data_needs_display_name(const Object *object) const;

  std::string find_name(const ID *id) const;
  std::string get_object_computed_name(const Object *object) const;
  std::string get_object_data_computed_name(const Object *object) const;

 protected:
  virtual bool mark_as_weak_export(const Object *object) const override;

  virtual AbstractHierarchyWriter *create_transform_writer(
      const HierarchyContext *context) override;
  virtual AbstractHierarchyWriter *create_data_writer(const HierarchyContext *context) override;
  virtual AbstractHierarchyWriter *create_hair_writer(const HierarchyContext *context) override;
  virtual AbstractHierarchyWriter *create_particle_writer(
      const HierarchyContext *context) override;

  virtual void release_writer(AbstractHierarchyWriter *writer) override;

 private:
  USDExporterContext create_usd_export_context(const HierarchyContext *context);

  void add_usd_skel_export_mapping(const Object *obj, const pxr::SdfPath &usd_path);

  std::string generate_unique_name(const std::string &token);
  void store_name(const ID *id, const std::string &name);
  void cache_names_for_object(const Object *object);
  void process_names_for_object(const Object *object);

  void cache_material_names(const Material **materials, size_t count);
  void process_materials(const Material **materials, size_t count);
};

}  // namespace blender::io::usd
