/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_animsys.h"
#include "BKE_asset.hh"
#include "BKE_asset_edit.hh"
#include "BKE_context.hh"
#include "BKE_lib_id.hh"
#include "BKE_preferences.h"
#include "BKE_report.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_prototypes.hh"

#include "ED_asset_library.hh"
#include "ED_asset_list.hh"
#include "ED_asset_mark_clear.hh"
#include "ED_asset_menu_utils.hh"
#include "ED_asset_shelf.hh"
#include "ED_screen.hh"

#include "UI_resources.hh"

#include "BLT_translation.hh"

#include "ANIM_action.hh"
#include "ANIM_action_iterators.hh"
#include "ANIM_keyframing.hh"
#include "ANIM_rna.hh"

#include "AS_asset_catalog.hh"
#include "AS_asset_catalog_tree.hh"
#include "AS_asset_library.hh"
#include "AS_asset_representation.hh"

#include "anim_intern.hh"

namespace blender::ed::animrig {

static asset_system::AssetCatalog &asset_library_ensure_catalog(
    asset_system::AssetLibrary &library, const asset_system::AssetCatalogPath &path)
{
  if (asset_system::AssetCatalog *catalog = library.catalog_service().find_catalog_by_path(path)) {
    return *catalog;
  }
  return *library.catalog_service().create_catalog(path);
}

static asset_system::AssetCatalog &asset_library_ensure_catalogs_in_path(
    asset_system::AssetLibrary &library, const asset_system::AssetCatalogPath &path)
{
  /* Adding multiple catalogs in a path at a time with #AssetCatalogService::create_catalog()
   * doesn't work; add each potentially new catalog in the hierarchy manually here. */
  asset_system::AssetCatalogPath parent = "";
  path.iterate_components([&](StringRef component_name, bool /*is_last_component*/) {
    asset_library_ensure_catalog(library, parent / component_name);
    parent = parent / component_name;
  });
  return *library.catalog_service().find_catalog_by_path(path);
}

static AssetLibraryReference user_library_to_library_ref(const bUserAssetLibrary &user_library)
{
  AssetLibraryReference library_ref{};
  library_ref.custom_library_index = BLI_findindex(&U.asset_libraries, &user_library);
  library_ref.type = ASSET_LIBRARY_CUSTOM;
  return library_ref;
}

static const bUserAssetLibrary *library_ref_to_user_library(
    const AssetLibraryReference &library_ref)
{
  if (library_ref.type != ASSET_LIBRARY_CUSTOM) {
    return nullptr;
  }
  return static_cast<const bUserAssetLibrary *>(
      BLI_findlink(&U.asset_libraries, library_ref.custom_library_index));
}

static void visit_library_catalogs_catalog_for_search(
    const Main &bmain,
    const bUserAssetLibrary &user_library,
    const StringRef edit_text,
    const FunctionRef<void(StringPropertySearchVisitParams)> visit_fn)
{
  const asset_system::AssetLibrary *library = AS_asset_library_load(
      &bmain, user_library_to_library_ref(user_library));
  if (!library) {
    return;
  }

  if (!edit_text.is_empty()) {
    const asset_system::AssetCatalogPath edit_path = edit_text;
    if (!library->catalog_service().find_catalog_by_path(edit_path)) {
      visit_fn(StringPropertySearchVisitParams{edit_path.str(), std::nullopt, ICON_ADD});
    }
  }

  const asset_system::AssetCatalogTree &full_tree = library->catalog_service().catalog_tree();
  full_tree.foreach_item([&](const asset_system::AssetCatalogTreeItem &item) {
    visit_fn(StringPropertySearchVisitParams{item.catalog_path().str(), std::nullopt});
  });
}

static const EnumPropertyItem *rna_asset_library_reference_itemf(bContext * /*C*/,
                                                                 PointerRNA * /*ptr*/,
                                                                 PropertyRNA * /*prop*/,
                                                                 bool *r_free)
{
  const EnumPropertyItem *items = asset::library_reference_to_rna_enum_itemf(false);
  if (!items) {
    *r_free = false;
    return nullptr;
  }

  *r_free = true;
  return items;
}

static const bUserAssetLibrary *get_asset_library_from_prop(PointerRNA &ptr)
{
  const int enum_value = RNA_enum_get(&ptr, "asset_library_reference");
  const AssetLibraryReference lib_ref = asset::library_reference_from_enum_value(enum_value);
  return BKE_preferences_asset_library_find_index(&U, lib_ref.custom_library_index);
}

static void visit_library_prop_catalogs_catalog_for_search_fn(
    const bContext *C,
    PointerRNA *ptr,
    PropertyRNA * /*prop*/,
    const char *edit_text,
    FunctionRef<void(StringPropertySearchVisitParams)> visit_fn)
{
  /* NOTE: Using the all library would also be a valid choice. */
  if (const bUserAssetLibrary *user_library = get_asset_library_from_prop(*ptr)) {
    visit_library_catalogs_catalog_for_search(
        *CTX_data_main(C), *user_library, edit_text, visit_fn);
  }
}

static void refresh_asset_library(const bContext *C, const AssetLibraryReference &library_ref)
{
  asset::list::clear(&library_ref, C);
  /* TODO: Should the all library reference be automatically cleared? */
  AssetLibraryReference all_lib_ref = asset_system::all_library_reference();
  asset::list::clear(&all_lib_ref, C);
}

static void show_catalog_in_asset_shelf(const bContext &C, const StringRefNull catalog_path)
{
  /* Enable catalog in all visible asset shelves. */
  wmWindowManager *wm = CTX_wm_manager(&C);
  LISTBASE_FOREACH (wmWindow *, win, &wm->windows) {
    const bScreen *screen = WM_window_get_active_screen(win);
    LISTBASE_FOREACH (ScrArea *, area, &screen->areabase) {
      const AssetShelf *shelf = asset::shelf::active_shelf_from_area(area);
      if (shelf && BKE_preferences_asset_shelf_settings_ensure_catalog_path_enabled(
                       &U, shelf->idname, catalog_path.c_str()))
      {
        U.runtime.is_dirty = true;
      }
    }
  }
}

static blender::animrig::Action &extract_pose(Main &bmain, Object &pose_object)
{
  /* This currently only looks at the pose and not other things that could go onto different
   * slots on the same action. */

  using namespace blender::animrig;
  Action &action = action_add(bmain, "pose_create");
  Slot &slot = action.slot_add_for_id(pose_object.id);
  Layer &layer = action.layer_add("pose");
  Strip &strip = layer.strip_add(action, Strip::Type::Keyframe);
  StripKeyframeData &strip_data = strip.data<StripKeyframeData>(action);

  KeyframeSettings key_settings = {BEZT_KEYTYPE_KEYFRAME, HD_AUTO, BEZT_IPO_BEZ};

  LISTBASE_FOREACH (bPoseChannel *, pose_bone, &pose_object.pose->chanbase) {
    if (!(pose_bone->bone->flag & BONE_SELECTED)) {
      continue;
    }
    PointerRNA bone_pointer = RNA_pointer_create(&pose_object.id, &RNA_PoseBone, pose_bone);
    Vector<RNAPath> rna_paths = construct_rna_paths(&bone_pointer);
    for (const RNAPath &rna_path : rna_paths) {
      PointerRNA resolved_pointer;
      PropertyRNA *resolved_property;
      if (!RNA_path_resolve(
              &bone_pointer, rna_path.path.c_str(), &resolved_pointer, &resolved_property))
      {
        continue;
      }
      Vector<float> values = blender::animrig::get_rna_values(&resolved_pointer,
                                                              resolved_property);
      const std::optional<std::string> rna_path_id_to_prop = RNA_path_from_ID_to_property(
          &resolved_pointer, resolved_property);
      if (!rna_path_id_to_prop.has_value()) {
        continue;
      }
      int i = 0;
      for (const float value : values) {
        strip_data.keyframe_insert(
            &bmain, slot, {rna_path_id_to_prop.value(), i}, {1, value}, key_settings);
        i++;
      }
    }
  }
  return action;
}

static int pose_asset_create_exec(bContext *C, wmOperator *op)
{
  char name[MAX_NAME] = "";
  PropertyRNA *name_prop = RNA_struct_find_property(op->ptr, "name");
  if (RNA_property_is_set(op->ptr, name_prop)) {
    RNA_property_string_get(op->ptr, name_prop, name);
  }
  if (name[0] == '\0') {
    BKE_report(op->reports, RPT_ERROR, "No name set");
    return OPERATOR_CANCELLED;
  }

  const bUserAssetLibrary *user_library = get_asset_library_from_prop(*op->ptr);
  if (!user_library) {
    return OPERATOR_CANCELLED;
  }

  Main *bmain = CTX_data_main(C);
  asset_system::AssetLibrary *library = AS_asset_library_load(
      bmain, user_library_to_library_ref(*user_library));
  if (!library) {
    BKE_report(op->reports, RPT_ERROR, "Failed to load asset library");
    return OPERATOR_CANCELLED;
  }

  Object *pose_object = CTX_data_active_object(C);
  if (!pose_object || !pose_object->pose) {
    return OPERATOR_CANCELLED;
  }

  /* Temporary action in current main that will be exported and later deleted. */
  blender::animrig::Action &pose_action = extract_pose(*bmain, *pose_object);
  asset::mark_id(&pose_action.id);
  asset::generate_preview(C, &pose_action.id);

  /* Add asset to catalog. */
  char catalog_path[MAX_NAME];
  RNA_string_get(op->ptr, "catalog_path", catalog_path);

  AssetMetaData &meta_data = *pose_action.id.asset_data;
  if (catalog_path[0]) {
    const asset_system::AssetCatalog &catalog = asset_library_ensure_catalogs_in_path(
        *library, catalog_path);
    BKE_asset_metadata_catalog_id_set(&meta_data, catalog.catalog_id, catalog.simple_name.c_str());
  }

  AssetWeakReference pose_asset_reference;
  const std::optional<std::string> final_full_asset_filepath = bke::asset_edit_id_save_as(
      *bmain, pose_action.id, name, *user_library, pose_asset_reference, *op->reports);

  library->catalog_service().write_to_disk(*final_full_asset_filepath);
  show_catalog_in_asset_shelf(*C, catalog_path);

  BKE_id_free(bmain, &pose_action.id);

  // refresh_asset_library(C, user_library_to_library_ref(*user_library));

  WM_main_add_notifier(NC_ASSET | ND_ASSET_LIST | NA_ADDED, nullptr);

  return OPERATOR_FINISHED;
}

static int pose_asset_create_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  /* If the library isn't saved from the operator's last execution, use the first library. */
  if (!RNA_struct_property_is_set_ex(op->ptr, "asset_library_reference", false)) {
    const AssetLibraryReference first_library = user_library_to_library_ref(
        *static_cast<const bUserAssetLibrary *>(U.asset_libraries.first));
    RNA_enum_set(op->ptr,
                 "asset_library_reference",
                 asset::library_reference_to_enum_value(&first_library));
  }

  return WM_operator_props_dialog_popup(C, op, 400, std::nullopt, IFACE_("Create"));
}

static bool pose_asset_create_poll(bContext *C)
{
  if (!ED_operator_posemode_context(C)) {
    return false;
  }
  if (BLI_listbase_is_empty(&U.asset_libraries)) {
    CTX_wm_operator_poll_msg_set(C, "No asset library available to save to");
    return false;
  }
  return true;
}

void POSELIB_OT_asset_create(wmOperatorType *ot)
{
  ot->name = "Create Pose Asset";
  ot->description = "Create a new asset from the selection in the scene";
  ot->idname = "POSELIB_OT_asset_create";

  ot->exec = pose_asset_create_exec;
  ot->invoke = pose_asset_create_invoke;
  ot->poll = pose_asset_create_poll;

  ot->prop = RNA_def_string(
      ot->srna, "name", nullptr, MAX_NAME, "Name", "Name for the new pose asset");

  PropertyRNA *prop = RNA_def_property(ot->srna, "asset_library_reference", PROP_ENUM, PROP_NONE);
  RNA_def_enum_funcs(prop, rna_asset_library_reference_itemf);
  RNA_def_property_ui_text(prop, "Library", "Asset library used to store the new pose");

  prop = RNA_def_string(
      ot->srna, "catalog_path", nullptr, MAX_NAME, "Catalog", "Catalog to use for the new asset");
  RNA_def_property_string_search_func_runtime(
      prop, visit_library_prop_catalogs_catalog_for_search_fn, PROP_STRING_SEARCH_SUGGESTION);
}

static bAction *action_from_selected_asset(bContext *C)
{
  const AssetRepresentationHandle *asset_handle = CTX_wm_asset(C);
  if (!asset_handle) {
    return nullptr;
  }

  if (asset_handle->get_id_type() != ID_AC) {
    return nullptr;
  }

  AssetWeakReference asset_reference = asset_handle->make_weak_reference();
  Main *bmain = CTX_data_main(C);
  return reinterpret_cast<bAction *>(
      bke::asset_edit_id_from_weak_reference(*bmain, ID_AC, asset_reference));
}

static void update_pose_action_from_scene(Main *bmain,
                                          blender::animrig::Action &action,
                                          Object &pose_object)
{
  if (action.slot_array_num < 1) {
    /* All actions should have slots at this point. */
    BLI_assert_unreachable();
    return;
  }

  Set<RNAPath> existing_paths;
  blender::animrig::foreach_fcurve_in_action_slot(
      action, action.slot_array[0]->handle, [&](FCurve &fcurve) {
        existing_paths.add({fcurve.rna_path, std::nullopt, fcurve.array_index});
      });

  blender::animrig::KeyframeSettings key_settings = {BEZT_KEYTYPE_KEYFRAME, HD_AUTO, BEZT_IPO_BEZ};
  BLI_assert(action.strip_keyframe_data_array_num == 1);
  BLI_assert(action.slot_array_num == 1);
  blender::animrig::StripKeyframeData *strip_data = action.strip_keyframe_data()[0];
  blender::animrig::Slot *slot = action.slot(0);

  LISTBASE_FOREACH (bPoseChannel *, pose_bone, &pose_object.pose->chanbase) {
    if (!(pose_bone->bone->flag & BONE_SELECTED)) {
      continue;
    }
    PointerRNA bone_pointer = RNA_pointer_create(&pose_object.id, &RNA_PoseBone, pose_bone);
    PointerRNA resolved_pointer;
    PropertyRNA *resolved_property;
    if (!RNA_path_resolve(&bone_pointer, "location", &resolved_pointer, &resolved_property)) {
      continue;
    }
    const std::optional<std::string> rna_path_id_to_prop = RNA_path_from_ID_to_property(
        &resolved_pointer, resolved_property);
    if (!rna_path_id_to_prop.has_value()) {
      continue;
    }
    Vector<float> values = blender::animrig::get_rna_values(&resolved_pointer, resolved_property);
    int i = 0;
    for (const float value : values) {
      RNAPath path = {rna_path_id_to_prop.value(), std::nullopt, i};
      /* Only updating existing channels. */
      if (existing_paths.contains(path)) {
        strip_data->keyframe_insert(
            bmain, *slot, {rna_path_id_to_prop.value(), i}, {1, value}, key_settings);
      }
      i++;
    }
  }
}

static int pose_asset_overwrite_exec(bContext *C, wmOperator *op)
{
  bAction *action = action_from_selected_asset(C);
  BLI_assert_msg(action, "Poll should have checked action exists");

  Main *bmain = CTX_data_main(C);
  Object *pose_object = CTX_data_active_object(C);
  if (!pose_object || !pose_object->pose) {
    return OPERATOR_CANCELLED;
  }

  update_pose_action_from_scene(bmain, action->wrap(), *pose_object);

  bke::asset_edit_id_save(*bmain, action->id, *op->reports);

  WM_main_add_notifier(NC_ASSET | ND_ASSET_LIST | NA_EDITED, nullptr);

  return OPERATOR_FINISHED;
}

static bool pose_asset_overwrite_poll(bContext *C)
{
  if (!ED_operator_posemode_context(C)) {
    return false;
  }

  bAction *action = action_from_selected_asset(C);

  if (!action) {
    return false;
  }

  if (!bke::asset_edit_id_is_editable(action->id)) {
    return false;
  }

  if (!bke::asset_edit_id_is_writable(action->id)) {
    CTX_wm_operator_poll_msg_set(C, "Asset blend file is not editable");
    return false;
  }

  return true;
}

/* Calling it overwrite instead of save because we aren't actually saving an opened asset. */
void POSELIB_OT_asset_overwrite(wmOperatorType *ot)
{
  ot->name = "Overwrite Pose Asset";
  ot->description =
      "Update the selected pose asset in the asset library from the currently selected bones";
  ot->idname = "POSELIB_OT_asset_overwrite";

  ot->exec = pose_asset_overwrite_exec;
  ot->poll = pose_asset_overwrite_poll;
}

}  // namespace blender::ed::animrig
