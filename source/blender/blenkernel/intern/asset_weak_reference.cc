/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include <memory>
#include <utility>

#include "BLI_string.h"

#include "DNA_space_types.h"

#include "AS_asset_identifier.hh"
#include "AS_asset_library.hh"

#include "BKE_asset.hh"
#include "BKE_blendfile_link_append.hh"
#include "BKE_idtype.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"

#include "BLI_vector.hh"

#include "BLO_read_write.hh"
#include "BLO_readfile.h"

#include "DNA_asset_types.h"

#include "MEM_guardedalloc.h"

using namespace blender;

/* #AssetWeakReference -------------------------------------------- */

AssetWeakReference::AssetWeakReference()
    : asset_library_type(0), asset_library_identifier(nullptr), relative_asset_identifier(nullptr)
{
}

AssetWeakReference::AssetWeakReference(AssetWeakReference &&other)
    : asset_library_type(other.asset_library_type),
      asset_library_identifier(other.asset_library_identifier),
      relative_asset_identifier(other.relative_asset_identifier)
{
  other.asset_library_type = 0; /* Not a valid type. */
  other.asset_library_identifier = nullptr;
  other.relative_asset_identifier = nullptr;
}

AssetWeakReference::~AssetWeakReference()
{
  MEM_delete(asset_library_identifier);
  MEM_delete(relative_asset_identifier);
}

void BKE_asset_weak_reference_free(AssetWeakReference **weak_ref)
{
  MEM_delete(*weak_ref);
  *weak_ref = nullptr;
}

AssetWeakReference *BKE_asset_weak_reference_copy(AssetWeakReference *weak_ref)
{
  if (weak_ref == nullptr) {
    return nullptr;
  }

  AssetWeakReference *weak_ref_copy = MEM_new<AssetWeakReference>(__func__);
  weak_ref_copy->asset_library_type = weak_ref->asset_library_type;
  weak_ref_copy->asset_library_identifier = BLI_strdup(weak_ref->asset_library_identifier);
  weak_ref_copy->relative_asset_identifier = BLI_strdup(weak_ref->relative_asset_identifier);

  return weak_ref_copy;
}

AssetWeakReference *AssetWeakReference::make_reference(
    const asset_system::AssetLibrary &library,
    const asset_system::AssetIdentifier &asset_identifier)
{
  AssetWeakReference *weak_ref = MEM_new<AssetWeakReference>(__func__);

  weak_ref->asset_library_type = library.library_type();
  StringRefNull name = library.name();
  if (!name.is_empty()) {
    weak_ref->asset_library_identifier = BLI_strdupn(name.c_str(), name.size());
  }

  StringRefNull relative_identifier = asset_identifier.library_relative_identifier();
  weak_ref->relative_asset_identifier = BLI_strdupn(relative_identifier.c_str(),
                                                    relative_identifier.size());

  return weak_ref;
}

void BKE_asset_weak_reference_write(BlendWriter *writer, const AssetWeakReference *weak_ref)
{
  BLO_write_struct(writer, AssetWeakReference, weak_ref);
  BLO_write_string(writer, weak_ref->asset_library_identifier);
  BLO_write_string(writer, weak_ref->relative_asset_identifier);
}

void BKE_asset_weak_reference_read(BlendDataReader *reader, AssetWeakReference *weak_ref)
{
  BLO_read_data_address(reader, &weak_ref->asset_library_identifier);
  BLO_read_data_address(reader, &weak_ref->relative_asset_identifier);
}

/* Main database for each brush asset blend file.
 *
 * This avoids mixing asset datablocks in the regular main, which leads to naming conflicts and
 * confusing user interface.
 *
 * TODO: Heavily WIP code. */

struct AssetWeakReferenceMain {
  /* TODO: not sure if this is the best unique identifier. */
  std::string filepath;
  Main *main;

  AssetWeakReferenceMain(const char *filepath) : filepath(filepath), main(BKE_main_new()) {}
  AssetWeakReferenceMain(const AssetWeakReferenceMain &) = delete;
  AssetWeakReferenceMain(AssetWeakReferenceMain &&other)
      : filepath(std::exchange(other.filepath, "")), main(std::exchange(other.main, nullptr))
  {
  }

  ~AssetWeakReferenceMain()
  {
    if (main) {
      BKE_main_free(main);
    }
  }
};

blender::Vector<AssetWeakReferenceMain> ASSET_WEAK_REFERENCE_MAINS;

Main *BKE_asset_weak_reference_main(Main *global_main, const ID *id)
{
  if (!(id->tag & LIB_TAG_ASSET_MAIN)) {
    return global_main;
  }

  for (const AssetWeakReferenceMain &weak_ref_main : ASSET_WEAK_REFERENCE_MAINS) {
    /* TODO: just loop over listbase of same type, or make this whole thing
     * more efficient. */
    ID *other_id;
    FOREACH_MAIN_ID_BEGIN (weak_ref_main.main, other_id) {
      if (id == other_id) {
        return weak_ref_main.main;
      }
    }
    FOREACH_MAIN_ID_END;
  }

  BLI_assert_unreachable();
  return nullptr;
}

static Main *asset_weak_reference_main_ensure(const char *filepath)
{
  for (const AssetWeakReferenceMain &weak_ref_main : ASSET_WEAK_REFERENCE_MAINS) {
    if (weak_ref_main.filepath == filepath) {
      return weak_ref_main.main;
    }
  }

  ASSET_WEAK_REFERENCE_MAINS.append(filepath);
  return ASSET_WEAK_REFERENCE_MAINS.last().main;
}

void BKE_asset_weak_reference_main_free()
{
  ASSET_WEAK_REFERENCE_MAINS.clear_and_shrink();
}

ID *BKE_asset_weak_reference_ensure(Main *global_main, const AssetWeakReference *weak_ref)
{
  BLI_assert(weak_ref != nullptr);

  char asset_full_path_buffer[FILE_MAX_LIBEXTRA];
  char *asset_lib_path, *asset_group, *asset_name;

  AS_asset_full_path_explode_from_weak_ref(
      weak_ref, asset_full_path_buffer, &asset_lib_path, &asset_group, &asset_name);

  if (asset_lib_path == nullptr && asset_group == nullptr && asset_name == nullptr) {
    return nullptr;
  }

  // TODO: make not brush specific
  BLI_assert(STREQ(asset_group, IDType_ID_BR.name));
  BLI_assert(asset_name != nullptr);

  /* If weak reference resolves to a null library path, assume we are in local asset case. */
  Main *asset_main = (asset_lib_path) ? asset_weak_reference_main_ensure(asset_lib_path) :
                                        global_main;

  /* Check if we have the asset already, or if it's global main and there is nothing we can add. */
  ID *local_asset = reinterpret_cast<ID *>(
      BLI_findstring(&asset_main->brushes, asset_name, offsetof(ID, name) + 2));
  if (local_asset || asset_lib_path == nullptr) {
    BLI_assert(local_asset == nullptr || ID_IS_ASSET(local_asset));
    return local_asset;
  }

  /* Load asset from asset library. */
  LibraryLink_Params lapp_parameters{};
  lapp_parameters.bmain = asset_main;
  BlendfileLinkAppendContext *lapp_context = BKE_blendfile_link_append_context_new(
      &lapp_parameters);
  BKE_blendfile_link_append_context_flag_set(lapp_context, BLO_LIBLINK_FORCE_INDIRECT, true);
  BKE_blendfile_link_append_context_flag_set(lapp_context, 0, true);

  BKE_blendfile_link_append_context_library_add(lapp_context, asset_lib_path, nullptr);

  // TODO: make not brush specific
  BlendfileLinkAppendContextItem *lapp_item = BKE_blendfile_link_append_context_item_add(
      lapp_context, asset_name, ID_BR, nullptr);
  BKE_blendfile_link_append_context_item_library_index_enable(lapp_context, lapp_item, 0);

  BKE_blendfile_link(lapp_context, nullptr);
  BKE_blendfile_append(lapp_context, nullptr);

  local_asset = BKE_blendfile_link_append_context_item_newid_get(lapp_context, lapp_item);

  BKE_blendfile_link_append_context_free(lapp_context);

  /* TODO: only do for new ones? */
  BKE_main_id_tag_all(asset_main, LIB_TAG_ASSET_MAIN, true);

  /* Verify that the name matches. It must for referencing the same asset again to work.  */
  BLI_assert(local_asset == nullptr || STREQ(local_asset->name + 2, asset_name));

  return local_asset;
}

