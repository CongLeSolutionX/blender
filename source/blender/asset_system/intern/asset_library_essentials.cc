/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup asset_system
 */

#include "BKE_appdir.hh"
#include "BKE_blendfile.hh"
#include "BKE_idtype.hh"

#include "BLI_fileops.h"
#include "BLI_path_utils.hh"
#include "BLI_string.h"

#include "utils.hh"

#include "AS_asset_representation.hh"
#include "AS_essentials_library.hh"
#include "asset_library_essentials.hh"

namespace blender::asset_system {

EssentialsAssetLibrary::EssentialsAssetLibrary()
    : OnDiskAssetLibrary(ASSET_LIBRARY_ESSENTIALS,
                         {},
                         utils::normalize_directory_path(essentials_directory_path()))
{
  import_method_ = ASSET_IMPORT_APPEND_REUSE;
}

StringRefNull essentials_directory_path()
{
  static std::string path = []() {
    const std::optional<std::string> datafiles_path = BKE_appdir_folder_id(
        BLENDER_SYSTEM_DATAFILES, "assets");
    return datafiles_path.value_or("");
  }();
  return path;
}

StringRefNull essentials_override_directory_path()
{
  static std::string path = []() -> std::string {
    const std::optional<std::string> datafiles_path = BKE_appdir_folder_id_user_notest(
        BLENDER_USER_DATAFILES, "assets");
    if (!datafiles_path) {
      BLI_assert_unreachable();
      return "";
    }
    return *datafiles_path + SEP + "essentials_overrides";
  }();
  return path;
}

bool essentials_override_is_path_inside(const StringRefNull path)
{
  return BLI_path_contains(essentials_override_directory_path().c_str(), path.c_str());
}

std::string essentials_asset_path_resolve_to_override_path(StringRefNull essentials_libpath_abs,
                                                           const ID_Type id_type,
                                                           StringRefNull asset_name)
{
  const std::string essentials_directory = asset_system::essentials_directory_path() + SEP_STR;
  const StringRefNull essentials_override_directory =
      asset_system::essentials_override_directory_path();

  if (!BLI_path_contains(essentials_directory.c_str(), essentials_libpath_abs.c_str())) {
    BLI_assert_unreachable();
    return "";
  }
  BLI_assert(BKE_blendfile_extension_check(essentials_libpath_abs.c_str()));

  /* Filepath within the essentials library. */
  char path_relative[FILE_MAX];
  BLI_strncpy(path_relative, essentials_libpath_abs.c_str(), sizeof(path_relative));
  BLI_path_rel(path_relative, essentials_directory.c_str());

  /* Remove the `.blend` extension. */
  BLI_path_extension_strip(path_relative);

  const std::string rootpath = essentials_override_directory + SEP_STR +
                               /* +2 to skip "//" relative path marker. */
                               (path_relative + 2) + SEP_STR + BKE_idtype_idcode_to_name(id_type);

  /* Make sure filename only contains valid characters for file-system. */
  char base_name_filesafe[FILE_MAXFILE];
  BLI_strncpy(base_name_filesafe, asset_name.c_str(), sizeof(base_name_filesafe));
  BLI_path_make_safe_filename(base_name_filesafe);

  return rootpath + SEP + base_name_filesafe + BLENDER_ASSET_FILE_SUFFIX;
}

std::string essentials_asset_override_full_path(const AssetRepresentation &asset)
{
  const StringRefNull essentials_override_directory =
      asset_system::essentials_override_directory_path();
  return essentials_override_directory + SEP_STR + asset.library_relative_identifier();
}

bool essentials_asset_override_exists(const AssetRepresentation &asset)
{
  BLI_assert(asset.owner_asset_library().library_type() == ASSET_LIBRARY_ESSENTIALS);

  /* Overriding overrides is not supported. */
  if (asset.is_essentials_override()) {
    return false;
  }

  const std::string asset_libpath = asset.full_library_path();
  /* For unoverridden essentials assets, the path should always exists, they come bundled. */
  BLI_assert(BLI_exists(asset_libpath.c_str()));
  BLI_assert(
      BLI_path_contains(asset_system::essentials_directory_path().c_str(), asset_libpath.c_str()));

  const std::string override_path = asset_system::essentials_asset_path_resolve_to_override_path(
      asset_libpath, asset.get_id_type(), asset.get_name());

  return BLI_exists(override_path.c_str());
}

}  // namespace blender::asset_system
