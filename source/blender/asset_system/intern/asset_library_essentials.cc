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

std::string essentials_asset_override_path_to_essentials_blend_path(
    StringRefNull essentials_override_path)
{
  const std::string essentials_directory = essentials_directory_path();
  const StringRefNull essentials_override_directory = essentials_override_directory_path();

  BLI_assert(essentials_override_is_path_inside(essentials_override_path));
  BLI_assert(essentials_override_path.startswith(essentials_override_directory));

  char relative_path_buf[FILE_MAX];
  BLI_assert(essentials_override_path.size() > essentials_override_directory.size());
  STRNCPY(relative_path_buf,
          essentials_override_path.c_str() + essentials_override_directory.size());

  char *slash;
  while ((slash = (char *)BLI_path_slash_rfind(relative_path_buf))) {
    *slash = '\0';
    BLI_path_extension_ensure(relative_path_buf, sizeof(relative_path_buf), ".blend");

    std::string possible_essentials_path = essentials_directory + relative_path_buf;
    if (BLI_is_file(possible_essentials_path.c_str())) {
      return possible_essentials_path;
    }
  }

  return "";
}

std::string essentials_asset_override_blend_path_resolve(StringRefNull essentials_blendpath_abs,
                                                         const ID_Type id_type,
                                                         StringRefNull asset_name)
{
  const std::string essentials_directory = essentials_directory_path() + SEP_STR;
  const StringRefNull essentials_override_directory = essentials_override_directory_path();

  BLI_assert(BKE_blendfile_extension_check(essentials_blendpath_abs.c_str()));
  if (!BLI_path_contains(essentials_directory.c_str(), essentials_blendpath_abs.c_str())) {
    /* Ensure path is actually a .blend withing the original essentials location. */
    BLI_assert_unreachable();
    return "";
  }

  /* Filepath within the essentials library. */
  char path_relative[FILE_MAX];
  BLI_strncpy(path_relative, essentials_blendpath_abs.c_str(), sizeof(path_relative));
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

static int groupname_to_code(const char *group)
{
  char buf[32 /*BLO_GROUP_MAX*/];
  char *lslash;

  BLI_assert(group);

  STRNCPY(buf, group);
  lslash = (char *)BLI_path_slash_rfind(buf);
  if (lslash) {
    lslash[0] = '\0';
  }

  return buf[0] ? BKE_idtype_idcode_from_name(buf) : 0;
}

static bool essentials_asset_override_paths_from_reference(
    const AssetWeakReference &asset_reference,
    std::string *r_blend_path = nullptr,
    std::string *r_full_path = nullptr)
{
  BLI_assert(asset_reference.asset_library_type == ASSET_LIBRARY_ESSENTIALS);

  /* First, see if the weak reference already points to an override. */
  {
    const std::string path_in_overrides = essentials_override_directory_path() + SEP_STR +
                                          asset_reference.relative_asset_identifier;
    char blend_path[1090 /*FILE_MAX_LIBEXTRA*/];
    char *group, *name;
    if (BKE_blendfile_library_path_explode(path_in_overrides.c_str(), blend_path, &group, &name)) {
      BLI_assert(BLI_is_file(blend_path));

      if (r_blend_path) {
        *r_blend_path = blend_path;
      }
      if (r_full_path) {
        *r_full_path = path_in_overrides;
      }
      return true;
    }
  }

  /* Second, resolve the weak reference into an override path and see if an override file exists at
   * that location. */
  {
    const std::string path_in_essentials = essentials_directory_path() + SEP_STR +
                                           asset_reference.relative_asset_identifier;

    char blend_path[1090 /*FILE_MAX_LIBEXTRA*/];
    char *group, *name;
    if (BKE_blendfile_library_path_explode(path_in_essentials.c_str(), blend_path, &group, &name))
    {
      const int idcode = groupname_to_code(group);
      BLI_assert(idcode != 0);
      const std::string override_blend_path = essentials_asset_override_blend_path_resolve(
          blend_path, ID_Type(idcode), name);

      if (BLI_is_file(override_blend_path.c_str())) {
        if (r_blend_path) {
          *r_blend_path = override_blend_path;
        }
        if (r_full_path) {
          *r_full_path = override_blend_path + SEP + group + SEP + name;
        }
        return true;
      }
    }
  }

  return false;
}

std::string essentials_asset_override_blend_path_from_reference(
    const AssetWeakReference &asset_reference)
{
  std::string blend_path;
  if (essentials_asset_override_paths_from_reference(asset_reference, &blend_path)) {
    return blend_path;
  }
  return "";
}

std::string essentials_asset_override_full_path_from_reference(
    const AssetWeakReference &asset_reference)
{
  std::string full_path;
  if (essentials_asset_override_paths_from_reference(asset_reference, nullptr, &full_path)) {
    return full_path;
  }
  return "";
}

std::string essentials_asset_override_full_path(const AssetRepresentation &asset)
{
  BLI_assert(asset.owner_asset_library().library_type() == ASSET_LIBRARY_ESSENTIALS);
  BLI_assert(asset.is_essentials_override());

  return essentials_override_directory_path() + SEP_STR + asset.library_relative_identifier(true);
}

bool essentials_asset_override_exists(const AssetRepresentation &asset)
{
  BLI_assert(asset.owner_asset_library().library_type() == ASSET_LIBRARY_ESSENTIALS);

  /* Overriding overrides is not supported. */
  if (asset.is_essentials_override()) {
    return false;
  }

  const std::string asset_libpath = asset.full_library_path(false);
  /* For unoverridden essentials assets, the path should always exists, they come bundled. */
  BLI_assert(BLI_exists(asset_libpath.c_str()));
  BLI_assert(BLI_path_contains(essentials_directory_path().c_str(), asset_libpath.c_str()));

  const std::string override_path = essentials_asset_override_blend_path_resolve(
      asset_libpath, asset.get_id_type(), asset.get_name());
  return BLI_exists(override_path.c_str());
}

bool essentials_asset_override_exists(const AssetWeakReference &asset_reference)
{
  return essentials_asset_override_paths_from_reference(asset_reference);
}

}  // namespace blender::asset_system
