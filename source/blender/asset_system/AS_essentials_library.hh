/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup asset_system
 */

#pragma once

#include "DNA_ID_enums.h"

#include "BLI_string_ref.hh"

struct AssetWeakReference;

namespace blender::asset_system {

class AssetRepresentation;

StringRefNull essentials_directory_path();
StringRefNull essentials_override_directory_path();

bool essentials_override_is_path_inside(const StringRefNull path);

/**
 * Given a path to an essentials override, attempt to reconstruct the original asset blend file
 * location.
 */
std::string essentials_asset_override_path_to_essentials_blend_path(
    StringRefNull essentials_override_path);
/**
 * Given an original essentials file path and further asset info, return the path at which the
 * overriding .asset.blend will be expected. Does not check if any of the involved file paths
 * exist on disk.
 */
std::string essentials_asset_override_blend_path_resolve(StringRefNull essentials_blendpath_abs,
                                                         ID_Type id_type,
                                                         StringRefNull asset_name);
/**
 * Returns the path to the override .blend file if it exists. Otherwise returns an empty string.
 */
std::string essentials_asset_override_blend_path_from_reference(
    const AssetWeakReference &asset_reference);
/**
 * Returns the path to the asset if the override .blend file exists. Otherwise returns an empty
 * string.
 */
std::string essentials_asset_override_full_path_from_reference(
    const AssetWeakReference &asset_reference);
/**
 * Given an asset that overrides an essential (#AssetRepresentation.is_essentials_override()
 * returns true), return the full path to the override version of the asset.
 */
std::string essentials_asset_override_full_path(const AssetRepresentation &asset);
/**
 * Return if a file exists at the expected location that would override the given essentials
 * asset.
 */
bool essentials_asset_override_exists(const AssetRepresentation &asset);
bool essentials_asset_override_exists(const AssetWeakReference &asset_reference);

}  // namespace blender::asset_system
