/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup asset_system
 */

#pragma once

#include "DNA_ID_enums.h"

#include "BLI_string_ref.hh"

namespace blender::asset_system {

class AssetRepresentation;

StringRefNull essentials_directory_path();
StringRefNull essentials_override_directory_path();

bool essentials_override_is_path_inside(const StringRefNull path);

std::string essentials_asset_path_resolve_to_override_path(StringRefNull essentials_libpath_abs,
                                                           ID_Type id_type,
                                                           StringRefNull asset_name);
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

}  // namespace blender::asset_system
