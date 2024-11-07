/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

#include <pxr/usd/usd/prim.h>

#include <string>

struct ID;

namespace blender::io::usd {

/**
 * Return a valid USD identifier based on the passed in string.
 *
 * \param name: Incoming name to sanitize
 * \param allow_unicode: Whether to allow unicode encoded characters in the USD identifier
 * \return A valid USD identifier
 */
std::string make_safe_name(const std::string &name, bool allow_unicode);

void set_id_name_to_prim(const ID *id, const pxr::TfToken &token, const pxr::UsdPrim &prim);

}  // namespace blender::io::usd
