/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <pxr/base/tf/token.h>

namespace usdtokens {

/* USD Blender material attribute name. */
inline const pxr::TfToken blender_ns_data_name("userProperties:blender:data_name",
                                               pxr::TfToken::Immortal);

/* USD Blender object attribute name. */
inline const pxr::TfToken blender_ns_object_name("userProperties:blender:object_name",
                                                 pxr::TfToken::Immortal);

}  // namespace usdtokens
