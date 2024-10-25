/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup animrig
 *
 * \brief Work with Actions as Assets.
 */
#pragma once

#include <stdint.h>

struct ID;
struct Main;

namespace blender::animrig {
class Action;
}

namespace blender::animrig::asset {

enum class InjectActionResult : int8_t {
  OK = 0,
  NO_SUITABLE_SLOT = 1,
};

InjectActionResult inject_action(Main &bmain,
                                 ID &animated_id,
                                 const Action &action_asset,
                                 float frame);

}  // namespace blender::animrig::asset
