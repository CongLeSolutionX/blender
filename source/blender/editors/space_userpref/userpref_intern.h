/* SPDX-FileCopyrightText: 2008 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spuserpref
 */

#pragma once

#include "BLI_bitmap.h"

/* internal exports only */

struct SpaceUserPref_Runtime {
  /** For filtering properties displayed in the space. */
  char search_string[128];
  /**
   * Bit-field (in the same order as the tabs) for whether each tab has properties
   * that match the search filter. Only valid when #search_string is set.
   */
  BLI_bitmap *tab_search_results;
};
