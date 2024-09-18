/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editors
 */

#pragma once

#include "BLI_vector.hh"

void ED_operatortypes_userpref();

blender::Vector<int> ED_userpref_tabs_list(SpaceUserPref *prefs);
bool ED_userpref_tab_has_search_result(SpaceUserPref *sprefs, int index);
void ED_userpref_search_string_set(SpaceUserPref *sprefs, const char *value);
int ED_userpref_search_string_length(SpaceUserPref *sprefs);
const char *ED_userpref_search_string_get(SpaceUserPref *sprefs);
