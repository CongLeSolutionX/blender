/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 * \brief Platform independent time functions.
 */

#pragma once

#include "BLI_string_utils.hh"
#include "BLI_sys_types.h"

/**
 * Generate time string and store in \a str
 *
 * Format is like "5h 5m"
 *
 * \param str: destination string
 * \param maxncpy: maximum number of characters to copy `sizeof(str)`
 * \param time_seconds: time total time in seconds
 * \return length of \a str
 */
size_t BLI_time_approx_duration_string_from_seconds(char *str,
                                                    const size_t maxncpy,
                                                    const double time_seconds);
