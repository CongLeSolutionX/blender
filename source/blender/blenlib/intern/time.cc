/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 */

#include "BLI_math_base.h"
#include "BLI_string.h"
#include "BLI_string_utils.hh"
#include "BLI_time.hh"


size_t BLI_time_approx_duration_string_from_seconds(char *str,
                                                    const size_t maxncpy,
                                                    const double time_seconds)
{
  /* long enough to hold "nnn days\0"
   *                     "nn hours\0"
   *                     "nn minutes\0"
   *                     "nn seconds\0"
   * respectively
   */
  char days_str[9] = "";
  char hours_str[9] = "";
  char minutes_str[11] = "";
  char seconds_str[11] = "";
  const char *string_array[4];
  uint i = 0;
  int precision = 2;

  const int days = ((int)time_seconds) / (60 * 60 * 24);
  const int hours = ((int)time_seconds) / (60 * 60);
  const int minutes = (((int)time_seconds) / 60) % 60;
  const int seconds = ((int)ceil(time_seconds)) % 60;

  if (days) {
    BLI_snprintf(days_str, sizeof(days_str), "%dd", min_ii(999, days));
    string_array[i++] = days_str;
    precision--;
  }
  if (hours && precision>0) {
    BLI_snprintf(hours_str, sizeof(hours_str), "%dh", hours);
    string_array[i++] = hours_str;
    precision--;
  }
  if (minutes && precision>0) {
    BLI_snprintf(minutes_str, sizeof(minutes_str), "%dm", minutes);
    string_array[i++] = minutes_str;
    precision--;
  }
  if (seconds && precision>0) {
    BLI_snprintf(seconds_str, sizeof(seconds_str), "%ds", seconds);
    string_array[i++] = seconds_str;
  }

  return BLI_string_join_array_by_sep_char(str, maxncpy, ' ', string_array, i);
}