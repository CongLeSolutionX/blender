/* SPDX-FileCopyrightText: 2019-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(select_lib.glsl)

void main()
{
  fragColor = finalColor;

  /* Stipple */
  const float dash_width = 6.0;
  const float dash_factor = 0.5;

  lineOutput = pack_line_data(gl_FragCoord.xy, stipple_start, stipple_coord);

  float dist = distance(stipple_start, stipple_coord);

  if (fragColor.a == 0.0) {
    /* Disable stippling. */
    dist = 0.0;
  }

  fragColor.a = 1.0;

#if !(defined(SELECT_EDGES) || defined(SELECT_ENABLE))
  if (fract(dist / dash_width) > dash_factor) {
    discard;
  }
#endif
  select_id_output(select_id);
}
