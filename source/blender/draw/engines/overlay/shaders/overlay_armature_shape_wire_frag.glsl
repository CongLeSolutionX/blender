/* SPDX-FileCopyrightText: 2019-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(select_lib.glsl)

/**
 * We want to know how much a pixel is covered by a line.
 * We replace the square pixel with a circle of the same area and try to find the intersection
 * area. The area we search is the circular segment. https://en.wikipedia.org/wiki/Circular_segment
 * The formula for the area uses inverse trig function and is quite complex. Instead,
 * we approximate it by using the smooth-step function and a 1.05 factor to the disc radius.
 */

#define M_1_SQRTPI 0.5641895835477563 /* `1/sqrt(pi)`. */

#define DISC_RADIUS (M_1_SQRTPI * 1.05)
#define GRID_LINE_SMOOTH_START (0.5 - DISC_RADIUS)
#define GRID_LINE_SMOOTH_END (0.5 + DISC_RADIUS)

bool test_occlusion()
{
  return gl_FragCoord.z > texelFetch(depthTex, ivec2(gl_FragCoord.xy), 0).r;
}

float edge_step(float dist)
{
  if (do_smooth_wire) {
    return smoothstep(GRID_LINE_SMOOTH_START, GRID_LINE_SMOOTH_END, dist);
  }
  else {
    return step(0.5, dist);
  }
}

void main()
{
  const float dist = abs(geometry_noperspective_out.edgeCoord) - max(geometry_out.wire_width/2.0 - 0.5, 0.0);
  const float mix_w = edge_step(dist);

  fragColor = mix(vec4(0), vec4(geometry_out.finalColor.rgb, alpha), 1.0 - mix_w);
  fragColor.a *= 1.0 - mix_w;
  fragColor.a *= test_occlusion() ? alpha : 1.0;
  select_id_output(select_id);
}
