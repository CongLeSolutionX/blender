/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(common_view_clipping_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)

void do_vertex(vec4 color, vec4 pos, float coord, vec2 offset)
{
  geometry_out.finalColor = color;
  geometry_noperspective_out.edgeCoord = coord;
  gl_Position = pos;
  /* Multiply offset by 2 because gl_Position range is [-1..1]. */
  gl_Position.xy += offset * 2.0 * pos.w;
  /* Correct but fails due to an AMD compiler bug, see: #62792.
   * Do inline instead. */
#if 0
  view_clipping_distances_set(gl_in[i]);
#endif
  gpu_EmitVertex();
}

void main()
{
  /* Clip line against near plane to avoid deformed lines. */
  vec4 pos0 = gl_in[0].gl_Position;
  vec4 pos1 = gl_in[1].gl_Position;
  const vec2 pz_ndc = vec2(pos0.z / pos0.w, pos1.z / pos1.w);
  const bvec2 clipped = lessThan(pz_ndc, vec2(-1.0));
  if (all(clipped)) {
    /* Totally clipped. */
    return;
  }

  const vec4 pos01 = pos0 - pos1;
  const float ofs = abs((pz_ndc.y + 1.0) / (pz_ndc.x - pz_ndc.y));
  if (clipped.y) {
    pos1 += pos01 * ofs;
  }
  else if (clipped.x) {
    pos0 -= pos01 * (1.0 - ofs);
  }

  vec2 screen_space_pos[2];
  screen_space_pos[0] = pos0.xy / pos0.w;
  screen_space_pos[1] = pos1.xy / pos1.w;

  const float wire_width = geometry_in[0].wire_width;
  geometry_out.wire_width = wire_width;
  float half_size = max(wire_width / 2.0, 0.5);

  if (do_smooth_wire) {
    /* Add 1px for AA */
    half_size += 0.5;
  }

  const vec2 line = abs(screen_space_pos[0] - screen_space_pos[1]) * sizeViewport.xy;
  vec2 edge_ofs = half_size * sizeViewportInv;
  if (line.x > line.y) {
    edge_ofs[0] = 0.0;
  }
  else {
    edge_ofs[1] = 0.0;
  }

  /* Due to an AMD glitch, this line was moved out of the `do_vertex`
   * function (see #62792). */
  view_clipping_distances_set(gl_in[0]);
  do_vertex(geometry_in[0].finalColor, pos0, half_size, edge_ofs);
  do_vertex(geometry_in[0].finalColor, pos0, -half_size, -edge_ofs);

  view_clipping_distances_set(gl_in[1]);
  const vec4 final_color = (geometry_in[0].selectOverride_ == 0u) ? geometry_in[1].finalColor :
                                                              geometry_in[0].finalColor;
  do_vertex(final_color, pos1, half_size, edge_ofs);
  do_vertex(final_color, pos1, -half_size, -edge_ofs);

  EndPrimitive();
}
