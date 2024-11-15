/* SPDX-FileCopyrightText: 2018-2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#define rect parameters[widgetID * MAX_PARAM + 0]
#define colorInner parameters[widgetID * MAX_PARAM + 1]
#define colorOutline parameters[widgetID * MAX_PARAM + 2]
#define outlineThickness parameters[widgetID * MAX_PARAM + 3].x
#define outlineOffset parameters[widgetID * MAX_PARAM + 3].y
#define dotRadius parameters[widgetID * MAX_PARAM + 3].z
#define shape parameters[widgetID * MAX_PARAM + 3].w

void main()
{
  /* Offsetting by a pixel further to avoid losing pixels. */
  vec2 ofs = vec2(outlineOffset + 1.0, -outlineOffset - 1.0);
  vec2 pos;
  switch (gl_VertexID) {
    default:
    case 0: {
      pos = rect.xz + ofs.yy;
      break;
    }
    case 1: {
      pos = rect.xw + ofs.yx;
      break;
    }
    case 2: {
      pos = rect.yz + ofs.xy;
      break;
    }
    case 3: {
      pos = rect.yw + ofs.xx;
      break;
    }
  }

  gl_Position = ModelViewProjectionMatrix * vec4(pos, 0.0, 1.0);

  vec2 rectSize = rect.yw - rect.xz + 2.0 * vec2(outlineOffset, outlineOffset);
  float minSize = min(rectSize.x, rectSize.y);

  vec2 centeredCoordinates = pos - ((rect.xz + rect.yw) / 2.0);
  uv = centeredCoordinates / minSize;

  /* Calculate the necessary "extrusion" of the coordinates to draw the middle part of
   * multi sockets. */
  float aspect = rectSize.x / rectSize.y;
  extrusion = (aspect > 1.0) ? vec2((aspect - 1.0) / 2.0, 0.0) :
                               vec2(0.0, ((1.0 / aspect) - 1.0) / 2.0);

  /* Shape parameters. */
  finalShape = int(shape);
  finalOutlineThickness = outlineThickness / minSize;
  finalDotRadius = dotRadius / minSize;
  AAsize = 1.0 / minSize;

  /* Pass through parameters. */
  finalColor = colorInner;
  finalOutlineColor = colorOutline;
}
