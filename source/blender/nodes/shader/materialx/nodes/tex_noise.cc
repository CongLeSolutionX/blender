/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem TexNoiseNodeParser::compute()
{
  NodeItem scale = get_input_value("Scale");
  NodeItem detail = get_input_value("Detail");
  NodeItem lacunarity = get_input_value("Lacunarity");

  if (detail.value && detail.type() == "float") {
    detail = value(int(detail.value->asA<float>()));
  }

  NodeItem texcoord = create_node("position", "vector3");

  NodeItem res = create_node("fractal3d", "color3", false);
  res.set_input("position", texcoord * scale);
  res.set_input("octaves", detail);
  res.set_input("lacunarity", lacunarity);
  return res;
}

}  // namespace blender::nodes::materialx
