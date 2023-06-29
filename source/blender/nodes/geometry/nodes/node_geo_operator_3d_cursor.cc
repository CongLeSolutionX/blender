/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_view3d_types.h"

#include "BLI_math_matrix.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_operator_3d_cursor_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Vector>("Location").subtype(PROP_DISTANCE);
  b.add_output<decl::Rotation>("Rotation");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  if (!params.user_data()->operator_data) {
    params.error_message_add(NodeWarningType::Error, "Node must be run as operator");
    params.set_default_remaining_outputs();
    return;
  }
  const float4x4 world_to_object(params.user_data()->operator_data->self_object->world_to_object);
  const View3DCursor &cursor = params.user_data()->operator_data->scene->cursor;
  const float3 location_global(cursor.location);
  const math::Quaternion rotation_global(float4(cursor.rotation_quaternion));
  params.set_output("Location", math::transform_point(world_to_object, location_global));
  params.set_output("Rotation", math::to_quaternion(world_to_object) * rotation_global);
}

}  // namespace blender::nodes::node_geo_operator_3d_cursor_cc

void register_node_type_geo_operator_3d_cursor()
{
  namespace file_ns = blender::nodes::node_geo_operator_3d_cursor_cc;
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_OPERATOR_3D_CURSOR, "3D Cursor", NODE_CLASS_INPUT);
  ntype.declare = file_ns::node_declare;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.gather_add_node_search_ops = blender::nodes::search_link_ops_for_for_operator_node;
  ntype.gather_link_search_ops = blender::nodes::search_link_ops_for_operator_node;
  nodeRegisterType(&ntype);
}
