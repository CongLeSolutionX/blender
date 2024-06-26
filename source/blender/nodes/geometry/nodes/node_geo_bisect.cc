/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include "BLI_math_geom.h"
#include "BLI_math_solvers.hh"
#include "BLI_task.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_pointcloud_types.h"

#include "BKE_curves.hh"
#include "BKE_mesh.h"
//#include "BKE_pointcloud.h"
// #include "BKE_volume.h"

// #include "bmesh_mesh.hh"
//  #include "bmesh_tools.hh"

#include "GEO_trim_curves.hh"
#include "GEO_mesh_bisect.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_curve_bisect_cc {

/*
 * Mesh
 */
static void geo_node_bisect_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(N_("Geometry"));

  b.add_input<decl::Vector>(N_("Origin")).default_value({0.0f, 0.0f, 0.0f}).subtype(PROP_XYZ);
  b.add_input<decl::Vector>(N_("Direction"))
      .default_value({1.0f, 0.0f, 0.0f})
      .subtype(PROP_DIRECTION);
  b.add_input<decl::Bool>(N_("Fill")).default_value(true);
  b.add_input<decl::Bool>(N_("Clear Inner")).default_value(true);
  b.add_input<decl::Bool>(N_("Clear Outer")).default_value(false);
  // TODO: Vary based on input.
  // b.add_input<decl::Float>(N_("Distance")).subtype(PROP_EULER);
  b.add_output<decl::Geometry>(N_("Geometry")).propagate_all();
}

class BisectSplineSampler;

using bke::attribute_math::mix2;

/*
static Mesh *plane_clip_mesh(const Mesh *mesh_in, const blender::geometry::BisectArgs &args)
{
  BMeshCreateParams bm_params{1};  // Default
  BMeshFromMeshParams to_bm_params{};
  BMesh *bm = BKE_mesh_to_bmesh_ex(mesh_in, &bm_params, &to_bm_params);

  BMOperator bmop;
  BMO_op_initf(
      bm,
      &bmop,
      0,
      "bisect_plane geom=%avef plane_co=%v plane_no=%v dist=%f clear_inner=%b clear_outer=%b",
      args.plane_co,
      args.plane_no,
      0.0f,
      args.clear_inner,
      args.clear_outer);
  BMO_op_exec(bm, &bmop);

  if (args.use_fill) {
    float normal_fill[3];
    BMOperator bmop_fill;
    BMOperator bmop_attr;

    //The fill normal sign is ignored as the face-winding is defined by surrounding faces.
    // The normal is passed so triangle fill won't have to calculate it.
    normalize_v3_v3(normal_fill, args.plane_no);

    // Fill
    BMO_op_initf(bm,
                 &bmop_fill,
                 0,
                 "triangle_fill edges=%S normal=%v use_dissolve=%b",
                 &bmop,
                 "geom_cut.out",
                 normal_fill,
                 true);
    BMO_op_exec(bm, &bmop_fill);

    // Copy Attributes
    BMO_op_initf(bm,
                 &bmop_attr,
                 0,
                 "face_attribute_fill faces=%S use_normals=%b use_data=%b",
                 &bmop_fill,
                 "geom.out",
                 true,
                 true);
    BMO_op_exec(bm, &bmop_attr);

    BMO_slot_buffer_hflag_enable(
        bm, bmop_fill.slots_out, "geom.out", BM_FACE, BM_ELEM_SELECT, true);

    BMO_op_finish(bm, &bmop_attr);
    BMO_op_finish(bm, &bmop_fill);
  }
  BMO_op_finish(bm, &bmop);
  BMeshToMeshParams from_bm_params{};
  return BKE_mesh_from_bmesh_nomain(bm, &from_bm_params, mesh_in);
}
*/

/*
static void plane_clip_point_cloud(GeometrySet &geometry_set, const blender::geometry::BisectArgs
&args)
{
  const PointCloudComponent &src_points =
      *geometry_set.get_component_for_read<PointCloudComponent>();

  Vector<int64_t> indices;
  // TODO: COmpute indices
  const PointCloud *src_pcloud = src_points.get_for_read();

  for (auto index : IndexRange(src_pcloud->totpoint)) {

    float d = dist_signed_to_plane_v3(src_pcloud->co[index], args.plane);
    bool infront = d >= 0.0f;
    if ((infront && !args.clear_outer) || (!infront && !args.clear_inner)) {
      indices.append(index);
    }
  }

  const IndexMask mask(indices);
  const int total = mask.size();
  PointCloud *pointcloud = BKE_pointcloud_new_nomain(total);


  if (total == 0) {
    geometry_set.replace_pointcloud(pointcloud);
    return;
  }

  PointCloudComponent dst_points;
  dst_points.replace(pointcloud, GeometryOwnershipType::Editable);

  Map<AttributeIDRef, AttributeKind> attributes;
  geometry_set.gather_attributes_for_propagation(
      {GEO_COMPONENT_TYPE_POINT_CLOUD}, GEO_COMPONENT_TYPE_POINT_CLOUD, false, attributes);

  node_geo_delete_geometry_cc::copy_attributes_based_on_mask(
      attributes, src_points, dst_points, ATTR_DOMAIN_POINT, mask);
  geometry_set.replace_pointcloud(pointcloud);
}
*/

static void geometry_set_curve_bisect(GeometrySet &geometry_set,
                                      const blender::geometry::BisectArgs &args,
                                      const AnonymousAttributePropagationInfo &propagation_info)
{
  if (!geometry_set.has_curves()) {
    return;
  }
  const Curves &src_curves_id = *geometry_set.get_curves();
  const bke::CurvesGeometry &src_curves = src_curves_id.geometry.wrap();
  if (src_curves.curves_num() == 0) {
    return;
  }

  /*
  bke::CurvesFieldContext field_context{src_curves, ATTR_DOMAIN_CURVE};
  fn::FieldEvaluator evaluator{field_context, src_curves.curves_num()};
  evaluator.add(selection_field);
  evaluator.add(start_field);
  evaluator.add(end_field);
  evaluator.evaluate();

  const IndexMask selection = evaluator.get_evaluated_as_mask(0);
  const VArray<float> starts = evaluator.get_evaluated<float>(1);
  const VArray<float> ends = evaluator.get_evaluated<float>(2);

  if (selection.is_empty()) {
    return;
  }
  */

  bke::CurvesGeometry dst_curves = geometry::bisect_curves(
      src_curves, src_curves.curves_range(), args, propagation_info);

  Curves *dst_curves_id = bke::curves_new_nomain(std::move(dst_curves));
  bke::curves_copy_parameters(src_curves_id, *dst_curves_id);
  geometry_set.replace_curves(dst_curves_id);
}

static void geo_node_bisect_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
  const AnonymousAttributePropagationInfo &propagation_info = params.get_output_propagation_info(
      "Geometry");

  blender::geometry::BisectArgs args;

  args.plane_no = params.extract_input<float3>("Direction");
  args.plane_co = params.extract_input<float3>("Origin");
  //args.use_fill = params.extract_input<bool>("Fill");
  args.clear_inner = params.extract_input<bool>("Clear Inner");
  args.clear_outer = params.extract_input<bool>("Clear Outer");

  // Always normalize to avoid ambiguity
  float norm = normalize_v3(args.plane_no);
  if (norm == 0.0f) {  // Zero if norm ~0
  }
  else {
    plane_from_point_normal_v3(args.plane, args.plane_co, args.plane_no);

    geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
      
      if (geometry_set.has_mesh()) {
        const Mesh *mesh_in = geometry_set.get_mesh();

        std::pair<Mesh *, geometry::BisectResult> result = geometry::bisect_mesh(*mesh_in, args, propagation_info);
        if (result.second == geometry::BisectResult::Keep) {
          /* Do nothing => forward original mesh */
        }
        else {
          geometry_set.replace_mesh(result.first, bke::GeometryOwnershipType::Owned);
        }
      }
      /*
      if (geometry_set.has_pointcloud()) {
        plane_clip_point_cloud(geometry_set, args);
      }
      */

      if (geometry_set.has_curves()) {
        geometry_set_curve_bisect(geometry_set, args, propagation_info);
      }
    });
  }

  params.set_output("Geometry", std::move(geometry_set));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_BISECT, "Bisect", NODE_CLASS_GEOMETRY);
  ntype.declare = geo_node_bisect_declare;
  ntype.geometry_node_execute = geo_node_bisect_exec;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_curve_bisect_cc
