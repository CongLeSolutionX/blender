#include "BLI_array_utils.hh"
#include "BLI_vector_set.hh"

#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_pointcloud_types.h"

#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_rna_define.hh"

#include "node_geometry_util.hh"
#include <list>
#include <unordered_map>

#include "voro++.hh"

namespace blender::nodes::node_geo_voronoi {

NODE_STORAGE_FUNCS(NodeGeometryVoronoi)

namespace {
struct AttributeOutputs {
  std::optional<std::string> cell_id;
  std::optional<std::string> cell_centers;
};
}  // namespace

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Sites");
  auto &b_min = b.add_input<decl::Vector>("Min").default_value(float3(-1.0f));
  auto &b_max = b.add_input<decl::Vector>("Max").default_value(float3(1.0f));
  auto &a = b.add_input<decl::Float>("A").default_value(1.0f);
  auto &bx = b.add_input<decl::Float>("Bx").default_value(0.0f);
  auto &by = b.add_input<decl::Float>("By").default_value(1.0f);
  auto &cx = b.add_input<decl::Float>("Cx").default_value(0.0f);
  auto &cy = b.add_input<decl::Float>("Cy").default_value(0.0f);
  auto &cz = b.add_input<decl::Float>("Cz").default_value(1.0f);
  auto &px = b.add_input<decl::Bool>("Periodic X");
  auto &py = b.add_input<decl::Bool>("Periodic Y");
  auto &pz = b.add_input<decl::Bool>("Periodic Z");
  b.add_input<decl::Bool>("Boundary");
  b.add_input<decl::Bool>("Group By Edge");
  b.add_input<decl::Int>("Group ID").implicit_field(implicit_field_inputs::index);
  b.add_output<decl::Geometry>("Voronoi");
  b.add_output<decl::Int>("Cell ID").field_on_all();
  b.add_output<decl::Vector>("Cell Centers").field_on_all();

  const bNode *node = b.node_or_null();
  if (node != nullptr) {
    const NodeGeometryVoronoi &storage = node_storage(*node);
    const GeometryNodeVoronoiMode mode = GeometryNodeVoronoiMode(storage.mode);

    b_min.available(mode == GEO_NODE_VORONOI_BOUNDS);
    b_max.available(mode == GEO_NODE_VORONOI_BOUNDS);
    px.available(mode == GEO_NODE_VORONOI_BOUNDS);
    py.available(mode == GEO_NODE_VORONOI_BOUNDS);
    pz.available(mode == GEO_NODE_VORONOI_BOUNDS);
    a.available(mode == GEO_NODE_VORONOI_BRAVAIS);
    bx.available(mode == GEO_NODE_VORONOI_BRAVAIS);
    by.available(mode == GEO_NODE_VORONOI_BRAVAIS);
    cx.available(mode == GEO_NODE_VORONOI_BRAVAIS);
    cy.available(mode == GEO_NODE_VORONOI_BRAVAIS);
    cz.available(mode == GEO_NODE_VORONOI_BRAVAIS);
  }
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryVoronoi *data = MEM_cnew<NodeGeometryVoronoi>(__func__);

  data->mode = GEO_NODE_VORONOI_BOUNDS;
  node->storage = data;
}

static Mesh *compute_voronoi_bounds(GeometrySet &sites,
                                    AttributeOutputs &attribute_outputs,
                                    const float3 &min,
                                    const float3 &max,
                                    const Field<int> &group_id,
                                    bool edge_group,
                                    bool x_p,
                                    bool y_p,
                                    bool z_p,
                                    bool boundary)
{
  Span<float3> positions;
  VArray<int> group_ids;
  std::unordered_map<int, std::list<int>> adjacency_list;

  if (sites.has_mesh()) {
    const Mesh *site_mesh = sites.get_mesh();
    positions = site_mesh->vert_positions();
    if (edge_group) {
      Span<int2> edges = site_mesh->edges();
      for (auto edge : edges) {
        adjacency_list[edge[0]].push_back(edge[1]);
        adjacency_list[edge[1]].push_back(edge[0]);
      }
    }
    const bke::AttrDomain att_domain = bke::AttrDomain::Point;
    const int domain_size = site_mesh->attributes().domain_size(att_domain);
    bke::MeshFieldContext field_context{*site_mesh, att_domain};
    FieldEvaluator field_evaluator{field_context, domain_size};
    field_evaluator.add(group_id);
    field_evaluator.evaluate();
    group_ids = field_evaluator.get_evaluated<int>(0);
  }
  if (sites.has_pointcloud()) {
    const PointCloud *site_pc = sites.get_pointcloud();
    positions = site_pc->positions();

    bke::PointCloudFieldContext field_context{*site_pc};
    FieldEvaluator field_evaluator{field_context, site_pc->totpoint};
    field_evaluator.add(group_id);
    field_evaluator.evaluate();
    group_ids = field_evaluator.get_evaluated<int>(0);
  }

  if (sites.has_curves()) {
    const Curves *site_curves = sites.get_curves();
    const bke::CurvesGeometry &src_curves = site_curves->geometry.wrap();
    positions = src_curves.evaluated_positions();
    // default id would have been the index which is not available for the evaluated points on the
    // curve creating a VArray with the size of the positions instead
    group_ids = VArray<int>::ForFunc(positions.size(), [](const int64_t i) { return i; });
  }

  // Set the computational grid size
  const int n_x = 14, n_y = 14, n_z = 14;

  // Create a container with the geometry given above, and make it
  // non-periodic in each of the three coordinates. Allocate space for
  // eight particles within each computational block.
  voro::container con(
      min[0], max[0], min[1], max[1], min[2], max[2], n_x, n_y, n_z, x_p, y_p, z_p, 8);
  int i, j, k, l, n, id;

  i = 0;
  for (float3 p : positions) {
    con.put(group_ids[i], p[0], p[1], p[2]);
    i++;
  }

  voro::voronoicell_neighbor c;
  voro::c_loop_all vl(con);

  std::vector<int> neigh, f_vert;
  std::vector<double> v;

  Vector<float3> verts;
  Vector<int> face_sizes;
  Vector<int> corner_verts;
  Vector<int> ids;
  Vector<float3> centers;
  double x, y, z;

  int offset = 0;

  if (!edge_group) {
    if (vl.start())
      do
        if (con.compute_cell(c, vl)) {
          vl.pos(x, y, z);
          id = vl.pid();

          c.neighbors(neigh);
          c.face_vertices(f_vert);
          c.vertices(x, y, z, v);

          for (i = 0, j = 0; i < neigh.size(); i++) {
            if (neigh[i] != id && (boundary || neigh[i] > -1)) {
              l = f_vert[j];
              n = f_vert[j];
              face_sizes.append(n);
              for (k = 0; k < n; k++) {
                l = 3 * f_vert[j + k + 1];
                verts.append(float3(v[l], v[l + 1], v[l + 2]));
                corner_verts.append(offset);
                ids.append(id);
                offset++;
                centers.append(float3(x, y, z));
              }
            }
            j += f_vert[j] + 1;
          }
        }
      while (vl.inc());
  }
  else {
    if (vl.start())
      do
        if (con.compute_cell(c, vl)) {
          vl.pos(x, y, z);
          id = vl.pid();

          c.neighbors(neigh);
          c.face_vertices(f_vert);
          c.vertices(x, y, z, v);

          for (i = 0, j = 0; i < neigh.size(); i++) {
            if (std::find(adjacency_list[id].begin(), adjacency_list[id].end(), neigh[i]) ==
                    adjacency_list[id].end() &&
                (boundary || neigh[i] > -1))
            {
              l = f_vert[j];
              n = f_vert[j];
              face_sizes.append(n);
              for (k = 0; k < n; k++) {
                l = 3 * f_vert[j + k + 1];
                verts.append(float3(v[l], v[l + 1], v[l + 2]));
                corner_verts.append(offset);
                ids.append(id);
                offset++;
                centers.append(float3(x, y, z));
              }
            }
            j += f_vert[j] + 1;
          }
        }
      while (vl.inc());
  }

  Mesh *mesh = BKE_mesh_new_nomain(verts.size(), 0, face_sizes.size(), corner_verts.size());
  mesh->vert_positions_for_write().copy_from(verts);

  MutableSpan<int> face_offs = mesh->face_offsets_for_write();
  MutableSpan<int> corns = mesh->corner_verts_for_write();

  offset = 0;
  for (i = 0; i < face_sizes.size(); i++) {
    int size = face_sizes[i];
    face_offs[i] = offset;
    for (j = 0; j < size; j++) {
      corns[offset + j] = corner_verts[offset + j];
    }
    offset += size;
  }

  MutableAttributeAccessor mesh_attributes = mesh->attributes_for_write();
  SpanAttributeWriter<int> cell_id;
  SpanAttributeWriter<float3> cell_centers;

  if (attribute_outputs.cell_id) {
    cell_id = mesh_attributes.lookup_or_add_for_write_only_span<int>(*attribute_outputs.cell_id,
                                                                     AttrDomain::Point);
    std::copy(ids.begin(), ids.end(), cell_id.span.begin());
    cell_id.finish();
  }
  if (attribute_outputs.cell_centers) {
    cell_centers = mesh_attributes.lookup_or_add_for_write_only_span<float3>(
        *attribute_outputs.cell_centers, AttrDomain::Point);
    std::copy(centers.begin(), centers.end(), cell_centers.span.begin());
    cell_centers.finish();
  }

  bke::mesh_calc_edges(*mesh, true, false);
  return mesh;
}

static Mesh *compute_voronoi_bravais(GeometrySet &sites,
                                     AttributeOutputs &attribute_outputs,
                                     const double &a,
                                     const double &bx,
                                     const double &by,
                                     const double &cx,
                                     const double &cy,
                                     const double &cz,
                                     const Field<int> &group_id,
                                     bool edge_group,
                                     bool boundary)
{
  Span<float3> positions;
  VArray<int> group_ids;
  std::unordered_map<int, std::list<int>> adjacency_list;

  if (sites.has_mesh()) {
    const Mesh *site_mesh = sites.get_mesh();
    positions = site_mesh->vert_positions();
    if (edge_group) {
      Span<int2> edges = site_mesh->edges();
      for (auto edge : edges) {
        adjacency_list[edge[0]].push_back(edge[1]);
        adjacency_list[edge[1]].push_back(edge[0]);
      }
    }

    const bke::AttrDomain att_domain = bke::AttrDomain::Point;
    const int domain_size = site_mesh->attributes().domain_size(att_domain);
    bke::MeshFieldContext field_context{*site_mesh, att_domain};
    FieldEvaluator field_evaluator{field_context, domain_size};
    field_evaluator.add(group_id);
    field_evaluator.evaluate();
    group_ids = field_evaluator.get_evaluated<int>(0);
  }
  if (sites.has_pointcloud()) {
    const PointCloud *site_pc = sites.get_pointcloud();
    positions = site_pc->positions();

    bke::PointCloudFieldContext field_context{*site_pc};
    FieldEvaluator field_evaluator{field_context, site_pc->totpoint};
    field_evaluator.add(group_id);
    field_evaluator.evaluate();
    group_ids = field_evaluator.get_evaluated<int>(0);
  }

  if (sites.has_curves()) {
    const Curves *site_curves = sites.get_curves();
    const bke::CurvesGeometry &src_curves = site_curves->geometry.wrap();
    positions = src_curves.evaluated_positions();
    // default id would have been the index which is not available for the evaluated points on the
    // curve creating a VArray with the size of the positions instead
    group_ids = VArray<int>::ForFunc(positions.size(), [](const int64_t i) { return i; });
  }

  // Set the computational grid size
  const int n_x = 3, n_y = 3, n_z = 3;

  // Create a container with the geometry given above, and make it
  // non-periodic in each of the three coordinates. Allocate space for
  // eight particles within each computational block.

  voro::container_periodic con(a, bx, by, cx, cy, cz, n_x, n_y, n_z, 8);
  int i, j, k, l, n, id;

  i = 0;
  for (float3 p : positions) {
    con.put(group_ids[i], p[0], p[1], p[2]);
    i++;
  }

  voro::voronoicell_neighbor c;
  voro::c_loop_all_periodic vl(con);

  std::vector<int> neigh, f_vert;
  std::vector<double> v;

  Vector<float3> verts;
  Vector<int> face_sizes;
  Vector<int> corner_verts;
  Vector<int> ids;
  Vector<float3> centers;
  double x, y, z;

  int offset = 0;

  if (!edge_group) {
    if (vl.start())
      do
        if (con.compute_cell(c, vl)) {
          vl.pos(x, y, z);
          id = vl.pid();

          c.neighbors(neigh);
          c.face_vertices(f_vert);
          c.vertices(x, y, z, v);

          for (i = 0, j = 0; i < neigh.size(); i++) {
            if (neigh[i] != id && (boundary || neigh[i] > -1)) {
              l = f_vert[j];
              n = f_vert[j];
              face_sizes.append(n);
              for (k = 0; k < n; k++) {
                l = 3 * f_vert[j + k + 1];
                verts.append(float3(v[l], v[l + 1], v[l + 2]));
                corner_verts.append(offset);
                ids.append(id);
                offset++;
                centers.append(float3(x, y, z));
              }
            }
            j += f_vert[j] + 1;
          }
        }
      while (vl.inc());
  }
  else {
    if (vl.start())
      do
        if (con.compute_cell(c, vl)) {
          vl.pos(x, y, z);
          id = vl.pid();

          c.neighbors(neigh);
          c.face_vertices(f_vert);
          c.vertices(x, y, z, v);

          for (i = 0, j = 0; i < neigh.size(); i++) {
            if (std::find(adjacency_list[id].begin(), adjacency_list[id].end(), neigh[i]) ==
                    adjacency_list[id].end() &&
                (boundary || neigh[i] > -1))
            {
              l = f_vert[j];
              n = f_vert[j];
              face_sizes.append(n);
              for (k = 0; k < n; k++) {
                l = 3 * f_vert[j + k + 1];
                verts.append(float3(v[l], v[l + 1], v[l + 2]));
                corner_verts.append(offset);
                ids.append(id);
                offset++;
                centers.append(float3(x, y, z));
              }
            }
            j += f_vert[j] + 1;
          }
        }
      while (vl.inc());
  }

  Mesh *mesh = BKE_mesh_new_nomain(verts.size(), 0, face_sizes.size(), corner_verts.size());
  mesh->vert_positions_for_write().copy_from(verts);

  MutableSpan<int> face_offs = mesh->face_offsets_for_write();
  MutableSpan<int> corns = mesh->corner_verts_for_write();

  offset = 0;
  for (i = 0; i < face_sizes.size(); i++) {
    int size = face_sizes[i];
    face_offs[i] = offset;
    for (j = 0; j < size; j++) {
      corns[offset + j] = corner_verts[offset + j];
    }
    offset += size;
  }

  MutableAttributeAccessor mesh_attributes = mesh->attributes_for_write();
  SpanAttributeWriter<int> cell_id;
  SpanAttributeWriter<float3> cell_centers;

  if (attribute_outputs.cell_id) {
    cell_id = mesh_attributes.lookup_or_add_for_write_only_span<int>(*attribute_outputs.cell_id,
                                                                     AttrDomain::Point);
    std::copy(ids.begin(), ids.end(), cell_id.span.begin());
    cell_id.finish();
  }
  if (attribute_outputs.cell_centers) {
    cell_centers = mesh_attributes.lookup_or_add_for_write_only_span<float3>(
        *attribute_outputs.cell_centers, AttrDomain::Point);
    std::copy(centers.begin(), centers.end(), cell_centers.span.begin());
    cell_centers.finish();
  }

  bke::mesh_calc_edges(*mesh, true, false);
  return mesh;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet site_geometry = params.extract_input<GeometrySet>("Sites");

  const NodeGeometryVoronoi &storage = node_storage(params.node());
  const GeometryNodeVoronoiMode mode = (GeometryNodeVoronoiMode)storage.mode;

  const bool boundary = params.extract_input<bool>("Boundary");
  const bool edge_group = params.extract_input<bool>("Group By Edge");
  Field<int> id_field = params.extract_input<Field<int>>("Group ID");

  // Map<int, std::unique_ptr<GeometrySet>> geometry_by_group_id;

  AttributeOutputs attribute_outputs;
  attribute_outputs.cell_id = params.get_output_anonymous_attribute_id_if_needed("Cell ID");
  attribute_outputs.cell_centers = params.get_output_anonymous_attribute_id_if_needed(
      "Cell Centers");

  switch (mode) {
    case GEO_NODE_VORONOI_BOUNDS:
      if (site_geometry.has_mesh() || site_geometry.has_pointcloud() || site_geometry.has_curves())
      {
        const float3 min = params.extract_input<float3>("Min");
        const float3 max = params.extract_input<float3>("Max");
        const bool x_p = params.extract_input<bool>("Periodic X");
        const bool y_p = params.extract_input<bool>("Periodic Y");
        const bool z_p = params.extract_input<bool>("Periodic Z");
        Mesh *voronoi = compute_voronoi_bounds(site_geometry,
                                               attribute_outputs,
                                               min,
                                               max,
                                               id_field,
                                               edge_group,
                                               x_p,
                                               y_p,
                                               z_p,
                                               boundary);
        site_geometry.replace_mesh(voronoi);
        site_geometry.keep_only_during_modify({GeometryComponent::Type::Mesh});
      }
      else {
        params.error_message_add(NodeWarningType::Error,
                                 TIP_("Input should contain one of the following to compute the "
                                      "Voronoi: mesh, point cloud, curve"));
      }
      break;
    case GEO_NODE_VORONOI_BRAVAIS:
      if (site_geometry.has_mesh() || site_geometry.has_pointcloud() || site_geometry.has_curves())
      {
        const float a = params.extract_input<float>("A");
        const float bx = params.extract_input<float>("Bx");
        const float by = params.extract_input<float>("By");
        const float cx = params.extract_input<float>("Cx");
        const float cy = params.extract_input<float>("Cy");
        const float cz = params.extract_input<float>("Cz");
        Mesh *voronoi = compute_voronoi_bravais(site_geometry,
                                                attribute_outputs,
                                                a,
                                                bx,
                                                by,
                                                cx,
                                                cy,
                                                cz,
                                                id_field,
                                                edge_group,
                                                boundary);
        site_geometry.replace_mesh(voronoi);
        site_geometry.keep_only_during_modify({GeometryComponent::Type::Mesh});
      }
      else {
        params.error_message_add(NodeWarningType::Error,
                                 TIP_("Input should contain one of the following to compute the "
                                      "Voronoi: mesh, point cloud, curve"));
      }
      break;
  }

  params.set_output("Voronoi", std::move(site_geometry));
}

static void node_rna(StructRNA *srna)
{
  static EnumPropertyItem mode_items[] = {
      {GEO_NODE_VORONOI_BOUNDS,
       "BOUNDS",
       0,
       "Bounds",
       "Use the min and max bounds for voronoi computation"},
      {GEO_NODE_VORONOI_BRAVAIS,
       "BRAVAIS",
       0,
       "Bravais",
       "Sample the specified number of points along each spline"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "mode",
                    "Mode",
                    "Defining voronoi bounds",
                    mode_items,
                    NOD_storage_enum_accessors(mode));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_VORONOI, "Voronoi", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  blender::bke::node_type_storage(
      &ntype, "NodeGeometryVoronoi", node_free_standard_storage, node_copy_standard_storage);
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_voronoi
