/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edcurves
 */

#include "ED_curves.h"
#include "ED_object.h"
#include "ED_screen.h"
#include "ED_select_utils.h"
#include "ED_view3d.h"

#include "WM_api.h"

#include "BKE_asset.h"
#include "BKE_attribute_math.hh"
#include "BKE_compute_contexts.hh"
#include "BKE_context.h"
#include "BKE_curves.hh"
#include "BKE_editmesh.h"
#include "BKE_geometry_set.hh"
#include "BKE_layer.h"
#include "BKE_lib_id.h"
#include "BKE_material.h"
#include "BKE_mesh.hh"
#include "BKE_mesh_wrapper.h"
#include "BKE_node_runtime.hh"
#include "BKE_object.h"
#include "BKE_pointcloud.h"
#include "BKE_report.h"
#include "BKE_screen.h"

#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "DEG_depsgraph.h"
#include "DEG_depsgraph_query.h"

#include "RNA_access.h"
#include "RNA_enum_types.h"

#include "UI_interface.h"
#include "UI_resources.h"

#include "ED_asset.h"
#include "ED_geometry.h"
#include "ED_mesh.h"

#include "BLT_translation.h"

#include "FN_lazy_function_execute.hh"

#include "NOD_geometry_nodes_execute.hh"
#include "NOD_geometry_nodes_lazy_function.hh"

#include "AS_asset_catalog.hh"
#include "AS_asset_catalog_path.hh"
#include "AS_asset_catalog_tree.hh"
#include "AS_asset_library.hh"
#include "AS_asset_representation.hh"

#include "geometry_intern.hh"

namespace blender::ed::geometry {

/* -------------------------------------------------------------------- */
/** \name Operator
 * \{ */

static const bNodeTree *get_node_group(const bContext &C)
{
  const asset_system::AssetRepresentation *asset = CTX_wm_asset(&C);
  if (!asset) {
    return nullptr;
  }
  bNodeTree *node_group = reinterpret_cast<bNodeTree *>(
      ED_asset_get_local_id_from_asset_or_append_and_reuse(CTX_data_main(&C), *asset, ID_NT));
  if (!node_group) {
    return nullptr;
  }
  if (node_group->type != NTREE_GEOMETRY) {
    return nullptr;
  }
  return node_group;
}

class OperatorComputeContext : public ComputeContext {
 private:
  static constexpr const char *s_static_type = "OPERATOR";

  std::string operator_name_;

 public:
  OperatorComputeContext(std::string operator_name)
      : ComputeContext(s_static_type, nullptr), operator_name_(std::move(operator_name))
  {
    hash_.mix_in(s_static_type, strlen(s_static_type));
    hash_.mix_in(operator_name_.data(), operator_name_.size());
  }

 private:
  void print_current_in_line(std::ostream &stream) const override
  {
    stream << "Operator: " << operator_name_;
  }
};

/**
 * Geometry nodes currently requires working on "evaluated" data-blocks (rather than "original"
 * data-blocks that are part of a #Main data-base). This could change in the future, but for now,
 * we need to create evaluated copies of geometry before passing it to geometry nodes. Implicit
 * sharing lets us avoid copying attribute data though.
 */
static bke::GeometrySet get_original_geometry_eval_copy(Object &object)
{
  switch (object.type) {
    case OB_CURVES: {
      Curves *curves = BKE_curves_copy_for_eval(static_cast<const Curves *>(object.data));
      return bke::GeometrySet::create_with_curves(curves);
    }
    case OB_POINTCLOUD: {
      PointCloud *points = BKE_pointcloud_copy_for_eval(
          static_cast<const PointCloud *>(object.data));
      return bke::GeometrySet::create_with_pointcloud(points);
    }
    case OB_MESH: {
      const Mesh *mesh = static_cast<const Mesh *>(object.data);
      if (mesh->edit_mesh) {
        Mesh *mesh_copy = BKE_mesh_wrapper_from_editmesh(mesh->edit_mesh, nullptr, mesh);
        BKE_mesh_wrapper_ensure_mdata(mesh_copy);
        Mesh *final_copy = BKE_mesh_copy_for_eval(mesh_copy);
        BKE_id_free(nullptr, mesh_copy);
        return bke::GeometrySet::create_with_mesh(final_copy);
      }
      return bke::GeometrySet::create_with_mesh(BKE_mesh_copy_for_eval(mesh));
    }
    default:
      return {};
  }
}

static void store_result_geometry(Main &bmain, Object &object, bke::GeometrySet geometry)
{
  switch (object.type) {
    case OB_CURVES: {
      Curves &curves = *static_cast<Curves *>(object.data);
      Curves *new_curves = geometry.get_curves_for_write();
      if (!new_curves) {
        curves.geometry.wrap() = {};
        break;
      }

      /* Anonymous attributes shouldn't be available on the applied geometry. */
      new_curves->geometry.wrap().attributes_for_write().remove_anonymous();

      curves.geometry.wrap() = std::move(new_curves->geometry.wrap());
      BKE_object_material_from_eval_data(&bmain, &object, &new_curves->id);
      break;
    }
    case OB_POINTCLOUD: {
      PointCloud &points = *static_cast<PointCloud *>(object.data);
      PointCloud *new_points =
          geometry.get_component_for_write<bke::PointCloudComponent>().release();
      if (!new_points) {
        CustomData_free(&points.pdata, points.totpoint);
        points.totpoint = 0;
        break;
      }

      /* Anonymous attributes shouldn't be available on the applied geometry. */
      new_points->attributes_for_write().remove_anonymous();

      BKE_object_material_from_eval_data(&bmain, &object, &new_points->id);
      BKE_pointcloud_nomain_to_pointcloud(new_points, &points);
      break;
    }
    case OB_MESH: {
      Mesh &mesh = *static_cast<Mesh *>(object.data);
      Mesh *new_mesh = geometry.get_component_for_write<bke::MeshComponent>().release();
      if (!new_mesh) {
        BKE_mesh_clear_geometry(&mesh);
        if (object.mode == OB_MODE_EDIT) {
          EDBM_mesh_make(&object, SCE_SELECT_VERTEX, true);
        }
        break;
      }

      /* Anonymous attributes shouldn't be available on the applied geometry. */
      new_mesh->attributes_for_write().remove_anonymous();

      BKE_object_material_from_eval_data(&bmain, &object, &new_mesh->id);
      BKE_mesh_nomain_to_mesh(new_mesh, &mesh, &object);
      if (object.mode == OB_MODE_EDIT) {
        EDBM_mesh_make(&object, SCE_SELECT_VERTEX, true);
        BKE_editmesh_looptri_and_normals_calc(mesh.edit_mesh);
      }
      break;
    }
  }
}

static int run_node_group_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  Object *active_object = CTX_data_active_object(C);
  if (!active_object) {
    return OPERATOR_CANCELLED;
  }
  if (active_object->mode == OB_MODE_OBJECT) {
    return OPERATOR_CANCELLED;
  }
  const eObjectMode mode = eObjectMode(active_object->mode);

  const bNodeTree *node_tree = get_node_group(*C);
  if (!node_tree) {
    return OPERATOR_CANCELLED;
  }

  const nodes::GeometryNodesLazyFunctionGraphInfo *lf_graph_info =
      nodes::ensure_geometry_nodes_lazy_function_graph(*node_tree);
  if (lf_graph_info == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Cannot evaluate node group");
    return OPERATOR_CANCELLED;
  }

  uint objects_len = 0;
  Object **objects = BKE_view_layer_array_from_objects_in_mode_unique_data(
      scene, view_layer, CTX_wm_view3d(C), &objects_len, mode);

  OperatorComputeContext compute_context(op->type->idname);

  for (Object *object : Span(objects, objects_len)) {
    if (!ELEM(object->type, OB_CURVES, OB_POINTCLOUD, OB_MESH)) {
      continue;
    }
    nodes::GeoNodesOperatorData operator_eval_data{};
    operator_eval_data.depsgraph = depsgraph;
    operator_eval_data.self_object = object;
    operator_eval_data.scene = scene;

    bke::GeometrySet geometry_orig = get_original_geometry_eval_copy(*object);

    bke::GeometrySet new_geometry = nodes::execute_geometry_nodes_on_geometry(
        *node_tree,
        op->properties,
        compute_context,
        std::move(geometry_orig),
        [&](nodes::GeoNodesLFUserData &user_data) {
          user_data.operator_data = &operator_eval_data;
          user_data.log_socket_values = false;
        });

    store_result_geometry(*bmain, *object, std::move(new_geometry));

    DEG_id_tag_update(static_cast<ID *>(object->data), ID_RECALC_GEOMETRY);
    WM_event_add_notifier(C, NC_GEOM | ND_DATA, object->data);
  }

  MEM_SAFE_FREE(objects);

  return OPERATOR_FINISHED;
}

static int run_node_group_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  const bNodeTree *node_tree = get_node_group(*C);
  if (!node_tree) {
    return OPERATOR_CANCELLED;
  }

  nodes::update_input_properties_from_node_tree(*node_tree, op->properties, *op->properties);
  nodes::update_output_properties_from_node_tree(*node_tree, op->properties, *op->properties);

  return run_node_group_exec(C, op);
}

static char *run_node_group_get_description(bContext *C,
                                            wmOperatorType * /*ot*/,
                                            PointerRNA * /*ptr*/)
{
  const asset_system::AssetRepresentation *asset = CTX_wm_asset(C);
  if (!asset) {
    return nullptr;
  }
  const char *description = asset->get_metadata().description;
  if (!description) {
    return nullptr;
  }
  return BLI_strdup(description);
}

static bool run_node_group_poll(bContext *C)
{
  const asset_system::AssetRepresentation *asset = CTX_wm_asset(C);
  if (!asset) {
    return false;
  }
  const Object *object = CTX_data_active_object(C);
  if (object->type != OB_CURVES) {
    return false;
  }
  if (object->mode != OB_MODE_SCULPT_CURVES) {
    return false;
  }
  return true;
}

static void add_attribute_search_or_value_buttons(const bContext &C,
                                                  uiLayout *layout,
                                                  PointerRNA *md_ptr,
                                                  const bNodeSocket &socket)
{
  char socket_id_esc[sizeof(socket.identifier) * 2];
  BLI_str_escape(socket_id_esc, socket.identifier, sizeof(socket_id_esc));
  const std::string rna_path = "[\"" + std::string(socket_id_esc) + "\"]";
  const std::string rna_path_use_attribute = "[\"" + std::string(socket_id_esc) +
                                             nodes::input_use_attribute_suffix() + "\"]";
  const std::string rna_path_attribute_name = "[\"" + std::string(socket_id_esc) +
                                              nodes::input_attribute_name_suffix() + "\"]";

  /* We're handling this manually in this case. */
  uiLayoutSetPropDecorate(layout, false);

  uiLayout *split = uiLayoutSplit(layout, 0.4f, false);
  uiLayout *name_row = uiLayoutRow(split, false);
  uiLayoutSetAlignment(name_row, UI_LAYOUT_ALIGN_RIGHT);

  const int use_attribute = RNA_int_get(md_ptr, rna_path_use_attribute.c_str()) != 0;
  if (socket.type == SOCK_BOOLEAN && !use_attribute) {
    uiItemL(name_row, "", ICON_NONE);
  }
  else {
    uiItemL(name_row, socket.name, ICON_NONE);
  }

  uiLayout *prop_row = uiLayoutRow(split, true);
  if (socket.type == SOCK_BOOLEAN) {
    uiLayoutSetPropSep(prop_row, false);
    uiLayoutSetAlignment(prop_row, UI_LAYOUT_ALIGN_EXPAND);
  }

  if (use_attribute) {
    // TODO: PROPER ATTRIBUTE SEARCH
    uiItemR(layout, md_ptr, rna_path_attribute_name.c_str(), 0, "", ICON_NONE);
    uiItemL(layout, "", ICON_BLANK1);
  }
  else {
    const char *name = socket.type == SOCK_BOOLEAN ? socket.name : "";
    uiItemR(prop_row, md_ptr, rna_path.c_str(), 0, name, ICON_NONE);
  }

  uiItemR(
      prop_row, md_ptr, rna_path_use_attribute.c_str(), UI_ITEM_R_ICON_ONLY, "", ICON_SPREADSHEET);
}

static void draw_property_for_socket(const bContext &C,
                                     const bNodeTree &node_tree,
                                     uiLayout *layout,
                                     IDProperty *op_properties,
                                     PointerRNA *bmain_ptr,
                                     PointerRNA *op_ptr,
                                     const bNodeSocket &socket,
                                     const int socket_index)
{
  /* The property should be created in #MOD_nodes_update_interface with the correct type. */
  IDProperty *property = IDP_GetPropertyFromGroup(op_properties, socket.identifier);

  /* IDProperties can be removed with python, so there could be a situation where
   * there isn't a property for a socket or it doesn't have the correct type. */
  if (property == nullptr || !nodes::id_property_type_matches_socket(socket, *property)) {
    return;
  }

  char socket_id_esc[sizeof(socket.identifier) * 2];
  BLI_str_escape(socket_id_esc, socket.identifier, sizeof(socket_id_esc));

  char rna_path[sizeof(socket_id_esc) + 4];
  SNPRINTF(rna_path, "[\"%s\"]", socket_id_esc);

  uiLayout *row = uiLayoutRow(layout, true);
  uiLayoutSetPropDecorate(row, true);

  /* Use #uiItemPointerR to draw pointer properties because #uiItemR would not have enough
   * information about what type of ID to select for editing the values. This is because
   * pointer IDProperties contain no information about their type. */
  switch (socket.type) {
    case SOCK_OBJECT:
      uiItemPointerR(row, op_ptr, rna_path, bmain_ptr, "objects", socket.name, ICON_OBJECT_DATA);
      break;
    case SOCK_COLLECTION:
      uiItemPointerR(
          row, op_ptr, rna_path, bmain_ptr, "collections", socket.name, ICON_OUTLINER_COLLECTION);
      break;
    case SOCK_MATERIAL:
      uiItemPointerR(row, op_ptr, rna_path, bmain_ptr, "materials", socket.name, ICON_MATERIAL);
      break;
    case SOCK_TEXTURE:
      uiItemPointerR(row, op_ptr, rna_path, bmain_ptr, "textures", socket.name, ICON_TEXTURE);
      break;
    case SOCK_IMAGE:
      uiItemPointerR(row, op_ptr, rna_path, bmain_ptr, "images", socket.name, ICON_IMAGE);
      break;
    default:
      if (nodes::input_has_attribute_toggle(node_tree, socket_index)) {
        add_attribute_search_or_value_buttons(C, row, op_ptr, socket);
      }
      else {
        uiItemR(row, op_ptr, rna_path, 0, socket.name, ICON_NONE);
      }
  }
  if (!nodes::input_has_attribute_toggle(node_tree, socket_index)) {
    uiItemL(row, "", ICON_BLANK1);
  }
}

static void run_node_group_ui(bContext *C, wmOperator *op)
{
  uiLayout *layout = op->layout;
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  Main *bmain = CTX_data_main(C);
  PointerRNA bmain_ptr;
  RNA_main_pointer_create(bmain, &bmain_ptr);

  // TODO: DOESN'T WORK
  const bNodeTree *node_tree = get_node_group(*C);
  if (!node_tree) {
    return;
  }

  int socket_index;
  LISTBASE_FOREACH_INDEX (bNodeSocket *, io_socket, &node_tree->inputs, socket_index) {
    std::cout << "Input identifier: " << io_socket->identifier << '\n';
    draw_property_for_socket(
        *C, *node_tree, layout, op->properties, &bmain_ptr, op->ptr, *io_socket, socket_index);
  }
  // uiDefAutoButsRNA(
  //     layout, op->ptr, nullptr, nullptr, nullptr, UI_BUT_LABEL_ALIGN_SPLIT_COLUMN, false);
}

void GEOMETRY_OT_execute_node_group(wmOperatorType *ot)
{
  ot->name = "Run Node Group";
  ot->idname = __func__;
  ot->description = "Execute a node group on geometry";

  ot->poll = run_node_group_poll;
  ot->invoke = run_node_group_invoke;
  ot->exec = run_node_group_exec;
  ot->get_description = run_node_group_get_description;
  ot->get_name = run_node_group_get_name;
  ot->ui = run_node_group_ui;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Menu
 * \{ */

static bool asset_menu_poll(const bContext *C, MenuType * /*mt*/)
{
  return CTX_wm_view3d(C);
}

static asset::AssetItemTree &get_static_item_tree()
{
  static asset::AssetItemTree tree;
  return tree;
}

static bool all_loading_finished()
{
  AssetLibraryReference all_library_ref = asset_system::all_library_reference();
  return ED_assetlist_is_loaded(&all_library_ref);
}

static asset::AssetItemTree build_catalog_tree(const bContext &C)
{
  AssetFilterSettings type_filter{};
  type_filter.id_types = FILTER_ID_NT;
  AssetTag operator_tag;
  STRNCPY(operator_tag.name, "Operator");
  BLI_addtail(&type_filter.tags, &operator_tag);
  auto meta_data_filter = [&](const AssetMetaData &meta_data) {
    const IDProperty *tree_type = BKE_asset_metadata_idprop_find(&meta_data, "type");
    if (tree_type == nullptr || IDP_Int(tree_type) != NTREE_GEOMETRY) {
      return false;
    }
    return true;
  };
  const AssetLibraryReference library = asset_system::all_library_reference();
  return asset::build_filtered_all_catalog_tree(library, C, type_filter, meta_data_filter);
}

/**
 * Avoid adding a separate root catalog when the assets have already been added to one of the
 * builtin menus. The need to define the builtin menu labels here is non-ideal. We don't have
 * any UI introspection that can do this though.
 */
static Set<std::string> get_builtin_menus(const ObjectType object_type, const eObjectMode mode)
{
  Set<std::string> menus;
  switch (object_type) {
    case OB_CURVES:
      menus.add_new("View");
      menus.add_new("Select");
      menus.add_new("Curves");
      break;
    case OB_MESH:
      switch (mode) {
        case OB_MODE_EDIT:
          menus.add_new("View");
          menus.add_new("Select");
          menus.add_new("Add");
          menus.add_new("Mesh");
          menus.add_new("Vertex");
          menus.add_new("Edge");
          menus.add_new("Face");
          menus.add_new("UV");
          break;
        case OB_MODE_SCULPT:
          menus.add_new("View");
          menus.add_new("Sculpt");
          menus.add_new("Mask");
          menus.add_new("Face Sets");
          break;
        case OB_MODE_VERTEX_PAINT:
          menus.add_new("View");
          menus.add_new("Paint");
          break;
        case OB_MODE_WEIGHT_PAINT:
          menus.add_new("View");
          menus.add_new("Weights");
          break;
        default:
          break;
      }
    default:
      break;
  }
  return menus;
}

static void node_add_catalog_assets_draw(const bContext *C, Menu *menu)
{
  bScreen &screen = *CTX_wm_screen(C);
  asset::AssetItemTree &tree = get_static_item_tree();
  const PointerRNA menu_path_ptr = CTX_data_pointer_get(C, "asset_catalog_path");
  if (RNA_pointer_is_null(&menu_path_ptr)) {
    return;
  }
  const auto &menu_path = *static_cast<const asset_system::AssetCatalogPath *>(menu_path_ptr.data);
  const Span<asset_system::AssetRepresentation *> assets = tree.assets_per_path.lookup(menu_path);
  asset_system::AssetCatalogTreeItem *catalog_item = tree.catalogs.find_item(menu_path);
  BLI_assert(catalog_item != nullptr);

  if (assets.is_empty() && !catalog_item->has_children()) {
    return;
  }

  uiLayout *layout = menu->layout;
  uiItemS(layout);

  for (const asset_system::AssetRepresentation *asset : assets) {
    uiLayout *col = uiLayoutColumn(layout, false);
    PointerRNA asset_ptr = asset::create_asset_rna_ptr(asset);
    uiLayoutSetContextPointer(col, "asset", &asset_ptr);
    uiItemO(col, IFACE_(asset->get_name().c_str()), ICON_NONE, "GEOMETRY_OT_execute_node_group");
  }

  asset_system::AssetLibrary *all_library = ED_assetlist_library_get_once_available(
      asset_system::all_library_reference());
  if (!all_library) {
    return;
  }

  catalog_item->foreach_child([&](asset_system::AssetCatalogTreeItem &child_item) {
    PointerRNA path_ptr = asset::persistent_catalog_path_rna_pointer(
        screen, *all_library, child_item);
    if (path_ptr.data == nullptr) {
      return;
    }
    uiLayout *col = uiLayoutColumn(layout, false);
    uiLayoutSetContextPointer(col, "asset_catalog_path", &path_ptr);
    uiItemM(col,
            "GEO_MT_node_operator_catalog_assets",
            IFACE_(child_item.get_name().c_str()),
            ICON_NONE);
  });
}

MenuType node_group_operator_assets_menu()
{
  MenuType type{};
  STRNCPY(type.idname, "GEO_MT_node_operator_catalog_assets");
  type.poll = asset_menu_poll;
  type.draw = node_add_catalog_assets_draw;
  type.listener = asset::asset_reading_region_listen_fn;
  return type;
}

void ui_template_node_operator_asset_menu_items(uiLayout &layout,
                                                bContext &C,
                                                const StringRef catalog_path)
{
  bScreen &screen = *CTX_wm_screen(&C);
  asset::AssetItemTree &tree = get_static_item_tree();
  const asset_system::AssetCatalogTreeItem *item = tree.catalogs.find_root_item(catalog_path);
  if (!item) {
    return;
  }
  asset_system::AssetLibrary *all_library = ED_assetlist_library_get_once_available(
      asset_system::all_library_reference());
  if (!all_library) {
    return;
  }
  PointerRNA path_ptr = asset::persistent_catalog_path_rna_pointer(screen, *all_library, *item);
  if (path_ptr.data == nullptr) {
    return;
  }
  uiItemS(&layout);
  uiLayout *col = uiLayoutColumn(&layout, false);
  uiLayoutSetContextPointer(col, "asset_catalog_path", &path_ptr);
  uiItemMContents(col, "GEO_MT_node_operator_catalog_assets");
}

void ui_template_node_operator_asset_root_items(uiLayout &layout, bContext &C)
{
  bScreen &screen = *CTX_wm_screen(&C);
  const Object *active_object = CTX_data_active_object(&C);
  if (!active_object) {
    return;
  }
  asset::AssetItemTree &tree = get_static_item_tree();
  tree = build_catalog_tree(C);

  const bool loading_finished = all_loading_finished();
  if (tree.catalogs.is_empty() && loading_finished) {
    return;
  }
  if (!loading_finished) {
    uiItemL(&layout, IFACE_("Loading Asset Libraries"), ICON_INFO);
  }

  asset_system::AssetLibrary *all_library = ED_assetlist_library_get_once_available(
      asset_system::all_library_reference());
  if (!all_library) {
    return;
  }

  const Set<std::string> builtin_menus = get_builtin_menus(ObjectType(active_object->type),
                                                           eObjectMode(active_object->mode));

  tree.catalogs.foreach_root_item([&](asset_system::AssetCatalogTreeItem &item) {
    if (builtin_menus.contains(item.get_name())) {
      return;
    }
    PointerRNA path_ptr = asset::persistent_catalog_path_rna_pointer(screen, *all_library, item);
    if (path_ptr.data == nullptr) {
      return;
    }
    uiLayout *col = uiLayoutColumn(&layout, false);
    uiLayoutSetContextPointer(col, "asset_catalog_path", &path_ptr);
    const char *text = IFACE_(item.get_name().c_str());
    uiItemM(col, "GEO_MT_node_operator_catalog_assets", text, ICON_NONE);
  });
}

/** \} */

}  // namespace blender::ed::geometry
