/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "WM_api.h"
#include "WM_types.h"

#include "ED_object.h"
#include "ED_screen.h"

#include "RNA_access.h"
#include "RNA_define.h"

#include "DNA_modifier_types.h"

#include "MOD_nodes.hh"

#include "BKE_bake_geometry_nodes.hh"
#include "BKE_bake_items_serialize.hh"
#include "BKE_context.h"
#include "BKE_main.h"
#include "BKE_modifier.h"
#include "BKE_scene.h"

#include "DEG_depsgraph.h"

#include "BLI_fileops.hh"
#include "BLI_path_util.h"
#include "BLI_string_utils.h"

#include "object_intern.h"

namespace blender::ed::object::bake_geometry_nodes {

static void bake_operator_props(wmOperatorType *ot)
{
  PropertyRNA *prop;

  WM_operator_properties_id_lookup(ot, false);

  prop = RNA_def_string(
      ot->srna, "modifier", nullptr, MAX_NAME, "Modifier", "Name of the modifier");
  RNA_def_property_flag(prop, PROP_HIDDEN);

  prop = RNA_def_int(ot->srna, "bake_id", 0, 0, INT32_MAX, "Bake ID", "", 0, INT32_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);
}

[[nodiscard]] static bool find_edit_bake(Main *bmain,
                                         wmOperator *op,
                                         Object **r_object,
                                         NodesModifierData **r_modifier,
                                         NodesModifierBake **r_bake)
{
  Object *object = reinterpret_cast<Object *>(
      WM_operator_properties_id_lookup_from_name_or_session_uuid(bmain, op->ptr, ID_OB));
  if (object == nullptr) {
    return false;
  }
  char modifier_name[MAX_NAME];
  RNA_string_get(op->ptr, "modifier", modifier_name);
  ModifierData *md = BKE_modifiers_findby_name(object, modifier_name);
  if (md == nullptr || md->type != eModifierType_Nodes) {
    return false;
  }
  NodesModifierData *nmd = reinterpret_cast<NodesModifierData *>(md);
  const int bake_id = RNA_int_get(op->ptr, "bake_id");
  NodesModifierBake *bake = nullptr;
  for (NodesModifierBake &bake_iter : MutableSpan(nmd->bakes, nmd->bakes_num)) {
    if (bake_iter.id == bake_id) {
      bake = &bake_iter;
    }
  }
  if (bake == nullptr) {
    return false;
  }

  *r_object = object;
  *r_modifier = nmd;
  *r_bake = bake;
  return true;
}

static int geometry_node_bake_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);

  Object *object;
  NodesModifierData *modifier;
  NodesModifierBake *bake;
  if (!find_edit_bake(bmain, op, &object, &modifier, &bake)) {
    return OPERATOR_CANCELLED;
  }
  NodesModifierData &nmd = *modifier;
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);

  if (!nmd.runtime->bakes) {
    nmd.runtime->bakes = std::make_shared<bke::GeometryNodesModifierBakes>();
  }

  bke::GeometryNodesModifierBakes &bakes = *nmd.runtime->bakes;
  bke::BakeNodeStorage &bake_storage = *bakes.storage_by_id.lookup_or_add_cb(
      bake->id, []() { return std::make_unique<bke::BakeNodeStorage>(); });
  bake_storage.states.clear();

  switch (NodesModifierBakeType(bake->bake_type)) {
    case NODES_MODIFIER_BAKE_TYPE_STILL: {
      bake_storage.current_bake_state = std::make_unique<bke::BakeNodeState>();

      DEG_id_tag_update(&object->id, ID_RECALC_GEOMETRY);
      BKE_scene_graph_update_tagged(depsgraph, bmain);

      bake_storage.states.append({0, std::move(bake_storage.current_bake_state)});
      break;
    }
    case NODES_MODIFIER_BAKE_TYPE_ANIMATED: {
      for (int frame_i = bake->frame_start; frame_i <= bake->frame_end; frame_i++) {
        const SubFrame frame{frame_i};
        bake_storage.current_bake_state = std::make_unique<bke::BakeNodeState>();
        DEG_id_tag_update(&object->id, ID_RECALC_GEOMETRY);

        scene->r.cfra = frame.frame();
        scene->r.subframe = frame.subframe();

        BKE_scene_graph_update_for_newframe(depsgraph);

        bake_storage.states.append({frame, std::move(bake_storage.current_bake_state)});
      }
      break;
    }
  }

  if (!StringRef(bake->directory).is_empty()) {
    bke::BDataSharing bdata_sharing;

    const char *base_path = ID_BLEND_PATH(bmain, &object->id);
    char absolute_bake_dir[FILE_MAX];
    STRNCPY(absolute_bake_dir, bake->directory);
    BLI_path_abs(absolute_bake_dir, base_path);

    for (const bke::BakeNodeStateAtFrame &state_at_frame : bake_storage.states) {
      const SubFrame frame = state_at_frame.frame;

      char frame_file_c_str[64];
      SNPRINTF(frame_file_c_str, "%011.5f", double(frame));
      BLI_string_replace_char(frame_file_c_str, '.', '_');
      const StringRefNull frame_file_str = frame_file_c_str;

      const std::string bdata_file_name = frame_file_str + ".bdata";
      const std::string meta_file_name = frame_file_str + ".json";

      char bdata_path[FILE_MAX];
      BLI_path_join(
          bdata_path, sizeof(bdata_path), absolute_bake_dir, "bdata", bdata_file_name.c_str());
      char meta_path[FILE_MAX];
      BLI_path_join(
          meta_path, sizeof(meta_path), absolute_bake_dir, "meta", meta_file_name.c_str());

      BLI_file_ensure_parent_dir_exists(bdata_path);
      fstream bdata_file{bdata_path, std::ios::out | std::ios::binary};
      bke::DiskBDataWriter bdata_writer{bdata_file_name, bdata_file, 0};

      io::serialize::DictionaryValue io_root;
      io_root.append_int("version", 1);
      std::shared_ptr<io::serialize::ArrayValue> io_bake_items = io_root.append_array(
          "bake_items");
      for (const auto item : state_at_frame.state->item_by_identifier.items()) {
        const int id = item.key;
        auto io_bake_item = io_bake_items->append_dict();
        io_bake_item->append_int("id", id);
        bke::serialize_bake_item(*item.value, bdata_writer, bdata_sharing, *io_bake_item);
      }

      BLI_file_ensure_parent_dir_exists(meta_path);
      io::serialize::write_json_file(meta_path, io_root);
    }
  }

  WM_main_add_notifier(NC_OBJECT | ND_MODIFIER, nullptr);

  return OPERATOR_CANCELLED;
}

static int geometry_node_bake_delete_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);

  Object *object;
  NodesModifierData *modifier;
  NodesModifierBake *bake;
  if (!find_edit_bake(bmain, op, &object, &modifier, &bake)) {
    return OPERATOR_CANCELLED;
  }
  NodesModifierData &nmd = *modifier;

  if (!nmd.runtime->bakes) {
    return OPERATOR_FINISHED;
  }

  nmd.runtime->bakes->storage_by_id.remove(bake->id);

  DEG_id_tag_update(&object->id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_OBJECT | ND_MODIFIER, nullptr);
  return OPERATOR_FINISHED;
}

static int geometry_node_bake_delete_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  if (edit_modifier_invoke_properties(C, op)) {
    return geometry_node_bake_delete_exec(C, op);
  }
  return OPERATOR_CANCELLED;
}

}  // namespace blender::ed::object::bake_geometry_nodes

void OBJECT_OT_geometry_node_bake(wmOperatorType *ot)
{
  using namespace blender::ed::object::bake_geometry_nodes;

  ot->name = "Bake Geometry Node";
  ot->description = "Bake geometry in a Bake node in geometry nodes";
  ot->idname = __func__;

  ot->exec = geometry_node_bake_exec;
  ot->poll = ED_operator_object_active_editable;

  bake_operator_props(ot);
}

void OBJECT_OT_geometry_node_bake_delete(wmOperatorType *ot)
{
  using namespace blender::ed::object::bake_geometry_nodes;

  ot->name = "Delete Geometry Node Bake";
  ot->description = "Delete baked data";
  ot->idname = __func__;

  ot->invoke = geometry_node_bake_delete_invoke;
  ot->exec = geometry_node_bake_delete_exec;
  ot->poll = ED_operator_object_active_editable;

  bake_operator_props(ot);
}
