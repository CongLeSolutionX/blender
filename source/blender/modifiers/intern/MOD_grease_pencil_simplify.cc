/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "BLI_index_mask.hh"

#include "BLT_translation.hh"

#include "BLO_read_write.hh"

#include "DNA_defaults.h"
#include "DNA_modifier_types.h"
#include "DNA_screen_types.h"

#include "BKE_geometry_set.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_modifier.hh"
#include "BKE_geometry_fields.hh"

#include "GEO_resample_curves.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "MOD_grease_pencil_util.hh"
#include "MOD_ui_common.hh"

#include "RNA_prototypes.h"

namespace blender {

static void init_data(ModifierData *md)
{
  auto *gpmd = reinterpret_cast<GreasePencilSimplifyModifierData *>(md);

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(gpmd, modifier));

  MEMCPY_STRUCT_AFTER(gpmd, DNA_struct_default_get(GreasePencilSimplifyModifierData), modifier);
  modifier::greasepencil::init_influence_data(&gpmd->influence, true);
}

static void free_data(ModifierData *md)
{
  auto *mmd = reinterpret_cast<GreasePencilSimplifyModifierData *>(md);

  modifier::greasepencil::free_influence_data(&mmd->influence);
}

static void copy_data(const ModifierData *md, ModifierData *target, int flag)
{
  const auto *gmd = reinterpret_cast<const GreasePencilSimplifyModifierData *>(md);
  auto *tgmd = reinterpret_cast<GreasePencilSimplifyModifierData *>(target);

  BKE_modifier_copydata_generic(md, target, flag);
  modifier::greasepencil::copy_influence_data(&gmd->influence, &tgmd->influence, flag);
}

static void blend_write(BlendWriter *writer, const ID * /*id_owner*/, const ModifierData *md)
{
  const auto *mmd = reinterpret_cast<const GreasePencilSimplifyModifierData *>(md);

  BLO_write_struct(writer, GreasePencilSimplifyModifierData, mmd);
  modifier::greasepencil::write_influence_data(writer, &mmd->influence);
}

static void blend_read(BlendDataReader *reader, ModifierData *md)
{
  auto *mmd = reinterpret_cast<GreasePencilSimplifyModifierData *>(md);

  modifier::greasepencil::read_influence_data(reader, &mmd->influence);
}

static void subdivide_drawing(GreasePencilSimplifyModifierData &mmd,
                              Object &ob,
                              bke::greasepencil::Drawing &drawing)
{
  IndexMaskMemory memory;
  const IndexMask strokes = modifier::greasepencil::get_filtered_stroke_mask(
      &ob, drawing.strokes_for_write(), mmd.influence, memory);

  //bke::GreasePencilLayerFieldContext field_context(
  //    grease_pencil, AttrDomain::Curve, layer_index);
//
  //fn::IndexFieldInput

  switch (mmd.mode) {
    case MOD_GREASE_PENCIL_SIMPLIFY_FIXED: {
      for (int i = 0; i < mmd.step; i++) {

        //BKE_gpencil_stroke_simplify_fixed(gpd, gps);
      }
      break;
    }
    case MOD_GREASE_PENCIL_SIMPLIFY_ADAPTIVE: {
      /* simplify stroke using Ramer-Douglas-Peucker algorithm */
      //BKE_gpencil_stroke_simplify_adaptive(gpd, gps, mmd->factor);
      break;
    }
    case MOD_GREASE_PENCIL_SIMPLIFY_SAMPLE: {
      //geometry::resample_to_length(drawing.strokes(),field_context,
      //BKE_gpencil_stroke_sample(gpd, gps, mmd->length, false, mmd->sharp_threshold);
      break;
    }
    case MOD_GREASE_PENCIL_SIMPLIFY_MERGE: {
      //BKE_gpencil_stroke_merge_distance(gpd, gpf, gps, mmd->distance, true);
      break;
    }
    default:
      break;
  }

  drawing.tag_topology_changed();
}

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                bke::GeometrySet *geometry_set)
{
  auto *mmd = reinterpret_cast<GreasePencilSimplifyModifierData *>(md);

  if (!geometry_set->has_grease_pencil()) {
    return;
  }

  GreasePencil &grease_pencil = *geometry_set->get_grease_pencil_for_write();
  const int current_frame = grease_pencil.runtime->eval_frame;

  IndexMaskMemory mask_memory;
  const IndexMask layer_mask = modifier::greasepencil::get_filtered_layer_mask(
      grease_pencil, mmd->influence, mask_memory);
  const Vector<bke::greasepencil::Drawing *> drawings =
      modifier::greasepencil::get_drawings_for_write(grease_pencil, layer_mask, current_frame);

  threading::parallel_for_each(drawings, [&](bke::greasepencil::Drawing *drawing) {
    subdivide_drawing(*mmd, *ctx->object, *drawing);
  });
}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  auto *mmd = reinterpret_cast<GreasePencilSimplifyModifierData *>(md);

  modifier::greasepencil::foreach_influence_ID_link(&mmd->influence, ob, walk, user_data);
}

static void panel_draw(const bContext *C, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, nullptr);

  uiLayoutSetPropSep(layout, true);

  uiItemR(layout, ptr, "subdivision_type", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "level", UI_ITEM_NONE, IFACE_("Subdivisions"), ICON_NONE);

  if (uiLayout *influence_panel = uiLayoutPanelProp(
          C, layout, ptr, "open_influence_panel", "Influence"))
  {
    modifier::greasepencil::draw_layer_filter_settings(C, influence_panel, ptr);
    modifier::greasepencil::draw_material_filter_settings(C, influence_panel, ptr);
    modifier::greasepencil::draw_vertex_group_settings(C, influence_panel, ptr);
  }

  modifier_panel_end(layout, ptr);
}

static void panel_register(ARegionType *region_type)
{
  modifier_panel_register(region_type, eModifierType_GreasePencilSimplify, panel_draw);
}

}  // namespace blender

ModifierTypeInfo modifierType_GreasePencilSimplify = {
    /*idname*/ "GreasePencilSimplifyModifier",
    /*name*/ N_("Simplify"),
    /*struct_name*/ "GreasePencilSimplifyModifierData",
    /*struct_size*/ sizeof(GreasePencilSimplifyModifierData),
    /*srna*/ &RNA_GreasePencilSimplifyModifier,
    /*type*/ ModifierTypeType::Nonconstructive,
    /*flags*/
    eModifierTypeFlag_AcceptsGreasePencil | eModifierTypeFlag_SupportsEditmode |
        eModifierTypeFlag_EnableInEditmode,
    /*icon*/ ICON_MOD_SIMPLIFY,

    /*copy_data*/ blender::copy_data,

    /*deform_verts*/ nullptr,
    /*deform_matrices*/ nullptr,
    /*deform_verts_EM*/ nullptr,
    /*deform_matrices_EM*/ nullptr,
    /*modify_mesh*/ nullptr,
    /*modify_geometry_set*/ blender::modify_geometry_set,

    /*init_data*/ blender::init_data,
    /*required_data_mask*/ nullptr,
    /*free_data*/ blender::free_data,
    /*is_disabled*/ nullptr,
    /*update_depsgraph*/ nullptr,
    /*depends_on_time*/ nullptr,
    /*depends_on_normals*/ nullptr,
    /*foreach_ID_link*/ blender::foreach_ID_link,
    /*foreach_tex_link*/ nullptr,
    /*free_runtime_data*/ nullptr,
    /*panel_register*/ blender::panel_register,
    /*blend_write*/ blender::blend_write,
    /*blend_read*/ blender::blend_read,
    /*foreach_cache*/ nullptr,
};
