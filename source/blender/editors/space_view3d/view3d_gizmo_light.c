/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spview3d
 */

#include "BLI_math.h"
#include "BLI_math_base_safe.h"
#include "BLI_utildefines.h"

#include "BKE_context.h"
#include "BKE_layer.h"
#include "BKE_object.h"

#include "DEG_depsgraph.h"

#include "DNA_light_types.h"
#include "DNA_object_types.h"

#include "ED_gizmo_library.h"
#include "ED_screen.h"

#include "UI_resources.h"

#include "MEM_guardedalloc.h"

#include "RNA_access.h"
#include "RNA_prototypes.h"

#include "WM_api.h"
#include "WM_types.h"

#include "view3d_intern.h" /* own include */

/* -------------------------------------------------------------------- */
/** \name Spot Light Gizmos
 * \{ */
/* NOTE: scaling from `overlay_extra.cc`. */
#define CONE_SCALE 10.0f
#define INV_CONE_SCALE 0.1f

typedef struct LightSpotWidgetGroup {
  wmGizmo *spot_angle;
  wmGizmo *spot_blend;
  wmGizmo *spot_radius;
} LightSpotWidgetGroup;

static void gizmo_spot_blend_prop_size_get(wmGizmoProperty *gz_prop, void *value)
{
  float *blend = value;

  Light *la = gz_prop->ptr.data;

  float a = cosf(la->spotsize * 0.5f);
  float b = la->spotblend;
  /* Cosine of the angle where spot attenuation == 1. */
  float c = (1.0f - a) * b + a;
  /* Tangent. */
  float t = sqrtf(1.0f - c * c) / c;

  *blend = 2.0f * CONE_SCALE * t * a;
}

static void gizmo_spot_blend_prop_size_set(wmGizmoProperty *gz_prop, void *value)
{
  float *blend = value;

  Light *la = gz_prop->ptr.data;

  float a = cosf(la->spotsize * 0.5f);
  float t = *blend * 0.5f * INV_CONE_SCALE / a;
  float c = 1.0f / sqrt(t * t + 1.0f);

  *blend = safe_divide(clamp_f(c - a, 0.0f, 1.0f - a), 1.0f - a);
}

/* Used by spot light and point light. */
static void gizmo_light_radius_prop_size_get(wmGizmoProperty *UNUSED(gz_prop), void *radius)
{
  float *diameter = radius;
  *diameter = 2.0f * (*(float *)radius);
}

static void gizmo_light_radius_prop_size_set(wmGizmoProperty *UNUSED(gz_prop), void *diameter)
{
  float *radius = diameter;
  *radius = 0.5f * (*(float *)diameter);
}

static bool WIDGETGROUP_light_spot_poll(const bContext *C, wmGizmoGroupType *UNUSED(gzgt))
{
  View3D *v3d = CTX_wm_view3d(C);
  if (v3d->gizmo_flag & (V3D_GIZMO_HIDE | V3D_GIZMO_HIDE_CONTEXT)) {
    return false;
  }
  if ((v3d->gizmo_show_light & V3D_GIZMO_SHOW_LIGHT_SIZE) == 0) {
    return false;
  }

  const Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  BKE_view_layer_synced_ensure(scene, view_layer);
  Base *base = BKE_view_layer_active_base_get(view_layer);
  if (base && BASE_SELECTABLE(v3d, base)) {
    Object *ob = base->object;
    if (ob->type == OB_LAMP) {
      Light *la = ob->data;
      return (la->type == LA_SPOT);
    }
  }
  return false;
}

static void WIDGETGROUP_light_spot_setup(const bContext *UNUSED(C), wmGizmoGroup *gzgroup)
{
  LightSpotWidgetGroup *ls_gzgroup = MEM_mallocN(sizeof(LightSpotWidgetGroup), __func__);

  gzgroup->customdata = ls_gzgroup;

  /* Spot angle gizmo. */
  {
    ls_gzgroup->spot_angle = WM_gizmo_new("GIZMO_GT_arrow_3d", gzgroup, NULL);
    wmGizmo *gz = ls_gzgroup->spot_angle;
    RNA_enum_set(gz->ptr, "transform", ED_GIZMO_ARROW_XFORM_FLAG_INVERTED);
    ED_gizmo_arrow3d_set_range_fac(gz, 4.0f);
    UI_GetThemeColor3fv(TH_GIZMO_SECONDARY, gz->color);

    WM_gizmo_enable_undo(gz, "Adjust");
  }

  /* Spot blend gizmo. */
  {
    ls_gzgroup->spot_blend = WM_gizmo_new("GIZMO_GT_cage_2d", gzgroup, NULL);
    wmGizmo *gz = ls_gzgroup->spot_blend;
    RNA_enum_set(gz->ptr,
                 "transform",
                 ED_GIZMO_CAGE_XFORM_FLAG_SCALE | ED_GIZMO_CAGE_XFORM_FLAG_SCALE_UNIFORM);
    RNA_enum_set(gz->ptr, "draw_style", ED_GIZMO_CAGE2D_STYLE_CIRCLE);
    WM_gizmo_set_flag(gz, WM_GIZMO_DRAW_HOVER, true);
    UI_GetThemeColor3fv(TH_GIZMO_PRIMARY, gz->color);
    UI_GetThemeColor3fv(TH_GIZMO_HI, gz->color_hi);

    WM_gizmo_enable_undo(gz, "Adjust");
  }

  /* Spot radius gizmo. */
  {
    ls_gzgroup->spot_radius = WM_gizmo_new("GIZMO_GT_cage_2d", gzgroup, NULL);
    wmGizmo *gz = ls_gzgroup->spot_radius;
    RNA_enum_set(gz->ptr,
                 "transform",
                 ED_GIZMO_CAGE_XFORM_FLAG_SCALE | ED_GIZMO_CAGE_XFORM_FLAG_SCALE_UNIFORM);
    RNA_enum_set(gz->ptr, "draw_style", ED_GIZMO_CAGE2D_STYLE_CIRCLE);
    WM_gizmo_set_flag(gz, WM_GIZMO_DRAW_HOVER, true);
    UI_GetThemeColor3fv(TH_GIZMO_PRIMARY, gz->color);
    UI_GetThemeColor3fv(TH_GIZMO_HI, gz->color_hi);

    WM_gizmo_enable_undo(gz, "Adjust");
  }
}

static void WIDGETGROUP_light_spot_refresh(const bContext *C, wmGizmoGroup *gzgroup)
{
  LightSpotWidgetGroup *ls_gzgroup = gzgroup->customdata;
  const Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  BKE_view_layer_synced_ensure(scene, view_layer);
  Object *ob = BKE_view_layer_active_object_get(view_layer);
  Light *la = ob->data;

  PointerRNA lamp_ptr;
  RNA_pointer_create(&la->id, &RNA_Light, la, &lamp_ptr);

  /* Need to set property here for `lamp_ptr`. TODO: would prefer to do this in _init. */

  /* Spot angle gizmo. */
  {
    wmGizmo *gz = ls_gzgroup->spot_angle;
    float dir[3];
    negate_v3_v3(dir, ob->object_to_world[2]);
    WM_gizmo_set_matrix_rotation_from_z_axis(gz, dir);
    WM_gizmo_set_matrix_location(gz, ob->object_to_world[3]);

    const char *propname = "spot_size";
    WM_gizmo_target_property_def_rna(gz, "offset", &lamp_ptr, propname, -1);
  }

  /* Spot blend gizmo. */
  {
    wmGizmo *gz = ls_gzgroup->spot_blend;

    copy_m4_m4(gz->matrix_basis, ob->object_to_world);

    /* Move center to the cone base plane. */
    float dir[3];
    negate_v3_v3(dir, ob->object_to_world[2]);
    mul_v3_fl(dir, CONE_SCALE * cosf(0.5f * la->spotsize));
    add_v3_v3(gz->matrix_basis[3], dir);

    WM_gizmo_target_property_def_rna_func(gz,
                                          "size",
                                          &lamp_ptr,
                                          "spot_blend",
                                          -1,
                                          &(const struct wmGizmoPropertyFnParams){
                                              .transform_get_fn = gizmo_spot_blend_prop_size_get,
                                              .transform_set_fn = gizmo_spot_blend_prop_size_set,
                                          });
  }

  /* Spot radius gizmo. */
  {
    wmGizmo *gz = ls_gzgroup->spot_radius;
    WM_gizmo_target_property_def_rna_func(gz,
                                          "size",
                                          &lamp_ptr,
                                          "shadow_soft_size",
                                          -1,
                                          &(const struct wmGizmoPropertyFnParams){
                                              .transform_get_fn = gizmo_light_radius_prop_size_get,
                                              .transform_set_fn = gizmo_light_radius_prop_size_set,
                                          });
  }
}

static void WIDGETGROUP_light_spot_draw_prepare(const bContext *C, wmGizmoGroup *gzgroup)
{
  LightSpotWidgetGroup *ls_gzgroup = gzgroup->customdata;
  ViewLayer *view_layer = CTX_data_view_layer(C);
  BKE_view_layer_synced_ensure(CTX_data_scene(C), view_layer);
  Object *ob = BKE_view_layer_active_object_get(view_layer);

  /* Spot radius gizmo. */
  wmGizmo *gz = ls_gzgroup->spot_radius;

  /* Draw circle in the screen space. */
  RegionView3D *rv3d = CTX_wm_region(C)->regiondata;
  WM_gizmo_set_matrix_rotation_from_z_axis(gz, rv3d->viewinv[2]);

  WM_gizmo_set_matrix_location(gz, ob->object_to_world[3]);
}

void VIEW3D_GGT_light_spot(wmGizmoGroupType *gzgt)
{
  gzgt->name = "Spot Light Widgets";
  gzgt->idname = "VIEW3D_GGT_light_spot";

  gzgt->flag |= (WM_GIZMOGROUPTYPE_PERSISTENT | WM_GIZMOGROUPTYPE_3D | WM_GIZMOGROUPTYPE_DEPTH_3D);

  gzgt->poll = WIDGETGROUP_light_spot_poll;
  gzgt->setup = WIDGETGROUP_light_spot_setup;
  gzgt->setup_keymap = WM_gizmogroup_setup_keymap_generic_maybe_drag;
  gzgt->refresh = WIDGETGROUP_light_spot_refresh;
  gzgt->draw_prepare = WIDGETGROUP_light_spot_draw_prepare;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Point Light Gizmo
 * \{ */

static bool WIDGETGROUP_light_point_poll(const bContext *C, wmGizmoGroupType *UNUSED(gzgt))
{
  const View3D *v3d = CTX_wm_view3d(C);
  if (v3d->gizmo_flag & (V3D_GIZMO_HIDE | V3D_GIZMO_HIDE_CONTEXT)) {
    return false;
  }
  if ((v3d->gizmo_show_light & V3D_GIZMO_SHOW_LIGHT_SIZE) == 0) {
    return false;
  }

  const Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  BKE_view_layer_synced_ensure(scene, view_layer);
  const Base *base = BKE_view_layer_active_base_get(view_layer);
  if (base && BASE_SELECTABLE(v3d, base)) {
    const Object *ob = base->object;
    if (ob->type == OB_LAMP) {
      const Light *la = ob->data;
      return (la->type == LA_LOCAL);
    }
  }
  return false;
}

static void WIDGETGROUP_light_point_setup(const bContext *UNUSED(C), wmGizmoGroup *gzgroup)
{
  wmGizmoWrapper *wwrapper = MEM_mallocN(sizeof(wmGizmoWrapper), __func__);
  wwrapper->gizmo = WM_gizmo_new("GIZMO_GT_cage_2d", gzgroup, NULL);
  /* Point radius gizmo. */
  wmGizmo *gz = wwrapper->gizmo;
  gzgroup->customdata = wwrapper;

  RNA_enum_set(gz->ptr,
               "transform",
               ED_GIZMO_CAGE_XFORM_FLAG_SCALE | ED_GIZMO_CAGE_XFORM_FLAG_SCALE_UNIFORM);
  RNA_enum_set(gz->ptr, "draw_style", ED_GIZMO_CAGE2D_STYLE_CIRCLE);
  WM_gizmo_set_flag(gz, WM_GIZMO_DRAW_HOVER, true);
  UI_GetThemeColor3fv(TH_GIZMO_PRIMARY, gz->color);
  UI_GetThemeColor3fv(TH_GIZMO_HI, gz->color_hi);

  WM_gizmo_enable_undo(gz, "Adjust");
}

static void WIDGETGROUP_light_point_draw_prepare(const bContext *C, wmGizmoGroup *gzgroup)
{
  wmGizmoWrapper *wwrapper = gzgroup->customdata;
  ViewLayer *view_layer = CTX_data_view_layer(C);
  BKE_view_layer_synced_ensure(CTX_data_scene(C), view_layer);
  const Object *ob = BKE_view_layer_active_object_get(view_layer);

  /* Point radius gizmo. */
  wmGizmo *gz = wwrapper->gizmo;

  /* Draw circle in the screen space. */
  const RegionView3D *rv3d = CTX_wm_region(C)->regiondata;
  WM_gizmo_set_matrix_rotation_from_z_axis(gz, rv3d->viewinv[2]);

  WM_gizmo_set_matrix_location(gz, ob->object_to_world[3]);
}

static void WIDGETGROUP_light_point_refresh(const bContext *C, wmGizmoGroup *gzgroup)
{
  wmGizmoWrapper *wwrapper = gzgroup->customdata;

  ViewLayer *view_layer = CTX_data_view_layer(C);
  BKE_view_layer_synced_ensure(CTX_data_scene(C), view_layer);
  Light *la = BKE_view_layer_active_object_get(view_layer)->data;

  PointerRNA lamp_ptr;
  RNA_pointer_create(&la->id, &RNA_Light, la, &lamp_ptr);

  /* Need to set property here for `lamp_ptr`. TODO: would prefer to do this in _init. */
  WM_gizmo_target_property_def_rna_func(wwrapper->gizmo,
                                        "size",
                                        &lamp_ptr,
                                        "shadow_soft_size",
                                        -1,
                                        &(const struct wmGizmoPropertyFnParams){
                                            .transform_get_fn = gizmo_light_radius_prop_size_get,
                                            .transform_set_fn = gizmo_light_radius_prop_size_set,
                                        });
}

void VIEW3D_GGT_light_point(wmGizmoGroupType *gzgt)
{
  gzgt->name = "Point Light Widgets";
  gzgt->idname = "VIEW3D_GGT_light_point";

  gzgt->flag |= (WM_GIZMOGROUPTYPE_PERSISTENT | WM_GIZMOGROUPTYPE_3D | WM_GIZMOGROUPTYPE_DEPTH_3D);

  gzgt->poll = WIDGETGROUP_light_point_poll;
  gzgt->setup = WIDGETGROUP_light_point_setup;
  gzgt->setup_keymap = WM_gizmogroup_setup_keymap_generic_maybe_drag;
  gzgt->refresh = WIDGETGROUP_light_point_refresh;
  gzgt->draw_prepare = WIDGETGROUP_light_point_draw_prepare;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Area Light Gizmos
 * \{ */

static bool WIDGETGROUP_light_area_poll(const bContext *C, wmGizmoGroupType *UNUSED(gzgt))
{
  View3D *v3d = CTX_wm_view3d(C);
  if (v3d->gizmo_flag & (V3D_GIZMO_HIDE | V3D_GIZMO_HIDE_CONTEXT)) {
    return false;
  }
  if ((v3d->gizmo_show_light & V3D_GIZMO_SHOW_LIGHT_SIZE) == 0) {
    return false;
  }

  const Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  BKE_view_layer_synced_ensure(scene, view_layer);
  Base *base = BKE_view_layer_active_base_get(view_layer);
  if (base && BASE_SELECTABLE(v3d, base)) {
    Object *ob = base->object;
    if (ob->type == OB_LAMP) {
      Light *la = ob->data;
      return (la->type == LA_AREA);
    }
  }
  return false;
}

static void WIDGETGROUP_light_area_setup(const bContext *UNUSED(C), wmGizmoGroup *gzgroup)
{
  wmGizmoWrapper *wwrapper = MEM_mallocN(sizeof(wmGizmoWrapper), __func__);
  wwrapper->gizmo = WM_gizmo_new("GIZMO_GT_cage_2d", gzgroup, NULL);
  wmGizmo *gz = wwrapper->gizmo;
  RNA_enum_set(gz->ptr, "transform", ED_GIZMO_CAGE_XFORM_FLAG_SCALE);

  gzgroup->customdata = wwrapper;

  WM_gizmo_set_flag(gz, WM_GIZMO_DRAW_HOVER, true);

  UI_GetThemeColor3fv(TH_GIZMO_PRIMARY, gz->color);
  UI_GetThemeColor3fv(TH_GIZMO_HI, gz->color_hi);

  WM_gizmo_enable_undo(gz, "Resize");
}

static void WIDGETGROUP_light_area_refresh(const bContext *C, wmGizmoGroup *gzgroup)
{
  wmGizmoWrapper *wwrapper = gzgroup->customdata;
  const Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  BKE_view_layer_synced_ensure(scene, view_layer);
  Object *ob = BKE_view_layer_active_object_get(view_layer);
  Light *la = ob->data;
  wmGizmo *gz = wwrapper->gizmo;

  copy_m4_m4(gz->matrix_basis, ob->object_to_world);

  int flag = ED_GIZMO_CAGE_XFORM_FLAG_SCALE;
  if (ELEM(la->area_shape, LA_AREA_SQUARE, LA_AREA_DISK)) {
    flag |= ED_GIZMO_CAGE_XFORM_FLAG_SCALE_UNIFORM;
    flag |= ED_GIZMO_CAGE_XFORM_FLAG_SHAPE_UNIFORM;
  }
  RNA_enum_set(gz->ptr, "transform", flag);

  PointerRNA lamp_ptr;
  RNA_pointer_create(&la->id, &RNA_Light, la, &lamp_ptr);

  /* Need to set property here for `lamp_ptr`. TODO: would prefer to do this in _init. */
  WM_gizmo_target_property_def_rna(gz, "size", &lamp_ptr, "size_xy", -1);
}

void VIEW3D_GGT_light_area(wmGizmoGroupType *gzgt)
{
  gzgt->name = "Area Light Widgets";
  gzgt->idname = "VIEW3D_GGT_light_area";

  gzgt->flag |= (WM_GIZMOGROUPTYPE_PERSISTENT | WM_GIZMOGROUPTYPE_3D | WM_GIZMOGROUPTYPE_DEPTH_3D);

  gzgt->poll = WIDGETGROUP_light_area_poll;
  gzgt->setup = WIDGETGROUP_light_area_setup;
  gzgt->setup_keymap = WM_gizmogroup_setup_keymap_generic_maybe_drag;
  gzgt->refresh = WIDGETGROUP_light_area_refresh;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Light Target Gizmo
 * \{ */

static bool WIDGETGROUP_light_target_poll(const bContext *C, wmGizmoGroupType *UNUSED(gzgt))
{
  View3D *v3d = CTX_wm_view3d(C);
  if (v3d->gizmo_flag & (V3D_GIZMO_HIDE | V3D_GIZMO_HIDE_CONTEXT)) {
    return false;
  }
  if ((v3d->gizmo_show_light & V3D_GIZMO_SHOW_LIGHT_LOOK_AT) == 0) {
    return false;
  }

  const Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  BKE_view_layer_synced_ensure(scene, view_layer);
  Base *base = BKE_view_layer_active_base_get(view_layer);
  if (base && BASE_SELECTABLE(v3d, base)) {
    Object *ob = base->object;
    if (ob->type == OB_LAMP) {
      Light *la = ob->data;
      return ELEM(la->type, LA_SUN, LA_SPOT, LA_AREA);
    }
#if 0
    else if (ob->type == OB_CAMERA) {
      return true;
    }
#endif
  }
  return false;
}

static void WIDGETGROUP_light_target_setup(const bContext *UNUSED(C), wmGizmoGroup *gzgroup)
{
  wmGizmoWrapper *wwrapper = MEM_mallocN(sizeof(wmGizmoWrapper), __func__);
  wwrapper->gizmo = WM_gizmo_new("GIZMO_GT_move_3d", gzgroup, NULL);
  wmGizmo *gz = wwrapper->gizmo;

  gzgroup->customdata = wwrapper;

  UI_GetThemeColor3fv(TH_GIZMO_PRIMARY, gz->color);
  UI_GetThemeColor3fv(TH_GIZMO_HI, gz->color_hi);

  gz->scale_basis = 0.06f;

  wmOperatorType *ot = WM_operatortype_find("OBJECT_OT_transform_axis_target", true);

  RNA_enum_set(
      gz->ptr, "draw_options", ED_GIZMO_MOVE_DRAW_FLAG_FILL | ED_GIZMO_MOVE_DRAW_FLAG_ALIGN_VIEW);

  WM_gizmo_operator_set(gz, 0, ot, NULL);
}

static void WIDGETGROUP_light_target_draw_prepare(const bContext *C, wmGizmoGroup *gzgroup)
{
  wmGizmoWrapper *wwrapper = gzgroup->customdata;
  const Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  BKE_view_layer_synced_ensure(scene, view_layer);
  Object *ob = BKE_view_layer_active_object_get(view_layer);
  wmGizmo *gz = wwrapper->gizmo;

  normalize_m4_m4(gz->matrix_basis, ob->object_to_world);
  unit_m4(gz->matrix_offset);

  if (ob->type == OB_LAMP) {
    Light *la = ob->data;
    if (la->type == LA_SPOT) {
      /* Draw just past the light size angle gizmo. */
      madd_v3_v3fl(gz->matrix_basis[3], gz->matrix_basis[2], -la->spotsize);
    }
  }
  gz->matrix_offset[3][2] -= 23.0;
  WM_gizmo_set_flag(gz, WM_GIZMO_DRAW_OFFSET_SCALE, true);
}

void VIEW3D_GGT_light_target(wmGizmoGroupType *gzgt)
{
  gzgt->name = "Target Light Widgets";
  gzgt->idname = "VIEW3D_GGT_light_target";

  gzgt->flag |= (WM_GIZMOGROUPTYPE_PERSISTENT | WM_GIZMOGROUPTYPE_3D);

  gzgt->poll = WIDGETGROUP_light_target_poll;
  gzgt->setup = WIDGETGROUP_light_target_setup;
  gzgt->setup_keymap = WM_gizmogroup_setup_keymap_generic_maybe_drag;
  gzgt->draw_prepare = WIDGETGROUP_light_target_draw_prepare;
}

/** \} */
