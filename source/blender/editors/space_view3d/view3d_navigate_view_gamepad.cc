/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_context.hh"

#include "BLI_math_rotation.h"
#include "BLI_math_vector.h"

#include "WM_api.hh"

#include "ED_screen.hh"

#include "BLI_math_basis_types.hh"
#include "BLI_math_vector_types.hh"

#include "view3d_intern.hh"
#include "view3d_navigate.hh"

#ifdef WITH_INPUT_GAMEPAD
namespace blender::view3d {
void gamepad_fly(const wmGamepadAxisData &gamepad,
                 View3D * /*v3d*/,
                 RegionView3D *rv3d,
                 const bool /*use_precision*/,
                 const short /*protect_flag*/,
                 bool &r_has_translate,
                 bool &r_has_rotate)
{
  float3 translation_vector{gamepad.left_thumb.value[0], 0.0f, -gamepad.left_thumb.value[1]};
  float4 view_inv;

  invert_qt_qt_normalized(view_inv, rv3d->viewquat);

  bool has_translation = translation_vector;
  if (has_translation) {
    const float speed = 50.0f;
    translation_vector *= speed * gamepad.dt;
    mul_qt_v3(view_inv, translation_vector);
    sub_v3_v3(rv3d->ofs, translation_vector);
  }

  float3 rotation_vector{-gamepad.right_thumb.value[1],
                         gamepad.right_thumb.value[0],
                         -gamepad.left_trigger.value + gamepad.right_trigger.value};

  bool has_rotation = rotation_vector;
  if (has_rotation) {
    float4 rotation{};
    const float3 rotation_speed{1.5f, 2.0f, 2.0f};
    rotation_vector *= rotation_speed * gamepad.dt;
    mul_qt_v3(view_inv, rotation_vector);

    eul_to_quat(rotation, rotation_vector);
    mul_qt_qtqt(rv3d->viewquat, rv3d->viewquat, rotation);
  }
  r_has_translate = has_translation;
  r_has_rotate = has_rotation;
}

static void gamepad_move(bContext *C, float3 translation, const float dt)
{
  ARegion *region = CTX_wm_region(C);
  RegionView3D *rv3d = static_cast<RegionView3D *>(region->regiondata);
  float4 view_inv;

  invert_qt_qt_normalized(view_inv, rv3d->viewquat);

  bool has_translation = translation;
  if (has_translation) {
    const float speed = 50.0f;
    translation *= speed * dt;
    mul_qt_v3(view_inv, translation);
    sub_v3_v3(rv3d->ofs, translation);
  }
};

static void gamepad_rotate(bContext *C, float3 rotation, float dt)
{
  ARegion *region = CTX_wm_region(C);
  RegionView3D *rv3d = static_cast<RegionView3D *>(region->regiondata);

  float4 view_inv;
  invert_qt_qt_normalized(view_inv, rv3d->viewquat);

  const float3 rotation_speed{1.5f, 2.0f, 2.0f};
  rotation *= rotation_speed * dt;
  mul_qt_v3(view_inv, rotation);
  float4 quad_rotation;
  eul_to_quat(quad_rotation, rotation);
  mul_qt_qtqt(rv3d->viewquat, rv3d->viewquat, quad_rotation);

  {
    invert_qt_qt_normalized(view_inv, rv3d->viewquat);
    float3 view_horizon{1.0f, 0.0f, 0.0f};
    float3 view_direction{0.0f, 0.0f, -1.0f};
    mul_qt_v3(view_inv, view_horizon);
    mul_qt_v3(view_inv, view_direction);
    const float angle = -asinf(view_horizon[2]);

    axis_angle_to_quat(rotation, view_direction, angle);
    mul_qt_qtqt(rv3d->viewquat, rv3d->viewquat, rotation);
  }
};

static int gamepad_all_invoke_impl(bContext *C, wmOperator * /*op*/, const wmEvent *event)
{
  if (ELEM(event->type,
           GAMEPAD_BUTTON_DPAD_UP,
           GAMEPAD_BUTTON_DPAD_DOWN,
           GAMEPAD_BUTTON_DPAD_LEFT,
           GAMEPAD_BUTTON_DPAD_RIGHT))
  {
    const float up = event->type == GAMEPAD_BUTTON_DPAD_UP   ? 1.0f :
                     event->type == GAMEPAD_BUTTON_DPAD_DOWN ? -1.0f :
                                                               0.0f;
    const float right = event->type == GAMEPAD_BUTTON_DPAD_RIGHT ? 1.0f :
                        event->type == GAMEPAD_BUTTON_DPAD_LEFT  ? -1.0f :
                                                                   0.0f;
    gamepad_move(C, {right, up, 0.0f}, 0.01f);
  }
  if (event->type == GAMEPAD_LEFT_THUMB) {
    gamepad_move(C, {event->axis_value[0], event->axis_value[1], 0.0f}, event->dt);
  }
  if (event->type == GAMEPAD_RIGHT_THUMB) {
    gamepad_rotate(C, {-event->axis_value[1], event->axis_value[0], 0.0f}, event->dt);
  }
  if (event->type == GAMEPAD_LEFT_TRIGGER || event->type == GAMEPAD_RIGHT_TRIGGER) {
    const float dir = event->type == GAMEPAD_LEFT_TRIGGER ? 1.0f : -1.0f;
    gamepad_move(C, {0.0f, 0.0f, dir * event->axis_value[0]}, 0.01f);
  }

  ARegion *region = CTX_wm_region(C);
  ED_region_tag_redraw(region);
  return OPERATOR_FINISHED;
}
}  // namespace blender::view3d

void VIEW3D_OT_gamepad_all(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Gamepad Transform View";
  ot->description = "Move and rotate the view with gamepad";
  ot->idname = "VIEW3D_OT_gamepad_all";

  /* api callbacks */
  ot->invoke = blender::view3d::gamepad_all_invoke_impl;
  ot->poll = ED_operator_view3d_active;

  /* flags */
  ot->flag = 0;
}

#endif /* WITH_INPUT_GAMEPAD */
