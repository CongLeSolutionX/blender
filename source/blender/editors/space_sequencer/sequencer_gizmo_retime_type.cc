/* SPDX-FileCopyrightText: 2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spseq
 */

#include "MEM_guardedalloc.h"

#include "BLI_blenlib.h"
#include "BLI_span.hh"

#include "DNA_anim_types.h"
#include "DNA_sequence_types.h"

#include "BKE_context.h"
#include "BKE_fcurve.h"
#include "BKE_scene.h"

#include "BLF_api.h"

#include "GPU_batch.h"
#include "GPU_batch_utils.h"
#include "GPU_immediate.h"
#include "GPU_immediate_util.h"
#include "GPU_matrix.h"
#include "GPU_select.h"
#include "GPU_state.h"

#include "RNA_access.h"
#include "RNA_define.h"
#include "RNA_enum_types.h"

#include "WM_api.h"
#include "WM_types.h"

#include "ED_gizmo_library.h"
#include "ED_keyframes_draw.h"
#include "ED_keyframes_keylist.h"
#include "ED_screen.h"
#include "ED_view3d.h"

#include "UI_interface.h"
#include "UI_interface_icons.h"
#include "UI_resources.h"
#include "UI_view2d.h"

#include "SEQ_iterator.h"
#include "SEQ_retiming.h"
#include "SEQ_retiming.hh"
#include "SEQ_sequencer.h"
#include "SEQ_time.h"

/* Own include. */
#include "sequencer_intern.h"
#include "sequencer_intern.hh"

using blender::MutableSpan;

/** Size in pixels. */
#define RETIME_HANDLE_MOUSEOVER_THRESHOLD (16.0f * UI_SCALE_FAC)
/** Factor based on icon size. */
#define RETIME_BUTTON_SIZE 0.6f
#define KEY_SIZE (10 * U.pixelsize)
#define KEY_CENTER (UI_view2d_view_to_region_y(v2d, strip_y_rescale(seq, 0.0f)) + 4 + KEY_SIZE / 2)

static float strip_y_rescale(const Sequence *seq, const float y_value)
{
  const float y_range = SEQ_STRIP_OFSTOP - SEQ_STRIP_OFSBOTTOM;
  return (y_value * y_range) + seq->machine + SEQ_STRIP_OFSBOTTOM;
}

static float handle_x_get(const Scene *scene, const Sequence *seq, const SeqRetimingHandle *handle)
{
  return SEQ_retiming_handle_timeline_frame_get(scene, seq, handle);
}

static float pixels_to_view_width(const bContext *C, const float width)
{
  const View2D *v2d = UI_view2d_fromcontext(C);
  float scale_x = UI_view2d_view_to_region_x(v2d, 1) - UI_view2d_view_to_region_x(v2d, 0.0f);
  return width / scale_x;
}

static float pixels_to_view_height(const bContext *C, const float height)
{
  const View2D *v2d = UI_view2d_fromcontext(C);
  float scale_y = UI_view2d_view_to_region_y(v2d, 1) - UI_view2d_view_to_region_y(v2d, 0.0f);
  return height / scale_y;
}

static float strip_start_screenspace_get(const bContext *C, const Sequence *seq)
{
  const View2D *v2d = UI_view2d_fromcontext(C);
  const Scene *scene = CTX_data_scene(C);
  return UI_view2d_view_to_region_x(v2d, SEQ_time_left_handle_frame_get(scene, seq));
}

static float strip_end_screenspace_get(const bContext *C, const Sequence *seq)
{
  const View2D *v2d = UI_view2d_fromcontext(C);
  const Scene *scene = CTX_data_scene(C);
  return UI_view2d_view_to_region_x(v2d, SEQ_time_right_handle_frame_get(scene, seq));
}

static Sequence *active_seq_from_context(const bContext *C)
{
  const Editing *ed = SEQ_editing_get(CTX_data_scene(C));
  return ed->act_seq;
}

static rctf strip_box_get(const bContext *C, const Sequence *seq)
{
  const View2D *v2d = UI_view2d_fromcontext(C);
  rctf rect;
  rect.xmin = strip_start_screenspace_get(C, seq);
  rect.xmax = strip_end_screenspace_get(C, seq);
  rect.ymin = UI_view2d_view_to_region_y(v2d, strip_y_rescale(seq, 0));
  rect.ymax = UI_view2d_view_to_region_y(v2d, strip_y_rescale(seq, 1));
  return rect;
}

static rctf handles_box_get(const bContext *C, const Sequence *seq)
{
  const View2D *v2d = UI_view2d_fromcontext(C);
  rctf rect = strip_box_get(C, seq);
  rect.ymax = KEY_CENTER + KEY_SIZE / 2;
  rect.ymin = KEY_CENTER - KEY_SIZE / 2;
  return rect;
}

static bool mouse_is_inside_box(const rctf *box, const int mval[2])
{
  return mval[0] >= box->xmin && mval[0] <= box->xmax && mval[1] >= box->ymin &&
         mval[1] <= box->ymax;
}

blender::Vector<Sequence *> sequencer_visible_strips_get(const bContext *C)
{
  const View2D *v2d = UI_view2d_fromcontext(C);
  const Scene *scene = CTX_data_scene(C);
  const Editing *ed = SEQ_editing_get(CTX_data_scene(C));
  blender::Vector<Sequence *> strips;

  LISTBASE_FOREACH (Sequence *, seq, ed->seqbasep) {
    if (min_ii(SEQ_time_left_handle_frame_get(scene, seq), SEQ_time_start_frame_get(seq)) >
        v2d->cur.xmax)
    {
      continue;
    }
    if (max_ii(SEQ_time_right_handle_frame_get(scene, seq),
               SEQ_time_content_end_frame_get(scene, seq)) < v2d->cur.xmin)
    {
      continue;
    }
    if (seq->machine + 1.0f < v2d->cur.ymin) {
      continue;
    }
    if (seq->machine > v2d->cur.ymax) {
      continue;
    }
    strips.append(seq);
  }
  return strips;
}

static const SeqRetimingHandle *mouse_over_handle_get_from_strip(bContext *C,
                                                                 const Sequence *seq,
                                                                 const int mval[2])
{
  Scene *scene = CTX_data_scene(C);
  const View2D *v2d = UI_view2d_fromcontext(C);

  rctf box = handles_box_get(C, seq);
  box.xmax += RETIME_HANDLE_MOUSEOVER_THRESHOLD; /* Fix selecting last handle. */
  if (!mouse_is_inside_box(&box, mval)) {
    return nullptr;
  }

  int best_distance = INT_MAX;
  const SeqRetimingHandle *best_handle = nullptr;

  MutableSpan handles = SEQ_retiming_handles_get(seq);
  for (const SeqRetimingHandle &handle : handles) {
    int distance = round_fl_to_int(
        fabsf(UI_view2d_view_to_region_x(v2d, handle_x_get(scene, seq, &handle)) - mval[0]));

    if (distance < RETIME_HANDLE_MOUSEOVER_THRESHOLD && distance < best_distance) {
      best_distance = distance;
      best_handle = &handle;
    }
  }

  return best_handle;
}

const SeqRetimingHandle *mousover_handle_get(bContext *C, const int mval[2], Sequence **r_seq)
{

  Scene *scene = CTX_data_scene(C);
  for (Sequence *seq : sequencer_visible_strips_get(C)) {
    const SeqRetimingHandle *handle = mouse_over_handle_get_from_strip(C, seq, mval);

    if (handle == nullptr) {
      continue;
    }

    if (handle_x_get(scene, seq, handle) == SEQ_time_left_handle_frame_get(scene, seq) ||
        SEQ_retiming_handle_index_get(seq, handle) == 0)
    {
      continue;
    }

    if (r_seq != nullptr) {
      *r_seq = seq;
    }

    return handle;
  }

  return nullptr;
}

/* -------------------------------------------------------------------- */
/** \name Retiming Move Handle Gizmo
 * \{ */

static void draw_half_keyframe(const bContext *C, const Sequence *seq)
{
  const Scene *scene = CTX_data_scene(C);
  const View2D *v2d = UI_view2d_fromcontext(C);
  Editing *ed = SEQ_editing_get(scene);

  const int right_handle_timeline_frame = SEQ_time_right_handle_frame_get(scene, seq);
  const float x = UI_view2d_view_to_region_x(v2d, right_handle_timeline_frame) - 1;
  const float y = KEY_CENTER;

  GPU_blend(GPU_BLEND_ALPHA);
  uint pos = GPU_vertformat_attr_add(immVertexFormat(), "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);

  const SeqRetimingHandle *handle = SEQ_retiming_handle_get_by_timeline_frame(
      scene, seq, right_handle_timeline_frame);
  bool is_selected = false;

  if (handle) {
    if (SEQ_retiming_selection_contains(ed, seq, handle) ||
        SEQ_retiming_selection_contains(ed, seq, handle - 1))
    {
      immUniform4f("color", 0.65f, 0.5f, 0.2f, 1.0f);
    }
    else {
      immUniform4f("color", 0.0f, 0.0f, 0.0f, 0.1f);
    }

    is_selected = SEQ_retiming_selection_contains(SEQ_editing_get(scene), seq, handle);
  }

  const int size = KEY_SIZE * 0.5f;
  unsigned char col[4];

  /* Triangle body. */
  UI_GetThemeColor4ubv(is_selected ? TH_KEYTYPE_KEYFRAME_SELECT : TH_KEYTYPE_KEYFRAME, col);
  immUniformColor4ubv(col);
  immBegin(GPU_PRIM_TRI_FAN, 3);
  immVertex2f(pos, x, y + size);
  immVertex2f(pos, x, y - size);
  immVertex2f(pos, x - size, y);
  immEnd();

  /* Outline. */
  UI_GetThemeColor4ubv(is_selected ? TH_KEYBORDER_SELECT : TH_KEYBORDER, col);
  immUniformColor4ubv(col);
  immBegin(GPU_PRIM_LINE_LOOP, 3);
  immVertex2f(pos, x, y + size);
  immVertex2f(pos, x, y - size);
  immVertex2f(pos, x - size, y);
  immEnd();
  immUnbindProgram();
  GPU_blend(GPU_BLEND_NONE);
}

static void retime_handle_draw(const bContext *C,
                               const Sequence *seq,
                               const SeqRetimingHandle *handle)
{

  if (SEQ_retiming_is_last_handle(seq, handle)) {
    return;
  }

  const Scene *scene = CTX_data_scene(C);
  const float handle_x = handle_x_get(scene, seq, handle);

  if (handle_x == SEQ_time_left_handle_frame_get(scene, seq)) {
    return;
  }

  const View2D *v2d = UI_view2d_fromcontext(C);
  const rctf strip_box = strip_box_get(C, seq);
  if (!BLI_rctf_isect_x(&strip_box, UI_view2d_view_to_region_x(v2d, handle_x))) {
    return; /* Handle out of strip bounds. */
  }

  float col[4] = {1.0f, 1.0f, 1.0f, 1.0f};

  bool is_selected = SEQ_retiming_selection_contains(SEQ_editing_get(scene), seq, handle);

  /*
  if (handle == gizmo->mouse_over_handle || is_selected) {

    bool handle_is_transition = SEQ_retiming_handle_is_transition_type(handle);
    bool prev_handle_is_transition = SEQ_retiming_handle_is_transition_type(handle - 1);
    bool handle_is_freeze_frame = SEQ_retiming_handle_is_freeze_frame(handle);
    bool prev_handle_is_freeze_frame = SEQ_retiming_handle_is_freeze_frame(handle - 1);

    if (!(handle_is_transition || prev_handle_is_transition || handle_is_freeze_frame ||
          prev_handle_is_freeze_frame))
    {
      if (gizmo->operation == MAKE_TRANSITION) {
        col[0] = 0.5f;
        col[2] = 0.4f;
      }
      else if (gizmo->operation == MAKE_FREEZE_FRAME) {
        col[0] = 0.4f;
        col[1] = 0.8f;
      }
    }
  }
  else {
    mul_v3_fl(col, 0.65f);
  }*/

  const int size = KEY_SIZE;
  const float handle_position = UI_view2d_view_to_region_x(v2d, handle_x);
  const float bottom = KEY_CENTER;

  GPUVertFormat *format = immVertexFormat();
  KeyframeShaderBindings sh_bindings;

  sh_bindings.pos_id = GPU_vertformat_attr_add(format, "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  sh_bindings.size_id = GPU_vertformat_attr_add(format, "size", GPU_COMP_F32, 1, GPU_FETCH_FLOAT);
  sh_bindings.color_id = GPU_vertformat_attr_add(
      format, "color", GPU_COMP_U8, 4, GPU_FETCH_INT_TO_FLOAT_UNIT);
  sh_bindings.outline_color_id = GPU_vertformat_attr_add(
      format, "outlineColor", GPU_COMP_U8, 4, GPU_FETCH_INT_TO_FLOAT_UNIT);
  sh_bindings.flags_id = GPU_vertformat_attr_add(format, "flags", GPU_COMP_U32, 1, GPU_FETCH_INT);

  if (SEQ_retiming_handle_timeline_frame_get(scene, seq, handle) !=
      SEQ_time_right_handle_frame_get(scene, seq))
  {
    GPU_blend(GPU_BLEND_ALPHA);
    GPU_program_point_size(true);
    immBindBuiltinProgram(GPU_SHADER_KEYFRAME_SHAPE);
    immUniform1f("outline_scale", 1.0f);
    immUniform2f("ViewportSize", BLI_rcti_size_x(&v2d->mask) + 1, BLI_rcti_size_y(&v2d->mask) + 1);
    immBegin(GPU_PRIM_POINTS, 1);
    draw_keyframe_shape(handle_position,
                        bottom,
                        size,
                        is_selected && sequencer_retiming_tool_is_active(C),
                        0,
                        KEYFRAME_SHAPE_BOTH,
                        1,
                        &sh_bindings,
                        0,
                        0);
    immEnd();
    GPU_program_point_size(false);
    immUnbindProgram();
    GPU_blend(GPU_BLEND_NONE);
  }
}

const void draw_continuity(const bContext *C, const Sequence *seq, const SeqRetimingHandle *handle)
{
  if (!sequencer_retiming_tool_is_active(C)) {
    return;
  }
  const View2D *v2d = UI_view2d_fromcontext(C);
  const Scene *scene = CTX_data_scene(C);
  Editing *ed = SEQ_editing_get(scene);

  float retiming_handle_position = UI_view2d_view_to_region_x(v2d,
                                                              handle_x_get(scene, seq, handle));
  float prev_retiming_handle_position = UI_view2d_view_to_region_x(
      v2d, handle_x_get(scene, seq, handle - 1));

  const float left_handle_position = UI_view2d_view_to_region_x(
      v2d, SEQ_time_left_handle_frame_get(scene, seq));
  const float right_handle_position = UI_view2d_view_to_region_x(
      v2d, SEQ_time_right_handle_frame_get(scene, seq));

  prev_retiming_handle_position = max_ff(prev_retiming_handle_position, left_handle_position);
  retiming_handle_position = min_ff(retiming_handle_position, right_handle_position);

  const int size = KEY_SIZE;
  const float y_center = KEY_CENTER;

  const float width_fac = 0.5f;
  const float bottom = y_center - size * width_fac;
  const float top = y_center + size * width_fac;

  GPU_blend(GPU_BLEND_ALPHA);
  uint pos = GPU_vertformat_attr_add(immVertexFormat(), "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);

  if (SEQ_retiming_selection_contains(ed, seq, handle) ||
      SEQ_retiming_selection_contains(ed, seq, handle - 1))
  {
    immUniform4f("color", 0.65f, 0.5f, 0.2f, 1.0f);
  }
  else {
    immUniform4f("color", 0.0f, 0.0f, 0.0f, 0.1f);
  }
  immRectf(pos, prev_retiming_handle_position, bottom, retiming_handle_position, top);
  immUnbindProgram();
  GPU_blend(GPU_BLEND_NONE);
}

static void draw_backdrop(const bContext *C, const Sequence *seq)
{
  if (!sequencer_retiming_tool_is_active(C)) {
    return;
  }
  return;
  const View2D *v2d = UI_view2d_fromcontext(C);
  const Scene *scene = CTX_data_scene(C);

  const float start = UI_view2d_view_to_region_x(v2d, SEQ_time_left_handle_frame_get(scene, seq));
  const float end = UI_view2d_view_to_region_x(v2d, SEQ_time_right_handle_frame_get(scene, seq));

  const int size = KEY_SIZE;
  const float y_center = KEY_CENTER;
  const float bottom = y_center - size / 2;
  const float top = y_center + size / 2;

  GPU_blend(GPU_BLEND_ALPHA);
  uint pos = GPU_vertformat_attr_add(immVertexFormat(), "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);

  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);
  immUniform4f("color", 0.0f, 0.0f, 0.0f, 0.3f);
  immRectf(pos, start, bottom, end, top);
  immUnbindProgram();
  GPU_blend(GPU_BLEND_NONE);
}

static void gizmo_retime_handle_draw(const bContext *C, wmGizmo *gz)
{
  wmOrtho2_region_pixelspace(CTX_wm_region(C));

  for (Sequence *seq : sequencer_visible_strips_get(C)) {
    draw_backdrop(C, seq);

    MutableSpan handles = SEQ_retiming_handles_get(seq);
    for (const SeqRetimingHandle &handle : handles) {
      if (&handle == handles.begin()) {
        continue; /* Ignore first handle. */
      }
      draw_continuity(C, seq, &handle);
    }
    for (const SeqRetimingHandle &handle : handles) {
      if (&handle == handles.begin()) {
        continue; /* Ignore first handle. */
      }
      retime_handle_draw(C, seq, &handle);
    }

    /* Special case for last handle. */
    draw_half_keyframe(C, seq);
  }
}

static int gizmo_retime_handle_test_select(bContext *C, wmGizmo *gz, const int mval[2])
{
  return -1;
}

static void gizmo_retime_handle_setup(wmGizmo *gz)
{
  gz->flag = WM_GIZMO_DRAW_MODAL;
}

void GIZMO_GT_retime_handle(wmGizmoType *gzt)
{
  /* Identifiers. */
  gzt->idname = "GIZMO_GT_retime_handle_move";

  /* Api callbacks. */
  gzt->setup = gizmo_retime_handle_setup;
  gzt->draw = gizmo_retime_handle_draw;
  gzt->test_select = gizmo_retime_handle_test_select;
  gzt->struct_size = sizeof(wmGizmo);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Retiming Speed Set Gizmo
 * \{ */

static size_t label_str_get(const Sequence *seq,
                            const SeqRetimingHandle *handle,
                            size_t str_len,
                            char *r_label_str)
{
  const SeqRetimingHandle *next_handle = handle + 1;
  if (SEQ_retiming_handle_is_transition_type(handle)) {
    const float prev_speed = SEQ_retiming_handle_speed_get(seq, handle - 1);
    const float next_speed = SEQ_retiming_handle_speed_get(seq, next_handle + 1);
    return BLI_snprintf_rlen(r_label_str,
                             str_len,
                             "%d%% - %d%%",
                             round_fl_to_int(prev_speed * 100.0f),
                             round_fl_to_int(next_speed * 100.0f));
  }
  const float speed = SEQ_retiming_handle_speed_get(seq, next_handle);
  return BLI_snprintf_rlen(r_label_str, str_len, "%d%%", round_fl_to_int(speed * 100.0f));
}

static bool label_rect_get(const bContext *C,
                           const Sequence *seq,
                           const SeqRetimingHandle *handle,
                           char *label_str,
                           size_t label_len,
                           rctf *rect)
{
  const Scene *scene = CTX_data_scene(C);
  const SeqRetimingHandle *next_handle = handle + 1;
  const float width = pixels_to_view_width(C, BLF_width(BLF_default(), label_str, label_len));
  const float height = pixels_to_view_height(C, BLF_height(BLF_default(), label_str, label_len));

  const float xmin = max_ff(SEQ_time_left_handle_frame_get(scene, seq),
                            handle_x_get(scene, seq, handle));
  const float xmax = min_ff(SEQ_time_right_handle_frame_get(scene, seq),
                            handle_x_get(scene, seq, next_handle));

  rect->xmin = (xmin + xmax - width) / 2;
  rect->xmax = rect->xmin + width;
  rect->ymin = strip_y_rescale(seq, 0) + pixels_to_view_height(C, 5);
  rect->ymax = rect->ymin + height;

  return width < xmax - xmin - pixels_to_view_width(C, KEY_SIZE);
}

static void label_rect_apply_mouseover_offset(const View2D *v2d, rctf *rect)
{
  float scale_x, scale_y;
  UI_view2d_scale_get_inverse(v2d, &scale_x, &scale_y);
  rect->xmin -= RETIME_HANDLE_MOUSEOVER_THRESHOLD * scale_x;
  rect->xmax += RETIME_HANDLE_MOUSEOVER_THRESHOLD * scale_x;
  rect->ymax += RETIME_HANDLE_MOUSEOVER_THRESHOLD * scale_y;
}

static void retime_speed_text_draw(const bContext *C,
                                   const Sequence *seq,
                                   const SeqRetimingHandle *handle)
{
  if (SEQ_retiming_is_last_handle(seq, handle)) {
    return;
  }

  const Scene *scene = CTX_data_scene(C);
  const int start_frame = SEQ_time_left_handle_frame_get(scene, seq);
  const int end_frame = SEQ_time_right_handle_frame_get(scene, seq);

  const SeqRetimingHandle *next_handle = handle + 1;
  if (handle_x_get(scene, seq, next_handle) < start_frame ||
      handle_x_get(scene, seq, handle) > end_frame)
  {
    return; /* Label out of strip bounds. */
  }

  char label_str[40];
  rctf label_rect;
  size_t label_len = label_str_get(seq, handle, sizeof(label_str), label_str);

  if (!label_rect_get(C, seq, handle, label_str, label_len, &label_rect)) {
    return; /* Not enough space to draw label. */
  }

  const uchar col[4] = {255, 255, 255, 255};
  UI_view2d_text_cache_add(
      UI_view2d_fromcontext(C), label_rect.xmin, label_rect.ymin, label_str, label_len, col);
}

static void gizmo_retime_speed_set_draw(const bContext *C, wmGizmo * /* gz */)
{
  if (!sequencer_retiming_tool_is_active(C)) {
    return;
  }

  const View2D *v2d = UI_view2d_fromcontext(C);

  wmOrtho2_region_pixelspace(CTX_wm_region(C));
  GPU_blend(GPU_BLEND_ALPHA);
  GPU_vertformat_attr_add(immVertexFormat(), "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);

  for (Sequence *seq : sequencer_visible_strips_get(C)) {
    for (const SeqRetimingHandle &handle : SEQ_retiming_handles_get(seq)) {
      retime_speed_text_draw(C, seq, &handle);
    }
  }

  immUnbindProgram();
  GPU_blend(GPU_BLEND_NONE);

  UI_view2d_text_cache_draw(CTX_wm_region(C));
  UI_view2d_view_ortho(v2d); /* 'UI_view2d_text_cache_draw()' messes up current view. */
}

static int gizmo_retime_speed_set_test_select(bContext *C, wmGizmo *gz, const int mval[2])
{
  return -1;
}

void GIZMO_GT_speed_set_remove(wmGizmoType *gzt)
{
  /* Identifiers. */
  gzt->idname = "GIZMO_GT_retime_speed_set";

  /* Api callbacks. */
  gzt->draw = gizmo_retime_speed_set_draw;
  gzt->test_select = gizmo_retime_speed_set_test_select;
  gzt->struct_size = sizeof(wmGizmo);
}

/** \} */
