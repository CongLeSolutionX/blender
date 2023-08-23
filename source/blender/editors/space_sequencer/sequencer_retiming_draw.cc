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

#include "WM_api.hh"
#include "WM_types.hh"

#include "ED_keyframes_draw.hh"
#include "ED_keyframes_keylist.hh"
#include "ED_screen.hh"
#include "ED_view3d.hh"

#include "UI_interface.hh"
#include "UI_interface_icons.hh"
#include "UI_resources.hh"
#include "UI_view2d.hh"

#include "SEQ_iterator.h"
#include "SEQ_retiming.h"
#include "SEQ_retiming.hh"
#include "SEQ_sequencer.h"
#include "SEQ_time.h"

/* Own include. */
#include "sequencer_intern.h"
#include "sequencer_intern.hh"

#define KEY_SIZE (10 * U.pixelsize)
#define KEY_CENTER (UI_view2d_view_to_region_y(v2d, strip_y_rescale(seq, 0.0f)) + 4 + KEY_SIZE / 2)

static float strip_y_rescale(const Sequence *seq, const float y_value)
{
  const float y_range = SEQ_STRIP_OFSTOP - SEQ_STRIP_OFSBOTTOM;
  return (y_value * y_range) + seq->machine + SEQ_STRIP_OFSBOTTOM;
}

static float key_x_get(const Scene *scene, const Sequence *seq, const SeqRetimingKey *key)
{
  return SEQ_retiming_key_timeline_frame_get(scene, seq, key);
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

/** Size in pixels. */
#define RETIME_KEY_MOUSEOVER_THRESHOLD (16.0f * UI_SCALE_FAC)

static rctf keys_box_get(const bContext *C, const Sequence *seq)
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

bool retiming_last_key_is_clicked(const bContext *C, const Sequence *seq, const int mval[2])
{
  const Scene *scene = CTX_data_scene(C);
  const View2D *v2d = UI_view2d_fromcontext(C);

  rctf box = keys_box_get(C, seq);
  box.xmax += RETIME_KEY_MOUSEOVER_THRESHOLD; /* Fix selecting last key. */
  if (!mouse_is_inside_box(&box, mval)) {
    return false;
  }

  const float right_handle_pos = UI_view2d_view_to_region_x(
      v2d, SEQ_time_right_handle_frame_get(scene, seq));
  const float distance = fabs(right_handle_pos - mval[0]);
  return distance < RETIME_KEY_MOUSEOVER_THRESHOLD;
}

static const SeqRetimingKey *mouse_over_key_get_from_strip(const bContext *C,
                                                           const Sequence *seq,
                                                           const int mval[2])
{
  const Scene *scene = CTX_data_scene(C);
  const View2D *v2d = UI_view2d_fromcontext(C);

  int best_distance = INT_MAX;
  const SeqRetimingKey *best_key = nullptr;

  for (const SeqRetimingKey &key : SEQ_retiming_keys_get(seq)) {
    int distance = round_fl_to_int(
        fabsf(UI_view2d_view_to_region_x(v2d, key_x_get(scene, seq, &key)) - mval[0]));

    if (distance < RETIME_KEY_MOUSEOVER_THRESHOLD && distance < best_distance) {
      best_distance = distance;
      best_key = &key;
    }
  }

  return best_key;
}

const SeqRetimingKey *retiming_mousover_key_get(const bContext *C,
                                                const int mval[2],
                                                Sequence **r_seq)
{
  const Scene *scene = CTX_data_scene(C);
  for (Sequence *seq : sequencer_visible_strips_get(C)) {

    rctf box = keys_box_get(C, seq);
    box.xmax += RETIME_KEY_MOUSEOVER_THRESHOLD; /* Fix selecting last key. */
    if (!mouse_is_inside_box(&box, mval)) {
      continue;
    }

    if (r_seq != nullptr) {
      *r_seq = seq;
    }

    const SeqRetimingKey *key = mouse_over_key_get_from_strip(C, seq, mval);

    if (key == nullptr) {
      continue;
    }

    if (key_x_get(scene, seq, key) == SEQ_time_left_handle_frame_get(scene, seq) ||
        SEQ_retiming_key_index_get(seq, key) == 0)
    {
      continue;
    }

    return key;
  }

  return nullptr;
}

/* -------------------------------------------------------------------- */
/** \name Retiming Key
 * \{ */

static void draw_half_keyframe(const bContext *C, const Sequence *seq)
{
  const Scene *scene = CTX_data_scene(C);
  const View2D *v2d = UI_view2d_fromcontext(C);
  const Editing *ed = SEQ_editing_get(scene);

  float x;
  const float y = KEY_CENTER;

  GPU_blend(GPU_BLEND_ALPHA);
  uint pos = GPU_vertformat_attr_add(immVertexFormat(), "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);
  SeqRetimingKey *key = nullptr;

  if (SEQ_time_has_right_still_frames(scene, seq)) {
    const int content_end = SEQ_time_content_end_frame_get(scene, seq);
    /* Assume key at content end. */
    x = UI_view2d_view_to_region_x(v2d, content_end) - 1;
    key = SEQ_retiming_last_key_get(seq);
  }
  else {
    const int right_handle_timeline_frame = SEQ_time_right_handle_frame_get(scene, seq);
    /* Assume key at right strip handle. */
    x = UI_view2d_view_to_region_x(v2d, right_handle_timeline_frame) - 1;
    key = SEQ_retiming_key_get_by_timeline_frame(scene, seq, right_handle_timeline_frame);
  }

  bool is_selected = false;

  if (key) {
    if (SEQ_retiming_selection_contains(ed, seq, key) ||
        SEQ_retiming_selection_contains(ed, seq, key - 1))
    {
      immUniform4f("color", 0.65f, 0.5f, 0.2f, 1.0f);
    }
    else {
      immUniform4f("color", 0.0f, 0.0f, 0.0f, 0.1f);
    }

    is_selected = SEQ_retiming_selection_contains(SEQ_editing_get(scene), seq, key);
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

static void retime_key_draw(const bContext *C, const Sequence *seq, const SeqRetimingKey *key)
{
  const Scene *scene = CTX_data_scene(C);
  const float key_x = key_x_get(scene, seq, key);

  if (key_x == SEQ_time_left_handle_frame_get(scene, seq)) {
    return;
  }

  const View2D *v2d = UI_view2d_fromcontext(C);
  const rctf strip_box = strip_box_get(C, seq);
  if (!BLI_rctf_isect_x(&strip_box, UI_view2d_view_to_region_x(v2d, key_x))) {
    return; /* Key out of strip bounds. */
  }

  GPUVertFormat *format = immVertexFormat();
  KeyframeShaderBindings sh_bindings;

  sh_bindings.pos_id = GPU_vertformat_attr_add(format, "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  sh_bindings.size_id = GPU_vertformat_attr_add(format, "size", GPU_COMP_F32, 1, GPU_FETCH_FLOAT);
  sh_bindings.color_id = GPU_vertformat_attr_add(
      format, "color", GPU_COMP_U8, 4, GPU_FETCH_INT_TO_FLOAT_UNIT);
  sh_bindings.outline_color_id = GPU_vertformat_attr_add(
      format, "outlineColor", GPU_COMP_U8, 4, GPU_FETCH_INT_TO_FLOAT_UNIT);
  sh_bindings.flags_id = GPU_vertformat_attr_add(format, "flags", GPU_COMP_U32, 1, GPU_FETCH_INT);

  GPU_blend(GPU_BLEND_ALPHA);
  GPU_program_point_size(true);
  immBindBuiltinProgram(GPU_SHADER_KEYFRAME_SHAPE);
  immUniform1f("outline_scale", 1.0f);
  immUniform2f("ViewportSize", BLI_rcti_size_x(&v2d->mask) + 1, BLI_rcti_size_y(&v2d->mask) + 1);
  immBegin(GPU_PRIM_POINTS, 1);

  eBezTriple_KeyframeType key_type = BEZT_KEYTYPE_KEYFRAME;
  if (SEQ_retiming_key_is_freeze_frame(key)) {
    key_type = BEZT_KEYTYPE_BREAKDOWN;
  }
  if (SEQ_retiming_key_is_transition_type(key)) {
    key_type = BEZT_KEYTYPE_MOVEHOLD;
  }

  const bool is_selected = SEQ_retiming_selection_contains(SEQ_editing_get(scene), seq, key);
  const int size = KEY_SIZE;
  const float key_position = UI_view2d_view_to_region_x(v2d, key_x);
  const float bottom = KEY_CENTER;

  draw_keyframe_shape(key_position,
                      bottom,
                      size,
                      is_selected && sequencer_retiming_tool_is_active(C),
                      key_type,
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

const void draw_continuity(const bContext *C, const Sequence *seq, const SeqRetimingKey *key)
{
  if (!sequencer_retiming_tool_is_active(C)) {
    return;
  }
  const View2D *v2d = UI_view2d_fromcontext(C);
  const Scene *scene = CTX_data_scene(C);
  const Editing *ed = SEQ_editing_get(scene);

  const float left_handle_position = UI_view2d_view_to_region_x(
      v2d, SEQ_time_left_handle_frame_get(scene, seq));
  const float right_handle_position = UI_view2d_view_to_region_x(
      v2d, SEQ_time_right_handle_frame_get(scene, seq));

  float key_position = UI_view2d_view_to_region_x(v2d, key_x_get(scene, seq, key));
  float prev_key_position = UI_view2d_view_to_region_x(v2d, key_x_get(scene, seq, key - 1));
  prev_key_position = max_ff(prev_key_position, left_handle_position);
  key_position = min_ff(key_position, right_handle_position);

  const int size = KEY_SIZE;
  const float y_center = KEY_CENTER;

  const float width_fac = 0.5f;
  const float bottom = y_center - size * width_fac;
  const float top = y_center + size * width_fac;

  GPU_blend(GPU_BLEND_ALPHA);
  uint pos = GPU_vertformat_attr_add(immVertexFormat(), "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);

  if (SEQ_retiming_selection_contains(ed, seq, key) ||
      SEQ_retiming_selection_contains(ed, seq, key - 1))
  {
    immUniform4f("color", 0.65f, 0.5f, 0.2f, 1.0f);
  }
  else {
    immUniform4f("color", 0.0f, 0.0f, 0.0f, 0.1f);
  }
  immRectf(pos, prev_key_position, bottom, key_position, top);
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

static void retime_keys_draw(const bContext *C)
{
  const SpaceSeq *sseq = CTX_wm_space_seq(C);
  if (!sequencer_retiming_tool_is_active(C) &&
      (sseq->timeline_overlay.flag & SEQ_TIMELINE_SHOW_STRIP_RETIMING) == 0)
  {
    return;
  }

  wmOrtho2_region_pixelspace(CTX_wm_region(C));

  for (Sequence *seq : sequencer_visible_strips_get(C)) {
    if (!SEQ_retiming_is_allowed(seq)) {
      continue;
    }

    draw_backdrop(C, seq);

    blender::MutableSpan keys = SEQ_retiming_keys_get(seq);
    for (const SeqRetimingKey &key : keys) {
      if (&key == keys.begin()) {
        continue; /* Ignore first key. */
      }
      draw_continuity(C, seq, &key);
    }
    for (const SeqRetimingKey &key : keys) {
      if (&key == keys.begin()) {
        continue; /* Ignore first key. */
      }
      retime_key_draw(C, seq, &key);
    }
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Retiming Speed Label
 * \{ */

static size_t label_str_get(const Sequence *seq,
                            const SeqRetimingKey *key,
                            char *r_label_str,
                            const size_t label_str_maxncpy)
{
  const SeqRetimingKey *next_key = key + 1;
  if (SEQ_retiming_key_is_transition_start(key)) {
    const float prev_speed = SEQ_retiming_key_speed_get(seq, key);
    const float next_speed = SEQ_retiming_key_speed_get(seq, next_key + 1);
    return BLI_snprintf_rlen(r_label_str,
                             label_str_maxncpy,
                             "%d%% - %d%%",
                             round_fl_to_int(prev_speed * 100.0f),
                             round_fl_to_int(next_speed * 100.0f));
  }
  const float speed = SEQ_retiming_key_speed_get(seq, next_key);
  return BLI_snprintf_rlen(
      r_label_str, label_str_maxncpy, "%d%%", round_fl_to_int(speed * 100.0f));
}

static bool label_rect_get(const bContext *C,
                           const Sequence *seq,
                           const SeqRetimingKey *key,
                           char *label_str,
                           const size_t label_len,
                           rctf *rect)
{
  const Scene *scene = CTX_data_scene(C);
  const SeqRetimingKey *next_key = key + 1;
  const float width = pixels_to_view_width(C, BLF_width(BLF_default(), label_str, label_len));
  const float height = pixels_to_view_height(C, BLF_height(BLF_default(), label_str, label_len));

  const float xmin = max_ff(SEQ_time_left_handle_frame_get(scene, seq),
                            key_x_get(scene, seq, key));
  const float xmax = min_ff(SEQ_time_right_handle_frame_get(scene, seq),
                            key_x_get(scene, seq, next_key));

  rect->xmin = (xmin + xmax - width) / 2;
  rect->xmax = rect->xmin + width;
  rect->ymin = strip_y_rescale(seq, 0) + pixels_to_view_height(C, 5);
  rect->ymax = rect->ymin + height;

  return width < xmax - xmin - pixels_to_view_width(C, KEY_SIZE);
}

static void retime_speed_text_draw(const bContext *C,
                                   const Sequence *seq,
                                   const SeqRetimingKey *key)
{
  if (SEQ_retiming_is_last_key(seq, key)) {
    return;
  }

  const Scene *scene = CTX_data_scene(C);
  const int start_frame = SEQ_time_left_handle_frame_get(scene, seq);
  const int end_frame = SEQ_time_right_handle_frame_get(scene, seq);

  const SeqRetimingKey *next_key = key + 1;
  if (key_x_get(scene, seq, next_key) < start_frame || key_x_get(scene, seq, key) > end_frame) {
    return; /* Label out of strip bounds. */
  }

  char label_str[40];
  rctf label_rect;
  size_t label_len = label_str_get(seq, key, label_str, sizeof(label_str));

  if (!label_rect_get(C, seq, key, label_str, label_len, &label_rect)) {
    return; /* Not enough space to draw label. */
  }

  const uchar col[4] = {255, 255, 255, 255};
  UI_view2d_text_cache_add(
      UI_view2d_fromcontext(C), label_rect.xmin, label_rect.ymin, label_str, label_len, col);
}

static void retime_speed_draw(const bContext *C)
{
  const SpaceSeq *sseq = CTX_wm_space_seq(C);
  if (!sequencer_retiming_tool_is_active(C) &&
      (sseq->timeline_overlay.flag & SEQ_TIMELINE_SHOW_STRIP_RETIMING) == 0)
  {
    return;
  }

  const View2D *v2d = UI_view2d_fromcontext(C);

  wmOrtho2_region_pixelspace(CTX_wm_region(C));
  GPU_blend(GPU_BLEND_ALPHA);
  GPU_vertformat_attr_add(immVertexFormat(), "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);

  for (const Sequence *seq : sequencer_visible_strips_get(C)) {
    for (const SeqRetimingKey &key : SEQ_retiming_keys_get(seq)) {
      retime_speed_text_draw(C, seq, &key);
    }
  }

  immUnbindProgram();
  GPU_blend(GPU_BLEND_NONE);

  UI_view2d_text_cache_draw(CTX_wm_region(C));
  UI_view2d_view_ortho(v2d); /* 'UI_view2d_text_cache_draw()' messes up current view. */
}

/** \} */

void sequencer_draw_retiming(const bContext *C)
{
  retime_keys_draw(C);
  retime_speed_draw(C);
}
