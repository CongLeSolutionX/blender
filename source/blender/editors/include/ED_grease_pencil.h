/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editors
 */

#pragma once

struct bContext;

struct GreasePencil;
struct GreasePencilDrawing;
struct Main;
struct Object;
struct ViewContext;

struct wmKeyConfig;

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------- */
/** \name C Wrappers
 * \{ */

void ED_operatortypes_grease_pencil(void);
void ED_keymap_grease_pencil(struct wmKeyConfig *keyconf);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

namespace blender::ed::greasepencil {

bool editable_grease_pencil_poll(bContext *C);
bool editable_grease_pencil_point_selection_poll(bContext *C);
bool editable_grease_pencil_no_segment_selection_poll(bContext *C);

void create_blank(Main &bmain, Object &object, int frame_number);
void create_stroke(Main &bmain, Object &object, float4x4 matrix, int frame_number);
void create_suzanne(Main &bmain, Object &object, float4x4 matrix, const int frame_number);

/**
 * Structure for strokes (curves) converted to viewport 2D space.
 */
struct Stroke2DSpace {
  /* Stroke points in 2D space. */
  Vector<float2> points;
  /* Offset index of the stroke in GP points array. */
  int first_index;
  /* Bounding box of the stroke in 2D space. */
  rctf bbox;
};

/**
 * Collect all editable strokes in a GP object and convert them to
 * viewport 2D space.
 *
 * \param vc: The view context, retrieved by #ED_view3d_viewcontext_init.
 * \param grease_pencil: The edited Grease Pencil object.
 * \return A vector with the 2D representation of all editable strokes.
 */
Vector<Stroke2DSpace> editable_strokes_in_2d_space_get(ViewContext *vc,
                                                       GreasePencil *grease_pencil);

/**
 * Checks if a segment of 2 points is intersected by any of the given 2D strokes.
 *
 * \param segment_start: Start coordinates of the segment.
 * \param segment_end: End coordinates of the segment.
 * \param segment_stroke_index: Index of the segment stroke in the `strokes_2d` vector.
 * Used to avoid a false-positive self-intersecting result.
 * \param strokes_2d: A vector with the 2D representation of all editable strokes.
 * Obtained by #editable_strokes_in_2d_space_get.
 * \return: True if the segment is intersected by a stroke, else false.
 */
bool intersect_segment_strokes_2d(const float2 segment_start,
                                  const float2 segment_end,
                                  const int segment_stroke_index,
                                  const Vector<Stroke2DSpace> &strokes_2d);

/**
 * Get the selection state of all points in a Grease Pencil drawing.
 *
 * \param drawing: The Grease Pencil drawing.
 * \return A vector with the selection state of all points in curves.
 */
Vector<bool> point_selection_get(const GreasePencilDrawing *drawing);

/**
 * Expand the point selection in a Grease Pencil drawing to stroke segments.
 * A segment is the part of a stroke between other, intersecting strokes.
 *
 * \param stored_selection: The selection state of points before the selection change.
 * Obtained by #point_selection_get.
 * \param curves: The curves in a Grease Pencil drawing.
 * \param curve_offset: The curve offset index for the curves in the GP geometry array.
 * \param strokes_2d: A vector with the 2D representation of all editable strokes.
 * Obtained by #editable_strokes_in_2d_space_get.
 */
void expand_changed_selection_to_segments(Vector<bool> &stored_selection,
                                          bke::CurvesGeometry &curves,
                                          const int curve_offset,
                                          const Vector<Stroke2DSpace> &strokes_2d);

}  // namespace blender::ed::greasepencil
#endif
