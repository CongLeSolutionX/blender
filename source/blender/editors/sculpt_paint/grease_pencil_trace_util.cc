/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_color.hh"
#include "BLI_math_matrix.hh"

#include "BKE_curves.hh"

#include "IMB_imbuf.hh"
#include "potracelib.h"

#include "grease_pencil_trace_util.hh"

#include <iostream>

namespace blender::ed::image_trace {

using PathSegment = potrace_dpoint_t[3];

static int to_potrace(const TurnPolicy turn_policy)
{
  switch (turn_policy) {
    case TurnPolicy ::Foreground:
      return POTRACE_TURNPOLICY_BLACK;
    case TurnPolicy ::Background:
      return POTRACE_TURNPOLICY_WHITE;
    case TurnPolicy ::Left:
      return POTRACE_TURNPOLICY_LEFT;
    case TurnPolicy ::Right:
      return POTRACE_TURNPOLICY_RIGHT;
    case TurnPolicy ::Minority:
      return POTRACE_TURNPOLICY_MINORITY;
    case TurnPolicy ::Majority:
      return POTRACE_TURNPOLICY_MAJORITY;
    case TurnPolicy ::Random:
      return POTRACE_TURNPOLICY_RANDOM;
  }
  BLI_assert_unreachable();
  return POTRACE_TURNPOLICY_MINORITY;
}

Bitmap *create_bitmap(const int2 &size)
{
#ifdef WITH_POTRACE
  constexpr int BM_WORDSIZE = int(sizeof(potrace_word));
  constexpr int BM_WORDBITS = 8 * BM_WORDSIZE;

  /* Number of words per scanline. */
  const int32_t dy = (size.x + BM_WORDBITS - 1) / BM_WORDBITS;

  potrace_bitmap_t *bm = (potrace_bitmap_t *)MEM_mallocN(sizeof(potrace_bitmap_t), __func__);
  if (!bm) {
    return nullptr;
  }
  bm->w = size.x;
  bm->h = size.y;
  bm->dy = dy;
  bm->map = (potrace_word *)calloc(size.y, dy * BM_WORDSIZE);
  if (!bm->map) {
    free(bm);
    return nullptr;
  }

  return bm;
#else
  UNUSED_VARS(size);
  return nullptr;
#endif
}

void free_bitmap(Bitmap *bm)
{
#ifdef WITH_POTRACE
  if (bm != nullptr) {
    free(bm->map);
  }
  MEM_SAFE_FREE(bm);
#else
  UNUSED_VARS(bm);
#endif
}

ImBuf *bitmap_to_image(const Bitmap &bm)
{
  constexpr int BM_WORDSIZE = int(sizeof(potrace_word));
  constexpr int BM_WORDBITS = 8 * BM_WORDSIZE;
  constexpr potrace_word BM_HIBIT = potrace_word(1) << (BM_WORDBITS - 1);

  const int2 size = {bm.w, bm.h};
  const uint imb_flag = IB_rect;
  ImBuf *ibuf = IMB_allocImBuf(size.x, size.y, 32, imb_flag);
  BLI_assert(ibuf->byte_buffer.data != nullptr);

  const int num_words = bm.dy * bm.h;
  const int words_per_scanline = bm.dy;
  const Span<potrace_word> words = {bm.map, num_words};
  MutableSpan<ColorGeometry4b> colors = {
      reinterpret_cast<ColorGeometry4b *>(ibuf->byte_buffer.data), ibuf->x * ibuf->y};
  threading::parallel_for(IndexRange(ibuf->y), 4096, [&](const IndexRange range) {
    for (const int y : range) {
      Span<potrace_word> scanline_words = words.slice(
          IndexRange(words_per_scanline * y, words_per_scanline));
      const MutableSpan<ColorGeometry4b> scanline_colors = colors.slice(
          IndexRange(y * ibuf->x, ibuf->x));
      for (uint32_t x = 0; x < ibuf->x; x++) {
        const potrace_word &word = scanline_words[x / BM_WORDBITS];
        const potrace_word mask = BM_HIBIT >> (x & (BM_WORDBITS - 1));
        scanline_colors[x] = ((word & mask) != 0 ? ColorGeometry4b(255, 0, 0, 255) :
                                                   ColorGeometry4b(0, 0, 255, 255));
      }
    }
  });

  return ibuf;
}

Trace *trace_bitmap(const TraceParams &params, Bitmap &bm)
{
  potrace_param_t *po_params = potrace_param_default();
  if (!po_params) {
    return nullptr;
  }
  po_params->turdsize = params.size_threshold;
  po_params->turnpolicy = to_potrace(params.turn_policy);
  po_params->alphamax = params.alpha_max;
  po_params->opticurve = params.optimize_curves;
  po_params->opttolerance = params.optimize_tolerance;

  potrace_state_t *st = potrace_trace(po_params, &bm);
  potrace_param_free(po_params);

  if (!st || st->status != POTRACE_STATUS_OK) {
    if (st) {
      potrace_state_free(st);
    }
    return nullptr;
  }
  return st;
}

void free_trace(Trace *trace)
{
  potrace_state_free(trace);
}
bke::CurvesGeometry trace_to_curves(const Trace &trace, const float4x4 &transform)
{

  return trace_to_curves(trace, [=](const int2 &pixel) {
    return math::transform_point(transform, float3(pixel.x, pixel.y, 0));
  });
}

bke::CurvesGeometry trace_to_curves(const Trace &trace,
                                    FunctionRef<float3(const int2 &)> pixel_to_position)
{
  auto project_pixel = [&](const potrace_dpoint_t &point) -> float3 {
    return pixel_to_position(int2(point.x, point.y));
  };

  ///* Find materials and create them if not found. */
  // BKE_object_material
  // int32_t mat_fill_idx = BKE_gpencil_material_find_index_by_name_prefix(ob, "Stroke");
  // int32_t mat_mask_idx = BKE_gpencil_material_find_index_by_name_prefix(ob, "Holdout");

  // const float default_color[4] = {0.0f, 0.0f, 0.0f, 1.0f};
  ///* Stroke and Fill material. */
  // if (mat_fill_idx == -1) {
  //   int32_t new_idx;
  //   Material *mat_gp = BKE_gpencil_object_material_new(bmain, ob, "Stroke", &new_idx);
  //   MaterialGPencilStyle *gp_style = mat_gp->gp_style;

  //  copy_v4_v4(gp_style->stroke_rgba, default_color);
  //  gp_style->flag |= GP_MATERIAL_STROKE_SHOW;
  //  gp_style->flag |= GP_MATERIAL_FILL_SHOW;
  //  mat_fill_idx = ob->totcol - 1;
  //}
  ///* Holdout material. */
  // if (mat_mask_idx == -1) {
  //   int32_t new_idx;
  //   Material *mat_gp = BKE_gpencil_object_material_new(bmain, ob, "Holdout", &new_idx);
  //   MaterialGPencilStyle *gp_style = mat_gp->gp_style;

  //  copy_v4_v4(gp_style->stroke_rgba, default_color);
  //  copy_v4_v4(gp_style->fill_rgba, default_color);
  //  gp_style->flag |= GP_MATERIAL_STROKE_SHOW;
  //  gp_style->flag |= GP_MATERIAL_FILL_SHOW;
  //  gp_style->flag |= GP_MATERIAL_IS_STROKE_HOLDOUT;
  //  gp_style->flag |= GP_MATERIAL_IS_FILL_HOLDOUT;
  //  mat_mask_idx = ob->totcol - 1;
  //}

  /* Count paths and points. */
  Vector<int> offsets;
  for (const potrace_path_t *path = trace.plist; path != nullptr; path = path->next) {
    const Span<int> path_tags = {path->curve.tag, path->curve.n};
    const Span<PathSegment> path_segments = {path->curve.c, path->curve.n};

    int point_num = 0;
    for (const int segment_i : path_segments.index_range()) {
      switch (path_tags[segment_i]) {
        case POTRACE_CORNER:
          point_num += 2;
          break;
        case POTRACE_CURVETO:
          point_num += 1;
          break;
        default:
          BLI_assert_unreachable();
          break;
      }
    }
    offsets.append(point_num);
  }
  /* Last element stores total size. */
  offsets.append(0);
  const OffsetIndices points_by_curve = offset_indices::accumulate_counts_to_offsets(offsets);
  if (points_by_curve.is_empty()) {
    return {};
  }

  bke::CurvesGeometry curves(points_by_curve.total_size(), points_by_curve.size());
  curves.offsets_for_write().copy_from(offsets);

  /* Construct all curves as Bezier curves. */
  curves.curve_types_for_write().fill(CURVE_TYPE_BEZIER);
  curves.update_curve_types();
  /* All trace curves are cyclic. */
  curves.cyclic_for_write().fill(true);

  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
  bke::SpanAttributeWriter<int> material_indices = attributes.lookup_or_add_for_write_span<int>(
      "material_index", bke::AttrDomain::Curve);
  MutableSpan<int8_t> handle_types_left = curves.handle_types_left_for_write();
  MutableSpan<int8_t> handle_types_right = curves.handle_types_right_for_write();
  MutableSpan<float3> handle_positions_left = curves.handle_positions_left_for_write();
  MutableSpan<float3> handle_positions_right = curves.handle_positions_right_for_write();
  MutableSpan<float3> positions = curves.positions_for_write();

  /* Draw each curve. */
  int curve_i = 0;
  for (const potrace_path_t *path = trace.plist; path != nullptr; path = path->next, ++curve_i) {
    const Span<int> path_tags = {path->curve.tag, path->curve.n};
    const Span<PathSegment> path_segments = {path->curve.c, path->curve.n};

    const IndexRange points = points_by_curve[curve_i];
    if (points.is_empty()) {
      continue;
    }

    material_indices.span[curve_i] = 0 /*path->sign == '+' ? mat_fill_idx : mat_mask_idx*/;

    /* Potrace stores the last 3 points of a bezier segment.
     * The start point is the last segment's end point. */
    int point_i = points.last();
    auto next_point = [&]() {
      point_i = (point_i == points.last() ? points.first() : point_i + 1);
    };

    for (const int segment_i : path_segments.index_range()) {
      const PathSegment &segment = path_segments[segment_i];
      switch (path_tags[segment_i]) {
        case POTRACE_CORNER:
          /* Potrace corners are formed by straight lines from the previous/next point.
           * segment[0] is unused, segment[1] is the corner position, segment[2] is the next point.
           */
          handle_types_right[point_i] = BEZIER_HANDLE_VECTOR;

          next_point();
          positions[point_i] = project_pixel(segment[1]);
          handle_types_left[point_i] = BEZIER_HANDLE_VECTOR;
          handle_types_right[point_i] = BEZIER_HANDLE_VECTOR;

          next_point();
          positions[point_i] = project_pixel(segment[2]);
          handle_types_left[point_i] = BEZIER_HANDLE_VECTOR;
          break;
        case POTRACE_CURVETO:
          /* segment[0] is the previous point's right-side handle, segment[1] is the next point's
           * left-side handle, segment[2] is the next point. */
          handle_types_right[point_i] = BEZIER_HANDLE_FREE;
          handle_positions_right[point_i] = project_pixel(segment[0]);

          next_point();
          positions[point_i] = project_pixel(segment[2]);
          handle_types_left[point_i] = BEZIER_HANDLE_FREE;
          handle_positions_left[point_i] = project_pixel(segment[1]);
          break;
        default:
          BLI_assert_unreachable();
          break;
      }
    }
  }

  material_indices.finish();
  curves.tag_topology_changed();
  curves.tag_positions_changed();
  curves.tag_radii_changed();

  /* Calculate handles for all corner points (vector handle type). */
  curves.calculate_bezier_auto_handles();

  return curves;
}

}  // namespace blender::ed::image_trace
