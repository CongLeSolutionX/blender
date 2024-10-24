/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 * Operators for creating new Grease Pencil primitives (boxes, circles, ...).
 */

#include <fmt/format.h>

#include <algorithm>
#include <cstring>

#include "BKE_attribute.hh"
#include "BKE_brush.hh"
#include "BKE_colortools.hh"
#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_material.h"
#include "BKE_paint.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_enum_types.hh"

#include "DEG_depsgraph_query.hh"

#include "ED_grease_pencil.hh"
#include "ED_screen.hh"
#include "ED_space_api.hh"
#include "ED_view3d.hh"

#include "BLI_array_utils.hh"
#include "BLI_string.h"
#include "BLI_vector.hh"

#include "BLT_translation.hh"

#include "GPU_immediate.hh"
#include "GPU_state.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

namespace blender::ed::greasepencil {

enum class PrimitiveType : int8_t {
  Line = 0,
  Polyline = 1,
  Arc = 2,
  Curve = 3,
  Box = 4,
  Circle = 5,
  Semicircle = 6,
};

enum class OperatorMode : int8_t {
  Idle = 0,
  Extruding = 1,
  /* Set the active control point to the mouse. */
  Grab = 2,
  /* Move the active control point. */
  Drag = 3,
  /* Move all control points. */
  DragAll = 4,
  /* Rotate all control points. */
  RotateAll = 5,
  /* Scale all control points. */
  ScaleAll = 6,
};

enum class ControlPointType : int8_t {
  /* The points that are at the end of segments. */
  JoinPoint = 0,
  /* The points inside of the segments not including the end points. */
  HandlePoint = 1,
};

enum class ModalKeyMode : int8_t {
  Cancel = 1,
  Confirm,
  Extrude,
  Panning,
  Grab,
  Rotate,
  Scale,
  IncreaseSubdivision,
  DecreaseSubdivision,
  Flip,
  Quad,
};

static constexpr float ui_primary_point_draw_size_px = 8.0f;
static constexpr float ui_secondary_point_draw_size_px = 5.0f;
static constexpr float ui_tertiary_point_draw_size_px = 3.0f;
static constexpr float ui_point_hit_size_px = 20.0f;
static constexpr float ui_point_max_hit_size_px = 600.0f;
static constexpr float ui_point_secondary_max_hit_size_px = 200.0f;

/* These three points are only used for `Circle` type. */
static constexpr int control_point_first = 0;
static constexpr int control_point_center = 1;
static constexpr int control_point_last = 2;

/* These points are only used for `Box` type. */
/*
 * Clockwise from nw to w.
 *
 * +-----------+
 * |nw   n   ne|
 * |           |
 * |w  center e|
 * |           |
 * |sw   s   se|
 * +-----------+
 *
 */
static constexpr int box_____nw = 0;
static constexpr int box______n = 1;
static constexpr int box_____ne = 2;
static constexpr int box______e = 3;
static constexpr int box_____se = 4;
static constexpr int box______s = 5;
static constexpr int box_____sw = 6;
static constexpr int box______w = 7;
static constexpr int box_center = 8;

struct PrimitiveToolOperation {
  ARegion *region;
  /* For drawing preview loop. */
  void *draw_handle;
  ViewContext vc;

  int segments;
  Vector<float3> control_points;
  /* Store the control points temporally. */
  Vector<float3> temp_control_points;
  int temp_segments;

  PrimitiveType type;
  int subdivision;
  float4x4 projection;
  /* Helper class to project screen space coordinates to 3D. */
  DrawingPlacement placement;

  bke::greasepencil::Drawing *drawing;
  BrushGpencilSettings *settings;
  std::optional<ColorGeometry4f> vertex_color;
  std::optional<ColorGeometry4f> fill_color;
  int material_index;
  float softness;
  Brush *brush;
  float4x2 texture_space;

  OperatorMode mode;
  float2 start_position_2d;
  int active_control_point_index;
  int last_active_control_point_index;

  float2 start_drag_position_2d;
  float2 end_drag_position_2d;
  bool reverse_extrude;
  bool flip_segment;
  bool quad_mode;

  ViewOpsData *vod;
};

static float2 calc_circumcenter_2d(float2 a, float2 b, float2 c)
{
  const float2 ba = b - a;
  const float2 ca = c - a;
  const float dba = math::dot(ba, ba);
  const float dca = math::dot(ca, ca);
  const float bc1 = ba[1] * dca - ca[1] * dba;
  const float bc2 = -ba[0] * dca + ca[0] * dba;
  const float cba = ba[1] * ca[0] - ca[1] * ba[0];
  const float2 center = float2(a[0] + 0.5f * bc1 / cba, a[1] + 0.5f * bc2 / cba);
  return center;
}

static float calc_min_angle(const float2 a, const float2 b)
{
  const float angle = math::abs(math::abs(angle_v2v2(a, b)) - math::numbers::pi);
  return math::numbers::pi - angle;
}

static float2 rotate_point(const float2 p,
                           const float angle,
                           const float2 origin,
                           const float scale = 1.0f);
static float2 rotate_point(const float2 p,
                           const float angle,
                           const float2 origin,
                           const float scale)
{
  const float2 dif = (p - origin);
  const float c = math::cos(angle);
  const float s = math::sin(angle);
  const float2x2 rot = float2x2(float2(c, s), float2(-s, c));
  return rot * dif * scale + origin;
}

static void set_control_point(PrimitiveToolOperation &ptd,
                              const int active_index,
                              const float2 point)
{
  ptd.control_points[active_index] = ptd.placement.project(point);
}

static float2 point_2d_from_temp_index(const PrimitiveToolOperation &ptd, const int active_index)
{
  return ED_view3d_project_float_v2_m4(
      ptd.vc.region, ptd.temp_control_points[active_index], ptd.projection);
}

static float2 point_2d_from_index(const PrimitiveToolOperation &ptd, const int active_index)
{
  return ED_view3d_project_float_v2_m4(
      ptd.vc.region, ptd.control_points[active_index], ptd.projection);
}

static int control_points_per_segment(const PrimitiveToolOperation &ptd)
{
  switch (ptd.type) {
    case PrimitiveType::Polyline:
    case PrimitiveType::Line: {
      return 1;
    }
    case PrimitiveType::Semicircle:
    case PrimitiveType::Circle:
    case PrimitiveType::Arc: {
      return 2;
    }
    case PrimitiveType::Curve: {
      return 3;
    }
    case PrimitiveType::Box: {
      return 8;
    }
  }

  BLI_assert_unreachable();
  return 0;
}

static bool primitive_is_cyclic(const PrimitiveToolOperation &ptd)
{
  return ELEM(ptd.type, PrimitiveType::Circle, PrimitiveType::Box);
}

static bool primitive_is_multi_segment(const PrimitiveToolOperation &ptd)
{
  return !primitive_is_cyclic(ptd);
}

static ControlPointType get_control_point_type(const PrimitiveToolOperation &ptd, const int point)
{
  BLI_assert(point != -1);
  if (ptd.type == PrimitiveType::Box) {
    if (ELEM(point, box______n, box______e, box______s, box______w, box_center)) {
      return ControlPointType::JoinPoint;
    }
    else {
      return ControlPointType::HandlePoint;
    }
  }

  if (primitive_is_cyclic(ptd)) {
    return ControlPointType::JoinPoint;
  }

  const int num_shared_points = control_points_per_segment(ptd);
  if (math::mod(point, num_shared_points) == 0) {
    return ControlPointType::JoinPoint;
  }
  return ControlPointType::HandlePoint;
}

static void control_point_colors_and_sizes(const PrimitiveToolOperation &ptd,
                                           MutableSpan<ColorGeometry4f> colors,
                                           MutableSpan<float> sizes)
{
  ColorGeometry4f color_gizmo_primary;
  ColorGeometry4f color_gizmo_secondary;
  ColorGeometry4f color_gizmo_a;
  ColorGeometry4f color_gizmo_b;
  UI_GetThemeColor4fv(TH_GIZMO_PRIMARY, color_gizmo_primary);
  UI_GetThemeColor4fv(TH_GIZMO_SECONDARY, color_gizmo_secondary);
  UI_GetThemeColor4fv(TH_GIZMO_A, color_gizmo_a);
  UI_GetThemeColor4fv(TH_GIZMO_B, color_gizmo_b);

  const float size_primary = ui_primary_point_draw_size_px;
  const float size_secondary = ui_secondary_point_draw_size_px;
  const float size_tertiary = ui_tertiary_point_draw_size_px;

  if (ptd.segments == 0) {
    colors.fill(color_gizmo_primary);
    sizes.fill(size_primary);
    return;
  }

  if (ptd.type == PrimitiveType::Box) {
    colors.fill(color_gizmo_primary);
    sizes.fill(size_primary);

    for (const int i : colors.index_range()) {
      const ControlPointType control_point_type = get_control_point_type(ptd, i);

      if (control_point_type == ControlPointType::JoinPoint) {
        colors[i] = color_gizmo_b;
        sizes[i] = size_tertiary;
      }
    }

    /* Set the center point's color. */
    colors[box_center] = color_gizmo_b;
    sizes[box_center] = size_secondary;
  }
  else if (ptd.type == PrimitiveType::Circle) {
    colors.fill(color_gizmo_primary);
    sizes.fill(size_primary);

    /* Set the center point's color. */
    colors[control_point_center] = color_gizmo_b;
    sizes[control_point_center] = size_secondary;
  }
  else {
    colors.fill(color_gizmo_secondary);
    sizes.fill(size_secondary);

    for (const int i : colors.index_range()) {
      const ControlPointType control_point_type = get_control_point_type(ptd, i);

      if (control_point_type == ControlPointType::JoinPoint) {
        colors[i] = color_gizmo_b;
        sizes[i] = size_tertiary;
      }
    }

    colors.last() = color_gizmo_primary;
    sizes.last() = size_primary;

    if (ELEM(ptd.type, PrimitiveType::Line, PrimitiveType::Polyline)) {
      colors.last(1) = color_gizmo_secondary;
      sizes.last(1) = size_primary;
    }
  }

  const int active_index = ptd.active_control_point_index;
  if (active_index != -1) {
    sizes[active_index] *= 1.5;
    colors[active_index] = math::interpolate(colors[active_index], color_gizmo_a, 0.5f);
  }
}

static void draw_control_points(PrimitiveToolOperation &ptd, const bool drag_line)
{
  GPUVertFormat *format3d = immVertexFormat();
  const uint pos3d = GPU_vertformat_attr_add(format3d, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
  const uint col3d = GPU_vertformat_attr_add(format3d, "color", GPU_COMP_F32, 4, GPU_FETCH_FLOAT);
  const uint siz3d = GPU_vertformat_attr_add(format3d, "size", GPU_COMP_F32, 1, GPU_FETCH_FLOAT);
  immBindBuiltinProgram(GPU_SHADER_3D_POINT_VARYING_SIZE_VARYING_COLOR);

  const int total_points = drag_line ? ptd.control_points.size() + 1 : ptd.control_points.size();

  GPU_program_point_size(true);
  immBegin(GPU_PRIM_POINTS, total_points);

  Array<ColorGeometry4f> colors(ptd.control_points.size());
  Array<float> sizes(ptd.control_points.size());
  control_point_colors_and_sizes(ptd, colors, sizes);

  for (const int point : ptd.control_points.index_range()) {
    const float3 point3d = ptd.control_points[point];
    const ColorGeometry4f color = colors[point];
    const float size = sizes[point];

    immAttr4f(col3d, color[0], color[1], color[2], color[3]);
    immAttr1f(siz3d, size * 2.0f);
    immVertex3fv(pos3d, point3d);
  }

  if (drag_line) {
    const float3 point3d = ptd.placement.project(ptd.start_drag_position_2d);
    ColorGeometry4f color;
    UI_GetThemeColor4fv(TH_GIZMO_B, color);

    immAttr4f(col3d, color[0], color[1], color[2], color[3]);
    immAttr1f(siz3d, ui_secondary_point_draw_size_px);
    immVertex3fv(pos3d, point3d);
  }

  immEnd();
  immUnbindProgram();
  GPU_program_point_size(false);

  if (drag_line) {
    const float3 p1 = ptd.placement.project(ptd.start_drag_position_2d);
    const float3 p2 = ptd.placement.project(ptd.end_drag_position_2d);

    GPUVertFormat *format3d = immVertexFormat();
    const uint pos3d = GPU_vertformat_attr_add(format3d, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
    immBindBuiltinProgram(GPU_SHADER_3D_LINE_DASHED_UNIFORM_COLOR);

    immUniformColor4f(0.0f, 0.0f, 0.0f, 0.5f);

    immBegin(GPU_PRIM_LINES, 2);
    immVertex3fv(pos3d, p1);
    immVertex3fv(pos3d, p2);
    immEnd();
    immUnbindProgram();
  }
}

static void grease_pencil_primitive_draw(const bContext * /*C*/, ARegion * /*region*/, void *arg)
{
  PrimitiveToolOperation &ptd = *reinterpret_cast<PrimitiveToolOperation *>(arg);
  const bool drag_line = ELEM(ptd.mode,
                              OperatorMode::ScaleAll,
                              OperatorMode::Grab,
                              OperatorMode::Drag,
                              OperatorMode::RotateAll,
                              OperatorMode::DragAll);
  draw_control_points(ptd, drag_line);
}

static void grease_pencil_primitive_save(PrimitiveToolOperation &ptd)
{
  ptd.temp_segments = ptd.segments;
  ptd.temp_control_points.resize(ptd.control_points.size());
  array_utils::copy(ptd.control_points.as_span(), ptd.temp_control_points.as_mutable_span());
}

static void grease_pencil_primitive_load(PrimitiveToolOperation &ptd)
{
  ptd.segments = ptd.temp_segments;
  ptd.control_points.resize(ptd.temp_control_points.size());
  array_utils::copy(ptd.temp_control_points.as_span(), ptd.control_points.as_mutable_span());
}

static void primitive_calulate_curve_positions(PrimitiveToolOperation &ptd,
                                               Span<float2> control_points,
                                               MutableSpan<float2> new_positions)
{
  const int subdivision = ptd.subdivision;
  const int new_points_num = new_positions.size();

  if (ptd.segments == 0) {
    new_positions.fill(control_points.last());
    return;
  }

  switch (ptd.type) {
    case PrimitiveType::Line:
    case PrimitiveType::Polyline: {
      for (const int i : new_positions.index_range().drop_back(1)) {
        const float t = math::mod(i / float(subdivision + 1), 1.0f);
        const int point = int(i / (subdivision + 1));
        const int point_next = point + 1;
        new_positions[i] = math::interpolate(control_points[point], control_points[point_next], t);
      }
      new_positions.last() = control_points.last();
      return;
    }
    case PrimitiveType::Semicircle: {
      const int num_shared_points = control_points_per_segment(ptd);
      const int num_segments = ptd.segments;
      for (const int segment_i : IndexRange(num_segments)) {
        const float2 A = control_points[num_shared_points * segment_i + 0];
        const float2 CP = control_points[num_shared_points * segment_i + 1];
        const float2 B = control_points[num_shared_points * segment_i + 2];
        const float2 MP = math::midpoint(A, B);
        const float2 AB = A - B;
        const float angle_cp = angle_signed_v2v2(A - CP, AB);
        /* Draw straight line if points are virtually collinear or too close. */
        if (math::is_equal(A, B) || (math::abs(angle_cp) < 0.0075f) ||
            (math::numbers::pi - math::abs(angle_cp) < 0.0075f))
        {
          for (const int i : IndexRange(subdivision + 1)) {
            const float t = i / float(subdivision + 1);
            new_positions[i + segment_i * (subdivision + 1)] = math::interpolate(A, B, t);
          }
        }
        else {
          const float2 center = calc_circumcenter_2d(A, B, CP);
          const float radius = math::distance(A, center);
          const float angle_offset = angle_signed_v2v2(A - center, float2(1.0f, 0.0f));
          const float dist_cp_mp = math::distance(CP, MP);
          const float dist_a_mp = math::distance(A, MP);
          const float flip = (angle_cp >= 0.0f) ^ (dist_a_mp > dist_cp_mp) ? 1.0f : -1.0f;

          float angle = calc_min_angle(B - center, A - center);
          /* Use larger angle if CP is beyond reference radius. */
          if (dist_cp_mp >= dist_a_mp) {
            angle -= 2.0f * math::numbers::pi;
          }

          const float theta_step = angle / float(subdivision + 1) * flip;
          for (const int i : IndexRange(subdivision + 1)) {
            const float t = theta_step * i + angle_offset;
            new_positions[i + segment_i * (subdivision + 1)] = center +
                                                               radius * float2(cos(t), sin(t));
          }
        }
      }
      new_positions.last() = control_points.last();
      return;
    }
    case PrimitiveType::Arc: {
      const int num_shared_points = control_points_per_segment(ptd);
      const int num_segments = ptd.segments;
      for (const int segment_i : IndexRange(num_segments)) {
        const float2 A = control_points[num_shared_points * segment_i + 0];
        const float2 B = control_points[num_shared_points * segment_i + 1];
        const float2 C = control_points[num_shared_points * segment_i + 2];
        for (const int i : IndexRange(subdivision + 1)) {
          const float t = i / float(subdivision + 1);
          const float2 AB = math::interpolate(A, B, t);
          const float2 BC = math::interpolate(B, C, t);
          new_positions[i + segment_i * (subdivision + 1)] = math::interpolate(AB, BC, t);
        }
      }
      new_positions.last() = control_points.last();
      return;
    }
    case PrimitiveType::Curve: {
      const int num_shared_points = control_points_per_segment(ptd);
      const int num_segments = ptd.segments;

      for (const int segment_i : IndexRange(num_segments)) {
        const float2 A = control_points[num_shared_points * segment_i + 0];
        const float2 B = control_points[num_shared_points * segment_i + 1];
        const float2 C = control_points[num_shared_points * segment_i + 2];
        const float2 D = control_points[num_shared_points * segment_i + 3];
        for (const int i : IndexRange(subdivision + 1)) {
          const float t = i / float(subdivision + 1);
          const float2 AB = math::interpolate(A, B, t);
          const float2 BC = math::interpolate(B, C, t);
          const float2 CD = math::interpolate(C, D, t);
          const float2 ABBC = math::interpolate(AB, BC, t);
          const float2 BCCD = math::interpolate(BC, CD, t);
          new_positions[i + segment_i * (subdivision + 1)] = math::interpolate(ABBC, BCCD, t);
        }
      }
      new_positions.last() = control_points.last();
      return;
    }
    case PrimitiveType::Circle: {
      const float2 center = control_points[control_point_center];
      const float2 offset = control_points[control_point_first] - center;
      for (const int i : new_positions.index_range()) {
        const float t = i / float(new_points_num);
        const float a = t * math::numbers::pi * 2.0f;
        new_positions[i] = offset * float2(sinf(a), cosf(a)) + center;
      }
      return;
    }
    case PrimitiveType::Box: {
      const float2 center = control_points[box_center];
      const float2 nw = control_points[box_____nw];
      const float2 ne = control_points[box_____ne];
      const float2 se = control_points[box_____se];
      const float2 sw = control_points[box_____sw];
      /* Update dependent control poonts. */
      ptd.control_points[box______e] = ptd.placement.project(math::midpoint(ne, se));
      ptd.control_points[box______w] = ptd.placement.project(math::midpoint(nw, sw));
      ptd.control_points[box______n] = ptd.placement.project(math::midpoint(nw, ne));
      ptd.control_points[box______s] = ptd.placement.project(math::midpoint(sw, se));
      const float2 center_of_mass = (nw + ne + sw + se) / 4.0f;
      ptd.control_points[box_center] = ptd.placement.project(center_of_mass);

      const float2 corners[4] = {nw, ne, se, sw};
      for (const int i : new_positions.index_range()) {
        const float t = math::mod(i / float(subdivision + 1), 1.0f);
        const int point = int(i / (subdivision + 1));
        const int point_next = math::mod(point + 1, 4);
        new_positions[i] = math::interpolate(corners[point], corners[point_next], t);
      }
      return;
    }
  }
}

static void primitive_calulate_curve_positions_2d(PrimitiveToolOperation &ptd,
                                                  MutableSpan<float2> new_positions)
{
  Array<float2> control_points_2d(ptd.control_points.size());
  for (const int i : ptd.control_points.index_range()) {
    control_points_2d[i] = point_2d_from_index(ptd, i);
  }

  primitive_calulate_curve_positions(ptd, control_points_2d, new_positions);
}

static int grease_pencil_primitive_curve_points_number(PrimitiveToolOperation &ptd)
{
  const int subdivision = ptd.subdivision;

  switch (ptd.type) {
    case PrimitiveType::Polyline:
    case PrimitiveType::Curve:
    case PrimitiveType::Line:
    case PrimitiveType::Semicircle:
    case PrimitiveType::Circle:
    case PrimitiveType::Arc: {
      const int join_points = ptd.segments + 1;
      return join_points + subdivision * ptd.segments;
      break;
    }
    case PrimitiveType::Box: {
      return 4 + subdivision * 4;
      break;
    }
  }

  BLI_assert_unreachable();
  return 0;
}

/* Attributes that are defined explicitly and should not be copied from original geometry. */
static Set<std::string> skipped_attribute_ids(const PrimitiveToolOperation &ptd,
                                              const bke::AttrDomain domain)
{
  switch (domain) {
    case bke::AttrDomain::Point:
      if (ptd.vertex_color) {
        return {"position", "radius", "opacity", "vertex_color"};
      }
      else {
        return {"position", "radius", "opacity"};
      }
    case bke::AttrDomain::Curve:
      if (ptd.fill_color) {
        return {"curve_type",
                "material_index",
                "cyclic",
                "softness",
                "start_cap",
                "end_cap",
                "fill_color"};
      }
      else {
        return {"curve_type", "material_index", "cyclic", "softness", "start_cap", "end_cap"};
      }
    default:
      return {};
  }
  return {};
}

static void grease_pencil_primitive_update_curves(PrimitiveToolOperation &ptd)
{
  bke::CurvesGeometry &curves = ptd.drawing->strokes_for_write();

  const int last_points_num = curves.points_by_curve()[curves.curves_range().last()].size();

  const int new_points_num = grease_pencil_primitive_curve_points_number(ptd);

  curves.resize(curves.points_num() - last_points_num + new_points_num, curves.curves_num());
  curves.offsets_for_write().last() = curves.points_num();
  const IndexRange curve_points = curves.points_by_curve()[curves.curves_range().last()];

  MutableSpan<float3> positions_3d = curves.positions_for_write().slice(curve_points);
  Array<float2> positions_2d(new_points_num);

  primitive_calulate_curve_positions_2d(ptd, positions_2d);
  ptd.placement.project(positions_2d, positions_3d);

  MutableSpan<float> new_radii = ptd.drawing->radii_for_write().slice(curve_points);
  MutableSpan<float> new_opacities = ptd.drawing->opacities_for_write().slice(curve_points);

  if (ptd.vertex_color) {
    ptd.drawing->vertex_colors_for_write().slice(curve_points).fill(*ptd.vertex_color);
  }

  const ToolSettings *ts = ptd.vc.scene->toolsettings;
  const GP_Sculpt_Settings *gset = &ts->gp_sculpt;

  for (const int point : curve_points.index_range()) {
    float pressure = 1.0f;
    /* Apply pressure curve. */
    if (gset->flag & GP_SCULPT_SETT_FLAG_PRIMITIVE_CURVE) {
      const float t = point / float(new_points_num - 1);
      pressure = BKE_curvemapping_evaluateF(gset->cur_primitive, 0, t);
    }

    const float radius = ed::greasepencil::radius_from_input_sample(ptd.vc.rv3d,
                                                                    ptd.region,
                                                                    ptd.brush,
                                                                    pressure,
                                                                    positions_3d[point],
                                                                    ptd.placement.to_world_space(),
                                                                    ptd.settings);
    const float opacity = ed::greasepencil::opacity_from_input_sample(
        pressure, ptd.brush, ptd.settings);

    new_radii[point] = radius;
    new_opacities[point] = opacity;
  }

  /* Initialize the rest of the attributes with default values. */
  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
  bke::fill_attribute_range_default(
      attributes,
      bke::AttrDomain::Point,
      bke::attribute_filter_from_skip_ref(skipped_attribute_ids(ptd, bke::AttrDomain::Point)),
      curve_points);
  bke::fill_attribute_range_default(
      attributes,
      bke::AttrDomain::Curve,
      bke::attribute_filter_from_skip_ref(skipped_attribute_ids(ptd, bke::AttrDomain::Curve)),
      curves.curves_range().take_back(1));

  ptd.drawing->tag_topology_changed();
  ptd.drawing->set_texture_matrices({ptd.texture_space},
                                    IndexRange::from_single(curves.curves_range().last()));
}

static void grease_pencil_primitive_init_curves(PrimitiveToolOperation &ptd)
{
  /* Resize the curves geometry so there is one more curve with a single point. */
  bke::CurvesGeometry &curves = ptd.drawing->strokes_for_write();
  const int num_old_points = curves.points_num();
  curves.resize(curves.points_num() + 1, curves.curves_num() + 1);
  curves.offsets_for_write().last(1) = num_old_points;

  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
  bke::SpanAttributeWriter<int> materials = attributes.lookup_or_add_for_write_span<int>(
      "material_index", bke::AttrDomain::Curve);
  bke::SpanAttributeWriter<bool> cyclic = attributes.lookup_or_add_for_write_span<bool>(
      "cyclic", bke::AttrDomain::Curve);
  bke::SpanAttributeWriter<float> softness = attributes.lookup_or_add_for_write_span<float>(
      "softness", bke::AttrDomain::Curve);

  /* Only set the attribute if the type is not the default or if it already exists. */
  if (ptd.settings->caps_type != GP_STROKE_CAP_TYPE_ROUND || attributes.contains("start_cap")) {
    bke::SpanAttributeWriter<int8_t> start_caps = attributes.lookup_or_add_for_write_span<int8_t>(
        "start_cap", bke::AttrDomain::Curve);
    start_caps.span.last() = ptd.settings->caps_type;
    start_caps.finish();
  }

  if (ptd.settings->caps_type != GP_STROKE_CAP_TYPE_ROUND || attributes.contains("end_cap")) {
    bke::SpanAttributeWriter<int8_t> end_caps = attributes.lookup_or_add_for_write_span<int8_t>(
        "end_cap", bke::AttrDomain::Curve);
    end_caps.span.last() = ptd.settings->caps_type;
    end_caps.finish();
  }

  const bool is_cyclic = primitive_is_cyclic(ptd);
  cyclic.span.last() = is_cyclic;
  materials.span.last() = ptd.material_index;
  softness.span.last() = ptd.softness;

  if (ptd.fill_color) {
    ptd.drawing->fill_colors_for_write().last() = *ptd.fill_color;
  }

  cyclic.finish();
  materials.finish();
  softness.finish();

  curves.curve_types_for_write().last() = CURVE_TYPE_POLY;
  curves.update_curve_types();

  /* Initialize the rest of the attributes with default values. */
  bke::fill_attribute_range_default(
      attributes,
      bke::AttrDomain::Point,
      bke::attribute_filter_from_skip_ref(skipped_attribute_ids(ptd, bke::AttrDomain::Point)),
      curves.points_range().take_back(1));
  bke::fill_attribute_range_default(
      attributes,
      bke::AttrDomain::Curve,
      bke::attribute_filter_from_skip_ref(skipped_attribute_ids(ptd, bke::AttrDomain::Curve)),
      curves.curves_range().take_back(1));

  grease_pencil_primitive_update_curves(ptd);
}

static void grease_pencil_primitive_undo_curves(PrimitiveToolOperation &ptd)
{
  bke::CurvesGeometry &curves = ptd.drawing->strokes_for_write();
  curves.remove_curves(IndexMask({curves.curves_range().last(), 1}), {});
  ptd.drawing->tag_topology_changed();
}

/* Helper: Draw status message while the user is running the operator. */
static void grease_pencil_primitive_status_indicators(bContext *C,
                                                      wmOperator *op,
                                                      PrimitiveToolOperation &ptd)
{
  std::string header;

  switch (ptd.type) {
    case PrimitiveType::Line: {
      header += RPT_("Line: ");
      break;
    }
    case (PrimitiveType::Polyline): {
      header += RPT_("Polyline: ");
      break;
    }
    case (PrimitiveType::Box): {
      header += RPT_("Rectangle: ");
      break;
    }
    case (PrimitiveType::Circle): {
      header += RPT_("Circle: ");
      break;
    }
    case (PrimitiveType::Semicircle): {
      header += RPT_("Semicircle: ");
      break;
    }
    case (PrimitiveType::Arc): {
      header += RPT_("Arc: ");
      break;
    }
    case (PrimitiveType::Curve): {
      header += RPT_("Curve: ");
      break;
    }
  }

  auto get_modal_key_str = [&](ModalKeyMode id) {
    return WM_modalkeymap_operator_items_to_string(op->type, int(id), true).value_or("");
  };

  header += fmt::format(IFACE_("{}: confirm, {}: cancel, Shift: align"),
                        get_modal_key_str(ModalKeyMode::Confirm),
                        get_modal_key_str(ModalKeyMode::Cancel));

  header += fmt::format(IFACE_(", {}/{}: adjust subdivisions: {}"),
                        get_modal_key_str(ModalKeyMode::IncreaseSubdivision),
                        get_modal_key_str(ModalKeyMode::DecreaseSubdivision),
                        int(ptd.subdivision));

  if (ptd.segments == 1) {
    header += IFACE_(", Alt: center");
  }

  if (primitive_is_multi_segment(ptd)) {
    header += fmt::format(IFACE_(", Shift: constrain"));
    header += fmt::format(IFACE_(", Ctrl: snap to points"));
    header += fmt::format(IFACE_(", {}: extrude"), get_modal_key_str(ModalKeyMode::Extrude));
    if (ptd.type == PrimitiveType::Semicircle) {
      header += fmt::format(IFACE_(", {}: flip"), get_modal_key_str(ModalKeyMode::Flip));
    }
  }

  if (ptd.type == PrimitiveType::Box) {
    header += fmt::format(IFACE_(", {}: square/quad mode"), get_modal_key_str(ModalKeyMode::Quad));
  }

  header += fmt::format(IFACE_(", {}: grab, {}: rotate, {}: scale"),
                        get_modal_key_str(ModalKeyMode::Grab),
                        get_modal_key_str(ModalKeyMode::Rotate),
                        get_modal_key_str(ModalKeyMode::Scale));

  ED_workspace_status_text(C, header.c_str());
}

static void grease_pencil_primitive_update_view(bContext *C, PrimitiveToolOperation &ptd)
{
  GreasePencil *grease_pencil = static_cast<GreasePencil *>(ptd.vc.obact->data);

  DEG_id_tag_update(&grease_pencil->id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GEOM | ND_DATA, grease_pencil);

  ED_region_tag_redraw(ptd.region);
}

/* Invoke handler: Initialize the operator. */
static int grease_pencil_primitive_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  int return_value = ed::greasepencil::grease_pencil_draw_operator_invoke(C, op, false);
  if (return_value != OPERATOR_RUNNING_MODAL) {
    return return_value;
  }

  /* If in tools region, wait till we get to the main (3D-space)
   * region before allowing drawing to take place. */
  op->flag |= OP_IS_MODAL_CURSOR_REGION;

  wmWindow *win = CTX_wm_window(C);

  /* Set cursor to indicate modal. */
  WM_cursor_modal_set(win, WM_CURSOR_CROSS);

  ViewContext vc = ED_view3d_viewcontext_init(C, CTX_data_depsgraph_pointer(C));

  /* Allocate new data. */
  PrimitiveToolOperation *ptd_pointer = MEM_new<PrimitiveToolOperation>(__func__);
  op->customdata = ptd_pointer;

  PrimitiveToolOperation &ptd = *ptd_pointer;

  ptd.vc = vc;
  ptd.region = vc.region;
  View3D *view3d = CTX_wm_view3d(C);
  const float2 start_coords = float2(event->mval);

  GreasePencil *grease_pencil = static_cast<GreasePencil *>(vc.obact->data);

  /* Initialize helper class for projecting screen space coordinates. */
  DrawingPlacement placement = DrawingPlacement(
      *vc.scene, *vc.region, *view3d, *vc.obact, grease_pencil->get_active_layer());
  if (placement.use_project_to_surface()) {
    placement.cache_viewport_depths(CTX_data_depsgraph_pointer(C), vc.region, view3d);
  }
  else if (placement.use_project_to_nearest_stroke()) {
    placement.cache_viewport_depths(CTX_data_depsgraph_pointer(C), vc.region, view3d);
    placement.set_origin_to_nearest_stroke(start_coords);
  }

  ptd.placement = placement;

  ptd.vod = ED_view3d_navigation_init(C, nullptr);

  ptd.start_position_2d = start_coords;
  ptd.subdivision = RNA_int_get(op->ptr, "subdivision");
  ptd.type = PrimitiveType(RNA_enum_get(op->ptr, "type"));
  const float3 pos = ptd.placement.project(ptd.start_position_2d);
  ptd.segments = 0;
  ptd.control_points = Vector<float3>({pos});

  grease_pencil_primitive_save(ptd);

  ptd.mode = OperatorMode::Extruding;
  ptd.segments++;
  ptd.control_points.append_n_times(pos, control_points_per_segment(ptd));
  ptd.active_control_point_index = -1;
  ptd.last_active_control_point_index = -1;
  ptd.projection = ED_view3d_ob_project_mat_get(ptd.vc.rv3d, ptd.vc.obact);

  ptd.start_drag_position_2d = start_coords;
  ptd.end_drag_position_2d = start_coords;
  ptd.reverse_extrude = false;
  ptd.flip_segment = false;

  Paint *paint = &vc.scene->toolsettings->gp_paint->paint;
  ptd.brush = BKE_paint_brush(paint);
  if (ptd.brush->gpencil_settings == nullptr) {
    BKE_brush_init_gpencil_settings(ptd.brush);
  }
  ptd.settings = ptd.brush->gpencil_settings;

  BKE_curvemapping_init(ptd.settings->curve_sensitivity);
  BKE_curvemapping_init(ptd.settings->curve_strength);
  BKE_curvemapping_init(ptd.settings->curve_jitter);
  BKE_curvemapping_init(ptd.settings->curve_rand_pressure);
  BKE_curvemapping_init(ptd.settings->curve_rand_strength);
  BKE_curvemapping_init(ptd.settings->curve_rand_uv);
  BKE_curvemapping_init(ptd.settings->curve_rand_hue);
  BKE_curvemapping_init(ptd.settings->curve_rand_saturation);
  BKE_curvemapping_init(ptd.settings->curve_rand_value);

  ToolSettings *ts = vc.scene->toolsettings;
  GP_Sculpt_Settings *gset = &ts->gp_sculpt;
  /* Initialize pressure curve. */
  if (gset->flag & GP_SCULPT_SETT_FLAG_PRIMITIVE_CURVE) {
    BKE_curvemapping_init(ts->gp_sculpt.cur_primitive);
  }

  Material *material = BKE_grease_pencil_object_material_ensure_from_active_input_brush(
      CTX_data_main(C), vc.obact, ptd.brush);
  ptd.material_index = BKE_object_material_index_get(vc.obact, material);

  const bool use_vertex_color = (vc.scene->toolsettings->gp_paint->mode ==
                                 GPPAINT_FLAG_USE_VERTEXCOLOR);
  if (use_vertex_color) {
    ColorGeometry4f color_base;
    srgb_to_linearrgb_v3_v3(color_base, ptd.brush->rgb);
    color_base.a = ptd.settings->vertex_factor;
    ptd.vertex_color = ELEM(ptd.settings->vertex_mode, GPPAINT_MODE_STROKE, GPPAINT_MODE_BOTH) ?
                           std::make_optional(color_base) :
                           std::nullopt;
    ptd.fill_color = ELEM(ptd.settings->vertex_mode, GPPAINT_MODE_FILL, GPPAINT_MODE_BOTH) ?
                         std::make_optional(color_base) :
                         std::nullopt;
  }
  else {
    ptd.vertex_color = std::nullopt;
    ptd.fill_color = std::nullopt;
  }

  ptd.softness = 1.0 - ptd.settings->hardness;

  ptd.texture_space = ed::greasepencil::calculate_texture_space(
      vc.scene, ptd.region, ptd.start_position_2d, ptd.placement);

  BLI_assert(grease_pencil->has_active_layer());
  ptd.drawing = grease_pencil->get_editable_drawing_at(*grease_pencil->get_active_layer(),
                                                       vc.scene->r.cfra);

  grease_pencil_primitive_init_curves(ptd);
  grease_pencil_primitive_update_view(C, ptd);

  ptd.draw_handle = ED_region_draw_cb_activate(
      ptd.region->type, grease_pencil_primitive_draw, ptd_pointer, REGION_DRAW_POST_VIEW);

  /* Updates indicator in header. */
  grease_pencil_primitive_status_indicators(C, op, ptd);

  /* Add a modal handler for this operator. */
  WM_event_add_modal_handler(C, op);

  return OPERATOR_RUNNING_MODAL;
}

/* Exit and free memory. */
static void grease_pencil_primitive_exit(bContext *C, wmOperator *op)
{
  PrimitiveToolOperation *ptd = static_cast<PrimitiveToolOperation *>(op->customdata);

  /* Clear status message area. */
  ED_workspace_status_text(C, nullptr);

  WM_cursor_modal_restore(ptd->vc.win);

  /* Deactivate the extra drawing stuff in 3D-View. */
  ED_region_draw_cb_exit(ptd->region->type, ptd->draw_handle);

  ED_view3d_navigation_free(C, ptd->vod);

  grease_pencil_primitive_update_view(C, *ptd);

  MEM_delete<PrimitiveToolOperation>(ptd);
  /* Clear pointer. */
  op->customdata = nullptr;
}

static float2 snap_diagonals(float2 p)
{
  using namespace math;
  return sign(p) * float2(1.0f / numbers::sqrt2) * length(p);
}

/* Using Chebyshev distance instead of Euclidean. */
static float2 snap_diagonals_box(float2 p)
{
  using namespace math;
  return sign(p) * float2(std::max(abs(p[0]), abs(p[1])));
}

/* Snaps to the closest diagonal, horizontal or vertical. */
static float2 snap_8_angles(float2 p)
{
  using namespace math;
  /* sin(pi/8) or sin of 22.5 degrees.*/
  const float sin225 = 0.3826834323650897717284599840304f;
  return sign(p) * length(p) * normalize(sign(normalize(abs(p)) - sin225) + 1.0f);
}

/* Snaps to the closest 15 degrees. */
static float2 snap_12_angles(float2 p)
{
  using namespace math;
  /* sin(pi/12) or sin of 15 degrees.*/
  const float sin15 = 0.2588190451f;
  return sign(p) * length(p) * normalize(sign(normalize(abs(p)) - sin15) + 1.0f);
}

/* Snap angle to 15 degree intervals in radians. */
static float snap_rotation(float angle)
{
  using namespace math;
  const float period = float(numbers::pi / 12);
  return floor(safe_divide(angle, period)) * period;
}

static float2 snap_to_control_points(PrimitiveToolOperation &ptd,
                                     const float2 p,
                                     const float2 r_default,
                                     const int active_index)
{
  for (const int point_index : ptd.temp_control_points.index_range()) {
    if (point_index == active_index) {
      continue;
    }
    const float2 pos = point_2d_from_temp_index(ptd, point_index);
    const float distance_squared = math::distance_squared(pos, p);
    const float radius_sq = ui_point_hit_size_px * ui_point_hit_size_px;
    if (distance_squared <= radius_sq) {
      return pos;
    }
  }
  return r_default;
}

static void primitive_move_point(PrimitiveToolOperation &ptd,
                                 const int active_index,
                                 const float2 offset)
{
  const float2 point = point_2d_from_temp_index(ptd, active_index);
  ptd.control_points[active_index] = ptd.placement.project(point + offset);
}

static void primitive_scale_all(PrimitiveToolOperation &ptd,
                                const float2 start,
                                const float2 end,
                                const float2 origin)
{
  const float scale = math::length(end - origin) / math::length(start - origin) *
                      math::sign(math::dot(end - origin, start - origin));

  for (const int point_index : ptd.control_points.index_range()) {
    const float2 start_pos2 = point_2d_from_temp_index(ptd, point_index);

    const float2 pos2 = (start_pos2 - origin) * scale + origin;
    const float3 pos = ptd.placement.project(pos2);
    ptd.control_points[point_index] = pos;
  }
  ptd.start_drag_position_2d = origin;
  ptd.end_drag_position_2d = end;
}

static int box_index(PrimitiveToolOperation &ptd, const int index)
{
  return math::mod_periodic(index, control_points_per_segment(ptd));
}

static void box_move_cps(PrimitiveToolOperation &ptd,
                         const wmEvent *event,
                         const int active_index,
                         const float2 offset)
{
  /* Scale. */
  if (event->modifier & KM_SHIFT) {
    const float2 pos = point_2d_from_temp_index(ptd, active_index);
    const int origin_index = event->modifier & KM_ALT ? box_center :
                                                        box_index(ptd, active_index + 4);
    const float2 origin = point_2d_from_temp_index(ptd, origin_index);
    primitive_scale_all(ptd, pos, pos + offset, origin);
    return;
  }

  /* Individual corners and edges. */
  if (ptd.quad_mode) {
    if (ELEM(active_index, box_____ne, box_____sw, box_____nw, box_____se)) {
      primitive_move_point(ptd, active_index, offset);
      return;
    }
    else if (ELEM(active_index, box______n, box______s, box______w, box______e)) {
      primitive_move_point(ptd, box_index(ptd, active_index + 1), offset);
      primitive_move_point(ptd, box_index(ptd, active_index - 1), offset);
      return;
    }
  }
  else {
    /* Corners and edges. */
    if (ELEM(active_index, box_____ne, box_____sw, box_____nw, box_____se)) {
      float2 pos = point_2d_from_temp_index(ptd, active_index) + offset;
      const float2 opposite = point_2d_from_temp_index(ptd, box_index(ptd, active_index + 4));
      const float2 cw = point_2d_from_temp_index(ptd, box_index(ptd, active_index + 2));
      const float2 ccw = point_2d_from_temp_index(ptd, box_index(ptd, active_index - 2));
      float2 p_cw = math::project(pos, cw - opposite) - opposite;
      float2 p_ccw = math::project(pos, ccw - opposite) - opposite;
      closest_to_line_v2(p_cw, pos, cw, opposite);
      closest_to_line_v2(p_ccw, pos, ccw, opposite);
      set_control_point(ptd, box_index(ptd, active_index + 2), p_cw);
      set_control_point(ptd, box_index(ptd, active_index - 2), p_ccw);
      set_control_point(ptd, box_index(ptd, active_index), pos);
      return;
    }
    else if (ELEM(active_index, box______n, box______s, box______w, box______e)) {
      float2 pos = point_2d_from_temp_index(ptd, active_index) + offset;
      const float2 cw = point_2d_from_temp_index(ptd, box_index(ptd, active_index + 1));
      const float2 cw2 = point_2d_from_temp_index(ptd, box_index(ptd, active_index + 3));
      const float2 ccw = point_2d_from_temp_index(ptd, box_index(ptd, active_index - 1));
      const float2 ccw2 = point_2d_from_temp_index(ptd, box_index(ptd, active_index - 2));
      float2 p_cw = math::project(pos, cw - cw2) - cw2;
      float2 p_ccw = math::project(pos, ccw - ccw2) - ccw2;
      closest_to_line_v2(p_cw, pos, cw, cw2);
      closest_to_line_v2(p_ccw, pos, ccw, ccw2);
      set_control_point(ptd, box_index(ptd, active_index + 1), p_cw);
      set_control_point(ptd, box_index(ptd, active_index - 1), p_ccw);
      return;
    }
  }
}

static void primitive_rotate_cps(PrimitiveToolOperation &ptd,
                                 const float2 active_pos,
                                 const int offset)
{
  const int end_index = ptd.active_control_point_index;
  const int cp_index = end_index + offset;
  const int start_index = cp_index + offset;

  if (start_index < 0 || start_index >= ptd.control_points.size()) {
    return;
  }

  const float2 start = point_2d_from_temp_index(ptd, start_index);
  const float2 end = point_2d_from_temp_index(ptd, end_index);
  const float2 cp = point_2d_from_temp_index(ptd, cp_index);

  const float angle = angle_signed_v2v2(active_pos - start, end - start);
  const float scale = math::length(active_pos - start) / math::length(end - start);
  const float2 pos2 = rotate_point(cp, angle, start, scale);
  ptd.control_points[cp_index] = ptd.placement.project(pos2);
}

static float2 snap_2d_list(const float2 p, const float2 r_default, const Vector<float2> points)
{
  for (const float2 point : points) {
    const float distance_squared = math::distance_squared(point, p);
    const float radius_sq = ui_point_hit_size_px * ui_point_hit_size_px;
    if (distance_squared <= radius_sq) {
      return point;
    }
  }
  return r_default;
}

static void grease_pencil_primitive_extruding_update(PrimitiveToolOperation &ptd,
                                                     const wmEvent *event)
{
  using namespace math;
  const float2 start = ptd.start_position_2d;
  const float2 end = float2(event->mval);

  const float2 dif = end - start;
  float2 offset = dif;

  /* Constrain control points. */
  if (event->modifier & KM_SHIFT) {
    if (ptd.type == PrimitiveType::Box) {
      offset = snap_diagonals(dif);  // snap_diagonals_box
    }
    else if (ptd.type == PrimitiveType::Circle) {
      offset = snap_diagonals(dif);
    }
    else { /* Line, Polyline, Arc, Semicircle and Curve. */
      offset = snap_8_angles(dif);
    }
  }

  /* Snap control points. */
  if (event->modifier & KM_CTRL) {
    offset = snap_to_control_points(
        ptd, float2(event->mval), offset + start, ptd.temp_control_points.size());
    offset -= start;
  }

  offset *= 0.5f;

  float2 center = start + offset;

  if (event->modifier & KM_ALT && ptd.segments == 1) {
    center = start;
    offset *= 2.0f;
  }

  switch (ptd.type) {
    case PrimitiveType::Semicircle: {
      const float2 start = center - offset;
      const float2 end = center + offset;
      const float angle = numbers::pi * 0.5f + ptd.flip_segment * numbers::pi;
      const float2 cp = rotate_point(end, angle, center);

      ptd.control_points.last(2) = ptd.placement.project(start);
      ptd.control_points.last(1) = ptd.placement.project(cp);
      ptd.control_points.last(0) = ptd.placement.project(end);
      break;
    }
    case PrimitiveType::Box: {
      const float2 nw = center + offset;
      const float2 se = center - offset;
      const float2 sw = center + float2(offset.y, -offset.x);
      const float2 ne = center + float2(-offset.y, offset.x);
      ptd.control_points[box_____ne] = ptd.placement.project(ne);
      ptd.control_points[box_____sw] = ptd.placement.project(sw);
      ptd.control_points[box_____se] = ptd.placement.project(se);
      ptd.control_points[box_____nw] = ptd.placement.project(nw);
      break;
    }
    default: {
      const float3 start_pos = ptd.placement.project(center - offset);
      const float3 end_pos = ptd.placement.project(center + offset);

      const int number_control_points = control_points_per_segment(ptd);
      for (const int i : IndexRange(number_control_points + 1)) {
        ptd.control_points.last(i) = interpolate(
            end_pos, start_pos, (i / float(number_control_points)));
      }
      break;
    }
  }
}

static void grease_pencil_primitive_drag_all_update(PrimitiveToolOperation &ptd,
                                                    const wmEvent *event)
{
  const float2 start = ptd.start_position_2d;
  float2 end = float2(event->mval);

  /* Constrain control points. */
  if (event->modifier & KM_SHIFT) {
    end = snap_8_angles(end - start) + start;
  }

  const float2 dif = end - start;

  for (const int point_index : ptd.control_points.index_range()) {
    const float2 start_pos2 = point_2d_from_temp_index(ptd, point_index);

    float3 pos = ptd.placement.project(start_pos2 + dif);
    ptd.control_points[point_index] = pos;
  }
  ptd.start_drag_position_2d = start;
  ptd.end_drag_position_2d = end;
}

static void grease_pencil_primitive_grab_update(PrimitiveToolOperation &ptd, const wmEvent *event)
{
  BLI_assert(ptd.active_control_point_index != -1);
  float2 active_pos = float2(event->mval);
  const float2 start = point_2d_from_temp_index(ptd, ptd.active_control_point_index);

  if (primitive_is_multi_segment(ptd) && event->modifier & KM_SHIFT) {
    active_pos = snap_8_angles(active_pos - start) + start;
  }

  if (primitive_is_multi_segment(ptd) && event->modifier & KM_CTRL) {
    active_pos = snap_to_control_points(
        ptd, float2(event->mval), active_pos, ptd.active_control_point_index);
  }

  const float3 pos = ptd.placement.project(active_pos);
  ptd.control_points[ptd.active_control_point_index] = pos;
  ptd.start_drag_position_2d = start;
  ptd.end_drag_position_2d = active_pos;

  if (ptd.type == PrimitiveType::Semicircle) {
    primitive_rotate_cps(ptd, active_pos, -1);
    primitive_rotate_cps(ptd, active_pos, 1);
    return;
  }

  if (primitive_is_multi_segment(ptd)) {
    return;
  }

  if (ptd.type == PrimitiveType::Box) {
    /* If the center point is been grabbed, move all points. */
    if (ptd.active_control_point_index == box_center) {
      grease_pencil_primitive_drag_all_update(ptd, event);
      return;
    }
    else {
      const float3 pos = ptd.placement.project(active_pos);
      ptd.control_points[ptd.active_control_point_index] = pos;
      ptd.start_drag_position_2d = start;
      ptd.end_drag_position_2d = active_pos;
      float2 dif = active_pos - start;
      box_move_cps(ptd, event, ptd.active_control_point_index, dif);
      return;
    }
  }

  /* If the center point is been grabbed, move all points. */
  if (ptd.active_control_point_index == control_point_center) {
    grease_pencil_primitive_drag_all_update(ptd, event);
    return;
  }

  const int other_point = ptd.active_control_point_index == control_point_first ?
                              control_point_last :
                              control_point_first;

  /* Get the location of the other control point.*/
  const float2 other_point_2d = point_2d_from_temp_index(ptd, other_point);

  /* Set the center point to between the first and last point. */
  ptd.control_points[control_point_center] = ptd.placement.project(
      (other_point_2d + float2(event->mval)) / 2.0f);
}

static void grease_pencil_primitive_drag_update(PrimitiveToolOperation &ptd, const wmEvent *event)
{
  BLI_assert(ptd.active_control_point_index != -1);

  const float2 start = point_2d_from_temp_index(ptd, ptd.active_control_point_index);
  // const float2 start = ptd.start_position_2d;
  const float2 end = float2(event->mval);
  float2 active_pos = end;
  const float2 dif = end - start;
  ptd.start_drag_position_2d = start;
  ptd.end_drag_position_2d = active_pos;

  if (ptd.type == PrimitiveType::Box) {
    /* If the center point is been grabbed, move all points. */
    if (ptd.active_control_point_index == box_center) {
      grease_pencil_primitive_drag_all_update(ptd, event);
      return;
    }
    else {
      const float3 pos = ptd.placement.project(active_pos);
      ptd.control_points[ptd.active_control_point_index] = pos;
      ptd.start_drag_position_2d = start;
      ptd.end_drag_position_2d = active_pos;
      float2 dif = active_pos - start;
      box_move_cps(ptd, event, ptd.active_control_point_index, dif);
      return;
    }
  }

  /* Constrain control points. */
  if (event->modifier & KM_SHIFT) {
    if (ptd.type == PrimitiveType::Semicircle) {
      const float2 start = point_2d_from_temp_index(ptd, ptd.active_control_point_index + 1);
      const float2 end = point_2d_from_temp_index(ptd, ptd.active_control_point_index - 1);
      float2 mp = math::midpoint(start, end);
      float2 cp = rotate_point(end, math::numbers::pi * 0.5f, mp);
      float2 cp2 = rotate_point(end, -math::numbers::pi * 0.5f, mp);
      const float lambda = closest_to_line_v2(active_pos, active_pos, cp, cp2);

      active_pos = math::interpolate(cp, cp2, math::clamp(lambda, 0.0f, 1.0f));
      Vector<float2> point_list;
      point_list.append(cp);
      point_list.append(cp2);
      point_list.append(mp);
      active_pos = snap_2d_list(active_pos, active_pos, point_list);
      const float3 pos = ptd.placement.project(active_pos);
      ptd.control_points[ptd.active_control_point_index] = pos;
      return;
    }
    else if (primitive_is_multi_segment(ptd)) {
      active_pos = snap_8_angles(end - start);
      const float3 pos = ptd.placement.project(active_pos);
      ptd.control_points[ptd.active_control_point_index] = pos;
      return;
    }
  }

  /* Snap control points. */
  if (event->modifier & KM_CTRL) {
    active_pos = snap_to_control_points(
        ptd, float2(event->mval), float2(event->mval), ptd.active_control_point_index);
    const float3 pos = ptd.placement.project(active_pos);
    ptd.control_points[ptd.active_control_point_index] = pos;

    return;
  }

  const float3 pos = ptd.placement.project(start + dif);
  ptd.control_points[ptd.active_control_point_index] = pos;
}

static float2 primitive_center_of_mass(const PrimitiveToolOperation &ptd)
{
  if (ptd.type == PrimitiveType::Box) {
    return point_2d_from_temp_index(ptd, box_center);
  }

  if (ptd.type == PrimitiveType::Circle) {
    return point_2d_from_temp_index(ptd, control_point_center);
  }

  float2 center_of_mass = float2(0.0f, 0.0f);
  for (const int point_index : ptd.control_points.index_range()) {
    center_of_mass += point_2d_from_temp_index(ptd, point_index);
  }
  center_of_mass /= ptd.control_points.size();
  return center_of_mass;
}

static void grease_pencil_primitive_rotate_all_update(PrimitiveToolOperation &ptd,
                                                      const wmEvent *event)
{
  const float2 start = ptd.start_position_2d;
  const float2 end = float2(event->mval);

  const float2 center_of_mass = primitive_center_of_mass(ptd);

  const float2 end_ = end - center_of_mass;

  const float2 start_ = start - center_of_mass;
  float rotation = math::atan2(start_[0], start_[1]) - math::atan2(end_[0], end_[1]);

  /* Constrain to 15 degree intervals. */
  if (event->modifier & KM_SHIFT) {
    rotation = snap_rotation(rotation);
  }

  for (const int point_index : ptd.control_points.index_range()) {
    const float2 start_pos2 = point_2d_from_temp_index(ptd, point_index);
    const float2 pos2 = rotate_point(start_pos2, rotation, center_of_mass);
    const float3 pos = ptd.placement.project(pos2);
    ptd.control_points[point_index] = pos;
  }
  ptd.start_drag_position_2d = center_of_mass;
  ptd.end_drag_position_2d = end;
}

static void grease_pencil_primitive_scale_all_update(PrimitiveToolOperation &ptd,
                                                     const wmEvent *event)
{
  const float2 start = ptd.start_position_2d;
  const float2 end = float2(event->mval);
  const float2 center_of_mass = primitive_center_of_mass(ptd);
  primitive_scale_all(ptd, start, end, center_of_mass);
}

static int primitive_check_ui_hover(PrimitiveToolOperation &ptd, const wmEvent *event)
{
  const int ui_hit_point_size = (ptd.type == PrimitiveType::Semicircle) ?
                                    ui_point_secondary_max_hit_size_px :
                                    ui_point_max_hit_size_px;

  float closest_distance_squared = std::numeric_limits<float>::max();
  int closest_point = -1;

  for (const int i : ptd.control_points.index_range()) {
    const int point = (ptd.control_points.size() - 1) - i;
    const float2 pos_proj = point_2d_from_index(ptd, point);
    const float radius_sq = ui_point_hit_size_px * ui_point_hit_size_px;
    const float distance_squared = math::distance_squared(pos_proj, float2(event->mval));
    /* If the mouse is over a control point. */
    if (distance_squared <= radius_sq) {
      ptd.last_active_control_point_index = point;
      return point;
    }

    const ControlPointType control_point_type = get_control_point_type(ptd, point);

    /* Save the closest handle point. */
    if (distance_squared < closest_distance_squared &&
        control_point_type == ControlPointType::HandlePoint &&
        distance_squared < ui_hit_point_size * ui_hit_point_size)
    {
      closest_point = point;
      closest_distance_squared = distance_squared;
    }
  }

  if (closest_point != -1) {
    ptd.last_active_control_point_index = closest_point;
    return closest_point;
  }

  return -1;
}

static void grease_pencil_primitive_cursor_update(bContext *C,
                                                  PrimitiveToolOperation &ptd,
                                                  const wmEvent *event)
{
  wmWindow *win = CTX_wm_window(C);

  if (ptd.mode != OperatorMode::Idle) {
    WM_cursor_modal_set(win, WM_CURSOR_CROSS);
    return;
  }

  const int ui_id = primitive_check_ui_hover(ptd, event);
  ptd.reverse_extrude = (ui_id == 0);
  ptd.active_control_point_index = ui_id;
  if (ui_id == -1) {
    if (ptd.type == PrimitiveType::Polyline) {
      WM_cursor_modal_set(win, WM_CURSOR_CROSS);
      return;
    }

    WM_cursor_modal_set(win, WM_CURSOR_HAND);
    return;
  }

  WM_cursor_modal_set(win, WM_CURSOR_NSEW_SCROLL);
  return;
}

static void primitive_reverse(PrimitiveToolOperation &ptd)
{
  if (ptd.reverse_extrude == true) {
    std::reverse(ptd.control_points.begin(), ptd.control_points.end());
    std::reverse(ptd.temp_control_points.begin(), ptd.temp_control_points.end());
    ptd.reverse_extrude = false;
    ptd.flip_segment = !ptd.flip_segment;
  }
}

static int grease_pencil_primitive_event_modal_map(bContext *C,
                                                   wmOperator *op,
                                                   PrimitiveToolOperation &ptd,
                                                   const wmEvent *event)
{
  switch (event->val) {
    case int(ModalKeyMode::Cancel): {
      grease_pencil_primitive_undo_curves(ptd);
      grease_pencil_primitive_exit(C, op);

      return OPERATOR_CANCELLED;
    }
    case int(ModalKeyMode::Confirm): {
      grease_pencil_primitive_exit(C, op);

      return OPERATOR_FINISHED;
    }
    case int(ModalKeyMode::Extrude): {
      if (ptd.mode == OperatorMode::Idle && ELEM(ptd.type,
                                                 PrimitiveType::Line,
                                                 PrimitiveType::Arc,
                                                 PrimitiveType::Curve,
                                                 PrimitiveType::Semicircle))
      {
        ptd.mode = OperatorMode::Extruding;

        primitive_reverse(ptd);

        grease_pencil_primitive_save(ptd);

        ptd.start_position_2d = ED_view3d_project_float_v2_m4(
            ptd.vc.region, ptd.control_points.last(), ptd.projection);
        const float3 pos = ptd.placement.project(ptd.start_position_2d);

        const int number_control_points = control_points_per_segment(ptd);
        ptd.control_points.append_n_times(pos, number_control_points);
        ptd.active_control_point_index = -1;
        ptd.segments++;

        return OPERATOR_RUNNING_MODAL;
      }

      if (ptd.type == PrimitiveType::Polyline &&
          ELEM(ptd.mode, OperatorMode::Idle, OperatorMode::Extruding))
      {
        ptd.mode = OperatorMode::Extruding;
        grease_pencil_primitive_save(ptd);

        ptd.start_position_2d = ED_view3d_project_float_v2_m4(
            ptd.vc.region, ptd.control_points.last(), ptd.projection);
        ptd.active_control_point_index = -1;
        const float3 pos = ptd.placement.project(float2(event->mval));

        /* If we have only two points and they're the same then don't extrude new a point. */
        if (ptd.segments == 1 &&
            math::distance_squared(ptd.control_points.first(), ptd.control_points.last()) == 0.0f)
        {
          ptd.control_points.last() = pos;
        }
        else {
          ptd.control_points.append(pos);
          ptd.segments++;
        }

        return OPERATOR_RUNNING_MODAL;
      }

      return OPERATOR_RUNNING_MODAL;
    }
    case int(ModalKeyMode::Flip): {
      if (ELEM(ptd.type, PrimitiveType::Semicircle)) {
        if (ptd.mode == OperatorMode::Extruding) {
          ptd.flip_segment = !ptd.flip_segment;
        }
        if (ptd.mode == OperatorMode::Idle) {
          const int active_index = (ptd.last_active_control_point_index != -1) ?
                                       ptd.last_active_control_point_index :
                                       ptd.active_control_point_index;

          if (active_index != -1 &&
              get_control_point_type(ptd, active_index) == ControlPointType::HandlePoint)
          {
            const float2 cp = point_2d_from_index(ptd, active_index);
            const float2 a = point_2d_from_index(ptd, active_index + 1);
            const float2 b = point_2d_from_index(ptd, active_index - 1);
            const float2 fcp = rotate_point(cp, math::numbers::pi, math::midpoint(a, b));
            set_control_point(ptd, active_index, fcp);
            grease_pencil_primitive_save(ptd);
          }
        }
      }
      return OPERATOR_RUNNING_MODAL;
    }
    case int(ModalKeyMode::Quad): {
      if (ELEM(ptd.type, PrimitiveType::Box)) {
        ptd.quad_mode = !ptd.quad_mode;
      }
      return OPERATOR_RUNNING_MODAL;
    }
    case int(ModalKeyMode::Grab): {
      if (ptd.mode == OperatorMode::Idle) {
        ptd.start_position_2d = float2(event->mval);
        ptd.mode = OperatorMode::DragAll;

        grease_pencil_primitive_save(ptd);
      }
      return OPERATOR_RUNNING_MODAL;
    }
    case int(ModalKeyMode::Rotate): {
      if (ptd.mode == OperatorMode::Idle) {
        ptd.start_position_2d = float2(event->mval);
        ptd.mode = OperatorMode::RotateAll;

        grease_pencil_primitive_save(ptd);
      }
      return OPERATOR_RUNNING_MODAL;
    }
    case int(ModalKeyMode::Scale): {
      if (ptd.mode == OperatorMode::Idle) {
        ptd.start_position_2d = float2(event->mval);
        ptd.mode = OperatorMode::ScaleAll;

        grease_pencil_primitive_save(ptd);
      }
      return OPERATOR_RUNNING_MODAL;
    }
    case int(ModalKeyMode::IncreaseSubdivision): {
      if (event->val != KM_RELEASE) {
        ptd.subdivision++;
        RNA_int_set(op->ptr, "subdivision", ptd.subdivision);
      }
      return OPERATOR_RUNNING_MODAL;
    }
    case int(ModalKeyMode::DecreaseSubdivision): {
      if (event->val != KM_RELEASE) {
        ptd.subdivision--;
        ptd.subdivision = std::max(ptd.subdivision, 0);
        RNA_int_set(op->ptr, "subdivision", ptd.subdivision);
      }
      return OPERATOR_RUNNING_MODAL;
    }
  }

  return OPERATOR_RUNNING_MODAL;
}

static int grease_pencil_primitive_mouse_event(PrimitiveToolOperation &ptd, const wmEvent *event)
{
  if (event->val == KM_RELEASE && ELEM(ptd.mode,
                                       OperatorMode::Grab,
                                       OperatorMode::Drag,
                                       OperatorMode::Extruding,
                                       OperatorMode::DragAll,
                                       OperatorMode::RotateAll,
                                       OperatorMode::ScaleAll))
  {
    ptd.mode = OperatorMode::Idle;
    return OPERATOR_RUNNING_MODAL;
  }

  if (ptd.mode == OperatorMode::Idle && event->val == KM_PRESS) {
    const int ui_id = primitive_check_ui_hover(ptd, event);
    ptd.active_control_point_index = ui_id;
    ptd.reverse_extrude = (ui_id == 0);
    if (ui_id == -1) {
      if (ptd.type != PrimitiveType::Polyline) {
        ptd.start_position_2d = float2(event->mval);
        ptd.mode = OperatorMode::DragAll;

        grease_pencil_primitive_save(ptd);

        return OPERATOR_RUNNING_MODAL;
      }
    }
    else {
      const ControlPointType control_point_type = get_control_point_type(ptd, ui_id);

      if (control_point_type == ControlPointType::JoinPoint) {
        ptd.start_position_2d = point_2d_from_index(ptd, ptd.active_control_point_index);
        ptd.mode = OperatorMode::Grab;

        grease_pencil_primitive_save(ptd);
      }
      else if (control_point_type == ControlPointType::HandlePoint) {
        ptd.start_position_2d = float2(event->mval);
        ptd.mode = OperatorMode::Drag;

        grease_pencil_primitive_save(ptd);
      }

      return OPERATOR_RUNNING_MODAL;
    }
  }

  if (ptd.type == PrimitiveType::Polyline && ptd.mode == OperatorMode::Idle &&
      event->val == KM_PRESS)
  {
    ptd.mode = OperatorMode::Extruding;
    grease_pencil_primitive_save(ptd);

    ptd.start_position_2d = ED_view3d_project_float_v2_m4(
        ptd.vc.region, ptd.control_points[ptd.active_control_point_index], ptd.projection);
    const float3 pos = ptd.placement.project(float2(event->mval));

    /* If we have only two points and they're the same then don't extrude new a point. */
    if (ptd.segments == 1 &&
        math::distance_squared(ptd.control_points.first(), ptd.control_points.last()) == 0.0f)
    {
      ptd.control_points.last() = pos;
    }
    else {
      ptd.control_points.append(pos);
      ptd.segments++;
    }
  }

  return OPERATOR_RUNNING_MODAL;
}

static void grease_pencil_primitive_operator_update(PrimitiveToolOperation &ptd,
                                                    const wmEvent *event)
{
  switch (ptd.mode) {
    case OperatorMode::Extruding: {
      grease_pencil_primitive_extruding_update(ptd, event);
      break;
    }
    case OperatorMode::Grab: {
      grease_pencil_primitive_grab_update(ptd, event);
      break;
    }
    case OperatorMode::Drag: {
      grease_pencil_primitive_drag_update(ptd, event);
      break;
    }
    case OperatorMode::DragAll: {
      grease_pencil_primitive_drag_all_update(ptd, event);
      break;
    }
    case OperatorMode::ScaleAll: {
      grease_pencil_primitive_scale_all_update(ptd, event);
      break;
    }
    case OperatorMode::RotateAll: {
      grease_pencil_primitive_rotate_all_update(ptd, event);
      break;
    }
    case OperatorMode::Idle: {
      /* Do nothing. */
      break;
    }
  }
}

/* Modal handler: Events handling during interactive part. */
static int grease_pencil_primitive_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  PrimitiveToolOperation &ptd = *reinterpret_cast<PrimitiveToolOperation *>(op->customdata);

  ptd.projection = ED_view3d_ob_project_mat_get(ptd.vc.rv3d, ptd.vc.obact);
  grease_pencil_primitive_cursor_update(C, ptd, event);

  if (event->type == EVT_MODAL_MAP) {
    const int return_val = grease_pencil_primitive_event_modal_map(C, op, ptd, event);
    if (return_val != OPERATOR_RUNNING_MODAL) {
      return return_val;
    }
  }

  switch (event->type) {
    case LEFTMOUSE: {
      const int return_val = grease_pencil_primitive_mouse_event(ptd, event);
      if (return_val != OPERATOR_RUNNING_MODAL) {
        return return_val;
      }

      break;
    }
    case RIGHTMOUSE: {
      if (event->val != KM_PRESS) {
        break;
      }

      if (ptd.mode == OperatorMode::Idle) {
        grease_pencil_primitive_undo_curves(ptd);
        grease_pencil_primitive_exit(C, op);

        return OPERATOR_CANCELLED;
      }
      else {
        ptd.mode = OperatorMode::Idle;

        grease_pencil_primitive_load(ptd);
        break;
      }
    }
  }

  /* Updating is done every event not just `MOUSEMOVE`. */
  grease_pencil_primitive_operator_update(ptd, event);
  grease_pencil_primitive_update_curves(ptd);

  /* Updates indicator in header. */
  grease_pencil_primitive_status_indicators(C, op, ptd);
  grease_pencil_primitive_update_view(C, ptd);

  /* Still running... */
  return OPERATOR_RUNNING_MODAL;
}

/* Cancel handler. */
static void grease_pencil_primitive_cancel(bContext *C, wmOperator *op)
{
  /* This is just a wrapper around exit() */
  grease_pencil_primitive_exit(C, op);
}

static void grease_pencil_primitive_common_props(wmOperatorType *ot,
                                                 const int default_subdiv,
                                                 const PrimitiveType default_type)
{
  static const EnumPropertyItem grease_pencil_primitive_type[] = {
      {int(PrimitiveType::Box), "BOX", 0, "Box", ""},
      {int(PrimitiveType::Line), "LINE", 0, "Line", ""},
      {int(PrimitiveType::Polyline), "POLYLINE", 0, "Polyline", ""},
      {int(PrimitiveType::Circle), "CIRCLE", 0, "Circle", ""},
      {int(PrimitiveType::Arc), "ARC", 0, "Arc", ""},
      {int(PrimitiveType::Curve), "CURVE", 0, "Curve", ""},
      {int(PrimitiveType::Semicircle), "SEMICIRCLE", 0, "Semicircle", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  prop = RNA_def_int(ot->srna,
                     "subdivision",
                     default_subdiv,
                     0,
                     INT_MAX,
                     "Subdivisions",
                     "Number of subdivisions per segment",
                     0,
                     INT_MAX);
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);

  RNA_def_enum(
      ot->srna, "type", grease_pencil_primitive_type, int(default_type), "Type", "Type of shape");
}

static void GREASE_PENCIL_OT_primitive_line(wmOperatorType *ot)
{
  /* Identifiers. */
  ot->name = "Grease Pencil Line Shape";
  ot->idname = "GREASE_PENCIL_OT_primitive_line";
  ot->description = "Create predefined Grease Pencil stroke lines";

  /* Callbacks. */
  ot->invoke = grease_pencil_primitive_invoke;
  ot->modal = grease_pencil_primitive_modal;
  ot->cancel = grease_pencil_primitive_cancel;

  /* Flags. */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_BLOCKING;

  /* Properties and Flags. */
  grease_pencil_primitive_common_props(ot, 6, PrimitiveType::Line);
}

static void GREASE_PENCIL_OT_primitive_polyline(wmOperatorType *ot)
{
  /* Identifiers. */
  ot->name = "Grease Pencil Polyline Shape";
  ot->idname = "GREASE_PENCIL_OT_primitive_polyline";
  ot->description = "Create predefined Grease Pencil stroke polylines";

  /* Callbacks. */
  ot->invoke = grease_pencil_primitive_invoke;
  ot->modal = grease_pencil_primitive_modal;
  ot->cancel = grease_pencil_primitive_cancel;

  /* Flags. */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_BLOCKING;

  /* Properties. */
  grease_pencil_primitive_common_props(ot, 6, PrimitiveType::Polyline);
}

static void GREASE_PENCIL_OT_primitive_arc(wmOperatorType *ot)
{
  /* Identifiers. */
  ot->name = "Grease Pencil Arc Shape";
  ot->idname = "GREASE_PENCIL_OT_primitive_arc";
  ot->description = "Create predefined Grease Pencil stroke arcs";

  /* Callbacks. */
  ot->invoke = grease_pencil_primitive_invoke;
  ot->modal = grease_pencil_primitive_modal;
  ot->cancel = grease_pencil_primitive_cancel;

  /* Flags. */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_BLOCKING;

  /* Properties. */
  grease_pencil_primitive_common_props(ot, 62, PrimitiveType::Arc);
}

static void GREASE_PENCIL_OT_primitive_curve(wmOperatorType *ot)
{
  /* Identifiers. */
  ot->name = "Grease Pencil Curve Shape";
  ot->idname = "GREASE_PENCIL_OT_primitive_curve";
  ot->description = "Create predefined Grease Pencil stroke curve shapes";

  /* Callbacks. */
  ot->invoke = grease_pencil_primitive_invoke;
  ot->modal = grease_pencil_primitive_modal;
  ot->cancel = grease_pencil_primitive_cancel;

  /* Flags. */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_BLOCKING;

  /* Properties. */
  grease_pencil_primitive_common_props(ot, 62, PrimitiveType::Curve);
}

static void GREASE_PENCIL_OT_primitive_semicircle(wmOperatorType *ot)
{
  /* Identifiers. */
  ot->name = "Grease Pencil Semicircle Shape";
  ot->idname = "GREASE_PENCIL_OT_primitive_semicircle";
  ot->description = "Create predefined grease pencil stroke semicircle shapes";

  /* Callbacks. */
  ot->invoke = grease_pencil_primitive_invoke;
  ot->modal = grease_pencil_primitive_modal;
  ot->cancel = grease_pencil_primitive_cancel;

  /* Flags. */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_BLOCKING;

  /* Properties. */
  grease_pencil_primitive_common_props(ot, 62, PrimitiveType::Semicircle);
}

static void GREASE_PENCIL_OT_primitive_box(wmOperatorType *ot)
{
  /* Identifiers. */
  ot->name = "Grease Pencil Box Shape";
  ot->idname = "GREASE_PENCIL_OT_primitive_box";
  ot->description = "Create predefined Grease Pencil stroke boxes";

  /* Callbacks. */
  ot->invoke = grease_pencil_primitive_invoke;
  ot->modal = grease_pencil_primitive_modal;
  ot->cancel = grease_pencil_primitive_cancel;

  /* Flags. */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_BLOCKING;

  /* Properties. */
  grease_pencil_primitive_common_props(ot, 3, PrimitiveType::Box);
}

static void GREASE_PENCIL_OT_primitive_circle(wmOperatorType *ot)
{
  /* Identifiers. */
  ot->name = "Grease Pencil Circle Shape";
  ot->idname = "GREASE_PENCIL_OT_primitive_circle";
  ot->description = "Create predefined Grease Pencil stroke circles";

  /* Callbacks. */
  ot->invoke = grease_pencil_primitive_invoke;
  ot->modal = grease_pencil_primitive_modal;
  ot->cancel = grease_pencil_primitive_cancel;

  /* Flags. */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_BLOCKING;

  /* Properties. */
  grease_pencil_primitive_common_props(ot, 94, PrimitiveType::Circle);
}

}  // namespace blender::ed::greasepencil

void ED_operatortypes_grease_pencil_primitives()
{
  using namespace blender::ed::greasepencil;
  WM_operatortype_append(GREASE_PENCIL_OT_primitive_line);
  WM_operatortype_append(GREASE_PENCIL_OT_primitive_polyline);
  WM_operatortype_append(GREASE_PENCIL_OT_primitive_arc);
  WM_operatortype_append(GREASE_PENCIL_OT_primitive_curve);
  WM_operatortype_append(GREASE_PENCIL_OT_primitive_box);
  WM_operatortype_append(GREASE_PENCIL_OT_primitive_circle);
  WM_operatortype_append(GREASE_PENCIL_OT_primitive_semicircle);
}

void ED_primitivetool_modal_keymap(wmKeyConfig *keyconf)
{
  using namespace blender::ed::greasepencil;
  static const EnumPropertyItem modal_items[] = {
      {int(ModalKeyMode::Cancel), "CANCEL", 0, "Cancel", ""},
      {int(ModalKeyMode::Confirm), "CONFIRM", 0, "Confirm", ""},
      {int(ModalKeyMode::Panning), "PANNING", 0, "Panning", ""},
      {int(ModalKeyMode::Extrude), "EXTRUDE", 0, "Extrude", ""},
      {int(ModalKeyMode::Grab), "GRAB", 0, "Grab", ""},
      {int(ModalKeyMode::Rotate), "ROTATE", 0, "Rotate", ""},
      {int(ModalKeyMode::Scale), "SCALE", 0, "Scale", ""},
      {int(ModalKeyMode::Flip), "FLIP", 0, "Flip", ""},
      {int(ModalKeyMode::Quad), "QUAD", 0, "Quad", ""},
      {int(ModalKeyMode::IncreaseSubdivision),
       "INCREASE_SUBDIVISION",
       0,
       "Increase Subdivision",
       ""},
      {int(ModalKeyMode::DecreaseSubdivision),
       "DECREASE_SUBDIVISION",
       0,
       "Decrease Subdivision",
       ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  wmKeyMap *keymap = WM_modalkeymap_find(keyconf, "Primitive Tool Modal Map");

  /* This function is called for each space-type, only needs to add map once. */
  if (keymap && keymap->modal_items) {
    return;
  }

  keymap = WM_modalkeymap_ensure(keyconf, "Primitive Tool Modal Map", modal_items);

  WM_modalkeymap_assign(keymap, "GREASE_PENCIL_OT_primitive_line");
  WM_modalkeymap_assign(keymap, "GREASE_PENCIL_OT_primitive_polyline");
  WM_modalkeymap_assign(keymap, "GREASE_PENCIL_OT_primitive_arc");
  WM_modalkeymap_assign(keymap, "GREASE_PENCIL_OT_primitive_curve");
  WM_modalkeymap_assign(keymap, "GREASE_PENCIL_OT_primitive_box");
  WM_modalkeymap_assign(keymap, "GREASE_PENCIL_OT_primitive_circle");
  WM_modalkeymap_assign(keymap, "GREASE_PENCIL_OT_primitive_semicircle");
}
