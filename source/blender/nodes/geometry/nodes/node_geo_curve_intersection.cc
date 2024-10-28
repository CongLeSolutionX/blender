/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>

#include "DNA_pointcloud_types.h"

#include "BKE_curves.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"

#include "BLI_kdopbvh.h"
#include "BLI_math_geom.h"
#include "BLI_task.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_rna_define.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_curve_intersection_cc {

/* Epsilon values for curve intersections and bvh tree. */
constexpr float curve_isect_eps = 0.000001f;
constexpr float curve_dot_eps = 0.000000001f;
constexpr float pi_2_f = math::numbers::pi * 0.5f;
constexpr float pi_2_f_eps = pi_2_f - 0.0001f;

enum class PairData {
  PointsOnly = 0,
  FullPair = 1,
  HalfPair = 2,
};

enum class IntersectionMode {
  Curve = 0,
  Plane = 1,
  Surface = 2,
  Curve_Project = 3,
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Curve").supported_type(GeometryComponent::Type::Curve);
  b.add_input<decl::Geometry>("Mesh")
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .make_available([](bNode &node) { node.custom1 = int16_t(IntersectionMode::Surface); });
  b.add_input<decl::Bool>("Self Intersections")
      .default_value(false)
      .description("Include self intersections");
  b.add_input<decl::Bool>("All Intersections")
      .default_value(true)
      .description("Include all intersections except self intersections");
  b.add_input<decl::Vector>("Direction")
      .default_value({0.0f, 0.0f, 1.0f})
      .make_available([](bNode &node) { node.custom1 = int16_t(IntersectionMode::Plane); })
      .description("Direction of orthographic plane");
  b.add_input<decl::Vector>("Center")
      .subtype(PROP_DISTANCE)
      .make_available([](bNode &node) { node.custom1 = int16_t(IntersectionMode::Plane); })
      .description("Center of plane");
  b.add_input<decl::Float>("Epsilon")
      .subtype(PROP_DISTANCE)
      .min(0.0f)
      .description("Epsilon for max distance between intersections");
  b.add_input<decl::Float>("Min Angle")
      .subtype(PROP_ANGLE)
      .min(0.0f)
      .max(pi_2_f)
      .description("Minimum shortest angle for intersections");

  b.add_output<decl::Geometry>("Points");
  b.add_output<decl::Int>("Curve Index").field_on_all();
  b.add_output<decl::Vector>("Direction")
      .field_on_all()
      .description(
          "The direction of the curve at the intersection point. For project mode, this is the "
          "projected direction");
  b.add_output<decl::Float>("Factor").field_on_all().description(
      "The portion of the spline's total length at the intersection point");
  b.add_output<decl::Float>("Length").field_on_all().description(
      "The distance along the spline at the intersection point");
  b.add_output<decl::Vector>("Normal")
      .field_on_all()
      .make_available([](bNode &node) { node.custom1 = int16_t(IntersectionMode::Surface); })
      .description("The normal of surface or plane intersection");
  b.add_output<decl::Vector>("Pair Position")
      .field_on_all()
      .make_available([](bNode &node) { node.custom1 = int16_t(IntersectionMode::Curve); })
      .description("Position of the oppposing pair point");
  b.add_output<decl::Vector>("Pair Direction")
      .field_on_all()
      .make_available([](bNode &node) { node.custom1 = int16_t(IntersectionMode::Curve); })
      .description(
          "Direction of the oppposing pair point. For project mode, this is the "
          "projected direction");
  b.add_output<decl::Bool>("Pair")
      .field_on_all()
      .make_available([](bNode &node) { node.custom1 = int16_t(IntersectionMode::Curve); })
      .description("If the intersection is one of a pair of matching intersections");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  const IntersectionMode mode = IntersectionMode(static_cast<const bNode *>(ptr->data)->custom1);
  uiItemR(layout, ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);
  if (ELEM(mode, IntersectionMode::Curve, IntersectionMode::Curve_Project)) {
    uiItemR(layout, ptr, "pair_data_mode", UI_ITEM_NONE, "", ICON_NONE);
  }
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = int16_t(IntersectionMode::Curve);
  node->custom2 = int16_t(PairData::PointsOnly);
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  const IntersectionMode mode = IntersectionMode(node->custom1);

  bNodeSocket *mesh = static_cast<bNodeSocket *>(node->inputs.first)->next;
  bNodeSocket *self = mesh->next;
  bNodeSocket *all = self->next;
  bNodeSocket *direction = all->next;
  bNodeSocket *plane_center = direction->next;
  bNodeSocket *distance = plane_center->next;

  const bool curve_mode = ELEM(mode, IntersectionMode::Curve, IntersectionMode::Curve_Project);

  bke::node_set_socket_availability(ntree, mesh, mode == IntersectionMode::Surface);
  bke::node_set_socket_availability(ntree, self, curve_mode);
  bke::node_set_socket_availability(ntree, all, curve_mode);
  bke::node_set_socket_availability(
      ntree, direction, ELEM(mode, IntersectionMode::Plane, IntersectionMode::Curve_Project));
  bke::node_set_socket_availability(ntree, plane_center, mode == IntersectionMode::Plane);
  bke::node_set_socket_availability(ntree, distance, mode == IntersectionMode::Curve);

  const bool use_paired_data = ELEM(
      PairData(node->custom2), PairData::FullPair, PairData::HalfPair);

  const bool points_only_mode = curve_mode && PairData(node->custom2) == PairData::PointsOnly;

  LISTBASE_FOREACH (bNodeSocket *, socket, &node->outputs) {
    if (STREQ(socket->name, "Curve Index")) {
      bke::node_set_socket_availability(ntree, socket, !points_only_mode);
    }
    if (STREQ(socket->name, "Direction")) {
      bke::node_set_socket_availability(ntree, socket, !points_only_mode);
    }
    if (STREQ(socket->name, "Factor")) {
      bke::node_set_socket_availability(ntree, socket, !points_only_mode);
    }
    if (STREQ(socket->name, "Length")) {
      bke::node_set_socket_availability(ntree, socket, !points_only_mode);
    }
    if (STREQ(socket->name, "Pair Position")) {
      bke::node_set_socket_availability(ntree, socket, curve_mode && use_paired_data);
    }
    if (STREQ(socket->name, "Pair Direction")) {
      bke::node_set_socket_availability(ntree, socket, curve_mode && use_paired_data);
    }
    if (STREQ(socket->name, "Pair")) {
      bke::node_set_socket_availability(
          ntree, socket, curve_mode && PairData(node->custom2) == PairData::FullPair);
    }
    if (STREQ(socket->name, "Normal")) {
      bke::node_set_socket_availability(ntree, socket, mode == IntersectionMode::Surface);
    }
  }
}

/* Attribute outputs. */
struct AttributeOutputs {
  std::optional<std::string> curve_index;
  std::optional<std::string> direction;
  std::optional<std::string> factor;
  std::optional<std::string> length;
  std::optional<std::string> normal;
  std::optional<std::string> pair_position;
  std::optional<std::string> pair_direction;
  std::optional<std::string> pair;
};

/* Store information from line intersection calculations. */
struct IntersectingLineInfo {
  float3 closest_ab;
  float3 closest_cd;
  float lambda_ab;
  float lambda_cd;
  bool is_intersection;
};

/* Store segment details. Start and End must be at the start for BVHTree search. */
struct Segment {
  float3 start;
  float3 end;
  float3 orig_start;
  float3 orig_end;
  float3 direction;
  float len_start;
  float len_end;
  float curve_length;
  int pos_index;
  int curve_index;
  bool is_cyclic_segment;
};

/* Store data that will be used for attributes and sorting. */
struct IntersectionData {
  Vector<float> sortkey;
  Vector<float3> position;
  Vector<int> curve_index;
  Vector<float3> direction;
  Vector<float> length;
  Vector<float> factor;
  Vector<float3> normal;
  Vector<float3> pair_position;
  Vector<float3> pair_direction;
  Vector<bool> pair;
};

using ThreadLocalData = threading::EnumerableThreadSpecific<IntersectionData>;

static void add_intersection_data(IntersectionData &data,
                                  const float3 position,
                                  const int curve_index,
                                  const float3 direction,
                                  const float length,
                                  const float curve_length,
                                  const float3 normal,
                                  const float3 pair_position,
                                  const float3 pair_direction,
                                  const bool pair,
                                  const AttributeOutputs &attribute_outputs)
{
  const float factor = math::safe_divide(length, curve_length);

  /* Create sortkey for index. Order by curve_index then factor. */
  const float sortkey = float(curve_index * 2) + factor;
  data.sortkey.append(sortkey);

  data.position.append(position);

  if (attribute_outputs.curve_index) {
    data.curve_index.append(curve_index);
  }
  if (attribute_outputs.length) {
    data.length.append(length);
  }
  if (attribute_outputs.factor) {
    data.factor.append(factor);
  }
  if (attribute_outputs.direction) {
    data.direction.append(direction);
  }
  if (attribute_outputs.normal) {
    data.normal.append(normal);
  }
  if (attribute_outputs.pair_position) {
    data.pair_position.append(pair_position);
  }
  if (attribute_outputs.pair_direction) {
    data.pair_direction.append(pair_direction);
  }
  if (attribute_outputs.pair) {
    data.pair.append(pair);
  }
}

static void gather_thread_storage(ThreadLocalData &thread_storage,
                                  IntersectionData &r_data,
                                  const AttributeOutputs &attribute_outputs)
{
  int64_t total_intersections = 0;
  for (const IntersectionData &local_data : thread_storage) {
    const int64_t local_size = local_data.position.size();
    BLI_assert(local_data.sortkey.size() == local_size);
    BLI_assert(attribute_outputs.curve_index && local_data.curve_index.size() == local_size);
    BLI_assert(attribute_outputs.length && local_data.length.size() == local_size);
    BLI_assert(attribute_outputs.factor && local_data.factor.size() == local_size);
    BLI_assert(attribute_outputs.direction && local_data.direction.size() == local_size);
    BLI_assert(attribute_outputs.normal && local_data.normal.size() == local_size);
    BLI_assert(attribute_outputs.pair_position && local_data.pair_position.size() == local_size);
    BLI_assert(attribute_outputs.pair_direction && local_data.pair_direction.size() == local_size);
    BLI_assert(attribute_outputs.pair && local_data.pair.size() == local_size);
    total_intersections += local_size;
  }
  const int64_t start_index = r_data.position.size();
  const int64_t new_size = start_index + total_intersections;
  r_data.position.reserve(new_size);
  r_data.sortkey.reserve(new_size);

  if (attribute_outputs.curve_index) {
    r_data.curve_index.reserve(new_size);
  }
  if (attribute_outputs.length) {
    r_data.length.reserve(new_size);
  }
  if (attribute_outputs.factor) {
    r_data.factor.reserve(new_size);
  }
  if (attribute_outputs.direction) {
    r_data.direction.reserve(new_size);
  }
  if (attribute_outputs.normal) {
    r_data.normal.reserve(new_size);
  }
  if (attribute_outputs.pair_position) {
    r_data.pair_position.reserve(new_size);
  }
  if (attribute_outputs.pair_direction) {
    r_data.pair_direction.reserve(new_size);
  }
  if (attribute_outputs.pair) {
    r_data.pair.reserve(new_size);
  }

  for (IntersectionData &local_data : thread_storage) {
    r_data.position.extend(local_data.position);
    r_data.sortkey.extend(local_data.sortkey);

    if (attribute_outputs.curve_index) {
      r_data.curve_index.extend(local_data.curve_index);
    };
    if (attribute_outputs.length) {
      r_data.length.extend(local_data.length);
    }
    if (attribute_outputs.factor) {
      r_data.factor.extend(local_data.factor);
    }
    if (attribute_outputs.direction) {
      r_data.direction.extend(local_data.direction);
    }
    if (attribute_outputs.normal) {
      r_data.normal.extend(local_data.normal);
    }
    if (attribute_outputs.pair_position) {
      r_data.pair_position.extend(local_data.pair_position);
    }
    if (attribute_outputs.pair_direction) {
      r_data.pair_direction.extend(local_data.pair_direction);
    }
    if (attribute_outputs.pair) {
      r_data.pair.extend(local_data.pair);
    }
  }
}

/* Minimum angle between two vectors in 0-PI/2 (90 degree) range. Option to test if one argument is
 * a face normal. */
static float calc_min_angle(const float3 an, const float3 bn, const bool is_face_normal)
{
  const float angle = math::abs(math::abs(angle_normalized_v3v3(an, bn)) - pi_2_f);
  return is_face_normal ? angle : pi_2_f - angle;
}

/* `isect_line_line_epsilon_v3` is too strict for checking parallel lines. This
 * version adds an epsilon to the parallel line check^. */
static int isect_line_line_epsilon_v3_loose(const float v1[3],
                                            const float v2[3],
                                            const float v3[3],
                                            const float v4[3],
                                            float r_i1[3],
                                            float r_i2[3],
                                            const float epsilon,
                                            const float dot_epsilon)
{
  float a[3], b[3], c[3], ab[3], cb[3];
  float d, div;
  sub_v3_v3v3(a, v2, v1);
  sub_v3_v3v3(b, v4, v3);

  cross_v3_v3v3(ab, a, b);
  div = dot_v3v3(ab, ab);

  /* ^Epsilon has been added here. */
  if (fabsf(div) <= dot_epsilon) {
    return 0;
  }

  d = dot_v3v3(c, ab);
  sub_v3_v3v3(c, v3, v1);

  /* test if the two lines are coplanar */
  if (UNLIKELY(fabsf(d) <= epsilon)) {
    cross_v3_v3v3(cb, c, b);

    mul_v3_fl(a, dot_v3v3(cb, ab) / div);
    add_v3_v3v3(r_i1, v1, a);
    copy_v3_v3(r_i2, r_i1);

    return 1; /* one intersection only */
  }
  /* if not */

  float n[3], t[3];
  float v3t[3], v4t[3];
  sub_v3_v3v3(t, v1, v3);

  /* offset between both plane where the lines lies */
  cross_v3_v3v3(n, a, b);
  project_v3_v3v3(t, t, n);

  /* for the first line, offset the second line until it is coplanar */
  add_v3_v3v3(v3t, v3, t);
  add_v3_v3v3(v4t, v4, t);

  sub_v3_v3v3(c, v3t, v1);
  sub_v3_v3v3(a, v2, v1);
  sub_v3_v3v3(b, v4t, v3t);

  cross_v3_v3v3(ab, a, b);
  cross_v3_v3v3(cb, c, b);

  mul_v3_fl(a, dot_v3v3(cb, ab) / dot_v3v3(ab, ab));
  add_v3_v3v3(r_i1, v1, a);

  /* for the second line, just subtract the offset from the first intersection point */
  sub_v3_v3v3(r_i2, r_i1, t);

  return 2; /* two nearest points */
}

/* Check intersection between the line segments ab and cd. Return true only if intersection point
 * is located on both line segments. */
static IntersectingLineInfo intersecting_lines(const Segment &ab,
                                               const Segment &cd,
                                               const float distance,
                                               const float angle)
{
  IntersectingLineInfo isectinfo{};
  /* Discard by angle. */
  if (angle > 0.0f && calc_min_angle(ab.direction, cd.direction, false) < angle) {
    isectinfo.is_intersection = false;
    return isectinfo;
  }
  /* Begin intersection checks. */
  if (isect_line_line_epsilon_v3_loose(ab.start,
                                       ab.end,
                                       cd.start,
                                       cd.end,
                                       isectinfo.closest_ab,
                                       isectinfo.closest_cd,
                                       curve_isect_eps,
                                       curve_dot_eps) != 0)
  {
    /* Discard intersections too far away. Previous function returns intersections that are not
     * located on the actual segment. */
    if (math::distance(isectinfo.closest_ab, isectinfo.closest_cd) > distance) {
      isectinfo.is_intersection = false;
      return isectinfo;
    }
    /* Check intersection is on both line segments ab and cd. Lambda value is
     * captured for interpolation. Epsilon is required for matches very close to the segment end
     * points. */
    isectinfo.lambda_ab = closest_to_line_v3(
        isectinfo.closest_ab, isectinfo.closest_ab, ab.start, ab.end);
    if (isectinfo.lambda_ab <= -curve_isect_eps || isectinfo.lambda_ab >= 1.0f + curve_isect_eps) {
      isectinfo.is_intersection = false;
      return isectinfo;
    }
    isectinfo.lambda_cd = closest_to_line_v3(
        isectinfo.closest_cd, isectinfo.closest_cd, cd.start, cd.end);
    if (isectinfo.lambda_cd <= -curve_isect_eps || isectinfo.lambda_cd >= 1.0f + curve_isect_eps) {
      isectinfo.is_intersection = false;
      return isectinfo;
    }
    if (math::distance(isectinfo.closest_ab, isectinfo.closest_cd) <= distance) {
      /* Remove epsilon and clamp to 0,1 range. */
      isectinfo.lambda_ab = math::clamp(isectinfo.lambda_ab, 0.0f, 1.0f);
      isectinfo.lambda_cd = math::clamp(isectinfo.lambda_cd, 0.0f, 1.0f);
      isectinfo.is_intersection = true;
      return isectinfo;
    }
  }
  isectinfo.is_intersection = false;
  return isectinfo;
}

static float3 project_v3_plane(const float3 vector, const float3 direction)
{
  return vector - math::project(vector, direction);
}

/* Buuild curve segment bvh. */
static BVHTree *create_curve_segment_bvhtree(const bke::CurvesGeometry &src_curves,
                                             Vector<Segment> *r_curve_segments,
                                             const float angle,
                                             const bool project,
                                             const float3 project_axis,
                                             const AttributeOutputs &attribute_outputs)
{
  /* Lengths are required to return factor information for sorting. */
  src_curves.ensure_evaluated_lengths();

  const int bvh_points_num = src_curves.evaluated_points_num() + src_curves.curves_num();
  BVHTree *bvhtree = BLI_bvhtree_new(bvh_points_num, curve_isect_eps, 8, 8);
  const VArray<bool> cyclic = src_curves.cyclic();
  const OffsetIndices evaluated_points_by_curve = src_curves.evaluated_points_by_curve();
  const bool use_direction_data = angle > 0.0f || attribute_outputs.direction ||
                                  attribute_outputs.pair_direction;

  /* Preprocess curve segments for each curve. */
  for (const int64_t curve_i : src_curves.curves_range()) {
    const IndexRange points = evaluated_points_by_curve[curve_i];
    const Span<float3> positions = src_curves.evaluated_positions().slice(points);
    const Span<float> lengths = src_curves.evaluated_lengths_for_curve(curve_i, cyclic[curve_i]);
    const float curve_length = src_curves.evaluated_length_total_for_curve(curve_i,
                                                                           cyclic[curve_i]);
    const int segment_count = positions.size() - 1;

    auto add_segment = [&](const bool is_cyclic,
                           const int index,
                           const float3 start,
                           const float3 end,
                           const float len_start,
                           const float len_end) {
      Segment segment;
      segment.is_cyclic_segment = is_cyclic;
      segment.pos_index = index;
      segment.orig_start = start;
      segment.orig_end = end;
      segment.len_start = len_start;
      segment.len_end = len_end;
      segment.curve_index = curve_i;
      segment.curve_length = curve_length;
      segment.start = project ? project_v3_plane(start, project_axis) : start;
      segment.end = project ? project_v3_plane(end, project_axis) : end;
      segment.direction = use_direction_data ? math::normalize(segment.end - segment.start) :
                                               float3(0.0f);
      const int bvh_index = r_curve_segments->append_and_get_index(segment);
      BLI_bvhtree_insert(bvhtree, bvh_index, reinterpret_cast<float *>(&segment), 2);
    };

    for (const int index : IndexRange(positions.size()).drop_back(1)) {
      const float3 start = positions[index];
      const float3 end = positions[1 + index];
      const float len_start = (index == 0) ? 0.0f : lengths[index - 1];
      const float len_end = lengths[index];
      add_segment(false, index, start, end, len_start, len_end);
    }
    if (cyclic[curve_i]) {
      const float3 start = positions.last();
      const float3 end = positions.first();
      const float len_start = lengths[segment_count - 1];
      const float len_end = 1.0f;
      add_segment(true, segment_count, start, end, len_start, len_end);
    }
  }

  BLI_bvhtree_balance(bvhtree);

  return bvhtree;
}

/* Based on isect_line_plane_v3 with additional check that lines cross between start and end
 * points. It also stores the lambda.^ */
static bool isect_line_plane_v3_crossing(const float3 point_1,
                                         const float3 point_2,
                                         const float3 surface_center,
                                         const float3 surface_normal,
                                         float3 &r_isect_co,
                                         float &lambda)
{
  const float3 u = point_2 - point_1;
  const float dot = math::dot(surface_normal, u);

  /* The segment is parallel to plane */
  if (math::abs(dot) <= FLT_EPSILON) {
    return false;
  }
  const float3 h = point_1 - surface_center;
  lambda = -math::dot(surface_normal, h) / dot;

  /* Test lambda to check intersection is between the start and end points.^ */
  if (lambda >= -curve_isect_eps && lambda <= 1.0f + curve_isect_eps) {
    /* Remove epsilon from lambda. */
    lambda = math::clamp(lambda, 0.0f, 1.0f);
    r_isect_co = point_1 + u * lambda;
    return true;
  }

  return false;
}

/* Calculate intersections between a curve and a plane. */
static void set_curve_intersections_plane(const bke::CurvesGeometry &src_curves,
                                          const float3 plane_center,
                                          const float3 plane_direction,
                                          const float angle,
                                          const AttributeOutputs &attribute_outputs,
                                          IntersectionData &r_data)
{
  const VArray<bool> cyclic = src_curves.cyclic();
  const OffsetIndices evaluated_points_by_curve = src_curves.evaluated_points_by_curve();
  src_curves.ensure_evaluated_lengths();
  const bool use_angle = angle > 0.0f;
  const bool use_direction_data = attribute_outputs.direction || attribute_outputs.pair_direction;

  /* Loop each curve for intersections. */
  ThreadLocalData thread_storage;
  threading::parallel_for(src_curves.curves_range(), 128, [&](IndexRange curve_range) {
    IntersectionData &local_data = thread_storage.local();
    threading::isolate_task([&]() {
      for (const int64_t curve_i : curve_range) {
        const IndexRange points = evaluated_points_by_curve[curve_i];
        const Span<float3> positions = src_curves.evaluated_positions().slice(points);
        if (positions.size() <= 1) {
          continue;
        }
        const Span<float> lengths = src_curves.evaluated_lengths_for_curve(curve_i,
                                                                           cyclic[curve_i]);
        const float length = src_curves.evaluated_length_total_for_curve(curve_i, cyclic[curve_i]);

        auto add_closest = [&](const float3 a,
                               const float3 b,
                               const float len_start,
                               const float len_end,
                               const float curve_length) {
          float3 closest = float3(0.0f);
          float lambda = 0.0f;

          const float3 segment_direction = use_direction_data || use_angle ?
                                               math::normalize(b - a) :
                                               float3(0.0f);

          /* Discard by angle. */
          if (use_angle && calc_min_angle(segment_direction, plane_direction, true) < angle) {
            return;
          }

          if (isect_line_plane_v3_crossing(a, b, plane_center, plane_direction, closest, lambda)) {

            add_intersection_data(local_data,
                                  closest,
                                  curve_i,
                                  segment_direction,
                                  math::interpolate(len_start, len_end, lambda),
                                  curve_length,
                                  float3(0.0f),
                                  float3(0.0f),
                                  float3(0.0f),
                                  false,
                                  attribute_outputs);
          }
        };

        /* Loop segments from start until we have an intersection. */
        for (const int index : IndexRange(positions.size()).drop_back(1)) {
          const float3 a = positions[index];
          const float3 b = positions[1 + index];
          const float len_start = (index == 0) ? 0.0f : lengths[index - 1];
          const float len_end = lengths[index];
          add_closest(a, b, len_start, len_end, length);
        }
        if (cyclic[curve_i]) {
          const float3 a = positions.last();
          const float3 b = positions.first();
          const float len_start = lengths.last();
          const float len_end = 1.0f;
          add_closest(a, b, len_start, len_end, length);
        }
      }
    });
  });
  gather_thread_storage(thread_storage, r_data, attribute_outputs);
}

/* Calculate intersections between curve and mesh surface. */
static void set_curve_intersections_mesh(GeometrySet &mesh_set,
                                         const bke::CurvesGeometry &src_curves,
                                         const float angle,
                                         const AttributeOutputs &attribute_outputs,
                                         IntersectionData &r_data)
{
  /* Build bvh. */
  Vector<Segment> curve_segments;
  BVHTree *bvhtree = create_curve_segment_bvhtree(
      src_curves, &curve_segments, angle, false, float3(0.0f), attribute_outputs);
  BLI_SCOPED_DEFER([&]() { BLI_bvhtree_free(bvhtree); });
  const bool use_angle = angle > 0.0f;
  const bool use_normal = use_angle || attribute_outputs.normal;

  /* Loop mesh data. */
  mesh_set.modify_geometry_sets([&](GeometrySet &mesh_set) {
    if (!mesh_set.has_mesh()) {
      return;
    }
    const Mesh &mesh = *mesh_set.get_mesh();
    if (mesh.faces_num < 1) {
      return;
    }
    const Span<float3> positions = mesh.vert_positions();
    const Span<int> corner_verts = mesh.corner_verts();
    const Span<int3> corner_tris = mesh.corner_tris();

    /* Loop face data. */
    ThreadLocalData thread_storage;
    threading::parallel_for(corner_tris.index_range(), 128, [&](IndexRange range) {
      IntersectionData &local_data = thread_storage.local();
      threading::isolate_task([&]() {
        for (const int64_t face_index : range) {
          const int3 &tri = corner_tris[face_index];
          const int v0_loop = tri[0];
          const int v1_loop = tri[1];
          const int v2_loop = tri[2];
          const float3 &v0_pos = positions[corner_verts[v0_loop]];
          const float3 &v1_pos = positions[corner_verts[v1_loop]];
          const float3 &v2_pos = positions[corner_verts[v2_loop]];

          float3 center_pos;
          interp_v3_v3v3v3(center_pos, v0_pos, v1_pos, v2_pos, float3(1.0f / 3.0f));

          const float distance = math::max(
              math::max(math::distance(center_pos, v0_pos), math::distance(center_pos, v1_pos)),
              math::distance(center_pos, v2_pos));

          BLI_bvhtree_range_query_cpp(
              *bvhtree,
              center_pos,
              distance + curve_isect_eps,
              [&](const int index, const float3 & /*co*/, const float /*dist_sq*/) {
                const Segment seg = curve_segments[index];
                float lambda = 0.0f;

                float3 normal = float3(0.0f);
                if (use_normal) {
                  normal_tri_v3(normal, v0_pos, v1_pos, v2_pos);
                }

                /* Discard by angle. */
                if (use_angle && calc_min_angle(seg.direction, normal, true) < angle) {
                  return;
                }

                if (isect_line_segment_tri_epsilon_v3(seg.start,
                                                      seg.end,
                                                      v0_pos,
                                                      v1_pos,
                                                      v2_pos,
                                                      &lambda,
                                                      nullptr,
                                                      curve_isect_eps))
                {
                  const float len_at_isect = math::interpolate(seg.len_start, seg.len_end, lambda);
                  const float3 closest_position = math::interpolate(seg.start, seg.end, lambda);
                  add_intersection_data(local_data,
                                        closest_position,
                                        seg.curve_index,
                                        seg.direction,
                                        len_at_isect,
                                        seg.curve_length,
                                        normal,
                                        float3(0.0f),
                                        float3(0.0f),
                                        false,
                                        attribute_outputs);
                }
              });
        }
      });
    });
    gather_thread_storage(thread_storage, r_data, attribute_outputs);
  });
}

/* Calculate intersections between 3d curves, optionally projected onto 2d plane. */
static void set_curve_intersections(const bke::CurvesGeometry &src_curves,
                                    const bool self_intersect,
                                    const bool all_intersect,
                                    const float distance,
                                    const float angle,
                                    const bool project,
                                    const float3 direction,
                                    const PairData &pair_data_mode,
                                    const AttributeOutputs &attribute_outputs,
                                    IntersectionData &r_data)
{
  /* Build bvh. */
  Vector<Segment> curve_segments;
  BVHTree *bvhtree = create_curve_segment_bvhtree(
      src_curves, &curve_segments, angle, project, direction, attribute_outputs);
  BLI_SCOPED_DEFER([&]() { BLI_bvhtree_free(bvhtree); });

  const float max_distance = math::max(curve_isect_eps, distance);

  /* Loop through segments. */
  ThreadLocalData thread_storage;
  threading::parallel_for(curve_segments.index_range(), 128, [&](IndexRange range) {
    IntersectionData &local_data = thread_storage.local();
    threading::isolate_task([&]() {
      for (const int64_t segment_index : range) {
        const Segment ab = curve_segments[segment_index];
        BLI_bvhtree_range_query_cpp(
            *bvhtree,
            math::midpoint(ab.start, ab.end),
            math::distance(ab.start, ab.end) + max_distance,
            [&](const int index, const float3 & /*co*/, const float /*dist_sq*/) {
              if (segment_index <= index) {
                /* Skip matching segments or previously matched segments. */
                return;
              }
              const Segment cd = curve_segments[index];
              const bool same_curve = ab.curve_index == cd.curve_index;
              const bool calc_all = (all_intersect && !same_curve);
              /* Skip adjecent segments in same curve. */
              const bool calc_self = (self_intersect && same_curve &&
                                      ((math::abs(ab.pos_index - cd.pos_index) > 1) &&
                                       !((ab.pos_index == 0 && cd.is_cyclic_segment) ||
                                         (cd.pos_index == 0 && ab.is_cyclic_segment))));
              if (calc_all || calc_self) {
                const IntersectingLineInfo isectinfo = intersecting_lines(
                    ab, cd, max_distance, angle);

                if (isectinfo.is_intersection) {
                  const float3 closest_ab = math::interpolate(
                      ab.orig_start, ab.orig_end, isectinfo.lambda_ab);
                  const float3 closest_cd = math::interpolate(
                      cd.orig_start, cd.orig_end, isectinfo.lambda_cd);
                  const bool pair_weight = ab.curve_index > cd.curve_index;
                  add_intersection_data(
                      local_data,
                      closest_ab,
                      ab.curve_index,
                      ab.direction,
                      math::interpolate(ab.len_start, ab.len_end, isectinfo.lambda_ab),
                      ab.curve_length,
                      float3(0.0f),
                      closest_cd,
                      cd.direction,
                      !pair_weight,
                      attribute_outputs);
                  /* Only return both intersection points if required. */
                  if (pair_data_mode == PairData::FullPair ||
                      (pair_data_mode == PairData::PointsOnly &&
                       math::distance(closest_ab, closest_cd) > curve_isect_eps))
                  {
                    add_intersection_data(
                        local_data,
                        closest_cd,
                        cd.curve_index,
                        cd.direction,
                        math::interpolate(cd.len_start, cd.len_end, isectinfo.lambda_cd),
                        cd.curve_length,
                        float3(0.0f),
                        closest_ab,
                        ab.direction,
                        pair_weight,
                        attribute_outputs);
                  }
                }
              }
            });
      }
    });
  });
  gather_thread_storage(thread_storage, r_data, attribute_outputs);
}

/* Sorting intersections by sortkey (which is curve_index and factor postion on curve). */
static IntersectionData sort_intersection_data(IntersectionData &data,
                                               const AttributeOutputs &attribute_outputs)
{
  const int64_t data_size = data.position.size();

  Vector<std::pair<int64_t, float>> sort_index;
  sort_index.reserve(data_size);

  for (int64_t i = 0; i < data_size; i++) {
    sort_index.append(std::pair(i, data.sortkey[i]));
  }

  std::sort(sort_index.begin(),
            sort_index.end(),
            [&](const std::pair<int64_t, float> &a, const std::pair<int64_t, float> &b) {
              return (a.second < b.second);
            });

  /* Ignore sortdata for return data. */
  IntersectionData r_data;
  r_data.position.reserve(data_size);
  if (attribute_outputs.curve_index) {
    r_data.curve_index.reserve(data_size);
  }
  if (attribute_outputs.length) {
    r_data.length.reserve(data_size);
  }
  if (attribute_outputs.factor) {
    r_data.factor.reserve(data_size);
  }
  if (attribute_outputs.direction) {
    r_data.direction.reserve(data_size);
  }
  if (attribute_outputs.normal) {
    r_data.normal.reserve(data_size);
  }
  if (attribute_outputs.pair_position) {
    r_data.pair_position.reserve(data_size);
  }
  if (attribute_outputs.pair_direction) {
    r_data.pair_direction.reserve(data_size);
  }
  if (attribute_outputs.pair) {
    r_data.pair.reserve(data_size);
  }

  for (const std::pair key_val : sort_index) {
    const int64_t key_index = key_val.first;
    r_data.position.append(data.position[key_index]);
    if (attribute_outputs.curve_index) {
      r_data.curve_index.append(data.curve_index[key_index]);
    }
    if (attribute_outputs.length) {
      r_data.length.append(data.length[key_index]);
    }
    if (attribute_outputs.factor) {
      r_data.factor.append(data.factor[key_index]);
    }
    if (attribute_outputs.direction) {
      r_data.direction.append(data.direction[key_index]);
    }
    if (attribute_outputs.normal) {
      r_data.normal.append(data.normal[key_index]);
    }
    if (attribute_outputs.pair_position) {
      r_data.pair_position.append(data.pair_position[key_index]);
    }
    if (attribute_outputs.pair_direction) {
      r_data.pair_direction.append(data.pair_direction[key_index]);
    }
    if (attribute_outputs.pair) {
      r_data.pair.append(data.pair[key_index]);
    }
  }
  BLI_assert(data.position.size() == r_data.position.size());
  return r_data;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const IntersectionMode mode = IntersectionMode(params.node().custom1);
  const PairData pair_data_mode = PairData(params.node().custom2);
  const bool curve_mode = ELEM(mode, IntersectionMode::Curve, IntersectionMode::Curve_Project);
  const bool use_paired_data = ELEM(pair_data_mode, PairData::FullPair, PairData::HalfPair);
  const bool points_only_mode = curve_mode && pair_data_mode == PairData::PointsOnly;

  GeometrySet geometry_set = params.extract_input<GeometrySet>("Curve");
  GeometryComponentEditData::remember_deformed_positions_if_necessary(geometry_set);

  lazy_threading::send_hint();

  AttributeOutputs attribute_outputs;

  if (!points_only_mode) {
    attribute_outputs.direction = params.get_output_anonymous_attribute_id_if_needed("Direction");
    attribute_outputs.curve_index = params.get_output_anonymous_attribute_id_if_needed(
        "Curve Index");
    attribute_outputs.factor = params.get_output_anonymous_attribute_id_if_needed("Factor");
    attribute_outputs.length = params.get_output_anonymous_attribute_id_if_needed("Length");
  }

  if (mode == IntersectionMode::Surface) {
    attribute_outputs.normal = params.get_output_anonymous_attribute_id_if_needed("Normal");
  }

  if (curve_mode && use_paired_data) {
    attribute_outputs.pair_position = params.get_output_anonymous_attribute_id_if_needed(
        "Pair Position");
    attribute_outputs.pair_direction = params.get_output_anonymous_attribute_id_if_needed(
        "Pair Direction");
  }

  if (curve_mode && pair_data_mode == PairData::FullPair) {
    attribute_outputs.pair = params.get_output_anonymous_attribute_id_if_needed("Pair");
  }

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    if (!geometry_set.has_curves()) {
      geometry_set.remove_geometry_during_modify();
      return;
    }
    const Curves &src_curves_id = *geometry_set.get_curves();
    const bke::CurvesGeometry &src_curves = src_curves_id.geometry.wrap();

    if (src_curves.curves_range().is_empty()) {
      return;
    }

    IntersectionData r_data;

    switch (mode) {
      case IntersectionMode::Curve: {
        const float distance = params.extract_input<float>("Epsilon");
        const bool self = params.extract_input<bool>("Self Intersections");
        const bool all = params.extract_input<bool>("All Intersections");
        const float angle = params.extract_input<float>("Min Angle");
        set_curve_intersections(src_curves,
                                self,
                                all,
                                distance,
                                math::clamp(angle, 0.0f, pi_2_f_eps),
                                false,
                                float3(0.0f),
                                pair_data_mode,
                                attribute_outputs,
                                r_data);
        break;
      }
      case IntersectionMode::Curve_Project: {
        const float3 direction = params.extract_input<float3>("Direction");
        const bool self = params.extract_input<bool>("Self Intersections");
        const bool all = params.extract_input<bool>("All Intersections");
        const float angle = params.extract_input<float>("Min Angle");
        set_curve_intersections(src_curves,
                                self,
                                all,
                                0.0f,
                                math::clamp(angle, 0.0f, pi_2_f_eps),
                                true,
                                direction,
                                pair_data_mode,
                                attribute_outputs,
                                r_data);
        break;
      }
      case IntersectionMode::Plane: {
        const float3 direction = params.extract_input<float3>("Direction");
        const float3 plane_center = params.extract_input<float3>("Center");
        const float angle = params.extract_input<float>("Min Angle");
        set_curve_intersections_plane(src_curves,
                                      plane_center,
                                      math::normalize(direction),
                                      math::clamp(angle, 0.0f, pi_2_f_eps),
                                      attribute_outputs,
                                      r_data);
        break;
      }
      case IntersectionMode::Surface: {
        GeometrySet mesh_set = params.extract_input<GeometrySet>("Mesh");
        const float angle = params.extract_input<float>("Min Angle");
        if (mesh_set.has_mesh()) {
          set_curve_intersections_mesh(mesh_set,
                                       src_curves,
                                       math::clamp(angle, 0.0f, pi_2_f_eps),
                                       attribute_outputs,
                                       r_data);
        }
        else {
          geometry_set.remove_geometry_during_modify();
          return;
        }
        break;
      }
      default: {
        BLI_assert_unreachable();
        break;
      }
    }

    geometry_set.remove_geometry_during_modify();

    /* Gather and sort data for attributes. */
    if (r_data.position.size() > 0) {

      IntersectionData sorted_data = sort_intersection_data(r_data, attribute_outputs);

      PointCloud *pointcloud = BKE_pointcloud_new_nomain(sorted_data.position.size());
      MutableAttributeAccessor attributes = pointcloud->attributes_for_write();

      geometry_set.replace_pointcloud(pointcloud);

      /* Builtin attributes. */
      SpanAttributeWriter<float3> point_positions =
          attributes.lookup_or_add_for_write_only_span<float3>("position", AttrDomain::Point);
      point_positions.span.copy_from(sorted_data.position);
      point_positions.finish();

      SpanAttributeWriter<float> point_radii = attributes.lookup_or_add_for_write_only_span<float>(
          "radius", AttrDomain::Point);
      point_radii.span.fill(0.05f);
      point_radii.finish();

      /* Output attributes. */
      if (attribute_outputs.curve_index) {
        SpanAttributeWriter<int> curve_index = attributes.lookup_or_add_for_write_only_span<int>(
            *attribute_outputs.curve_index, AttrDomain::Point);
        curve_index.span.copy_from(sorted_data.curve_index);
        curve_index.finish();
      }

      if (attribute_outputs.direction) {
        SpanAttributeWriter<float3> directions =
            attributes.lookup_or_add_for_write_only_span<float3>(*attribute_outputs.direction,
                                                                 AttrDomain::Point);
        directions.span.copy_from(sorted_data.direction);
        directions.finish();
      }

      if (attribute_outputs.normal) {
        SpanAttributeWriter<float3> normal = attributes.lookup_or_add_for_write_only_span<float3>(
            *attribute_outputs.normal, AttrDomain::Point);
        normal.span.copy_from(sorted_data.normal);
        normal.finish();
      }

      if (attribute_outputs.factor) {
        SpanAttributeWriter<float> factor = attributes.lookup_or_add_for_write_only_span<float>(
            *attribute_outputs.factor, AttrDomain::Point);
        factor.span.copy_from(sorted_data.factor);
        factor.finish();
      }

      if (attribute_outputs.length) {
        SpanAttributeWriter<float> length = attributes.lookup_or_add_for_write_only_span<float>(
            *attribute_outputs.length, AttrDomain::Point);
        length.span.copy_from(sorted_data.length);
        length.finish();
      }

      if (attribute_outputs.pair_position) {
        SpanAttributeWriter<float3> pair_position =
            attributes.lookup_or_add_for_write_only_span<float3>(*attribute_outputs.pair_position,
                                                                 AttrDomain::Point);
        pair_position.span.copy_from(sorted_data.pair_position);
        pair_position.finish();
      }

      if (attribute_outputs.pair_direction) {
        SpanAttributeWriter<float3> pair_direction =
            attributes.lookup_or_add_for_write_only_span<float3>(*attribute_outputs.pair_direction,
                                                                 AttrDomain::Point);
        pair_direction.span.copy_from(sorted_data.pair_direction);
        pair_direction.finish();
      }

      if (attribute_outputs.pair && pair_data_mode == PairData::FullPair) {
        SpanAttributeWriter<bool> pair = attributes.lookup_or_add_for_write_only_span<bool>(
            *attribute_outputs.pair, AttrDomain::Point);
        pair.span.copy_from(sorted_data.pair);
        pair.finish();
      }
    }
  });

  params.set_output("Points", std::move(geometry_set));
}

static void node_rna(StructRNA *srna)
{
  static EnumPropertyItem mode_items[] = {
      {int16_t(IntersectionMode::Curve),
       "CURVE",
       0,
       "Curve",
       "Find the intersection positions between curves in 3d space"},
      {int16_t(IntersectionMode::Curve_Project),
       "CURVE_PROJECT",
       0,
       "Curve Project",
       "Find all the intersection positions for all curves projected onto orthographic plane"},
      {int16_t(IntersectionMode::Plane),
       "PLANE",
       0,
       "Plane",
       "Find all the intersection positions for each curve in reference to a plane"},
      {int16_t(IntersectionMode::Surface),
       "SURFACE",
       0,
       "Surface",
       "Find all the intersection positions for each curve in reference to a mesh surface"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "mode",
                    "Mode",
                    "Method to find intersection positions for the spline",
                    mode_items,
                    NOD_inline_enum_accessors(custom1));

  static EnumPropertyItem pair_data_mode_items[] = {
      {int16_t(PairData::PointsOnly),
       "POINTS_ONLY",
       0,
       "Points Only",
       "Return intersection points only"},
      {int16_t(PairData::HalfPair),
       "HALF_PAIR",
       0,
       "Half Pair",
       "Return first intersection of a pair, weighted to lowest curve id and corresponding pair "
       "data"},
      {int16_t(PairData::FullPair),
       "FULL_PAIR",
       0,
       "Full Pair",
       "Return all intersections and corresponding pair data"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "pair_data_mode",
                    "Paired Data Mode",
                    "Return either single, single with pair data or all intersections",
                    pair_data_mode_items,
                    NOD_inline_enum_accessors(custom2));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, GEO_NODE_CURVE_INTERSECTIONS, "Curve Intersections", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.updatefunc = node_update;
  blender::bke::node_register_type(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_curve_intersection_cc
