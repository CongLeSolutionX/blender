/* SPDX-FileCopyrightText: 2024 Tenkai Raiko
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma BLENDER_REQUIRE(gpu_shader_common_math_utils.glsl)

vec3 calculate_out_fields_2d_full_roundness_irregular_elliptical(

    bool calculate_r_gon_parameter_field,
    bool normalize_r_gon_parameter,
    float r_gon_sides,
    vec2 coord,
    float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
  float ref_A_angle_bisector = M_PI / r_gon_sides;
  float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
  float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI - floor(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;

  if ((x_axis_A_coord >= ref_A_angle_bisector) &&
      (x_axis_A_coord < M_TAU - last_ref_A_x_axis - ref_A_angle_bisector))
  {
    float r_gon_parameter_2d = 0.0;
    if (calculate_r_gon_parameter_field) {
      r_gon_parameter_2d = abs(ref_A_angle_bisector - ref_A_coord);
      if (ref_A_coord < ref_A_angle_bisector) {
        r_gon_parameter_2d *= -1.0;
      }
      if (normalize_r_gon_parameter) {
        r_gon_parameter_2d /= ref_A_angle_bisector;
      }
    }
    return vec3(l_projection_2d, r_gon_parameter_2d, ref_A_angle_bisector);
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis - ref_A_angle_bisector) &&
        (x_axis_A_coord < M_TAU - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1;
    }
    float l_angle_bisector_2d = 0.0;
    float r_gon_parameter_2d = 0.0;
    float max_unit_parameter_2d = 0.0;

    float l_basis_vector_1 = tan(ref_A_angle_bisector);
    /* When the fractional part of r_gon_sides is very small division by l_basis_vector_2 causes
     * precision issues. Change to double if necessary */
    float l_basis_vector_2 = sin(last_angle_bisector_A_x_axis) *
                             sqrt(square(tan(ref_A_angle_bisector)) + 1.0);
    vec2 ellipse_center =
        vec2(cos(ref_A_angle_bisector) / cos(ref_A_angle_bisector - ref_A_angle_bisector),
             sin(ref_A_angle_bisector) / cos(ref_A_angle_bisector - ref_A_angle_bisector)) -
        l_basis_vector_2 *
            vec2(sin(last_angle_bisector_A_x_axis), cos(last_angle_bisector_A_x_axis));
    vec2 transformed_direction_vector = vec2(
        cos(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
            (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
        cos(ref_A_angle_bisector - nearest_ref_MSA_coord) /
            (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
    vec2 transformed_origin = vec2(
        (ellipse_center.y * sin(last_angle_bisector_A_x_axis) -
         ellipse_center.x * cos(last_angle_bisector_A_x_axis)) /
            (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
        -(ellipse_center.y * sin(ref_A_angle_bisector) +
          ellipse_center.x * cos(ref_A_angle_bisector)) /
            (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
    float l_coord_R_l_angle_bisector_2d =
        (-(transformed_direction_vector.x * transformed_origin.x +
           transformed_direction_vector.y * transformed_origin.y) +
         sqrt(square(transformed_direction_vector.x * transformed_origin.x +
                     transformed_direction_vector.y * transformed_origin.y) -
              (square(transformed_direction_vector.x) + square(transformed_direction_vector.y)) *
                  (square(transformed_origin.x) + square(transformed_origin.y) - 1.0))) /
        (square(transformed_direction_vector.x) + square(transformed_direction_vector.y));
    l_angle_bisector_2d = l_projection_2d / l_coord_R_l_angle_bisector_2d;
    if (nearest_ref_MSA_coord < 0.0) {
      float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                             cos(last_angle_bisector_A_x_axis);
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = (last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) *
                             l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter_2d *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= last_angle_bisector_A_x_axis *
                                l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        }
      }
      max_unit_parameter_2d = last_angle_bisector_A_x_axis *
                              l_angle_bisector_2d_R_l_last_angle_bisector_2d;
    }
    else {
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = abs(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= ref_A_angle_bisector;
        }
      }
      max_unit_parameter_2d = ref_A_angle_bisector;
    }
    return vec3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
  }
}

vec3 calculate_out_fields_2d_irregular_elliptical(bool calculate_r_gon_parameter_field,
                                                  bool calculate_max_unit_parameter,
                                                  bool normalize_r_gon_parameter,
                                                  float r_gon_sides,
                                                  float r_gon_roundness,
                                                  vec2 coord,
                                                  float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
  float ref_A_angle_bisector = M_PI / r_gon_sides;
  float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
  float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1.0 - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI - floor(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1.0 - r_gon_roundness) *
                                               tan(last_angle_bisector_A_x_axis));

  if ((x_axis_A_coord >= ref_A_bevel_start) &&
      (x_axis_A_coord < M_TAU - last_ref_A_x_axis - ref_A_bevel_start))
  {
    if ((ref_A_coord >= ref_A_bevel_start) && (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) {
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;

      l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d * tan(abs(ref_A_angle_bisector - ref_A_coord));
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= l_angle_bisector_2d *
                                    tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                ref_A_bevel_start;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) + ref_A_bevel_start;
      }
      return vec3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
    else {
      /* SA == Signed Angle in [-M_PI, M_PI]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;

      float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
      float l_circle_center = sin(ref_A_angle_bisector - ref_A_bevel_start) /
                              sin(ref_A_angle_bisector);
      float l_coord_R_l_bevel_start = cos(nearest_ref_SA_coord) * l_circle_center +
                                      sqrt(square(cos(nearest_ref_SA_coord) * l_circle_center) +
                                           square(l_circle_radius) - square(l_circle_center));
      l_angle_bisector_2d = cos(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
                            l_coord_R_l_bevel_start;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d * tan(ref_A_angle_bisector - ref_A_bevel_start) +
                             ref_A_bevel_start - abs(nearest_ref_SA_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= l_angle_bisector_2d *
                                    tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                ref_A_bevel_start;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) + ref_A_bevel_start;
      }
      return vec3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;

      float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                             cos(last_angle_bisector_A_x_axis);
      l_angle_bisector_2d = l_projection_2d * cos(last_angle_bisector_A_x_axis - ref_A_coord) *
                            l_angle_bisector_2d_R_l_last_angle_bisector_2d;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d *
                             tan(abs(last_angle_bisector_A_x_axis - ref_A_coord));
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter_2d *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis -
                                                          inner_last_bevel_start_A_x_axis) +
                                inner_last_bevel_start_A_x_axis *
                                    l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis -
                                    inner_last_bevel_start_A_x_axis) +
                                inner_last_bevel_start_A_x_axis *
                                    l_angle_bisector_2d_R_l_last_angle_bisector_2d;
      }
      return vec3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis - ref_A_bevel_start) &&
          (x_axis_A_coord < M_TAU - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1;
      }
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;

      float l_basis_vector_1 = r_gon_roundness * tan(ref_A_angle_bisector);
      float l_basis_vector_2 = r_gon_roundness * sin(last_angle_bisector_A_x_axis) *
                               sqrt(square(tan(ref_A_angle_bisector)) + 1.0);
      vec2 ellipse_center =
          vec2(cos(ref_A_bevel_start) / cos(ref_A_angle_bisector - ref_A_bevel_start),
               sin(ref_A_bevel_start) / cos(ref_A_angle_bisector - ref_A_bevel_start)) -
          l_basis_vector_2 *
              vec2(sin(last_angle_bisector_A_x_axis), cos(last_angle_bisector_A_x_axis));
      vec2 transformed_direction_vector = vec2(
          cos(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
              (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          cos(ref_A_angle_bisector - nearest_ref_MSA_coord) /
              (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      vec2 transformed_origin = vec2(
          (ellipse_center.y * sin(last_angle_bisector_A_x_axis) -
           ellipse_center.x * cos(last_angle_bisector_A_x_axis)) /
              (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          -(ellipse_center.y * sin(ref_A_angle_bisector) +
            ellipse_center.x * cos(ref_A_angle_bisector)) /
              (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      float l_coord_R_l_angle_bisector_2d =
          (-(transformed_direction_vector.x * transformed_origin.x +
             transformed_direction_vector.y * transformed_origin.y) +
           sqrt(square(transformed_direction_vector.x * transformed_origin.x +
                       transformed_direction_vector.y * transformed_origin.y) -
                (square(transformed_direction_vector.x) + square(transformed_direction_vector.y)) *
                    (square(transformed_origin.x) + square(transformed_origin.y) - 1.0))) /
          (square(transformed_direction_vector.x) + square(transformed_direction_vector.y));
      l_angle_bisector_2d = l_projection_2d / l_coord_R_l_angle_bisector_2d;
      if (nearest_ref_MSA_coord < 0.0) {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                               cos(last_angle_bisector_A_x_axis);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d * tan(abs(last_angle_bisector_A_x_axis -
                                                             inner_last_bevel_start_A_x_axis)) +
                               (inner_last_bevel_start_A_x_axis + nearest_ref_MSA_coord) *
                                   l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis -
                                                            inner_last_bevel_start_A_x_axis) +
                                  inner_last_bevel_start_A_x_axis *
                                      l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis -
                                      inner_last_bevel_start_A_x_axis) +
                                  inner_last_bevel_start_A_x_axis *
                                      l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        }
      }
      else {
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                                   tan(ref_A_angle_bisector - ref_A_bevel_start) +
                               ref_A_bevel_start - nearest_ref_MSA_coord;
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
        }
      }
      return vec3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
  }
}

vec3 calculate_out_fields_2d_full_roundness_irregular_circular(

    bool calculate_r_gon_parameter_field,
    bool calculate_max_unit_parameter,
    bool normalize_r_gon_parameter,
    float r_gon_sides,
    vec2 coord,
    float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
  float ref_A_angle_bisector = M_PI / r_gon_sides;
  float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
  float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI - floor(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;
  float l_last_circle_radius = tan(last_angle_bisector_A_x_axis) /
                               tan(0.5 * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  vec2 last_circle_center = vec2(cos(last_angle_bisector_A_x_axis) -
                                     l_last_circle_radius * cos(last_angle_bisector_A_x_axis),
                                 l_last_circle_radius * sin(last_angle_bisector_A_x_axis) -
                                     sin(last_angle_bisector_A_x_axis));
  vec2 outer_last_bevel_start = last_circle_center +
                                l_last_circle_radius *
                                    vec2(cos(ref_A_angle_bisector), sin(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                               outer_last_bevel_start.x);

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis - ref_A_angle_bisector) ||
        (x_axis_A_coord < ref_A_angle_bisector))
    {
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;

      l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d * tan(abs(ref_A_angle_bisector - ref_A_coord));
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= l_angle_bisector_2d *
                                    tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                x_axis_A_outer_last_bevel_start;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter_2d = tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                x_axis_A_outer_last_bevel_start;
      }
      return vec3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
    else {
      float r_gon_parameter_2d = 0.0;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = abs(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= ref_A_angle_bisector;
        }
      }
      return vec3(l_projection_2d, r_gon_parameter_2d, ref_A_angle_bisector);
    }
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
        (x_axis_A_coord < M_TAU - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1;
    }
    float l_angle_bisector_2d = 0.0;
    float r_gon_parameter_2d = 0.0;
    float max_unit_parameter_2d = 0.0;

    float l_coord_R_l_last_angle_bisector_2d =
        sin(nearest_ref_MSA_coord) * last_circle_center.y +
        cos(nearest_ref_MSA_coord) * last_circle_center.x +
        sqrt(square(sin(nearest_ref_MSA_coord) * last_circle_center.y +
                    cos(nearest_ref_MSA_coord) * last_circle_center.x) +
             square(l_last_circle_radius) - square(last_circle_center.x) -
             square(last_circle_center.y));
    l_angle_bisector_2d = (cos(ref_A_angle_bisector) * l_projection_2d) /
                          (cos(last_angle_bisector_A_x_axis) * l_coord_R_l_last_angle_bisector_2d);
    if (nearest_ref_MSA_coord < 0.0) {
      float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                             cos(last_angle_bisector_A_x_axis);
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = (last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) *
                             l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter_2d *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= last_angle_bisector_A_x_axis *
                                l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        }
      }
      max_unit_parameter_2d = last_angle_bisector_A_x_axis *
                              l_angle_bisector_2d_R_l_last_angle_bisector_2d;
    }
    else {
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d *
                                 tan(abs(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start)) +
                             x_axis_A_outer_last_bevel_start - nearest_ref_MSA_coord;
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= l_angle_bisector_2d *
                                    tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                x_axis_A_outer_last_bevel_start;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter_2d = tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                x_axis_A_outer_last_bevel_start;
      }
    }
    return vec3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
  }
}

vec3 calculate_out_fields_2d_irregular_circular(bool calculate_r_gon_parameter_field,
                                                bool calculate_max_unit_parameter,
                                                bool normalize_r_gon_parameter,
                                                float r_gon_sides,
                                                float r_gon_roundness,
                                                vec2 coord,
                                                float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
  float ref_A_angle_bisector = M_PI / r_gon_sides;
  float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
  float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1.0 - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI - floor(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1.0 - r_gon_roundness) *
                                               tan(last_angle_bisector_A_x_axis));
  float l_last_circle_radius = r_gon_roundness * tan(last_angle_bisector_A_x_axis) /
                               tan(0.5 * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  vec2 last_circle_center = vec2(
      (cos(inner_last_bevel_start_A_x_axis) /
       cos(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)) -
          l_last_circle_radius * cos(last_angle_bisector_A_x_axis),
      l_last_circle_radius * sin(last_angle_bisector_A_x_axis) -
          (sin(inner_last_bevel_start_A_x_axis) /
           cos(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)));
  vec2 outer_last_bevel_start = last_circle_center +
                                l_last_circle_radius *
                                    vec2(cos(ref_A_angle_bisector), sin(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                               outer_last_bevel_start.x);

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if (((ref_A_coord >= ref_A_bevel_start) &&
         (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) ||
        (x_axis_A_coord >= M_TAU - last_ref_A_x_axis - ref_A_bevel_start) ||
        (x_axis_A_coord < ref_A_bevel_start))
    {
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;

      l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
      if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis - ref_A_angle_bisector) ||
          (x_axis_A_coord < ref_A_angle_bisector))
      {
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d * tan(abs(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                  x_axis_A_outer_last_bevel_start;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                  x_axis_A_outer_last_bevel_start;
        }
      }
      else {
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d * tan(abs(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
        }
      }
      return vec3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
    else {
      /* SA == Signed Angle in [-M_PI, M_PI]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;

      float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
      float l_circle_center = sin(ref_A_angle_bisector - ref_A_bevel_start) /
                              sin(ref_A_angle_bisector);
      float l_coord_R_l_bevel_start = cos(nearest_ref_SA_coord) * l_circle_center +
                                      sqrt(square(cos(nearest_ref_SA_coord) * l_circle_center) +
                                           square(l_circle_radius) - square(l_circle_center));
      l_angle_bisector_2d = cos(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
                            l_coord_R_l_bevel_start;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d * tan(ref_A_angle_bisector - ref_A_bevel_start) +
                             ref_A_bevel_start - abs(nearest_ref_SA_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= l_angle_bisector_2d *
                                    tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                ref_A_bevel_start;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) + ref_A_bevel_start;
      }
      return vec3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;

      float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                             cos(last_angle_bisector_A_x_axis);
      l_angle_bisector_2d = l_projection_2d * cos(last_angle_bisector_A_x_axis - ref_A_coord) *
                            l_angle_bisector_2d_R_l_last_angle_bisector_2d;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d *
                             tan(abs(last_angle_bisector_A_x_axis - ref_A_coord));
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter_2d *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis -
                                                          inner_last_bevel_start_A_x_axis) +
                                inner_last_bevel_start_A_x_axis *
                                    l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis -
                                    inner_last_bevel_start_A_x_axis) +
                                inner_last_bevel_start_A_x_axis *
                                    l_angle_bisector_2d_R_l_last_angle_bisector_2d;
      }
      return vec3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
          (x_axis_A_coord < M_TAU - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1;
      }
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;

      float l_coord_R_l_last_angle_bisector_2d =
          sin(nearest_ref_MSA_coord) * last_circle_center.y +
          cos(nearest_ref_MSA_coord) * last_circle_center.x +
          sqrt(square(sin(nearest_ref_MSA_coord) * last_circle_center.y +
                      cos(nearest_ref_MSA_coord) * last_circle_center.x) +
               square(l_last_circle_radius) - square(last_circle_center.x) -
               square(last_circle_center.y));
      l_angle_bisector_2d = (cos(ref_A_angle_bisector) * l_projection_2d) /
                            (cos(last_angle_bisector_A_x_axis) *
                             l_coord_R_l_last_angle_bisector_2d);
      if (nearest_ref_MSA_coord < 0.0) {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                               cos(last_angle_bisector_A_x_axis);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d * tan(abs(last_angle_bisector_A_x_axis -
                                                             inner_last_bevel_start_A_x_axis)) +
                               (inner_last_bevel_start_A_x_axis + nearest_ref_MSA_coord) *
                                   l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis -
                                                            inner_last_bevel_start_A_x_axis) +
                                  inner_last_bevel_start_A_x_axis *
                                      l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis -
                                      inner_last_bevel_start_A_x_axis) +
                                  inner_last_bevel_start_A_x_axis *
                                      l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        }
      }
      else {
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d * tan(abs(ref_A_angle_bisector -
                                                             x_axis_A_outer_last_bevel_start)) +
                               x_axis_A_outer_last_bevel_start - nearest_ref_MSA_coord;
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                  x_axis_A_outer_last_bevel_start;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                  x_axis_A_outer_last_bevel_start;
        }
      }
      return vec3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
  }
}

vec3 calculate_out_fields_2d(bool calculate_r_gon_parameter_field,
                             bool calculate_max_unit_parameter,
                             bool normalize_r_gon_parameter,
                             bool elliptical_corners,
                             float r_gon_sides,
                             float r_gon_roundness,
                             vec2 coord)
{
  float l_projection_2d = sqrt(square(coord.x) + square(coord.y));

  if (fract(r_gon_sides) == 0.0) {
    float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
    float ref_A_angle_bisector = M_PI / r_gon_sides;
    float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
    float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
    float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

    if (r_gon_roundness == 0.0) {
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;

      l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d * tan(abs(ref_A_angle_bisector - ref_A_coord));
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0;
        }
        if (normalize_r_gon_parameter && (r_gon_sides != 2.0)) {
          r_gon_parameter_2d /= l_angle_bisector_2d * tan(ref_A_angle_bisector);
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter_2d = (r_gon_sides != 2.0) ? tan(ref_A_angle_bisector) : 0.0;
      }
      return vec3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
    if (r_gon_roundness == 1.0) {
      float r_gon_parameter_2d = 0.0;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = abs(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= ref_A_angle_bisector;
        }
      }
      return vec3(l_projection_2d, r_gon_parameter_2d, ref_A_angle_bisector);
    }
    else {
      float ref_A_bevel_start = ref_A_angle_bisector -
                                atan((1.0 - r_gon_roundness) * tan(ref_A_angle_bisector));

      if ((ref_A_coord >= ref_A_next_ref - ref_A_bevel_start) || (ref_A_coord < ref_A_bevel_start))
      {
        /* SA == Signed Angle in [-M_PI, M_PI]. Counterclockwise angles are positive, clockwise
         * angles are negative.*/
        float nearest_ref_SA_coord = ref_A_coord -
                                     float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
        float l_angle_bisector_2d = 0.0;
        float r_gon_parameter_2d = 0.0;
        float max_unit_parameter_2d = 0.0;

        float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
        float l_circle_center = sin(ref_A_angle_bisector - ref_A_bevel_start) /
                                sin(ref_A_angle_bisector);
        float l_coord_R_l_bevel_start = cos(nearest_ref_SA_coord) * l_circle_center +
                                        sqrt(square(cos(nearest_ref_SA_coord) * l_circle_center) +
                                             square(l_circle_radius) - square(l_circle_center));
        l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_bevel_start) /
                              l_coord_R_l_bevel_start;
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                                   tan(ref_A_angle_bisector - ref_A_bevel_start) +
                               ref_A_bevel_start - abs(nearest_ref_SA_coord);
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
        }
        return vec3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
      }
      else {
        float l_angle_bisector_2d = 0.0;
        float r_gon_parameter_2d = 0.0;
        float max_unit_parameter_2d = 0.0;

        l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d * tan(abs(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
        }
        return vec3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
      }
    }
  }
  else {
    if (r_gon_roundness == 0.0) {
      float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
      float ref_A_angle_bisector = M_PI / r_gon_sides;
      float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
      float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
      float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

      float last_angle_bisector_A_x_axis = M_PI - floor(r_gon_sides) * ref_A_angle_bisector;
      float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;

      if (x_axis_A_coord < M_TAU - last_ref_A_x_axis) {
        float l_angle_bisector_2d = 0.0;
        float r_gon_parameter_2d = 0.0;
        float max_unit_parameter_2d = 0.0;

        l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d * tan(abs(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d * tan(ref_A_angle_bisector);
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector);
        }
        return vec3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
      }
      else {
        float l_angle_bisector_2d = 0.0;
        float r_gon_parameter_2d = 0.0;
        float max_unit_parameter_2d = 0.0;

        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                               cos(last_angle_bisector_A_x_axis);
        l_angle_bisector_2d = l_projection_2d * cos(last_angle_bisector_A_x_axis - ref_A_coord) *
                              l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(abs(last_angle_bisector_A_x_axis - ref_A_coord));
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis);
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis);
        }
        return vec3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
      }
    }
    if (r_gon_roundness == 1.0) {
      if (elliptical_corners) {
        return calculate_out_fields_2d_full_roundness_irregular_elliptical(

            calculate_r_gon_parameter_field,
            normalize_r_gon_parameter,
            r_gon_sides,
            coord,
            l_projection_2d);
      }
      else {
        return calculate_out_fields_2d_full_roundness_irregular_circular(

            calculate_r_gon_parameter_field,
            calculate_max_unit_parameter,
            normalize_r_gon_parameter,
            r_gon_sides,
            coord,
            l_projection_2d);
      }
    }
    else {
      if (elliptical_corners) {
        return calculate_out_fields_2d_irregular_elliptical(calculate_r_gon_parameter_field,
                                                            calculate_max_unit_parameter,
                                                            normalize_r_gon_parameter,
                                                            r_gon_sides,
                                                            r_gon_roundness,
                                                            coord,
                                                            l_projection_2d);
      }
      else {
        return calculate_out_fields_2d_irregular_circular(calculate_r_gon_parameter_field,
                                                          calculate_max_unit_parameter,
                                                          normalize_r_gon_parameter,
                                                          r_gon_sides,
                                                          r_gon_roundness,
                                                          coord,
                                                          l_projection_2d);
      }
    }
  }
}

void node_tex_rounded_polygon(vec3 coord,
                              float scale,
                              float r_gon_sides,
                              float r_gon_roundness,
                              float normalize_r_gon_parameter,
                              float elliptical_corners,
                              float calculate_r_gon_parameter_field,
                              float calculate_max_unit_parameter,
                              out float out_r_gon_field,
                              out float out_radial_coordinates,
                              out float out_max_unit_parameter)
{
  vec3 out_variables = calculate_out_fields_2d(bool(calculate_r_gon_parameter_field),
                                               bool(calculate_max_unit_parameter),
                                               bool(normalize_r_gon_parameter),
                                               bool(elliptical_corners),
                                               max(r_gon_sides, 2.0),
                                               clamp(r_gon_roundness, 0.0, 1.0),
                                               scale * vec2(coord.x, coord.y));

  out_r_gon_field = out_variables.x;
  out_radial_coordinates = out_variables.y;
  out_max_unit_parameter = out_variables.z;
}
