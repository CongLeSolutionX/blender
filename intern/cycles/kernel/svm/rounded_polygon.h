/* SPDX-FileCopyrightText: 2024 Tenkai Raiko
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

CCL_NAMESPACE_BEGIN

struct RoundedPolygonStackOffsets {
  uint vector;
  uint scale;
  uint r_gon_sides;
  uint r_gon_roundness;
  uint r_gon_field;
  uint segment_coordinates;
  uint max_unit_parameter;
  uint x_axis_A_angle_bisector;
};

/* Naming convention for the Rounded Polygon Texture node code:
 * Let x and y be 2D vectors.
 * The length of X is expressed as l_x, which is an abbreviation of length_x.
 * The counterclockwise unsinged angle in [0.0, M_TAU_F] from X to Y is expressed as x_A_y, which
 * is an abbreviation of x_Angle_y. The singed angle in [-M_PI_F, M_PI_F] from x to y is expressed
 * as x_SA_y, which is an abbreviation of x_SingedAngle_y. Counterclockwise angles are positive,
 * clockwise angles are negative. A signed angle from x to y of which the output is mirrored along
 * a certain vector is expressed as x_MSA_y, which is an abbreviation of x_MirroredSingedAngle_y.
 *
 * Let z and w be scalars.
 * The ratio z/w is expressed as z_R_w, which is an abbreviation of z_Ratio_y. */

ccl_device float4
calculate_out_fields_full_roundness_irregular_elliptical(bool calculate_r_gon_parameter_field,
                                                         bool normalize_r_gon_parameter,
                                                         float r_gon_sides,
                                                         float2 coord,
                                                         float l_coord)
{
  float x_axis_A_coord = atan2f(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;

  if ((x_axis_A_coord >= ref_A_angle_bisector) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector))
  {
    float r_gon_parameter = 0.0f;
    if (calculate_r_gon_parameter_field) {
      r_gon_parameter = fabsf(ref_A_angle_bisector - ref_A_coord);
      if (ref_A_coord < ref_A_angle_bisector) {
        r_gon_parameter *= -1.0f;
      }
      if (normalize_r_gon_parameter) {
        r_gon_parameter /= ref_A_angle_bisector;
      }
    }
    return make_float4(l_coord,
                       r_gon_parameter,
                       ref_A_angle_bisector,
                       segment_id * ref_A_next_ref + ref_A_angle_bisector);
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2f(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) &&
        (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1.0f;
    }
    float l_angle_bisector = 0.0f;
    float r_gon_parameter = 0.0f;
    float max_unit_parameter = 0.0f;
    float x_axis_A_angle_bisector = 0.0f;

    float l_basis_vector_1 = tanf(ref_A_angle_bisector);
    /* When the fractional part of r_gon_sides is very small division by l_basis_vector_2 causes
     * precision issues. Change to double if necessary */
    float l_basis_vector_2 = sinf(last_angle_bisector_A_x_axis) *
                             sqrtf(squaref(tanf(ref_A_angle_bisector)) + 1.0f);
    float2 ellipse_center = make_float2(cosf(ref_A_angle_bisector) /
                                            cosf(ref_A_angle_bisector - ref_A_angle_bisector),
                                        sinf(ref_A_angle_bisector) /
                                            cosf(ref_A_angle_bisector - ref_A_angle_bisector)) -
                            l_basis_vector_2 * make_float2(sinf(last_angle_bisector_A_x_axis),
                                                           cosf(last_angle_bisector_A_x_axis));
    float2 transformed_direction_vector = make_float2(
        cosf(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
            (l_basis_vector_1 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
        cosf(ref_A_angle_bisector - nearest_ref_MSA_coord) /
            (l_basis_vector_2 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
    float2 transformed_origin = make_float2(
        (ellipse_center.y * sinf(last_angle_bisector_A_x_axis) -
         ellipse_center.x * cosf(last_angle_bisector_A_x_axis)) /
            (l_basis_vector_1 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
        -(ellipse_center.y * sinf(ref_A_angle_bisector) +
          ellipse_center.x * cosf(ref_A_angle_bisector)) /
            (l_basis_vector_2 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
    float l_coord_R_l_angle_bisector =
        (-(transformed_direction_vector.x * transformed_origin.x +
           transformed_direction_vector.y * transformed_origin.y) +
         sqrtf(
             squaref(transformed_direction_vector.x * transformed_origin.x +
                     transformed_direction_vector.y * transformed_origin.y) -
             (squaref(transformed_direction_vector.x) + squaref(transformed_direction_vector.y)) *
                 (squaref(transformed_origin.x) + squaref(transformed_origin.y) - 1.0f))) /
        (squaref(transformed_direction_vector.x) + squaref(transformed_direction_vector.y));

    l_angle_bisector = l_coord / l_coord_R_l_angle_bisector;

    if (nearest_ref_MSA_coord < 0.0f) {
      float l_angle_bisector_R_l_last_angle_bisector = cosf(ref_A_angle_bisector) /
                                                       cosf(last_angle_bisector_A_x_axis);
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = l_angle_bisector_R_l_last_angle_bisector *
                          (last_angle_bisector_A_x_axis + nearest_ref_MSA_coord);
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter /= l_angle_bisector_R_l_last_angle_bisector *
                             last_angle_bisector_A_x_axis;
        }
      }
      max_unit_parameter = l_angle_bisector_R_l_last_angle_bisector * last_angle_bisector_A_x_axis;
      x_axis_A_angle_bisector = segment_id * ref_A_next_ref + last_angle_bisector_A_x_axis;
    }
    else {
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = fabsf(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter /= ref_A_angle_bisector;
        }
      }
      max_unit_parameter = ref_A_angle_bisector;
      x_axis_A_angle_bisector = segment_id * ref_A_next_ref + ref_A_angle_bisector;
    }
    return make_float4(
        l_angle_bisector, r_gon_parameter, max_unit_parameter, x_axis_A_angle_bisector);
  }
}

ccl_device float4 calculate_out_fields_irregular_elliptical(bool calculate_r_gon_parameter_field,
                                                            bool calculate_max_unit_parameter,
                                                            bool normalize_r_gon_parameter,
                                                            float r_gon_sides,
                                                            float r_gon_roundness,
                                                            float2 coord,
                                                            float l_coord)
{
  float x_axis_A_coord = atan2f(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atanf((1.0f - r_gon_roundness) * tanf(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atanf((1.0f - r_gon_roundness) *
                                                tanf(last_angle_bisector_A_x_axis));
  float inner_last_bevel_start_A_last_angle_bisector = last_angle_bisector_A_x_axis -
                                                       inner_last_bevel_start_A_x_axis;

  if ((x_axis_A_coord >= ref_A_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start))
  {
    float bevel_start_A_angle_bisector = ref_A_angle_bisector - ref_A_bevel_start;

    if ((ref_A_coord >= ref_A_bevel_start) && (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) {
      float l_angle_bisector = 0.0f;
      float r_gon_parameter = 0.0f;
      float max_unit_parameter = 0.0f;

      l_angle_bisector = l_coord * cosf(ref_A_angle_bisector - ref_A_coord);

      float spline_start_bevel_start = (1.0f - r_gon_roundness) * ref_A_bevel_start;

      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = l_angle_bisector * tanf(fabsf(ref_A_angle_bisector - ref_A_coord));
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          float normalize_based_on_l_angle_bisector = l_angle_bisector *
                                                          tanf(bevel_start_A_angle_bisector) +
                                                      spline_start_bevel_start *
                                                          (0.5f * l_angle_bisector + 0.5f) +
                                                      r_gon_roundness * ref_A_bevel_start;

          r_gon_parameter /= normalize_based_on_l_angle_bisector;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter = tanf(bevel_start_A_angle_bisector) + spline_start_bevel_start +
                             r_gon_roundness * ref_A_bevel_start;
      }
      return make_float4(l_angle_bisector,
                         r_gon_parameter,
                         max_unit_parameter,
                         segment_id * ref_A_next_ref + ref_A_angle_bisector);
    }
    else {
      /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_angle_bisector = 0.0f;
      float r_gon_parameter = 0.0f;
      float max_unit_parameter = 0.0f;

      float l_circle_radius = sinf(ref_A_bevel_start) / sinf(ref_A_angle_bisector);
      float l_circle_center = sinf(bevel_start_A_angle_bisector) / sinf(ref_A_angle_bisector);
      float l_coord_R_l_bevel_start = cosf(nearest_ref_SA_coord) * l_circle_center +
                                      sqrtf(squaref(cosf(nearest_ref_SA_coord) * l_circle_center) +
                                            squaref(l_circle_radius) - squaref(l_circle_center));

      l_angle_bisector = l_coord * cosf(bevel_start_A_angle_bisector) / l_coord_R_l_bevel_start;

      float spline_start_bevel_start = (1.0f - r_gon_roundness) * ref_A_bevel_start;

      if (calculate_r_gon_parameter_field) {
        float coord_A_bevel_start = ref_A_bevel_start - fabsf(nearest_ref_SA_coord);
        r_gon_parameter = l_coord * sinf(bevel_start_A_angle_bisector);

        if (coord_A_bevel_start < spline_start_bevel_start) {
          r_gon_parameter += l_coord * cosf(bevel_start_A_angle_bisector) * coord_A_bevel_start +
                             0.5f * (1.0f - l_coord * cosf(bevel_start_A_angle_bisector)) *
                                 squaref(coord_A_bevel_start) / spline_start_bevel_start;
        }
        else {
          r_gon_parameter += spline_start_bevel_start *
                                 (0.5f * l_coord * cosf(bevel_start_A_angle_bisector) - 0.5f) +
                             coord_A_bevel_start;
        }
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          float normalize_based_on_l_angle_bisector = l_angle_bisector *
                                                          tanf(bevel_start_A_angle_bisector) +
                                                      spline_start_bevel_start *
                                                          (0.5f * l_angle_bisector + 0.5f) +
                                                      r_gon_roundness * ref_A_bevel_start;
          float normalize_based_on_l_coord =
              l_coord * sinf(bevel_start_A_angle_bisector) +
              spline_start_bevel_start *
                  (0.5f * l_coord * cosf(bevel_start_A_angle_bisector) + 0.5f) +
              r_gon_roundness * ref_A_bevel_start;

          /* For r_gon_roundness -> 1.0 the normalize_based_on_l_angle_bisector field and
           * normalize_based_on_l_coord field converge against the same scalar field. */
          r_gon_parameter /= mix(normalize_based_on_l_angle_bisector,
                                 normalize_based_on_l_coord,
                                 coord_A_bevel_start / ref_A_bevel_start);
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter = tanf(bevel_start_A_angle_bisector) + spline_start_bevel_start +
                             r_gon_roundness * ref_A_bevel_start;
      }
      return make_float4(l_angle_bisector,
                         r_gon_parameter,
                         max_unit_parameter,
                         segment_id * ref_A_next_ref + ref_A_angle_bisector);
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU_F - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector = 0.0f;
      float r_gon_parameter = 0.0f;
      float max_unit_parameter = 0.0f;

      float l_angle_bisector_R_l_last_angle_bisector = cosf(ref_A_angle_bisector) /
                                                       cosf(last_angle_bisector_A_x_axis);
      float l_last_angle_bisector = l_coord * cosf(last_angle_bisector_A_x_axis - ref_A_coord);

      l_angle_bisector = l_angle_bisector_R_l_last_angle_bisector * l_last_angle_bisector;

      float spline_start_bevel_start = (1.0f - r_gon_roundness) * inner_last_bevel_start_A_x_axis;

      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = l_angle_bisector_R_l_last_angle_bisector * l_last_angle_bisector *
                          tanf(fabsf(last_angle_bisector_A_x_axis - ref_A_coord));
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          float normalize_based_on_l_l_angle_bisector =
              (l_last_angle_bisector * tanf(inner_last_bevel_start_A_last_angle_bisector) +
               spline_start_bevel_start * (0.5f * l_last_angle_bisector + 0.5f) +
               r_gon_roundness * inner_last_bevel_start_A_x_axis);

          r_gon_parameter /= l_angle_bisector_R_l_last_angle_bisector *
                             normalize_based_on_l_l_angle_bisector;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter = tanf(inner_last_bevel_start_A_last_angle_bisector) +
                             l_angle_bisector_R_l_last_angle_bisector *
                                 (spline_start_bevel_start *
                                      ((0.5f / l_angle_bisector_R_l_last_angle_bisector) + 0.5f) +
                                  r_gon_roundness * inner_last_bevel_start_A_x_axis);
      }
      return make_float4(l_angle_bisector,
                         r_gon_parameter,
                         max_unit_parameter,
                         segment_id * ref_A_next_ref + last_angle_bisector_A_x_axis);
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2f(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start) &&
          (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1.0f;
      }
      float bevel_start_A_angle_bisector = ref_A_angle_bisector - ref_A_bevel_start;

      float l_angle_bisector = 0.0f;
      float r_gon_parameter = 0.0f;
      float max_unit_parameter = 0.0f;
      float x_axis_A_angle_bisector = 0.0f;

      float l_basis_vector_1 = r_gon_roundness * tanf(ref_A_angle_bisector);
      float l_basis_vector_2 = r_gon_roundness * sinf(last_angle_bisector_A_x_axis) *
                               sqrtf(squaref(tanf(ref_A_angle_bisector)) + 1.0f);
      float2 ellipse_center = make_float2(
                                  cosf(ref_A_bevel_start) / cosf(bevel_start_A_angle_bisector),
                                  sinf(ref_A_bevel_start) / cosf(bevel_start_A_angle_bisector)) -
                              l_basis_vector_2 * make_float2(sinf(last_angle_bisector_A_x_axis),
                                                             cosf(last_angle_bisector_A_x_axis));
      float2 transformed_direction_vector = make_float2(
          cosf(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
              (l_basis_vector_1 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          cosf(ref_A_angle_bisector - nearest_ref_MSA_coord) /
              (l_basis_vector_2 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      float2 transformed_origin = make_float2(
          (ellipse_center.y * sinf(last_angle_bisector_A_x_axis) -
           ellipse_center.x * cosf(last_angle_bisector_A_x_axis)) /
              (l_basis_vector_1 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          -(ellipse_center.y * sinf(ref_A_angle_bisector) +
            ellipse_center.x * cosf(ref_A_angle_bisector)) /
              (l_basis_vector_2 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      float l_coord_R_l_angle_bisector =
          (-(transformed_direction_vector.x * transformed_origin.x +
             transformed_direction_vector.y * transformed_origin.y) +
           sqrtf(squaref(transformed_direction_vector.x * transformed_origin.x +
                         transformed_direction_vector.y * transformed_origin.y) -
                 (squaref(transformed_direction_vector.x) +
                  squaref(transformed_direction_vector.y)) *
                     (squaref(transformed_origin.x) + squaref(transformed_origin.y) - 1.0f))) /
          (squaref(transformed_direction_vector.x) + squaref(transformed_direction_vector.y));

      l_angle_bisector = l_coord / l_coord_R_l_angle_bisector;

      if (nearest_ref_MSA_coord < 0.0f) {
        float l_angle_bisector_R_l_last_angle_bisector = cosf(ref_A_angle_bisector) /
                                                         cosf(last_angle_bisector_A_x_axis);
        float l_last_angle_bisector = l_angle_bisector / l_angle_bisector_R_l_last_angle_bisector;

        float spline_start_bevel_start = (1.0f - r_gon_roundness) *
                                         inner_last_bevel_start_A_x_axis;

        if (calculate_r_gon_parameter_field) {
          float coord_A_bevel_start = inner_last_bevel_start_A_x_axis -
                                      fabsf(nearest_ref_MSA_coord);
          r_gon_parameter = l_angle_bisector_R_l_last_angle_bisector * l_coord *
                            sinf(inner_last_bevel_start_A_last_angle_bisector);

          if (coord_A_bevel_start < spline_start_bevel_start) {
            r_gon_parameter +=
                l_angle_bisector_R_l_last_angle_bisector *
                (l_coord * cosf(inner_last_bevel_start_A_last_angle_bisector) *
                     coord_A_bevel_start +
                 0.5f * (1.0f - l_coord * cosf(inner_last_bevel_start_A_last_angle_bisector)) *
                     squaref(coord_A_bevel_start) / spline_start_bevel_start);
          }
          else {
            r_gon_parameter +=
                l_angle_bisector_R_l_last_angle_bisector *
                (spline_start_bevel_start *
                     (0.5f * l_coord * cosf(inner_last_bevel_start_A_last_angle_bisector) - 0.5f) +
                 coord_A_bevel_start);
          }
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            float normalize_based_on_l_l_angle_bisector =
                l_last_angle_bisector * tanf(inner_last_bevel_start_A_last_angle_bisector) +
                spline_start_bevel_start * (0.5f * l_last_angle_bisector + 0.5f) +
                r_gon_roundness * inner_last_bevel_start_A_x_axis;
            float normalize_based_on_l_coord =
                l_coord * sinf(inner_last_bevel_start_A_last_angle_bisector) +
                spline_start_bevel_start *
                    (0.5f * l_coord * cosf(inner_last_bevel_start_A_last_angle_bisector) + 0.5f) +
                r_gon_roundness * inner_last_bevel_start_A_x_axis;

            /* For r_gon_roundness -> 1.0 the normalize_based_on_l_l_angle_bisector field and
             * normalize_based_on_l_coord field converge against the same scalar field. */
            r_gon_parameter /= l_angle_bisector_R_l_last_angle_bisector *
                               mix(normalize_based_on_l_l_angle_bisector,
                                   normalize_based_on_l_coord,
                                   coord_A_bevel_start / inner_last_bevel_start_A_x_axis);
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter = tanf(inner_last_bevel_start_A_last_angle_bisector) +
                               l_angle_bisector_R_l_last_angle_bisector *
                                   (spline_start_bevel_start *
                                        ((0.5f / l_angle_bisector_R_l_last_angle_bisector) +
                                         0.5f) +
                                    r_gon_roundness * inner_last_bevel_start_A_x_axis);
        }
        x_axis_A_angle_bisector = segment_id * ref_A_next_ref + last_angle_bisector_A_x_axis;
      }
      else {
        float spline_start_bevel_start = (1.0f - r_gon_roundness) * ref_A_bevel_start;

        if (calculate_r_gon_parameter_field) {
          float coord_A_bevel_start = ref_A_bevel_start - fabsf(nearest_ref_MSA_coord);
          r_gon_parameter = l_coord * sinf(bevel_start_A_angle_bisector);

          if (coord_A_bevel_start < spline_start_bevel_start) {
            r_gon_parameter += l_coord * cosf(bevel_start_A_angle_bisector) * coord_A_bevel_start +
                               0.5f * (1.0f - l_coord * cosf(bevel_start_A_angle_bisector)) *
                                   squaref(coord_A_bevel_start) / spline_start_bevel_start;
          }
          else {
            r_gon_parameter += spline_start_bevel_start *
                                   (0.5f * l_coord * cosf(bevel_start_A_angle_bisector) - 0.5f) +
                               coord_A_bevel_start;
          }
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            float normalize_based_on_l_angle_bisector = l_angle_bisector *
                                                            tanf(bevel_start_A_angle_bisector) +
                                                        spline_start_bevel_start *
                                                            (0.5f * l_angle_bisector + 0.5f) +
                                                        r_gon_roundness * ref_A_bevel_start;
            float normalize_based_on_l_coord =
                l_coord * sinf(bevel_start_A_angle_bisector) +
                spline_start_bevel_start *
                    (0.5f * l_coord * cosf(bevel_start_A_angle_bisector) + 0.5f) +
                r_gon_roundness * ref_A_bevel_start;

            /* For r_gon_roundness -> 1.0 the normalize_based_on_l_angle_bisector field and
             * normalize_based_on_l_coord field converge against the same scalar field. */
            r_gon_parameter /= mix(normalize_based_on_l_angle_bisector,
                                   normalize_based_on_l_coord,
                                   coord_A_bevel_start / ref_A_bevel_start);
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter = tanf(bevel_start_A_angle_bisector) + spline_start_bevel_start +
                               r_gon_roundness * ref_A_bevel_start;
        }
        x_axis_A_angle_bisector = segment_id * ref_A_next_ref + ref_A_angle_bisector;
      }
      return make_float4(
          l_angle_bisector, r_gon_parameter, max_unit_parameter, x_axis_A_angle_bisector);
    }
  }
}

ccl_device float4
calculate_out_fields_full_roundness_irregular_circular(bool calculate_r_gon_parameter_field,
                                                       bool calculate_max_unit_parameter,
                                                       bool normalize_r_gon_parameter,
                                                       float r_gon_sides,
                                                       float2 coord,
                                                       float l_coord)
{
  float x_axis_A_coord = atan2f(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float l_last_circle_radius = tanf(last_angle_bisector_A_x_axis) /
                               tanf(0.5f * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  float2 last_circle_center = make_float2(
      cosf(last_angle_bisector_A_x_axis) -
          l_last_circle_radius * cosf(last_angle_bisector_A_x_axis),
      l_last_circle_radius * sinf(last_angle_bisector_A_x_axis) -
          sinf(last_angle_bisector_A_x_axis));
  float2 outer_last_bevel_start = last_circle_center +
                                  l_last_circle_radius * make_float2(cosf(ref_A_angle_bisector),
                                                                     sinf(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atanf(outer_last_bevel_start.y /
                                                outer_last_bevel_start.x);
  float outer_last_bevel_start_A_angle_bisector = ref_A_angle_bisector -
                                                  x_axis_A_outer_last_bevel_start;

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) ||
        (x_axis_A_coord < ref_A_angle_bisector))
    {
      float l_angle_bisector = 0.0f;
      float r_gon_parameter = 0.0f;
      float max_unit_parameter = 0.0f;

      l_angle_bisector = l_coord * cosf(ref_A_angle_bisector - ref_A_coord);

      float effective_roundness = 1.0f -
                                  tanf(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) /
                                      tanf(ref_A_angle_bisector);
      float spline_start_bevel_start = (1.0f - effective_roundness) *
                                       x_axis_A_outer_last_bevel_start;

      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = l_angle_bisector * tanf(fabsf(ref_A_angle_bisector - ref_A_coord));
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          float normalize_based_on_l_angle_bisector =
              l_angle_bisector * tanf(outer_last_bevel_start_A_angle_bisector) +
              spline_start_bevel_start * (0.5f * l_angle_bisector + 0.5f) +
              effective_roundness * x_axis_A_outer_last_bevel_start;

          r_gon_parameter /= normalize_based_on_l_angle_bisector;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter = tanf(outer_last_bevel_start_A_angle_bisector) +
                             spline_start_bevel_start +
                             effective_roundness * x_axis_A_outer_last_bevel_start;
      }
      return make_float4(l_angle_bisector,
                         r_gon_parameter,
                         max_unit_parameter,
                         segment_id * ref_A_next_ref + ref_A_angle_bisector);
    }
    else {
      float r_gon_parameter = 0.0f;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = fabsf(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter /= ref_A_angle_bisector;
        }
      }
      return make_float4(l_coord,
                         r_gon_parameter,
                         ref_A_angle_bisector,
                         segment_id * ref_A_next_ref + ref_A_angle_bisector);
    }
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2f(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
        (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1.0f;
    }
    float l_angle_bisector = 0.0f;
    float r_gon_parameter = 0.0f;
    float max_unit_parameter = 0.0f;
    float x_axis_A_angle_bisector = 0.0f;

    float l_coord_R_l_last_angle_bisector =
        sinf(nearest_ref_MSA_coord) * last_circle_center.y +
        cosf(nearest_ref_MSA_coord) * last_circle_center.x +
        sqrtf(squaref(sinf(nearest_ref_MSA_coord) * last_circle_center.y +
                      cosf(nearest_ref_MSA_coord) * last_circle_center.x) +
              squaref(l_last_circle_radius) - squaref(last_circle_center.x) -
              squaref(last_circle_center.y));
    float l_angle_bisector_R_l_last_angle_bisector = cosf(ref_A_angle_bisector) /
                                                     cosf(last_angle_bisector_A_x_axis);

    l_angle_bisector = l_angle_bisector_R_l_last_angle_bisector * l_coord /
                       l_coord_R_l_last_angle_bisector;

    if (nearest_ref_MSA_coord < 0.0f) {
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = l_angle_bisector_R_l_last_angle_bisector *
                          (last_angle_bisector_A_x_axis + nearest_ref_MSA_coord);
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter /= l_angle_bisector_R_l_last_angle_bisector *
                             last_angle_bisector_A_x_axis;
        }
      }
      max_unit_parameter = l_angle_bisector_R_l_last_angle_bisector * last_angle_bisector_A_x_axis;
      x_axis_A_angle_bisector = segment_id * ref_A_next_ref + last_angle_bisector_A_x_axis;
    }
    else {
      float effective_roundness = 1.0f -
                                  tanf(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) /
                                      tanf(ref_A_angle_bisector);
      float spline_start_bevel_start = (1.0f - effective_roundness) *
                                       x_axis_A_outer_last_bevel_start;

      if (calculate_r_gon_parameter_field) {
        float coord_A_bevel_start = x_axis_A_outer_last_bevel_start - fabsf(nearest_ref_MSA_coord);
        r_gon_parameter = l_coord * sinf(outer_last_bevel_start_A_angle_bisector);

        if (coord_A_bevel_start < spline_start_bevel_start) {
          r_gon_parameter += l_coord * cosf(outer_last_bevel_start_A_angle_bisector) *
                                 coord_A_bevel_start +
                             0.5f *
                                 (1.0f - l_coord * cosf(outer_last_bevel_start_A_angle_bisector)) *
                                 squaref(coord_A_bevel_start) / spline_start_bevel_start;
        }
        else {
          r_gon_parameter += spline_start_bevel_start *
                                 (0.5f * l_coord * cosf(outer_last_bevel_start_A_angle_bisector) -
                                  0.5f) +
                             coord_A_bevel_start;
        }
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          float normalize_based_on_l_angle_bisector =
              l_angle_bisector * tanf(outer_last_bevel_start_A_angle_bisector) +
              spline_start_bevel_start * (0.5f * l_angle_bisector + 0.5f) +
              effective_roundness * x_axis_A_outer_last_bevel_start;
          float normalize_based_on_l_coord =
              l_coord * sinf(outer_last_bevel_start_A_angle_bisector) +
              spline_start_bevel_start *
                  (0.5f * l_coord * cosf(outer_last_bevel_start_A_angle_bisector) + 0.5f) +
              effective_roundness * x_axis_A_outer_last_bevel_start;

          /* For effective_roundness -> 1.0 the normalize_based_on_l_angle_bisector field and
           * normalize_based_on_l_coord field converge against the same scalar field. */
          r_gon_parameter /= mix(normalize_based_on_l_angle_bisector,
                                 normalize_based_on_l_coord,
                                 coord_A_bevel_start / x_axis_A_outer_last_bevel_start);
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter = tanf(outer_last_bevel_start_A_angle_bisector) +
                             spline_start_bevel_start +
                             effective_roundness * x_axis_A_outer_last_bevel_start;
      }
      x_axis_A_angle_bisector = segment_id * ref_A_next_ref + ref_A_angle_bisector;
    }
    return make_float4(
        l_angle_bisector, r_gon_parameter, max_unit_parameter, x_axis_A_angle_bisector);
  }
}

ccl_device float4 calculate_out_fields_irregular_circular(bool calculate_r_gon_parameter_field,
                                                          bool calculate_max_unit_parameter,
                                                          bool normalize_r_gon_parameter,
                                                          float r_gon_sides,
                                                          float r_gon_roundness,
                                                          float2 coord,
                                                          float l_coord)
{
  float x_axis_A_coord = atan2f(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atanf((1.0f - r_gon_roundness) * tanf(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atanf((1.0f - r_gon_roundness) *
                                                tanf(last_angle_bisector_A_x_axis));
  float l_last_circle_radius = r_gon_roundness * tanf(last_angle_bisector_A_x_axis) /
                               tanf(0.5f * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  float2 last_circle_center = make_float2(
      (cosf(inner_last_bevel_start_A_x_axis) /
       cosf(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)) -
          l_last_circle_radius * cosf(last_angle_bisector_A_x_axis),
      l_last_circle_radius * sinf(last_angle_bisector_A_x_axis) -
          (sinf(inner_last_bevel_start_A_x_axis) /
           cosf(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)));
  float2 outer_last_bevel_start = last_circle_center +
                                  l_last_circle_radius * make_float2(cosf(ref_A_angle_bisector),
                                                                     sinf(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atanf(outer_last_bevel_start.y /
                                                outer_last_bevel_start.x);
  float outer_last_bevel_start_A_angle_bisector = ref_A_angle_bisector -
                                                  x_axis_A_outer_last_bevel_start;

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    float bevel_start_A_angle_bisector = ref_A_angle_bisector - ref_A_bevel_start;

    if (((ref_A_coord >= ref_A_bevel_start) &&
         (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) ||
        (x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start) ||
        (x_axis_A_coord < ref_A_bevel_start))
    {
      float l_angle_bisector = 0.0f;
      float r_gon_parameter = 0.0f;
      float max_unit_parameter = 0.0f;

      l_angle_bisector = l_coord * cosf(ref_A_angle_bisector - ref_A_coord);

      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) ||
          (x_axis_A_coord < ref_A_angle_bisector))
      {
        float effective_roundness = 1.0f -
                                    tanf(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) /
                                        tanf(ref_A_angle_bisector);
        float spline_start_bevel_start = (1.0f - effective_roundness) *
                                         x_axis_A_outer_last_bevel_start;

        if (calculate_r_gon_parameter_field) {
          r_gon_parameter = l_angle_bisector * tanf(fabsf(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            float normalize_based_on_l_angle_bisector =
                l_angle_bisector * tanf(outer_last_bevel_start_A_angle_bisector) +
                spline_start_bevel_start * (0.5f * l_angle_bisector + 0.5f) +
                effective_roundness * x_axis_A_outer_last_bevel_start;

            r_gon_parameter /= normalize_based_on_l_angle_bisector;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter = tanf(outer_last_bevel_start_A_angle_bisector) +
                               spline_start_bevel_start +
                               effective_roundness * x_axis_A_outer_last_bevel_start;
        }
      }
      else {
        float spline_start_bevel_start = (1.0f - r_gon_roundness) * ref_A_bevel_start;

        if (calculate_r_gon_parameter_field) {
          r_gon_parameter = l_angle_bisector * tanf(fabsf(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            float normalize_based_on_l_angle_bisector = l_angle_bisector *
                                                            tanf(bevel_start_A_angle_bisector) +
                                                        spline_start_bevel_start *
                                                            (0.5f * l_angle_bisector + 0.5f) +
                                                        r_gon_roundness * ref_A_bevel_start;

            r_gon_parameter /= normalize_based_on_l_angle_bisector;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter = tanf(bevel_start_A_angle_bisector) + spline_start_bevel_start +
                               r_gon_roundness * ref_A_bevel_start;
        }
      }
      return make_float4(l_angle_bisector,
                         r_gon_parameter,
                         max_unit_parameter,
                         segment_id * ref_A_next_ref + ref_A_angle_bisector);
    }
    else {
      /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_angle_bisector = 0.0f;
      float r_gon_parameter = 0.0f;
      float max_unit_parameter = 0.0f;

      float l_circle_radius = sinf(ref_A_bevel_start) / sinf(ref_A_angle_bisector);
      float l_circle_center = sinf(bevel_start_A_angle_bisector) / sinf(ref_A_angle_bisector);
      float l_coord_R_l_bevel_start = cosf(nearest_ref_SA_coord) * l_circle_center +
                                      sqrtf(squaref(cosf(nearest_ref_SA_coord) * l_circle_center) +
                                            squaref(l_circle_radius) - squaref(l_circle_center));

      l_angle_bisector = l_coord * cosf(bevel_start_A_angle_bisector) / l_coord_R_l_bevel_start;

      float spline_start_bevel_start = (1.0f - r_gon_roundness) * ref_A_bevel_start;

      if (calculate_r_gon_parameter_field) {
        float coord_A_bevel_start = ref_A_bevel_start - fabsf(nearest_ref_SA_coord);
        r_gon_parameter = l_coord * sinf(bevel_start_A_angle_bisector);

        if (coord_A_bevel_start < spline_start_bevel_start) {
          r_gon_parameter += l_coord * cosf(bevel_start_A_angle_bisector) * coord_A_bevel_start +
                             0.5f * (1.0f - l_coord * cosf(bevel_start_A_angle_bisector)) *
                                 squaref(coord_A_bevel_start) / spline_start_bevel_start;
        }
        else {
          r_gon_parameter += spline_start_bevel_start *
                                 (0.5f * l_coord * cosf(bevel_start_A_angle_bisector) - 0.5f) +
                             coord_A_bevel_start;
        }
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          float normalize_based_on_l_angle_bisector = l_angle_bisector *
                                                          tanf(bevel_start_A_angle_bisector) +
                                                      spline_start_bevel_start *
                                                          (0.5f * l_angle_bisector + 0.5f) +
                                                      r_gon_roundness * ref_A_bevel_start;
          float normalize_based_on_l_coord =
              l_coord * sinf(bevel_start_A_angle_bisector) +
              spline_start_bevel_start *
                  (0.5f * l_coord * cosf(bevel_start_A_angle_bisector) + 0.5f) +
              r_gon_roundness * ref_A_bevel_start;

          /* For r_gon_roundness -> 1.0 the normalize_based_on_l_angle_bisector field and
           * normalize_based_on_l_coord field converge against the same scalar field. */
          r_gon_parameter /= mix(normalize_based_on_l_angle_bisector,
                                 normalize_based_on_l_coord,
                                 coord_A_bevel_start / ref_A_bevel_start);
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter = tanf(bevel_start_A_angle_bisector) + spline_start_bevel_start +
                             r_gon_roundness * ref_A_bevel_start;
      }
      return make_float4(l_angle_bisector,
                         r_gon_parameter,
                         max_unit_parameter,
                         segment_id * ref_A_next_ref + ref_A_angle_bisector);
    }
  }
  else {
    float inner_last_bevel_start_A_last_angle_bisector = last_angle_bisector_A_x_axis -
                                                         inner_last_bevel_start_A_x_axis;

    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU_F - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector = 0.0f;
      float r_gon_parameter = 0.0f;
      float max_unit_parameter = 0.0f;

      float l_angle_bisector_R_l_last_angle_bisector = cosf(ref_A_angle_bisector) /
                                                       cosf(last_angle_bisector_A_x_axis);
      float l_last_angle_bisector = l_coord * cosf(last_angle_bisector_A_x_axis - ref_A_coord);

      l_angle_bisector = l_angle_bisector_R_l_last_angle_bisector * l_last_angle_bisector;

      float spline_start_bevel_start = (1.0f - r_gon_roundness) * inner_last_bevel_start_A_x_axis;

      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = l_angle_bisector_R_l_last_angle_bisector * l_last_angle_bisector *
                          tanf(fabsf(last_angle_bisector_A_x_axis - ref_A_coord));
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          float normalize_based_on_l_l_angle_bisector =
              l_angle_bisector_R_l_last_angle_bisector *
              (l_last_angle_bisector * tanf(inner_last_bevel_start_A_last_angle_bisector) +
               spline_start_bevel_start * (0.5f * l_last_angle_bisector + 0.5f) +
               r_gon_roundness * inner_last_bevel_start_A_x_axis);

          r_gon_parameter /= normalize_based_on_l_l_angle_bisector;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter = tanf(inner_last_bevel_start_A_last_angle_bisector) +
                             l_angle_bisector_R_l_last_angle_bisector *
                                 (spline_start_bevel_start *
                                      ((0.5f / l_angle_bisector_R_l_last_angle_bisector) + 0.5f) +
                                  r_gon_roundness * inner_last_bevel_start_A_x_axis);
      }
      return make_float4(l_angle_bisector,
                         r_gon_parameter,
                         max_unit_parameter,
                         segment_id * ref_A_next_ref + last_angle_bisector_A_x_axis);
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2f(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
          (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1.0f;
      }
      float l_angle_bisector = 0.0f;
      float r_gon_parameter = 0.0f;
      float max_unit_parameter = 0.0f;
      float x_axis_A_angle_bisector = 0.0f;

      float l_coord_R_l_last_angle_bisector =
          sinf(nearest_ref_MSA_coord) * last_circle_center.y +
          cosf(nearest_ref_MSA_coord) * last_circle_center.x +
          sqrtf(squaref(sinf(nearest_ref_MSA_coord) * last_circle_center.y +
                        cosf(nearest_ref_MSA_coord) * last_circle_center.x) +
                squaref(l_last_circle_radius) - squaref(last_circle_center.x) -
                squaref(last_circle_center.y));
      float l_angle_bisector_R_l_last_angle_bisector = cosf(ref_A_angle_bisector) /
                                                       cosf(last_angle_bisector_A_x_axis);
      float l_last_angle_bisector = l_coord / l_coord_R_l_last_angle_bisector;

      l_angle_bisector = l_angle_bisector_R_l_last_angle_bisector * l_last_angle_bisector;

      if (nearest_ref_MSA_coord < 0.0f) {
        float spline_start_bevel_start = (1.0f - r_gon_roundness) *
                                         inner_last_bevel_start_A_x_axis;

        if (calculate_r_gon_parameter_field) {
          float coord_A_bevel_start = inner_last_bevel_start_A_x_axis -
                                      fabsf(nearest_ref_MSA_coord);
          r_gon_parameter = l_angle_bisector_R_l_last_angle_bisector * l_coord *
                            sinf(inner_last_bevel_start_A_last_angle_bisector);

          if (coord_A_bevel_start < spline_start_bevel_start) {
            r_gon_parameter +=
                l_angle_bisector_R_l_last_angle_bisector *
                (l_coord * cosf(inner_last_bevel_start_A_last_angle_bisector) *
                     coord_A_bevel_start +
                 0.5f * (1.0f - l_coord * cosf(inner_last_bevel_start_A_last_angle_bisector)) *
                     squaref(coord_A_bevel_start) / spline_start_bevel_start);
          }
          else {
            r_gon_parameter +=
                l_angle_bisector_R_l_last_angle_bisector *
                (spline_start_bevel_start *
                     (0.5f * l_coord * cosf(inner_last_bevel_start_A_last_angle_bisector) - 0.5f) +
                 coord_A_bevel_start);
          }
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            float normalize_based_on_l_l_angle_bisector =
                l_last_angle_bisector * tanf(inner_last_bevel_start_A_last_angle_bisector) +
                spline_start_bevel_start * (0.5f * l_last_angle_bisector + 0.5f) +
                r_gon_roundness * inner_last_bevel_start_A_x_axis;
            float normalize_based_on_l_coord =
                l_coord * sinf(inner_last_bevel_start_A_last_angle_bisector) +
                spline_start_bevel_start *
                    (0.5f * l_coord * cosf(inner_last_bevel_start_A_last_angle_bisector) + 0.5f) +
                r_gon_roundness * inner_last_bevel_start_A_x_axis;

            /* For r_gon_roundness -> 1.0 the normalize_based_on_l_l_angle_bisector field and
             * normalize_based_on_l_coord field converge against the same scalar field. */
            r_gon_parameter /= l_angle_bisector_R_l_last_angle_bisector *
                               mix(normalize_based_on_l_l_angle_bisector,
                                   normalize_based_on_l_coord,
                                   coord_A_bevel_start / inner_last_bevel_start_A_x_axis);
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter = tanf(inner_last_bevel_start_A_last_angle_bisector) +
                               l_angle_bisector_R_l_last_angle_bisector *
                                   (spline_start_bevel_start *
                                        ((0.5f / l_angle_bisector_R_l_last_angle_bisector) +
                                         0.5f) +
                                    r_gon_roundness * inner_last_bevel_start_A_x_axis);
        }
        x_axis_A_angle_bisector = segment_id * ref_A_next_ref + last_angle_bisector_A_x_axis;
      }
      else {
        float effective_roundness = 1.0f -
                                    tanf(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) /
                                        tanf(ref_A_angle_bisector);
        float spline_start_bevel_start = (1.0f - effective_roundness) *
                                         x_axis_A_outer_last_bevel_start;

        if (calculate_r_gon_parameter_field) {
          float coord_A_bevel_start = x_axis_A_outer_last_bevel_start -
                                      fabsf(nearest_ref_MSA_coord);
          r_gon_parameter = l_coord * sinf(outer_last_bevel_start_A_angle_bisector);

          if (coord_A_bevel_start < spline_start_bevel_start) {
            r_gon_parameter +=
                l_coord * cosf(outer_last_bevel_start_A_angle_bisector) * coord_A_bevel_start +
                0.5f * (1.0f - l_coord * cosf(outer_last_bevel_start_A_angle_bisector)) *
                    squaref(coord_A_bevel_start) / spline_start_bevel_start;
          }
          else {
            r_gon_parameter +=
                spline_start_bevel_start *
                    (0.5f * l_coord * cosf(outer_last_bevel_start_A_angle_bisector) - 0.5f) +
                coord_A_bevel_start;
          }
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            float normalize_based_on_l_angle_bisector =
                l_angle_bisector * tanf(outer_last_bevel_start_A_angle_bisector) +
                spline_start_bevel_start * (0.5f * l_angle_bisector + 0.5f) +
                effective_roundness * x_axis_A_outer_last_bevel_start;
            float normalize_based_on_l_coord =
                l_coord * sinf(outer_last_bevel_start_A_angle_bisector) +
                spline_start_bevel_start *
                    (0.5f * l_coord * cosf(outer_last_bevel_start_A_angle_bisector) + 0.5f) +
                effective_roundness * x_axis_A_outer_last_bevel_start;

            /* For effective_roundness -> 1.0 the normalize_based_on_l_angle_bisector field and
             * normalize_based_on_l_coord field converge against the same scalar field. */
            r_gon_parameter /= mix(normalize_based_on_l_angle_bisector,
                                   normalize_based_on_l_coord,
                                   coord_A_bevel_start / x_axis_A_outer_last_bevel_start);
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter = tanf(outer_last_bevel_start_A_angle_bisector) +
                               spline_start_bevel_start +
                               effective_roundness * x_axis_A_outer_last_bevel_start;
        }
        x_axis_A_angle_bisector = segment_id * ref_A_next_ref + ref_A_angle_bisector;
      }
      return make_float4(
          l_angle_bisector, r_gon_parameter, max_unit_parameter, x_axis_A_angle_bisector);
    }
  }
}

ccl_device float4 calculate_out_fields(bool calculate_r_gon_parameter_field,
                                       bool calculate_max_unit_parameter,
                                       bool normalize_r_gon_parameter,
                                       bool elliptical_corners,
                                       float r_gon_sides,
                                       float r_gon_roundness,
                                       float2 coord)
{
  float l_coord = sqrtf(squaref(coord.x) + squaref(coord.y));

  if (fractf(r_gon_sides) == 0.0f) {
    float x_axis_A_coord = atan2f(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
    float ref_A_angle_bisector = M_PI_F / r_gon_sides;
    float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
    float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
    float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

    if (r_gon_roundness == 0.0f) {
      float l_angle_bisector = 0.0f;
      float r_gon_parameter = 0.0f;
      float max_unit_parameter = 0.0f;

      l_angle_bisector = l_coord * cosf(ref_A_angle_bisector - ref_A_coord);

      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = l_angle_bisector * tanf(fabsf(ref_A_angle_bisector - ref_A_coord));
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0f;
        }
        if (normalize_r_gon_parameter && (r_gon_sides != 2.0f)) {
          r_gon_parameter /= l_angle_bisector * tanf(ref_A_angle_bisector);
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter = (r_gon_sides != 2.0f) ? tanf(ref_A_angle_bisector) : 0.0f;
      }
      return make_float4(l_angle_bisector,
                         r_gon_parameter,
                         max_unit_parameter,
                         segment_id * ref_A_next_ref + ref_A_angle_bisector);
    }
    else if (r_gon_roundness == 1.0f) {
      float r_gon_parameter = 0.0f;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = fabsf(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter /= ref_A_angle_bisector;
        }
      }
      return make_float4(l_coord,
                         r_gon_parameter,
                         ref_A_angle_bisector,
                         segment_id * ref_A_next_ref + ref_A_angle_bisector);
    }
    else {
      float ref_A_bevel_start = ref_A_angle_bisector -
                                atanf((1.0f - r_gon_roundness) * tanf(ref_A_angle_bisector));
      float bevel_start_A_angle_bisector = ref_A_angle_bisector - ref_A_bevel_start;

      if ((ref_A_coord >= ref_A_next_ref - ref_A_bevel_start) || (ref_A_coord < ref_A_bevel_start))
      {
        /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
         * angles are negative.*/
        float nearest_ref_SA_coord = ref_A_coord -
                                     float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
        float l_angle_bisector = 0.0f;
        float r_gon_parameter = 0.0f;
        float max_unit_parameter = 0.0f;

        float l_circle_radius = sinf(ref_A_bevel_start) / sinf(ref_A_angle_bisector);
        float l_circle_center = sinf(bevel_start_A_angle_bisector) / sinf(ref_A_angle_bisector);
        float l_coord_R_l_bevel_start = cosf(nearest_ref_SA_coord) * l_circle_center +
                                        sqrtf(
                                            squaref(cosf(nearest_ref_SA_coord) * l_circle_center) +
                                            squaref(l_circle_radius) - squaref(l_circle_center));

        l_angle_bisector = l_coord * cosf(bevel_start_A_angle_bisector) / l_coord_R_l_bevel_start;

        float spline_start_bevel_start = (1.0f - r_gon_roundness) * ref_A_bevel_start;

        if (calculate_r_gon_parameter_field) {
          float coord_A_bevel_start = ref_A_bevel_start - fabsf(nearest_ref_SA_coord);
          r_gon_parameter = l_coord * sinf(bevel_start_A_angle_bisector);

          if (coord_A_bevel_start < spline_start_bevel_start) {
            r_gon_parameter += l_coord * cosf(bevel_start_A_angle_bisector) * coord_A_bevel_start +
                               0.5f * (1.0f - l_coord * cosf(bevel_start_A_angle_bisector)) *
                                   squaref(coord_A_bevel_start) / spline_start_bevel_start;
          }
          else {
            r_gon_parameter += spline_start_bevel_start *
                                   (0.5f * l_coord * cosf(bevel_start_A_angle_bisector) - 0.5f) +
                               coord_A_bevel_start;
          }
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            float normalize_based_on_l_angle_bisector = l_angle_bisector *
                                                            tanf(bevel_start_A_angle_bisector) +
                                                        spline_start_bevel_start *
                                                            (0.5f * l_angle_bisector + 0.5f) +
                                                        r_gon_roundness * ref_A_bevel_start;
            float normalize_based_on_l_coord =
                l_coord * sinf(bevel_start_A_angle_bisector) +
                spline_start_bevel_start *
                    (0.5f * l_coord * cosf(bevel_start_A_angle_bisector) + 0.5f) +
                r_gon_roundness * ref_A_bevel_start;

            /* For r_gon_roundness -> 1.0 the normalize_based_on_l_angle_bisector field and
             * normalize_based_on_l_coord field converge against the same scalar field. */
            r_gon_parameter /= mix(normalize_based_on_l_angle_bisector,
                                   normalize_based_on_l_coord,
                                   coord_A_bevel_start / ref_A_bevel_start);
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter = tanf(bevel_start_A_angle_bisector) + spline_start_bevel_start +
                               r_gon_roundness * ref_A_bevel_start;
        }
        return make_float4(l_angle_bisector,
                           r_gon_parameter,
                           max_unit_parameter,
                           segment_id * ref_A_next_ref + ref_A_angle_bisector);
      }
      else {
        float l_angle_bisector = 0.0f;
        float r_gon_parameter = 0.0f;
        float max_unit_parameter = 0.0f;

        l_angle_bisector = l_coord * cosf(ref_A_angle_bisector - ref_A_coord);

        float spline_start_bevel_start = (1.0f - r_gon_roundness) * ref_A_bevel_start;

        if (calculate_r_gon_parameter_field) {
          r_gon_parameter = l_angle_bisector * tanf(fabsf(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            float normalize_based_on_l_angle_bisector = l_angle_bisector *
                                                            tanf(bevel_start_A_angle_bisector) +
                                                        spline_start_bevel_start *
                                                            (0.5f * l_angle_bisector + 0.5f) +
                                                        r_gon_roundness * ref_A_bevel_start;

            r_gon_parameter /= normalize_based_on_l_angle_bisector;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter = tanf(bevel_start_A_angle_bisector) + spline_start_bevel_start +
                               r_gon_roundness * ref_A_bevel_start;
        }
        return make_float4(l_angle_bisector,
                           r_gon_parameter,
                           max_unit_parameter,
                           segment_id * ref_A_next_ref + ref_A_angle_bisector);
      }
    }
  }
  else {
    if (r_gon_roundness == 0.0f) {
      float x_axis_A_coord = atan2f(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
      float ref_A_angle_bisector = M_PI_F / r_gon_sides;
      float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
      float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
      float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

      float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
      float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;

      if (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis) {
        float l_angle_bisector = 0.0f;
        float r_gon_parameter = 0.0f;
        float max_unit_parameter = 0.0f;

        l_angle_bisector = l_coord * cosf(ref_A_angle_bisector - ref_A_coord);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter = l_angle_bisector * tanf(fabsf(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter /= l_angle_bisector * tanf(ref_A_angle_bisector);
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter = tanf(ref_A_angle_bisector);
        }
        return make_float4(l_angle_bisector,
                           r_gon_parameter,
                           max_unit_parameter,
                           segment_id * ref_A_next_ref + ref_A_angle_bisector);
      }
      else {
        float l_angle_bisector = 0.0f;
        float r_gon_parameter = 0.0f;
        float max_unit_parameter = 0.0f;

        float l_angle_bisector_R_l_last_angle_bisector = cosf(ref_A_angle_bisector) /
                                                         cosf(last_angle_bisector_A_x_axis);
        float l_last_angle_bisector = l_coord * cosf(last_angle_bisector_A_x_axis - ref_A_coord);

        l_angle_bisector = l_angle_bisector_R_l_last_angle_bisector * l_last_angle_bisector;

        if (calculate_r_gon_parameter_field) {
          r_gon_parameter = l_angle_bisector_R_l_last_angle_bisector * l_last_angle_bisector *
                            tanf(fabsf(last_angle_bisector_A_x_axis - ref_A_coord));
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter /= l_angle_bisector_R_l_last_angle_bisector * l_last_angle_bisector *
                               tanf(last_angle_bisector_A_x_axis);
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter = tanf(last_angle_bisector_A_x_axis);
        }
        return make_float4(l_angle_bisector,
                           r_gon_parameter,
                           max_unit_parameter,
                           segment_id * ref_A_next_ref + last_angle_bisector_A_x_axis);
      }
    }
    else if (r_gon_roundness == 1.0f) {
      if (elliptical_corners) {
        return calculate_out_fields_full_roundness_irregular_elliptical(

            calculate_r_gon_parameter_field,
            normalize_r_gon_parameter,
            r_gon_sides,
            coord,
            l_coord);
      }
      else {
        return calculate_out_fields_full_roundness_irregular_circular(

            calculate_r_gon_parameter_field,
            calculate_max_unit_parameter,
            normalize_r_gon_parameter,
            r_gon_sides,
            coord,
            l_coord);
      }
    }
    else {
      if (elliptical_corners) {
        return calculate_out_fields_irregular_elliptical(calculate_r_gon_parameter_field,
                                                         calculate_max_unit_parameter,
                                                         normalize_r_gon_parameter,
                                                         r_gon_sides,
                                                         r_gon_roundness,
                                                         coord,
                                                         l_coord);
      }
      else {
        return calculate_out_fields_irregular_circular(calculate_r_gon_parameter_field,
                                                       calculate_max_unit_parameter,
                                                       normalize_r_gon_parameter,
                                                       r_gon_sides,
                                                       r_gon_roundness,
                                                       coord,
                                                       l_coord);
      }
    }
  }
}

template<uint node_feature_mask>
ccl_device_noinline int svm_node_tex_rounded_polygon(
    KernelGlobals kg, ccl_private ShaderData *sd, ccl_private float *stack, uint4 node, int offset)
{
  RoundedPolygonStackOffsets so;

  uint normalize_r_gon_parameter, elliptical_corners;

  svm_unpack_node_uchar4(
      node.y, &(normalize_r_gon_parameter), &(elliptical_corners), &(so.vector), &(so.scale));
  svm_unpack_node_uchar4(node.z,
                         &(so.r_gon_sides),
                         &(so.r_gon_roundness),
                         &(so.r_gon_field),
                         &(so.segment_coordinates));
  svm_unpack_node_uchar2(node.w, &(so.max_unit_parameter), &(so.x_axis_A_angle_bisector));

  bool calculate_r_gon_parameter_field = stack_valid(so.segment_coordinates);
  bool calculate_max_unit_parameter = stack_valid(so.max_unit_parameter);

  float3 coord = stack_load_float3(stack, so.vector);
  uint4 defaults = read_node(kg, &offset);
  float scale = stack_load_float_default(stack, so.scale, defaults.x);
  float r_gon_sides = stack_load_float_default(stack, so.r_gon_sides, defaults.y);
  float r_gon_roundness = stack_load_float_default(stack, so.r_gon_roundness, defaults.z);

  float4 out_variables = calculate_out_fields(calculate_r_gon_parameter_field,
                                              calculate_max_unit_parameter,
                                              normalize_r_gon_parameter,
                                              elliptical_corners,
                                              float_max(r_gon_sides, 2.0f),
                                              clamp(r_gon_roundness, 0.0f, 1.0f),
                                              scale * make_float2(coord.x, coord.y));

  if (stack_valid(so.r_gon_field)) {
    stack_store_float(stack, so.r_gon_field, out_variables.x);
  }
  if (stack_valid(so.segment_coordinates)) {
    stack_store_float3(
        stack, so.segment_coordinates, make_float3(out_variables.y, out_variables.x - 1.0f, 0.0));
  }
  if (stack_valid(so.max_unit_parameter)) {
    stack_store_float(stack, so.max_unit_parameter, out_variables.z);
  }
  if (stack_valid(so.x_axis_A_angle_bisector)) {
    stack_store_float(stack, so.x_axis_A_angle_bisector, out_variables.w);
  }

  return offset;
}

CCL_NAMESPACE_END
