/* SPDX-FileCopyrightText: 2024 Tenkai Raiko
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

CCL_NAMESPACE_BEGIN

struct RaikoBaseStackOffsets {
  uint vector;
  uint w;
  uint scale;
  uint r_gon_sides;
  uint r_gon_roundness;
  uint r_gon_exponent;
  uint sphere_exponent;
  uint r_sphere_field;
  uint r_gon_parameter_field;
  uint max_unit_parameter;
};

ccl_device float chebychev_norm(float coord)
{
  return fabsf(coord);
}

ccl_device float chebychev_norm(float2 coord)
{
  return float_max(fabsf(coord.x), fabsf(coord.y));
}

ccl_device float chebychev_norm(float3 coord)
{
  return float_max(fabsf(coord.x), float_max(fabsf(coord.y), fabsf(coord.z)));
}

ccl_device float p_norm(float coord)
{
  return fabsf(coord);
}

ccl_device float p_norm(float2 coord, float exponent)
{
  /* Use Chebychev norm instead of p-norm for high exponent values to avoid going out of the
   * floating point representable range. */
  return (exponent > 32.0f) ? chebychev_norm(coord) :
                              powf(powf(fabsf(coord.x), exponent) + powf(fabsf(coord.y), exponent),
                                   1.0f / exponent);
}

ccl_device float p_norm(float3 coord, float exponent)
{
  /* Use Chebychev norm instead of p-norm for high exponent values to avoid going out of the
   * floating point representable range. */
  return (exponent > 32.0f) ?
             chebychev_norm(coord) :
             powf(powf(fabsf(coord.x), exponent) + powf(fabsf(coord.y), exponent) +
                      powf(fabsf(coord.z), exponent),
                  1.0f / exponent);
}

ccl_device float3
calculate_out_fields_2d_full_roundness_irregular_elliptical(bool calculate_r_gon_parameter_field,
                                                            bool normalize_r_gon_parameter,
                                                            float r_gon_sides,
                                                            float2 coord,
                                                            float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;

  if ((x_axis_A_coord >= ref_A_angle_bisector) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector))
  {
    float r_gon_parameter_2d = 0.0f;
    if (calculate_r_gon_parameter_field) {
      r_gon_parameter_2d = fabsf(ref_A_angle_bisector - ref_A_coord);
      if (ref_A_coord < ref_A_angle_bisector) {
        r_gon_parameter_2d *= -1.0f;
      }
      if (normalize_r_gon_parameter) {
        r_gon_parameter_2d /= ref_A_angle_bisector;
      }
    }
    return make_float3(l_projection_2d, r_gon_parameter_2d, ref_A_angle_bisector);
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) &&
        (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1;
    }
    float l_angle_bisector_2d = 0.0f;
    float r_gon_parameter_2d = 0.0f;
    float max_unit_parameter_2d = 0.0f;

    float l_basis_vector_1 = tan(ref_A_angle_bisector);
    /* When the fractional part of r_gon_sides is very small division by l_basis_vector_2 causes
     * precision issues. Change to double if necessary */
    float l_basis_vector_2 = sinf(last_angle_bisector_A_x_axis) *
                             sqrtf(square(tan(ref_A_angle_bisector)) + 1.0f);
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
    float l_coord_R_l_angle_bisector_2d =
        (-(transformed_direction_vector.x * transformed_origin.x +
           transformed_direction_vector.y * transformed_origin.y) +
         sqrtf(square(transformed_direction_vector.x * transformed_origin.x +
                      transformed_direction_vector.y * transformed_origin.y) -
               (square(transformed_direction_vector.x) + square(transformed_direction_vector.y)) *
                   (square(transformed_origin.x) + square(transformed_origin.y) - 1.0f))) /
        (square(transformed_direction_vector.x) + square(transformed_direction_vector.y));
    l_angle_bisector_2d = l_projection_2d / l_coord_R_l_angle_bisector_2d;
    if (nearest_ref_MSA_coord < 0.0f) {
      float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cosf(ref_A_angle_bisector) /
                                                             cosf(last_angle_bisector_A_x_axis);
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = (last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) *
                             l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter_2d *= -1.0f;
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
        r_gon_parameter_2d = fabsf(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= ref_A_angle_bisector;
        }
      }
      max_unit_parameter_2d = ref_A_angle_bisector;
    }
    return make_float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
  }
}

ccl_device float3
calculate_out_fields_2d_irregular_elliptical(bool calculate_r_gon_parameter_field,
                                             bool calculate_max_unit_parameter,
                                             bool normalize_r_gon_parameter,
                                             float r_gon_sides,
                                             float r_gon_roundness,
                                             float2 coord,
                                             float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1.0f - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1.0f - r_gon_roundness) *
                                               tan(last_angle_bisector_A_x_axis));

  if ((x_axis_A_coord >= ref_A_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start))
  {
    if ((ref_A_coord >= ref_A_bevel_start) && (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      l_angle_bisector_2d = l_projection_2d * cosf(ref_A_angle_bisector - ref_A_coord);
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d * tan(fabsf(ref_A_angle_bisector - ref_A_coord));
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
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
      return make_float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
    else {
      /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      float l_circle_radius = sinf(ref_A_bevel_start) / sinf(ref_A_angle_bisector);
      float l_circle_center = sinf(ref_A_angle_bisector - ref_A_bevel_start) /
                              sinf(ref_A_angle_bisector);
      float l_coord_R_l_bevel_start = cosf(nearest_ref_SA_coord) * l_circle_center +
                                      sqrtf(square(cosf(nearest_ref_SA_coord) * l_circle_center) +
                                            square(l_circle_radius) - square(l_circle_center));
      l_angle_bisector_2d = cosf(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
                            l_coord_R_l_bevel_start;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d * tan(ref_A_angle_bisector - ref_A_bevel_start) +
                             ref_A_bevel_start - fabsf(nearest_ref_SA_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
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
      return make_float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU_F - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cosf(ref_A_angle_bisector) /
                                                             cosf(last_angle_bisector_A_x_axis);
      l_angle_bisector_2d = l_projection_2d * cosf(last_angle_bisector_A_x_axis - ref_A_coord) *
                            l_angle_bisector_2d_R_l_last_angle_bisector_2d;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d *
                             tan(fabsf(last_angle_bisector_A_x_axis - ref_A_coord));
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter_2d *= -1.0f;
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
      return make_float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start) &&
          (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1;
      }
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      float l_basis_vector_1 = r_gon_roundness * tan(ref_A_angle_bisector);
      float l_basis_vector_2 = r_gon_roundness * sinf(last_angle_bisector_A_x_axis) *
                               sqrtf(square(tan(ref_A_angle_bisector)) + 1.0f);
      float2 ellipse_center =
          make_float2(cosf(ref_A_bevel_start) / cosf(ref_A_angle_bisector - ref_A_bevel_start),
                      sinf(ref_A_bevel_start) / cosf(ref_A_angle_bisector - ref_A_bevel_start)) -
          l_basis_vector_2 *
              make_float2(sinf(last_angle_bisector_A_x_axis), cosf(last_angle_bisector_A_x_axis));
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
      float l_coord_R_l_angle_bisector_2d =
          (-(transformed_direction_vector.x * transformed_origin.x +
             transformed_direction_vector.y * transformed_origin.y) +
           sqrtf(
               square(transformed_direction_vector.x * transformed_origin.x +
                      transformed_direction_vector.y * transformed_origin.y) -
               (square(transformed_direction_vector.x) + square(transformed_direction_vector.y)) *
                   (square(transformed_origin.x) + square(transformed_origin.y) - 1.0f))) /
          (square(transformed_direction_vector.x) + square(transformed_direction_vector.y));
      l_angle_bisector_2d = l_projection_2d / l_coord_R_l_angle_bisector_2d;
      if (nearest_ref_MSA_coord < 0.0f) {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cosf(ref_A_angle_bisector) /
                                                               cosf(last_angle_bisector_A_x_axis);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d * tan(fabsf(last_angle_bisector_A_x_axis -
                                                               inner_last_bevel_start_A_x_axis)) +
                               (inner_last_bevel_start_A_x_axis + nearest_ref_MSA_coord) *
                                   l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0f;
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
            r_gon_parameter_2d *= -1.0f;
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
      return make_float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
  }
}

ccl_device float3
calculate_out_fields_2d_full_roundness_irregular_circular(bool calculate_r_gon_parameter_field,
                                                          bool calculate_max_unit_parameter,
                                                          bool normalize_r_gon_parameter,
                                                          float r_gon_sides,
                                                          float2 coord,
                                                          float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float l_last_circle_radius = tan(last_angle_bisector_A_x_axis) /
                               tan(0.5f * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  float2 last_circle_center = make_float2(
      cosf(last_angle_bisector_A_x_axis) -
          l_last_circle_radius * cosf(last_angle_bisector_A_x_axis),
      l_last_circle_radius * sinf(last_angle_bisector_A_x_axis) -
          sinf(last_angle_bisector_A_x_axis));
  float2 outer_last_bevel_start = last_circle_center +
                                  l_last_circle_radius * make_float2(cosf(ref_A_angle_bisector),
                                                                     sinf(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                               outer_last_bevel_start.x);

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) ||
        (x_axis_A_coord < ref_A_angle_bisector))
    {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      l_angle_bisector_2d = l_projection_2d * cosf(ref_A_angle_bisector - ref_A_coord);
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d * tan(fabsf(ref_A_angle_bisector - ref_A_coord));
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
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
      return make_float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
    else {
      float r_gon_parameter_2d = 0.0f;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = fabsf(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= ref_A_angle_bisector;
        }
      }
      return make_float3(l_projection_2d, r_gon_parameter_2d, ref_A_angle_bisector);
    }
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
        (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1;
    }
    float l_angle_bisector_2d = 0.0f;
    float r_gon_parameter_2d = 0.0f;
    float max_unit_parameter_2d = 0.0f;

    float l_coord_R_l_last_angle_bisector_2d =
        sinf(nearest_ref_MSA_coord) * last_circle_center.y +
        cosf(nearest_ref_MSA_coord) * last_circle_center.x +
        sqrtf(square(sinf(nearest_ref_MSA_coord) * last_circle_center.y +
                     cosf(nearest_ref_MSA_coord) * last_circle_center.x) +
              square(l_last_circle_radius) - square(last_circle_center.x) -
              square(last_circle_center.y));
    l_angle_bisector_2d = (cosf(ref_A_angle_bisector) * l_projection_2d) /
                          (cosf(last_angle_bisector_A_x_axis) *
                           l_coord_R_l_last_angle_bisector_2d);
    if (nearest_ref_MSA_coord < 0.0f) {
      float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cosf(ref_A_angle_bisector) /
                                                             cosf(last_angle_bisector_A_x_axis);
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = (last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) *
                             l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter_2d *= -1.0f;
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
        r_gon_parameter_2d = l_angle_bisector_2d * tan(fabsf(ref_A_angle_bisector -
                                                             x_axis_A_outer_last_bevel_start)) +
                             x_axis_A_outer_last_bevel_start - nearest_ref_MSA_coord;
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
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
    return make_float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
  }
}

ccl_device float3 calculate_out_fields_2d_irregular_circular(bool calculate_r_gon_parameter_field,
                                                             bool calculate_max_unit_parameter,
                                                             bool normalize_r_gon_parameter,
                                                             float r_gon_sides,
                                                             float r_gon_roundness,
                                                             float2 coord,
                                                             float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1.0f - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1.0f - r_gon_roundness) *
                                               tan(last_angle_bisector_A_x_axis));
  float l_last_circle_radius = r_gon_roundness * tan(last_angle_bisector_A_x_axis) /
                               tan(0.5f * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
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
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                               outer_last_bevel_start.x);

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if (((ref_A_coord >= ref_A_bevel_start) &&
         (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) ||
        (x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start) ||
        (x_axis_A_coord < ref_A_bevel_start))
    {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      l_angle_bisector_2d = l_projection_2d * cosf(ref_A_angle_bisector - ref_A_coord);
      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) ||
          (x_axis_A_coord < ref_A_angle_bisector))
      {
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
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
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
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
      return make_float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
    else {
      /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      float l_circle_radius = sinf(ref_A_bevel_start) / sinf(ref_A_angle_bisector);
      float l_circle_center = sinf(ref_A_angle_bisector - ref_A_bevel_start) /
                              sinf(ref_A_angle_bisector);
      float l_coord_R_l_bevel_start = cosf(nearest_ref_SA_coord) * l_circle_center +
                                      sqrtf(square(cosf(nearest_ref_SA_coord) * l_circle_center) +
                                            square(l_circle_radius) - square(l_circle_center));
      l_angle_bisector_2d = cosf(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
                            l_coord_R_l_bevel_start;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d * tan(ref_A_angle_bisector - ref_A_bevel_start) +
                             ref_A_bevel_start - fabsf(nearest_ref_SA_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
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
      return make_float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU_F - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cosf(ref_A_angle_bisector) /
                                                             cosf(last_angle_bisector_A_x_axis);
      l_angle_bisector_2d = l_projection_2d * cosf(last_angle_bisector_A_x_axis - ref_A_coord) *
                            l_angle_bisector_2d_R_l_last_angle_bisector_2d;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d *
                             tan(fabsf(last_angle_bisector_A_x_axis - ref_A_coord));
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter_2d *= -1.0f;
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
      return make_float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
          (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1;
      }
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      float l_coord_R_l_last_angle_bisector_2d =
          sinf(nearest_ref_MSA_coord) * last_circle_center.y +
          cosf(nearest_ref_MSA_coord) * last_circle_center.x +
          sqrtf(square(sinf(nearest_ref_MSA_coord) * last_circle_center.y +
                       cosf(nearest_ref_MSA_coord) * last_circle_center.x) +
                square(l_last_circle_radius) - square(last_circle_center.x) -
                square(last_circle_center.y));
      l_angle_bisector_2d = (cosf(ref_A_angle_bisector) * l_projection_2d) /
                            (cosf(last_angle_bisector_A_x_axis) *
                             l_coord_R_l_last_angle_bisector_2d);
      if (nearest_ref_MSA_coord < 0.0f) {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cosf(ref_A_angle_bisector) /
                                                               cosf(last_angle_bisector_A_x_axis);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d * tan(fabsf(last_angle_bisector_A_x_axis -
                                                               inner_last_bevel_start_A_x_axis)) +
                               (inner_last_bevel_start_A_x_axis + nearest_ref_MSA_coord) *
                                   l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0f;
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
          r_gon_parameter_2d = l_angle_bisector_2d * tan(fabsf(ref_A_angle_bisector -
                                                               x_axis_A_outer_last_bevel_start)) +
                               x_axis_A_outer_last_bevel_start - nearest_ref_MSA_coord;
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
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
      return make_float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
  }
}

ccl_device float3 calculate_out_fields_2d(bool calculate_r_gon_parameter_field,
                                          bool calculate_max_unit_parameter,
                                          bool normalize_r_gon_parameter,
                                          bool elliptical_corners,
                                          float r_gon_sides,
                                          float r_gon_roundness,
                                          float r_gon_exponent,
                                          float2 coord)
{
  float l_projection_2d = p_norm(coord, r_gon_exponent);

  if (fractf(r_gon_sides) == 0.0f) {
    float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
    float ref_A_angle_bisector = M_PI_F / r_gon_sides;
    float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
    float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
    float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

    if (r_gon_roundness == 0.0f) {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;

      l_angle_bisector_2d = l_projection_2d * cosf(ref_A_angle_bisector - ref_A_coord);
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = l_angle_bisector_2d * tan(fabsf(ref_A_angle_bisector - ref_A_coord));
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter && (r_gon_sides != 2.0f)) {
          r_gon_parameter_2d /= l_angle_bisector_2d * tan(ref_A_angle_bisector);
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter_2d = (r_gon_sides != 2.0f) ? tan(ref_A_angle_bisector) : 0.0f;
      }
      return make_float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
    }
    if (r_gon_roundness == 1.0f) {
      float r_gon_parameter_2d = 0.0f;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = fabsf(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= ref_A_angle_bisector;
        }
      }
      return make_float3(l_projection_2d, r_gon_parameter_2d, ref_A_angle_bisector);
    }
    else {
      float ref_A_bevel_start = ref_A_angle_bisector -
                                atan((1.0f - r_gon_roundness) * tan(ref_A_angle_bisector));

      if ((ref_A_coord >= ref_A_next_ref - ref_A_bevel_start) || (ref_A_coord < ref_A_bevel_start))
      {
        /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
         * angles are negative.*/
        float nearest_ref_SA_coord = ref_A_coord -
                                     float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
        float l_angle_bisector_2d = 0.0f;
        float r_gon_parameter_2d = 0.0f;
        float max_unit_parameter_2d = 0.0f;

        float l_circle_radius = sinf(ref_A_bevel_start) / sinf(ref_A_angle_bisector);
        float l_circle_center = sinf(ref_A_angle_bisector - ref_A_bevel_start) /
                                sinf(ref_A_angle_bisector);
        float l_coord_R_l_bevel_start = cosf(nearest_ref_SA_coord) * l_circle_center +
                                        sqrtf(
                                            square(cosf(nearest_ref_SA_coord) * l_circle_center) +
                                            square(l_circle_radius) - square(l_circle_center));
        l_angle_bisector_2d = l_projection_2d * cosf(ref_A_angle_bisector - ref_A_bevel_start) /
                              l_coord_R_l_bevel_start;
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                                   tan(ref_A_angle_bisector - ref_A_bevel_start) +
                               ref_A_bevel_start - fabsf(nearest_ref_SA_coord);
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
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
        return make_float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
      }
      else {
        float l_angle_bisector_2d = 0.0f;
        float r_gon_parameter_2d = 0.0f;
        float max_unit_parameter_2d = 0.0f;

        l_angle_bisector_2d = l_projection_2d * cosf(ref_A_angle_bisector - ref_A_coord);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
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
        return make_float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
      }
    }
  }
  else {
    if (r_gon_roundness == 0.0f) {
      float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
      float ref_A_angle_bisector = M_PI_F / r_gon_sides;
      float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
      float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
      float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

      float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
      float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;

      if (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis) {
        float l_angle_bisector_2d = 0.0f;
        float r_gon_parameter_2d = 0.0f;
        float max_unit_parameter_2d = 0.0f;

        l_angle_bisector_2d = l_projection_2d * cosf(ref_A_angle_bisector - ref_A_coord);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d * tan(ref_A_angle_bisector);
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector);
        }
        return make_float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
      }
      else {
        float l_angle_bisector_2d = 0.0f;
        float r_gon_parameter_2d = 0.0f;
        float max_unit_parameter_2d = 0.0f;

        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cosf(ref_A_angle_bisector) /
                                                               cosf(last_angle_bisector_A_x_axis);
        l_angle_bisector_2d = l_projection_2d * cosf(last_angle_bisector_A_x_axis - ref_A_coord) *
                              l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(last_angle_bisector_A_x_axis - ref_A_coord));
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis);
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis);
        }
        return make_float3(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d);
      }
    }
    if (r_gon_roundness == 1.0f) {
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

ccl_device float3 calculate_out_fields_4d(bool calculate_r_gon_parameter_field,
                                          bool calculate_max_unit_parameter,
                                          bool normalize_r_gon_parameter,
                                          bool elliptical_corners,
                                          float r_gon_sides,
                                          float r_gon_roundness,
                                          float r_gon_exponent,
                                          float sphere_exponent,
                                          float4 coord)
{
  float3 out_fields = calculate_out_fields_2d(calculate_r_gon_parameter_field,
                                              calculate_max_unit_parameter,
                                              normalize_r_gon_parameter,
                                              elliptical_corners,
                                              r_gon_sides,
                                              r_gon_roundness,
                                              r_gon_exponent,
                                              make_float2(coord.x, coord.y));
  out_fields.x = p_norm(make_float3(out_fields.x, coord.z, coord.w), sphere_exponent);
  return out_fields;
}

template<uint node_feature_mask>
ccl_device_noinline int svm_node_tex_raiko_base(
    KernelGlobals kg, ccl_private ShaderData *sd, ccl_private float *stack, uint4 node, int offset)
{
  RaikoBaseStackOffsets so;

  uint normalize_r_gon_parameter, elliptical_corners;

  svm_unpack_node_uchar4(
      node.y, &(normalize_r_gon_parameter), &(elliptical_corners), &(so.vector), &(so.w));
  svm_unpack_node_uchar4(
      node.z, &(so.scale), &(so.r_gon_sides), &(so.r_gon_roundness), &(so.r_gon_exponent));
  svm_unpack_node_uchar4(node.w,
                         &(so.sphere_exponent),
                         &(so.r_sphere_field),
                         &(so.r_gon_parameter_field),
                         &(so.max_unit_parameter));

  bool calculate_r_gon_parameter_field = stack_valid(so.r_gon_parameter_field);
  bool calculate_max_unit_parameter = stack_valid(so.max_unit_parameter);

  uint4 defaults_1 = read_node(kg, &offset);
  float3 coord = stack_load_float3(stack, so.vector);
  float in_w = stack_load_float_default(stack, so.w, defaults_1.x);
  float scale = stack_load_float_default(stack, so.scale, defaults_1.y);
  float r_gon_sides = stack_load_float_default(stack, so.r_gon_sides, defaults_1.z);
  float r_gon_roundness = stack_load_float_default(stack, so.r_gon_roundness, defaults_1.w);

  uint4 defaults_2 = read_node(kg, &offset);
  float r_gon_exponent = stack_load_float_default(stack, so.r_gon_exponent, defaults_2.x);
  float sphere_exponent = stack_load_float_default(stack, so.sphere_exponent, defaults_2.y);

  float3 out_variables = calculate_out_fields_4d(calculate_r_gon_parameter_field,
                                                 calculate_max_unit_parameter,
                                                 normalize_r_gon_parameter,
                                                 elliptical_corners,
                                                 float_max(r_gon_sides, 2.0f),
                                                 clamp(r_gon_roundness, 0.0f, 1.0f),
                                                 float_max(r_gon_exponent, 0.0),
                                                 float_max(sphere_exponent, 0.0),
                                                 scale *
                                                     make_float4(coord.x, coord.y, coord.z, in_w));

  if (stack_valid(so.r_sphere_field)) {
    stack_store_float(stack, so.r_sphere_field, out_variables.x);
  }
  if (stack_valid(so.r_gon_parameter_field)) {
    stack_store_float(stack, so.r_gon_parameter_field, out_variables.y);
  }
  if (stack_valid(so.max_unit_parameter)) {
    stack_store_float(stack, so.max_unit_parameter, out_variables.z);
  }

  return offset;
}

CCL_NAMESPACE_END
