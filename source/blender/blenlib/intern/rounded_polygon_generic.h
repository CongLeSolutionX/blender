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

RETURN_ARGUMENTS calculate_out_fields_full_roundness_irregular_elliptical(
    bool calculate_r_gon_parameter_field,
    bool normalize_r_gon_parameter,
    float r_gon_sides,
    float2 coord,
    float l_coord)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
  float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI_F - floor(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;

  if ((x_axis_A_coord >= ref_A_angle_bisector) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector))
  {
    /* Regular rounded part. */

    float r_gon_parameter = 0.0;
    if (calculate_r_gon_parameter_field) {
      r_gon_parameter = abs(ref_A_angle_bisector - ref_A_coord);
      if (ref_A_coord < ref_A_angle_bisector) {
        r_gon_parameter *= -1.0;
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
    /* Irregular rounded part. */

    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) &&
        (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1.0;
    }
    float l_angle_bisector = 0.0;
    float r_gon_parameter = 0.0;
    float max_unit_parameter = 0.0;
    float x_axis_A_angle_bisector = 0.0;

    float l_basis_vector_1 = tan(ref_A_angle_bisector);
    /* When the fractional part of r_gon_sides is very small division by l_basis_vector_2 causes
     * precision issues. Change to double if necessary */
    float l_basis_vector_2 = sin(last_angle_bisector_A_x_axis) *
                             sqrt(square(tan(ref_A_angle_bisector)) + 1.0);
    float2 ellipse_center = make_float2(cos(ref_A_angle_bisector) /
                                       cos(ref_A_angle_bisector - ref_A_angle_bisector),
                                   sin(ref_A_angle_bisector) /
                                       cos(ref_A_angle_bisector - ref_A_angle_bisector)) -
                            l_basis_vector_2 * make_float2(sin(last_angle_bisector_A_x_axis),
                                                      cos(last_angle_bisector_A_x_axis));
    float2 transformed_direction_vector = make_float2(
        cos(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
            (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
        cos(ref_A_angle_bisector - nearest_ref_MSA_coord) /
            (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
    float2 transformed_origin = make_float2(
        (ellipse_center.y * sin(last_angle_bisector_A_x_axis) -
         ellipse_center.x * cos(last_angle_bisector_A_x_axis)) /
            (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
        -(ellipse_center.y * sin(ref_A_angle_bisector) +
          ellipse_center.x * cos(ref_A_angle_bisector)) /
            (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
    float l_coord_R_l_angle_bisector =
        (-(transformed_direction_vector.x * transformed_origin.x +
           transformed_direction_vector.y * transformed_origin.y) +
         sqrt(square(transformed_direction_vector.x * transformed_origin.x +
                                 transformed_direction_vector.y * transformed_origin.y) -
                    (square(transformed_direction_vector.x) +
                     square(transformed_direction_vector.y)) *
                        (square(transformed_origin.x) + square(transformed_origin.y) -
                         1.0))) /
        (square(transformed_direction_vector.x) +
         square(transformed_direction_vector.y));

    l_angle_bisector = l_coord / l_coord_R_l_angle_bisector;

    if (nearest_ref_MSA_coord < 0.0) {
      /* Irregular rounded inner part. */

      float l_angle_bisector_R_l_last_angle_bisector = cos(ref_A_angle_bisector) /
                                                       cos(last_angle_bisector_A_x_axis);
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = l_angle_bisector_R_l_last_angle_bisector *
                          (last_angle_bisector_A_x_axis + nearest_ref_MSA_coord);
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter *= -1.0;
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
      /* Irregular rounded outer part. */

      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = abs(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter /= ref_A_angle_bisector;
        }
      }
      max_unit_parameter = ref_A_angle_bisector;
      x_axis_A_angle_bisector = segment_id * ref_A_next_ref + ref_A_angle_bisector;
    }
    return make_float4(l_angle_bisector, r_gon_parameter, max_unit_parameter, x_axis_A_angle_bisector);
  }
}

RETURN_ARGUMENTS calculate_out_fields_irregular_elliptical(bool calculate_r_gon_parameter_field,
                                                 bool calculate_max_unit_parameter,
                                                 bool normalize_r_gon_parameter,
                                                 float r_gon_sides,
                                                 float r_gon_roundness,
                                                 float2 coord,
                                                 float l_coord)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
  float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1.0 - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI_F - floor(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1.0 - r_gon_roundness) *
                                                     tan(last_angle_bisector_A_x_axis));
  float inner_last_bevel_start_A_last_angle_bisector = last_angle_bisector_A_x_axis -
                                                       inner_last_bevel_start_A_x_axis;

  if ((x_axis_A_coord >= ref_A_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start))
  {
    float bevel_start_A_angle_bisector = ref_A_angle_bisector - ref_A_bevel_start;

    if ((ref_A_coord >= ref_A_bevel_start) && (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) {
      /* Regular straight part. */

      float l_angle_bisector = 0.0;
      float r_gon_parameter = 0.0;
      float max_unit_parameter = 0.0;

      l_angle_bisector = l_coord * cos(ref_A_angle_bisector - ref_A_coord);

      float spline_start_bevel_start = (1.0 - r_gon_roundness) * ref_A_bevel_start;

      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = l_angle_bisector *
                          tan(abs(ref_A_angle_bisector - ref_A_coord));
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          float normalize_based_on_l_angle_bisector = l_angle_bisector *
                                                          tan(bevel_start_A_angle_bisector) +
                                                      spline_start_bevel_start *
                                                          (0.5 * l_angle_bisector + 0.5) +
                                                      r_gon_roundness * ref_A_bevel_start;

          r_gon_parameter /= normalize_based_on_l_angle_bisector;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter = tan(bevel_start_A_angle_bisector) + spline_start_bevel_start +
                             r_gon_roundness * ref_A_bevel_start;
      }
      return make_float4(l_angle_bisector,
                    r_gon_parameter,
                    max_unit_parameter,
                    segment_id * ref_A_next_ref + ref_A_angle_bisector);
    }
    else {
      /* Regular rounded part. */

      /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_angle_bisector = 0.0;
      float r_gon_parameter = 0.0;
      float max_unit_parameter = 0.0;

      float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
      float l_circle_center = sin(bevel_start_A_angle_bisector) /
                              sin(ref_A_angle_bisector);
      float l_coord_R_l_bevel_start =
          cos(nearest_ref_SA_coord) * l_circle_center +
          sqrt(square(cos(nearest_ref_SA_coord) * l_circle_center) +
                     square(l_circle_radius) - square(l_circle_center));

      l_angle_bisector = l_coord * cos(bevel_start_A_angle_bisector) /
                         l_coord_R_l_bevel_start;

      float spline_start_bevel_start = (1.0 - r_gon_roundness) * ref_A_bevel_start;

      if (calculate_r_gon_parameter_field) {
        float coord_A_bevel_start = ref_A_bevel_start - abs(nearest_ref_SA_coord);
        r_gon_parameter = l_coord * sin(bevel_start_A_angle_bisector);

        if (coord_A_bevel_start < spline_start_bevel_start) {
          r_gon_parameter += l_coord * cos(bevel_start_A_angle_bisector) *
                                 coord_A_bevel_start +
                             0.5 * (1.0 - l_coord * cos(bevel_start_A_angle_bisector)) *
                                 square(coord_A_bevel_start) / spline_start_bevel_start;
        }
        else {
          r_gon_parameter += spline_start_bevel_start *
                                 (0.5 * l_coord * cos(bevel_start_A_angle_bisector) -
                                  0.5) +
                             coord_A_bevel_start;
        }
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          float normalize_based_on_l_angle_bisector = l_angle_bisector *
                                                          tan(bevel_start_A_angle_bisector) +
                                                      spline_start_bevel_start *
                                                          (0.5 * l_angle_bisector + 0.5) +
                                                      r_gon_roundness * ref_A_bevel_start;
          float normalize_based_on_l_coord =
              l_coord * sin(bevel_start_A_angle_bisector) +
              spline_start_bevel_start *
                  (0.5 * l_coord * cos(bevel_start_A_angle_bisector) + 0.5) +
              r_gon_roundness * ref_A_bevel_start;

          /* For r_gon_roundness -> 1.0 the normalize_based_on_l_angle_bisector field and
           * normalize_based_on_l_coord field converge against the same scalar field. */
          r_gon_parameter /= mix(normalize_based_on_l_angle_bisector,
                                       normalize_based_on_l_coord,
                                       coord_A_bevel_start / ref_A_bevel_start);
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter = tan(bevel_start_A_angle_bisector) + spline_start_bevel_start +
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
      /* Irregular straight part. */

      float l_angle_bisector = 0.0;
      float r_gon_parameter = 0.0;
      float max_unit_parameter = 0.0;

      float l_angle_bisector_R_l_last_angle_bisector = cos(ref_A_angle_bisector) /
                                                       cos(last_angle_bisector_A_x_axis);
      float l_last_angle_bisector = l_coord *
                                    cos(last_angle_bisector_A_x_axis - ref_A_coord);

      l_angle_bisector = l_angle_bisector_R_l_last_angle_bisector * l_last_angle_bisector;

      float spline_start_bevel_start = (1.0 - r_gon_roundness) * inner_last_bevel_start_A_x_axis;

      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = l_angle_bisector_R_l_last_angle_bisector * l_last_angle_bisector *
                          tan(abs(last_angle_bisector_A_x_axis - ref_A_coord));
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          float normalize_based_on_l_l_angle_bisector =
              (l_last_angle_bisector * tan(inner_last_bevel_start_A_last_angle_bisector) +
               spline_start_bevel_start * (0.5 * l_last_angle_bisector + 0.5) +
               r_gon_roundness * inner_last_bevel_start_A_x_axis);

          r_gon_parameter /= l_angle_bisector_R_l_last_angle_bisector *
                             normalize_based_on_l_l_angle_bisector;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter = tan(inner_last_bevel_start_A_last_angle_bisector) +
                             l_angle_bisector_R_l_last_angle_bisector *
                                 (spline_start_bevel_start *
                                      ((0.5 / l_angle_bisector_R_l_last_angle_bisector) + 0.5) +
                                  r_gon_roundness * inner_last_bevel_start_A_x_axis);
      }
      return make_float4(l_angle_bisector,
                    r_gon_parameter,
                    max_unit_parameter,
                    segment_id * ref_A_next_ref + last_angle_bisector_A_x_axis);
    }
    else {
      /* Irregular rounded part. */

      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start) &&
          (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1.0;
      }
      float bevel_start_A_angle_bisector = ref_A_angle_bisector - ref_A_bevel_start;

      float l_angle_bisector = 0.0;
      float r_gon_parameter = 0.0;
      float max_unit_parameter = 0.0;
      float x_axis_A_angle_bisector = 0.0;

      float l_basis_vector_1 = r_gon_roundness * tan(ref_A_angle_bisector);
      float l_basis_vector_2 = r_gon_roundness * sin(last_angle_bisector_A_x_axis) *
                               sqrt(square(tan(ref_A_angle_bisector)) + 1.0);
      float2 ellipse_center =
          make_float2(cos(ref_A_bevel_start) / cos(bevel_start_A_angle_bisector),
                 sin(ref_A_bevel_start) / cos(bevel_start_A_angle_bisector)) -
          l_basis_vector_2 * make_float2(sin(last_angle_bisector_A_x_axis),
                                    cos(last_angle_bisector_A_x_axis));
      float2 transformed_direction_vector = make_float2(
          cos(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
              (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          cos(ref_A_angle_bisector - nearest_ref_MSA_coord) /
              (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      float2 transformed_origin = make_float2(
          (ellipse_center.y * sin(last_angle_bisector_A_x_axis) -
           ellipse_center.x * cos(last_angle_bisector_A_x_axis)) /
              (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          -(ellipse_center.y * sin(ref_A_angle_bisector) +
            ellipse_center.x * cos(ref_A_angle_bisector)) /
              (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      float l_coord_R_l_angle_bisector =
          (-(transformed_direction_vector.x * transformed_origin.x +
             transformed_direction_vector.y * transformed_origin.y) +
           sqrt(square(transformed_direction_vector.x * transformed_origin.x +
                                   transformed_direction_vector.y * transformed_origin.y) -
                      (square(transformed_direction_vector.x) +
                       square(transformed_direction_vector.y)) *
                          (square(transformed_origin.x) +
                           square(transformed_origin.y) - 1.0))) /
          (square(transformed_direction_vector.x) +
           square(transformed_direction_vector.y));

      l_angle_bisector = l_coord / l_coord_R_l_angle_bisector;

      if (nearest_ref_MSA_coord < 0.0) {
        /* Irregular rounded inner part. */

        float l_angle_bisector_R_l_last_angle_bisector = cos(ref_A_angle_bisector) /
                                                         cos(last_angle_bisector_A_x_axis);
        float l_last_angle_bisector = l_angle_bisector / l_angle_bisector_R_l_last_angle_bisector;

        float spline_start_bevel_start = (1.0 - r_gon_roundness) *
                                         inner_last_bevel_start_A_x_axis;

        if (calculate_r_gon_parameter_field) {
          float coord_A_bevel_start = inner_last_bevel_start_A_x_axis -
                                      abs(nearest_ref_MSA_coord);
          r_gon_parameter = l_angle_bisector_R_l_last_angle_bisector * l_coord *
                            sin(inner_last_bevel_start_A_last_angle_bisector);

          if (coord_A_bevel_start < spline_start_bevel_start) {
            r_gon_parameter +=
                l_angle_bisector_R_l_last_angle_bisector *
                (l_coord * cos(inner_last_bevel_start_A_last_angle_bisector) *
                     coord_A_bevel_start +
                 0.5 *
                     (1.0 - l_coord * cos(inner_last_bevel_start_A_last_angle_bisector)) *
                     square(coord_A_bevel_start) / spline_start_bevel_start);
          }
          else {
            r_gon_parameter += l_angle_bisector_R_l_last_angle_bisector *
                               (spline_start_bevel_start *
                                    (0.5 * l_coord *
                                         cos(inner_last_bevel_start_A_last_angle_bisector) -
                                     0.5) +
                                coord_A_bevel_start);
          }
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            float normalize_based_on_l_l_angle_bisector =
                l_last_angle_bisector * tan(inner_last_bevel_start_A_last_angle_bisector) +
                spline_start_bevel_start * (0.5 * l_last_angle_bisector + 0.5) +
                r_gon_roundness * inner_last_bevel_start_A_x_axis;
            float normalize_based_on_l_coord =
                l_coord * sin(inner_last_bevel_start_A_last_angle_bisector) +
                spline_start_bevel_start *
                    (0.5 * l_coord * cos(inner_last_bevel_start_A_last_angle_bisector) +
                     0.5) +
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
          max_unit_parameter = tan(inner_last_bevel_start_A_last_angle_bisector) +
                               l_angle_bisector_R_l_last_angle_bisector *
                                   (spline_start_bevel_start *
                                        ((0.5 / l_angle_bisector_R_l_last_angle_bisector) +
                                         0.5) +
                                    r_gon_roundness * inner_last_bevel_start_A_x_axis);
        }
        x_axis_A_angle_bisector = segment_id * ref_A_next_ref + last_angle_bisector_A_x_axis;
      }
      else {
        /* Irregular rounded outer part. */

        float spline_start_bevel_start = (1.0 - r_gon_roundness) * ref_A_bevel_start;

        if (calculate_r_gon_parameter_field) {
          float coord_A_bevel_start = ref_A_bevel_start - abs(nearest_ref_MSA_coord);
          r_gon_parameter = l_coord * sin(bevel_start_A_angle_bisector);

          if (coord_A_bevel_start < spline_start_bevel_start) {
            r_gon_parameter += l_coord * cos(bevel_start_A_angle_bisector) *
                                   coord_A_bevel_start +
                               0.5 * (1.0 - l_coord * cos(bevel_start_A_angle_bisector)) *
                                   square(coord_A_bevel_start) / spline_start_bevel_start;
          }
          else {
            r_gon_parameter += spline_start_bevel_start *
                                   (0.5 * l_coord * cos(bevel_start_A_angle_bisector) -
                                    0.5) +
                               coord_A_bevel_start;
          }
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            float normalize_based_on_l_angle_bisector =
                l_angle_bisector * tan(bevel_start_A_angle_bisector) +
                spline_start_bevel_start * (0.5 * l_angle_bisector + 0.5) +
                r_gon_roundness * ref_A_bevel_start;
            float normalize_based_on_l_coord =
                l_coord * sin(bevel_start_A_angle_bisector) +
                spline_start_bevel_start *
                    (0.5 * l_coord * cos(bevel_start_A_angle_bisector) + 0.5) +
                r_gon_roundness * ref_A_bevel_start;

            /* For r_gon_roundness -> 1.0 the normalize_based_on_l_angle_bisector field and
             * normalize_based_on_l_coord field converge against the same scalar field. */
            r_gon_parameter /= mix(normalize_based_on_l_angle_bisector,
                                         normalize_based_on_l_coord,
                                         coord_A_bevel_start / ref_A_bevel_start);
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter = tan(bevel_start_A_angle_bisector) + spline_start_bevel_start +
                               r_gon_roundness * ref_A_bevel_start;
        }
        x_axis_A_angle_bisector = segment_id * ref_A_next_ref + ref_A_angle_bisector;
      }
      return make_float4(
          l_angle_bisector, r_gon_parameter, max_unit_parameter, x_axis_A_angle_bisector);
    }
  }
}

RETURN_ARGUMENTS calculate_out_fields_full_roundness_irregular_circular(bool calculate_r_gon_parameter_field,
                                                              bool normalize_r_gon_parameter,
                                                              float r_gon_sides,
                                                              float2 coord,
                                                              float l_coord)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
  float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI_F - floor(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;
  float l_last_circle_radius = tan(last_angle_bisector_A_x_axis) /
                               tan(0.5 *
                                         (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  float2 last_circle_center = make_float2(
      cos(last_angle_bisector_A_x_axis) -
          l_last_circle_radius * cos(last_angle_bisector_A_x_axis),
      l_last_circle_radius * sin(last_angle_bisector_A_x_axis) -
          sin(last_angle_bisector_A_x_axis));
  float2 outer_last_bevel_start = last_circle_center +
                                  l_last_circle_radius * make_float2(cos(ref_A_angle_bisector),
                                                                sin(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                                     outer_last_bevel_start.x);
  float outer_last_bevel_start_A_angle_bisector = ref_A_angle_bisector -
                                                  x_axis_A_outer_last_bevel_start;

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) ||
        (x_axis_A_coord < ref_A_angle_bisector))
    {
      /* Regular straight part. */

      float l_angle_bisector = 0.0;
      float r_gon_parameter = 0.0;

      l_angle_bisector = l_coord * cos(ref_A_angle_bisector - ref_A_coord);

      float effective_roundness = 1.0 - tan(ref_A_angle_bisector -
                                                   x_axis_A_outer_last_bevel_start) /
                                             tan(ref_A_angle_bisector);
      float spline_start_outer_last_bevel_start = (1.0 - effective_roundness) *
                                                  x_axis_A_outer_last_bevel_start;

      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = l_angle_bisector *
                          tan(abs(ref_A_angle_bisector - ref_A_coord));
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          float normalize_based_on_l_angle_bisector =
              l_angle_bisector * tan(outer_last_bevel_start_A_angle_bisector) +
              spline_start_outer_last_bevel_start * (0.5 * l_angle_bisector + 0.5) +
              effective_roundness * x_axis_A_outer_last_bevel_start;

          r_gon_parameter /= normalize_based_on_l_angle_bisector;
        }
      }
      return make_float4(l_angle_bisector,
                    r_gon_parameter,
                    ref_A_angle_bisector,
                    segment_id * ref_A_next_ref + ref_A_angle_bisector);
    }
    else {
      /* Regular rounded part. */

      float r_gon_parameter = 0.0;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = abs(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0;
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
    /* Irregular rounded part. */

    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
        (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1.0;
    }
    float l_angle_bisector = 0.0;
    float r_gon_parameter = 0.0;
    float max_unit_parameter = 0.0;
    float x_axis_A_angle_bisector = 0.0;

    float l_coord_R_l_last_angle_bisector =
        sin(nearest_ref_MSA_coord) * last_circle_center.y +
        cos(nearest_ref_MSA_coord) * last_circle_center.x +
        sqrt(square(sin(nearest_ref_MSA_coord) * last_circle_center.y +
                                cos(nearest_ref_MSA_coord) * last_circle_center.x) +
                   square(l_last_circle_radius) - square(last_circle_center.x) -
                   square(last_circle_center.y));
    float l_angle_bisector_R_l_last_angle_bisector = cos(ref_A_angle_bisector) /
                                                     cos(last_angle_bisector_A_x_axis);

    l_angle_bisector = l_angle_bisector_R_l_last_angle_bisector * l_coord /
                       l_coord_R_l_last_angle_bisector;

    if (nearest_ref_MSA_coord < 0.0) {
      /* Irregular rounded inner part. */

      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = l_angle_bisector_R_l_last_angle_bisector *
                          (last_angle_bisector_A_x_axis + nearest_ref_MSA_coord);
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter *= -1.0;
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
      /* Irregular rounded outer part. */

      float effective_roundness = 1.0 - tan(ref_A_angle_bisector -
                                                   x_axis_A_outer_last_bevel_start) /
                                             tan(ref_A_angle_bisector);
      float spline_start_outer_last_bevel_start = (1.0 - effective_roundness) *
                                                  x_axis_A_outer_last_bevel_start;

      if (calculate_r_gon_parameter_field) {
        float coord_A_bevel_start = x_axis_A_outer_last_bevel_start -
                                    abs(nearest_ref_MSA_coord);
        r_gon_parameter = l_coord * sin(outer_last_bevel_start_A_angle_bisector);

        if (coord_A_bevel_start < spline_start_outer_last_bevel_start) {
          r_gon_parameter +=
              l_coord * cos(outer_last_bevel_start_A_angle_bisector) * coord_A_bevel_start +
              0.5 * (1.0 - l_coord * cos(outer_last_bevel_start_A_angle_bisector)) *
                  square(coord_A_bevel_start) / spline_start_outer_last_bevel_start;
        }
        else {
          r_gon_parameter +=
              spline_start_outer_last_bevel_start *
                  (0.5 * l_coord * cos(outer_last_bevel_start_A_angle_bisector) - 0.5) +
              coord_A_bevel_start;
        }
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          float normalize_based_on_l_angle_bisector =
              l_angle_bisector * tan(outer_last_bevel_start_A_angle_bisector) +
              spline_start_outer_last_bevel_start * (0.5 * l_angle_bisector + 0.5) +
              effective_roundness * x_axis_A_outer_last_bevel_start;
          float normalize_based_on_l_coord =
              l_coord * sin(outer_last_bevel_start_A_angle_bisector) +
              spline_start_outer_last_bevel_start *
                  (0.5 * l_coord * cos(outer_last_bevel_start_A_angle_bisector) + 0.5) +
              effective_roundness * x_axis_A_outer_last_bevel_start;

          /* For effective_roundness -> 1.0 the normalize_based_on_l_angle_bisector field and
           * normalize_based_on_l_coord field converge against the same scalar field. */
          r_gon_parameter /= mix(normalize_based_on_l_angle_bisector,
                                       normalize_based_on_l_coord,
                                       coord_A_bevel_start / x_axis_A_outer_last_bevel_start);
        }
      }
      max_unit_parameter = ref_A_angle_bisector;
      x_axis_A_angle_bisector = segment_id * ref_A_next_ref + ref_A_angle_bisector;
    }
    return make_float4(l_angle_bisector, r_gon_parameter, max_unit_parameter, x_axis_A_angle_bisector);
  }
}

RETURN_ARGUMENTS calculate_out_fields_irregular_circular(bool calculate_r_gon_parameter_field,
                                               bool calculate_max_unit_parameter,
                                               bool normalize_r_gon_parameter,
                                               float r_gon_sides,
                                               float r_gon_roundness,
                                               float2 coord,
                                               float l_coord)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
  float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1.0 - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI_F - floor(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1.0 - r_gon_roundness) *
                                                     tan(last_angle_bisector_A_x_axis));
  float l_last_circle_radius = r_gon_roundness * tan(last_angle_bisector_A_x_axis) /
                               tan(0.5 *
                                         (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  float2 last_circle_center = make_float2(
      (cos(inner_last_bevel_start_A_x_axis) /
       cos(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)) -
          l_last_circle_radius * cos(last_angle_bisector_A_x_axis),
      l_last_circle_radius * sin(last_angle_bisector_A_x_axis) -
          (sin(inner_last_bevel_start_A_x_axis) /
           cos(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)));
  float2 outer_last_bevel_start = last_circle_center +
                                  l_last_circle_radius * make_float2(cos(ref_A_angle_bisector),
                                                                sin(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
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
      /* Regular straight part. */

      float l_angle_bisector = 0.0;
      float r_gon_parameter = 0.0;
      float max_unit_parameter = 0.0;

      l_angle_bisector = l_coord * cos(ref_A_angle_bisector - ref_A_coord);

      float spline_start_bevel_start = (1.0 - r_gon_roundness) * ref_A_bevel_start;

      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) ||
          (x_axis_A_coord < ref_A_angle_bisector))
      {
        /* Irregular rounded outer part. */

        float effective_roundness = 1.0 - tan(ref_A_angle_bisector -
                                                     x_axis_A_outer_last_bevel_start) /
                                               tan(ref_A_angle_bisector);
        float spline_start_outer_last_bevel_start = (1.0 - effective_roundness) *
                                                    x_axis_A_outer_last_bevel_start;

        if (calculate_r_gon_parameter_field) {
          r_gon_parameter = l_angle_bisector *
                            tan(abs(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            float normalize_based_on_l_angle_bisector =
                l_angle_bisector * tan(outer_last_bevel_start_A_angle_bisector) +
                spline_start_outer_last_bevel_start * (0.5 * l_angle_bisector + 0.5) +
                effective_roundness * x_axis_A_outer_last_bevel_start;

            r_gon_parameter /= normalize_based_on_l_angle_bisector;
          }
        }
      }
      else {
        /* Regular straight part. */

        float spline_start_bevel_start = (1.0 - r_gon_roundness) * ref_A_bevel_start;

        if (calculate_r_gon_parameter_field) {
          r_gon_parameter = l_angle_bisector *
                            tan(abs(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            float normalize_based_on_l_angle_bisector =
                l_angle_bisector * tan(bevel_start_A_angle_bisector) +
                spline_start_bevel_start * (0.5 * l_angle_bisector + 0.5) +
                r_gon_roundness * ref_A_bevel_start;

            r_gon_parameter /= normalize_based_on_l_angle_bisector;
          }
        }
      }

      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = l_angle_bisector *
                          tan(abs(ref_A_angle_bisector - ref_A_coord));
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0;
        }

        if (normalize_r_gon_parameter) {
          if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) ||
              (x_axis_A_coord < ref_A_angle_bisector))
          {
            /* Irregular rounded outer part. */

            float effective_roundness = 1.0 - tan(ref_A_angle_bisector -
                                                         x_axis_A_outer_last_bevel_start) /
                                                   tan(ref_A_angle_bisector);
            float spline_start_outer_last_bevel_start = (1.0 - effective_roundness) *
                                                        x_axis_A_outer_last_bevel_start;

            float normalize_based_on_l_angle_bisector =
                l_angle_bisector * tan(outer_last_bevel_start_A_angle_bisector) +
                spline_start_outer_last_bevel_start * (0.5 * l_angle_bisector + 0.5) +
                effective_roundness * x_axis_A_outer_last_bevel_start;

            r_gon_parameter /= normalize_based_on_l_angle_bisector;
          }
          else {
            /* Regular straight part. */

            float spline_start_bevel_start = (1.0 - r_gon_roundness) * ref_A_bevel_start;

            float normalize_based_on_l_angle_bisector =
                l_angle_bisector * tan(bevel_start_A_angle_bisector) +
                spline_start_bevel_start * (0.5 * l_angle_bisector + 0.5) +
                r_gon_roundness * ref_A_bevel_start;

            r_gon_parameter /= normalize_based_on_l_angle_bisector;
          }
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter = tan(bevel_start_A_angle_bisector) + spline_start_bevel_start +
                             r_gon_roundness * ref_A_bevel_start;
      }
      return make_float4(l_angle_bisector,
                    r_gon_parameter,
                    max_unit_parameter,
                    segment_id * ref_A_next_ref + ref_A_angle_bisector);
    }
    else {
      /* Regular rounded part. */

      /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_angle_bisector = 0.0;
      float r_gon_parameter = 0.0;
      float max_unit_parameter = 0.0;

      float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
      float l_circle_center = sin(bevel_start_A_angle_bisector) /
                              sin(ref_A_angle_bisector);
      float l_coord_R_l_bevel_start =
          cos(nearest_ref_SA_coord) * l_circle_center +
          sqrt(square(cos(nearest_ref_SA_coord) * l_circle_center) +
                     square(l_circle_radius) - square(l_circle_center));

      l_angle_bisector = l_coord * cos(bevel_start_A_angle_bisector) /
                         l_coord_R_l_bevel_start;

      float spline_start_bevel_start = (1.0 - r_gon_roundness) * ref_A_bevel_start;

      if (calculate_r_gon_parameter_field) {
        float coord_A_bevel_start = ref_A_bevel_start - abs(nearest_ref_SA_coord);
        r_gon_parameter = l_coord * sin(bevel_start_A_angle_bisector);

        if (coord_A_bevel_start < spline_start_bevel_start) {
          r_gon_parameter += l_coord * cos(bevel_start_A_angle_bisector) *
                                 coord_A_bevel_start +
                             0.5 * (1.0 - l_coord * cos(bevel_start_A_angle_bisector)) *
                                 square(coord_A_bevel_start) / spline_start_bevel_start;
        }
        else {
          r_gon_parameter += spline_start_bevel_start *
                                 (0.5 * l_coord * cos(bevel_start_A_angle_bisector) -
                                  0.5) +
                             coord_A_bevel_start;
        }
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          float normalize_based_on_l_angle_bisector = l_angle_bisector *
                                                          tan(bevel_start_A_angle_bisector) +
                                                      spline_start_bevel_start *
                                                          (0.5 * l_angle_bisector + 0.5) +
                                                      r_gon_roundness * ref_A_bevel_start;
          float normalize_based_on_l_coord =
              l_coord * sin(bevel_start_A_angle_bisector) +
              spline_start_bevel_start *
                  (0.5 * l_coord * cos(bevel_start_A_angle_bisector) + 0.5) +
              r_gon_roundness * ref_A_bevel_start;

          /* For r_gon_roundness -> 1.0 the normalize_based_on_l_angle_bisector field and
           * normalize_based_on_l_coord field converge against the same scalar field. */
          r_gon_parameter /= mix(normalize_based_on_l_angle_bisector,
                                       normalize_based_on_l_coord,
                                       coord_A_bevel_start / ref_A_bevel_start);
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter = tan(bevel_start_A_angle_bisector) + spline_start_bevel_start +
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
      /* Irregular straight part. */

      float l_angle_bisector = 0.0;
      float r_gon_parameter = 0.0;
      float max_unit_parameter = 0.0;

      float l_angle_bisector_R_l_last_angle_bisector = cos(ref_A_angle_bisector) /
                                                       cos(last_angle_bisector_A_x_axis);
      float l_last_angle_bisector = l_coord *
                                    cos(last_angle_bisector_A_x_axis - ref_A_coord);

      l_angle_bisector = l_angle_bisector_R_l_last_angle_bisector * l_last_angle_bisector;

      float spline_start_bevel_start = (1.0 - r_gon_roundness) * inner_last_bevel_start_A_x_axis;

      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = l_angle_bisector_R_l_last_angle_bisector * l_last_angle_bisector *
                          tan(abs(last_angle_bisector_A_x_axis - ref_A_coord));
        if (ref_A_coord < last_angle_bisector_A_x_axis) {
          r_gon_parameter *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          float normalize_based_on_l_l_angle_bisector =
              l_angle_bisector_R_l_last_angle_bisector *
              (l_last_angle_bisector * tan(inner_last_bevel_start_A_last_angle_bisector) +
               spline_start_bevel_start * (0.5 * l_last_angle_bisector + 0.5) +
               r_gon_roundness * inner_last_bevel_start_A_x_axis);

          r_gon_parameter /= normalize_based_on_l_l_angle_bisector;
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter = tan(inner_last_bevel_start_A_last_angle_bisector) +
                             l_angle_bisector_R_l_last_angle_bisector *
                                 (spline_start_bevel_start *
                                      ((0.5 / l_angle_bisector_R_l_last_angle_bisector) + 0.5) +
                                  r_gon_roundness * inner_last_bevel_start_A_x_axis);
      }
      return make_float4(l_angle_bisector,
                    r_gon_parameter,
                    max_unit_parameter,
                    segment_id * ref_A_next_ref + last_angle_bisector_A_x_axis);
    }
    else {
      /* Irregular rounded part. */

      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
          (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1.0;
      }
      float l_angle_bisector = 0.0;
      float r_gon_parameter = 0.0;
      float max_unit_parameter = 0.0;
      float x_axis_A_angle_bisector = 0.0;

      float l_coord_R_l_last_angle_bisector =
          sin(nearest_ref_MSA_coord) * last_circle_center.y +
          cos(nearest_ref_MSA_coord) * last_circle_center.x +
          sqrt(square(sin(nearest_ref_MSA_coord) * last_circle_center.y +
                                  cos(nearest_ref_MSA_coord) * last_circle_center.x) +
                     square(l_last_circle_radius) - square(last_circle_center.x) -
                     square(last_circle_center.y));
      float l_angle_bisector_R_l_last_angle_bisector = cos(ref_A_angle_bisector) /
                                                       cos(last_angle_bisector_A_x_axis);
      float l_last_angle_bisector = l_coord / l_coord_R_l_last_angle_bisector;

      l_angle_bisector = l_angle_bisector_R_l_last_angle_bisector * l_last_angle_bisector;

      if (nearest_ref_MSA_coord < 0.0) {
        /* Irregular rounded inner part. */

        float spline_start_bevel_start = (1.0 - r_gon_roundness) *
                                         inner_last_bevel_start_A_x_axis;

        if (calculate_r_gon_parameter_field) {
          float coord_A_bevel_start = inner_last_bevel_start_A_x_axis -
                                      abs(nearest_ref_MSA_coord);
          r_gon_parameter = l_angle_bisector_R_l_last_angle_bisector * l_coord *
                            sin(inner_last_bevel_start_A_last_angle_bisector);

          if (coord_A_bevel_start < spline_start_bevel_start) {
            r_gon_parameter +=
                l_angle_bisector_R_l_last_angle_bisector *
                (l_coord * cos(inner_last_bevel_start_A_last_angle_bisector) *
                     coord_A_bevel_start +
                 0.5 *
                     (1.0 - l_coord * cos(inner_last_bevel_start_A_last_angle_bisector)) *
                     square(coord_A_bevel_start) / spline_start_bevel_start);
          }
          else {
            r_gon_parameter += l_angle_bisector_R_l_last_angle_bisector *
                               (spline_start_bevel_start *
                                    (0.5 * l_coord *
                                         cos(inner_last_bevel_start_A_last_angle_bisector) -
                                     0.5) +
                                coord_A_bevel_start);
          }
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            float normalize_based_on_l_l_angle_bisector =
                l_last_angle_bisector * tan(inner_last_bevel_start_A_last_angle_bisector) +
                spline_start_bevel_start * (0.5 * l_last_angle_bisector + 0.5) +
                r_gon_roundness * inner_last_bevel_start_A_x_axis;
            float normalize_based_on_l_coord =
                l_coord * sin(inner_last_bevel_start_A_last_angle_bisector) +
                spline_start_bevel_start *
                    (0.5 * l_coord * cos(inner_last_bevel_start_A_last_angle_bisector) +
                     0.5) +
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
          max_unit_parameter = tan(inner_last_bevel_start_A_last_angle_bisector) +
                               l_angle_bisector_R_l_last_angle_bisector *
                                   (spline_start_bevel_start *
                                        ((0.5 / l_angle_bisector_R_l_last_angle_bisector) +
                                         0.5) +
                                    r_gon_roundness * inner_last_bevel_start_A_x_axis);
        }
        x_axis_A_angle_bisector = segment_id * ref_A_next_ref + last_angle_bisector_A_x_axis;
      }
      else {
        /* Irregular rounded outer part. */

        float effective_roundness = 1.0 - tan(ref_A_angle_bisector -
                                                     x_axis_A_outer_last_bevel_start) /
                                               tan(ref_A_angle_bisector);
        float spline_start_outer_last_bevel_start = (1.0 - effective_roundness) *
                                                    x_axis_A_outer_last_bevel_start;

        if (calculate_r_gon_parameter_field) {
          float coord_A_bevel_start = x_axis_A_outer_last_bevel_start -
                                      abs(nearest_ref_MSA_coord);
          r_gon_parameter = l_coord * sin(outer_last_bevel_start_A_angle_bisector);

          if (coord_A_bevel_start < spline_start_outer_last_bevel_start) {
            r_gon_parameter +=
                l_coord * cos(outer_last_bevel_start_A_angle_bisector) *
                    coord_A_bevel_start +
                0.5 * (1.0 - l_coord * cos(outer_last_bevel_start_A_angle_bisector)) *
                    square(coord_A_bevel_start) / spline_start_outer_last_bevel_start;
          }
          else {
            r_gon_parameter +=
                spline_start_outer_last_bevel_start *
                    (0.5 * l_coord * cos(outer_last_bevel_start_A_angle_bisector) - 0.5) +
                coord_A_bevel_start;
          }
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            float normalize_based_on_l_angle_bisector =
                l_angle_bisector * tan(outer_last_bevel_start_A_angle_bisector) +
                spline_start_outer_last_bevel_start * (0.5 * l_angle_bisector + 0.5) +
                effective_roundness * x_axis_A_outer_last_bevel_start;
            float normalize_based_on_l_coord =
                l_coord * sin(outer_last_bevel_start_A_angle_bisector) +
                spline_start_outer_last_bevel_start *
                    (0.5 * l_coord * cos(outer_last_bevel_start_A_angle_bisector) + 0.5) +
                effective_roundness * x_axis_A_outer_last_bevel_start;

            /* For effective_roundness -> 1.0 the normalize_based_on_l_angle_bisector field and
             * normalize_based_on_l_coord field converge against the same scalar field. */
            r_gon_parameter /= mix(normalize_based_on_l_angle_bisector,
                                         normalize_based_on_l_coord,
                                         coord_A_bevel_start / x_axis_A_outer_last_bevel_start);
          }
        }
        if (calculate_max_unit_parameter) {
          float bevel_start_A_angle_bisector = ref_A_angle_bisector - ref_A_bevel_start;
          float spline_start_bevel_start = (1.0 - r_gon_roundness) * ref_A_bevel_start;

          max_unit_parameter = tan(bevel_start_A_angle_bisector) + spline_start_bevel_start +
                               r_gon_roundness * ref_A_bevel_start;
        }
        x_axis_A_angle_bisector = segment_id * ref_A_next_ref + ref_A_angle_bisector;
      }
      return make_float4(
          l_angle_bisector, r_gon_parameter, max_unit_parameter, x_axis_A_angle_bisector);
    }
  }
}

RETURN_ARGUMENTS calculate_out_fields(bool calculate_r_gon_parameter_field,
                            bool calculate_max_unit_parameter,
                            bool normalize_r_gon_parameter,
                            bool elliptical_corners,
                            float r_gon_sides,
                            float r_gon_roundness,
                            float2 coord)
{
  float l_coord = sqrt(square(coord.x) + square(coord.y));

  if (fract(r_gon_sides) == 0.0) {
    float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU_F;
    float ref_A_angle_bisector = M_PI_F / r_gon_sides;
    float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
    float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
    float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

    if (r_gon_roundness == 0.0) {
      /* Regular straight part. */

      float l_angle_bisector = 0.0;
      float r_gon_parameter = 0.0;
      float max_unit_parameter = 0.0;

      l_angle_bisector = l_coord * cos(ref_A_angle_bisector - ref_A_coord);

      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = l_angle_bisector *
                          tan(abs(ref_A_angle_bisector - ref_A_coord));
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0;
        }
        if (normalize_r_gon_parameter && (r_gon_sides != 2.0)) {
          r_gon_parameter /= l_angle_bisector * tan(ref_A_angle_bisector);
        }
      }
      if (calculate_max_unit_parameter) {
        max_unit_parameter = (r_gon_sides != 2.0) ? tan(ref_A_angle_bisector) : 0.0;
      }
      return make_float4(l_angle_bisector,
                    r_gon_parameter,
                    max_unit_parameter,
                    segment_id * ref_A_next_ref + ref_A_angle_bisector);
    }
    else if (r_gon_roundness == 1.0) {
      /* Regular rounded part. */

      float r_gon_parameter = 0.0;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter = abs(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter *= -1.0;
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
      float ref_A_bevel_start = ref_A_angle_bisector - atan((1.0 - r_gon_roundness) *
                                                                  tan(ref_A_angle_bisector));
      float bevel_start_A_angle_bisector = ref_A_angle_bisector - ref_A_bevel_start;

      if ((ref_A_coord >= ref_A_next_ref - ref_A_bevel_start) || (ref_A_coord < ref_A_bevel_start))
      {
        /* Regular rounded part. */

        /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
         * angles are negative.*/
        float nearest_ref_SA_coord = ref_A_coord -
                                     float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
        float l_angle_bisector = 0.0;
        float r_gon_parameter = 0.0;
        float max_unit_parameter = 0.0;

        float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
        float l_circle_center = sin(bevel_start_A_angle_bisector) /
                                sin(ref_A_angle_bisector);
        float l_coord_R_l_bevel_start =
            cos(nearest_ref_SA_coord) * l_circle_center +
            sqrt(square(cos(nearest_ref_SA_coord) * l_circle_center) +
                       square(l_circle_radius) - square(l_circle_center));

        l_angle_bisector = l_coord * cos(bevel_start_A_angle_bisector) /
                           l_coord_R_l_bevel_start;

        float spline_start_bevel_start = (1.0 - r_gon_roundness) * ref_A_bevel_start;

        if (calculate_r_gon_parameter_field) {
          float coord_A_bevel_start = ref_A_bevel_start - abs(nearest_ref_SA_coord);
          r_gon_parameter = l_coord * sin(bevel_start_A_angle_bisector);

          if (coord_A_bevel_start < spline_start_bevel_start) {
            r_gon_parameter += l_coord * cos(bevel_start_A_angle_bisector) *
                                   coord_A_bevel_start +
                               0.5 * (1.0 - l_coord * cos(bevel_start_A_angle_bisector)) *
                                   square(coord_A_bevel_start) / spline_start_bevel_start;
          }
          else {
            r_gon_parameter += spline_start_bevel_start *
                                   (0.5 * l_coord * cos(bevel_start_A_angle_bisector) -
                                    0.5) +
                               coord_A_bevel_start;
          }
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            float normalize_based_on_l_angle_bisector =
                l_angle_bisector * tan(bevel_start_A_angle_bisector) +
                spline_start_bevel_start * (0.5 * l_angle_bisector + 0.5) +
                r_gon_roundness * ref_A_bevel_start;
            float normalize_based_on_l_coord =
                l_coord * sin(bevel_start_A_angle_bisector) +
                spline_start_bevel_start *
                    (0.5 * l_coord * cos(bevel_start_A_angle_bisector) + 0.5) +
                r_gon_roundness * ref_A_bevel_start;

            /* For r_gon_roundness -> 1.0 the normalize_based_on_l_angle_bisector field and
             * normalize_based_on_l_coord field converge against the same scalar field. */
            r_gon_parameter /= mix(normalize_based_on_l_angle_bisector,
                                         normalize_based_on_l_coord,
                                         coord_A_bevel_start / ref_A_bevel_start);
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter = tan(bevel_start_A_angle_bisector) + spline_start_bevel_start +
                               r_gon_roundness * ref_A_bevel_start;
        }
        return make_float4(l_angle_bisector,
                      r_gon_parameter,
                      max_unit_parameter,
                      segment_id * ref_A_next_ref + ref_A_angle_bisector);
      }
      else {
        /* Regular straight part. */

        float l_angle_bisector = 0.0;
        float r_gon_parameter = 0.0;
        float max_unit_parameter = 0.0;

        l_angle_bisector = l_coord * cos(ref_A_angle_bisector - ref_A_coord);

        float spline_start_bevel_start = (1.0 - r_gon_roundness) * ref_A_bevel_start;

        if (calculate_r_gon_parameter_field) {
          r_gon_parameter = l_angle_bisector *
                            tan(abs(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            float normalize_based_on_l_angle_bisector =
                l_angle_bisector * tan(bevel_start_A_angle_bisector) +
                spline_start_bevel_start * (0.5 * l_angle_bisector + 0.5) +
                r_gon_roundness * ref_A_bevel_start;

            r_gon_parameter /= normalize_based_on_l_angle_bisector;
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter = tan(bevel_start_A_angle_bisector) + spline_start_bevel_start +
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
    if (r_gon_roundness == 0.0) {
      float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU_F;
      float ref_A_angle_bisector = M_PI_F / r_gon_sides;
      float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
      float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
      float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

      float last_angle_bisector_A_x_axis = M_PI_F -
                                           floor(r_gon_sides) * ref_A_angle_bisector;
      float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;

      if (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis) {
        /* Regular straight part. */

        float l_angle_bisector = 0.0;
        float r_gon_parameter = 0.0;
        float max_unit_parameter = 0.0;

        l_angle_bisector = l_coord * cos(ref_A_angle_bisector - ref_A_coord);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter = l_angle_bisector *
                            tan(abs(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter /= l_angle_bisector * tan(ref_A_angle_bisector);
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter = tan(ref_A_angle_bisector);
        }
        return make_float4(l_angle_bisector,
                      r_gon_parameter,
                      max_unit_parameter,
                      segment_id * ref_A_next_ref + ref_A_angle_bisector);
      }
      else {
        /* Irregular straight part. */

        float l_angle_bisector = 0.0;
        float r_gon_parameter = 0.0;
        float max_unit_parameter = 0.0;

        float l_angle_bisector_R_l_last_angle_bisector = cos(ref_A_angle_bisector) /
                                                         cos(last_angle_bisector_A_x_axis);
        float l_last_angle_bisector = l_coord *
                                      cos(last_angle_bisector_A_x_axis - ref_A_coord);

        l_angle_bisector = l_angle_bisector_R_l_last_angle_bisector * l_last_angle_bisector;

        if (calculate_r_gon_parameter_field) {
          r_gon_parameter = l_angle_bisector_R_l_last_angle_bisector * l_last_angle_bisector *
                            tan(abs(last_angle_bisector_A_x_axis - ref_A_coord));
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter /= l_angle_bisector_R_l_last_angle_bisector * l_last_angle_bisector *
                               tan(last_angle_bisector_A_x_axis);
          }
        }
        if (calculate_max_unit_parameter) {
          max_unit_parameter = tan(last_angle_bisector_A_x_axis);
        }
        return make_float4(l_angle_bisector,
                      r_gon_parameter,
                      max_unit_parameter,
                      segment_id * ref_A_next_ref + last_angle_bisector_A_x_axis);
      }
    }
    else if (r_gon_roundness == 1.0) {
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
