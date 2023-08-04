#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_compositor_texture_utilities.glsl)

/* An implementation of the Anisotropic Kuwahara filter described in the paper:
 *
 *   Kyprianidis, Jan Eric, Henry Kang, and Jurgen Dollner. "Image and video abstraction by
 *   anisotropic Kuwahara filtering." 2009.
 *
 * But with the polynomial weighting functions described in the paper:
 *
 *   Kyprianidis, Jan Eric, et al. "Anisotropic Kuwahara Filtering with Polynomial Weighting
 *   Functions." 2010.
 */
void main()
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);

  /* The structure tensor is encoded in a vec4 using a column major storage order, as can be seen
   * in the compositor_kuwahara_anisotropic_compute_structure_tensor.glsl shader. */
  vec4 encoded_structure_tensor = texture_load(structure_tensor_tx, texel);
  float dxdx = encoded_structure_tensor.x;
  float dxdy = encoded_structure_tensor.y;
  float dydy = encoded_structure_tensor.w;

  /* Compute the first and second eigenvalues of the structure tensor using the equations in
   * section "3.1 Orientation and Anisotropy Estimation" of the paper. */
  float eigenvalue_first_term = (dxdx + dydy) / 2.0;
  float eigenvalue_square_root_term = sqrt(pow(dxdx - dydy, 2.0) + 4.0 * pow(dxdy, 2.0)) / 2.0;
  float first_eigenvalue = eigenvalue_first_term + eigenvalue_square_root_term;
  float second_eigenvalue = eigenvalue_first_term - eigenvalue_square_root_term;

  /* Compute the normalized eigenvector of the structure tensor oriented in direction of the
   * minimum rate of change using the equations in section "3.1 Orientation and Anisotropy
   * Estimation" of the paper. */
  vec2 eigenvector = vec2(first_eigenvalue - dxdx, -dxdy);
  float eigenvector_length = length(eigenvector);
  vec2 unit_eigenvector = eigenvector_length != 0.0 ? eigenvector / eigenvector_length : vec2(1.0);

  /* Compute the amount of anisotropy using equations in section "3.1 Orientation and Anisotropy
   * Estimation" of the paper. The anisotropy ranges from 0 to 1, where 0 corresponds to isotropic
   * and 1 corresponds to entirely anisotropic regions. */
  float eigenvalue_sum = first_eigenvalue + second_eigenvalue;
  float eigenvalue_difference = first_eigenvalue - second_eigenvalue;
  float anisotropy = eigenvalue_sum > 0.0 ? eigenvalue_difference / eigenvalue_sum : 0.0;

  /* Compute the width and height of an ellipse that is more width-elongated for high anisotropy
   * and more circular for low anisotropy, controlled using the eccentricity factor. Since the
   * anisotropy is in the [0, 1] range, the width factor tends to 1 as the eccentricity tends to
   * infinity. This is based on the equations in section "3.2. Anisotropic Kuwahara Filtering" of
   * the paper. */
  float ellipse_width_factor = (eccentricity + anisotropy) / eccentricity;
  float ellipse_width = ellipse_width_factor * radius;
  float ellipse_height = radius / ellipse_width_factor;

  /* Compute the cosine and sine of the angle that the eigenvector makes with the x axis. Since the
   * eigenvector is normalized, its x and y components are the cosine and sine of the angle it
   * makes with the x axis. */
  float cosine = unit_eigenvector.x;
  float sine = unit_eigenvector.y;

  /* Compute an inverse transformation matrix that represents an ellipse of the given width and
   * height and makes and an angle with the x axis of the given cosine and sine. This is an inverse
   * matrix, so it transforms the ellipse into a disk of unit radius. */
  mat2 inverse_ellipse_matrix = mat2(cosine / ellipse_width,
                                     -sine / ellipse_height,
                                     sine / ellipse_width,
                                     cosine / ellipse_height);

  /* Compute the bounding box of a zero centered ellipse whose major axis is aligned with the
   * eigenvector and has the given width and height. This is based on the equations described in:
   *
   *   https://iquilezles.org/articles/ellipses/
   *
   * Notice that we only compute the upper bound, the lower bound is just negative that since the
   * ellipse is zero centered. */
  vec2 ellipse_major_axis = ellipse_width * unit_eigenvector;
  vec2 ellipse_minor_axis = ellipse_height * unit_eigenvector.yx * vec2(-1, 1);
  ivec2 ellipse_bounds = ivec2(
      sqrt(pow(ellipse_major_axis, vec2(2.0)) + pow(ellipse_minor_axis, vec2(2.0))));

  /* Compute the overlap polynomial parameters for 8-sector ellipse based on the equations in
   * section "3 Alternative Weighting Functions" of the polynomial weights paper. More on this
   * later in the code. */
  int number_of_sectors = 8;
  float sector_center_overlap_parameter = 2.0 / radius;
  float sector_envelope_angle = ((3.0 / 2.0) * M_PI) / number_of_sectors;
  float cross_sector_overlap_parameter = (sector_center_overlap_parameter +
                                          cos(sector_envelope_angle)) /
                                         pow(sin(sector_envelope_angle), 2.0);

  /* We need to compute the weighted mean of color and squared color of each of the 8 sectors of
   * the ellipse, so we initialize zero arrays for accumulating that. */
  vec4 weighted_mean_of_squared_color_of_sectors[8] = vec4[](
      vec4(0.0), vec4(0.0), vec4(0.0), vec4(0.0), vec4(0.0), vec4(0.0), vec4(0.0), vec4(0.0));
  vec4 weighted_mean_of_color_of_sectors[8] = vec4[](
      vec4(0.0), vec4(0.0), vec4(0.0), vec4(0.0), vec4(0.0), vec4(0.0), vec4(0.0), vec4(0.0));
  float sum_of_weights_of_sectors[8] = float[](0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  /* Loop over the window of pixels inside the bounding box of the ellipse. However, we utilize the
   * fact that ellipses are mirror symmetric along the horizontal axis, so we reduce the window to
   * only the upper two quadrants, and compute each two mirrored pixels at the same time using the
   * same weight as an optimization. */
  for (int j = 0; j <= ellipse_bounds.y; j++) {
    for (int i = -ellipse_bounds.x; i <= ellipse_bounds.x; i++) {
      /* Since we compute each two mirrored pixels at the same time, we need to also exempt the
       * pixels whose x coordinates are negative and their y coordinates are zero, that's because
       * those are mirrored versions of the pixels whose x coordinates are positive and their y
       * coordinates are zero, and we don't want to compute and accumulate them twice. */
      if (j == 0 && i <= 0) {
        continue;
      }

      /* Map the pixels of the ellipse into a unit disk, exempting any points that are not part of
       * the ellipse or disk. */
      vec2 disk_point = inverse_ellipse_matrix * vec2(i, j);
      float disk_point_length_squared = dot(disk_point, disk_point);
      if (disk_point_length_squared > 1.0) {
        continue;
      }

      /* While each pixel belongs to a single sector in the ellipse, we expand the definition of
       * a sector a bit to also overlap with other sectors as illustrated in Figure 8 of the
       * polynomial weights paper. So each pixel may contribute to multiple sectors, and thus we
       * compute its weight in each of the 8 sectors. */
      float sector_weights[8];

      /* We evaluate the weighting polynomial at each of the 8 sectors by rotating the disk point
       * by 45 degrees and evaluating the weighting polynomial at each incremental rotation. To
       * avoid potentially expensive rotations, we utilize the fact that rotations by 90 degrees
       * are simply swapping of the coordinates and negating the x component. We also note that
       * since the y term of the weighting polynomial is squared, it is not affected by the sign
       * and can be computed once for the x and once for the y coordinates. So we compute every
       * other 4 weights by successive 90 degree rotations as discussed. */
      vec2 polynomial = sector_center_overlap_parameter -
                        cross_sector_overlap_parameter * pow(disk_point, vec2(2.0));
      sector_weights[0] = pow(max(0.0, disk_point.y + polynomial.x), 2.0);
      sector_weights[2] = pow(max(0.0, -disk_point.x + polynomial.y), 2.0);
      sector_weights[4] = pow(max(0.0, -disk_point.y + polynomial.x), 2.0);
      sector_weights[6] = pow(max(0.0, disk_point.x + polynomial.y), 2.0);

      /* Then we rotate the disk point by 45 degrees, which is a simple expression involving a
       * constant as can be demonstrated by applying a 45 degree rotation matrix. */
      vec2 rotated_disk_point = M_SQRT1_2 *
                                vec2(disk_point.x - disk_point.y, disk_point.x + disk_point.y);

      /* Finally, we compute the other every other 4 weights starting from the 45 degreed rotated
       * disk point. */
      vec2 rotated_polynomial = sector_center_overlap_parameter -
                                cross_sector_overlap_parameter *
                                    pow(rotated_disk_point, vec2(2.0));
      sector_weights[1] = pow(max(0.0, rotated_disk_point.y + rotated_polynomial.x), 2.0);
      sector_weights[3] = pow(max(0.0, -rotated_disk_point.x + rotated_polynomial.y), 2.0);
      sector_weights[5] = pow(max(0.0, -rotated_disk_point.y + rotated_polynomial.x), 2.0);
      sector_weights[7] = pow(max(0.0, rotated_disk_point.x + rotated_polynomial.y), 2.0);

      /* We compute a radial Gaussian weighting component such that pixels further away from the
       * sector center gets attenuated, and we also divide by the sum of sector weights to
       * normalize them, since the radial weight will eventually be multiplied to the sector weight
       * below. */
      float sector_weights_sum = sector_weights[0] + sector_weights[1] + sector_weights[2] +
                                 sector_weights[3] + sector_weights[4] + sector_weights[5] +
                                 sector_weights[6] + sector_weights[7];
      float radial_gaussian_weight = exp(-M_PI * disk_point_length_squared) / sector_weights_sum;

      /* Load the color of the pixel and its mirrored pixel and compute their square. */
      vec4 upper_color = texture_load(input_tx, texel + ivec2(i, j));
      vec4 lower_color = texture_load(input_tx, texel - ivec2(i, j));
      vec4 upper_color_squared = upper_color * upper_color;
      vec4 lower_color_squared = lower_color * lower_color;

      for (int k = 0; k < number_of_sectors; k++) {
        float weight = sector_weights[k] * radial_gaussian_weight;

        /* Accumulate the pixel to each of the sectors multiplied by the sector weight. */
        int upper_index = k;
        sum_of_weights_of_sectors[upper_index] += weight;
        weighted_mean_of_color_of_sectors[upper_index] += upper_color * weight;
        weighted_mean_of_squared_color_of_sectors[upper_index] += upper_color_squared * weight;

        /* Accumulate the mirrored pixel to each of the sectors multiplied by the sector weight. */
        int lower_index = (k + number_of_sectors / 2) % number_of_sectors;
        sum_of_weights_of_sectors[lower_index] += weight;
        weighted_mean_of_color_of_sectors[lower_index] += lower_color * weight;
        weighted_mean_of_squared_color_of_sectors[lower_index] += lower_color_squared * weight;
      }
    }
  }

  /* Compute the weighted sum of mean of sectors, such that sectors with lower variance gets more
   * significant weight than sectors with higher variance. */
  float sum_of_weights = 0.0;
  vec4 weighted_sum = vec4(0.0);
  for (int i = 0; i < number_of_sectors; i++) {
    weighted_mean_of_color_of_sectors[i] /= sum_of_weights_of_sectors[i];
    weighted_mean_of_squared_color_of_sectors[i] /= sum_of_weights_of_sectors[i];

    vec4 color_mean = weighted_mean_of_color_of_sectors[i];
    vec4 squared_color_mean = weighted_mean_of_squared_color_of_sectors[i];
    vec4 color_variance = squared_color_mean - color_mean * color_mean;

    float variance = dot(color_variance.rgb, vec3(1.0));

    /* Compute the sector weight based on the weight function introduced in section "3.2
     * Anisotropic Kuwahara Filtering" of the paper. The weighting function assumes an 8-bit
     * integer image, so we need to scale the variance by 255. */
    float weight = 1.0 / (1.0 + pow(255.0 * variance, sharpness));

    sum_of_weights += weight;
    weighted_sum += color_mean * weight;
  }
  weighted_sum /= sum_of_weights;

  imageStore(output_img, texel, weighted_sum);
}
