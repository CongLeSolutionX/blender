/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#ifndef __UTIL_MATH_EIGENSOLVER_H__
#define __UTIL_MATH_EIGENSOLVER_H__

CCL_NAMESPACE_BEGIN

ccl_device_inline void identity_basis(ccl_private float3 &v0,
                                      ccl_private float3 &v1,
                                      ccl_private float3 &v2)
{
  v0 = make_float3(1.0f, 0.0f, 0.0f);
  v1 = make_float3(0.0f, 1.0f, 0.0f);
  v2 = make_float3(0.0f, 0.0f, 1.0f);
}

/* Performs eigendecomposition of a symmetric 3x3 matrix with the given entries (lower-diagonal
 * entries are implicit through symmetry). Returns eigenvalues in ascending order, and sets
 * v0/v1/v2 to the corresponding eigenvectors.
 *
 * Based on "A Robust Eigensolver for 3 x 3 Symmetric Matrices" by David Eberly
 * (https://www.geometrictools.com/Documentation/RobustEigenSymmetric3x3.pdf). */
ccl_device float3 eigendecomposition_3x3_symmetric(float a00,
                                                   float a01,
                                                   float a02,
                                                   float a11,
                                                   float a12,
                                                   float a22,
                                                   ccl_private float3 &v0,
                                                   ccl_private float3 &v1,
                                                   ccl_private float3 &v2)
{
  const float max_element = max(max(max(fabsf(a00), fabsf(a01)), fabsf(a02)),
                                max(max(fabsf(a11), fabsf(a12)), fabsf(a22)));

  if (max_element == 0.0f) {
    /* Matrix is all zeroes. */
    identity_basis(v0, v1, v2);
    return zero_float3();
  }

  const float scale = 1.0f / max_element;
  a00 *= scale;
  a01 *= scale;
  a02 *= scale;
  a11 *= scale;
  a12 *= scale;
  a22 *= scale;

  const float norm = sqr(a01) + sqr(a02) + sqr(a12);
  if (norm == 0.0f) {
    /* Diagonal matrix, so directly extract eigenvalues (but make sure to sort them). */
    identity_basis(v0, v1, v2);
    const float minVal = min(min(a00, a11), a22);
    const float maxVal = max(max(a00, a11), a22);

    float midVal = a00;
    if (a00 == maxVal) {
      midVal = (a11 == minVal) ? a22 : a11;
    }
    else if (a00 == minVal) {
      midVal = (a11 == maxVal) ? a22 : a11;
    }

    return make_float3(minVal, midVal, maxVal);
  }

  /* Compute eigenvalues. */
  const float q = (a00 + a11 + a22) * (1.0f / 3.0f);
  const float b00 = a00 - q, b11 = a11 - q, b22 = a22 - q;
  const float p = sqrtf((1.0f / 6.0f) * (sqr(b00) + sqr(b11) + sqr(b22) + 2.0f * norm));
  const float c0 = b11 * b22 - a12 * a12, c1 = a01 * b22 - a12 * a02, c2 = a01 * a12 - b11 * a02;
  const float half_det = (b00 * c0 - a01 * c1 + a02 * c2) / (2.0f * p * p * p);
  const float angle = safe_acosf(half_det) * (1.0f / 3.0f);

  const float beta2 = 2.0f * cosf(angle);
  const float beta0 = 2.0f * cosf(angle + (M_2PI_F / 3.0f));
  const float beta1 = -(beta0 + beta2);
  const float3 eigvals = make_float3(q + p * beta0, q + p * beta1, q + p * beta2);

  /* Compute first eigenvector. */
  const float first_val = (half_det >= 0.0f) ? eigvals.z : eigvals.x;
  const float3 r0 = make_float3(a00 - first_val, a01, a02);
  const float3 r1 = make_float3(a01, a11 - first_val, a12);
  const float3 r2 = make_float3(a02, a12, a22 - first_val);
  const float3 x01 = cross(r0, r1), x02 = cross(r0, r2), x12 = cross(r1, r2);
  const float l01sq = len_squared(x01), l02sq = len_squared(x02), l12sq = len_squared(x12);
  float3 first_vec;
  if (l01sq > l02sq) {
    if (l01sq > l12sq) {
      first_vec = x01 / sqrtf(l01sq);
    }
    else {
      first_vec = x12 / sqrtf(l12sq);
    }
  }
  else {
    if (l02sq > l12sq) {
      first_vec = x02 / sqrtf(l02sq);
    }
    else {
      first_vec = x12 / sqrtf(l12sq);
    }
  }
  if (half_det >= 0.0f) {
    v2 = first_vec;
  }
  else {
    v0 = first_vec;
  }

  /* Compute second eigenvalue. */
  float3 U, V;
  make_orthonormals(first_vec, &U, &V);
  const float3 a0 = make_float3(a00, a01, a02);
  const float3 a1 = make_float3(a01, a11, a12);
  const float3 a2 = make_float3(a02, a12, a22);
  const float3 AU = make_float3(dot(a0, U), dot(a1, U), dot(a2, U));
  const float3 AV = make_float3(dot(a0, V), dot(a1, V), dot(a2, V));
  float m00 = dot(U, AU) - eigvals.y, m01 = dot(U, AV), m11 = dot(V, AV) - eigvals.y;
  const float m00a = fabsf(m00), m01a = fabsf(m01), m11a = fabsf(m11);
  if (m00a > m11a) {
    if (max(m00a, m01a) > 0.0f) {
      if (m00a >= m01a) {
        m01 /= m00;
        m00 = 1.0f / sqrtf(1.0f + sqr(m01));
        m01 *= m00;
      }
      else {
        m00 /= m01;
        m01 = 1.0f / sqrtf(1.0f + sqr(m00));
        m00 *= m01;
      }
      v1 = m01 * U - m00 * V;
    }
    else {
      v1 = U;
    }
  }
  else {
    if (max(m11a, m01a) > 0.0f) {
      if (m11a >= m01a) {
        m01 /= m11;
        m11 = 1.0f / sqrtf(1.0f + sqr(m01));
        m01 *= m11;
      }
      else {
        m11 /= m01;
        m01 = 1.0f / sqrtf(1.0f + sqr(m11));
        m11 *= m01;
      }
      v1 = m11 * U - m01 * V;
    }
    else {
      v1 = U;
    }
  }

  /* Compute third eigenvector. */
  if (half_det >= 0.0f) {
    v0 = cross(v1, v2);
  }
  else {
    v2 = cross(v0, v1);
  }

  return max_element * eigvals;
}

CCL_NAMESPACE_END

#endif /* __UTIL_MATH_EIGENSOLVER_H__ */
