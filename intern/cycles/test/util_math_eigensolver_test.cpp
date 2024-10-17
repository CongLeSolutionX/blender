/* SPDX-FileCopyrightText: 2011-2024 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "util/math.h"
#include "util/math_eigensolver.h"

CCL_NAMESPACE_BEGIN

#define EXPECT_NEAR_VEC3(vec, vx, vy, vz, tol) \
  EXPECT_NEAR((vec).x, vx, tol); \
  EXPECT_NEAR((vec).y, vy, tol); \
  EXPECT_NEAR((vec).z, vz, tol)

TEST(math, eigendecomposition_3x3_symmetric)
{
  float3 v1, v2, v3, eig;

  /* Test general case. */
  eig = eigendecomposition_3x3_symmetric(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, v1, v2, v3);
  /* Check eigenvalues */
  EXPECT_NEAR_VEC3(eig, -0.51572947f, 0.17091518f, 11.3448142f, 1e-5f);
  /* Check eigenvectors */
  const float a = 0.32798527f, b = 0.59100904f, c = 0.73697622f;
  EXPECT_NEAR_VEC3(v1, -c, -a, b, 1e-5f);
  EXPECT_NEAR_VEC3(v2, b, -c, a, 1e-5f);
  EXPECT_NEAR_VEC3(v3, a, b, c, 1e-5f);

  /* Test diagonal matrix. */
  eig = eigendecomposition_3x3_symmetric(3.0f, 0.0f, 0.0f, 1.0f, 0.0f, 2.0f, v1, v2, v3);
  /* Check eigenvalues */
  EXPECT_NEAR_VEC3(eig, 1.0f, 2.0f, 3.0f, 1e-5f);
  /* Check eigenvectors */
  EXPECT_NEAR_VEC3(v1, 0.0f, 1.0f, 0.0f, 1e-5f);
  EXPECT_NEAR_VEC3(v2, 0.0f, 0.0f, 1.0f, 1e-5f);
  EXPECT_NEAR_VEC3(v3, 1.0f, 0.0f, 0.0f, 1e-5f);
}

CCL_NAMESPACE_END
