/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2015 by Blender Foundation.
 * All rights reserved.
 */

/** \file
 * \ingroup bli
 */
#include <float.h>
#include <math.h>
#include <type_traits>

#include "BLI_math_base.h"
#include "BLI_utildefines.h"

template<typename FP> FP quadratic(FP a, FP b, FP c, FP x)
{
  return a * x * x + b * x + c;
}

template<typename FP> FP cubic(FP a, FP b, FP c, FP d, FP x)
{
  FP xsq = x * x;
  return a * x * xsq + b * xsq + c * x + d;
}

/**
 * Implementation is based on https://people.eecs.berkeley.edu/~wkahan/Math128/Cubic.pdf.
 * 'To Solve a Real Cubic Equation' authored by W. Kahan.
 */
template<typename FP> inline int quadratic_roots(FP A, FP B, FP C, FP *xroots)
{
  constexpr FP EPSILON = std::is_same<float, typename std::remove_cv<FP>::type>::value ?
                             FLT_EPSILON :
                             DBL_EPSILON;

  if (fabs(A) < EPSILON) {
    /* Linear equation */
    if (fabs(B) > EPSILON) {
      *xroots = -C / B;
      return 1;
    }
    /* Constant*/
    return 0;
  }

  FP b = -B / (FP)2.0;
  FP q = b * b - A * C;
  if (q < (FP)0.0) {
    return 0;
    /* Complex roots
    X1 = b / A;
    X2 = X1;
    Y1 = std::sqrt(-q) / A;
    Y2 = -Y1;
    */
  }
  else {
    FP r = b + copysign(sqrt(q), b); /* sqrt(q) * sign(b) as q >= 0 */
    if (r == (FP)0.0) {

      xroots[0] = C / A;
      xroots[1] = -xroots[0];
    }
    else {
      xroots[0] = C / r;
      xroots[1] = r / A;
    }
  }
  return 2;
}

template<typename FP>
inline void qbc_eval(FP X, FP A, FP B, FP C, FP D, FP &Q, FP &Q_p, FP &B1, FP &C2)
{
  FP q0 = A * X;
  B1 = q0 + B;
  C2 = B1 * X + C;
  Q_p = (q0 + B1) * X + C2;
  Q = C2 * X + D;
}

/**
 * QBC algorithm as described in https://people.eecs.berkeley.edu/~wkahan/Math128/Cubic.pdf.
 * 'To Solve a Real Cubic Equation' authored by W. Kahan.
 *
 * Note* implementation only return real roots.
 */
template<typename FP> int cubic_roots(FP A, FP B, FP C, FP D, FP *xroots)
{
  constexpr FP EPSILON = std::is_same<float, typename std::remove_cv<FP>::type>::value ?
                             FLT_EPSILON :
                             DBL_EPSILON;

  int N = 0;
  FP b1, c2;
  if (fabs(A) < EPSILON) {
    /* Quadratic equation */
    A = B;
    b1 = C;
    c2 = D;
    // *xroots++ == INFINITY;
  }
  else if (fabs(D) < EPSILON) {
    /* Convert to a quadratic equation (divide by x) */
    *xroots++ = (FP)0.0;
    b1 = B;
    c2 = C;
    N = 1;
  }
  else {
    FP X = -(B / A) / (FP)3.0;
    FP q, q_p, t, r, s;
    qbc_eval(X, A, B, C, D, q, q_p, b1, c2);

    t = q / A;
    r = cbrt(fabs(t));
    s = copysign((FP)1.0, t);

    t = -q_p / A;
    if (t > 0) {
      r = (FP)1.324717957244746025960908854478097340734404056901733365 * fmax(r, sqrt(t));
    }

    FP x0 = X - r * s;
    if (x0 != X) {
      do {
        X = x0;
        qbc_eval(X, A, B, C, D, q, q_p, b1, c2);
        if (q_p == 0) {
          x0 = X;
        }
        else {
          x0 = X - (q / q_p) / (FP)1.000000000000001; /* 1.000..001 */
        }
      } while (x0 * s > X * s);

      if (fabs(A) * X * X > fabs(D / X)) {
        c2 = -D / X;
        b1 = (c2 - C) / X;
      }
    }
    N = 1;
    *xroots++ = X;
  }
  return N + quadratic_roots(A, b1, c2, xroots);
}
