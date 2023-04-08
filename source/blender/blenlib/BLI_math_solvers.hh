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
 * The Original Code is Copyright (C) 2015 by Blender Foundation
 * All rights reserved.
 */

#pragma once

/** \file
 * \ingroup bli
 */

#ifdef BLI_MATH_GCC_WARN_PRAGMA
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wredundant-decls"
#endif

/**
 * Evaluate the quadratic function at x.
 *
 * The quadratic function is a polynomial function on the form
 *	f(x) = ax^2 + bx + c
 */
template<typename FP> FP quadratic(FP a, FP b, FP c, FP x);

/**
 * Evaluate the cubic function at x.
 *
 * The cubic function is a polynomial function on the form
 *	f(x) = ax^3 + bx^2 + cx + d
 */
template<typename FP> FP cubic(FP a, FP b, FP c, FP d, FP x);

/**
 * Compute the real roots for the quadratic equation
 *
 *		ax^2 + bx + c = 0
 */
template<typename FP> int quadratic_roots(FP a, FP b, FP c, FP *xroots);

/**
 * Compute the real roots for the cubic equation
 *
 *		ax^3 + bx^2 + cx + d = 0
 */
template<typename FP> int cubic_roots(FP a, FP b, FP c, FP d, FP *xroots);

#include "intern/math_solvers.ipp"

#ifdef BLI_MATH_GCC_WARN_PRAGMA
#  pragma GCC diagnostic pop
#endif
