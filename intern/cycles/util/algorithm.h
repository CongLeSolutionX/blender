/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#ifndef __UTIL_ALGORITHM_H__
#define __UTIL_ALGORITHM_H__

#ifndef __KERNEL_GPU__
#  include <algorithm>
#endif

CCL_NAMESPACE_BEGIN

#ifdef __KERNEL_GPU__

template<typename T> void swap(T &a, T &b)
{
  T tmp(a);
  a = b;
  b = tmp;
}

#else

using std::remove;
using std::sort;
using std::stable_sort;
using std::swap;

#endif

CCL_NAMESPACE_END

#endif /* __UTIL_ALGORITHM_H__ */
