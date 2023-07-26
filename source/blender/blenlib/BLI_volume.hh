/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_cpp_type.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_parameter_pack_utils.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

namespace blender {

class ResourceScope;
namespace volume {
template<typename T> class Grid;
template<typename T> class MutableGrid;
}  // namespace volume

/* XXX OpenVDB expects some math functions on vector types. */
template<typename T, int Size> inline VecBase<T, Size> Abs(VecBase<T, Size> v)
{
  VecBase<T, Size> r;
  for (int i = 0; i < Size; i++) {
    r[i] = math::abs(v[i]);
  }
  return r;
}
/* Specialization: math::abs is not defined for unsigned types. */
template<int Size> inline VecBase<uint32_t, Size> Abs(VecBase<uint32_t, Size> v)
{
  return v;
}

namespace volume {

/* -------------------------------------------------------------------- */
/** \name Tree and Grid types for Blender CPP types
 * \{ */

#ifdef WITH_OPENVDB

namespace grid_types {

template<typename ValueType>
using TreeCommon = typename openvdb::tree::Tree4<ValueType, 5, 4, 3>::Type;
template<typename ValueType> using GridCommon = typename openvdb::Grid<TreeCommon<ValueType>>;

/* TODO add more as needed. */
/* TODO could use template magic to generate all from 1 list, but not worth it for now. */
/* TODO some types disabled because of missing CPPType registration. */

using BoolTree = TreeCommon<bool>;
using MaskTree = TreeCommon<openvdb::ValueMask>;
using FloatTree = TreeCommon<float>;
using Float2Tree = TreeCommon<float2>;
using Float3Tree = TreeCommon<float3>;
// using Float4Tree = TreeCommon<float4>;
using IntTree = TreeCommon<int32_t>;
using Int2Tree = TreeCommon<int2>;
// using Int3Tree = TreeCommon<int3>;
// using Int4Tree = TreeCommon<int4>;
using UIntTree = TreeCommon<uint32_t>;
// using UInt2Tree = TreeCommon<uint2>;
// using UInt3Tree = TreeCommon<uint3>;
// using UInt4Tree = TreeCommon<uint4>;
using ScalarTree = FloatTree;
using TopologyTree = MaskTree;

using BoolGrid = openvdb::Grid<BoolTree>;
using MaskGrid = openvdb::Grid<MaskTree>;
using FloatGrid = openvdb::Grid<FloatTree>;
using Float2Grid = openvdb::Grid<Float2Tree>;
using Float3Grid = openvdb::Grid<Float3Tree>;
// using Float4Grid = openvdb::Grid<Float4Tree>;
using IntGrid = openvdb::Grid<IntTree>;
using Int2Grid = openvdb::Grid<Int2Tree>;
// using Int3Grid = openvdb::Grid<Int3Tree>;
// using Int4Grid = openvdb::Grid<Int4Tree>;
using UIntGrid = openvdb::Grid<UIntTree>;
// using UInt2Grid = openvdb::Grid<UInt2Tree>;
// using UInt3Grid = openvdb::Grid<UInt3Tree>;
// using UInt4Grid = openvdb::Grid<UInt4Tree>;
using ScalarGrid = openvdb::Grid<ScalarTree>;
using TopologyGrid = openvdb::Grid<TopologyTree>;

using SupportedGridValueTypes = std::tuple<bool,
                                           float,
                                           float2,
                                           float3,
                                           /*float4,*/
                                           int32_t,
                                           int2,
                                           /*int3,*/ /*int4,*/ uint32_t
                                           /*,uint2*/
                                           /*,uint3*/
                                           /*,uint4*/>;

using SupportedGridTypes = openvdb::TypeList<BoolGrid,
                                             MaskGrid,
                                             FloatGrid,
                                             Float2Grid,
                                             Float3Grid,
                                             /*Float4Grid,*/
                                             IntGrid,
                                             Int2Grid,
                                             /*Int3Grid,*/
                                             /*Int4Grid,*/
                                             UIntGrid,
                                             /*UInt2Grid,*/
                                             /*UInt3Grid,*/
                                             /*UInt4Grid,*/
                                             ScalarGrid,
                                             TopologyGrid>;

}  // namespace grid_types

#endif

/** \} */

/* Mask defined by active voxels of the grid. */
class GridMask {
#ifdef WITH_OPENVDB
  static const openvdb::MaskGrid::ConstPtr empty_grid_;
  openvdb::MaskGrid::ConstPtr grid_;
#endif

 public:
  GridMask()
#ifdef WITH_OPENVDB
      : grid_(empty_grid_)
#endif
  {
  }

  GridMask(const GridMask &other) = default;
  GridMask &operator=(const GridMask &other)
  {
#ifdef WITH_OPENVDB
    grid_ = other.grid_;
#else
    UNUSED_VARS(other);
#endif
    return *this;
  }

#ifdef WITH_OPENVDB
  GridMask(const openvdb::MaskGrid::ConstPtr &grid) : grid_(grid) {}
#endif

  static GridMask from_bools(const volume::GridMask &full_mask,
                             const volume::Grid<bool> &selection);

  bool is_empty() const;
  int64_t min_voxel_count() const;

#ifdef WITH_OPENVDB
  const openvdb::MaskGrid::ConstPtr &grid() const
  {
    return grid_;
  }
#endif
};

/* -------------------------------------------------------------------- */
/** \name Grid pointer wrappers
 *  \note Using wrappers avoids checking for WITH_OPENVDB everywhere.
 * \{ */

/* Generic grid reference. */
class GGrid {
 public:
#ifdef WITH_OPENVDB
  openvdb::GridBase::ConstPtr grid_ = nullptr;
#endif

  int64_t voxel_count() const;
  bool is_empty() const;
  operator bool() const;

  const CPPType *value_type() const;

  template<typename T> Grid<T> typed() const;
};

/* Generic grid reference. */
class GMutableGrid {
 public:
#ifdef WITH_OPENVDB
  openvdb::GridBase::Ptr grid_ = nullptr;
#endif

  operator GGrid() const
  {
#ifdef WITH_OPENVDB
    return GGrid{grid_};
#endif
  }

  /* Create an empty grid with a background value. */
  static GMutableGrid create(ResourceScope &scope,
                             const CPPType &type,
                             const void *background_value);
  /* Create an empty grid with the type default as background value. */
  static GMutableGrid create(ResourceScope &scope, const CPPType &type);
  /* Create a grid with the active volume mask voxels. */
  static GMutableGrid create(ResourceScope &scope,
                             const CPPType &type,
                             const GridMask &mask,
                             const void *inactive_value,
                             const void *active_value);

  bool try_assign(const GGrid &other);
  bool try_copy_masked(const GGrid &other, const GridMask &selection);

  int64_t voxel_count() const;
  bool is_empty() const;
  operator bool() const;

  const CPPType *value_type() const;

  template<typename T> MutableGrid<T> typed() const;
};

template<typename T> class Grid {
 public:
  using ValueType = T;
#ifdef WITH_OPENVDB
  using GridType = grid_types::GridCommon<T>;
  using TreeType = typename GridType::TreeType;
  using GridPtr = typename GridType::Ptr;
  using GridConstPtr = typename GridType::ConstPtr;

  GridConstPtr grid_ = nullptr;
#endif

  operator GGrid();
  operator GGrid const() const;

  int64_t voxel_count() const;
  bool is_empty() const;
  operator bool() const;

  const CPPType *value_type() const;
};

template<typename T> class MutableGrid {
 public:
  using ValueType = T;
#ifdef WITH_OPENVDB
  using GridType = grid_types::GridCommon<T>;
  using TreeType = typename GridType::TreeType;
  using GridPtr = typename GridType::Ptr;
  using GridConstPtr = typename GridType::ConstPtr;

  GridPtr grid_ = nullptr;
#endif

  /* Create an empty grid with a background value. */
  static MutableGrid<T> create(ResourceScope &scope, const T &background_value);
  /* Create an empty grid with the type default as background value. */
  static MutableGrid<T> create(ResourceScope &scope);
  /* Create a grid with the active volume mask voxels. */
  static MutableGrid<T> create(ResourceScope &scope,
                               const GridMask &mask,
                               const T &inactive_value,
                               const T &active_value);

  operator GMutableGrid();
  operator GMutableGrid const() const;

  int64_t voxel_count() const;
  bool is_empty() const;
  operator bool() const;

  const CPPType *value_type() const;
};

template<typename T> Grid<T> GGrid::typed() const
{
#ifdef WITH_OPENVDB
  using GridType = typename Grid<T>::GridType;
  using GridPtr = typename Grid<T>::GridConstPtr;

  if (!grid_) {
    return {};
  }
  GridPtr typed_grid = openvdb::GridBase::grid<GridType>(grid_);
  if (!typed_grid) {
    return {};
  }
  return {typed_grid};
#else
  return {};
#endif
}

template<typename T> MutableGrid<T> GMutableGrid::typed() const
{
#ifdef WITH_OPENVDB
  using GridType = typename MutableGrid<T>::GridType;
  using GridPtr = typename MutableGrid<T>::GridPtr;

  if (!grid_) {
    return {};
  }
  GridPtr typed_grid = grid_->grid<GridType>();
  if (!typed_grid) {
    return {};
  }
  return {typed_grid};
#else
  return {};
#endif
}

/** \} */

#ifdef WITH_OPENVDB

namespace detail {

template<typename Func> struct FilterVoidOp {
  Func func;
  void operator()(TypeTag<void> /*type_tag*/) const {}
  template<typename T> void operator()(TypeTag<T> type_tag) const
  {
    func(type_tag);
  }
};

/* Helper function to turn a tuple into a parameter pack by means of the dummy argument. */
template<typename... Types, typename Func>
void field_to_static_type_resolve(std::tuple<Types...> /*dummy*/, const CPPType &type, Func func)
{
  FilterVoidOp<Func> wrapper{func};
  type.to_static_type_tag<Types...>(wrapper);
}

}  // namespace detail

/* Helper function to evaluate a function with a static field type. */
template<typename Func> void field_to_static_type(const CPPType &type, Func func)
{
  detail::field_to_static_type_resolve(grid_types::SupportedGridValueTypes(), type, func);
}

/* Helper function to evaluate a function with a static field type. */
template<typename Func> void grid_to_static_type(const openvdb::GridBase::Ptr &grid, Func func)
{
  grid->apply<grid_types::SupportedGridTypes>(func);
}

/* Helper function to evaluate a function with a static field type. */
template<typename Func>
void grid_to_static_type(const openvdb::GridBase::ConstPtr &grid, Func func)
{
  grid->apply<grid_types::SupportedGridTypes>(func);
}

#endif

}  // namespace volume

}  // namespace blender
