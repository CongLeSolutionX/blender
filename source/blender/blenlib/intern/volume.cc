/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 */

#include "BLI_cpp_type.hh"
#include "BLI_math_base.hh"
#include "BLI_resource_scope.hh"
#include "BLI_volume_openvdb.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/tools/ValueTransformer.h>
#endif

namespace blender::volume {

#ifdef WITH_OPENVDB

openvdb::GridBase &make_grid_for_attribute_type(ResourceScope &scope, const CPPType &type)
{
  openvdb::GridBase *result = nullptr;
  volume::field_to_static_type(type, [&](auto tag) {
    using ValueType = typename decltype(tag)::type;
    using GridType = volume::grid_types::AttributeGrid<ValueType>;
    using Converter = volume::grid_types::Converter<GridType>;

    const ValueType &background_value = *static_cast<const ValueType *>(type.default_value());
    result = &scope.construct<GridType>(Converter::single_value_to_grid(background_value));
  });
  BLI_assert(result != nullptr);
  return *result;
}

openvdb::GridBase *make_grid_for_attribute_type(const CPPType &type)
{
  openvdb::GridBase *result = nullptr;
  volume::field_to_static_type(type, [&](auto tag) {
    using ValueType = typename decltype(tag)::type;
    using GridType = volume::grid_types::AttributeGrid<ValueType>;
    using Converter = volume::grid_types::Converter<GridType>;

    const ValueType &background_value = *static_cast<const ValueType *>(type.default_value());
    result = MEM_new<GridType>(__func__, Converter::single_value_to_grid(background_value));
  });
  BLI_assert(result != nullptr);
  return result;
}

const CPPType &grid_base_attribute_type(const openvdb::GridBase &grid)
{
  const CPPType *type = &CPPType::get<float>();
  volume::grid_to_static_type(grid,
                              [&](auto &typed_grid) { type = &grid_attribute_type(typed_grid); });
  return *type;
}

// int64_t GVGrid::voxel_count() const
//{
//   return grid_ ? grid_->activeVoxelCount() : 0;
// }
//
// bool GVGrid::is_empty() const
//{
//   return grid_ ? grid_->empty() : true;
// }
//
// GVGrid::operator bool() const
//{
//   return grid_ != nullptr;
// }
//
// const CPPType *GVGrid::value_type() const
//{
//   const CPPType *type = nullptr;
//   grid_to_static_type(grid_, [&](auto &grid) {
//     using GridType = typename std::decay<decltype(grid)>::type;
//     using Converter = grid_types::Converter<GridType>;
//
//     type = &CPPType::get<typename Converter::AttributeValueType>();
//   });
//   return type;
// }
//
// GVMutableGrid GVMutableGrid::create(const CPPType &type, const void *background_value)
//{
//   openvdb::GridBase::Ptr grid;
//   volume::field_to_static_type(type, [&grid, background_value](auto type_tag) {
//     using T = typename decltype(type_tag)::type;
//     using GridType = grid_types::AttributeGrid<T>;
//     using Converter = grid_types::Converter<GridType>;
//
//     const T &value = *static_cast<const T *>(background_value);
//     grid = grid_types::AttributeGrid<T>::create(Converter::single_value_to_grid(value));
//   });
//
//   return GVMutableGrid{std::move(grid)};
// }
//
// GVMutableGrid GVMutableGrid::create(const CPPType &type)
//{
//   openvdb::GridBase::Ptr grid;
//   volume::field_to_static_type(type, [&](auto type_tag) {
//     using T = typename decltype(type_tag)::type;
//     using GridType = grid_types::AttributeGrid<T>;
//     using Converter = grid_types::Converter<GridType>;
//
//     const T &value = *static_cast<const T *>(type.default_value());
//     grid = grid_types::AttributeGrid<T>::create(Converter::single_value_to_grid(value));
//   });
//
//   return GVMutableGrid{std::move(grid)};
// }
//
// GVMutableGrid GVMutableGrid::create(const CPPType &type,
//                                     const GVGrid &mask,
//                                     const void *inactive_value,
//                                     const void *active_value)
//{
//   openvdb::GridBase::Ptr grid = nullptr;
//   volume::field_to_static_type(type, [&](auto type_tag) {
//     using T = typename decltype(type_tag)::type;
//     using TreeType = grid_types::AttributeTree<T>;
//     using GridType = grid_types::AttributeGrid<T>;
//     using Converter = grid_types::Converter<GridType>;
//
//     if (mask.is_empty()) {
//       grid = GridType::create();
//       return;
//     }
//
//     const T &typed_inactive_value = *static_cast<const T *>(inactive_value);
//     const T &typed_active_value = *static_cast<const T *>(active_value);
//     typename TreeType::Ptr tree = nullptr;
//     volume::grid_to_static_type(mask.grid_, [&](auto &typed_mask) {
//       tree = typename TreeType::Ptr(
//           new TreeType(typed_mask.tree(),
//                        Converter::single_value_to_grid(typed_inactive_value),
//                        Converter::single_value_to_grid(typed_active_value),
//                        openvdb::TopologyCopy{}));
//     });
//     grid = typename GridType::Ptr(new GridType(tree));
//     grid->setTransform(mask.grid_->transform().copy());
//   });
//
//   return GVMutableGrid{std::move(grid)};
// }
//
// bool GVMutableGrid::try_copy_masked(const GVGrid &other, const GVGrid & /*mask*/)
//{
//   if (!grid_ || !other.grid_) {
//     return false;
//   }
//   *grid_ = *other.grid_->copyGridWithNewTree();
//   /* XXX TODO prune tree with mask */
//   return true;
// }
//
// int64_t GVMutableGrid::voxel_count() const
//{
//   return grid_ ? grid_->activeVoxelCount() : 0;
// }
//
// bool GVMutableGrid::is_empty() const
//{
//   return grid_ ? grid_->empty() : true;
// }
//
// GVMutableGrid::operator bool() const
//{
//   return grid_ != nullptr;
// }
//
// const CPPType *GVMutableGrid::value_type() const
//{
//   const CPPType *type = nullptr;
//   grid_to_static_type(grid_, [&](auto &grid) {
//     using GridType = typename std::decay<decltype(grid)>::type;
//     using Converter = grid_types::Converter<GridType>;
//
//     type = &CPPType::get<typename Converter::AttributeValueType>();
//   });
//   return type;
// }
//
// #else
//
// GGrid::operator bool() const
//{
//   return false;
// }
//
// int64_t GGrid::voxel_count() const
//{
//   return 0;
// }
//
// bool GGrid::is_empty() const
//{
//   return true;
// }
//
// const CPPType *GGrid::value_type() const
//{
//   return nullptr;
// }
//
// GGrid GGrid::create(ResourceScope & /*scope*/,
//                     const CPPType & /*type*/,
//                     const void * /*background_value*/)
//{
//   return GGrid{};
// }
//
// GGrid GGrid::create(ResourceScope & /*scope*/, const CPPType & /*type*/)
//{
//   return GGrid{};
// }
//
// GGrid GGrid::create(ResourceScope & /*scope*/,
//                     const CPPType & /*type*/,
//                     const GridMask & /*mask*/,
//                     const void * /*inactive_value*/,
//                     const void * /*active_value*/)
//{
//   return GGrid{};
// }

#endif

}  // namespace blender::volume
