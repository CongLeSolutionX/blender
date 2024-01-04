/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "MEM_guardedalloc.h"

#include "BKE_node.hh"
#include "BKE_node_socket_value.hh"

#include "NOD_geometry_exec.hh"
#include "NOD_register.hh"
#include "NOD_socket_declarations.hh"
#include "NOD_socket_declarations_geometry.hh"

#include "node_util.hh"

struct BVHTreeFromMesh;
struct GeometrySet;
namespace blender::nodes {
class GatherAddNodeSearchParams;
class GatherLinkSearchOpParams;
}  // namespace blender::nodes

void geo_node_type_base(bNodeType *ntype, int type, const char *name, short nclass);
bool geo_node_poll_default(const bNodeType *ntype,
                           const bNodeTree *ntree,
                           const char **r_disabled_hint);

namespace blender::nodes {

bool check_tool_context_and_error(GeoNodeExecParams &params);
void search_link_ops_for_tool_node(GatherLinkSearchOpParams &params);

void transform_mesh(Mesh &mesh, float3 translation, math::Quaternion rotation, float3 scale);

void transform_geometry_set(GeoNodeExecParams &params,
                            GeometrySet &geometry,
                            const float4x4 &transform,
                            const Depsgraph &depsgraph);

/**
 * Returns the parts of the geometry that are on the selection for the given domain. If the domain
 * is not applicable for the component, e.g. face domain for point cloud, nothing happens to that
 * component. If no component can work with the domain, then `error_message` is set to true.
 */
void separate_geometry(GeometrySet &geometry_set,
                       AttrDomain domain,
                       GeometryNodeDeleteGeometryMode mode,
                       const Field<bool> &selection_field,
                       const AnonymousAttributePropagationInfo &propagation_info,
                       bool &r_is_error);

void get_closest_in_bvhtree(BVHTreeFromMesh &tree_data,
                            const VArray<float3> &positions,
                            const IndexMask &mask,
                            MutableSpan<int> r_indices,
                            MutableSpan<float> r_distances_sq,
                            MutableSpan<float3> r_positions);

int apply_offset_in_cyclic_range(IndexRange range, int start_index, int offset);

class EvaluateAtIndexInput final : public bke::GeometryFieldInput {
 private:
  Field<int> index_field_;
  GField value_field_;
  AttrDomain value_field_domain_;

 public:
  EvaluateAtIndexInput(Field<int> index_field, GField value_field, AttrDomain value_field_domain);

  GVArray get_varray_for_context(const bke::GeometryFieldContext &context,
                                 const IndexMask &mask) const final;

  std::optional<AttrDomain> preferred_domain(const GeometryComponent & /*component*/) const final
  {
    return value_field_domain_;
  }
};

class EvaluateOnDomainInput final : public bke::GeometryFieldInput {
 private:
  GField src_field_;
  AttrDomain src_domain_;

 public:
  EvaluateOnDomainInput(GField field, AttrDomain domain);

  GVArray get_varray_for_context(const bke::GeometryFieldContext &context,
                                 const IndexMask & /*mask*/) const final;
  void for_each_field_input_recursive(FunctionRef<void(const FieldInput &)> fn) const override;

  std::optional<AttrDomain> preferred_domain(
      const GeometryComponent & /*component*/) const override;
};

const CPPType &get_simulation_item_cpp_type(eNodeSocketDatatype socket_type);
const CPPType &get_simulation_item_cpp_type(const NodeSimulationItem &item);

bke::bake::BakeState move_values_to_simulation_state(
    Span<NodeSimulationItem> node_simulation_items, Span<void *> input_values);
void move_simulation_state_to_values(Span<NodeSimulationItem> node_simulation_items,
                                     bke::bake::BakeState zone_state,
                                     const Object &self_object,
                                     const ComputeContext &compute_context,
                                     const bNode &sim_output_node,
                                     Span<void *> r_output_values);
void copy_simulation_state_to_values(Span<NodeSimulationItem> node_simulation_items,
                                     const bke::bake::BakeStateRef &zone_state,
                                     const Object &self_object,
                                     const ComputeContext &compute_context,
                                     const bNode &sim_output_node,
                                     Span<void *> r_output_values);

void copy_with_checked_indices(const GVArray &src,
                               const VArray<int> &indices,
                               const IndexMask &mask,
                               GMutableSpan dst);

void mix_baked_data_item(eNodeSocketDatatype socket_type,
                         void *prev,
                         const void *next,
                         const float factor);

namespace enums {

const EnumPropertyItem *attribute_type_type_with_socket_fn(bContext * /*C*/,
                                                           PointerRNA * /*ptr*/,
                                                           PropertyRNA * /*prop*/,
                                                           bool *r_free);

bool generic_attribute_type_supported(const EnumPropertyItem &item);

const EnumPropertyItem *domain_experimental_grease_pencil_version3_fn(bContext * /*C*/,
                                                                      PointerRNA * /*ptr*/,
                                                                      PropertyRNA * /*prop*/,
                                                                      bool *r_free);

const EnumPropertyItem *domain_without_corner_experimental_grease_pencil_version3_fn(
    bContext * /*C*/, PointerRNA * /*ptr*/, PropertyRNA * /*prop*/, bool *r_free);

}  // namespace enums

bool grid_type_supported(eCustomDataType data_type);
bool grid_type_supported(eNodeSocketDatatype socket_type);
const EnumPropertyItem *grid_custom_data_type_items_filter_fn(bContext *C,
                                                              PointerRNA *ptr,
                                                              PropertyRNA *prop,
                                                              bool *r_free);
const EnumPropertyItem *grid_socket_type_items_filter_fn(bContext *C,
                                                         PointerRNA *ptr,
                                                         PropertyRNA *prop,
                                                         bool *r_free);

void node_geo_exec_with_missing_openvdb(GeoNodeExecParams &params);

namespace grids {

BaseSocketDeclarationBuilder &declare_grid_type_input(NodeDeclarationBuilder &b,
                                                      eCustomDataType type,
                                                      StringRef name,
                                                      StringRef identifier = "");
BaseSocketDeclarationBuilder &declare_grid_type_output(NodeDeclarationBuilder &b,
                                                       eCustomDataType type,
                                                       StringRef name,
                                                       StringRef identifier = "");

bke::GVolumeGridPtr extract_grid_input(GeoNodeExecParams params,
                                       StringRef identifier,
                                       const eCustomDataType data_type);

template<typename T>
bke::VolumeGridPtr<T> extract_grid_input(GeoNodeExecParams params, StringRef identifier)
{
  const CPPType &cpp_type = CPPType::get<T>();
  const eCustomDataType data_type = bke::cpp_type_to_custom_data_type(cpp_type);
  return extract_grid_input(params, identifier, data_type).typed<T>();
}

/* Does not currently work, requires a CPPType for Vector<ValueOrField<T>>. */
#if 0
Vector<bke::GVolumeGridPtr> extract_grid_multi_input(GeoNodeExecParams params,
                                                     StringRef identifier,
                                                     eCustomDataType data_type);
#endif

void set_output_grid(GeoNodeExecParams params,
                     StringRef identifier,
                     eCustomDataType data_type,
                     const bke::GVolumeGridPtr &grid);

template<typename T>
void set_output_grid(GeoNodeExecParams params,
                     StringRef identifier,
                     const bke::VolumeGridPtr<T> &grid)
{
  const CPPType &cpp_type = CPPType::get<T>();
  const eCustomDataType data_type = bke::cpp_type_to_custom_data_type(cpp_type);
  set_output_grid(params, identifier, data_type, grid);
}

template<typename OpT> auto apply(const eCustomDataType data_type, OpT &op)
{
  switch (data_type) {
    case CD_PROP_FLOAT:
      return op.template operator()<float>();
    case CD_PROP_FLOAT3:
      return op.template operator()<float3>();
    default:
      BLI_assert_unreachable();
      break;
  }
  BLI_assert_unreachable();
  /* Dummy output value for compiler, should never get here. */
  return op.template operator()<float>();
}

#ifdef WITH_OPENVDB
openvdb::tools::NearestNeighbors get_vdb_neighbors_mode(
    GeometryNodeGridNeighborTopology neighbors_mode);
#endif

bke::GVolumeGridPtr try_capture_field_as_grid(const eCustomDataType data_type,
                                              eCustomDataType topology_data_type,
                                              const bke::GVolumeGridPtr &topology_grid,
                                              fn::GField value_field,
                                              GPointer background);

}  // namespace grids

}  // namespace blender::nodes
