/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"
#include "node_util.hh"

#include "DNA_space_types.h"

#include "BKE_context.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_runtime.hh"
#include "BKE_node.hh"
#include "BKE_pointcloud.hh"
#include "BKE_volume_grid.hh"

#include "NOD_rna_define.hh"
#include "NOD_socket_search_link.hh"

#include "RNA_enum_types.hh"

namespace blender::nodes {

bool check_tool_context_and_error(GeoNodeExecParams &params)
{
  if (!params.user_data()->call_data->operator_data) {
    params.error_message_add(NodeWarningType::Error, TIP_("Node must be run as tool"));
    params.set_default_remaining_outputs();
    return false;
  }
  return true;
}

void search_link_ops_for_tool_node(GatherLinkSearchOpParams &params)
{
  if (params.space_node().geometry_nodes_type == SNODE_GEOMETRY_TOOL) {
    search_link_ops_for_basic_node(params);
  }
}

namespace enums {

const EnumPropertyItem *attribute_type_type_with_socket_fn(bContext * /*C*/,
                                                           PointerRNA * /*ptr*/,
                                                           PropertyRNA * /*prop*/,
                                                           bool *r_free)
{
  *r_free = true;
  return enum_items_filter(rna_enum_attribute_type_items,
                           [](const EnumPropertyItem &item) -> bool {
                             return generic_attribute_type_supported(item) &&
                                    !ELEM(item.value, CD_PROP_BYTE_COLOR, CD_PROP_FLOAT2);
                           });
}

bool generic_attribute_type_supported(const EnumPropertyItem &item)
{
  return ELEM(item.value,
              CD_PROP_FLOAT,
              CD_PROP_FLOAT2,
              CD_PROP_FLOAT3,
              CD_PROP_COLOR,
              CD_PROP_BOOL,
              CD_PROP_INT32,
              CD_PROP_BYTE_COLOR,
              CD_PROP_QUATERNION);
}

const EnumPropertyItem *domain_experimental_grease_pencil_version3_fn(bContext * /*C*/,
                                                                      PointerRNA * /*ptr*/,
                                                                      PropertyRNA * /*prop*/,
                                                                      bool *r_free)
{
  *r_free = true;
  return enum_items_filter(rna_enum_attribute_domain_items,
                           [](const EnumPropertyItem &item) -> bool {
                             return (bke::AttrDomain(item.value) == bke::AttrDomain::Layer) ?
                                        U.experimental.use_grease_pencil_version3 :
                                        true;
                           });
}

const EnumPropertyItem *domain_without_corner_experimental_grease_pencil_version3_fn(
    bContext * /*C*/, PointerRNA * /*ptr*/, PropertyRNA * /*prop*/, bool *r_free)
{
  *r_free = true;
  return enum_items_filter(rna_enum_attribute_domain_without_corner_items,
                           [](const EnumPropertyItem &item) -> bool {
                             return (bke::AttrDomain(item.value) == bke::AttrDomain::Layer) ?
                                        U.experimental.use_grease_pencil_version3 :
                                        true;
                           });
}

}  // namespace enums

bool grid_type_supported(const eCustomDataType data_type)
{
  return ELEM(data_type, CD_PROP_FLOAT, CD_PROP_FLOAT3);
}

bool grid_type_supported(eNodeSocketDatatype socket_type)
{
  if (const std::optional<eCustomDataType> data_type = bke::socket_type_to_custom_data_type(
          socket_type))
  {
    return grid_type_supported(*data_type);
  }
  return false;
}

const EnumPropertyItem *grid_custom_data_type_items_filter_fn(bContext * /*C*/,
                                                              PointerRNA * /*ptr*/,
                                                              PropertyRNA * /*prop*/,
                                                              bool *r_free)
{
  *r_free = true;
  return enum_items_filter(rna_enum_attribute_type_items,
                           [](const EnumPropertyItem &item) -> bool {
                             return grid_type_supported(eCustomDataType(item.value));
                           });
}

const EnumPropertyItem *grid_socket_type_items_filter_fn(bContext * /*C*/,
                                                         PointerRNA * /*ptr*/,
                                                         PropertyRNA * /*prop*/,
                                                         bool *r_free)
{
  *r_free = true;
  return enum_items_filter(rna_enum_node_socket_data_type_items,
                           [](const EnumPropertyItem &item) -> bool {
                             return grid_type_supported(eNodeSocketDatatype(item.value));
                           });
}

void node_geo_exec_with_missing_openvdb(GeoNodeExecParams &params)
{
  params.set_default_remaining_outputs();
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without OpenVDB"));
}

namespace grids {

BaseSocketDeclarationBuilder &declare_grid_type_input(NodeDeclarationBuilder &b,
                                                      const eCustomDataType type,
                                                      const StringRef name,
                                                      const StringRef identifier)
{
  switch (type) {
    case CD_PROP_FLOAT:
      return b.add_input<decl::Float>(name, identifier).hide_value();
    case CD_PROP_FLOAT3:
      return b.add_input<decl::Vector>(name, identifier).hide_value();
    default:
      break;
  }
  BLI_assert_unreachable();
  return b.add_input<decl::Float>(name, identifier);
}

BaseSocketDeclarationBuilder &declare_grid_type_output(NodeDeclarationBuilder &b,
                                                       const eCustomDataType type,
                                                       const StringRef name,
                                                       const StringRef identifier)
{
  switch (type) {
    case CD_PROP_FLOAT:
      return b.add_output<decl::Float>(name, identifier);
    case CD_PROP_FLOAT3:
      return b.add_output<decl::Vector>(name, identifier);
    default:
      break;
  }
  BLI_assert_unreachable();
  return b.add_output<decl::Float>(name, identifier);
}

template<typename GridType> static int64_t get_voxel_count(const GridType &grid)
{
  return grid.tree().activeLeafVoxelCount();
}

template<typename GridType>
static void get_voxel_positions_span(GridType &grid, const MutableSpan<float3> positions)
{
  using TreeType = typename GridType::TreeType;
  using LeafManager = openvdb::tree::LeafManager<TreeType>;
  using LeafNodeType = typename TreeType::LeafNodeType;

  LeafManager leaf_mgr(grid.tree());
  /* XXX calculated twice (see store_voxel_values), can be done externally. */
  size_t *leaf_offsets = static_cast<size_t *>(
      MEM_malloc_arrayN(leaf_mgr.leafCount(), sizeof(size_t), __func__));
  size_t leaf_offsets_size = leaf_mgr.leafCount();
  leaf_mgr.getPrefixSum(leaf_offsets, leaf_offsets_size);

  const openvdb::math::Transform &transform = grid.transform();

  leaf_mgr.foreach ([&](const LeafNodeType &leaf, const size_t leaf_index) {
    int64_t index = leaf_offsets[leaf_index];
    typename LeafNodeType::ValueOnCIter iter = leaf.cbeginValueOn();
    for (; iter; ++iter, ++index) {
      const openvdb::Vec3d pos = transform.indexToWorld(iter.getCoord());
      positions[index] = float3(pos.x(), pos.y(), pos.z());
    }
  });

  MEM_delete(leaf_offsets);
}

template<typename T, typename GridType>
static void store_voxel_values(GridType &grid, const Span<T> values)
{
  using TreeType = typename GridType::TreeType;
  using LeafManager = openvdb::tree::LeafManager<TreeType>;
  using LeafNodeType = typename TreeType::LeafNodeType;
  using Converter = bke::grids::Converter<T>;

  LeafManager leaf_mgr(grid.tree());
  size_t *leaf_offsets = static_cast<size_t *>(
      MEM_malloc_arrayN(leaf_mgr.leafCount(), sizeof(size_t), __func__));
  size_t leaf_offsets_size = leaf_mgr.leafCount();
  leaf_mgr.getPrefixSum(leaf_offsets, leaf_offsets_size);

  leaf_mgr.foreach ([&](LeafNodeType &leaf, const size_t leaf_index) {
    int64_t index = leaf_offsets[leaf_index];
    typename LeafNodeType::ValueOnIter iter = leaf.beginValueOn();
    for (; iter; ++iter, ++index) {
      iter.setValue(Converter::to_openvdb(values[index]));
    }
  });

  MEM_delete(leaf_offsets);
}

template<typename GridType> class CaptureFieldContext : public FieldContext {
 private:
  typename GridType::Ptr grid_;

 public:
  CaptureFieldContext(typename GridType::Ptr grid) : grid_(std::move(grid)) {}

  GVArray get_varray_for_input(const fn::FieldInput &field_input,
                               const IndexMask & /*mask*/,
                               ResourceScope & /*scope*/) const override
  {
    const bke::AttributeFieldInput *attribute_field_input =
        dynamic_cast<const bke::AttributeFieldInput *>(&field_input);
    if (attribute_field_input == nullptr) {
      return {};
    }
    if (attribute_field_input->attribute_name() != "position") {
      return {};
    }

    Array<float3> positions(get_voxel_count(*grid_));
    get_voxel_positions_span(*grid_, positions);
    return VArray<float3>::ForContainer(std::move(positions));
  }
};

template<typename OutputGridPtr> struct TopologyInitOp {
  bke::GVolumeGrid topology_grid;
  OutputGridPtr output_grid;

  template<typename T> void operator()()
  {
    if (!this->topology_grid) {
      /* TODO should use topology union of inputs in this case. */
      return;
    }
    bke::VolumeGrid<T> typed_topology_grid = this->topology_grid.typed<T>();
    bke::VolumeTreeAccessToken tree_token;
    const bke::OpenvdbGridType<T> &vdb_grid = typed_topology_grid.grid(tree_token);

    output_grid->setTransform(vdb_grid.transform().copy());
    output_grid->insertMeta(vdb_grid);
    output_grid->topologyUnion(vdb_grid);
  }
};

struct CaptureGridOp {
  const eCustomDataType topology_data_type;
  bke::GVolumeGrid topology_grid;
  fn::GField value_field;
  GPointer background;

  template<typename T> bke::GVolumeGrid operator()()
  {
    using GridType = typename bke::OpenvdbGridType<T>;
    using GridPtr = std::shared_ptr<GridType>;
    using Converter = bke::grids::Converter<T>;

    const typename GridType::ValueType vdb_background = Converter::to_openvdb(
        *this->background.get<T>());

    /* Evaluate value field and fill in the grid. */
    const GridPtr output_grid = GridType::create(vdb_background);
    TopologyInitOp<GridPtr> topology_op{topology_grid, output_grid};
    grids::apply(this->topology_data_type, topology_op);
    /* All leaf nodes must be allocated for writing voxel values. */
    output_grid->tree().voxelizeActiveTiles();

    const int64_t voxels_num = get_voxel_count(*output_grid);
    CaptureFieldContext<GridType> context(output_grid);
    fn::FieldEvaluator evaluator(context, voxels_num);
    Array<T> values(voxels_num);
    evaluator.add_with_destination(this->value_field, values.as_mutable_span());
    evaluator.evaluate();
    store_voxel_values(*output_grid, values.as_span());

    return bke::GVolumeGrid(std::move(output_grid));
  }
};

bke::GVolumeGrid try_capture_field_as_grid(const eCustomDataType data_type,
                                           const eCustomDataType topology_data_type,
                                           const bke::GVolumeGrid &topology_grid,
                                           const fn::GField value_field,
                                           const GPointer background)
{
  CaptureGridOp capture_op = {topology_data_type, topology_grid, value_field, background};
  return grids::apply(data_type, capture_op);
}

}  // namespace grids

}  // namespace blender::nodes

bool geo_node_poll_default(const bNodeType * /*ntype*/,
                           const bNodeTree *ntree,
                           const char **r_disabled_hint)
{
  if (!STREQ(ntree->idname, "GeometryNodeTree")) {
    *r_disabled_hint = RPT_("Not a geometry node tree");
    return false;
  }
  return true;
}

void geo_node_type_base(bNodeType *ntype, int type, const char *name, short nclass)
{
  blender::bke::node_type_base(ntype, type, name, nclass);
  ntype->poll = geo_node_poll_default;
  ntype->insert_link = node_insert_link_default;
  ntype->gather_link_search_ops = blender::nodes::search_link_ops_for_basic_node;
}
