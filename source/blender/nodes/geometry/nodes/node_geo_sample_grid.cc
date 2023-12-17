/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_type_conversions.hh"
#include "BKE_volume.hh"
#include "BKE_volume_grid.hh"

#include "BLI_index_mask.hh"
#include "BLI_virtual_array.hh"

#include "node_geometry_util.hh"

#include "NOD_rna_define.hh"
#include "NOD_socket_search_link.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/tools/Interpolation.h>
#endif

namespace blender::nodes::node_geo_sample_grid_cc {

NODE_STORAGE_FUNCS(NodeGeometrySampleGrid)

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }
  const NodeGeometrySampleGrid &storage = node_storage(*node);
  const eCustomDataType data_type = eCustomDataType(storage.data_type);

  grids::declare_grid_type_input(b, data_type, "Grid");
  b.add_input<decl::Vector>("Position").implicit_field(implicit_field_inputs::position);

  switch (data_type) {
    case CD_PROP_FLOAT:
      b.add_output<decl::Float>("Value").dependent_field({1});
      break;
    case CD_PROP_FLOAT3:
      b.add_output<decl::Vector>("Value").dependent_field({1});
      break;
    case CD_PROP_BOOL:
      b.add_output<decl::Bool>("Value").dependent_field({1});
      break;
    case CD_PROP_INT32:
      b.add_output<decl::Int>("Value").dependent_field({1});
      break;
    default:
      BLI_assert_unreachable();
      break;
  }
}

static void search_link_ops(GatherLinkSearchOpParams &params)
{
  if (U.experimental.use_new_volume_nodes) {
    nodes::search_link_ops_for_basic_node(params);
  }
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "grid_type", UI_ITEM_NONE, "", ICON_NONE);
  uiItemR(layout, ptr, "interpolation_mode", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometrySampleGrid *data = MEM_cnew<NodeGeometrySampleGrid>(__func__);
  data->data_type = CD_PROP_FLOAT;
  data->interpolation_mode = GEO_NODE_SAMPLE_GRID_INTERPOLATION_MODE_TRILINEAR;
  node->storage = data;
}

#ifdef WITH_OPENVDB

template<typename T>
void sample_grid(const bke::OpenvdbGridType<T> &grid,
                 const Span<float3> positions,
                 const IndexMask &mask,
                 MutableSpan<T> dst,
                 const GeometryNodeSampleGridInterpolationMode interpolation_mode)
{
  using GridType = typename bke::OpenvdbGridType<T>;
  using GridValueT = typename GridType::ValueType;
  using AccessorT = typename GridType::ConstAccessor;
  using GridTraits = bke::VolumeGridTraits<T>;
  AccessorT accessor = grid.getConstAccessor();

  auto sample_data = [&](auto sampler) {
    mask.foreach_index([&](const int64_t i) {
      const float3 &pos = positions[i];
      GridValueT value = sampler.wsSample(openvdb::Vec3R(pos.x, pos.y, pos.z));
      dst[i] = GridTraits::to_blender(value);
    });
  };

  /* Use to the Nearest Neighbor sampler for Bool grids (no interpolation). */
  GeometryNodeSampleGridInterpolationMode real_interpolation_mode = interpolation_mode;
  if constexpr (std::is_same_v<T, bool>) {
    real_interpolation_mode = GEO_NODE_SAMPLE_GRID_INTERPOLATION_MODE_NEAREST;
  }
  switch (real_interpolation_mode) {
    case GEO_NODE_SAMPLE_GRID_INTERPOLATION_MODE_TRILINEAR: {
      openvdb::tools::GridSampler<AccessorT, openvdb::tools::BoxSampler> sampler(accessor,
                                                                                 grid.transform());
      sample_data(sampler);
      break;
    }
    case GEO_NODE_SAMPLE_GRID_INTERPOLATION_MODE_TRIQUADRATIC: {
      openvdb::tools::GridSampler<AccessorT, openvdb::tools::QuadraticSampler> sampler(
          accessor, grid.transform());
      sample_data(sampler);
      break;
    }
    case GEO_NODE_SAMPLE_GRID_INTERPOLATION_MODE_NEAREST:
    default: {
      openvdb::tools::GridSampler<AccessorT, openvdb::tools::PointSampler> sampler(
          accessor, grid.transform());
      sample_data(sampler);
      break;
    }
  }
}

template<typename T> class SampleGridFunction : public mf::MultiFunction {
  bke::VolumeGrid<T> volume_grid_;
  GeometryNodeSampleGridInterpolationMode interpolation_mode_;
  mf::Signature signature_;

 public:
  SampleGridFunction(bke::VolumeGrid<T> volume_grid,
                     GeometryNodeSampleGridInterpolationMode interpolation_mode)
      : volume_grid_(std::move(volume_grid)), interpolation_mode_(interpolation_mode)
  {
    BLI_assert(volume_grid_);

    const CPPType &cpp_type = CPPType::get<T>();
    mf::SignatureBuilder builder{"Sample Volume", signature_};
    builder.single_input<float3>("Position");
    builder.single_output("Value", cpp_type);
    this->set_signature(&signature_);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArraySpan<float3> positions = params.readonly_single_input<float3>(0, "Position");
    MutableSpan<T> dst = params.uninitialized_single_output<T>(1, "Value");

    const bke::VolumeTreeUser tree_user = volume_grid_->tree_user();
    sample_grid<T>(volume_grid_.grid(tree_user), positions, mask, dst, interpolation_mode_);
  }
};

struct SampleGridOp {
  GeoNodeExecParams params;
  GeometryNodeSampleGridInterpolationMode interpolation_mode;

  template<typename T> void operator()()
  {
    const bke::VolumeGrid<T> volume_grid = params.extract_input<bke::VolumeGrid<T>>("Grid");
    if (!volume_grid) {
      this->params.set_default_remaining_outputs();
      return;
    }

    fn::Field<float3> position_field = this->params.extract_input<fn::Field<float3>>("Position");
    auto fn = std::make_shared<SampleGridFunction<T>>(std::move(volume_grid),
                                                      this->interpolation_mode);
    auto op = FieldOperation::Create(std::move(fn), {position_field});
    fn::Field<T> output_field = fn::Field<T>(std::move(op));

    params.set_output("Value", output_field);
  }
};

#endif /* WITH_OPENVDB */

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  const NodeGeometrySampleGrid &storage = node_storage(params.node());
  const eCustomDataType data_type = eCustomDataType(storage.data_type);
  const GeometryNodeSampleGridInterpolationMode interpolation_mode =
      GeometryNodeSampleGridInterpolationMode(storage.interpolation_mode);

  SampleGridOp sample_op = {params, interpolation_mode};
  grids::apply(data_type, sample_op);
#else
  params.set_default_remaining_outputs();
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without OpenVDB"));
#endif
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem interpolation_mode_items[] = {
      {GEO_NODE_SAMPLE_GRID_INTERPOLATION_MODE_NEAREST, "NEAREST", 0, "Nearest Neighbor", ""},
      {GEO_NODE_SAMPLE_GRID_INTERPOLATION_MODE_TRILINEAR, "TRILINEAR", 0, "Trilinear", ""},
      {GEO_NODE_SAMPLE_GRID_INTERPOLATION_MODE_TRIQUADRATIC,
       "TRIQUADRATIC",
       0,
       "Triquadratic",
       ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem grid_type_items[] = {
      {CD_PROP_FLOAT, "FLOAT", 0, "Float", "Floating-point value"},
      {CD_PROP_FLOAT3, "FLOAT_VECTOR", 0, "Vector", "3D vector with floating-point values"},
      {CD_PROP_INT32, "INT", 0, "Integer", "32-bit integer"},
      {CD_PROP_BOOL, "BOOLEAN", 0, "Boolean", "True or false"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "grid_type",
                    "Grid Type",
                    "Type of grid to sample data from",
                    grid_type_items,
                    NOD_storage_enum_accessors(data_type),
                    CD_PROP_FLOAT);

  RNA_def_node_enum(srna,
                    "interpolation_mode",
                    "Interpolation Mode",
                    "How to interpolate the values from neighboring voxels",
                    interpolation_mode_items,
                    NOD_storage_enum_accessors(interpolation_mode),
                    GEO_NODE_SAMPLE_GRID_INTERPOLATION_MODE_TRILINEAR);
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SAMPLE_GRID, "Sample Grid", NODE_CLASS_CONVERTER);
  node_type_storage(
      &ntype, "NodeGeometrySampleGrid", node_free_standard_storage, node_copy_standard_storage);
  ntype.initfunc = node_init;
  ntype.declare = node_declare;
  ntype.gather_link_search_ops = search_link_ops;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_sample_grid_cc
