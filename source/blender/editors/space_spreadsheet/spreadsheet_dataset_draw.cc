/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_string.h"

#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "DNA_space_types.h"
#include "DNA_windowmanager_types.h"

#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_instances.hh"
#include "BKE_volume.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.h"

#include "UI_interface.hh"
#include "UI_tree_view.hh"

#include "WM_types.hh"

#include "BLT_translation.hh"

#include "spreadsheet_dataset_draw.hh"
#include "spreadsheet_intern.hh"

namespace blender::ed::spreadsheet {

class GeometryDataSetTreeView;

struct GeometryDataIdentifier {
  bke::GeometryComponent::Type component_type;
  std::optional<int> layer_index;
  std::optional<bke::AttrDomain> domain;
};

static void draw_count(ui::AbstractTreeViewItem &view_item, const int count)
{
  /* Using the tree row button instead of a separate right aligned button gives padding
   * to the right side of the number, which it didn't have with the button. */
  char element_count[BLI_STR_FORMAT_INT32_DECIMAL_UNIT_SIZE];
  BLI_str_format_decimal_unit(element_count, count);
  UI_but_hint_drawstr_set(reinterpret_cast<uiBut *>(view_item.view_item_button()), element_count);
}

static StringRefNull mesh_domain_to_label(const bke::AttrDomain domain)
{
  switch (domain) {
    case bke::AttrDomain::Point:
      return IFACE_("Vertex");
    case bke::AttrDomain::Edge:
      return IFACE_("Edge");
    case bke::AttrDomain::Face:
      return IFACE_("Face");
    case bke::AttrDomain::Corner:
      return IFACE_("Face Corner");
    default:
      BLI_assert_unreachable();
      return "";
  }
}

static StringRefNull curves_domain_to_label(const bke::AttrDomain domain)
{
  switch (domain) {
    case bke::AttrDomain::Point:
      return IFACE_("Control Point");
    case bke::AttrDomain::Curve:
      return IFACE_("Spline");
    default:
      BLI_assert_unreachable();
      return "";
  }
}

static BIFIconID mesh_domain_to_icon(const bke::AttrDomain domain)
{
  switch (domain) {
    case bke::AttrDomain::Point:
      return ICON_VERTEXSEL;
    case bke::AttrDomain::Edge:
      return ICON_EDGESEL;
    case bke::AttrDomain::Face:
      return ICON_FACESEL;
    case bke::AttrDomain::Corner:
      return ICON_FACE_CORNER;
    default:
      BLI_assert_unreachable();
      return ICON_NONE;
  }
}

static BIFIconID curves_domain_to_icon(const bke::AttrDomain domain)
{
  switch (domain) {
    case bke::AttrDomain::Point:
      return ICON_CURVE_BEZCIRCLE;
    case bke::AttrDomain::Curve:
      return ICON_CURVE_PATH;
    default:
      BLI_assert_unreachable();
      return ICON_NONE;
  }
}

class DataSetViewItem : public ui::AbstractTreeViewItem {
 private:
  GeometryDataSetTreeView &get_tree() const;

  void get_parent_instance_ids(Vector<SpreadsheetInstanceID> &r_instance_ids) const;

  void on_activate(bContext &C) override;

  std::optional<bool> should_be_active() const override;

  virtual std::optional<GeometryDataIdentifier> get_exact_data_id() const
  {
    return std::nullopt;
  }

  virtual std::optional<GeometryDataIdentifier> get_data_id_to_activate() const
  {
    return this->get_exact_data_id();
  }
};

class MeshViewItem : public DataSetViewItem {
 public:
  MeshViewItem()
  {
    label_ = IFACE_("Mesh");
  }

  std::optional<GeometryDataIdentifier> get_data_id_to_activate() const override
  {
    return GeometryDataIdentifier{
        bke::GeometryComponent::Type::Mesh, std::nullopt, bke::AttrDomain::Point};
  }

  void build_row(uiLayout &row) override
  {
    uiItemL(&row, label_.c_str(), ICON_MESH_DATA);
  }
};

class MeshDomainViewItem : public DataSetViewItem {
 private:
  const Mesh *mesh_;
  bke::AttrDomain domain_;

 public:
  MeshDomainViewItem(const Mesh *mesh, const bke::AttrDomain domain) : mesh_(mesh), domain_(domain)
  {
    label_ = mesh_domain_to_label(domain);
  }

  std::optional<GeometryDataIdentifier> get_exact_data_id() const override
  {
    return GeometryDataIdentifier{bke::GeometryComponent::Type::Mesh, std::nullopt, domain_};
  }

  void build_row(uiLayout &row)
  {
    const BIFIconID icon = mesh_domain_to_icon(domain_);
    uiItemL(&row, label_.c_str(), icon);

    const int count = mesh_ ? mesh_->attributes().domain_size(domain_) : 0;
    draw_count(*this, count);
  }
};

class CurvesViewItem : public DataSetViewItem {
 public:
  CurvesViewItem()
  {
    label_ = IFACE_("Curve");
  }

  std::optional<GeometryDataIdentifier> get_data_id_to_activate() const override
  {
    return GeometryDataIdentifier{
        bke::GeometryComponent::Type::Curve, std::nullopt, bke::AttrDomain::Point};
  }

  void build_row(uiLayout &row) override
  {
    uiItemL(&row, label_.c_str(), ICON_CURVE_DATA);
  }
};

class CurvesDomainViewItem : public DataSetViewItem {
 private:
  const Curves *curves_;
  bke::AttrDomain domain_;

 public:
  CurvesDomainViewItem(const Curves *curves, const bke::AttrDomain domain)
      : curves_(curves), domain_(domain)
  {
    label_ = curves_domain_to_label(domain);
  }

  std::optional<GeometryDataIdentifier> get_exact_data_id() const override
  {
    return GeometryDataIdentifier{bke::GeometryComponent::Type::Curve, std::nullopt, domain_};
  }

  void build_row(uiLayout &row)
  {
    const BIFIconID icon = curves_domain_to_icon(domain_);
    uiItemL(&row, label_.c_str(), icon);

    const int count = curves_ ? curves_->geometry.wrap().attributes().domain_size(domain_) : 0;
    draw_count(*this, count);
  }
};

class GreasePencilViewItem : public DataSetViewItem {
 public:
  GreasePencilViewItem()
  {
    label_ = IFACE_("Grease Pencil");
  }

  std::optional<GeometryDataIdentifier> get_data_id_to_activate() const override
  {
    return GeometryDataIdentifier{
        bke::GeometryComponent::Type::GreasePencil, std::nullopt, bke::AttrDomain::Layer};
  }

  void build_row(uiLayout &row) override
  {
    uiItemL(&row, label_.c_str(), ICON_OUTLINER_DATA_GREASEPENCIL);
  }
};

class GreasePencilLayersViewItem : public DataSetViewItem {
 private:
  const GreasePencil *grease_pencil_;

 public:
  GreasePencilLayersViewItem(const GreasePencil *grease_pencil) : grease_pencil_(grease_pencil)
  {
    label_ = IFACE_("Layer");
  }

  std::optional<GeometryDataIdentifier> get_exact_data_id() const override
  {
    return GeometryDataIdentifier{
        bke::GeometryComponent::Type::GreasePencil, std::nullopt, bke::AttrDomain::Layer};
  }

  void build_row(uiLayout &row) override
  {
    const int count = grease_pencil_ ? grease_pencil_->layers().size() : 0;
    uiItemL(&row, label_.c_str(), ICON_OUTLINER_DATA_GP_LAYER);
    draw_count(*this, count);
  }
};

class GreasePencilLayerViewItem : public DataSetViewItem {
 private:
  const bke::greasepencil::Layer &layer_;
  int layer_index_;

 public:
  GreasePencilLayerViewItem(const bke::greasepencil::Layer &layer, const int layer_index)
      : layer_(layer), layer_index_(layer_index)
  {
    label_ = layer_.name();
  }

  std::optional<GeometryDataIdentifier> get_data_id_to_activate() const override
  {
    return GeometryDataIdentifier{
        bke::GeometryComponent::Type::GreasePencil, layer_index_, bke::AttrDomain::Point};
  }

  void build_row(uiLayout &row) override
  {
    uiItemL(&row, label_.c_str(), ICON_CURVE_DATA);
  }
};

class GreasePencilLayerCurvesDomainViewItem : public DataSetViewItem {
 private:
  const GreasePencil &grease_pencil_;
  int layer_index_;
  bke::AttrDomain domain_;

 public:
  GreasePencilLayerCurvesDomainViewItem(const GreasePencil &grease_pencil,
                                        const int layer_index,
                                        const bke::AttrDomain domain)
      : grease_pencil_(grease_pencil), layer_index_(layer_index), domain_(domain)
  {
    label_ = curves_domain_to_label(domain);
  }

  std::optional<GeometryDataIdentifier> get_exact_data_id() const override
  {
    return GeometryDataIdentifier{
        bke::GeometryComponent::Type::GreasePencil, layer_index_, domain_};
  }

  void build_row(uiLayout &row)
  {
    const BIFIconID icon = curves_domain_to_icon(domain_);
    uiItemL(&row, label_.c_str(), icon);

    const bke::greasepencil::Drawing *drawing = grease_pencil_.get_eval_drawing(
        *grease_pencil_.layer(layer_index_));
    const int count = drawing ? drawing->strokes().attributes().domain_size(domain_) : 0;
    draw_count(*this, count);
  }
};

class PointCloudViewItem : public DataSetViewItem {
 public:
  PointCloudViewItem()
  {
    label_ = IFACE_("Point Cloud");
  }

  std::optional<GeometryDataIdentifier> get_data_id_to_activate() const override
  {
    return GeometryDataIdentifier{
        bke::GeometryComponent::Type::PointCloud, std::nullopt, bke::AttrDomain::Point};
  }

  void build_row(uiLayout &row) override
  {
    uiItemL(&row, label_.c_str(), ICON_POINTCLOUD_DATA);
  }
};

class PointsViewItem : public DataSetViewItem {
 private:
  const PointCloud *pointcloud_;

 public:
  PointsViewItem(const PointCloud *pointcloud) : pointcloud_(pointcloud)
  {
    label_ = IFACE_("Point");
  }

  std::optional<GeometryDataIdentifier> get_exact_data_id() const override
  {
    return GeometryDataIdentifier{
        bke::GeometryComponent::Type::PointCloud, std::nullopt, bke::AttrDomain::Point};
  }

  void build_row(uiLayout &row) override
  {
    uiItemL(&row, label_.c_str(), ICON_POINTCLOUD_POINT);
    const int count = pointcloud_ ? pointcloud_->totpoint : 0;
    draw_count(*this, count);
  }
};

class VolumeGridsViewItem : public DataSetViewItem {
 private:
  const Volume *volume_;

 public:
  VolumeGridsViewItem(const Volume *volume) : volume_(volume)
  {
    label_ = IFACE_("Volume Grids");
  }

  std::optional<GeometryDataIdentifier> get_exact_data_id() const override
  {
    return GeometryDataIdentifier{
        bke::GeometryComponent::Type::Volume, std::nullopt, std::nullopt};
  }

  void build_row(uiLayout &row) override
  {
    uiItemL(&row, label_.c_str(), ICON_VOLUME_DATA);
    const int count = volume_ ? BKE_volume_num_grids(volume_) : 0;
    draw_count(*this, count);
  }
};

class InstancesViewItem : public DataSetViewItem {
 private:
  const bke::Instances *instances_;

 public:
  InstancesViewItem(const bke::Instances *instances) : instances_(instances)
  {
    label_ = IFACE_("Instances");
  }

  std::optional<GeometryDataIdentifier> get_data_id_to_activate() const override
  {
    return GeometryDataIdentifier{
        bke::GeometryComponent::Type::Instance, std::nullopt, bke::AttrDomain::Instance};
  }

  void build_row(uiLayout &row) override
  {
    uiItemL(&row, label_.c_str(), ICON_EMPTY_AXIS);
    const int count = instances_ ? instances_->instances_num() : 0;
    draw_count(*this, count);
  }
};

class GeometryDataSetTreeView : public ui::AbstractTreeView {
 private:
  bke::GeometrySet root_geometry_set_;
  SpaceSpreadsheet &sspreadsheet_;
  bScreen &screen_;

  friend class DataSetViewItem;

 public:
  GeometryDataSetTreeView(bke::GeometrySet geometry_set, const bContext &C)
      : root_geometry_set_(std::move(geometry_set)),
        sspreadsheet_(*CTX_wm_space_spreadsheet(&C)),
        screen_(*CTX_wm_screen(&C))
  {
  }

  void build_tree() override
  {
    this->build_tree_for_geometry(root_geometry_set_, *this, true);
  }

  void build_tree_for_geometry(const bke::GeometrySet &geometry,
                               ui::TreeViewItemContainer &parent,
                               const bool all_components)
  {
    const Mesh *mesh = geometry.get_mesh();
    if (mesh || all_components) {
      this->build_tree_for_mesh(mesh, parent);
    }
    const Curves *curves = geometry.get_curves();
    if (curves || all_components) {
      this->build_tree_for_curves(curves, parent);
    }
    const GreasePencil *grease_pencil = geometry.get_grease_pencil();
    if (grease_pencil || all_components) {
      this->build_tree_for_grease_pencil(grease_pencil, parent);
    }
    const PointCloud *pointcloud = geometry.get_pointcloud();
    if (pointcloud || all_components) {
      this->build_tree_for_pointcloud(pointcloud, parent);
    }
    const Volume *volume = geometry.get_volume();
    if (volume || all_components) {
      this->build_tree_for_volume(volume, parent);
    }
    const bke::Instances *instances = geometry.get_instances();
    if (instances || all_components) {
      this->build_tree_for_instances(instances, parent);
    }
  }

  void build_tree_for_mesh(const Mesh *mesh, ui::TreeViewItemContainer &parent)
  {
    auto &mesh_item = parent.add_tree_item<MeshViewItem>();
    mesh_item.uncollapse_by_default();
    mesh_item.add_tree_item<MeshDomainViewItem>(mesh, bke::AttrDomain::Point);
    mesh_item.add_tree_item<MeshDomainViewItem>(mesh, bke::AttrDomain::Edge);
    mesh_item.add_tree_item<MeshDomainViewItem>(mesh, bke::AttrDomain::Face);
    mesh_item.add_tree_item<MeshDomainViewItem>(mesh, bke::AttrDomain::Corner);
  }

  void build_tree_for_curves(const Curves *curves, ui::TreeViewItemContainer &parent)
  {
    auto &curves_item = parent.add_tree_item<CurvesViewItem>();
    curves_item.uncollapse_by_default();
    curves_item.add_tree_item<CurvesDomainViewItem>(curves, bke::AttrDomain::Point);
    curves_item.add_tree_item<CurvesDomainViewItem>(curves, bke::AttrDomain::Curve);
  }

  void build_tree_for_grease_pencil(const GreasePencil *grease_pencil,
                                    ui::TreeViewItemContainer &parent)
  {
    auto &grease_pencil_item = parent.add_tree_item<GreasePencilViewItem>();
    grease_pencil_item.uncollapse_by_default();
    auto &layers_item = grease_pencil_item.add_tree_item<GreasePencilLayersViewItem>(
        grease_pencil);
    if (!grease_pencil) {
      return;
    }
    const Span<const bke::greasepencil::Layer *> layers = grease_pencil->layers();
    for (const int layer_i : layers.index_range()) {
      const bke::greasepencil::Layer &layer = *layers[layer_i];
      auto &layer_item = layers_item.add_tree_item<GreasePencilLayerViewItem>(layer, layer_i);
      layer_item.add_tree_item<GreasePencilLayerCurvesDomainViewItem>(
          *grease_pencil, layer_i, bke::AttrDomain::Point);
      layer_item.add_tree_item<GreasePencilLayerCurvesDomainViewItem>(
          *grease_pencil, layer_i, bke::AttrDomain::Curve);
    }
  }

  void build_tree_for_pointcloud(const PointCloud *pointcloud, ui::TreeViewItemContainer &parent)
  {
    auto &pointcloud_item = parent.add_tree_item<PointCloudViewItem>();
    pointcloud_item.uncollapse_by_default();
    pointcloud_item.add_tree_item<PointsViewItem>(pointcloud);
  }

  void build_tree_for_volume(const Volume *volume, ui::TreeViewItemContainer &parent)
  {
    parent.add_tree_item<VolumeGridsViewItem>(volume);
  }

  void build_tree_for_instances(const bke::Instances *instances, ui::TreeViewItemContainer &parent)
  {
    parent.add_tree_item<InstancesViewItem>(instances);
  }
};

GeometryDataSetTreeView &DataSetViewItem::get_tree() const
{
  return static_cast<GeometryDataSetTreeView &>(this->get_tree_view());
}

void DataSetViewItem::get_parent_instance_ids(Vector<SpreadsheetInstanceID> &r_instance_ids) const
{
  UNUSED_VARS(r_instance_ids);
}

void DataSetViewItem::on_activate(bContext &C)
{
  const std::optional<GeometryDataIdentifier> data_to_activate = this->get_data_id_to_activate();
  if (!data_to_activate) {
    return;
  }

  Vector<SpreadsheetInstanceID> instance_ids;
  this->get_parent_instance_ids(instance_ids);

  bScreen &screen = *CTX_wm_screen(&C);
  SpaceSpreadsheet &sspreadsheet = *CTX_wm_space_spreadsheet(&C);

  MEM_SAFE_FREE(sspreadsheet.instance_ids);
  sspreadsheet.instance_ids = MEM_cnew_array<SpreadsheetInstanceID>(instance_ids.size(), __func__);
  sspreadsheet.instance_ids_num = instance_ids.size();
  initialized_copy_n(instance_ids.data(), instance_ids.size(), sspreadsheet.instance_ids);

  sspreadsheet.geometry_component_type = uint8_t(data_to_activate->component_type);
  if (data_to_activate->domain) {
    sspreadsheet.attribute_domain = uint8_t(*data_to_activate->domain);
  }
  if (data_to_activate->layer_index) {
    sspreadsheet.active_layer_index = *data_to_activate->layer_index;
  }
  PointerRNA ptr = RNA_pointer_create(&screen.id, &RNA_SpaceSpreadsheet, &sspreadsheet);
  /* These updates also make sure that the attribute domain is set properly based on the
   * component type. */
  RNA_property_update(&C, &ptr, RNA_struct_find_property(&ptr, "attribute_domain"));
  RNA_property_update(&C, &ptr, RNA_struct_find_property(&ptr, "geometry_component_type"));
}

std::optional<bool> DataSetViewItem::should_be_active() const
{
  GeometryDataSetTreeView &tree_view = this->get_tree();
  SpaceSpreadsheet &sspreadsheet = tree_view.sspreadsheet_;

  const std::optional<GeometryDataIdentifier> data_id = this->get_exact_data_id();
  if (!data_id) {
    return false;
  }
  if (bke::GeometryComponent::Type(sspreadsheet.geometry_component_type) !=
      data_id->component_type)
  {
    return false;
  }
  if (data_id->domain) {
    if (bke::AttrDomain(sspreadsheet.attribute_domain) != data_id->domain) {
      return false;
    }
  }
  if (data_id->layer_index) {
    if (sspreadsheet.active_layer_index != *data_id->layer_index) {
      return false;
    }
  }
  Vector<SpreadsheetInstanceID> instance_ids;
  this->get_parent_instance_ids(instance_ids);
  if (sspreadsheet.instance_ids_num != instance_ids.size()) {
    return false;
  }
  for (const int i : instance_ids.index_range()) {
    const SpreadsheetInstanceID &a = sspreadsheet.instance_ids[i];
    const SpreadsheetInstanceID &b = instance_ids[i];
    if (a.reference_index != b.reference_index) {
      return false;
    }
  }
  return true;
}

void spreadsheet_data_set_panel_draw(const bContext *C, Panel *panel)
{
  const SpaceSpreadsheet *sspreadsheet = CTX_wm_space_spreadsheet(C);
  Object *object = spreadsheet_get_object_eval(sspreadsheet, CTX_data_depsgraph_pointer(C));
  if (!object) {
    return;
  }
  uiLayout *layout = panel->layout;

  uiBlock *block = uiLayoutGetBlock(layout);

  UI_block_layout_set_current(block, layout);

  ui::AbstractTreeView *tree_view = UI_block_add_view(
      *block,
      "Data Set Tree View",
      std::make_unique<GeometryDataSetTreeView>(
          spreadsheet_get_display_geometry_set(sspreadsheet, object), *C));
  tree_view->set_context_menu_title("Spreadsheet");
  ui::TreeViewBuilder::build_tree_view(*tree_view, *layout);
}

}  // namespace blender::ed::spreadsheet
