/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edasset
 */

#include "asset_shelf.hh"

#include "BKE_screen.hh"

#include "BLT_translation.hh"

#include "UI_interface_c.hh"
#include "UI_tree_view.hh"

#include "ED_asset_filter.hh"
#include "ED_asset_list.hh"
#include "ED_asset_shelf.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.h"

namespace blender::ed::asset::shelf {

class PopupAssetShelfStorage {
 public:
  ListBase popup_shelves;

  ~PopupAssetShelfStorage()
  {
    LISTBASE_FOREACH_MUTABLE (
        AssetShelf *, shelf, &PopupAssetShelfStorage::get_popup_asset_shelves())
    {
      MEM_delete(shelf);
    }
  }
  static ListBase &get_popup_asset_shelves()
  {
    static PopupAssetShelfStorage storage;
    return storage.popup_shelves;
  }
};

void type_popup_unlink(const AssetShelfType &shelf_type)
{
  LISTBASE_FOREACH (AssetShelf *, shelf, &PopupAssetShelfStorage::get_popup_asset_shelves()) {
    if (shelf->type == &shelf_type) {
      shelf->type = nullptr;
    }
  }
}

static AssetShelf *get_shelf_for_popup(const bContext *C, AssetShelfType &shelf_type)
{
  const SpaceType *space_type = BKE_spacetype_from_id(shelf_type.space_type);

  ListBase &popup_shelves = PopupAssetShelfStorage::get_popup_asset_shelves();

  LISTBASE_FOREACH (AssetShelf *, shelf, &popup_shelves) {
    if (STREQ(shelf->idname, shelf_type.idname)) {
      if (type_poll(*C, *space_type, type_ensure(*space_type, *shelf))) {
        return shelf;
      }
      break;
    }
  }

  if (type_poll(*C, *space_type, &shelf_type)) {
    AssetShelf *new_shelf = create_shelf_from_type(shelf_type);
    BLI_addtail(&popup_shelves, new_shelf);
    return new_shelf;
  }

  return nullptr;
}

class AssetCatalogTreeView : public ui::AbstractTreeView {
  AssetShelf &shelf_;
  asset_system::AssetCatalogTree catalog_tree_;

 public:
  AssetCatalogTreeView(const asset_system::AssetLibrary &library, AssetShelf &shelf)
      : shelf_(shelf)
  {
    catalog_tree_ = build_filtered_catalog_tree(
        library,
        shelf_.settings.asset_library_reference,
        [this](const asset_system::AssetRepresentation &asset) {
          return (!shelf_.type->asset_poll || shelf_.type->asset_poll(shelf_.type, &asset));
        });
  }

  void build_tree() override
  {
    if (catalog_tree_.is_empty()) {
      auto &item = this->add_tree_item<ui::BasicTreeViewItem>(RPT_("No applicable assets found"),
                                                              ICON_INFO);
      item.disable_interaction();
      return;
    }

    auto &all_item = this->add_tree_item<ui::BasicTreeViewItem>(IFACE_("All"));
    all_item.set_on_activate_fn([this](bContext &C, ui::BasicTreeViewItem &) {
      settings_set_all_catalog_active(shelf_.settings);
      send_redraw_notifier(C);
    });
    all_item.set_is_active_fn(
        [this]() { return settings_is_all_catalog_active(shelf_.settings); });
    all_item.uncollapse_by_default();

    catalog_tree_.foreach_root_item([&, this](
                                        const asset_system::AssetCatalogTreeItem &catalog_item) {
      ui::BasicTreeViewItem &item = this->build_catalog_items_recursive(all_item, catalog_item);
      item.uncollapse_by_default();
    });
  }

  ui::BasicTreeViewItem &build_catalog_items_recursive(
      ui::TreeViewOrItem &parent_view_item,
      const asset_system::AssetCatalogTreeItem &catalog_item) const
  {
    ui::BasicTreeViewItem &view_item = parent_view_item.add_tree_item<ui::BasicTreeViewItem>(
        catalog_item.get_name());

    std::string catalog_path = catalog_item.catalog_path().str();
    view_item.set_on_activate_fn([this, catalog_path](bContext &C, ui::BasicTreeViewItem &) {
      settings_set_active_catalog(shelf_.settings, catalog_path);
      send_redraw_notifier(C);
    });
    view_item.set_is_active_fn([this, catalog_path]() {
      return settings_is_active_catalog(shelf_.settings, catalog_path);
    });

    catalog_item.foreach_child(
        [&view_item, this](const asset_system::AssetCatalogTreeItem &child) {
          build_catalog_items_recursive(view_item, child);
        });

    return view_item;
  }
};

static void catalog_tree_draw(uiLayout &layout, AssetShelf &shelf)
{
  const asset_system::AssetLibrary *library = list::library_get_once_available(
      shelf.settings.asset_library_reference);
  if (!library) {
    return;
  }

  uiBlock *block = uiLayoutGetBlock(&layout);
  ui::AbstractTreeView *tree_view = UI_block_add_view(
      *block,
      "asset shelf catalog tree view",
      std::make_unique<AssetCatalogTreeView>(*library, shelf));

  ui::TreeViewBuilder::build_tree_view(*tree_view, layout);
}

uiBlock *popup_block_create(const bContext *C, ARegion *region, AssetShelfType *shelf_type)
{
  uiBlock *block = UI_block_begin(C, region, "_popup", UI_EMBOSS);
  UI_block_flag_enable(block, UI_BLOCK_KEEP_OPEN | UI_BLOCK_POPOVER);
  UI_block_theme_style_set(block, UI_BLOCK_THEME_STYLE_POPUP);

  AssetShelf *shelf = get_shelf_for_popup(C, *shelf_type);
  if (!shelf) {
    BLI_assert_unreachable();
    return block;
  }

  const uiStyle *style = UI_style_get_dpi();

  const float pad = 0.2f * UI_UNIT_Y; /* UI_MENU_PADDING */
  uiLayout *layout = UI_block_layout(
      block, UI_LAYOUT_VERTICAL, UI_LAYOUT_PANEL, pad, 0, UI_UNIT_X * 40, 0, pad / 2, style);

  PointerRNA library_ref_ptr = RNA_pointer_create(
      &CTX_wm_screen(C)->id, &RNA_AssetLibraryReference, &shelf->settings.asset_library_reference);
  uiLayoutSetContextPointer(layout, "asset_library_reference", &library_ref_ptr);

  uiLayout *row = uiLayoutRow(layout, false);
  uiLayout *sub = uiLayoutRow(row, false);
  uiLayoutSetUnitsX(sub, 10);
  uiLayoutSetFixedSize(sub, true);
  uiLayout *catalogs_col = uiLayoutColumn(sub, false);
  library_selector_draw(C, catalogs_col, *shelf);
  catalog_tree_draw(*catalogs_col, *shelf);

  uiLayout *asset_view_col = uiLayoutColumn(row, false);
  build_asset_view(*asset_view_col, shelf->settings.asset_library_reference, *shelf, *C, *region);

  return block;
}

}  // namespace blender::ed::asset::shelf
