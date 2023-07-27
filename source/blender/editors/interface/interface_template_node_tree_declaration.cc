/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edinterface
 */

#include "UI_interface.h"

#include "BKE_context.h"
#include "BKE_node_tree_interface.hh"
#include "BKE_node_tree_update.h"

#include "BLI_color.hh"
#include "BLI_string.h"

#include "BLT_translation.h"

#include "DNA_node_tree_interface_types.h"

#include "ED_node.h"

#include "RNA_access.h"
#include "RNA_prototypes.h"

#include "UI_interface.h"
#include "UI_interface.hh"
#include "UI_resources.h"
#include "UI_tree_view.hh"

#include "WM_api.h"

namespace node_interface = blender::bke::node_interface;

namespace blender::ui::nodes {

namespace {

class NodeTreeInterfaceView;

class NodeTreeInterfaceDragController : public AbstractViewItemDragController {
 public:
  explicit NodeTreeInterfaceDragController(NodeTreeInterfaceView &view,
                                           bNodeTreeInterfaceItem &item);
  virtual ~NodeTreeInterfaceDragController() = default;

  eWM_DragDataType get_drag_type() const;

  void *create_drag_data() const;

 private:
  bNodeTreeInterfaceItem &item_;
};

class NodeSocketDropTarget : public TreeViewItemDropTarget {
 public:
  explicit NodeSocketDropTarget(NodeTreeInterfaceView &view, bNodeTreeInterfaceSocket &socket);

  bool can_drop(const wmDrag &drag, const char **r_disabled_hint) const override;
  std::string drop_tooltip(const DragInfo &drag_info) const override;
  bool on_drop(struct bContext * /*C*/, const DragInfo &drag_info) const override;

 protected:
  wmDragNodeTreeInterface *get_drag_node_tree_declaration(const wmDrag &drag) const;

 private:
  bNodeTreeInterfaceSocket &socket_;
};

class NodePanelDropTarget : public TreeViewItemDropTarget {
 public:
  explicit NodePanelDropTarget(NodeTreeInterfaceView &view, bNodeTreeInterfacePanel &panel);

  bool can_drop(const wmDrag &drag, const char **r_disabled_hint) const override;
  std::string drop_tooltip(const DragInfo &drag_info) const override;
  bool on_drop(bContext *C, const DragInfo &drag_info) const override;

 protected:
  wmDragNodeTreeInterface *get_drag_node_tree_declaration(const wmDrag &drag) const;

 private:
  bNodeTreeInterfacePanel &panel_;
};

class NodeSocketViewItem : public BasicTreeViewItem {
 public:
  NodeSocketViewItem(bNodeTree &nodetree,
                     bNodeTreeInterface &interface,
                     bNodeTreeInterfaceSocket &socket)
      : BasicTreeViewItem(socket.name, ICON_NONE),
        nodetree_(nodetree),
        interface_(interface),
        socket_(socket)
  {
  }

  void build_row(uiLayout &row) override
  {
    uiLayoutSetPropDecorate(&row, false);

    uiLayout *input_socket_layout = uiLayoutRow(&row, true);
    if (socket_.flag & NODE_INTERFACE_SOCKET_INPUT) {
      /* XXX Socket template only draws in embossed layouts (Julian). */
      uiLayoutSetEmboss(input_socket_layout, UI_EMBOSS);
      /* XXX Context is not used by the template function. */
      bContext *C = nullptr;
      uiTemplateNodeSocket(input_socket_layout, C, socket_.socket_color());
    }
    else {
      /* Blank item to align output socket labels with inputs. */
      uiItemL(input_socket_layout, "", ICON_BLANK1);
    }

    add_label(row);

    uiLayout *output_socket_layout = uiLayoutRow(&row, true);
    if (socket_.flag & NODE_INTERFACE_SOCKET_OUTPUT) {
      /* XXX Socket template only draws in embossed layouts (Julian). */
      uiLayoutSetEmboss(output_socket_layout, UI_EMBOSS);
      /* XXX Context is not used by the template function. */
      bContext *C = nullptr;
      uiTemplateNodeSocket(output_socket_layout, C, socket_.socket_color());
    }
    else {
      /* Blank item to align input socket labels with outputs. */
      uiItemL(output_socket_layout, "", ICON_BLANK1);
    }
  }

 protected:
  bool matches(const AbstractViewItem &other) const override
  {
    const NodeSocketViewItem *other_item = dynamic_cast<const NodeSocketViewItem *>(&other);
    if (other_item == nullptr) {
      return false;
    }

    return &socket_ == &other_item->socket_;
  }

  std::optional<bool> should_be_active() const override
  {
    return interface_.active_item() == &socket_.item;
  }
  void on_activate() override
  {
    interface_.active_item_set(&socket_.item);
  }

  bool supports_renaming() const override
  {
    return true;
  }
  bool rename(const bContext &C, StringRefNull new_name) override
  {
    socket_.name = BLI_strdup(new_name.c_str());
    BKE_ntree_update_tag_interface(&nodetree_);
    ED_node_tree_propagate_change(&C, CTX_data_main(&C), &nodetree_);
    return true;
  }
  StringRef get_rename_string() const override
  {
    return socket_.name;
  }

  std::unique_ptr<AbstractViewItemDragController> create_drag_controller() const override;
  std::unique_ptr<TreeViewItemDropTarget> create_drop_target() override;

 private:
  bNodeTree &nodetree_;
  bNodeTreeInterface &interface_;
  bNodeTreeInterfaceSocket &socket_;
};

class NodePanelViewItem : public BasicTreeViewItem {
 public:
  NodePanelViewItem(bNodeTree &nodetree,
                    bNodeTreeInterface &interface,
                    bNodeTreeInterfacePanel &panel)
      : BasicTreeViewItem(panel.name, ICON_NONE),
        nodetree_(nodetree),
        interface_(interface),
        panel_(panel)
  {
  }

  void build_row(uiLayout &row) override
  {
    add_label(row);

    uiLayout *sub = uiLayoutRow(&row, true);
    uiLayoutSetPropDecorate(sub, false);
  }

 protected:
  bool matches(const AbstractViewItem &other) const override
  {
    const NodePanelViewItem *other_item = dynamic_cast<const NodePanelViewItem *>(&other);
    if (other_item == nullptr) {
      return false;
    }

    return &panel_ == &other_item->panel_;
  }

  std::optional<bool> should_be_active() const override
  {
    return interface_.active_item() == &panel_.item;
  }
  void on_activate() override
  {
    interface_.active_item_set(&panel_.item);
  }

  bool supports_renaming() const override
  {
    return true;
  }
  bool rename(const bContext &C, StringRefNull new_name) override
  {
    panel_.name = BLI_strdup(new_name.c_str());
    BKE_ntree_update_tag_interface(&nodetree_);
    ED_node_tree_propagate_change(&C, CTX_data_main(&C), &nodetree_);
    return true;
  }
  StringRef get_rename_string() const override
  {
    return panel_.name;
  }

  std::unique_ptr<AbstractViewItemDragController> create_drag_controller() const override;
  std::unique_ptr<TreeViewItemDropTarget> create_drop_target() override;

 private:
  bNodeTree &nodetree_;
  bNodeTreeInterface &interface_;
  bNodeTreeInterfacePanel &panel_;
};

class NodeTreeInterfaceView : public AbstractTreeView {
 public:
  explicit NodeTreeInterfaceView(bNodeTree &nodetree, bNodeTreeInterface &interface)
      : nodetree_(nodetree), interface_(interface)
  {
  }

  bNodeTree &nodetree()
  {
    return nodetree_;
  }

  bNodeTreeInterface &interface()
  {
    return interface_;
  }

  void build_tree() override
  {
    /* Draw root items */
    add_items_for_panel_recursive(interface_.root_panel, *this);
  }

 protected:
  void add_items_for_panel_recursive(bNodeTreeInterfacePanel &parent,
                                     ui::TreeViewOrItem &parent_item)
  {
    for (bNodeTreeInterfaceItem *item : parent.items()) {
      switch (item->item_type) {
        case NODE_INTERFACE_SOCKET: {
          bNodeTreeInterfaceSocket *socket = node_interface::get_as_ptr<bNodeTreeInterfaceSocket>(
              item);
          NodeSocketViewItem &socket_item = parent_item.add_tree_item<NodeSocketViewItem>(
              nodetree_, interface_, *socket);
          socket_item.set_collapsed(false);
          break;
        }
        case NODE_INTERFACE_PANEL: {
          bNodeTreeInterfacePanel *panel = node_interface::get_as_ptr<bNodeTreeInterfacePanel>(
              item);
          NodePanelViewItem &panel_item = parent_item.add_tree_item<NodePanelViewItem>(
              nodetree_, interface_, *panel);
          panel_item.set_collapsed(false);
          add_items_for_panel_recursive(*panel, panel_item);
          break;
        }
      }
    }
  }

 private:
  bNodeTree &nodetree_;
  bNodeTreeInterface &interface_;
};

std::unique_ptr<AbstractViewItemDragController> NodeSocketViewItem::create_drag_controller() const
{
  return std::make_unique<NodeTreeInterfaceDragController>(
      static_cast<NodeTreeInterfaceView &>(get_tree_view()), socket_.item);
}

std::unique_ptr<TreeViewItemDropTarget> NodeSocketViewItem::create_drop_target()
{
  return std::make_unique<NodeSocketDropTarget>(
      static_cast<NodeTreeInterfaceView &>(get_tree_view()), socket_);
}

std::unique_ptr<AbstractViewItemDragController> NodePanelViewItem::create_drag_controller() const
{
  return std::make_unique<NodeTreeInterfaceDragController>(
      static_cast<NodeTreeInterfaceView &>(get_tree_view()), panel_.item);
}

std::unique_ptr<TreeViewItemDropTarget> NodePanelViewItem::create_drop_target()
{
  return std::make_unique<NodePanelDropTarget>(
      static_cast<NodeTreeInterfaceView &>(get_tree_view()), panel_);
}

NodeTreeInterfaceDragController::NodeTreeInterfaceDragController(NodeTreeInterfaceView &view,
                                                                 bNodeTreeInterfaceItem &item)
    : AbstractViewItemDragController(view), item_(item)
{
}

eWM_DragDataType NodeTreeInterfaceDragController::get_drag_type() const
{
  return WM_DRAG_NODE_TREE_INTERFACE;
}

void *NodeTreeInterfaceDragController::create_drag_data() const
{
  wmDragNodeTreeInterface *drag_data = MEM_cnew<wmDragNodeTreeInterface>(__func__);
  drag_data->item = &item_;
  return drag_data;
}

NodeSocketDropTarget::NodeSocketDropTarget(NodeTreeInterfaceView &view,
                                           bNodeTreeInterfaceSocket &socket)
    : TreeViewItemDropTarget(view, DropBehavior::Reorder), socket_(socket)
{
}

bool NodeSocketDropTarget::can_drop(const wmDrag &drag, const char ** /*r_disabled_hint*/) const
{
  if (drag.type != WM_DRAG_NODE_TREE_INTERFACE) {
    return false;
  }
  wmDragNodeTreeInterface *drag_data = get_drag_node_tree_declaration(drag);

  /* Can't drop an item onto its children. */
  if (const bNodeTreeInterfacePanel *panel = node_interface::get_as_ptr<bNodeTreeInterfacePanel>(
          drag_data->item))
  {
    if (panel->contains_item(socket_.item)) {
      return false;
    }
  }
  return true;
}

std::string NodeSocketDropTarget::drop_tooltip(const DragInfo &drag_info) const
{
  switch (drag_info.drop_location) {
    case DropLocation::Into:
      return "";
    case DropLocation::Before:
      return N_("Insert before socket");
    case DropLocation::After:
      return N_("Insert after socket");
  }
  return "";
}

bool NodeSocketDropTarget::on_drop(bContext *C, const DragInfo &drag_info) const
{
  wmDragNodeTreeInterface *drag_data = get_drag_node_tree_declaration(drag_info.drag_data);
  BLI_assert(drag_data != nullptr);
  bNodeTreeInterfaceItem *drag_item = drag_data->item;
  BLI_assert(drag_item != nullptr);

  bNodeTree &nodetree = get_view<NodeTreeInterfaceView>().nodetree();
  bNodeTreeInterface &interface = get_view<NodeTreeInterfaceView>().interface();

  bNodeTreeInterfacePanel *parent = nullptr;
  int index = -1;

  /* Insert into same panel as the target. */
  interface.find_item_parent(socket_.item, parent);
  BLI_assert(parent != nullptr);
  switch (drag_info.drop_location) {
    case DropLocation::Before:
      index = parent->items().as_span().first_index_try(&socket_.item);
      break;
    case DropLocation::After:
      index = parent->items().as_span().first_index_try(&socket_.item) + 1;
      break;
    default:
      /* All valid cases should be handled above. */
      BLI_assert_unreachable();
      break;
  }
  if (parent == nullptr || index < 0) {
    return false;
  }

  interface.move_item_to_parent(*drag_item, parent, index);

  /* General update */
  BKE_ntree_update_tag_interface(&nodetree);
  ED_node_tree_propagate_change(C, CTX_data_main(C), &nodetree);
  return true;
}

wmDragNodeTreeInterface *NodeSocketDropTarget::get_drag_node_tree_declaration(
    const wmDrag &drag) const
{
  BLI_assert(drag.type == WM_DRAG_NODE_TREE_INTERFACE);
  return static_cast<wmDragNodeTreeInterface *>(drag.poin);
}

NodePanelDropTarget::NodePanelDropTarget(NodeTreeInterfaceView &view,
                                         bNodeTreeInterfacePanel &panel)
    : TreeViewItemDropTarget(view, DropBehavior::ReorderAndInsert), panel_(panel)
{
}

bool NodePanelDropTarget::can_drop(const wmDrag &drag, const char ** /*r_disabled_hint*/) const
{
  if (drag.type != WM_DRAG_NODE_TREE_INTERFACE) {
    return false;
  }
  wmDragNodeTreeInterface *drag_data = get_drag_node_tree_declaration(drag);

  /* Can't drop an item onto its children. */
  if (const bNodeTreeInterfacePanel *panel = node_interface::get_as_ptr<bNodeTreeInterfacePanel>(
          drag_data->item))
  {
    if (panel->contains_item(panel_.item)) {
      return false;
    }
  }

  return true;
}

std::string NodePanelDropTarget::drop_tooltip(const DragInfo &drag_info) const
{
  switch (drag_info.drop_location) {
    case DropLocation::Into:
      return "Insert into panel";
    case DropLocation::Before:
      return N_("Insert before panel");
    case DropLocation::After:
      return N_("Insert after panel");
  }
  return "";
}

bool NodePanelDropTarget::on_drop(bContext *C, const DragInfo &drag_info) const
{
  wmDragNodeTreeInterface *drag_data = get_drag_node_tree_declaration(drag_info.drag_data);
  BLI_assert(drag_data != nullptr);
  bNodeTreeInterfaceItem *drag_item = drag_data->item;
  BLI_assert(drag_item != nullptr);

  bNodeTree &nodetree = get_view<NodeTreeInterfaceView>().nodetree();
  bNodeTreeInterface &interface = get_view<NodeTreeInterfaceView>().interface();

  bNodeTreeInterfacePanel *parent = nullptr;
  int index = -1;
  switch (drag_info.drop_location) {
    case DropLocation::Into: {
      /* Insert into target */
      parent = &panel_;
      index = 0;
      break;
    }
    case DropLocation::Before: {
      /* Insert into same panel as the target. */
      interface.find_item_parent(panel_.item, parent);
      BLI_assert(parent != nullptr);
      index = parent->items().as_span().first_index_try(&panel_.item);
      break;
    }
    case DropLocation::After: {
      /* Insert into same panel as the target. */
      interface.find_item_parent(panel_.item, parent);
      BLI_assert(parent != nullptr);
      index = parent->items().as_span().first_index_try(&panel_.item) + 1;
      break;
    }
  }
  if (parent == nullptr || index < 0) {
    return false;
  }

  interface.move_item_to_parent(*drag_item, parent, index);

  /* General update */
  BKE_ntree_update_tag_interface(&nodetree);
  ED_node_tree_propagate_change(C, CTX_data_main(C), &nodetree);
  return true;
}

wmDragNodeTreeInterface *NodePanelDropTarget::get_drag_node_tree_declaration(
    const wmDrag &drag) const
{
  BLI_assert(drag.type == WM_DRAG_NODE_TREE_INTERFACE);
  return static_cast<wmDragNodeTreeInterface *>(drag.poin);
}

}  // namespace

}  // namespace blender::ui::nodes

namespace ui = blender::ui;

void uiTemplateNodeTreeDeclaration(struct uiLayout *layout, struct PointerRNA *ptr)
{
  if (!ptr->data) {
    return;
  }
  if (!RNA_struct_is_a(ptr->type, &RNA_NodeTreeInterface)) {
    return;
  }
  bNodeTree &nodetree = *reinterpret_cast<bNodeTree *>(ptr->owner_id);
  bNodeTreeInterface &interface = *static_cast<bNodeTreeInterface *>(ptr->data);

  uiBlock *block = uiLayoutGetBlock(layout);

  ui::AbstractTreeView *tree_view = UI_block_add_view(
      *block,
      "Node Tree Declaration Tree View",
      std::make_unique<blender::ui::nodes::NodeTreeInterfaceView>(nodetree, interface));
  tree_view->set_min_rows(3);

  ui::TreeViewBuilder::build_tree_view(*tree_view, *layout);
}
