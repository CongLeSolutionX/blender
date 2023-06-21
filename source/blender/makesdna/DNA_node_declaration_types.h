/* SPDX-FileCopyrightText: 2005 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup DNA
 */

#pragma once

#include "BLI_utildefines.h"

#ifdef __cplusplus
#  include "BLI_span.hh"
#  include "BLI_string_ref.hh"

#  include <memory>
#endif

struct bNodeTreeInterfacePanel;
struct bNodeTreeInterfaceSocket;

/** Socket side (input/output). */
typedef enum eNodeTreeInterfaceItemType {
  NODE_INTERFACE_SOCKET = 0,
  NODE_INTERFACE_PANEL = 1,
} eNodeTreeInterfaceItemType;

/** Describes a socket and all necessary details for a node declaration. */
typedef struct bNodeTreeInterfaceItem {
  /* eNodeTreeInterfaceItemType */
  char item_type;
  char _pad[3];

  /* Index in final item sequence. */
  int index;

  /* Panel in which to display the item. */
  struct bNodeTreeInterfacePanel *parent;

#ifdef __cplusplus
  template<typename T> T &get_as();
  template<typename T> const T &get_as() const;

  template<typename T> T *get_as_ptr();
  template<typename T> const T *get_as_ptr() const;
#endif
} bNodeTreeInterfaceItem;

/** Socket side (input/output). */
typedef enum eNodeSocketDeclarationInOut {
  SOCKDECL_IN = 1 << 0,
  SOCKDECL_OUT = 1 << 1,
} eNodeSocketDeclarationInOut;
ENUM_OPERATORS(eNodeSocketDeclarationInOut, SOCKDECL_OUT);

typedef struct bNodeTreeInterfaceSocket {
  bNodeTreeInterfaceItem item;

  char *name;
  char *description;
  char *type;
  /* eNodeSocketDeclarationInOut */
  int in_out;
  char _pad[4];
} bNodeTreeInterfaceSocket;

typedef struct bNodeTreeInterfacePanel {
  bNodeTreeInterfaceItem item;

  char *name;
} bNodeTreeInterfacePanel;

typedef struct bNodeTreeInterface {
  bNodeTreeInterfaceItem **items_array;
  int items_num;
  int active_item;

#ifdef __cplusplus
  blender::Span<const bNodeTreeInterfaceItem *> items() const;
  blender::MutableSpan<bNodeTreeInterfaceItem *> items();

  bNodeTreeInterfaceSocket *add_socket(blender::StringRef name,
                                       blender::StringRef description,
                                       blender::StringRef type,
                                       eNodeSocketDeclarationInOut in_out);
  bNodeTreeInterfaceSocket *insert_socket(blender::StringRef name,
                                          blender::StringRef description,
                                          blender::StringRef type,
                                          eNodeSocketDeclarationInOut in_out,
                                          int index);
  bNodeTreeInterfacePanel *add_panel(blender::StringRef name);
  bNodeTreeInterfacePanel *insert_panel(blender::StringRef name, int index);

  bool remove_item(bNodeTreeInterfaceItem &item);
  void clear_item_type(eNodeTreeInterfaceItemType type);
  void clear_items();
  bool move_item(bNodeTreeInterfaceItem &item, int new_index);

 protected:
  int item_index(bNodeTreeInterfaceItem &item) const;

  void add_item(bNodeTreeInterfaceItem &item);
  void insert_item(bNodeTreeInterfaceItem &item, int index);
  void free_item(bNodeTreeInterfaceItem &item);

  void update_order();

 private:
  void update_panels_order();
  void update_sockets_order();
  void update_index();

#endif
} bNodeTreeInterface;
