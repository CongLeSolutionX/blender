/* SPDX-FileCopyrightText: 2008 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup collada
 */

#include <assert.h>
#include <string.h>

#include "io_ops.hh" /* own include */

#include "DNA_screen_types.h"
#include "DNA_space_types.h"

#include "BLI_listbase.h"

#include "BKE_context.hh"
#include "BKE_screen.hh"
#include "WM_api.hh"

#ifdef WITH_COLLADA
#  include "io_collada.hh"
#endif

#ifdef WITH_ALEMBIC
#  include "io_alembic.hh"
#endif

#ifdef WITH_USD
#  include "io_usd.hh"
#endif

#include "io_cache.hh"
#include "io_gpencil.hh"
#include "io_obj.hh"
#include "io_ply_ops.hh"
#include "io_stl_ops.hh"

bool IO_paneltype_set_parent(struct PanelType *panel)
{
  PanelType *parent = NULL;

  SpaceType *space_type = BKE_spacetype_from_id(SPACE_FILE);
  assert(space_type);

  ARegionType *region = BKE_regiontype_from_id(space_type, RGN_TYPE_TOOL_PROPS);
  assert(region);

  LISTBASE_FOREACH (PanelType *, pt, &region->paneltypes) {
    if (strcasecmp(pt->idname, panel->parent_id) == 0) {
      parent = pt;
      break;
    }
  }

  if (parent) {
    panel->parent = parent;
    LinkData *pt_child_iter = static_cast<LinkData *>(parent->children.last);
    for (; pt_child_iter; pt_child_iter = pt_child_iter->prev) {
      PanelType *pt_child = static_cast<PanelType *>(pt_child_iter->data);
      if (pt_child->order <= panel->order) {
        break;
      }
    }
    BLI_insertlinkafter(&parent->children, pt_child_iter, BLI_genericNodeN(panel));
    return true;
  }

  return false;
}

void ED_operatortypes_io(void)

{
#ifdef WITH_COLLADA
  /* Collada operators: */
  WM_operatortype_append(WM_OT_collada_export);
  WM_operatortype_append(WM_OT_collada_import);
#endif
#ifdef WITH_ALEMBIC
  WM_operatortype_append(WM_OT_alembic_import);
  WM_operatortype_append(WM_OT_alembic_export);
#endif
#ifdef WITH_USD
  WM_operatortype_append(WM_OT_usd_import);
  WM_operatortype_append(WM_OT_usd_export);

  WM_PT_USDExportPanelsRegister();
  WM_PT_USDImportPanelsRegister();
#endif

#ifdef WITH_IO_GPENCIL
  WM_operatortype_append(WM_OT_gpencil_import_svg);
#  ifdef WITH_PUGIXML
  WM_operatortype_append(WM_OT_gpencil_export_svg);
#  endif
#  ifdef WITH_HARU
  WM_operatortype_append(WM_OT_gpencil_export_pdf);
#  endif
#endif

  WM_operatortype_append(CACHEFILE_OT_open);
  WM_operatortype_append(CACHEFILE_OT_reload);

  WM_operatortype_append(CACHEFILE_OT_layer_add);
  WM_operatortype_append(CACHEFILE_OT_layer_remove);
  WM_operatortype_append(CACHEFILE_OT_layer_move);
#ifdef WITH_IO_WAVEFRONT_OBJ
  WM_operatortype_append(WM_OT_obj_export);
  WM_operatortype_append(WM_OT_obj_import);
#endif

#ifdef WITH_IO_PLY
  WM_operatortype_append(WM_OT_ply_export);
  WM_operatortype_append(WM_OT_ply_import);
#endif

#ifdef WITH_IO_STL
  WM_operatortype_append(WM_OT_stl_import);
  WM_operatortype_append(WM_OT_stl_export);
#endif
}
