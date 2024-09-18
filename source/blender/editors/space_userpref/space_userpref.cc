/* SPDX-FileCopyrightText: 2008 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spuserpref
 */

#include <cstdio>
#include <cstring>

#include "MEM_guardedalloc.h"

#include "BLI_bitmap.h"
#include "BLI_blenlib.h"
#include "BLI_utildefines.h"

#include "BKE_context.hh"
#include "BKE_screen.hh"

#include "ED_screen.hh"
#include "ED_space_api.hh"
#include "ED_userpref.hh"

#include "RNA_access.hh"
#include "RNA_enum_types.hh"

#include "WM_types.hh"

#include "UI_interface.hh"

#include "BLO_read_write.hh"

#include "userpref_intern.hh"

/* ******************** default callbacks for userpref space ***************** */

static SpaceLink *userpref_create(const ScrArea *area, const Scene * /*scene*/)
{
  ARegion *region;
  SpaceUserPref *spref;

  spref = static_cast<SpaceUserPref *>(MEM_callocN(sizeof(SpaceUserPref), "inituserpref"));
  spref->spacetype = SPACE_USERPREF;

  /* header */
  region = static_cast<ARegion *>(MEM_callocN(sizeof(ARegion), "header for userpref"));

  BLI_addtail(&spref->regionbase, region);
  region->regiontype = RGN_TYPE_HEADER;
  /* Ignore user preference "USER_HEADER_BOTTOM" here (always show bottom for new types). */
  region->alignment = RGN_ALIGN_BOTTOM;

  /* navigation region */
  region = static_cast<ARegion *>(MEM_callocN(sizeof(ARegion), "navigation region for userpref"));

  BLI_addtail(&spref->regionbase, region);
  region->regiontype = RGN_TYPE_NAV_BAR;
  region->alignment = RGN_ALIGN_LEFT;

  /* Use smaller size when opened in area like properties editor. */
  if (area->winx && area->winx < 3.0f * UI_NAVIGATION_REGION_WIDTH * UI_SCALE_FAC) {
    region->sizex = UI_NARROW_NAVIGATION_REGION_WIDTH;
  }

  /* execution region */
  region = static_cast<ARegion *>(MEM_callocN(sizeof(ARegion), "execution region for userpref"));

  BLI_addtail(&spref->regionbase, region);
  region->regiontype = RGN_TYPE_EXECUTE;
  region->alignment = RGN_ALIGN_BOTTOM | RGN_SPLIT_PREV;
  region->flag |= RGN_FLAG_DYNAMIC_SIZE | RGN_FLAG_NO_USER_RESIZE;

  /* main region */
  region = static_cast<ARegion *>(MEM_callocN(sizeof(ARegion), "main region for userpref"));

  BLI_addtail(&spref->regionbase, region);
  region->regiontype = RGN_TYPE_WINDOW;

  return (SpaceLink *)spref;
}

/* Doesn't free the space-link itself. */
static void userpref_free(SpaceLink *sl)
{
  SpaceUserPref *spref = (SpaceUserPref *)sl;
  if (spref->runtime != nullptr) {
    MEM_SAFE_FREE(spref->runtime->tab_search_results);
    MEM_freeN(spref->runtime);
  }
}

/* spacetype; init callback */
static void userpref_init(wmWindowManager * /*wm*/, ScrArea *area)
{
  SpaceUserPref *spref = (SpaceUserPref *)area->spacedata.first;

  if (spref->runtime == nullptr) {
    spref->runtime = static_cast<SpaceUserPref_Runtime *>(
        MEM_mallocN(sizeof(SpaceUserPref_Runtime), __func__));
    spref->runtime->search_string[0] = '\0';
    spref->runtime->tab_search_results = BLI_BITMAP_NEW(BCONTEXT_TOT * 2, __func__);
  }
}

static SpaceLink *userpref_duplicate(SpaceLink *sl)
{
  SpaceUserPref *sprefn_old = (SpaceUserPref *)sl;
  SpaceUserPref *sprefn = static_cast<SpaceUserPref *>(MEM_dupallocN(sl));

  if (sprefn_old->runtime != nullptr) {
    sprefn->runtime = static_cast<SpaceUserPref_Runtime *>(MEM_dupallocN(sprefn_old->runtime));
    sprefn->runtime->search_string[0] = '\0';
    sprefn->runtime->tab_search_results = BLI_BITMAP_NEW(BCONTEXT_TOT, __func__);
  }

  /* clear or remove stuff from old */

  return (SpaceLink *)sprefn;
}

/* add handlers, stuff you only do once or on area/region changes */
static void userpref_main_region_init(wmWindowManager *wm, ARegion *region)
{
  /* do not use here, the properties changed in user-preferences do a system-wide refresh,
   * then scroller jumps back */
  // region->v2d.flag &= ~V2D_IS_INIT;

  region->v2d.scroll = V2D_SCROLL_RIGHT | V2D_SCROLL_VERTICAL_HIDE;

  ED_region_panels_init(wm, region);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Userpref Search Access API
 * \{ */

const char *ED_userpref_search_string_get(SpaceUserPref *spref)
{
  return spref->runtime->search_string;
}

int ED_userpref_search_string_length(SpaceUserPref *spref)
{
  return BLI_strnlen(spref->runtime->search_string, sizeof(spref->runtime->search_string));
}

void ED_userpref_search_string_set(SpaceUserPref *spref, const char *value)
{
  STRNCPY(spref->runtime->search_string, value);
}

bool ED_userpref_tab_has_search_result(SpaceUserPref *spref, const int index)
{
  return BLI_BITMAP_TEST(spref->runtime->tab_search_results, index);
}

/** \} */

blender::Vector<eUserPref_Section> ED_userpref_tabs_list(SpaceUserPref * /*prefs*/)
{
  blender::Vector<eUserPref_Section> result;
  const EnumPropertyItem *items = rna_enum_preference_section_items;
  for (const EnumPropertyItem *it = rna_enum_preference_section_items; it->identifier != nullptr;
       it++)
  {
    if (it->name) {
      result.append(eUserPref_Section(it->value));
    }
  }
  return result;
}

/* -------------------------------------------------------------------- */
/** \name "Off Screen" Layout Generation for Userpref Search
 * \{ */

static bool property_search_for_context(const bContext *C,
                                        ARegion *region,
                                        SpaceUserPref *sprefs,
                                        short section)
{
  char lower[64];
  const char *name = nullptr;
  RNA_enum_id_from_value(rna_enum_preference_section_items, section, &name);
  STRNCPY(lower, name);
  BLI_str_tolower_ascii(lower, strlen(lower));

  const char *contexts[2] = {lower, nullptr};
  return ED_region_property_search(C, region, &region->type->paneltypes, contexts, nullptr);
}

static void userpref_search_move_to_next_tab_with_results(
    SpaceUserPref *sbuts, const blender::Span<eUserPref_Section> context_tabs_array)
{
  int current_tab_index = 0;
  for (const int i : context_tabs_array.index_range()) {
    if (U.space_data.section_active == context_tabs_array[i]) {
      current_tab_index = i;
      break;
    }
  }

  /* Try the tabs after the current tab. */
  for (int i = current_tab_index + 1; i < context_tabs_array.size(); i++) {
    if (BLI_BITMAP_TEST(sbuts->runtime->tab_search_results, i)) {
      U.space_data.section_active = context_tabs_array[i];
      return;
    }
  }

  /* Try the tabs before the current tab. */
  for (int i = 0; i < current_tab_index; i++) {
    if (BLI_BITMAP_TEST(sbuts->runtime->tab_search_results, i)) {
      U.space_data.section_active = context_tabs_array[i];
      return;
    }
  }
}

static void userpref_search_all_tabs(const bContext *C,
                                     SpaceUserPref *sprefs,
                                     ARegion *region_original,
                                     const blender::Span<eUserPref_Section> context_tabs_array)
{
  /* Use local copies of the area and duplicate the region as a mainly-paranoid protection
   * against changing any of the space / region data while running the search. */
  ScrArea *area_original = CTX_wm_area(C);
  ScrArea area_copy = blender::dna::shallow_copy(*area_original);
  ARegion *region_copy = BKE_area_region_copy(area_copy.type, region_original);
  /* Set the region visible field. Otherwise some layout code thinks we're drawing in a popup.
   * This likely isn't necessary, but it's nice to emulate a "real" region where possible. */
  region_copy->visible = true;
  CTX_wm_area_set((bContext *)C, &area_copy);
  CTX_wm_region_set((bContext *)C, region_copy);

  SpaceUserPref sprefs_copy = blender::dna::shallow_copy(*sprefs);
  sprefs_copy.runtime = static_cast<SpaceUserPref_Runtime *>(MEM_dupallocN(sprefs->runtime));
  sprefs_copy.runtime->tab_search_results = nullptr;
  BLI_listbase_clear(&area_copy.spacedata);
  BLI_addtail(&area_copy.spacedata, &sprefs_copy);

  /* Loop through the tabs. */
  for (const int i : context_tabs_array.index_range()) {
    /* -1 corresponds to a spacer. */
    if (context_tabs_array[i] == -1) {
      continue;
    }

    /* Handle search for the current tab in the normal layout pass. */
    if (context_tabs_array[i] == U.space_data.section_active) {
      continue;
    }

    /* Actually do the search and store the result in the bitmap. */
    const bool found = property_search_for_context(
        C, region_copy, &sprefs_copy, context_tabs_array[i]);
    BLI_BITMAP_SET(sprefs->runtime->tab_search_results, i, found);

    UI_blocklist_free(C, region_copy);
  }

  BKE_area_region_free(area_copy.type, region_copy);
  MEM_freeN(region_copy);
  userpref_free((SpaceLink *)&sprefs_copy);

  CTX_wm_area_set((bContext *)C, area_original);
  CTX_wm_region_set((bContext *)C, region_original);
}

/**
 * Handle userpref search for the layout pass, including finding which tabs have
 * search results and switching if the current tab doesn't have a result.
 */
static void userpref_main_region_property_search(const bContext *C,
                                                 SpaceUserPref *sprefs,
                                                 ARegion *region)
{
  blender::Vector<eUserPref_Section> tabs = ED_userpref_tabs_list(sprefs);
  BLI_bitmap_set_all(sprefs->runtime->tab_search_results, false, tabs.size());

  userpref_search_all_tabs(C, sprefs, region, tabs);

  /* Check whether the current tab has a search match. */
  bool current_tab_has_search_match = false;
  LISTBASE_FOREACH (Panel *, panel, &region->panels) {
    if (UI_panel_is_active(panel) && UI_panel_matches_search_filter(panel)) {
      current_tab_has_search_match = true;
    }
  }

  /* Find which index in the list the current tab corresponds to. */
  int current_tab_index = -1;
  for (const int i : tabs.index_range()) {
    if (tabs[i] == U.space_data.section_active) {
      current_tab_index = i;
    }
  }
  BLI_assert(current_tab_index != -1);

  /* Update the tab search match flag for the current tab. */
  BLI_BITMAP_SET(
      sprefs->runtime->tab_search_results, current_tab_index, current_tab_has_search_match);

  /* Move to the next tab with a result */
  if (!current_tab_has_search_match) {
    if (region->flag & RGN_FLAG_SEARCH_FILTER_UPDATE) {
      userpref_search_move_to_next_tab_with_results(sprefs, tabs);
    }
    else {
      U.space_data.section_active = 0;
    }
  }
}

/** \} */

static void userpref_main_region_layout(const bContext *C, ARegion *region)
{
  char id_lower[64];
  const char *contexts[2] = {id_lower, nullptr};
  SpaceUserPref *spref = CTX_wm_space_userpref(C);

  /* Avoid duplicating identifiers, use existing RNA enum. */
  {
    const EnumPropertyItem *items = rna_enum_preference_section_items;
    int i = RNA_enum_from_value(items, U.space_data.section_active);
    /* File is from the future. */
    if (i == -1) {
      i = 0;
    }
    const char *id = items[i].identifier;
    BLI_assert(strlen(id) < sizeof(id_lower));
    STRNCPY(id_lower, id);
    BLI_str_tolower_ascii(id_lower, strlen(id_lower));
  }

  ED_region_panels_layout_ex(
      C, region, &region->type->paneltypes, WM_OP_INVOKE_REGION_WIN, contexts, nullptr);

  if (region->flag & RGN_FLAG_SEARCH_FILTER_ACTIVE) {
    userpref_main_region_property_search(C, spref, region);
  }
}

static void userpref_operatortypes() {}

static void userpref_keymap(wmKeyConfig * /*keyconf*/) {}

/* add handlers, stuff you only do once or on area/region changes */
static void userpref_header_region_init(wmWindowManager * /*wm*/, ARegion *region)
{
  ED_region_header_init(region);
}

static void userpref_header_region_draw(const bContext *C, ARegion *region)
{
  ED_region_header(C, region);
}

/* add handlers, stuff you only do once or on area/region changes */
static void userpref_navigation_region_init(wmWindowManager *wm, ARegion *region)
{
  region->v2d.scroll = V2D_SCROLL_RIGHT | V2D_SCROLL_VERTICAL_HIDE;

  ED_region_panels_init(wm, region);
}

static void userpref_navigation_region_draw(const bContext *C, ARegion *region)
{
  ED_region_panels(C, region);
}

static bool userpref_execute_region_poll(const RegionPollParams *params)
{
  const ARegion *region_header = BKE_area_find_region_type(params->area, RGN_TYPE_HEADER);
  return !region_header->visible;
}

/* add handlers, stuff you only do once or on area/region changes */
static void userpref_execute_region_init(wmWindowManager *wm, ARegion *region)
{
  ED_region_panels_init(wm, region);
  region->v2d.keepzoom |= V2D_LOCKZOOM_X | V2D_LOCKZOOM_Y;
}

static void userpref_main_region_listener(const wmRegionListenerParams * /*params*/) {}

static void userpref_header_listener(const wmRegionListenerParams * /*params*/) {}

static void userpref_navigation_region_listener(const wmRegionListenerParams * /*params*/) {}

static void userpref_execute_region_listener(const wmRegionListenerParams * /*params*/) {}

static void userpref_space_blend_write(BlendWriter *writer, SpaceLink *sl)
{
  BLO_write_struct(writer, SpaceUserPref, sl);
}

void ED_spacetype_userpref()
{
  std::unique_ptr<SpaceType> st = std::make_unique<SpaceType>();
  ARegionType *art;

  st->spaceid = SPACE_USERPREF;
  STRNCPY(st->name, "Userpref");

  st->create = userpref_create;
  st->free = userpref_free;
  st->init = userpref_init;
  st->duplicate = userpref_duplicate;
  st->operatortypes = userpref_operatortypes;
  st->keymap = userpref_keymap;
  st->blend_write = userpref_space_blend_write;

  /* regions: main window */
  art = static_cast<ARegionType *>(MEM_callocN(sizeof(ARegionType), "spacetype userpref region"));
  art->regionid = RGN_TYPE_WINDOW;
  art->init = userpref_main_region_init;
  art->layout = userpref_main_region_layout;
  art->draw = ED_region_panels_draw;
  art->listener = userpref_main_region_listener;
  art->keymapflag = ED_KEYMAP_UI;

  BLI_addhead(&st->regiontypes, art);

  /* regions: header */
  art = static_cast<ARegionType *>(MEM_callocN(sizeof(ARegionType), "spacetype userpref region"));
  art->regionid = RGN_TYPE_HEADER;
  art->prefsizey = HEADERY;
  art->keymapflag = ED_KEYMAP_UI | ED_KEYMAP_VIEW2D | ED_KEYMAP_HEADER;
  art->listener = userpref_header_listener;
  art->init = userpref_header_region_init;
  art->draw = userpref_header_region_draw;

  BLI_addhead(&st->regiontypes, art);

  /* regions: navigation window */
  art = static_cast<ARegionType *>(MEM_callocN(sizeof(ARegionType), "spacetype userpref region"));
  art->regionid = RGN_TYPE_NAV_BAR;
  art->prefsizex = UI_NAVIGATION_REGION_WIDTH;
  art->init = userpref_navigation_region_init;
  art->draw = userpref_navigation_region_draw;
  art->listener = userpref_navigation_region_listener;
  art->keymapflag = ED_KEYMAP_UI | ED_KEYMAP_NAVBAR;

  BLI_addhead(&st->regiontypes, art);

  /* regions: execution window */
  art = static_cast<ARegionType *>(MEM_callocN(sizeof(ARegionType), "spacetype userpref region"));
  art->regionid = RGN_TYPE_EXECUTE;
  art->prefsizey = HEADERY;
  art->poll = userpref_execute_region_poll;
  art->init = userpref_execute_region_init;
  art->layout = ED_region_panels_layout;
  art->draw = ED_region_panels_draw;
  art->listener = userpref_execute_region_listener;
  art->keymapflag = ED_KEYMAP_UI;

  BLI_addhead(&st->regiontypes, art);

  BKE_spacetype_register(std::move(st));
}
