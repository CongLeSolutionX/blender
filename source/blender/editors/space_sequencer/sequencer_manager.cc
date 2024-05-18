/* SPDX-FileCopyrightText: 2024 Blender Authors
*
* SPDX-License-Identifier: GPL-2.0-or-later */
#include "BLI_path_util.h"
#include "BKE_context.h"
#include "BKE_report.h"
#include "DNA_screen_types.h"
#include "DNA_space_types.h"
#include "ED_screen.h"
#include "UI_interface.h"
#include "UI_resources.h"
#include "WM_api.h"
#include "WM_types.h"

/* Operator for selecting text files */
static int open_text_file_exec(bContext *C, wmOperator *op)
{
    char filepath[FILE_MAX];
    RNA_string_get(op->ptr, "filepath", filepath);
    BKE_reportf(op->reports, RPT_INFO, "Selected text file: %s", filepath);
    return OPERATOR_FINISHED;
}

static void open_text_file(wmOperatorType *ot)
{
    ot->name = "Open Text File";
    ot->idname = "FILE_OT_open_text_file";
    ot->description = "Select a text file";

    ot->exec = open_text_file_exec;
    ot->poll = ED_operator_areaactive;

    WM_operator_properties_filesel(ot, FILE_TYPE_FOLDER | FILE_TYPE_TEXT, FILE_SPECIAL, FILE_OPENFILE, WM_FILESEL_FILEPATH, FILE_DEFAULTDISPLAY);
}

/* Operator for selecting video files */
static int open_video_file_exec(bContext *C, wmOperator *op)
{
    char filepath[FILE_MAX];
    RNA_string_get(op->ptr, "filepath", filepath);
    BKE_reportf(op->reports, RPT_INFO, "Selected video file: %s", filepath);
    return OPERATOR_FINISHED;
}

static void open_video_file(wmOperatorType *ot)
{
    ot->name = "Open Video File";
    ot->idname = "FILE_OT_open_video_file";
    ot->description = "Select a video file";

    ot->exec = open_video_file_exec;
    ot->poll = ED_operator_areaactive;

    WM_operator_properties_filesel(ot, FILE_TYPE_FOLDER | FILE_TYPE_MOVIE, FILE_SPECIAL, FILE_OPENFILE, WM_FILESEL_FILEPATH, FILE_DEFAULTDISPLAY);
}

/* Operator for selecting sound files */
static int open_sound_file_exec(bContext *C, wmOperator *op)
{
    char filepath[FILE_MAX];
    RNA_string_get(op->ptr, "filepath", filepath);
    BKE_reportf(op->reports, RPT_INFO, "Selected sound file: %s", filepath);
    return OPERATOR_FINISHED;
}

static void open_sound_file(wmOperatorType *ot)
{
    ot->name = "Open Sound File";
    ot->idname = "FILE_OT_open_sound_file";
    ot->description = "Select a sound file";

    ot->exec = open_sound_file_exec;
    ot->poll = ED_operator_areaactive;

    WM_operator_properties_filesel(ot, FILE_TYPE_FOLDER | FILE_TYPE_SOUND, FILE_SPECIAL, FILE_OPENFILE, WM_FILESEL_FILEPATH, FILE_DEFAULTDISPLAY);
}

/* Operator for selecting image files */
static int open_image_file_exec(bContext *C, wmOperator *op)
{
    char filepath[FILE_MAX];
    RNA_string_get(op->ptr, "filepath", filepath);
    BKE_reportf(op->reports, RPT_INFO, "Selected image file: %s", filepath);
    return OPERATOR_FINISHED;
}

static void open_image_file(wmOperatorType *ot)
{
    ot->name = "Open Image File";
    ot->idname = "FILE_OT_open_image_file";
    ot->description = "Select an image file";

    ot->exec = open_image_file_exec;
    ot->poll = ED_operator_areaactive;

    WM_operator_properties_filesel(ot, FILE_TYPE_FOLDER | FILE_TYPE_IMAGE, FILE_SPECIAL, FILE_OPENFILE, WM_FILESEL_FILEPATH, FILE_DEFAULTDISPLAY);
}

static void file_selector_panel_draw(const bContext *C, Panel *panel)
{
    uiLayout *layout = panel->layout;
    uiLayoutSetPropSep(layout, true);
    
    uiItemO(layout, "Select Text File", ICON_FILE_BLEND, "FILE_OT_open_text_file");
    uiItemO(layout, "Select Video File", ICON_FILE_MOVIE, "FILE_OT_open_video_file");
    uiItemO(layout, "Select Sound File", ICON_SOUND, "FILE_OT_open_sound_file");
    uiItemO(layout, "Select Image File", ICON_IMAGE_DATA, "FILE_OT_open_image_file");
}

void file_selector_panel_register(ARegionType *art)
{
    PanelType *pt;

    pt = MEM_callocN(sizeof(PanelType), "spacetype file selector panel");
    strcpy(pt->idname, "SEQUENCER_PT_file_selector");
    strcpy(pt->label, "File Selector");
    strcpy(pt->translation_context, BLT_I18NCONTEXT_DEFAULT_BPYRNA);
    pt->draw = file_selector_panel_draw;
    pt->poll = ED_operator_areaactive;

    BLI_addtail(&art->paneltypes, pt);
}

/* Register the panel in the appropriate space */
void register_file_selector_panel(void)
{
    SpaceType *st = BKE_spacetype_from_id(SPACE_SEQ);
    ARegionType *art = BKE_regiontype_from_id(st, RGN_TYPE_UI);
    file_selector_panel_register(art);
}
