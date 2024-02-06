# SPDX-FileCopyrightText: 2009-2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.types import (
    Operator, OperatorFileListElement, FileHandler,)
from bpy.props import (
    BoolProperty,
    CollectionProperty,
    StringProperty,)
from bpy.app.translations import pgettext_rpt as rpt_


class EditExternally(Operator):
    """Edit image in an external application"""
    bl_idname = "image.external_edit"
    bl_label = "Image Edit Externally"
    bl_options = {'REGISTER'}

    filepath: StringProperty(
        subtype='FILE_PATH',
    )

    @staticmethod
    def _editor_guess(context):
        import sys

        image_editor = context.preferences.filepaths.image_editor

        # use image editor in the preferences when available.
        if not image_editor:
            if sys.platform[:3] == "win":
                image_editor = ["start"]  # not tested!
            elif sys.platform == "darwin":
                image_editor = ["open"]
            else:
                image_editor = ["gimp"]
        else:
            if sys.platform == "darwin":
                # blender file selector treats .app as a folder
                # and will include a trailing backslash, so we strip it.
                image_editor.rstrip('\\')
                image_editor = ["open", "-a", image_editor]
            else:
                image_editor = [image_editor]

        return image_editor

    def execute(self, context):
        import os
        import subprocess

        filepath = self.filepath

        if not filepath:
            self.report({'ERROR'}, "Image path not set")
            return {'CANCELLED'}

        if not os.path.exists(filepath) or not os.path.isfile(filepath):
            self.report({'ERROR'},
                        rpt_("Image path %r not found, image may be packed or "
                             "unsaved") % filepath)
            return {'CANCELLED'}

        cmd = self._editor_guess(context) + [filepath]

        try:
            subprocess.Popen(cmd)
        except BaseException:
            import traceback
            traceback.print_exc()
            self.report({'ERROR'},
                        "Image editor could not be launched, ensure that "
                        "the path in User Preferences > File is valid, and Blender has rights to launch it")

            return {'CANCELLED'}

        return {'FINISHED'}

    def invoke(self, context, _event):
        import os
        sd = context.space_data
        try:
            image = sd.image
        except AttributeError:
            self.report({'ERROR'}, "Context incorrect, image not found")
            return {'CANCELLED'}

        if image.packed_file:
            self.report({'ERROR'}, "Image is packed, unpack before editing")
            return {'CANCELLED'}

        if sd.type == 'IMAGE_EDITOR':
            filepath = image.filepath_from_user(image_user=sd.image_user)
        else:
            filepath = image.filepath

        filepath = bpy.path.abspath(filepath, library=image.library)

        self.filepath = os.path.normpath(filepath)
        self.execute(context)

        return {'FINISHED'}


class ProjectEdit(Operator):
    """Edit a snapshot of the 3D Viewport in an external image editor"""
    bl_idname = "image.project_edit"
    bl_label = "Project Edit"
    bl_options = {'REGISTER'}

    _proj_hack = [""]

    def execute(self, context):
        import os

        EXT = "png"  # could be made an option but for now ok

        for image in bpy.data.images:
            image.tag = True

        # opengl buffer may fail, we can't help this, but best report it.
        try:
            bpy.ops.paint.image_from_view()
        except RuntimeError as ex:
            self.report({'ERROR'}, str(ex))
            return {'CANCELLED'}

        image_new = None
        for image in bpy.data.images:
            if not image.tag:
                image_new = image
                break

        if not image_new:
            self.report({'ERROR'}, "Could not make new image")
            return {'CANCELLED'}

        filepath = os.path.basename(bpy.data.filepath)
        filepath = os.path.splitext(filepath)[0]
        # fixes <memory> rubbish, needs checking
        # filepath = bpy.path.clean_name(filepath)

        if bpy.data.is_saved:
            filepath = "//" + filepath
        else:
            filepath = os.path.join(bpy.app.tempdir, "project_edit")

        obj = context.object

        if obj:
            filepath += "_" + bpy.path.clean_name(obj.name)

        filepath_final = filepath + "." + EXT
        i = 0

        while os.path.exists(bpy.path.abspath(filepath_final)):
            filepath_final = filepath + ("%.3d.%s" % (i, EXT))
            i += 1

        image_new.name = bpy.path.basename(filepath_final)
        ProjectEdit._proj_hack[0] = image_new.name

        image_new.filepath_raw = filepath_final  # TODO, filepath raw is crummy
        image_new.file_format = 'PNG'
        image_new.save()

        filepath_final = bpy.path.abspath(filepath_final)

        try:
            bpy.ops.image.external_edit(filepath=filepath_final)
        except RuntimeError as ex:
            self.report({'ERROR'}, str(ex))

        return {'FINISHED'}


class ProjectApply(Operator):
    """Project edited image back onto the object"""
    bl_idname = "image.project_apply"
    bl_label = "Project Apply"
    bl_options = {'REGISTER'}

    def execute(self, _context):
        image_name = ProjectEdit._proj_hack[0]  # TODO, deal with this nicer
        image = bpy.data.images.get((image_name, None))
        if image is None:
            self.report({'ERROR'}, rpt_("Could not find image '%s'") % image_name)
            return {'CANCELLED'}

        image.reload()
        bpy.ops.paint.project_image(image=image_name)

        return {'FINISHED'}


class IMAGE_OT_open_images(Operator):
    bl_idname = "image.open_images"
    bl_label = "Open Images"
    bl_options = {'REGISTER', 'UNDO'}

    directory: StringProperty(subtype='FILE_PATH', options={'SKIP_SAVE', 'HIDDEN'})
    files: CollectionProperty(type=OperatorFileListElement, options={'SKIP_SAVE', 'HIDDEN'})
    relative_path: BoolProperty(name="Use relative path", default=True)
    use_sequence_detection: BoolProperty(name="Use sequence detection", default=True)
    use_udim_detection: BoolProperty(name="Use UDIM detection", default=True)

    @classmethod
    def poll(cls, context):
        return (context.area and context.area.ui_type == 'IMAGE_EDITOR'
                and context.region and context.region.type == 'WINDOW')

    def execute(self, context):
        if not self.directory or len(self.files) == 0:
            return {'CANCELLED'}
        # List of files that are not part of an image sequence or UDIM group
        files = []
        # Groups of files that may be part of an image sequence or a UDIM group.
        # The elements looks like [(prefix,ext,frame_size,[file.name,...]),...]
        sequences = []
        for file in self.files:
            import re
            match = re.search("\\.[\\w\\d]+$", file.name)
            # Filter by extension
            if not match or not (match.group(0) in bpy.path.extensions_image or match.group(0)
                                 in bpy.path.extensions_movie):
                continue
            match = re.search("(\\d*)(\\.[\\w\\d]*)$", file.name)
            if match and (self.use_sequence_detection or self.use_udim_detection):
                prefix = file.name[:len(file.name) - len(match.group(0))]
                ext = match.group(2)
                frame_size = len(match.group(1))
                seq = None
                for test_seq in sequences:
                    if test_seq[0] == prefix and test_seq[1] == ext and test_seq[2] == frame_size:
                        seq = test_seq
                        seq[3].append(file.name)
                        break
                if not seq:
                    sequences.append((prefix, ext, frame_size, [file.name,]))
            else:
                files.append(file.name)

        for file in files:
            import os
            filepath = os.path.join(self.directory, file)
            bpy.ops.image.open(filepath=filepath, relative_path=self.relative_path)
        for seq in sequences:
            seq[3].sort()
            import os
            directory = self.directory
            filepath = os.path.join(directory, seq[3][0])
            files = [{"name": file} for file in seq[3]]
            bpy.ops.image.open(
                filepath=filepath,
                directory=directory,
                files=files,
                use_sequence_detection=self.use_sequence_detection,
                use_udim_detecting=self.use_udim_detection,
                relative_path=self.relative_path)
            if len(files) > 1 and self.use_sequence_detection:
                context.edit_image.name = "{prefix}{hash}{ext}".format(prefix=seq[0], hash=("#" * seq[2]), ext=seq[1])

        return {'FINISHED'}


class IMAGE_FH_drop_handler(FileHandler):
    bl_idname = "IMAGE_FH_drop_handler"
    bl_label = "Open droped imagenes"
    bl_import_operator = "image.open_images"
    bl_file_extensions = ';'.join(bpy.path.extensions_image) + ';'.join(bpy.path.extensions_movie)

    @classmethod
    def poll_drop(cls, context):
        return (context.area and context.area.ui_type == 'IMAGE_EDITOR'
                and context.region and context.region.type == 'WINDOW')


classes = (
    EditExternally,
    ProjectApply,
    IMAGE_OT_open_images,
    IMAGE_FH_drop_handler,
    ProjectEdit,
)
