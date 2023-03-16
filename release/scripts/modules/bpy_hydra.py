# SPDX-License-Identifier: GPL-2.0-or-later

"""
Implementation of class `HydraRenderEngine`.

Render Blender addon with Hydra render delegate should inherit `HydraRenderEngine`.
Example:
```
import bpy_hydra

class CustomHydraRenderEngine(HydraRenderEngine):
    bl_idname = 'CustomHydraRenderEngine'
    bl_label = "Hydra: Custom"
    bl_info = "Hydra Custom render delegate"

    delegate_id = 'HdCustomRendererPlugin'

    @classmethod
    def register(cls):
        super().register()

        bpy_hydra.register_plugins(["/path/to/plugin")], ["/additional/system/path")])

    def get_delegate_settings(self, engine_type):
        return {
            'setting1': 1,
            'setting2': "2",
        }
```
"""

__all__ = (
    "HydraRenderEngine",
    "export_mtlx",
    "register_plugins",
    "get_render_plugins",
)

import bpy
import _bpy_hydra

from _bpy_hydra import register_plugins, get_render_plugins


class HydraRenderEngine(bpy.types.RenderEngine):
    """ Render addon with Hydra render delegate should inherit this class """

    bl_use_shading_nodes_custom = False

    delegate_id = ''
    engine_ptr = None

    def __del__(self):
        if not self.engine_ptr:
            return

        _bpy_hydra.engine_free(self.engine_ptr)

    @classmethod
    def register(cls):
        _bpy_hydra.init()

    @classmethod
    def unregister(cls):
        pass

    def get_delegate_settings(self, engine_type):
        return {}

    # final render
    def update(self, data, depsgraph):
        pass

    def render(self, depsgraph):
        engine_type = 'PREVIEW' if self.is_preview else 'FINAL'

        self.engine_ptr = _bpy_hydra.engine_create(self.as_pointer(), engine_type, self.delegate_id)
        delegate_settings = self.get_delegate_settings(engine_type)

        _bpy_hydra.engine_sync(self.engine_ptr, depsgraph.as_pointer(), bpy.context.as_pointer(), delegate_settings)
        _bpy_hydra.engine_render(self.engine_ptr, depsgraph.as_pointer())

    # viewport render
    def view_update(self, context, depsgraph):
        if not self.engine_ptr:
            self.engine_ptr = _bpy_hydra.engine_create(self.as_pointer(), 'VIEWPORT', self.delegate_id)

        delegate_settings = self.get_delegate_settings('VIEWPORT')
        _bpy_hydra.engine_sync(self.engine_ptr, depsgraph.as_pointer(), context.as_pointer(), delegate_settings)

    def view_draw(self, context, depsgraph):
        if not self.engine_ptr:
            return

        _bpy_hydra.engine_view_draw(self.engine_ptr, depsgraph.as_pointer(), context.as_pointer())


def export_mtlx(material):
    """ Exports material to .mtlx file. It is called from Blender source code. """
    try:
        import materialx.utils as mx_utils

        doc = mx_utils.export(material, None)
        if not doc:
            return ""

        mtlx_file = mx_utils.get_temp_file(".mtlx", material.name)
        mx_utils.export_to_file(doc, mtlx_file, export_deps=True, copy_deps=False)
        return str(mtlx_file)

    except ImportError:
        print("ERROR: no MaterialX addon available")

    return ""
