# SPDX-FileCopyrightText: 2011-2022 Blender Foundation
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import bpy
import _cycles

from bpy.app.translations import pgettext_tip as tip_


def osl_compile(input_path, report):
    """compile .osl file with given filepath to temporary .oso file"""
    import tempfile
    output_file = tempfile.NamedTemporaryFile(mode='w', suffix=".oso", delete=False)
    output_path = output_file.name
    output_file.close()

    ok = _cycles.osl_compile(input_path, output_path)

    if ok:
        report({'INFO'}, "OSL shader compilation succeeded")

    return ok, output_path


def update_external_script(report, filepath, library):
    """compile and update OSL script"""
    import os
    import shutil

    oso_file_remove = False

    script_path = bpy.path.abspath(filepath, library=library)
    script_path_noext, script_ext = os.path.splitext(script_path)

    if script_ext == ".oso":
        # it's a .oso file, no need to compile
        ok, oso_path = True, script_path
    elif script_ext == ".osl":
        # compile .osl file
        ok, oso_path = osl_compile(script_path, report)
        oso_file_remove = True

        if ok:
            # copy .oso from temporary path to .osl directory
            dst_path = script_path_noext + ".oso"
            try:
                shutil.copy2(oso_path, dst_path)
            except:
                report({'ERROR'}, "Failed to write .oso file next to external .osl file at " + dst_path)
    elif os.path.dirname(filepath) == "":
        # module in search path
        oso_path = filepath
        ok = True
    else:
        # unknown
        report({'ERROR'}, "External shader script must have .osl or .oso extension, or be a module name")
        ok = False

    return ok, oso_path, oso_file_remove


def update_internal_script(report, script):
    """compile and update shader script node"""
    import os
    import tempfile

    bytecode = None

    osl_path = bpy.path.abspath(script.filepath, library=script.library)

    if script.is_in_memory or script.is_dirty or script.is_modified or not os.path.exists(osl_path):
        # write text datablock contents to temporary file
        osl_file = tempfile.NamedTemporaryFile(mode='w', suffix=".osl", delete=False)
        osl_file.write(script.as_string())
        osl_file.write("\n")
        osl_file.close()

        ok, oso_path = osl_compile(osl_file.name, report)
        os.remove(osl_file.name)
    else:
        # compile text datablock from disk directly
        ok, oso_path = osl_compile(osl_path, report)

    if ok:
        # read bytecode
        try:
            oso = open(oso_path, 'r')
            bytecode = oso.read()
            oso.close()
        except:
            import traceback
            traceback.print_exc()

            report({'ERROR'}, "Can't read OSO bytecode to store in node at %r" % oso_path)
            ok = False

    return ok, oso_path, bytecode


def update_script_node(node, report):
    """compile and update shader script node"""
    import os

    oso_file_remove = False

    if node.mode == 'EXTERNAL':
        # compile external script file
        ok, oso_path, oso_file_remove = update_external_script(report, node.filepath, node.id_data.library)

    elif node.mode == 'INTERNAL' and node.script:
        # internal script, we will store bytecode in the node
        ok, oso_path, bytecode = update_internal_script(report, node.script)
        if bytecode:
            node.bytecode = bytecode

    else:
        report({'WARNING'}, "No text or file specified in node, nothing to compile")
        return

    if ok:
        # now update node with new sockets
        data = bpy.data.as_pointer()
        ok = _cycles.osl_update_node(data, node.id_data.as_pointer(), node.as_pointer(), oso_path)

        if not ok:
            report({'ERROR'}, tip_("OSL query failed to open %s") % oso_path)
    else:
        report({'ERROR'}, "OSL script compilation failed, see console for errors")

    # remove temporary oso file
    if oso_file_remove:
        try:
            os.remove(oso_path)
        except:
            pass

    return ok


def update_camera_script(cam, report):
    """compile and update camera script"""
    import os
    import idprop
    import hashlib

    oso_file_remove = False

    ccam = cam.cycles
    if ccam.script_mode == 'EXTERNAL':
        # compile external script file
        ok, oso_path, oso_file_remove = update_external_script(report, ccam.script_path, cam.library)

    elif ccam.script_mode == 'INTERNAL' and ccam.script:
        # internal script, we will store bytecode in the node
        ok, oso_path, bytecode = update_internal_script(report, ccam.script)
        if bytecode:
            md5 = hashlib.md5(usedforsecurity=False)
            md5.update(bytecode.encode())
            ccam.script_bytecode = bytecode
            ccam.script_bytecode_hash = md5.hexdigest()
            cam.update_tag()

    else:
        report({'WARNING'}, "No text or file specified in node, nothing to compile")
        return

    if ok:
        # now update node with new sockets
        params = _cycles.osl_get_params(oso_path)
        ok = params is not None

        if not ok:
            report({'ERROR'}, tip_("OSL query failed to open %s") % oso_path)
        else:
            used_params = set()
            for datatype, semantics, name, is_output, is_closure, default, meta in params:
                if is_output or is_closure:
                    continue

                # Get metadata for the parameter to control UI display
                meta_data = {k: v[0] for _, _, k, _, _, v, _ in meta}
                if 'label' not in meta_data:
                    meta_data['label'] = name

                # OSl doesn't have boolean as a type, but we do
                if datatype == 'int' and meta_data.get('widget') in ('boolean', 'checkBox'):
                    datatype = 'bool'
                    default = [bool(v) for v in default]

                name = 'script_param_' + name
                used_params.add(name)

                if name in ccam:
                    # If the parameter already exists, only reset its value if its type
                    # or array length changed
                    cur_data = ccam[name]
                    if isinstance(cur_data, idprop.types.IDPropertyArray):
                        cur_length = len(cur_data)
                        cur_data = cur_data[0]
                    else:
                        cur_length = 1
                    cur_type = {float: 'float', int: 'int', bool: 'bool', str: 'string'}.get(type(cur_data))
                    do_replace = datatype != cur_type or len(default) != cur_length
                else:
                    # Parameter doesn't exist yet, so set it from the defaults
                    do_replace = True

                if do_replace:
                    ccam[name] = tuple(default) if len(default) > 1 else default[0]

                ui = ccam.id_properties_ui(name)
                ui.clear()

                # Determine subtype (no unit support for now)
                if semantics == 'color':
                    ui.update(subtype='COLOR')
                elif meta_data.get('slider'):
                    ui.update(subtype='FACTOR')

                # Map OSL metadata to Blender names
                option_map = {
                    'label': 'name', 'help': 'description',
                    'sensitivity': 'step', 'digits': 'precision',
                    'min': 'min', 'max': 'max',
                    'slidermin': 'soft_min', 'slidermax': 'soft_max',
                }
                for option, value in meta_data.items():
                    if option in option_map:
                        ui.update(**{option_map[option]: value})

        # Clean up unused parameters
        for prop in list(ccam.keys()):
            if prop.startswith('script_param_') and prop not in used_params:
                del ccam[prop]
    else:
        report({'ERROR'}, "OSL script compilation failed, see console for errors")

    # remove temporary oso file
    if oso_file_remove:
        try:
            os.remove(oso_path)
        except:
            pass

    return ok
