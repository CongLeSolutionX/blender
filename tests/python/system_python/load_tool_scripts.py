# SPDX-FileCopyrightText: 2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later


import os
import sys
import pathlib
import importlib.util
import contextlib
import traceback


# For some reasons, these two modules do not work with 'file-path-based' import method used by this script.
# So 'pre-load' them here instead.
# They are _not used_ directly  by this script.
import multiprocessing
import typing


# Directories, relative the to given `--root-dir` argument.
EXCLUDED_DIRS = {
    # Theses folders should only contain scripts designed to work within Blender's python.
    "extern",
    "intern",
    "lib",
    "scripts",
    "source",
    "tests",
    # Not relevant, never expected to be used on buildbots or similar systems.
    "tools/utils_ide",
    "tools/debug",
    "tools/triage",
    # Remanants from history, can still be present in some cases, just ignore.
    "release/scripts/addons",
    "release/scripts/addons_contrib",
}

# Paths to python modules, relative the to given `--root-dir` argument.
EXCLUDED_FILE_PATHS = {
    # self module.
    "tests/python/system_python/load_tool_scripts.py",

    # Will try to open a GUI.
    "tools/utils/make_cursor_gui.py",

    # Require `bpy` module.
    "tools/check_source/check_descriptions.py",
    "tools/utils/make_shape_2d_from_blend.py",
    "tools/utils_maintenance/blender_menu_search_coverage.py",
    "tools/utils_maintenance/blender_update_themes.py",
    "doc/python_api/sphinx_doc_gen.py",
    "release/datafiles/blender_icons_geom.py",

    # Require `gdb`  module.
    "tools/utils/gdb_struct_repr_c99.py",

    # Require `clang` module.
    "build_files/cmake/cmake_static_check_clang.py",

    # Require `yarl` module.
    "tools/utils/gitea_inactive_developers.py",

    # Require `enchant` module.
    "tools/check_source/check_spelling.py",

    # Require `requests` module.
    "release/lts/lts_issue.py",

    # XXX Also manipulates `sys.path`, fails to import.
    "tools/utils_maintenance/cmake_sort_filelists.py",

    # XXX These scripts execute on import! bad, need to be fixed or removed.
    # FIXME: Should be reasonably trivial to fix/cleanup for most of them.
    "tools/check_source/check_unused_defines.py",
    "tools/utils/autopep8_clean.py",
    "tools/utils/blender_merge_format_changes.py",
    "tools/utils/git_log_review_commits.py",
    "tools/utils_maintenance/c_sort_blocks.py",
    "tools/utils_maintenance/c_struct_clean.py",
    "tools/utils_doc/rna_manual_reference_updater.py",
    "tools/git/git_sh1_to_svn_rev.py",
    "build_files/cmake/cmake_netbeans_project.py",
    "build_files/cmake/clang_array_check.py",
    "build_files/package_spec/build_archive.py",
    "build_files/utils/make_test.py",
    "release/datafiles/blender_icons_geom_update.py",
    "release/datafiles/ctodata.py",
    "release/pypi/upload-release.py",
    "release/lts/create_release_notes.py",
    "doc/python_api/conf.py",
}


@contextlib.contextmanager
def add_to_sys_path(syspath):
    old_path = sys.path
    old_modules = sys.modules
    sys.modules = old_modules.copy()
    sys.path = sys.path[:]
    sys.path.insert(0, str(syspath))
    try:
        yield
    finally:
        sys.path = old_path
        sys.modules = old_modules


def import_module(file_path):
    """Returns `...` if not a python module, `True` if it's a package, `False` otherwise."""
    file_path = pathlib.Path(file_path)
    if file_path.suffix != ".py":
        return ...
    file_name = file_path.name
    if not file_name:
        return ...
    is_package = file_name == "__init__.py"
    if is_package:
        assert (len(file_path.parts) >= 2)
        module_name = file_path.parts[-2]
    else:
        module_name = file_path.stem

    with add_to_sys_path(file_path.parent):
        # ~ print(f"+++ Trying to import {module_name} from {file_path} (is_package: {is_package})")
        spec = importlib.util.spec_from_file_location(
            module_name, file_path, submodule_search_locations=file_path.parent)
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
        # ~ print(f"+++ Import success")

    return module_name, is_package


def import_modules(root_dir):
    print("+++", sys.executable)

    excluded_directories = {os.path.join(root_dir, p) for p in EXCLUDED_DIRS}
    excluded_file_paths = {os.path.join(root_dir, p) for p in EXCLUDED_FILE_PATHS}

    has_failures = False
    directories = [root_dir]
    while directories:
        path = directories.pop(0)
        sub_directories = []
        is_package = False
        # ~ print("+++", path)
        with os.scandir(path) as it:
            for entry in it:
                if entry.is_dir():
                    if entry.path in excluded_directories:
                        continue
                    if entry.name.startswith('.'):
                        continue
                    sub_directories.append(entry.path)
                    continue
                if not entry.is_file():
                    continue
                if entry.path in excluded_file_paths:
                    continue
                if not entry.name.endswith(".py"):
                    continue
                try:
                    is_current_package = import_module(entry.path)
                    is_package = is_package or is_current_package
                except SystemExit as e:
                    has_failures = True
                    print(f"+++ Failed to import {entry.path} (module called `sys.exit`), {e}")
                except Exception as e:
                    has_failures = True
                    print(f"+++ Failed to import {entry.path} ({e.__class__}), {e}")
                    traceback.print_tb(e.__traceback__)
                    print("\n\n")
        if not is_package:
            directories += sub_directories

    assert (not has_failures)


def main():
    import sys
    if sys.argv[1] == "--root-dir":
        root_dir = sys.argv[2]
        import_modules(root_dir=root_dir)
    else:
        print("Missing --root-dir parameter")
        assert (0)


if __name__ == "__main__":
    main()
