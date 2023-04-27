/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <Python.h>

#include <pxr/base/plug/plugin.h>
#include <pxr/base/plug/registry.h>
#include <pxr/usdImaging/usdImagingGL/engine.h>

#include "BKE_appdir.h"
#include "BLI_fileops.h"
#include "BLI_path_util.h"

#include "final_engine.h"
#include "preview_engine.h"
#include "viewport_engine.h"

namespace blender::render::hydra {

static PyObject *init_func(PyObject * /*self*/, PyObject *args)
{
  CLOG_INFO(LOG_RENDER_HYDRA, 1, "Init");

  pxr::PlugRegistry::GetInstance().RegisterPlugins(std::string(BKE_appdir_program_dir()) +
                                                   "/blender.shared/usd");

  Py_RETURN_NONE;
}

static PyObject *register_plugins_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pyplugin_dirs;
  if (!PyArg_ParseTuple(args, "O", &pyplugin_dirs)) {
    Py_RETURN_NONE;
  }

  std::vector<std::string> plugin_dirs;
  PyObject *pyiter, *pyitem;

  pyiter = PyObject_GetIter(pyplugin_dirs);
  if (pyiter) {
    while (pyitem = PyIter_Next(pyiter)) {
      plugin_dirs.push_back(PyUnicode_AsUTF8(pyitem));
      Py_DECREF(pyitem);
    }
    Py_DECREF(pyiter);
  }

  pxr::PlugRegistry &registry = pxr::PlugRegistry::GetInstance();
  registry.RegisterPlugins(plugin_dirs);

  /* logging */
  std::stringstream ss;
  ss << "plugins=[";
  for (auto &s : plugin_dirs) {
    ss << s << ", ";
  }
  ss << "]";
  CLOG_INFO(LOG_RENDER_HYDRA, 1, "Register %s", ss.str().c_str());

  Py_RETURN_NONE;
}

static PyObject *get_render_plugins_func(PyObject * /*self*/, PyObject *args)
{
  pxr::PlugRegistry &registry = pxr::PlugRegistry::GetInstance();
  pxr::TfTokenVector plugin_ids = pxr::UsdImagingGLEngine::GetRendererPlugins();
  PyObject *ret = PyTuple_New(plugin_ids.size());
  PyObject *val;
  for (int i = 0; i < plugin_ids.size(); ++i) {
    PyObject *descr = PyDict_New();

    PyDict_SetItemString(descr, "id", val = PyUnicode_FromString(plugin_ids[i].GetText()));
    Py_DECREF(val);

    PyDict_SetItemString(
        descr,
        "name",
        val = PyUnicode_FromString(
            pxr::UsdImagingGLEngine::GetRendererDisplayName(plugin_ids[i]).c_str()));
    Py_DECREF(val);

    std::string plugin_name = plugin_ids[i];
    plugin_name = plugin_name.substr(0, plugin_name.size() - 6);
    plugin_name[0] = tolower(plugin_name[0]);
    std::string path = "";
    pxr::PlugPluginPtr plugin = registry.GetPluginWithName(plugin_name);
    if (plugin) {
      path = plugin->GetPath();
    }
    PyDict_SetItemString(descr, "path", val = PyUnicode_FromString(path.c_str()));
    Py_DECREF(val);

    PyTuple_SetItem(ret, i, descr);
  }
  return ret;
}

static PyObject *engine_create_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pyengine;
  char *engine_type, *render_delegate_id;
  if (!PyArg_ParseTuple(args, "Oss", &pyengine, &engine_type, &render_delegate_id)) {
    Py_RETURN_NONE;
  }

  RenderEngine *bl_engine = (RenderEngine *)PyLong_AsVoidPtr(pyengine);

  Engine *engine;
  if (STREQ(engine_type, "VIEWPORT")) {
    engine = new ViewportEngine(bl_engine, render_delegate_id);
  }
  else if (STREQ(engine_type, "PREVIEW")) {
    engine = PreviewEngine::get_instance(bl_engine, render_delegate_id);
  }
  else {
    if (bl_engine->type->flag & RE_USE_GPU_CONTEXT) {
      engine = new FinalEngineGL(bl_engine, render_delegate_id);
    }
    else {
      engine = new FinalEngine(bl_engine, render_delegate_id);
    }
  }

  CLOG_INFO(LOG_RENDER_HYDRA, 2, "Engine %016llx %s", engine, engine_type);

  return PyLong_FromVoidPtr(engine);
}

static PyObject *engine_free_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pyengine;
  if (!PyArg_ParseTuple(args, "O", &pyengine)) {
    Py_RETURN_NONE;
  }

  Engine *engine = (Engine *)PyLong_AsVoidPtr(pyengine);
  PreviewEngine *preview_engine = dynamic_cast<PreviewEngine *>(engine);
  if (preview_engine) {
    PreviewEngine::schedule_free();
  }
  else {
    delete engine;
  }

  CLOG_INFO(LOG_RENDER_HYDRA, 2, "Engine %016llx", engine);
  Py_RETURN_NONE;
}

static PyObject *engine_sync_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pyengine, *pydepsgraph, *pycontext, *pysettings;
  if (!PyArg_ParseTuple(args, "OOOO", &pyengine, &pydepsgraph, &pycontext, &pysettings)) {
    Py_RETURN_NONE;
  }

  Engine *engine = (Engine *)PyLong_AsVoidPtr(pyengine);
  Depsgraph *depsgraph = (Depsgraph *)PyLong_AsVoidPtr(pydepsgraph);
  bContext *context = (bContext *)PyLong_AsVoidPtr(pycontext);

  pxr::HdRenderSettingsMap settings;
  PyObject *pyiter = PyObject_GetIter(pysettings);
  if (pyiter) {
    PyObject *pykey, *pyval;
    while (pykey = PyIter_Next(pyiter)) {
      pxr::TfToken key(PyUnicode_AsUTF8(pykey));
      pyval = PyDict_GetItem(pysettings, pykey);

      if (PyLong_Check(pyval)) {
        settings[key] = PyLong_AsLong(pyval);
      }
      else if (PyFloat_Check(pyval)) {
        settings[key] = PyFloat_AsDouble(pyval);
      }
      else if (PyUnicode_Check(pyval)) {
        settings[key] = PyUnicode_AsUTF8(pyval);
      }
      Py_DECREF(pykey);
    }
    Py_DECREF(pyiter);
  }

  engine->sync(depsgraph, context, settings);

  CLOG_INFO(LOG_RENDER_HYDRA, 2, "Engine %016llx", engine);
  Py_RETURN_NONE;
}

static PyObject *engine_render_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pyengine, *pydepsgraph;

  if (!PyArg_ParseTuple(args, "OO", &pyengine, &pydepsgraph)) {
    Py_RETURN_NONE;
  }

  Engine *engine = (Engine *)PyLong_AsVoidPtr(pyengine);
  Depsgraph *depsgraph = (Depsgraph *)PyLong_AsVoidPtr(pydepsgraph);

  /* Allow Blender to execute other Python scripts. */
  Py_BEGIN_ALLOW_THREADS;
  engine->render(depsgraph);
  Py_END_ALLOW_THREADS;

  CLOG_INFO(LOG_RENDER_HYDRA, 2, "Engine %016llx", engine);
  Py_RETURN_NONE;
}

static PyObject *engine_view_draw_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pyengine, *pydepsgraph, *pycontext;
  if (!PyArg_ParseTuple(args, "OOO", &pyengine, &pydepsgraph, &pycontext)) {
    Py_RETURN_NONE;
  }

  ViewportEngine *engine = (ViewportEngine *)PyLong_AsVoidPtr(pyengine);
  Depsgraph *depsgraph = (Depsgraph *)PyLong_AsVoidPtr(pydepsgraph);
  bContext *context = (bContext *)PyLong_AsVoidPtr(pycontext);

  /* Allow Blender to execute other Python scripts. */
  Py_BEGIN_ALLOW_THREADS;
  engine->render(depsgraph, context);
  Py_END_ALLOW_THREADS;

  CLOG_INFO(LOG_RENDER_HYDRA, 3, "Engine %016llx", engine);
  Py_RETURN_NONE;
}

static PyMethodDef methods[] = {
    {"init", init_func, METH_VARARGS, ""},
    {"register_plugins", register_plugins_func, METH_VARARGS, ""},
    {"get_render_plugins", get_render_plugins_func, METH_VARARGS, ""},

    {"engine_create", engine_create_func, METH_VARARGS, ""},
    {"engine_free", engine_free_func, METH_VARARGS, ""},
    {"engine_sync", engine_sync_func, METH_VARARGS, ""},
    {"engine_render", engine_render_func, METH_VARARGS, ""},
    {"engine_view_draw", engine_view_draw_func, METH_VARARGS, ""},

    {NULL, NULL, 0, NULL},
};

static struct PyModuleDef module = {
    PyModuleDef_HEAD_INIT,
    "_bpy_hydra",
    "Hydra render API",
    -1,
    methods,
    NULL,
    NULL,
    NULL,
    NULL,
};

}  // namespace blender::render::hydra

#ifdef __cplusplus
extern "C" {
#endif

PyObject *BPyInit_hydra(void)
{
  PyObject *mod = PyModule_Create(&blender::render::hydra::module);
  return mod;
}

#ifdef __cplusplus
}
#endif
