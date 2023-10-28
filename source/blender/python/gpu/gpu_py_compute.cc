/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bpygpu
 *
 * - Use `bpygpu_` for local API.
 * - Use `BPyGPU` for public API.
 */

#include <Python.h>

#include "BLI_utildefines.h"

#include "GPU_shader.h"
#include "GPU_texture.h"
#include "GPU_uniform_buffer.h"
#include "GPU_compute.h"

#include "../generic/py_capi_utils.h"
#include "../generic/python_compat.h"
#include "../generic/python_utildefines.h"

#include "../mathutils/mathutils.h"

#include "gpu_py.h"
#include "gpu_py_texture.h"
#include "gpu_py_uniformbuffer.h"
#include "gpu_py_vertex_format.h"
#include "gpu_py_shader.h"
#include "gpu_py_compute.h" /* own include */

PyDoc_STRVAR(
    pygpu_compute_dispatch_doc,
    ".. function:: dispatch(shader, groups_x_len,  groups_y_len,  groups_z_len)\n"
    "\n"
    "   Dispatches GPU compute.\n"
    "\n"
    "   :arg shader: The shader that you want to dispatch.\n"
    "   :type shader: :class:`gpu.types.GPUShader`\n"
    "   :arg groups_x_len: Int for group x length:\n"
    "   :type groups_x_len: int\n"
    "   :arg groups_y_len: Int for group y length:\n"
    "   :type groups_y_len: int\n"
    "   :arg groups_z_len: Int for group z length:\n"
    "   :type groups_z_len: int\n"
    "   :return: Shader object.\n"
    "   :rtype: :class:`bpy.types.GPUShader`\n");
static void pygpu_compute_dispatch(PyObject * /*self*/, PyObject *args, PyObject *kwds)
{
  BPyGPUShader *py_shader;
  int groups_x_len;
  int groups_y_len;
  int groups_z_len;

  static const char *_keywords[] = {"shader", "groups_x_len", "groups_y_len", "groups_z_len", nullptr};
  static _PyArg_Parser _parser = {
      PY_ARG_PARSER_HEAD_COMPAT()
      "O" /* `shader` */
      "i" /* `groups_x_len` */
      "i" /* `groups_y_len` */
      "i" /* `groups_z_len` */
      ":from_builtin",
      _keywords,
      nullptr,
  };
  if (_PyArg_ParseTupleAndKeywordsFast(args,
                                        kwds,
                                        &_parser,
                                        &py_shader,
                                        &groups_x_len,
                                        &groups_y_len,
                                        &groups_z_len))
  {
 
   if (!BPyGPUShader_Check(py_shader)) {
    PyErr_Format(PyExc_TypeError, "Expected a GPUShader, got %s", Py_TYPE(py_shader)->tp_name);
   }  else {
    // printf("py_shader: %p\n", py_shader);
    GPUShader *shader = py_shader->shader;
    // printf("shader: %p\n", shader);
    // printf("groups_x_len: %d\n", groups_x_len);
    // printf("groups_y_len: %d\n", groups_y_len);
    // printf("groups_z_len: %d\n", groups_z_len);
    return GPU_compute_dispatch(shader, groups_x_len, groups_y_len, groups_z_len);

   }

  }
}

/* -------------------------------------------------------------------- */
/** \name Module
 * \{ */

static PyMethodDef pygpu_compute__tp_methods[] = {
    {"dispatch", (PyCFunction)pygpu_compute_dispatch, METH_VARARGS | METH_KEYWORDS, pygpu_compute_dispatch_doc},
    {nullptr, nullptr, 0, nullptr},
};

#if (defined(__GNUC__) && !defined(__clang__))
#  pragma GCC diagnostic pop
#endif

PyDoc_STRVAR(pygpu_compute__tp_doc, "This module provides access to the global GPU compute functions");
static PyModuleDef pygpu_compute_module_def = {
    /*m_base*/ PyModuleDef_HEAD_INIT,
    /*m_name*/ "gpu.compute",
    /*m_doc*/ pygpu_compute__tp_doc,
    /*m_size*/ 0,
    /*m_methods*/ pygpu_compute__tp_methods,
    /*m_slots*/ nullptr,
    /*m_traverse*/ nullptr,
    /*m_clear*/ nullptr,
    /*m_free*/ nullptr,
};

PyObject *bpygpu_compute_init()
{
  PyObject *submodule;

  submodule = bpygpu_create_module(&pygpu_compute_module_def);

  return submodule;
}

/** \} */
