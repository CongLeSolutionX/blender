/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup pythonintern
 *
 * This file defines `_bpy.ops`, an internal python module which gives Python
 * the ability to inspect and call operators (defined by C or Python).
 *
 * \note
 * This C module is private, it should only be used by `scripts/modules/bpy/ops.py` which
 * exposes operators as dynamically defined modules & callable objects to access all operators.
 */

#include <Python.h>

#include "RNA_types.hh"

#include "BLI_listbase.h"
#include "BLI_utildefines.h"

#include "../generic/py_capi_rna.h"
#include "../generic/py_capi_utils.h"
#include "../generic/python_compat.h"
#include "../generic/python_utildefines.h"

#include "BPY_extern.h"
#include "bpy_capi_utils.h"
#include "bpy_operator.h"
#include "bpy_operator_wrap.h"
#include "bpy_rna.h" /* for setting argument properties & type method `get_rna_type`. */

#include "RNA_access.hh"
#include "RNA_enum_types.hh"
#include "RNA_prototypes.h"

#include "WM_api.hh"
#include "WM_op_handlers.h"
#include "WM_types.hh"

#include "MEM_guardedalloc.h"

#include "BLI_ghash.h"

#include "BKE_context.hh"
#include "BKE_report.h"

/* so operators called can spawn threads which acquire the GIL */
#define BPY_RELEASE_GIL

static wmOperatorType *ot_lookup_from_py_string(PyObject *value, const char *py_fn_id)
{
  const char *opname = PyUnicode_AsUTF8(value);
  if (opname == nullptr) {
    PyErr_Format(PyExc_TypeError, "%s() expects a string argument", py_fn_id);
    return nullptr;
  }

  wmOperatorType *ot = WM_operatortype_find(opname, true);
  if (ot == nullptr) {
    PyErr_Format(PyExc_KeyError, "%s(\"%s\") not found", py_fn_id, opname);
    return nullptr;
  }
  return ot;
}

static PyObject *pyop_poll(PyObject * /*self*/, PyObject *args)
{
  wmOperatorType *ot;
  const char *opname;
  const char *context_str = nullptr;
  PyObject *ret;

  wmOperatorCallContext context = WM_OP_EXEC_DEFAULT;

  /* XXX TODO: work out a better solution for passing on context,
   * could make a tuple from self and pack the name and Context into it. */
  bContext *C = BPY_context_get();

  if (C == nullptr) {
    PyErr_SetString(PyExc_RuntimeError, "Context is None, can't poll any operators");
    return nullptr;
  }

  /* All arguments are positional. */
  static const char *_keywords[] = {"", "", nullptr};
  static _PyArg_Parser _parser = {
      PY_ARG_PARSER_HEAD_COMPAT()
      "s" /* `opname` */
      "|" /* Optional arguments. */
      "s" /* `context_str` */
      ":_bpy.ops.poll",
      _keywords,
      nullptr,
  };
  if (!_PyArg_ParseTupleAndKeywordsFast(args, nullptr, &_parser, &opname, &context_str)) {
    return nullptr;
  }

  ot = WM_operatortype_find(opname, true);

  if (ot == nullptr) {
    PyErr_Format(PyExc_AttributeError,
                 "Polling operator \"bpy.ops.%s\" error, "
                 "could not be found",
                 opname);
    return nullptr;
  }

  if (context_str) {
    int context_int = context;

    if (RNA_enum_value_from_id(rna_enum_operator_context_items, context_str, &context_int) == 0) {
      char *enum_str = pyrna_enum_repr(rna_enum_operator_context_items);
      PyErr_Format(PyExc_TypeError,
                   "Calling operator \"bpy.ops.%s.poll\" error, "
                   "expected a string enum in (%s)",
                   opname,
                   enum_str);
      MEM_freeN(enum_str);
      return nullptr;
    }
    /* Copy back to the properly typed enum. */
    context = wmOperatorCallContext(context_int);
  }

  /* main purpose of this function */
  ret = WM_operator_poll_context((bContext *)C, ot, context) ? Py_True : Py_False;

  return Py_INCREF_RET(ret);
}

static PyObject *pyop_call(PyObject * /*self*/, PyObject *args)
{
  wmOperatorType *ot;
  int error_val = 0;
  PointerRNA ptr;
  int operator_ret = OPERATOR_CANCELLED;

  const char *opname;
  const char *context_str = nullptr;
  PyObject *kw = nullptr; /* optional args */

  wmOperatorCallContext context = WM_OP_EXEC_DEFAULT;
  int is_undo = false;

  /* XXX TODO: work out a better solution for passing on context,
   * could make a tuple from self and pack the name and Context into it. */
  bContext *C = BPY_context_get();

  if (C == nullptr) {
    PyErr_SetString(PyExc_RuntimeError, "Context is None, can't poll any operators");
    return nullptr;
  }

  /* All arguments are positional. */
  static const char *_keywords[] = {"", "", "", "", nullptr};
  static _PyArg_Parser _parser = {
      PY_ARG_PARSER_HEAD_COMPAT()
      "s"  /* `opname` */
      "|"  /* Optional arguments. */
      "O!" /* `kw` */
      "s"  /* `context_str` */
      "i"  /* `is_undo` */
      ":_bpy.ops.call",
      _keywords,
      nullptr,
  };
  if (!_PyArg_ParseTupleAndKeywordsFast(
          args, nullptr, &_parser, &opname, &PyDict_Type, &kw, &context_str, &is_undo))
  {
    return nullptr;
  }

  ot = WM_operatortype_find(opname, true);

  if (ot == nullptr) {
    PyErr_Format(PyExc_AttributeError,
                 "Calling operator \"bpy.ops.%s\" error, "
                 "could not be found",
                 opname);
    return nullptr;
  }

  if (!pyrna_write_check()) {
    PyErr_Format(PyExc_RuntimeError,
                 "Calling operator \"bpy.ops.%s\" error, "
                 "can't modify blend data in this state (drawing/rendering)",
                 opname);
    return nullptr;
  }

  if (context_str) {
    int context_int = context;

    if (RNA_enum_value_from_id(rna_enum_operator_context_items, context_str, &context_int) == 0) {
      char *enum_str = pyrna_enum_repr(rna_enum_operator_context_items);
      PyErr_Format(PyExc_TypeError,
                   "Calling operator \"bpy.ops.%s\" error, "
                   "expected a string enum in (%s)",
                   opname,
                   enum_str);
      MEM_freeN(enum_str);
      return nullptr;
    }
    /* Copy back to the properly typed enum. */
    context = wmOperatorCallContext(context_int);
  }

  if (WM_operator_poll_context((bContext *)C, ot, context) == false) {
    bool msg_free = false;
    const char *msg = CTX_wm_operator_poll_msg_get(C, &msg_free);
    PyErr_Format(PyExc_RuntimeError,
                 "Operator bpy.ops.%.200s.poll() %.200s",
                 opname,
                 msg ? msg : "failed, context is incorrect");
    CTX_wm_operator_poll_msg_clear(C);
    if (msg_free) {
      MEM_freeN((void *)msg);
    }
    error_val = -1;
  }
  else {
    WM_operator_properties_create_ptr(&ptr, ot);
    WM_operator_properties_sanitize(&ptr, false);

    if (kw && PyDict_Size(kw)) {
      error_val = pyrna_pydict_to_props(
          &ptr, kw, false, "Converting py args to operator properties: ");
    }

    if (error_val == 0) {
      ReportList *reports;

      reports = static_cast<ReportList *>(MEM_mallocN(sizeof(ReportList), "wmOperatorReportList"));

      /* Own so these don't move into global reports. */
      BKE_reports_init(reports, RPT_STORE | RPT_OP_HOLD | RPT_PRINT_HANDLED_BY_OWNER);

#ifdef BPY_RELEASE_GIL
      /* release GIL, since a thread could be started from an operator
       * that updates a driver */
      /* NOTE: I have not seen any examples of code that does this
       * so it may not be officially supported but seems to work ok. */
      {
        PyThreadState *ts = PyEval_SaveThread();
#endif

        operator_ret = WM_operator_call_py(C, ot, context, &ptr, reports, is_undo);

#ifdef BPY_RELEASE_GIL
        /* regain GIL */
        PyEval_RestoreThread(ts);
      }
#endif

      error_val = BPy_reports_to_error(reports, PyExc_RuntimeError, false);

      /* operator output is nice to have in the terminal/console too */
      if (!BLI_listbase_is_empty(&reports->list)) {
        BPy_reports_write_stdout(reports, nullptr);
      }

      BKE_reports_clear(reports);
      if ((reports->flag & RPT_FREE) == 0) {
        BKE_reports_free(reports);
        MEM_freeN(reports);
      }
      else {
        /* The WM is now responsible for running the modal operator,
         * show reports in the info window. */
        reports->flag &= ~RPT_OP_HOLD;
      }
    }

    WM_operator_properties_free(&ptr);

#if 0
    /* if there is some way to know an operator takes args we should use this */
    {
      /* no props */
      if (kw != nullptr) {
        PyErr_Format(PyExc_AttributeError, "Operator \"%s\" does not take any args", opname);
        return nullptr;
      }

      WM_operator_name_call(C, opname, WM_OP_EXEC_DEFAULT, nullptr, nullptr);
    }
#endif
  }

  if (error_val == -1) {
    return nullptr;
  }

  /* When calling `bpy.ops.wm.read_factory_settings()` `bpy.data's` main pointer
   * is freed by clear_globals(), further access will crash blender.
   * Setting context is not needed in this case, only calling because this
   * function corrects bpy.data (internal Main pointer) */
  BPY_modules_update();

  /* return operator_ret as a bpy enum */
  return pyrna_enum_bitfield_as_set(rna_enum_operator_return_items, operator_ret);
}

static PyObject *pyop_as_string(PyObject * /*self*/, PyObject *args)
{
  wmOperatorType *ot;

  const char *opname;
  PyObject *kw = nullptr; /* optional args */
  bool all_args = true;
  bool macro_args = true;
  int error_val = 0;

  PyObject *pybuf;

  bContext *C = BPY_context_get();

  if (C == nullptr) {
    PyErr_SetString(PyExc_RuntimeError,
                    "Context is None, can't get the string representation of this object.");
    return nullptr;
  }

  /* All arguments are positional. */
  static const char *_keywords[] = {"", "", "", "", nullptr};
  static _PyArg_Parser _parser = {
      PY_ARG_PARSER_HEAD_COMPAT()
      "s"  /* `opname` */
      "|"  /* Optional arguments. */
      "O!" /* `kw` */
      "O&" /* `all_args` */
      "O&" /* `macro_args` */
      ":_bpy.ops.as_string",
      _keywords,
      nullptr,
  };
  if (!_PyArg_ParseTupleAndKeywordsFast(args,
                                        nullptr,
                                        &_parser,
                                        &opname,
                                        &PyDict_Type,
                                        &kw,
                                        PyC_ParseBool,
                                        &all_args,
                                        PyC_ParseBool,
                                        &macro_args))
  {
    return nullptr;
  }

  ot = WM_operatortype_find(opname, true);

  if (ot == nullptr) {
    PyErr_Format(PyExc_AttributeError,
                 "_bpy.ops.as_string: operator \"%.200s\" "
                 "could not be found",
                 opname);
    return nullptr;
  }

  // WM_operator_properties_create(&ptr, opname);
  /* Save another lookup */
  PointerRNA ptr = RNA_pointer_create(nullptr, ot->srna, nullptr);

  if (kw && PyDict_Size(kw)) {
    error_val = pyrna_pydict_to_props(
        &ptr, kw, false, "Converting py args to operator properties: ");
  }

  std::string op_string;
  if (error_val == 0) {
    op_string = WM_operator_pystring_ex(C, nullptr, all_args, macro_args, ot, &ptr);
  }

  WM_operator_properties_free(&ptr);

  if (error_val == -1) {
    return nullptr;
  }

  if (!op_string.empty()) {
    pybuf = PyUnicode_FromString(op_string.c_str());
  }
  else {
    pybuf = PyUnicode_FromString("");
  }

  return pybuf;
}

static PyObject *pyop_dir(PyObject * /*self*/)
{
  GHashIterator iter;
  PyObject *list;
  int i;

  WM_operatortype_iter(&iter);
  list = PyList_New(BLI_ghash_len(iter.gh));

  for (i = 0; !BLI_ghashIterator_done(&iter); BLI_ghashIterator_step(&iter), i++) {
    wmOperatorType *ot = static_cast<wmOperatorType *>(BLI_ghashIterator_getValue(&iter));
    PyList_SET_ITEM(list, i, PyUnicode_FromString(ot->idname));
  }

  return list;
}

static PyObject *pyop_getrna_type(PyObject * /*self*/, PyObject *value)
{
  wmOperatorType *ot;
  if ((ot = ot_lookup_from_py_string(value, "get_rna_type")) == nullptr) {
    return nullptr;
  }

  PointerRNA ptr = RNA_pointer_create(nullptr, &RNA_Struct, ot->srna);
  BPy_StructRNA *pyrna = (BPy_StructRNA *)pyrna_struct_CreatePyObject(&ptr);
  return (PyObject *)pyrna;
}

static PyObject *pyop_get_bl_options(PyObject * /*self*/, PyObject *value)
{
  wmOperatorType *ot;
  if ((ot = ot_lookup_from_py_string(value, "get_bl_options")) == nullptr) {
    return nullptr;
  }
  return pyrna_enum_bitfield_as_set(rna_enum_operator_type_flag_items, ot->flag);
}

#if (defined(__GNUC__) && !defined(__clang__))
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wcast-function-type"
#endif

static PyMethodDef bpy_ops_methods[] = {
    {"poll", (PyCFunction)pyop_poll, METH_VARARGS, nullptr},
    {"call", (PyCFunction)pyop_call, METH_VARARGS, nullptr},
    {"as_string", (PyCFunction)pyop_as_string, METH_VARARGS, nullptr},
    {"dir", (PyCFunction)pyop_dir, METH_NOARGS, nullptr},
    {"get_rna_type", (PyCFunction)pyop_getrna_type, METH_O, nullptr},
    {"get_bl_options", (PyCFunction)pyop_get_bl_options, METH_O, nullptr},
    {"macro_define", (PyCFunction)PYOP_wrap_macro_define, METH_VARARGS, nullptr},
    {nullptr, nullptr, 0, nullptr},
};

#if (defined(__GNUC__) && !defined(__clang__))
#  pragma GCC diagnostic pop
#endif

static PyModuleDef bpy_ops_module = {
    /*m_base*/ PyModuleDef_HEAD_INIT,
    /*m_name*/ "_bpy.ops",
    /*m_doc*/ nullptr,
    /*m_size*/ -1, /* multiple "initialization" just copies the module dict. */
    /*m_methods*/ bpy_ops_methods,
    /*m_slots*/ nullptr,
    /*m_traverse*/ nullptr,
    /*m_clear*/ nullptr,
    /*m_free*/ nullptr,
};

static int bpy_op_handler_check(void *py_data, void* owner, void *callback)
{
  PyObject *py_owner = PyTuple_GET_ITEM(py_data, 0);
  PyObject *py_callback = PyTuple_GET_ITEM(py_data, 2);
  if (owner != NULL && callback != NULL) {
    return (py_owner == owner && py_callback == callback);
  }
  else if (owner != NULL)
  {
    return (py_owner == owner);
  }
  else {  //callback != NULL
    return (py_callback == callback);
  }
}

/*
 * @Properties Rna Properties that has been used to set the operator
 */
static PyObject *bpy_op_get_operator_params(PointerRNA *properties)
{
  const char *arg_name = NULL;
  PyObject *py_dict = PyDict_New();
  PyObject *data;
  RNA_STRUCT_BEGIN (properties, prop) {
    arg_name = RNA_property_identifier(prop);
    data = NULL;
    if (STREQ(arg_name, "rna_type")) {
      continue;
    }
    switch (RNA_property_type(prop)) {
      case PROP_BOOLEAN: {
        bool val = RNA_property_boolean_get(properties, prop);
        // from Py Docs, Py_False and Py_truee needs to be treated just like any other object with
        // respect to reference counts.
        data = val ? Py_False : Py_True;
        break;
      }
      case PROP_INT: {
        const int prop_array_length = RNA_property_array_length(properties, prop);
        if (prop_array_length == 0) {
          int val = RNA_property_int_get(properties, prop);
          data = PyLong_FromLong(val);
        }
        else {
          int *values = (int*) MEM_callocN(sizeof(int) * prop_array_length, __func__);
          RNA_property_int_get_array(properties, prop, values);
          data = PyTuple_New(prop_array_length);
          for (int i = 0; i < prop_array_length; i++) {
            PyObject *py_val = PyLong_FromLong(*(values + i));
            PyTuple_SET_ITEM(data, i, py_val);
          }
          MEM_freeN(values);
        }
        break;
      }
      case PROP_FLOAT: {
        const int prop_array_length = RNA_property_array_length(properties, prop);
        if (prop_array_length == 0) {
          float val;
          val = RNA_property_float_get(properties, prop);
          data = PyFloat_FromDouble(val);
        }
        else {
          float *values = (float*) MEM_callocN(sizeof(float) * prop_array_length, __func__);
          RNA_property_float_get_array(properties, prop, values);
          data = PyTuple_New(prop_array_length);
          for (int i = 0; i < prop_array_length; i++) {
            PyObject *py_val = PyFloat_FromDouble(*(values + i));
            PyTuple_SET_ITEM(data, i, py_val);
          }
          MEM_freeN(values);
        }
        break;
      }
      case PROP_STRING: {
        char buff[256];
        char *value = RNA_property_string_get_alloc(properties, prop, buff, sizeof(buff), NULL);
        data = PyUnicode_FromString(value);
        if (value != buff) {
          MEM_freeN(value);
        }
        break;
      }
      case PROP_ENUM: {
        int val = RNA_property_enum_get(properties, prop);
        data = PyLong_FromLong(val);
        break;
      }
      case PROP_POINTER: {
        data = PyUnicode_FromString("POINTER");
        break;
      }
      case PROP_COLLECTION: {
        data = PyUnicode_FromString("COLLECTION");
        break;
      }
      default:
        BLI_assert(false);
    }
    if (data != NULL) {
      PyDict_SetItemString(py_dict, arg_name, data);
    }
  }
  RNA_STRUCT_END;
  return py_dict;
}


static bool bpy_op_callback_get_return_value(PyObject *callback, PyObject *py_ret)
{
  bool ret = true; // Do not interrump on error
  if (py_ret == NULL) {
    PyC_Err_PrintWithFunc(callback);
  }
  else {
    if (py_ret == Py_None) {
      // pass
    }
    else if (py_ret == Py_True) {
      // pass
    }
    else if (py_ret == Py_False){
      ret = false;
    } else {
      PyErr_SetString(PyExc_ValueError, "the return value must be None or boolean");
      PyC_Err_PrintWithFunc(callback);
    }
    Py_DECREF(py_ret);
  }
  return ret;
}


static PyObject *bpy_op_get_callback_call(PyObject *callback,
    bContext *C, const wmEvent *event, int *operator_ret, PyObject *params, PyObject *callback_args)
{
  PointerRNA ctx_ptr;
  PointerRNA event_ptr;

  PyObject *bpy_ctx;
  PyObject *bpy_event;
  PyObject *py_ret;

  ctx_ptr = RNA_pointer_create(nullptr, &RNA_Context, C);
  bpy_ctx = pyrna_struct_CreatePyObject(&ctx_ptr);

    if (event != NULL) {
    event_ptr = RNA_pointer_create(NULL, &RNA_Event, (void*) event);
    bpy_event = pyrna_struct_CreatePyObject(&event_ptr);
  }
  else {
    bpy_event = Py_None;
  }

  int s = (operator_ret == NULL) ? 3 : 4;
  int c = (callback_args == Py_None) ? s : PyTuple_GET_SIZE(callback_args) + s;
  PyObject *func_args = PyTuple_New(c);

  PyTuple_SET_ITEM(func_args, 0, bpy_ctx);
  PyTuple_SET_ITEM(func_args, 1, bpy_event);
  PyTuple_SET_ITEM(func_args, 2, params);

  if (operator_ret != NULL) {
    PyObject *op_ret = pyrna_enum_bitfield_to_py(rna_enum_operator_return_items, *operator_ret);
    PyTuple_SET_ITEM(func_args, 3, op_ret);
  }

  for (int i = s; i < c; i++) {
    PyTuple_SET_ITEM(func_args, i, PyTuple_GET_ITEM(callback_args, i - s));
  }

  py_ret = PyObject_CallObject(callback, func_args);
  Py_DECREF(func_args);
  return py_ret;
}


static bool bpy_op_handler_poll(struct bContext *C,
                               const wmEvent *event, void *py_data,
                                PointerRNA *properties)
{
  bool ret = true;
  PyGILState_STATE gilstate;  // this is because is not thread safe
  bpy_context_set(C, &gilstate);
  {
    PyObject *callback_args = PyTuple_GET_ITEM(py_data, 3);
    PyObject *py_poll = PyTuple_GET_ITEM(py_data, 4);

      // Properties get null on modall poll, params are not bypassed to Py poll function
    PyObject *params = (properties == NULL) ? Py_None : bpy_op_get_operator_params(properties);
    if (py_poll != Py_None) {
      PyObject *py_ret = bpy_op_get_callback_call(py_poll, C, event, NULL, params, callback_args);

      if (py_ret == NULL) {
        // Error
        PyErr_Print();
        return false;
      }
      else if (py_ret == Py_True) {
        ret = true;
        Py_DECREF(py_ret);
      }
      else if (py_ret == Py_False) {
        ret = false;
        Py_DECREF(py_ret);
      }
      else {
        ret = false;
        Py_DECREF(py_ret);
        PyErr_SetString(PyExc_ValueError, "the return value must be boolean");
        PyC_Err_PrintWithFunc(py_poll);
      }
    }
    // Py_DECREF(params);
  }
  bpy_context_clear(C, &gilstate);
  return ret;
}

static bool bpy_op_handler_modal (bContext *C, const wmEvent *event, void *py_data, PointerRNA *properties, int operator_ret)
{
  // this is because is not thread safe
  bool ret = true;
  PyGILState_STATE gilstate;
  bpy_context_set(C, &gilstate);
  {
    PyObject *callback = PyTuple_GET_ITEM(py_data, 2);
    PyObject *callback_args = PyTuple_GET_ITEM(py_data, 3);
    PyObject *py_ret = bpy_op_get_callback_call(callback, C, event, &operator_ret, Py_None, callback_args);
    ret = bpy_op_callback_get_return_value(callback, py_ret);
  }
  bpy_context_clear(C, &gilstate);
  return ret;
}


static bool bpy_op_handler_invoke(bContext *C,
                                  const wmEvent *event,
                                  void *py_data,
                                  PointerRNA *properties,
                                  int operator_ret)
{
  bool ret = true;
  // this is because is not thread safe
  PyGILState_STATE gilstate;
   bpy_context_set(C, &gilstate);
  {
    PyObject *callback = PyTuple_GET_ITEM(py_data, 2);
    PyObject *callback_args = PyTuple_GET_ITEM(py_data, 3);
    PyObject *params = bpy_op_get_operator_params(properties);
    PyObject *py_ret =  bpy_op_get_callback_call(callback, C, event, operator_ret ? &operator_ret : NULL , params, callback_args);
    ret = bpy_op_callback_get_return_value(callback, py_ret);
  }
  bpy_context_clear(C, &gilstate);
  return ret;
}


static PyObject *bpy_op_handler_proc(
                                 PyObject *args,
                                 PyObject *kw)
{
  const char *error_prefix = "op_handler_proc";

  PyObject *py_op = NULL;
  PyObject *py_owner = NULL;  // Object who creates the handler
  PyObject *callback = NULL, *py_poll = NULL;
  PyObject *callback_args = NULL;


  if (PyTuple_GET_SIZE(args) != 0) {
    PyErr_Format(PyExc_TypeError, "%s: only keyword arguments are supported", error_prefix);
  }

  // see https://docs.python.org/3/c-api/arg.html
  static const char *_keywords[] = {"owner", "op", "cb", "args", "poll", NULL};
  static _PyArg_Parser _parser = {"OOOOO|:handler_proc", _keywords, 0};

  if (!_PyArg_ParseTupleAndKeywordsFast(args, kw, &_parser, &py_owner, &py_op, &callback, &callback_args, &py_poll)) {
    PyErr_SetString(
        PyExc_TypeError, "Cannot set arguments, or types does not match");
  }

  if (callback != Py_None && !PyFunction_Check(callback)) {
    // Callback may be none on remove
    PyErr_Format(
        PyExc_TypeError, "callback expects a function, found %.200s", Py_TYPE(callback)->tp_name);
  }

  if (py_poll != Py_None && !PyFunction_Check(py_poll)) {
    // Callback may be none on remove
    PyErr_Format(
        PyExc_TypeError, "poll expects a function, found %.200s", Py_TYPE(callback)->tp_name);
  }

  if (py_op != Py_None  && !PyUnicode_Check(py_op)) {
    PyErr_Format(
        PyExc_TypeError, "op expects an astring, found %.200s", Py_TYPE(py_op)->tp_name);
  }

  if (PyErr_Occurred() != NULL) {
    PyErr_Print();
    return NULL;
  }


  PyObject *py_data = PyTuple_New(5);
  PyTuple_SET_ITEMS(py_data,
                    Py_INCREF_RET(py_owner),   // 0
                    Py_INCREF_RET(py_op),      // 1
                    Py_INCREF_RET(callback),   // 2
                    Py_INCREF_RET(callback_args), // 3
                    Py_INCREF_RET(py_poll));  // 4

  return Py_INCREF_RET(py_data);
}

static PyObject *op_handler_append(int handler_id , PyObject *args, PyObject *kw)
{
  bContext *C = BPY_context_get();
  struct wmOpHandlers *op_handlers = CTX_wm_op_handlers(C);

  PyObject *py_data = bpy_op_handler_proc(args, kw);

  bool (*func)(bContext * C,const wmEvent * event, void *, PointerRNA *, int) = nullptr;

  switch (handler_id) {
    case HANDLER_TYPE_PRE_INVOKE:
    case HANDLER_TYPE_POST_INVOKE:
      func = bpy_op_handler_invoke;
      break;
    case HANDLER_TYPE_MODAL:
    case HANDLER_TYPE_MODAL_END:
      func = bpy_op_handler_modal;
      break;
  }

  if (py_data != NULL) {
    PyObject *py_owner = PyTuple_GET_ITEM(py_data, 0);
    PyObject *py_op = PyTuple_GET_ITEM(py_data, 1);
    PyObject *py_callback = PyTuple_GET_ITEM(py_data, 2);
    PyObject *py_poll = PyTuple_GET_ITEM(py_data, 4);
    if (py_op == Py_None) {
      PyErr_Format(PyExc_TypeError, "missing operator");
    }
    else if (py_callback == Py_None) {
      PyErr_Format(PyExc_TypeError, "callback expects a function");
    } else {
      WM_op_handlers_append(op_handlers,
                            handler_id,
                            py_owner,
                            PyUnicode_AsUTF8(py_op),
                            func,
                            bpy_op_handler_check,
                            py_poll == Py_None ? NULL : bpy_op_handler_poll,
                            py_data);
    }
  }

  if (PyErr_Occurred() != NULL) {
    PyErr_Print();
  }
  Py_RETURN_NONE;
}

static PyObject *op_handler_remove(int handler_id, PyObject *args, PyObject *kw)
{
  bContext *C = BPY_context_get();
  struct wmOpHandlers *op_handlers = CTX_wm_op_handlers(C);

  PyObject *py_data = bpy_op_handler_proc(args, kw);

  if (py_data != NULL) {
    PyObject *py_owner = PyTuple_GET_ITEM(py_data, 0);
    PyObject *py_op = PyTuple_GET_ITEM(py_data, 1);
    PyObject *py_cb = PyTuple_GET_ITEM(py_data, 2);
    if (py_owner == Py_None && py_cb == Py_None) {
      PyErr_Format(PyExc_TypeError, "missing owner or callback");
    } else {
      if (WM_op_handlers_remove(op_handlers,
                                handler_id,
                                (py_op == Py_None ? NULL : PyUnicode_AsUTF8(py_op)),
                                (py_cb == Py_None ? NULL : py_cb),
                                (py_owner == Py_None ? NULL : py_owner)) == 0) {
        PyErr_Format(PyExc_NameError, "data not found on %s", PyUnicode_AsUTF8(py_op));
      }
    }
  }

  if (PyErr_Occurred() != NULL) {
    PyErr_Print();
  }

  Py_RETURN_NONE;
}

static PyObject *op_handler_append_pre_invoke(PyObject *self, PyObject *args, PyObject *kw)
{
  return op_handler_append(HANDLER_TYPE_PRE_INVOKE, args, kw);
}

static PyObject *op_handler_append_post_invoke(PyObject *self, PyObject *args, PyObject *kw)
{
  return op_handler_append(HANDLER_TYPE_POST_INVOKE, args, kw);
}

static PyObject *op_handler_append_modal(PyObject *self, PyObject *args, PyObject *kw)
{
  return op_handler_append(HANDLER_TYPE_MODAL, args, kw);
}

static PyObject *op_handler_append_modal_end(PyObject *self, PyObject *args, PyObject *kw)
{
  return op_handler_append(HANDLER_TYPE_MODAL_END, args, kw);
}



static PyObject *op_handler_remove_pre_invoke(PyObject *self, PyObject *args, PyObject *kw)
{
  return op_handler_remove(HANDLER_TYPE_PRE_INVOKE, args, kw);
}

static PyObject *op_handler_remove_post_invoke(PyObject *self, PyObject *args, PyObject *kw)
{
  return op_handler_remove(HANDLER_TYPE_POST_INVOKE, args, kw);
}

static PyObject *op_handler_remove_modal(PyObject *self, PyObject *args, PyObject *kw)
{
  return op_handler_remove(HANDLER_TYPE_MODAL, args, kw);
}

static PyObject *op_handler_remove_modal_end(PyObject *self, PyObject *args, PyObject *kw)
{
  return op_handler_remove(HANDLER_TYPE_MODAL_END, args, kw);
}


static PyObject *op_handlers_remove(PyObject *self, PyObject *args, PyObject *kw)
{
  return op_handler_remove(HANDLER_TYPE_ALL, args, kw);
}


static struct PyMethodDef bpy_ops_handlers_methods[] = {
    {"pre_invoke", (PyCFunction)op_handler_append_pre_invoke, METH_VARARGS | METH_KEYWORDS, NULL},
    {"post_invoke", (PyCFunction)op_handler_append_post_invoke, METH_VARARGS | METH_KEYWORDS, NULL},
    {"modal", (PyCFunction)op_handler_append_modal, METH_VARARGS | METH_KEYWORDS, NULL},
    {"modal_end", (PyCFunction)op_handler_append_modal_end, METH_VARARGS | METH_KEYWORDS, NULL},
    {"pre_invoke_remove", (PyCFunction)op_handler_remove_pre_invoke, METH_VARARGS | METH_KEYWORDS, NULL},
    {"post_invoke_remove", (PyCFunction)op_handler_remove_post_invoke, METH_VARARGS | METH_KEYWORDS, NULL},
    {"modal_remove",(PyCFunction)op_handler_remove_post_invoke, METH_VARARGS | METH_KEYWORDS, NULL},
    {"modal_end_remove", (PyCFunction)op_handler_remove_modal_end, METH_VARARGS | METH_KEYWORDS, NULL},
    {"remove", (PyCFunction)op_handlers_remove,  METH_VARARGS | METH_KEYWORDS,  NULL},
    {NULL, NULL, 0, NULL},
};

static struct PyModuleDef bpy_ops_handlers = {
    PyModuleDef_HEAD_INIT,
    "_bpy.ops.handlers",
    NULL,
    -1, /* multiple "initialization" just copies the module dict. */
    bpy_ops_handlers_methods,
    NULL,
    NULL,
    NULL,
    NULL,
};



PyObject *BPY_operator_module()
{
  PyObject *submodule;

  submodule = PyModule_Create(&bpy_ops_module);

  PyObject *handlers = PyModule_Create(&bpy_ops_handlers);

  Py_INCREF(handlers);
  if (PyModule_AddObject(submodule, "handlers", handlers) < 0) {
    Py_DECREF(submodule);
    Py_DECREF(handlers);
    return NULL;
  }

  return submodule;
}
