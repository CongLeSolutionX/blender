# SPDX-FileCopyrightText: 2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later
import numpy as np
from contextlib import AbstractContextManager

_attribute_value_string = {
  'FLOAT': "value",
  'INT': "value",
  'FLOAT_VECTOR': "vector",
  'FLOAT_COLOR': "color",
  'BYTE_COLOR': "color",
  'STRING': "value",
  'BOOLEAN': "value",
  'FLOAT2': "value",
  'INT8': "value",
  'INT32_2D': "value",
  'QUATERNION': "value",
  'FLOAT4X4': "value",
}

_attribute_value_dtype = {
  'FLOAT': np.float32,
  'INT': np.dtype('int'),
  'FLOAT_VECTOR': np.float32,
  'FLOAT_COLOR': np.float32,
  'BYTE_COLOR': np.int8,
  'STRING': np.dtype('str'),
  'BOOLEAN': np.dtype('bool'),
  'FLOAT2': np.float32,
  'INT8': np.int8,
  'INT32_2D': np.dtype('int'),
  'QUATERNION': np.float32,
  'FLOAT4X4': np.float32,
}

_attribute_value_shape = {
  'FLOAT': (),
  'INT': (),
  'FLOAT_VECTOR': (3,),
  'FLOAT_COLOR': (4,),
  'BYTE_COLOR': (4,),
  'STRING': (),
  'BOOLEAN': (),
  'FLOAT2':(2,),
  'INT8': (),
  'INT32_2D': (2,),
  'QUATERNION': (4,),
  'FLOAT4X4': (4,4),
}

def get_attribute(attributes, name):
  attr = attributes[name]
  shape = (len(attr.data), *_attribute_value_shape[attr.data_type])

  data = np.empty(shape, dtype=_attribute_value_dtype[attr.data_type])
  attr.data.foreach_get(_attribute_value_string[attr.data_type], np.ravel(data))
  return data

class AttributeContextWriter(AbstractContextManager):
  __slots__ = ()

  def __init__(self, attributes, name):
    self.attributes = attributes
    self.name = name

  def __enter__(self):
    # Store the reference to the data to write it back on __exit__.
    self.data = get_attribute(self.attributes, self.name)
    return self.data
    
  def __exit__(self, *exc_details):
    attr = self.attributes[self.name]
    attr.data.foreach_set(_attribute_value_string[attr.data_type], np.ravel(self.data))
