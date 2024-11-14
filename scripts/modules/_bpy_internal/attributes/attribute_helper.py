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
  if name not in attributes:
    return None
  
  attr = attributes[name]
  domain_size = attributes.domain_size(attr.domain)
  shape = (domain_size, *_attribute_value_shape[attr.data_type])

  data = np.zeros(shape, dtype=_attribute_value_dtype[attr.data_type])
  attr.data.foreach_get(_attribute_value_string[attr.data_type], np.ravel(data))
  return data

class AttributeContextWriter(AbstractContextManager):
  __slots__ = ()

  def __init__(self, attributes, name):
    self.attributes = attributes
    self.name = name

  def __enter__(self):
    # TODO: Handle errors correctly
    if self.name not in self.attributes:
      return None
    
    self.attr = self.attributes[self.name]
    domain_size = self.attributes.domain_size(self.attr.domain)
    shape = (domain_size, *_attribute_value_shape[self.attr.data_type])

    self.data = np.zeros(shape, dtype=_attribute_value_dtype[self.attr.data_type])
    self.attr.data.foreach_get(_attribute_value_string[self.attr.data_type], np.ravel(self.data))
    return self.data
    
  def __exit__(self, *exc_details):
    # TODO: Handle errors correctly
    if self.name in self.attributes:
      self.attr.data.foreach_set(_attribute_value_string[self.attr.data_type], np.ravel(self.data))
