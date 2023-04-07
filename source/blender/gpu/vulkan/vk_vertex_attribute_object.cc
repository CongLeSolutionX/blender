/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

#include "vk_vertex_attribute_object.hh"

#include "vk_batch.hh"
#include "vk_context.hh"
#include "vk_shader.hh"
#include "vk_shader_interface.hh"
#include "vk_vertex_buffer.hh"

#include "BLI_array.hh"

namespace blender::gpu {
VKVertexAttributeObject::VKVertexAttributeObject()
{
  clear();
}

void VKVertexAttributeObject::clear()
{
  is_valid = false;
  info.pNext = NULL;
  bindings.clear();
  attributes.clear();
  vbos.clear();
}

VKVertexAttributeObject &VKVertexAttributeObject::operator=(const VKVertexAttributeObject &other)
{
  if (this == &other) {
    return *this;
  }

  is_valid = other.is_valid;
  info = other.info;
  bindings.clear();
  bindings.extend(other.bindings);
  attributes.clear();
  attributes.extend(other.attributes);
  vbos.clear();
  vbos.extend(other.vbos);
  return *this;
}

void VKVertexAttributeObject::bind(VKContext &context)
{
  Array<bool> visited_bindings(bindings.size());
  visited_bindings.fill(false);

  for (VkVertexInputAttributeDescription attribute : attributes) {
    if (visited_bindings[attribute.binding]) {
      continue;
    }
    visited_bindings[attribute.binding] = true;
    BLI_assert(vbos[attribute.binding]);
    VKVertexBuffer &vbo = *vbos[attribute.binding];
    vbo.upload();
    context.command_buffer_get().bind(attribute.binding, vbo, 0);
  }
}

void VKVertexAttributeObject::update_bindings(const VKContext &context, VKBatch &batch)
{
  clear();
  const VKShaderInterface &interface = unwrap(context.shader)->interface_get();
  AttributeMask occupied_attributes = 0;

  for (int v = 0; v < GPU_BATCH_VBO_MAX_LEN; v++) {
    VKVertexBuffer *vbo = batch.vertex_buffer_get(v);
    if (vbo) {
      update_bindings(*vbo, interface, occupied_attributes, false);
    }
  }

  for (int v = 0; v < GPU_BATCH_INST_VBO_MAX_LEN; v++) {
    VKVertexBuffer *vbo = batch.instance_buffer_get(v);
    if (vbo) {
      update_bindings(*vbo, interface, occupied_attributes, false);
    }
  }

  is_valid = true;

  BLI_assert(interface.enabled_attr_mask_ == occupied_attributes);
}

void VKVertexAttributeObject::update_bindings(VKVertexBuffer &vertex_buffer,
                                              const VKShaderInterface &interface,
                                              AttributeMask &r_occupied_attributes,
                                              const bool use_instancing)
{
  const GPUVertFormat &format = vertex_buffer.format;

  if (format.attr_len <= 0) {
    return;
  }

  uint32_t offset = 0;
  uint32_t stride = format.stride;

  for (uint32_t attribute_index = 0; attribute_index < format.attr_len; attribute_index++) {
    const GPUVertAttr &attribute = format.attrs[attribute_index];
    if (format.deinterleaved) {
      offset += ((attribute_index == 0) ? 0 : format.attrs[attribute_index - 1].size) *
                vertex_buffer.vertex_len;
      stride = attribute.size;
    }
    else {
      offset = attribute.offset;
    }

    const uint32_t binding = bindings.size();

    bool attribute_used_by_shader = false;
    for (uint32_t name_index = 0; name_index < attribute.name_len; name_index++) {
      const char *name = GPU_vertformat_attr_name_get(&format, &attribute, name_index);
      const ShaderInput *shader_input = interface.attr_get(name);
      if (shader_input == nullptr || shader_input->location == -1) {
        continue;
      }

      /* Don't overwrite attributes that are already occupied. */
      AttributeMask attribute_mask = 1 << shader_input->location;
      if (r_occupied_attributes & attribute_mask) {
        continue;
      }
      r_occupied_attributes |= attribute_mask;
      attribute_used_by_shader = true;

      VkVertexInputAttributeDescription attribute_description = {};
      attribute_description.binding = binding;
      attribute_description.location = shader_input->location;
      attribute_description.offset = offset;
      attribute_description.format = to_vk_format(
          static_cast<GPUVertCompType>(attribute.comp_type), attribute.size);
      attributes.append(attribute_description);
    }

    if (attribute_used_by_shader) {
      VkVertexInputBindingDescription vk_binding_descriptor = {};
      vk_binding_descriptor.binding = binding;
      vk_binding_descriptor.stride = stride;
      vk_binding_descriptor.inputRate = use_instancing ? VK_VERTEX_INPUT_RATE_INSTANCE :
                                                         VK_VERTEX_INPUT_RATE_VERTEX;
      bindings.append(vk_binding_descriptor);
      vbos.append(&vertex_buffer);
    }
  }
}

}  // namespace blender::gpu
