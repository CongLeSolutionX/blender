/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_shader_interface.hh"
#include "vk_backend.hh"
#include "vk_context.hh"

namespace blender::gpu {

static VkFormat to_vk_dummy_format(const shader::Type type)
{
  switch (type) {
    case shader::Type::FLOAT:
    case shader::Type::VEC2:
    case shader::Type::VEC3:
    case shader::Type::VEC4:
    case shader::Type::MAT3:
    case shader::Type::MAT4:
      return VK_FORMAT_R32_SFLOAT;
    case shader::Type::UINT:
    case shader::Type::UVEC2:
    case shader::Type::UVEC3:
    case shader::Type::UVEC4:
      return VK_FORMAT_R32_UINT;
    case shader::Type::INT:
    case shader::Type::IVEC2:
    case shader::Type::IVEC3:
    case shader::Type::IVEC4:
    case shader::Type::BOOL:
      return VK_FORMAT_R32_SINT;
    case shader::Type::UCHAR:
    case shader::Type::UCHAR2:
    case shader::Type::UCHAR3:
    case shader::Type::UCHAR4:
      return VK_FORMAT_R8_UINT;
    case shader::Type::CHAR:
    case shader::Type::CHAR2:
    case shader::Type::CHAR3:
    case shader::Type::CHAR4:
      return VK_FORMAT_R8_SINT;
    case shader::Type::VEC3_101010I2:
    default:
      BLI_assert_unreachable();
  }
  return VK_FORMAT_MAX_ENUM;
}

void VKShaderInterface::init(const shader::ShaderCreateInfo &info)
{
  static char PUSH_CONSTANTS_FALLBACK_NAME[] = "push_constants_fallback";
  static size_t PUSH_CONSTANTS_FALLBACK_NAME_LEN = strlen(PUSH_CONSTANTS_FALLBACK_NAME);

  using namespace blender::gpu::shader;
  shader_builtins_ = info.builtins_;

  attr_len_ = info.vertex_inputs_.size();
  uniform_len_ = info.push_constants_.size();
  ssbo_len_ = 0;
  ubo_len_ = 0;
  image_offset_ = -1;
  int image_max_binding = -1;
  Vector<ShaderCreateInfo::Resource> all_resources;
  all_resources.extend(info.pass_resources_);
  all_resources.extend(info.batch_resources_);

  for (ShaderCreateInfo::Resource &res : all_resources) {
    switch (res.bind_type) {
      case ShaderCreateInfo::Resource::BindType::IMAGE:
        uniform_len_++;
        image_max_binding = max_ii(image_max_binding, res.slot);
        break;
      case ShaderCreateInfo::Resource::BindType::SAMPLER:
        image_offset_ = max_ii(image_offset_, res.slot);
        uniform_len_++;
        break;
      case ShaderCreateInfo::Resource::BindType::UNIFORM_BUFFER:
        ubo_len_++;
        break;
      case ShaderCreateInfo::Resource::BindType::STORAGE_BUFFER:
        ssbo_len_++;
        break;
      case ShaderCreateInfo::Resource::BindType::INPUT_ATTACHMENT:
        break;
    }
  }
  uniform_len_ += info.subpass_inputs_.size();
  /* Reserve 1 uniform buffer for push constants fallback. */
  size_t names_size = info.interface_names_size_;
  const VKDevice &device = VKBackend::get().device_get();
  const VKPushConstants::StorageType push_constants_storage_type =
      VKPushConstants::Layout::determine_storage_type(info, device);
  if (push_constants_storage_type == VKPushConstants::StorageType::UNIFORM_BUFFER) {
    ubo_len_++;
    names_size += PUSH_CONSTANTS_FALLBACK_NAME_LEN + 1;
  }

  /* Make sure that the image slots don't overlap with other sampler or image slots. */
  image_offset_++;
  if (image_offset_ != 0 && image_offset_ <= image_max_binding) {
    image_offset_ = image_max_binding + 1;
  }

  int32_t input_tot_len = attr_len_ + ubo_len_ + uniform_len_ + ssbo_len_;
  inputs_ = static_cast<ShaderInput *>(
      MEM_calloc_arrayN(input_tot_len, sizeof(ShaderInput), __func__));
  ShaderInput *input = inputs_;

  name_buffer_ = (char *)MEM_mallocN(names_size, "name_buffer");
  uint32_t name_buffer_offset = 0;
  dummy_formats_.resize(16);
  /* Attributes */
  for (const ShaderCreateInfo::VertIn &attr : info.vertex_inputs_) {
    copy_input_name(input, attr.name, name_buffer_, name_buffer_offset);
    input->location = input->binding = attr.index;
    dummy_formats_[attr.index] = to_vk_dummy_format(attr.type);
    if (input->location != -1) {
      enabled_attr_mask_ |= (1 << input->location);

      /* Used in `GPU_shader_get_attribute_info`. */
      attr_types_[input->location] = uint8_t(attr.type);
    }

    input++;
  }

  /* Uniform blocks */
  for (const ShaderCreateInfo::Resource &res : all_resources) {
    if (res.bind_type == ShaderCreateInfo::Resource::BindType::UNIFORM_BUFFER) {
      copy_input_name(input, res.uniformbuf.name, name_buffer_, name_buffer_offset);
      input->location = input->binding = res.slot;
      input++;
    }
  }

  /* Input attachments */
  for (const ShaderCreateInfo::FragOut &res : info.subpass_inputs_) {
    copy_input_name(input, res.name, name_buffer_, name_buffer_offset);
    input->location = input->binding = res.index;
    input++;
  }
  /* Add push constant when using uniform buffer as fallback. */
  int32_t push_constants_fallback_location = -1;
  if (push_constants_storage_type == VKPushConstants::StorageType::UNIFORM_BUFFER) {
    copy_input_name(input, PUSH_CONSTANTS_FALLBACK_NAME, name_buffer_, name_buffer_offset);
    input->location = input->binding = -1;
    input++;
  }

  /* Images, Samplers and buffers. */
  for (const ShaderCreateInfo::Resource &res : all_resources) {
    if (res.bind_type == ShaderCreateInfo::Resource::BindType::SAMPLER) {
      copy_input_name(input, res.sampler.name, name_buffer_, name_buffer_offset);
      input->location = input->binding = res.slot;
      input++;
    }
    else if (res.bind_type == ShaderCreateInfo::Resource::BindType::IMAGE) {
      copy_input_name(input, res.image.name, name_buffer_, name_buffer_offset);
      input->location = input->binding = res.slot + image_offset_;
      input++;
    }
  }

  /* Push constants. */
  int32_t push_constant_location = 1024;
  for (const ShaderCreateInfo::PushConst &push_constant : info.push_constants_) {
    copy_input_name(input, push_constant.name, name_buffer_, name_buffer_offset);
    input->location = push_constant_location++;
    input->binding = -1;
    input++;
  }

  /* Storage buffers */
  for (const ShaderCreateInfo::Resource &res : all_resources) {
    if (res.bind_type == ShaderCreateInfo::Resource::BindType::STORAGE_BUFFER) {
      copy_input_name(input, res.storagebuf.name, name_buffer_, name_buffer_offset);
      input->location = input->binding = res.slot;
      input++;
    }
  }

  sort_inputs();

  /* Builtin Uniforms */
  for (int32_t u_int = 0; u_int < GPU_NUM_UNIFORMS; u_int++) {
    GPUUniformBuiltin u = static_cast<GPUUniformBuiltin>(u_int);
    const ShaderInput *uni = this->uniform_get(builtin_uniform_name(u));
    builtins_[u] = (uni != nullptr) ? uni->location : -1;
  }

  /* Builtin Uniforms Blocks */
  for (int32_t u_int = 0; u_int < GPU_NUM_UNIFORM_BLOCKS; u_int++) {
    GPUUniformBlockBuiltin u = static_cast<GPUUniformBlockBuiltin>(u_int);
    const ShaderInput *block = this->ubo_get(builtin_uniform_block_name(u));
    builtin_blocks_[u] = (block != nullptr) ? block->binding : -1;
  }

  /* Determine the descriptor set locations after the inputs have been sorted. */
  /* Note: input_tot_len is sometimes more than we need. */
  const uint32_t resources_len = input_tot_len;
  descriptor_set_locations_ = Array<VKDescriptorSet::Location>(resources_len);
  descriptor_set_locations_.fill(-1);
  descriptor_set_bind_types_ = Array<shader::ShaderCreateInfo::Resource::BindType>(resources_len);
  descriptor_set_bind_types_.fill(shader::ShaderCreateInfo::Resource::BindType::UNIFORM_BUFFER);
  uint32_t descriptor_set_location = 0;
  for (ShaderCreateInfo::Resource &res : all_resources) {
    const ShaderInput *input = shader_input_get(res);
    BLI_assert(input);
    descriptor_set_location_update(input, descriptor_set_location++, res.bind_type);
  }
  for (const ShaderCreateInfo::FragOut &res : info.subpass_inputs_) {
    const ShaderInput *input = shader_input_get(
        ShaderCreateInfo::Resource::BindType::INPUT_ATTACHMENT, res.index);
    BLI_assert(input);
    descriptor_set_location_update(
        input, descriptor_set_location++, ShaderCreateInfo::Resource::BindType::INPUT_ATTACHMENT);
  }
  /* Post initializing push constants. */
  /* Determine the binding location of push constants fallback buffer. */
  int32_t push_constant_descriptor_set_location = -1;
  if (push_constants_storage_type == VKPushConstants::StorageType::UNIFORM_BUFFER) {
    push_constant_descriptor_set_location = descriptor_set_location++;
    const ShaderInput *push_constant_input = ubo_get(PUSH_CONSTANTS_FALLBACK_NAME);
    descriptor_set_location_update(push_constant_input,
                                   push_constants_fallback_location,
                                   shader::ShaderCreateInfo::Resource::UNIFORM_BUFFER);
  }
  push_constants_layout_.init(
      info, *this, push_constants_storage_type, push_constant_descriptor_set_location);
}

static int32_t shader_input_index(const ShaderInput *shader_inputs,
                                  const ShaderInput *shader_input)
{
  int32_t index = (shader_input - shader_inputs);
  return index;
}

void VKShaderInterface::descriptor_set_location_update(
    const ShaderInput *shader_input,
    const VKDescriptorSet::Location location,
    const shader::ShaderCreateInfo::Resource::BindType bind_type)
{
  int32_t index = shader_input_index(inputs_, shader_input);
  BLI_assert(descriptor_set_locations_[index].binding == -1);
  descriptor_set_locations_[index] = location;
  descriptor_set_bind_types_[index] = bind_type;
}

const VKDescriptorSet::Location VKShaderInterface::descriptor_set_location(
    const ShaderInput *shader_input) const
{
  int32_t index = shader_input_index(inputs_, shader_input);
  return descriptor_set_locations_[index];
}

const shader::ShaderCreateInfo::Resource::BindType VKShaderInterface::descriptor_set_bind_type(
    const ShaderInput *shader_input) const
{
  int32_t index = shader_input_index(inputs_, shader_input);
  return descriptor_set_bind_types_[index];
}

const VKDescriptorSet::Location VKShaderInterface::descriptor_set_location(
    const shader::ShaderCreateInfo::Resource &resource) const
{
  const ShaderInput *shader_input = shader_input_get(resource);
  BLI_assert(shader_input);
  return descriptor_set_location(shader_input);
}

const std::optional<VKDescriptorSet::Location> VKShaderInterface::descriptor_set_location(
    const shader::ShaderCreateInfo::Resource::BindType &bind_type, int binding) const
{
  const ShaderInput *shader_input = shader_input_get(bind_type, binding);
  if (shader_input == nullptr) {
    return std::nullopt;
  }
  if (descriptor_set_bind_type(shader_input) != bind_type) {
    return std::nullopt;
  }
  return descriptor_set_location(shader_input);
}

const ShaderInput *VKShaderInterface::shader_input_get(
    const shader::ShaderCreateInfo::Resource &resource) const
{
  return shader_input_get(resource.bind_type, resource.slot);
}

const ShaderInput *VKShaderInterface::shader_input_get(
    const shader::ShaderCreateInfo::Resource::BindType &bind_type, int binding) const
{
  switch (bind_type) {
    case shader::ShaderCreateInfo::Resource::BindType::IMAGE:
      /* Not really nice, but the binding namespace between OpenGL and Vulkan don't match. To fix
       * this we need to check if one of both cases return a binding.
       * TODO: we might want to introduce a different API to fix this.  */
      return texture_get((binding >= image_offset_) ? binding : binding + image_offset_);
    case shader::ShaderCreateInfo::Resource::BindType::SAMPLER:
    case shader::ShaderCreateInfo::Resource::BindType::INPUT_ATTACHMENT:
      return texture_get(binding);
    case shader::ShaderCreateInfo::Resource::BindType::STORAGE_BUFFER:
      return ssbo_get(binding);
    case shader::ShaderCreateInfo::Resource::BindType::UNIFORM_BUFFER:
      return ubo_get(binding);
  }
  return nullptr;
}

}  // namespace blender::gpu
