/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#include "vk_pipeline.hh"
#include "vk_context.hh"
#include "vk_framebuffer.hh"
#include "vk_memory.hh"

namespace blender::gpu {

VKPipeline::VKPipeline(VkDescriptorSetLayout vk_descriptor_set_layout,
                       VKPushConstants &&push_constants)
    : vk_pipeline_(VK_NULL_HANDLE),
      descriptor_set_(vk_descriptor_set_layout),
      push_constants_(std::move(push_constants))
{
}

VKPipeline::VKPipeline(VkPipeline vk_pipeline,
                       VkDescriptorSetLayout vk_descriptor_set_layout,
                       VKPushConstants &&push_constants)
    : vk_pipeline_(vk_pipeline),
      descriptor_set_(vk_descriptor_set_layout),
      push_constants_(std::move(push_constants))
{
}

VKPipeline::~VKPipeline()
{
  VK_ALLOCATION_CALLBACKS
  VkDevice vk_device = VKContext::get()->device_get();
  if (vk_pipeline_ != VK_NULL_HANDLE) {
    vkDestroyPipeline(vk_device, vk_pipeline_, vk_allocation_callbacks);
  }
}

VKPipeline VKPipeline::create_compute_pipeline(
    VKContext &context,
    VkShaderModule compute_module,
    VkDescriptorSetLayout &descriptor_set_layout,
    VkPipelineLayout &pipeline_layout,
    const VKPushConstants::Layout &push_constants_layout)
{
  VK_ALLOCATION_CALLBACKS
  VkDevice vk_device = context.device_get();
  VkComputePipelineCreateInfo pipeline_info = {};
  pipeline_info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
  pipeline_info.flags = 0;
  pipeline_info.stage = {};
  pipeline_info.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  pipeline_info.stage.flags = 0;
  pipeline_info.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
  pipeline_info.stage.module = compute_module;
  pipeline_info.layout = pipeline_layout;
  pipeline_info.stage.pName = "main";

  VkPipeline vk_pipeline;
  if (vkCreateComputePipelines(
          vk_device, nullptr, 1, &pipeline_info, vk_allocation_callbacks, &vk_pipeline) !=
      VK_SUCCESS) {
    return VKPipeline();
  }

  VKPushConstants push_constants(&push_constants_layout);
  return VKPipeline(vk_pipeline, descriptor_set_layout, std::move(push_constants));
}

VKPipeline VKPipeline::create_graphics_pipeline(
    VkDescriptorSetLayout &descriptor_set_layout,
    const VKPushConstants::Layout &push_constants_layout)
{
  VKPushConstants push_constants(&push_constants_layout);
  return VKPipeline(descriptor_set_layout, std::move(push_constants));
}

VkPipeline VKPipeline::vk_handle() const
{
  return vk_pipeline_;
}

bool VKPipeline::is_valid() const
{
  return vk_pipeline_ != VK_NULL_HANDLE;
}

void VKPipeline::finalize(VKContext &context,
                          VkShaderModule vertex_module,
                          VkShaderModule fragment_module,
                          VkPipelineLayout &pipeline_layout)
{
  BLI_assert(vertex_module != VK_NULL_HANDLE);

  VK_ALLOCATION_CALLBACKS

  Vector<VkPipelineShaderStageCreateInfo> pipeline_stages;
  VkPipelineShaderStageCreateInfo vertex_stage_info = {};
  vertex_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  vertex_stage_info.stage = VK_SHADER_STAGE_VERTEX_BIT;
  vertex_stage_info.module = vertex_module;
  vertex_stage_info.pName = "main";
  pipeline_stages.append(vertex_stage_info);
  /*
    if (geometry_module_ != VK_NULL_HANDLE) {
      VkPipelineShaderStageCreateInfo geo_stage_info = {};
      geo_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      geo_stage_info.stage = VK_SHADER_STAGE_GEOMETRY_BIT;
      geo_stage_info.module = geometry_module_;
      geo_stage_info.pName = "main";
      pipeline_stages.append(geo_stage_info);
    }*/
  if (fragment_module != VK_NULL_HANDLE) {
    VkPipelineShaderStageCreateInfo fragment_stage_info = {};
    fragment_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragment_stage_info.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragment_stage_info.module = fragment_module;
    fragment_stage_info.pName = "main";
    pipeline_stages.append(fragment_stage_info);
  }

  VKFrameBuffer &framebuffer = *context.active_framebuffer_get();

  VkGraphicsPipelineCreateInfo pipeline_create_info = {};
  pipeline_create_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
  pipeline_create_info.stageCount = pipeline_stages.size();
  pipeline_create_info.pStages = pipeline_stages.data();
  pipeline_create_info.layout = pipeline_layout;
  pipeline_create_info.renderPass = framebuffer.vk_render_pass_get();
  pipeline_create_info.subpass = 0;

  /* Vertex input state. */
  VkPipelineVertexInputStateCreateInfo vertex_input_state = {};
  vertex_input_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
  vertex_input_state.vertexBindingDescriptionCount = 0;
  /* Dummy attribute containing the vertex positions. These should be extracted from shader create
   * infos. */
  VkVertexInputBindingDescription binding_description = {};
  binding_description.binding = 0;
  binding_description.stride = 4 * 3;
  binding_description.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
  vertex_input_state.vertexBindingDescriptionCount = 1;
  vertex_input_state.pVertexBindingDescriptions = &binding_description;
  VkVertexInputAttributeDescription attribute_description = {};
  attribute_description.location = 0;
  attribute_description.binding = 0;
  attribute_description.format = VK_FORMAT_R32G32B32_SFLOAT;
  attribute_description.offset = 0;
  vertex_input_state.vertexAttributeDescriptionCount = 1;
  vertex_input_state.pVertexAttributeDescriptions = &attribute_description;
  pipeline_create_info.pVertexInputState = &vertex_input_state;

  /* Input assembly state. */
  VkPipelineInputAssemblyStateCreateInfo pipeline_input_assembly = {};
  pipeline_input_assembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
  pipeline_input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;
  pipeline_create_info.pInputAssemblyState = &pipeline_input_assembly;

  VkPipelineRasterizationStateCreateInfo rasterization_state = {};
  rasterization_state.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
  pipeline_create_info.pRasterizationState = &rasterization_state;
  /* TODO: Needs to be sourced from GPU_state. */
  rasterization_state.lineWidth = 1.0;

  /* Viewport state. */
  VkPipelineViewportStateCreateInfo viewport_state = {};
  viewport_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
  VkViewport viewport = framebuffer.vk_viewport_get();
  viewport_state.pViewports = &viewport;
  viewport_state.viewportCount = 1;
  VkRect2D scissor = framebuffer.vk_render_area_get();
  viewport_state.pScissors = &scissor;
  viewport_state.scissorCount = 1;
  pipeline_create_info.pViewportState = &viewport_state;

  /* Multisample state. */
  VkPipelineMultisampleStateCreateInfo multisample_state = {};
  multisample_state.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
  multisample_state.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
  multisample_state.minSampleShading = 1.0f;
  pipeline_create_info.pMultisampleState = &multisample_state;

  /* Color blend state. */
  VkPipelineColorBlendStateCreateInfo pipeline_color_blend_state = {};
  pipeline_color_blend_state.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
  VkPipelineColorBlendAttachmentState color_blend_attachment = {};
  color_blend_attachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                                          VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
  pipeline_color_blend_state.attachmentCount = 1;
  pipeline_color_blend_state.pAttachments = &color_blend_attachment;
  pipeline_create_info.pColorBlendState = &pipeline_color_blend_state;

  VkDevice vk_device = context.device_get();
  vkCreateGraphicsPipelines(
      vk_device, VK_NULL_HANDLE, 1, &pipeline_create_info, vk_allocation_callbacks, &vk_pipeline_);
}

}  // namespace blender::gpu
