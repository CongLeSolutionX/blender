/* SPDX-FileCopyrightText: 2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */
#include "vk_context.hh"
#include "vk_debug.hh"

#include "vk_backend.hh"
#include "vk_framebuffer.hh"
#include "vk_immediate.hh"
#include "vk_memory.hh"
#include "vk_shader.hh"
#include "vk_shader_interface.hh"
#include "vk_state_manager.hh"
#include "vk_texture.hh"

#include "GHOST_C-api.h"

namespace blender::gpu {

VKContext::VKContext(void *ghost_window, void *ghost_context)
{
  ghost_window_ = ghost_window;
  ghost_context_ = ghost_context;

  state_manager = new VKStateManager();
  imm = new VKImmediate();

  /* For off-screen contexts. Default frame-buffer is empty. */
  VKFrameBuffer *framebuffer = new VKFrameBuffer("back_left");
  back_left = framebuffer;
  active_fb = framebuffer;
}

VKContext::~VKContext()
{
  if (surface_texture_) {
    GPU_texture_free(surface_texture_);
    surface_texture_ = nullptr;
  }
  VKBackend::get().device_.context_unregister(*this);

  delete imm;
  imm = nullptr;
  delete back_left;
  back_left = nullptr;
  destroy_discarded_resources();
}

void VKContext::sync_backbuffer()
{
  if (ghost_context_) {
    VKDevice &device = VKBackend::get().device_;
    if (!command_buffers_.is_initialized()) {
      command_buffers_.init(device);
      descriptor_pools_.init(device);
      device.init_dummy_buffer(*this);
    }
    descriptor_pools_.reset();
  }

  if (ghost_window_) {
    GHOST_VulkanSwapChainData swap_chain_data = {};
    GHOST_GetVulkanSwapChainFormat((GHOST_WindowHandle)ghost_window_, &swap_chain_data);

    const bool reset_framebuffer = swap_chain_format_ != swap_chain_data.format ||
                                   vk_extent_.width != swap_chain_data.extent.width ||
                                   vk_extent_.height != swap_chain_data.extent.height;
    if (reset_framebuffer) {
      if (has_active_framebuffer()) {
        deactivate_framebuffer();
      }
      if (surface_texture_) {
        GPU_texture_free(surface_texture_);
        surface_texture_ = nullptr;
      }
      surface_texture_ = GPU_texture_create_2d("back-left",
                                               swap_chain_data.extent.width,
                                               swap_chain_data.extent.height,
                                               1,
                                               to_gpu_format(swap_chain_data.format),
                                               GPU_TEXTURE_USAGE_ATTACHMENT,
                                               nullptr);

      GPUAttachment config[] = {GPU_ATTACHMENT_NONE, GPU_ATTACHMENT_TEXTURE(surface_texture_)};
      GPU_framebuffer_config_array(reinterpret_cast<GPUFrameBuffer *>(back_left),
                                   config,
                                   sizeof(config) / sizeof(GPUAttachment));
      back_left->bind(false);

      swap_chain_format_ = swap_chain_data.format;
      vk_extent_ = swap_chain_data.extent;
    }
  }
}

void VKContext::activate()
{
  /* Make sure no other context is already bound to this thread. */
  BLI_assert(is_active_ == false);

  is_active_ = true;

  sync_backbuffer();

  immActivate();
}

void VKContext::deactivate()
{
  immDeactivate();
  is_active_ = false;
}

void VKContext::begin_frame() {}

void VKContext::end_frame()
{
  destroy_discarded_resources();
}

void VKContext::flush()
{
  command_buffers_.submit();
}

void VKContext::finish()
{
  command_buffers_.finish();
}

void VKContext::memory_statistics_get(int *r_total_mem_kb, int *r_free_mem_kb)
{
  const VKDevice &device = VKBackend::get().device_get();
  device.memory_statistics_get(r_total_mem_kb, r_free_mem_kb);
}

/* -------------------------------------------------------------------- */
/** \name State manager
 * \{ */

VKStateManager &VKContext::state_manager_get() const
{
  return *static_cast<VKStateManager *>(state_manager);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Frame-buffer
 * \{ */

void VKContext::activate_framebuffer(VKFrameBuffer &framebuffer)
{
  if (has_active_framebuffer()) {
    deactivate_framebuffer();
  }

  BLI_assert(active_fb == nullptr);
  active_fb = &framebuffer;
  framebuffer.update_size();
  framebuffer.update_srgb();
  command_buffers_get().begin_render_pass(framebuffer);
}

VKFrameBuffer *VKContext::active_framebuffer_get() const
{
  return unwrap(active_fb);
}

bool VKContext::has_active_framebuffer() const
{
  return active_framebuffer_get() != nullptr;
}

void VKContext::deactivate_framebuffer()
{
  VKFrameBuffer *framebuffer = active_framebuffer_get();
  BLI_assert(framebuffer != nullptr);
  command_buffers_get().end_render_pass(*framebuffer);
  active_fb = nullptr;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Compute pipeline
 * \{ */

void VKContext::bind_compute_pipeline()
{
  VKShader *shader = unwrap(this->shader);
  BLI_assert(shader);
  shader->specialzation_ensure();
  VKPipeline &pipeline = shader->pipeline_get();
  pipeline.bind(*this, VK_PIPELINE_BIND_POINT_COMPUTE);
  pipeline.update_push_constants(*this);
  if (shader->has_descriptor_set()) {
    descriptor_set_.bind(*this, shader->vk_pipeline_layout_get(), VK_PIPELINE_BIND_POINT_COMPUTE);
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Graphics pipeline
 * \{ */

void VKContext::bind_graphics_pipeline(const GPUPrimType prim_type,
                                       const VKVertexAttributeObject &vertex_attribute_object)
{
  VKShader *shader = unwrap(this->shader);
  BLI_assert(shader);
  /*
  BLI_assert_msg(
      prim_type != GPU_PRIM_POINTS || shader->interface_get().is_point_shader(),
      "GPU_PRIM_POINTS is used with a shader that doesn't set point size before "
      "drawing fragments. Calling code should be adapted to use a shader that sets the "
      "gl_PointSize before entering the fragment stage. For example `GPU_SHADER_3D_POINT_*`.");
  */
  shader->update_graphics_pipeline(*this, prim_type, vertex_attribute_object);

  VKPipeline &pipeline = shader->pipeline_get();
  pipeline.bind(*this, VK_PIPELINE_BIND_POINT_GRAPHICS);
  pipeline.update_push_constants(*this);
  if (shader->has_descriptor_set()) {
    descriptor_set_.bind(*this, shader->vk_pipeline_layout_get(), VK_PIPELINE_BIND_POINT_GRAPHICS);
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Graphics pipeline
 * \{ */

void VKContext::swap_buffers_pre_callback(const GHOST_VulkanSwapChainData *swap_chain_data)
{
  VKContext *context = VKContext::get();
  BLI_assert(context);
  context->swap_buffers_pre_handler(*swap_chain_data);
}

void VKContext::swap_buffers_post_callback()
{
  VKContext *context = VKContext::get();
  BLI_assert(context);
  context->swap_buffers_post_handler();
}

void VKContext::swap_buffers_pre_handler(const GHOST_VulkanSwapChainData &swap_chain_data)
{
  /*
   * Ensure no graphics/compute commands are scheduled. They could use the back buffer, which
   * layout is altered here.
   */
  command_buffers_get().submit();

  VKFrameBuffer &framebuffer = *unwrap(back_left);

  VKTexture wrapper("display_texture");
  wrapper.init(swap_chain_data.image,
               VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
               to_gpu_format(swap_chain_data.format));
  wrapper.layout_ensure(*this,
                        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                        VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
                        VK_PIPELINE_STAGE_TRANSFER_BIT,
                        VK_ACCESS_TRANSFER_WRITE_BIT);

  VKTexture *color_attachment = unwrap(unwrap(framebuffer.color_tex(0)));
  BLI_assert(color_attachment->best_layout_get() == VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
  color_attachment->layout_ensure(*this,
                                  VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                                  VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                                  VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
                                  VK_PIPELINE_STAGE_TRANSFER_BIT,
                                  VK_ACCESS_TRANSFER_READ_BIT);

  VkImageBlit image_blit = {};
  image_blit.srcOffsets[0] = {0, color_attachment->height_get() - 1, 0};
  image_blit.srcOffsets[1] = {color_attachment->width_get(), 0, 1};
  image_blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  image_blit.srcSubresource.mipLevel = 0;
  image_blit.srcSubresource.baseArrayLayer = 0;
  image_blit.srcSubresource.layerCount = 1;

  image_blit.dstOffsets[0] = {0, 0, 0};
  image_blit.dstOffsets[1] = {
      int32_t(swap_chain_data.extent.width), int32_t(swap_chain_data.extent.height), 1};
  image_blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  image_blit.dstSubresource.mipLevel = 0;
  image_blit.dstSubresource.baseArrayLayer = 0;
  image_blit.dstSubresource.layerCount = 1;

  command_buffers_get().blit(wrapper,
                             VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                             *color_attachment,
                             VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                             Span<VkImageBlit>(&image_blit, 1));
  wrapper.layout_ensure(*this,
                        VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                        VK_PIPELINE_STAGE_TRANSFER_BIT,
                        VK_ACCESS_TRANSFER_WRITE_BIT,
                        VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
                        VK_ACCESS_MEMORY_READ_BIT);

  color_attachment->layout_ensure(*this,
                                  VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                                  VK_PIPELINE_STAGE_TRANSFER_BIT,
                                  VK_ACCESS_TRANSFER_READ_BIT,
                                  VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                                  VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT);
  command_buffers_get().submit();
}

void VKContext::swap_buffers_post_handler()
{
  sync_backbuffer();
}

void VKContext::discard_image(VkImage vk_image, VmaAllocation vma_allocation)
{
  discarded_images_.append(std::pair(vk_image, vma_allocation));
}

void VKContext::discard_image_view(VkImageView vk_image_view)
{
  discarded_image_views_.append(vk_image_view);
}

void VKContext::discard_buffer(VkBuffer vk_buffer, VmaAllocation vma_allocation)
{
  discarded_buffers_.append(std::pair(vk_buffer, vma_allocation));
}

void VKContext::discard_render_pass(VkRenderPass vk_render_pass)
{
  discarded_render_passes_.append(vk_render_pass);
}

void VKContext::discard_frame_buffer(VkFramebuffer vk_frame_buffer)
{
  discarded_frame_buffers_.append(vk_frame_buffer);
}

void VKContext::destroy_discarded_resources()
{
  VK_ALLOCATION_CALLBACKS
  VKDevice &device = VKBackend::get().device_get();
  while (!discarded_image_views_.is_empty()) {
    VkImageView vk_image_view = discarded_image_views_.pop_last();
    vkDestroyImageView(device.device_get(), vk_image_view, vk_allocation_callbacks);
  }

  while (!discarded_images_.is_empty()) {
    std::pair<VkImage, VmaAllocation> image_allocation = discarded_images_.pop_last();
    vmaDestroyImage(device.mem_allocator_get(), image_allocation.first, image_allocation.second);
  }

  while (!discarded_buffers_.is_empty()) {
    std::pair<VkBuffer, VmaAllocation> buffer_allocation = discarded_buffers_.pop_last();
    vmaDestroyBuffer(
        device.mem_allocator_get(), buffer_allocation.first, buffer_allocation.second);
  }

  while (!discarded_render_passes_.is_empty()) {
    VkRenderPass vk_render_pass = discarded_render_passes_.pop_last();
    vkDestroyRenderPass(device.device_get(), vk_render_pass, vk_allocation_callbacks);
  }

  while (!discarded_frame_buffers_.is_empty()) {
    VkFramebuffer vk_frame_buffer = discarded_frame_buffers_.pop_last();
    vkDestroyFramebuffer(device.device_get(), vk_frame_buffer, vk_allocation_callbacks);
  }
}
/** \} */

}  // namespace blender::gpu
