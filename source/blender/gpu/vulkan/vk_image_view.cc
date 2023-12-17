/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_image_view.hh"
#include "vk_backend.hh"
#include "vk_debug.hh"
#include "vk_device.hh"
#include "vk_memory.hh"
#include "vk_texture.hh"

namespace blender::gpu {
void vk_image_view_info_fill(VkImageViewCreateInfo &vk_image_view_info,
                                    bool use_stencil,
                                    IndexRange mip_range,
                                    IndexRange layer_range)
{
  const VkImageAspectFlags allowed_bits = VK_IMAGE_ASPECT_COLOR_BIT |
                                          (use_stencil ? VK_IMAGE_ASPECT_STENCIL_BIT :
                                                         VK_IMAGE_ASPECT_DEPTH_BIT);
  vk_image_view_info.subresourceRange.aspectMask = vk_image_view_info.subresourceRange.aspectMask &
                                                   allowed_bits;
  vk_image_view_info.subresourceRange.baseMipLevel = mip_range.first();
  vk_image_view_info.subresourceRange.levelCount = mip_range.size();

  auto layer_size = layer_range.size();
  vk_image_view_info.subresourceRange.baseArrayLayer = (layer_size == 0) ? 0 : layer_range.first();
  vk_image_view_info.subresourceRange.layerCount = (layer_size == 0) ? VK_REMAINING_ARRAY_LAYERS :
                                                                       layer_size;
}

VKImageView::VKImageView(const VkImageViewCreateInfo &vk_image_view_info,
                         bool use_stencil,
                         IndexRange mip_range,
                         IndexRange layer_range)
{
  vk_format_ = vk_image_view_info.format;
  use_stencil_ = use_stencil;
  view_type_ = vk_image_view_info.viewType;
  mip_range_ = mip_range;
  layer_range_ = layer_range;
  VK_ALLOCATION_CALLBACKS
  const VKDevice &device = VKBackend::get().device_get();
  vkCreateImageView(
      device.device_get(), &vk_image_view_info, vk_allocation_callbacks, &vk_image_view_);
}

VKImageView::~VKImageView()
{
  if (vk_image_view_ != VK_NULL_HANDLE) {
    VKDevice &device = VKBackend::get().device_get();
    device.discard_image_view(vk_image_view_);
    vk_image_view_ = VK_NULL_HANDLE;
  }
}

bool vk_image_view_equal(std::weak_ptr<VKImageView> a, std::weak_ptr<VKImageView> b)
{
  return a.lock()->vk_handle() == b.lock()->vk_handle();
}

}  // namespace blender::gpu
