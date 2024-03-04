/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_render_graph_resources.hh"

#include "BLI_index_range.hh"

namespace blender::gpu {

/* -------------------------------------------------------------------- */
/** \name Adding resources
 * \{ */

void VKRenderGraphResources::add_image(VkImage vk_image,
                                       VkImageLayout vk_image_layout,
                                       ResourceOwner owner)
{
  BLI_assert_msg(!image_resources_.contains(vk_image),
                 "Image resource is added twice to the render graph.");
  ResourceHandle handle = resources_.allocate();
  Resource &resource = resources_.get(handle);
  image_resources_.add_new(vk_image, handle);

  resource.owner = owner;
  resource.vk_buffer = VK_NULL_HANDLE;
  resource.vk_image = vk_image;
  resource.vk_image_layout = vk_image_layout;
  resource.graph_version = 0;
  resource.queue_version = 0;
}

void VKRenderGraphResources::add_buffer(VkBuffer vk_buffer)
{
  BLI_assert_msg(!buffer_resources_.contains(vk_buffer),
                 "Buffer resource is added twice to the render graph.");
  ResourceHandle handle = resources_.allocate();
  Resource &resource = resources_.get(handle);
  buffer_resources_.add_new(vk_buffer, handle);

  resource.owner = ResourceOwner::APPLICATION;
  resource.vk_buffer = vk_buffer;
  resource.vk_image = VK_NULL_HANDLE;
  resource.vk_image_layout = VK_IMAGE_LAYOUT_UNDEFINED;
  resource.queue_version = 0;
  resource.graph_version = 0;
}

/** \} */

ResourceHandle VKRenderGraphResources::get_image_handle(VkImage vk_image) const
{
  return image_resources_.lookup(vk_image);
}
ResourceHandle VKRenderGraphResources::get_buffer_handle(VkBuffer vk_buffer) const
{
  return buffer_resources_.lookup(vk_buffer);
}

VersionedResource VKRenderGraphResources::get_and_increase_version(ResourceHandle handle,
                                                                   Resource &resource)
{
  VersionedResource result;
  result.handle = handle;
  result.version = resource.graph_version;
  resource.graph_version += 1;
  return result;
}

VersionedResource VKRenderGraphResources::get_image_and_increase_version(VkImage vk_image)
{
  ResourceHandle handle = get_image_handle(vk_image);
  Resource &resource = resources_.get(handle);
  return get_and_increase_version(handle, resource);
}

VersionedResource VKRenderGraphResources::get_buffer_and_increase_version(VkBuffer vk_buffer)
{
  ResourceHandle handle = get_buffer_handle(vk_buffer);
  Resource &resource = resources_.get(handle);
  return get_and_increase_version(handle, resource);
}

}  // namespace blender::gpu
