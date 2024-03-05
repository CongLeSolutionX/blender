/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_render_graph_nodes.hh"

namespace blender::gpu {

NodeHandle VKRenderGraphNodes::add_clear_image_node(
    VkImage vk_image,
    VkClearColorValue &vk_clear_color_value,
    VkImageSubresourceRange &vk_image_subresource_range)
{
  NodeHandle handle = allocate();
  Node &node = nodes_.get(handle);
  BLI_assert(node.type == Node::Type::UNUSED);

  node.type = Node::Type::CLEAR_COLOR_IMAGE;
  node.clear_color_image.vk_image = vk_image;
  node.clear_color_image.vk_clear_color_value = vk_clear_color_value;
  node.clear_color_image.vk_image_subresource_range = vk_image_subresource_range;

  return handle;
}

NodeHandle VKRenderGraphNodes::add_fill_buffer_node(VkBuffer vk_buffer,
                                                    VkDeviceSize size,
                                                    uint32_t data)
{
  NodeHandle handle = allocate();
  Node &node = nodes_.get(handle);
  BLI_assert(node.type == Node::Type::UNUSED);

  node.type = Node::Type::FILL_BUFFER;
  node.fill_buffer.vk_buffer = vk_buffer;
  node.fill_buffer.size = size;
  node.fill_buffer.data = data;

  return handle;
}

NodeHandle VKRenderGraphNodes::add_copy_buffer_node(VkBuffer src_buffer,
                                                    VkBuffer dst_buffer,
                                                    const VkBufferCopy &region)
{
  NodeHandle handle = allocate();
  Node &node = nodes_.get(handle);
  BLI_assert(node.type == Node::Type::UNUSED);

  node.type = Node::Type::COPY_BUFFER;
  node.copy_buffer.src_buffer = src_buffer;
  node.copy_buffer.dst_buffer = dst_buffer;
  node.copy_buffer.region = region;

  return handle;
}

void VKRenderGraphNodes::add_read_resource(NodeHandle handle, VersionedResource resource_handle)
{
  read_resources_per_node_[handle].append(resource_handle);
}

void VKRenderGraphNodes::add_write_resource(NodeHandle handle, VersionedResource resource_handle)
{
  write_resources_per_node_[handle].append(resource_handle);
}

void VKRenderGraphNodes::remove_nodes(Span<NodeHandle> node_handles)
{
  for (NodeHandle node_handle : node_handles) {
    read_resources_per_node_[node_handle].clear();
    write_resources_per_node_[node_handle].clear();
    mark_unused(get(node_handle));
    nodes_.free(node_handle);
  }
}

void VKRenderGraphNodes::mark_unused(Node &node)
{
  BLI_assert(node.type != Node::Type::UNUSED);
  memset(&node, 0, sizeof(Node));
  node.type = Node::Type::UNUSED;
}

NodeHandle VKRenderGraphNodes::allocate()
{
  NodeHandle node_handle = nodes_.allocate();
  ensure_vector_sizes();
  return node_handle;
}

void VKRenderGraphNodes::ensure_vector_sizes()
{
  if (read_resources_per_node_.size() < nodes_.size()) {
    read_resources_per_node_.resize(nodes_.size());
  }
  if (write_resources_per_node_.size() < nodes_.size()) {
    write_resources_per_node_.resize(nodes_.size());
  }
}

}  // namespace blender::gpu
