/* SPDX-FileCopyrightText: 2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BKE_global.hh"
#include "BLI_sys_types.h"
#include <iostream>

struct GPUStorageBuf;

namespace blender::gpu {

class VertBuf;

#ifndef NDEBUG
#  define DEBUG_NAME_LEN 64
#else
#  define DEBUG_NAME_LEN 8
#endif

/**
 * Implementation of Storage Buffers.
 * Base class which is then specialized for each implementation (GL, VK, ...).
 */
class StorageBuf {
 private:
  bool data_initialized_ = false;

 protected:
  /** Data size in bytes. */
  size_t size_in_bytes_;
  /** Continuous memory block to copy to GPU. This data is owned by the StorageBuf. */
  void *data_ = nullptr;
  /** Debugging name */
  char name_[DEBUG_NAME_LEN];

 public:
  StorageBuf(size_t size, const char *name);
  virtual ~StorageBuf();

  virtual void update(const void *data) = 0;
  virtual void bind(int slot) = 0;
  virtual void unbind() = 0;
  virtual void clear(uint32_t clear_value) = 0;
  virtual void copy_sub(VertBuf *src, uint dst_offset, uint src_offset, uint copy_size) = 0;
  virtual void read(void *data) = 0;
  virtual void async_flush_to_host() = 0;
  virtual void sync_as_indirect_buffer() = 0;

  /* Used for device_only buffers. */
  void force_data_initialized()
  {
    data_initialized_ = true;
  }

  /* Fill the buffer with poison values for debugging purposes.
   * (NaN for floats, -1 for `int` and "max value" for `uint`). */
  void poison_fill()
  {
    blender::Vector<uchar> uninitialized_data(size_in_bytes_, 0xFF);
    update(uninitialized_data.data());
    /* `update` tags the buffer data as initialized so we revert that. */
    data_initialized_ = false;
  }

 protected:
  void set_data_initialized()
  {
    data_initialized_ = true;
  }

  void check_data_initialization_on_bind(int slot)
  {
    if (!data_initialized_ && (G.debug & G_DEBUG_GPU)) {
      std::cerr << "Binding uninitialized SSBO: " << name_ << " at slot " << slot << std::endl;
    }
  }
};

/* Syntactic sugar. */
static inline GPUStorageBuf *wrap(StorageBuf *storage_buf)
{
  return reinterpret_cast<GPUStorageBuf *>(storage_buf);
}
static inline StorageBuf *unwrap(GPUStorageBuf *storage_buf)
{
  return reinterpret_cast<StorageBuf *>(storage_buf);
}
static inline const StorageBuf *unwrap(const GPUStorageBuf *storage_buf)
{
  return reinterpret_cast<const StorageBuf *>(storage_buf);
}

#undef DEBUG_NAME_LEN

}  // namespace blender::gpu
