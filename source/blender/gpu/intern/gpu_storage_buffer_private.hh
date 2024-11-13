/* SPDX-FileCopyrightText: 2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_span.hh"
#include "BLI_sys_types.h"

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
 protected:
  /** Data size in bytes. */
  size_t size_in_bytes_;
  /** Continuous memory block to copy to GPU. This data is owned by the StorageBuf. */
  void *data_ = nullptr;
  bool data_initialized_ = false;
  /** Debugging name */
  char name_[DEBUG_NAME_LEN];

 public:
  StorageBuf(size_t size, const char *name);
  virtual ~StorageBuf();

  /* Used for device_only buffers. */
  void force_data_initialized()
  {
    data_initialized_ = true;
  }

  virtual void update(const void * /*data*/)
  {
    data_initialized_ = true;
  }

  virtual void bind(int /*slot*/)
  {
    BLI_assert(data_initialized_);
  }

  virtual void unbind() = 0;

  virtual void clear(uint32_t /*clear_value*/)
  {
    data_initialized_ = true;
  }

  virtual void copy_sub(VertBuf * /*src*/,
                        uint /*dst_offset*/,
                        uint /*src_offset*/,
                        uint copy_size)
  {
    if (copy_size == size_in_bytes_) {
      data_initialized_ = true;
    }
  }

  virtual void read(void *data) = 0;
  virtual void async_flush_to_host() = 0;
  virtual void sync_as_indirect_buffer() = 0;
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
