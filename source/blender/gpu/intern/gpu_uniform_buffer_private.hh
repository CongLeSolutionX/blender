/* SPDX-FileCopyrightText: 2020 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_assert.h"
#include "BLI_sys_types.h"
#include <iostream>

struct GPUUniformBuf;

namespace blender::gpu {

#ifndef NDEBUG
#  define DEBUG_NAME_LEN 64
#else
#  define DEBUG_NAME_LEN 8
#endif

/**
 * Implementation of Uniform Buffers.
 * Base class which is then specialized for each implementation (GL, VK, ...).
 */
class UniformBuf {
 protected:
  /** Data size in bytes. */
  size_t size_in_bytes_;
  /** Continuous memory block to copy to GPU. This data is owned by the UniformBuf. */
  void *data_ = nullptr;
  bool data_initialized_ = false;
  /** Debugging name */
  char name_[DEBUG_NAME_LEN];

 public:
  UniformBuf(size_t size, const char *name);
  virtual ~UniformBuf();

  /* Used after filling uninitialized buffers with poison values. */
  void force_data_uninitialized()
  {
    data_initialized_ = false;
  }

  virtual void update(const void * /*data*/)
  {
    data_initialized_ = true;
  };

  virtual void clear_to_zero()
  {
    data_initialized_ = true;
  };

  virtual void bind(int slot)
  {
    if (!data_initialized_) {
      std::cerr << "Binding uninitialized UBO: " << name_ << " at slot " << slot << std::endl;
    }
  }

  virtual void bind_as_ssbo(int slot)
  {
    if (!data_initialized_) {
      std::cerr << "Binding uninitialized UBO as SSBO: " << name_ << " at slot " << slot
                << std::endl;
    }
  }

  virtual void unbind() = 0;

  /** Used to defer data upload at drawing time.
   * This is useful if the thread has no context bound.
   * This transfers ownership to this UniformBuf. */
  void attach_data(void *data)
  {
    data_ = data;
    data_initialized_ = true;
  }
};

/* Syntactic sugar. */
static inline GPUUniformBuf *wrap(UniformBuf *vert)
{
  return reinterpret_cast<GPUUniformBuf *>(vert);
}
static inline UniformBuf *unwrap(GPUUniformBuf *vert)
{
  return reinterpret_cast<UniformBuf *>(vert);
}
static inline const UniformBuf *unwrap(const GPUUniformBuf *vert)
{
  return reinterpret_cast<const UniformBuf *>(vert);
}

#undef DEBUG_NAME_LEN

}  // namespace blender::gpu
