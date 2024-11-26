/* SPDX-FileCopyrightText: 2020 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BKE_global.hh"
#include "BLI_sys_types.h"
#include "BLI_vector.hh"
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
 private:
  bool data_initialized_ = false;

 protected:
  /** Data size in bytes. */
  size_t size_in_bytes_;
  /** Continuous memory block to copy to GPU. This data is owned by the UniformBuf. */
  void *data_ = nullptr;
  /** Debugging name */
  char name_[DEBUG_NAME_LEN];

 public:
  UniformBuf(size_t size, const char *name);
  virtual ~UniformBuf();

  virtual void update(const void *data) = 0;

  virtual void clear_to_zero() = 0;

  virtual void bind(int slot) = 0;

  virtual void bind_as_ssbo(int slot) = 0;

  virtual void unbind() = 0;

  /** Used to defer data upload at drawing time.
   * This is useful if the thread has no context bound.
   * This transfers ownership to this UniformBuf. */
  void attach_data(void *data)
  {
    data_ = data;
    set_data_initialized();
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

  void check_data_initialization_on_bind(int slot, bool as_SSBO)
  {
    if (!data_initialized_ && (G.debug & G_DEBUG_GPU)) {
      std::cerr << "Binding uninitialized UBO" << (as_SSBO ? " as SSBO" : "") << ": " << name_
                << " at slot " << slot << std::endl;
    }
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
