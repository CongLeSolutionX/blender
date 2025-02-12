/* SPDX-FileCopyrightText: 2016 by Mike Erwin. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 *
 * Mimics old style opengl immediate mode drawing.
 */

#include "GPU_capabilities.hh"

#include "gpu_context_private.hh"
#include "gpu_shader_private.hh"
#include "gpu_vertex_format_private.hh"

#include "gl_context.hh"
#include "gl_debug.hh"
#include "gl_primitive.hh"
#include "gl_vertex_array.hh"

#include "gl_immediate.hh"

namespace blender::gpu {

/* -------------------------------------------------------------------- */
/** \name Creation & Deletion
 * \{ */

GLImmediate::GLImmediate()
{
  glGenVertexArrays(1, &vao_id_);
  glBindVertexArray(vao_id_); /* Necessary for glObjectLabel. */

  buffer.buffer_size = DEFAULT_INTERNAL_BUFFER_SIZE;
  glGenBuffers(1, &buffer.vbo_id);
  glBindBuffer(GL_ARRAY_BUFFER, buffer.vbo_id);
  glBufferData(GL_ARRAY_BUFFER, buffer.buffer_size, nullptr, GL_DYNAMIC_DRAW);

  buffer_strict.buffer_size = DEFAULT_INTERNAL_BUFFER_SIZE;
  glGenBuffers(1, &buffer_strict.vbo_id);
  glBindBuffer(GL_ARRAY_BUFFER, buffer_strict.vbo_id);
  glBufferData(GL_ARRAY_BUFFER, buffer_strict.buffer_size, nullptr, GL_DYNAMIC_DRAW);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

  debug::object_label(GL_VERTEX_ARRAY, vao_id_, "Immediate");
  debug::object_label(GL_BUFFER, buffer.vbo_id, "ImmediateVbo");
  debug::object_label(GL_BUFFER, buffer_strict.vbo_id, "ImmediateVboStrict");
}

GLImmediate::~GLImmediate()
{
  glDeleteVertexArrays(1, &vao_id_);

  glDeleteBuffers(1, &buffer.vbo_id);
  glDeleteBuffers(1, &buffer_strict.vbo_id);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Buffer management
 * \{ */

uchar *GLImmediate::begin()
{
  /* How many bytes do we need for this draw call? */
  const size_t bytes_needed = vertex_buffer_size(&vertex_format, vertex_len);
  /* Does the current buffer have enough room? */
  const size_t available_bytes = buffer_size() - buffer_offset();

#ifndef NDEBUG
  if (unwrap(this->shader)->is_polyline) {
    /* Silence error. These are bound inside `immEnd()`. */
    GLContext::get()->bound_ssbo_slots |= 1 << GPU_SSBO_POLYLINE_POS_BUF_SLOT;
    GLContext::get()->bound_ssbo_slots |= 1 << GPU_SSBO_POLYLINE_COL_BUF_SLOT;
    GLContext::get()->bound_ssbo_slots |= 1 << GPU_SSBO_INDEX_BUF_SLOT;
  }
#endif

  GL_CHECK_RESOURCES("Immediate");

  glBindBuffer(GL_ARRAY_BUFFER, vbo_id());

  bool recreate_buffer = false;
  if (bytes_needed > buffer_size()) {
    /* expand the internal buffer */
    buffer_size() = bytes_needed;
    recreate_buffer = true;
  }
  else if (bytes_needed < DEFAULT_INTERNAL_BUFFER_SIZE &&
           buffer_size() > DEFAULT_INTERNAL_BUFFER_SIZE)
  {
    /* shrink the internal buffer */
    buffer_size() = DEFAULT_INTERNAL_BUFFER_SIZE;
    recreate_buffer = true;
  }

  uint vert_alignment = vertex_format.stride;
  if (unwrap(this->shader)->is_polyline) {
    /* Polyline needs to bind the buffer as SSBO.
     * The start of the range needs to match the SSBO alignment requirements. */
    vert_alignment = max_uu(vert_alignment, GPU_storage_buffer_alignment());
  }
  /* Ensure vertex data is aligned. Might waste a little space, but it's safe. */
  const uint pre_padding = padding(buffer_offset(), vert_alignment);

  if (!recreate_buffer && ((bytes_needed + pre_padding) <= available_bytes)) {
    buffer_offset() += pre_padding;
  }
  else {
    /* orphan this buffer & start with a fresh one */
    glBufferData(GL_ARRAY_BUFFER, buffer_size(), nullptr, GL_DYNAMIC_DRAW);
    buffer_offset() = 0;
  }

#ifndef NDEBUG
  {
    GLint bufsize;
    glGetBufferParameteriv(GL_ARRAY_BUFFER, GL_BUFFER_SIZE, &bufsize);
    BLI_assert(buffer_offset() + bytes_needed <= bufsize);
  }
#endif

  GLbitfield access = GL_MAP_WRITE_BIT | GL_MAP_UNSYNCHRONIZED_BIT;
  if (!strict_vertex_len) {
    access |= GL_MAP_FLUSH_EXPLICIT_BIT;
  }
  void *data = glMapBufferRange(GL_ARRAY_BUFFER, buffer_offset(), bytes_needed, access);
  BLI_assert(data != nullptr);

  bytes_mapped_ = bytes_needed;
  return (uchar *)data;
}

void GLImmediate::end()
{
  BLI_assert(prim_type != GPU_PRIM_NONE); /* make sure we're between a Begin/End pair */

  uint buffer_bytes_used = bytes_mapped_;
  if (!strict_vertex_len) {
    if (vertex_idx != vertex_len) {
      vertex_len = vertex_idx;
      buffer_bytes_used = vertex_buffer_size(&vertex_format, vertex_len);
      /* unused buffer bytes are available to the next immBegin */
    }
    /* tell OpenGL what range was modified so it doesn't copy the whole mapped range */
    glFlushMappedBufferRange(GL_ARRAY_BUFFER, 0, buffer_bytes_used);
  }
  glUnmapBuffer(GL_ARRAY_BUFFER);

  if (vertex_len == 0) {
    /* Noop. Nothing to draw. */
  }
  else if (unwrap(this->shader)->is_polyline) {
    GLintptr offset = buffer_offset();
    GLenum target = GL_SHADER_STORAGE_BUFFER;
    glBindBufferRange(target, GPU_SSBO_POLYLINE_POS_BUF_SLOT, vbo_id(), offset, buffer_bytes_used);
    glBindBufferRange(target, GPU_SSBO_POLYLINE_COL_BUF_SLOT, vbo_id(), offset, buffer_bytes_used);
    /* Not used. Satisfy the binding. */
    glBindBufferRange(target, GPU_SSBO_INDEX_BUF_SLOT, vbo_id(), offset, buffer_bytes_used);

    this->polyline_draw_workaround(0);

#ifndef NDEBUG
    GLContext::get()->bound_ssbo_slots &= ~(1 << GPU_SSBO_POLYLINE_POS_BUF_SLOT);
    GLContext::get()->bound_ssbo_slots &= ~(1 << GPU_SSBO_POLYLINE_COL_BUF_SLOT);
    GLContext::get()->bound_ssbo_slots &= ~(1 << GPU_SSBO_INDEX_BUF_SLOT);
#endif
  }
  else {
    GLContext::get()->state_manager->apply_state();

    /* We convert the offset in vertex offset from the buffer's start.
     * This works because we added some padding to align the first vertex. */
    uint v_first = buffer_offset() / vertex_format.stride;
    GLVertArray::update_bindings(
        vao_id_, v_first, &vertex_format, reinterpret_cast<Shader *>(shader)->interface);

    /* Update matrices. */
    GPU_shader_bind(shader);

    glDrawArrays(to_gl(prim_type), 0, vertex_len);

    /* These lines are causing crash on startup on some old GPU + drivers.
     * They are not required so just comment them. (#55722) */
    // glBindBuffer(GL_ARRAY_BUFFER, 0);
    // glBindVertexArray(0);
  }

  buffer_offset() += buffer_bytes_used;
}

/** \} */

}  // namespace blender::gpu
