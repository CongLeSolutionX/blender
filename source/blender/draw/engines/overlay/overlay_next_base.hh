/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class Overlay {
  virtual void begin_sync(Resources &res, const State &state, const View &view) = 0;

  virtual void object_sync(Manager &manager,
                           const ObjectRef &ob_ref,
                           const State &state,
                           Resources &res) = 0;

  virtual void edit_object_sync(Manager &manager,
                                const ObjectRef &ob_ref,
                                const State &state,
                                Resources &res) = 0;

  virtual void pre_draw(Framebuffer &framebuffer, Manager &manager, View &view) = 0;
  virtual void draw(Framebuffer &framebuffer, Manager &manager, View &view) = 0;

  virtual void draw_color_only(Framebuffer &framebuffer, Manager &manager, View &view) = 0;
  virtual void draw_on_render(GPUFrameBuffer *framebuffer, Manager &manager, View &view) = 0;
};

}  // namespace blender::draw::overlay
