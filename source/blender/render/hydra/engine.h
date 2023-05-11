/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <chrono>

#include <pxr/imaging/hd/driver.h>
#include <pxr/imaging/hd/engine.h>
#include <pxr/imaging/hd/pluginRenderDelegateUniqueHandle.h>
#include <pxr/imaging/hdx/freeCameraSceneDelegate.h>
#include <pxr/imaging/hgi/hgi.h>

#include "RE_engine.h"

#include "CLG_log.h"

#include "render_task_delegate.h"
#include "scene_delegate/blender_scene_delegate.h"
#include "simple_light_task_delegate.h"

namespace blender::render::hydra {

extern struct CLG_LogRef *LOG_RENDER_HYDRA;

class Engine {
 public:
  Engine(RenderEngine *bl_engine, const std::string &render_delegate_name);
  virtual ~Engine() = default;

  virtual void sync(Depsgraph *depsgraph,
                    bContext *context,
                    pxr::HdRenderSettingsMap &render_settings) = 0;
  virtual void render(Depsgraph *depsgraph) = 0;

 protected:
  float renderer_percent_done();

  RenderEngine *bl_engine_;
  std::string render_delegate_name_;

  /* The order is important due to deletion order */
  pxr::HgiUniquePtr hgi_;
  pxr::HdDriver hgi_driver_;
  pxr::HdPluginRenderDelegateUniqueHandle render_delegate_;
  std::unique_ptr<pxr::HdRenderIndex> render_index_;
  std::unique_ptr<BlenderSceneDelegate> scene_delegate_;
  std::unique_ptr<RenderTaskDelegate> render_task_delegate_;
  std::unique_ptr<pxr::HdxFreeCameraSceneDelegate> free_camera_delegate_;
  std::unique_ptr<SimpleLightTaskDelegate> simple_light_task_delegate_;
  std::unique_ptr<pxr::HdEngine> engine_;
};

}  // namespace blender::render::hydra
