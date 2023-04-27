/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/base/plug/plugin.h>
#include <pxr/base/plug/registry.h>
#include <pxr/imaging/hd/rendererPluginRegistry.h>
#include <pxr/imaging/hdSt/renderDelegate.h>
#include <pxr/imaging/hgi/tokens.h>
#include <pxr/usd/usdGeom/tokens.h>

#include "BLI_path_util.h"
#include "GPU_context.h"

#include "engine.h"

namespace blender::render::hydra {

CLG_LOGREF_DECLARE_GLOBAL(LOG_RENDER_HYDRA, "render.hydra");

Engine::Engine(RenderEngine *bl_engine, const std::string &render_delegate_name)
    : bl_engine_(bl_engine), render_delegate_name_(render_delegate_name)
{
  pxr::HdRendererPluginRegistry &registry = pxr::HdRendererPluginRegistry::GetInstance();

  pxr::TF_PY_ALLOW_THREADS_IN_SCOPE();
  render_delegate_ = registry.CreateRenderDelegate(pxr::TfToken(render_delegate_name_));

  /* Current USD (23.02) has limited support for Vulkan. To make it works USD should be built
   * with PXR_ENABLE_VULKAN_SUPPORT=TRUE which is not possible now */
  if (GPU_backend_get_type() == GPU_BACKEND_VULKAN) {
    BLI_setenv("HGI_ENABLE_VULKAN", "1");
  }

  pxr::HdDriverVector hd_drivers;
  if (bl_engine->type->flag & RE_USE_GPU_CONTEXT) {
    hgi_ = pxr::Hgi::CreatePlatformDefaultHgi();
    hgi_driver_.name = pxr::HgiTokens->renderDriver;
    hgi_driver_.driver = pxr::VtValue(hgi_.get());

    hd_drivers.push_back(&hgi_driver_);
  }

  render_index_.reset(pxr::HdRenderIndex::New(render_delegate_.Get(), hd_drivers));
  free_camera_delegate_ = std::make_unique<pxr::HdxFreeCameraSceneDelegate>(
      render_index_.get(), pxr::SdfPath::AbsoluteRootPath().AppendElementString("freeCamera"));
  render_task_delegate_ = std::make_unique<RenderTaskDelegate>(
      render_index_.get(), pxr::SdfPath::AbsoluteRootPath().AppendElementString("renderTask"));
  if (render_delegate_name_ == "HdStormRendererPlugin") {
    simple_light_task_delegate_ = std::make_unique<SimpleLightTaskDelegate>(
        render_index_.get(),
        pxr::SdfPath::AbsoluteRootPath().AppendElementString("simpleLightTask"));
  }

  engine_ = std::make_unique<pxr::HdEngine>();
}

float Engine::renderer_percent_done()
{
  pxr::VtDictionary render_stats = render_delegate_->GetRenderStats();
  auto it = render_stats.find("percentDone");
  if (it == render_stats.end()) {
    return 0.0;
  }
  return (float)it->second.UncheckedGet<double>();
}

}  // namespace blender::render::hydra
