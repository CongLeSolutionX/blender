/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/imaging/glf/drawTarget.h>

#include "BKE_lib_id.h"
#include "DEG_depsgraph_query.h"

#include "glog/logging.h"

#include "camera.h"
#include "final_engine.h"
#include "utils.h"

namespace blender::render::hydra {

void FinalEngine::sync(Depsgraph *depsgraph,
                       bContext *context,
                       pxr::HdRenderSettingsMap &render_settings)
{
  scene_delegate = std::make_unique<BlenderSceneDelegate>(
      render_index.get(),
      pxr::SdfPath::AbsoluteRootPath().AppendElementString("scene"),
      BlenderSceneDelegate::EngineType::FINAL);
  scene_delegate->populate(depsgraph, context);

  for (auto const &setting : render_settings) {
    render_delegate->SetRenderSetting(setting.first, setting.second);
  }
}

void FinalEngine::render(Depsgraph *depsgraph)
{
  Scene *scene = DEG_get_input_scene(depsgraph);
  ViewLayer *view_layer = DEG_get_input_view_layer(depsgraph);

  std::string scene_name(MAX_ID_FULL_NAME, 0);
  BKE_id_full_name_get(scene_name.data(), (ID *)scene, 0);
  std::string layer_name = view_layer->name;

  RenderData &r = scene->r;
  pxr::GfVec4f border(0, 0, 1, 1);
  if (r.mode & R_BORDER) {
    border = pxr::GfVec4f(r.border.xmin,
                          r.border.ymin,
                          r.border.xmax - r.border.xmin,
                          r.border.ymax - r.border.ymin);
  }
  pxr::GfVec2i image_res(r.xsch * r.size / 100, r.ysch * r.size / 100);
  pxr::GfVec2i res(int(image_res[0] * border[2]), int(image_res[1] * border[3]));
  pxr::GfCamera camera =
      CameraData(scene->camera, image_res, pxr::GfVec4f(0, 0, 1, 1)).gf_camera(border);

  free_camera_delegate->SetCamera(camera);
  render_task_delegate->set_camera_and_viewport(free_camera_delegate->GetCameraId(),
                                                pxr::GfVec4d(0, 0, res[0], res[1]));
  render_task_delegate->set_renderer_aov(pxr::HdAovTokens->color);
  if (simple_light_task_delegate) {
    simple_light_task_delegate->set_camera_path(free_camera_delegate->GetCameraId());
  }

  pxr::HdTaskSharedPtrVector tasks;
  if (simple_light_task_delegate) {
    tasks.push_back(simple_light_task_delegate->get_task());
  }
  tasks.push_back(render_task_delegate->get_task());

  std::chrono::time_point<std::chrono::steady_clock> time_begin = std::chrono::steady_clock::now(),
                                                     time_current;
  std::chrono::milliseconds elapsed_time;

  float percent_done = 0.0;

  std::map<std::string, std::vector<float>> render_images{
      {"Combined", std::vector<float>(res[0] * res[1] * 4)}};  // 4 - number of channels
  std::vector<float> &pixels = render_images["Combined"];

  {
    // Release the GIL before calling into hydra, in case any hydra plugins call into python.
    pxr::TF_PY_ALLOW_THREADS_IN_SCOPE();
    engine->Execute(render_index.get(), &tasks);
  }

  while (true) {
    if (RE_engine_test_break(bl_engine)) {
      break;
    }

    percent_done = renderer_percent_done();
    time_current = std::chrono::steady_clock::now();
    elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(time_current -
                                                                         time_begin);

    notify_status(percent_done / 100.0,
                  scene_name + ": " + layer_name,
                  "Render Time: " + format_duration(elapsed_time) +
                      " | Done: " + std::to_string(int(percent_done)) + "%");

    if (render_task_delegate->is_converged()) {
      break;
    }

    render_task_delegate->get_renderer_aov_data(pxr::HdAovTokens->color, pixels.data());
    update_render_result(render_images, layer_name, res[0], res[1]);
  }

  render_task_delegate->get_renderer_aov_data(pxr::HdAovTokens->color, pixels.data());
  update_render_result(render_images, layer_name, res[0], res[1]);
}

pxr::GfVec2i FinalEngine::get_resolution(Scene *scene)
{
  RenderData &r = scene->r;
  float border_w = 1.0, border_h = 1.0;
  if (r.mode & R_BORDER) {
    border_w = r.border.xmax - r.border.xmin;
    border_h = r.border.ymax - r.border.ymin;
  }
  return pxr::GfVec2i(int(r.xsch * border_w * r.size / 100),
                      int(r.ysch * border_h * r.size / 100));
}

void FinalEngine::update_render_result(std::map<std::string, std::vector<float>> &render_images,
                                       const std::string &layer_name,
                                       int width,
                                       int height)
{
  RenderResult *result = RE_engine_begin_result(
      bl_engine, 0, 0, width, height, layer_name.c_str(), nullptr);

  /* TODO: only for the first render layer */
  RenderLayer *layer = (RenderLayer *)result->layers.first;
  for (RenderPass *pass = (RenderPass *)layer->passes.first; pass != nullptr; pass = pass->next) {
    auto it_image = render_images.find(pass->name);
    if (it_image == render_images.end()) {
      continue;
    }
    memcpy(pass->rect,
           it_image->second.data(),
           sizeof(float) * pass->rectx * pass->recty * pass->channels);
  }

  RE_engine_end_result(bl_engine, result, false, false, false);
}

void FinalEngine::notify_status(float progress, const std::string &title, const std::string &info)
{
  RE_engine_update_progress(bl_engine, progress);
  RE_engine_update_stats(bl_engine, title.c_str(), info.c_str());
}

void FinalEngineGL::render(Depsgraph *depsgraph)
{
  Scene *scene = DEG_get_input_scene(depsgraph);
  ViewLayer *view_layer = DEG_get_input_view_layer(depsgraph);

  std::string scene_name(MAX_ID_FULL_NAME, 0);
  BKE_id_full_name_get(scene_name.data(), (ID *)scene, 0);
  std::string layer_name = view_layer->name;

  RenderData &r = scene->r;
  pxr::GfVec4f border(0, 0, 1, 1);
  if (r.mode & R_BORDER) {
    border = pxr::GfVec4f(r.border.xmin,
                          r.border.ymin,
                          r.border.xmax - r.border.xmin,
                          r.border.ymax - r.border.ymin);
  }
  pxr::GfVec2i image_res = {r.xsch * r.size / 100, r.ysch * r.size / 100};
  pxr::GfVec2i res = {int(image_res[0] * border[2]), int(image_res[1] * border[3])};
  pxr::GfCamera camera =
      CameraData(scene->camera, image_res, pxr::GfVec4f(0, 0, 1, 1)).gf_camera(border);

  free_camera_delegate->SetCamera(camera);
  render_task_delegate->set_camera_and_viewport(free_camera_delegate->GetCameraId(),
                                                pxr::GfVec4d(0, 0, res[0], res[1]));
  if (simple_light_task_delegate) {
    simple_light_task_delegate->set_camera_path(free_camera_delegate->GetCameraId());
  }

  pxr::HdTaskSharedPtrVector tasks;
  if (simple_light_task_delegate) {
    /* TODO: Uncomment this and fix GL error:
         invalid operation, reported from void __cdecl
       pxrInternal_v0_22__pxrReserved__::HgiGLResourceBindings::BindResources(void) */
    // tasks.push_back(simple_light_task_delegate->get_task());
  }
  tasks.push_back(render_task_delegate->get_task());

  std::chrono::time_point<std::chrono::steady_clock> time_begin = std::chrono::steady_clock::now(),
                                                     time_current;
  std::chrono::milliseconds elapsed_time;

  float percent_done = 0.0;

  std::map<std::string, std::vector<float>> render_images{
      {"Combined", std::vector<float>(res[0] * res[1] * 4)}};  // 4 - number of channels
  std::vector<float> &pixels = render_images["Combined"];

  GLuint framebuffer_name = 0;
  glGenFramebuffers(1, &framebuffer_name);
  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_name);

  // The texture we're going to render to
  GLuint rendered_texture;
  glGenTextures(1, &rendered_texture);

  // "Bind" the newly created texture : all future texture functions will modify this texture
  glBindTexture(GL_TEXTURE_2D, rendered_texture);

  // Give an empty image to OpenGL ( the last "0" )
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, res[0], res[1], 0, GL_RGBA, GL_FLOAT, 0);

  // Poor filtering. Needed !
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  // Set "rendered_texture" as our colour attachement #0
  glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, rendered_texture, 0);

  // Generate vertex array
  GLuint vao;
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  {
    // Release the GIL before calling into hydra, in case any hydra plugins call into python.
    pxr::TF_PY_ALLOW_THREADS_IN_SCOPE();
    engine->Execute(render_index.get(), &tasks);
  }

  while (true) {
    if (RE_engine_test_break(bl_engine)) {
      break;
    }

    percent_done = renderer_percent_done();
    time_current = std::chrono::steady_clock::now();
    elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(time_current -
                                                                         time_begin);

    notify_status(percent_done / 100.0,
                  scene_name + ": " + layer_name,
                  "Render Time: " + format_duration(elapsed_time) +
                      " | Done: " + std::to_string(int(percent_done)) + "%");

    if (render_task_delegate->is_converged()) {
      break;
    }

    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, pixels.data());
    update_render_result(render_images, layer_name, res[0], res[1]);
  }

  glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, pixels.data());
  update_render_result(render_images, layer_name, res[0], res[1]);
}

}  // namespace blender::render::hydra
