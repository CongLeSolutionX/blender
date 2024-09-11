/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "BKE_paint.hh"
#include "DEG_depsgraph_query.hh"

#include "draw_cache_impl.hh"

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class Paints {

 private:
  /* Draw selection state on top of the mesh to communicate which areas can be painted on. */
  PassSimple paint_region_ps_ = {"paint_region_ps_"};
  PassSimple::Sub *paint_region_edge_ps_ = nullptr;
  PassSimple::Sub *paint_region_face_ps_ = nullptr;
  PassSimple::Sub *paint_region_vert_ps_ = nullptr;

  PassSimple weight_ps_ = {"weight_ps_"};
  PassSimple vertex_paint_ps_ = {"vertex_paint_ps_"};
  PassSimple texture_paint_ps_ = {"texture_paint_ps_"};

  bool show_weight_ = false;
  bool show_wires_ = false;

  bool enabled_ = false;

 public:
  void begin_sync(Resources &res, const State &state)
  {
    enabled_ =
        (state.space_type == SPACE_VIEW3D) && (res.selection_type == SelectionType::DISABLED) &&
        ELEM(state.ctx_mode, CTX_MODE_PAINT_WEIGHT, CTX_MODE_PAINT_VERTEX, CTX_MODE_PAINT_TEXTURE);

    /* Init in any case to release the data. */
    paint_region_ps_.init();
    weight_ps_.init();
    vertex_paint_ps_.init();
    texture_paint_ps_.init();

    if (!enabled_) {
      return;
    }

    show_weight_ = state.ctx_mode == CTX_MODE_PAINT_WEIGHT;
    show_wires_ = state.overlay.paint_flag & V3D_OVERLAY_PAINT_WIRE;

    {
      auto &pass = paint_region_ps_;
      {
        auto &sub = pass.sub("Face");
        sub.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL |
                          DRW_STATE_BLEND_ALPHA,
                      state.clipping_plane_count);
        sub.shader_set(res.shaders.paint_region_face.get());
        sub.bind_ubo("globalsBlock", &res.globals_buf);
        sub.push_constant("ucolor", float4(1.0, 1.0, 1.0, 0.2));
        paint_region_face_ps_ = &sub;
      }
      {
        auto &sub = pass.sub("Edge");
        sub.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL |
                          DRW_STATE_BLEND_ALPHA,
                      state.clipping_plane_count);
        sub.shader_set(res.shaders.paint_region_edge.get());
        sub.bind_ubo("globalsBlock", &res.globals_buf);
        paint_region_edge_ps_ = &sub;
      }
      {
        auto &sub = pass.sub("Vert");
        sub.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL,
                      state.clipping_plane_count);
        sub.shader_set(res.shaders.paint_region_vert.get());
        sub.bind_ubo("globalsBlock", &res.globals_buf);
        paint_region_vert_ps_ = &sub;
      }
    }

    if (state.ctx_mode == CTX_MODE_PAINT_WEIGHT) {
      /* Support masked transparency in Workbench.
       * EEVEE can't be supported since depth won't match. */
      const eDrawType shading_type = eDrawType(state.v3d->shading.type);
      const bool masked_transparency_support = (shading_type <= OB_SOLID) ||
                                               BKE_scene_uses_blender_workbench(state.scene);
      const bool shadeless = shading_type == OB_WIRE;
      const bool draw_contours = state.overlay.wpaint_flag & V3D_OVERLAY_WPAINT_CONTOURS;

      auto &pass = weight_ps_;
      pass.state_set(DRW_STATE_WRITE_COLOR |
                         (masked_transparency_support ?
                              (DRW_STATE_DEPTH_EQUAL | DRW_STATE_BLEND_ALPHA) :
                              (DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_WRITE_DEPTH)),
                     state.clipping_plane_count);
      pass.shader_set(shadeless ? res.shaders.paint_weight.get() :
                                  res.shaders.paint_weight_fake_shading.get());
      pass.bind_ubo("globalsBlock", &res.globals_buf);
      pass.bind_texture("colorramp", &res.weight_ramp_tx);
      pass.push_constant("drawContours", draw_contours);
      pass.push_constant("opacity", state.overlay.weight_paint_mode_opacity);
      if (!shadeless) {
        /* Arbitrary light to give a hint of the geometry behind the weights. */
        pass.push_constant("light_dir", math::normalize(float3(0.0f, 0.5f, 0.86602f)));
      }
    }
  }

  void object_sync(Manager &manager, const ObjectRef &ob_ref, const State &state)
  {
    if (!enabled_) {
      return;
    }

    if (ob_ref.object->type != OB_MESH) {
      /* Only meshes are supported for now. */
      return;
    }

    switch (state.ctx_mode) {
      case CTX_MODE_PAINT_WEIGHT:
        if (ob_ref.object->mode != OB_MODE_WEIGHT_PAINT) {
          /* Not matching context mode. */
          return;
        }
        break;
      case CTX_MODE_PAINT_VERTEX:
        if (ob_ref.object->mode != OB_MODE_VERTEX_PAINT) {
          /* Not matching context mode. */
          return;
        }
        break;
      case CTX_MODE_PAINT_TEXTURE:
        if (ob_ref.object->mode != OB_MODE_TEXTURE_PAINT) {
          /* Not matching context mode. */
          return;
        }
        break;
      default:
        /* Not in paint mode. */
        return;
    }

    ResourceHandle handle = manager.resource_handle(ob_ref);

    switch (state.ctx_mode) {
      case CTX_MODE_PAINT_WEIGHT: {
        gpu::Batch *geom = DRW_cache_mesh_surface_weights_get(ob_ref.object);
        weight_ps_.draw(geom, handle);
        break;
      }
      case CTX_MODE_PAINT_VERTEX:
        break;
      case CTX_MODE_PAINT_TEXTURE:
        break;
      default:
        BLI_assert_unreachable();
        return;
    }

    /* Selection Display. */
    {
      /* NOTE(fclem): Why do we need original mesh here, only to get the flag? */
      const Mesh &mesh_orig = *static_cast<Mesh *>(DEG_get_original_object(ob_ref.object)->data);
      const bool use_face_selection = (mesh_orig.editflag & ME_EDIT_PAINT_FACE_SEL);
      const bool use_vert_selection = (mesh_orig.editflag & ME_EDIT_PAINT_VERT_SEL);

      if (use_face_selection || show_wires_) {
        gpu::Batch *geom = DRW_cache_mesh_surface_edges_get(ob_ref.object);
        paint_region_edge_ps_->push_constant("useSelect", use_face_selection);
        paint_region_edge_ps_->draw(geom, handle);
      }
      if (use_face_selection) {
        gpu::Batch *geom = DRW_cache_mesh_surface_get(ob_ref.object);
        paint_region_face_ps_->draw(geom, handle);
      }
      if (use_vert_selection) {
        gpu::Batch *geom = DRW_cache_mesh_all_verts_get(ob_ref.object);
        paint_region_vert_ps_->draw(geom, handle);
      }
    }
  }

  void draw(GPUFrameBuffer *framebuffer, Manager &manager, View &view)
  {
    if (!enabled_) {
      return;
    }
    GPU_framebuffer_bind(framebuffer);
    manager.submit(weight_ps_, view);
    manager.submit(vertex_paint_ps_, view);
    manager.submit(texture_paint_ps_, view);
    /* TODO(fclem): Draw this onto the line frame-buffer to get wide-line and anti-aliasing.
     * Just need to make sure the shaders output line data. */
    manager.submit(paint_region_ps_, view);
  }
};

}  // namespace blender::draw::overlay
