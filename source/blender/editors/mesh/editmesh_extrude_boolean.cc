/* SPDX-FileCopyrightText: 2004 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file \
 * \ingroup edmesh \
 */

#include "BLI_bounds.hh"
#include "BLI_kdopbvh.h"
#include "BLI_math_matrix.h"

#include "BKE_bvhutils.hh"
#include "BKE_context.hh"
#include "BKE_editmesh.hh"
#include "BKE_report.hh"

#include "ED_mesh.hh"
#include "ED_screen.hh"
#include "ED_space_api.hh"
#include "ED_transform.hh"
#include "ED_view3d.hh"

#include "GPU_batch.hh"
#include "GPU_matrix.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "UI_resources.hh"

#include "tools/bmesh_intersect.hh"
#include "tools/bmesh_intersect_edges.hh"

#include "mesh_intern.hh" /* Own include. */

using namespace blender;

#define RAYCAST_DEPTH_EPSILON (10 * FLT_EPSILON)
#define MESH_OPERAND_TAG BM_ELEM_TAG_ALT
#define MESH_OPERAND_TAG2 BM_ELEM_TAG

/* -------------------------------------------------------------------- */
/** \name Draw
 * \{ */

struct ExtrudeDrawData {
  void *draw_handle;
  ARegion *region;
  const float (*object_to_world)[4];
  float color[4];
  float point_size;

  gpu::IndexBuf *ibo_edges;
  gpu::IndexBuf *ibo_faces;
  gpu::Batch *batch_verts;
  gpu::Batch *batch_edges;
  gpu::Batch *batch_faces;
  gpu::VertBuf *vbo;
  MutableSpan<float3> v_co;
} draw_data;

static void extrude_boolean_drawdata_clear(ExtrudeDrawData &draw_data)
{
  ED_region_draw_cb_exit(draw_data.region->type, draw_data.draw_handle);

  GPU_batch_discard(draw_data.batch_edges);
  GPU_indexbuf_discard(draw_data.ibo_edges);
  GPU_BATCH_DISCARD_SAFE(draw_data.batch_verts);
  GPU_BATCH_DISCARD_SAFE(draw_data.batch_faces);
  GPU_INDEXBUF_DISCARD_SAFE(draw_data.ibo_faces);
  GPU_vertbuf_discard(draw_data.vbo);
}

static void extrude_boolean_draw_fn(const bContext * /*C*/, ARegion *region, void *data)
{
  ExtrudeDrawData *draw_data = static_cast<ExtrudeDrawData *>(data);
  RegionView3D *rv3d = static_cast<RegionView3D *>(region->regiondata);

  GPU_matrix_push();
  GPU_matrix_mul(draw_data->object_to_world);

  if (GPU_vertbuf_get_status(draw_data->vbo) & GPU_VERTBUF_DATA_DIRTY) {
    GPU_vertbuf_use(draw_data->vbo);
  }

  GPUShader *sh = draw_data->batch_edges->shader;
  GPU_shader_bind(sh);
  GPU_shader_uniform_4fv(sh, "color", draw_data->color);

  ED_view3d_polygon_offset(rv3d, 1.0f);
  GPU_depth_mask(false);

  if (draw_data->batch_faces) {
    GPU_blend(GPU_BLEND_ALPHA);
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_batch_draw(draw_data->batch_faces);
  }

  GPU_depth_test(GPU_DEPTH_NONE);
  GPU_blend(GPU_BLEND_NONE);
  GPU_batch_draw(draw_data->batch_edges);

  if (draw_data->batch_verts) {
    GPU_point_size(draw_data->point_size);
    GPU_batch_draw(draw_data->batch_verts);
  }

  ED_view3d_polygon_offset(rv3d, 0.0f);
  GPU_matrix_pop();

  GPU_depth_mask(true);
}

static void extrude_boolean_drawdata_create(ARegion *region,
                                            BMesh *bm,
                                            Span<BMVert *> moving_verts,
                                            Array<std::array<BMLoop *, 3>> corner_tris,
                                            const float (*object_to_world)[4],
                                            int edge_tag_len,
                                            int face_tag_len,
                                            bool draw_verts,
                                            ExtrudeDrawData *r_draw_data)
{
  static GPUVertFormat v_format = {0};
  static GPUVertFormat line_format = {0};
  if (v_format.attr_len == 0) {
    GPU_vertformat_attr_add(&v_format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
    GPU_vertformat_attr_add(&line_format, "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  }

  BMIter iter;
  BMVert *v;
  BMEdge *e;
  BMFace *f;
  int vert_len = moving_verts.size();
  int i;

  gpu::VertBuf *vbo = GPU_vertbuf_create_with_format_ex(v_format, GPU_USAGE_DYNAMIC);
  GPU_vertbuf_data_alloc(*vbo, vert_len);
  MutableSpan<float3> v_co = vbo->data<float3>();
  for (i = 0; i < vert_len; i++) {
    v = moving_verts[i];
    copy_v3_v3(v_co[i], v->co);
    /* Indexes used to identify the triangles in the drawing. */
    BM_elem_index_set(v, i);
  }

  bm->elem_index_dirty |= BM_VERT;

  gpu::IndexBuf *ibo_faces = nullptr;
  if (face_tag_len) {
    int looptris_draw_len = 0;
    BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
      if (BM_elem_flag_test(f, MESH_OPERAND_TAG | MESH_OPERAND_TAG2)) {
        looptris_draw_len += f->len - 2;
      }
    }

    GPUIndexBufBuilder builder;
    GPU_indexbuf_init(&builder, GPU_PRIM_TRIS, looptris_draw_len, vert_len);
    int loop_first = 0;
    BM_ITER_MESH_INDEX (f, &iter, bm, BM_FACES_OF_MESH, i) {
      if (BM_elem_flag_test(f, MESH_OPERAND_TAG | MESH_OPERAND_TAG2)) {
        int ltri_index = poly_to_tri_count(i, loop_first);
        int tri_len = f->len - 2;
        while (tri_len--) {
          std::array<BMLoop *, 3> &ltri = corner_tris[ltri_index++];
          GPU_indexbuf_add_tri_verts(&builder,
                                     BM_elem_index_get(ltri[0]->v),
                                     BM_elem_index_get(ltri[1]->v),
                                     BM_elem_index_get(ltri[2]->v));
        }
      }
      loop_first += f->len;
    }

    ibo_faces = GPU_indexbuf_build(&builder);
  }

  gpu::IndexBuf *ibo_edges;
  {
    GPUIndexBufBuilder builder;
    GPU_indexbuf_init(&builder, GPU_PRIM_LINES, edge_tag_len, vert_len);
    BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
      if (BM_elem_flag_test(e, MESH_OPERAND_TAG | MESH_OPERAND_TAG2)) {
        GPU_indexbuf_add_line_verts(&builder, BM_elem_index_get(e->v1), BM_elem_index_get(e->v2));
      }
    }
    ibo_edges = GPU_indexbuf_build(&builder);
  }

  gpu::Batch *batch_faces = nullptr;
  if (ibo_faces) {
    batch_faces = GPU_batch_create(GPU_PRIM_TRIS, vbo, ibo_faces);
    GPU_batch_program_set_builtin(batch_faces, GPU_SHADER_3D_UNIFORM_COLOR);
  }

  gpu::Batch *batch_edges = GPU_batch_create(GPU_PRIM_LINES, vbo, ibo_edges);
  GPU_batch_program_set_builtin(batch_edges, GPU_SHADER_3D_UNIFORM_COLOR);

  gpu::Batch *batch_verts = nullptr;
  if (draw_verts) {
    batch_verts = GPU_batch_create(GPU_PRIM_POINTS, vbo, nullptr);
    GPU_batch_program_set_builtin(batch_verts, GPU_SHADER_3D_UNIFORM_COLOR);
    r_draw_data->point_size = UI_GetThemeValuef(TH_VERTEX_SIZE) * UI_SCALE_FAC;
  }

  r_draw_data->region = region;
  r_draw_data->object_to_world = object_to_world;
  r_draw_data->batch_faces = batch_faces;
  r_draw_data->batch_edges = batch_edges;
  r_draw_data->batch_verts = batch_verts;
  r_draw_data->vbo = vbo;
  r_draw_data->ibo_faces = ibo_faces;
  r_draw_data->ibo_edges = ibo_edges;
  r_draw_data->v_co = std::move(v_co);

  UI_GetThemeColor4fv(TH_GIZMO_PRIMARY, r_draw_data->color);
  r_draw_data->color[3] = 0.25f;
  r_draw_data->draw_handle = ED_region_draw_cb_activate(
      region->type, extrude_boolean_draw_fn, r_draw_data, REGION_DRAW_POST_VIEW);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Extrude Data
 * \{ */

struct ExtrudeMeshData {
  Object *obedit;
  BMesh *bm;
  Array<float3, 1> island_centers;
  Array<BMVert *> moving_verts;
  Array<std::array<BMLoop *, 3>> corner_tris;
  int vert_tag_len;
  int edge_tag_len;
  int face_tag_len;
  int cd_offset;
  float doublimit;
  bool has_closed_geom;
  bool has_opened_geom;
  bool has_loose_edge;
  bool do_subtract;
  bool do_remove_coplanar;

  ExtrudeDrawData draw_data;
};

static void extrude_boolean_data_free(ExtrudeMeshData *extrudata)
{
  if (extrudata->draw_data.draw_handle) {
    extrude_boolean_drawdata_clear(extrudata->draw_data);
  }
  if (extrudata->bm) {
    BM_mesh_free(extrudata->bm);
  }
  MEM_delete(extrudata);
}

static void calc_point_in_tesselation(Span<std::array<BMLoop *, 3>> face_tesselation,
                                      float r_co[3])
{
  /* With this, we can be sure the point is inside a concave face. */
  int i_best = 0; /* use as fallback when unset */
  float area_best = -1.0f;
  for (int i : face_tesselation.index_range()) {
    auto corner_tri = &face_tesselation[i];
    const float *p1 = corner_tri[i][0]->v->co;
    const float *p2 = corner_tri[i][1]->v->co;
    const float *p3 = corner_tri[i][2]->v->co;
    const float area = area_squared_tri_v3(p1, p2, p3);
    if (area > area_best) {
      i_best = i;
      area_best = area;
    }
  }

  BMVert *tri_v[3] = {UNPACK3_EX(, face_tesselation[i_best], ->v)};
  mid_v3_v3v3v3(r_co, tri_v[0]->co, tri_v[1]->co, tri_v[2]->co);
}

static ExtrudeMeshData *extrude_boolean_data_create(Object *obedit, float doublimit)
{
  BMOIter oiter;
  BMElem *ele;
  BMFace *f;
  BMVert *v;

  if (!obedit || obedit->type != OB_MESH) {
    return nullptr;
  }

  BMEditMesh *em = BKE_editmesh_from_object(obedit);
  if (!em->bm->totvertsel) {
    return nullptr;
  }

  ExtrudeMeshData *extrudata = MEM_new<ExtrudeMeshData>(__func__);
  extrudata->bm = BM_mesh_copy(em->bm);
  extrudata->cd_offset = -1;

  if (em->bm->totfacesel) {
    int *groups_array = nullptr;
    int(*group_index)[2] = nullptr;
    groups_array = static_cast<int *>(
        MEM_mallocN(sizeof(*groups_array) * em->bm->totfacesel, __func__));
    int island_tot = BM_mesh_calc_face_groups(extrudata->bm,
                                              groups_array,
                                              &group_index,
                                              nullptr,
                                              nullptr,
                                              nullptr,
                                              BM_ELEM_SELECT,
                                              BM_EDGE);

    extrudata->island_centers.reinitialize(island_tot);
    if (island_tot > 1) {
      /* Use the customdata #CD_ORIGINDEX to identify which island a face belongs to. */
      BM_data_layer_add(extrudata->bm, &extrudata->bm->pdata, CD_ORIGINDEX);
      extrudata->cd_offset = CustomData_get_offset(&extrudata->bm->pdata, CD_ORIGINDEX);

      BM_mesh_elem_table_ensure(extrudata->bm, BM_FACE);
      for (int island_index : IndexRange(island_tot)) {
        const int fg_start = group_index[island_index][0];
        const int fg_length = group_index[island_index][1];
        Span<int> fg = {groups_array + fg_start, fg_length};
        for (int f_index : fg) {
          f = extrudata->bm->ftable[f_index];
          BM_ELEM_CD_SET_INT(f, extrudata->cd_offset, island_index);
        }
      }
    }
    MEM_freeN(groups_array);
    MEM_freeN(group_index);
  }

  BM_mesh_elem_hflag_disable_all(
      extrudata->bm, BM_VERT | BM_EDGE | BM_FACE, MESH_OPERAND_TAG | MESH_OPERAND_TAG2, false);

  int vert_tag_len = 0;
  int edge_tag_len = 0;
  int face_tag_len = 0;

  BMOperator dupop;
  BMOperator extrudeop;
  BMO_op_initf(extrudata->bm, &dupop, 0, "duplicate geom=%hvef", BM_ELEM_SELECT);
  BMO_op_exec(extrudata->bm, &dupop);
  BMO_ITER (ele, &oiter, dupop.slots_out, "geom_orig.out", BM_VERT | BM_EDGE | BM_FACE) {
    if (ele->head.htype != BM_FACE) {
      BM_elem_flag_disable(ele, BM_ELEM_SELECT);
    }
  }
  BMO_ITER (ele, &oiter, dupop.slots_out, "geom.out", BM_VERT | BM_EDGE) {
    BM_elem_flag_disable(ele, BM_ELEM_SELECT);
    BM_elem_flag_enable(ele, MESH_OPERAND_TAG2);
    if (ele->head.htype == BM_VERT) {
      vert_tag_len++;
    }
    else {
      edge_tag_len++;
    }
  }
  BMO_ITER (f, &oiter, dupop.slots_out, "geom.out", BM_FACE) {
    BM_elem_flag_disable(f, BM_ELEM_SELECT);
    BM_elem_flag_enable(f, MESH_OPERAND_TAG);
    face_tag_len++;
    BMLoop *l_iter, *l_first;
    l_iter = l_first = BM_FACE_FIRST_LOOP(f);
    do {
      BM_elem_flag_disable(l_iter->v, MESH_OPERAND_TAG2);
      BM_elem_flag_disable(l_iter->e, MESH_OPERAND_TAG2);
      BM_elem_flag_enable(l_iter->v, MESH_OPERAND_TAG);
      BM_elem_flag_enable(l_iter->e, MESH_OPERAND_TAG);
    } while ((l_iter = l_iter->next) != l_first);
  }
  BMO_ITER (ele, &oiter, dupop.slots_out, "geom.out", BM_VERT | BM_EDGE) {
    if (BM_elem_flag_test(ele, MESH_OPERAND_TAG2)) {
      if (ele->head.htype == BM_EDGE) {
        extrudata->has_opened_geom = true;
      }
      else if (reinterpret_cast<BMVert *>(ele)->e == nullptr) {
        extrudata->has_loose_edge = true;
      }
    }
  }

  extrudata->moving_verts.reinitialize(2 * vert_tag_len);
  vert_tag_len = 0;
  BMO_ITER (v, &oiter, dupop.slots_out, "geom.out", BM_VERT) {
    extrudata->moving_verts[vert_tag_len++] = v;
  }

  extrudata->has_closed_geom = face_tag_len != 0;

  BMO_op_initf(extrudata->bm, &extrudeop, 0, "extrude_face_region geom=%S", &dupop, "geom.out");
  BMO_op_exec(extrudata->bm, &extrudeop);
  BMO_ITER (ele, &oiter, extrudeop.slots_out, "geom.out", BM_ALL_NOLOOP) {
    BM_elem_select_set(extrudata->bm, ele, true);
    if (ele->head.htype == BM_VERT) {
      extrudata->moving_verts[vert_tag_len++] = reinterpret_cast<BMVert *>(ele);
      /* Tag the connected edge for drawing. */
      BMIter iter;
      BMEdge *e;
      BM_ITER_ELEM (e, &iter, ele, BM_EDGES_OF_VERT) {
        if (!BM_elem_flag_test(e, MESH_OPERAND_TAG | MESH_OPERAND_TAG2)) {
          /* Tag the connected edge for the wire intersection. */
          if (BM_elem_flag_test(ele, MESH_OPERAND_TAG2)) {
            BM_elem_flag_enable(e, MESH_OPERAND_TAG2);
          }
          else {
            BM_elem_flag_enable(e, MESH_OPERAND_TAG);
          }
          edge_tag_len++;
          break;
        }
      }
    }
    else if (ele->head.htype == BM_EDGE) {
      edge_tag_len++;
      if (BM_elem_flag_test(ele, MESH_OPERAND_TAG2)) {
        /* Tag the connected face for the opened intersection. */
        BM_elem_flag_enable(((BMEdge *)ele)->l->f, MESH_OPERAND_TAG2);
      }
      face_tag_len++;
    }
  }

  {
    /* Tessellation. */
    int corner_tris_len = poly_to_tri_count(extrudata->bm->totface, extrudata->bm->totloop);
    Array<std::array<BMLoop *, 3>> corner_tris(corner_tris_len);
    BMeshCalcTessellation_Params params = {};
    params.face_normals = true;
    BM_mesh_calc_tessellation_ex(extrudata->bm, corner_tris, &params);
    extrudata->corner_tris = std::move(corner_tris);

    /* Calculate a point on the islands after tessellation, as this will ensure that it is on the
     * surface used by the raycast. */
    int unfilled = extrudata->island_centers.size();
    extrudata->island_centers.fill(float3(std::nanf("")));
    BM_mesh_elem_index_ensure(extrudata->bm, BM_LOOP | BM_FACE);
    BMO_ITER (f, &oiter, dupop.slots_out, "geom.out", BM_FACE) {
      int island_index = 0;
      if (extrudata->cd_offset != -1) {
        island_index = std::max(BM_ELEM_CD_GET_INT(f, extrudata->cd_offset), 0);
      }
      if (std::isnan(extrudata->island_centers[island_index][0])) {
        int f_index = BM_elem_index_get(f);
        int l_index = BM_elem_index_get(f->l_first);
        int ltri_index = poly_to_tri_count(f_index, l_index);
        calc_point_in_tesselation(extrudata->corner_tris.as_span().slice(ltri_index, f->len - 2),
                                  extrudata->island_centers[island_index]);
        if (--unfilled == 0) {
          break;
        }
      }
    }
  }
  extrudata->edge_tag_len = edge_tag_len;
  extrudata->face_tag_len = face_tag_len;
  extrudata->obedit = obedit;
  extrudata->doublimit = doublimit;
  extrudata->do_subtract = false;
  extrudata->do_remove_coplanar = true;
  BMO_op_finish(extrudata->bm, &extrudeop);
  BMO_op_finish(extrudata->bm, &dupop);
  return extrudata;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Draw Update
 * \{ */

static void extrude_boolean_recalc_data_fn(void *usedata)
{
  ExtrudeMeshData *extrudata = static_cast<ExtrudeMeshData *>(usedata);
  if (!extrudata->draw_data.draw_handle) {
    return;
  }

  int i = 0;
  for (BMVert *v :
       extrudata->moving_verts.as_span().drop_front(extrudata->moving_verts.size() / 2))
  {
    extrudata->draw_data.v_co[i++] = v->co;
  }
  GPU_vertbuf_tag_dirty(extrudata->draw_data.vbo);
  ED_region_tag_redraw(extrudata->draw_data.region);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Apply Operator
 * \{ */

struct RaycastUserData {
  const float3 (*looptri_coords)[3];
  Vector<float> dists;
};

static void raycast_fn(void *userdata, int index, const BVHTreeRay *ray, BVHTreeRayHit *hit)
{
  RaycastUserData *data = static_cast<RaycastUserData *>(userdata);
  const float3(*coords)[3] = data->looptri_coords;
  float dist = bvhtree_ray_tri_intersection(ray, 0.0f, UNPACK3(coords[index]));
  if (dist < hit->dist) {
    data->dists.append(dist);
  }
}

static bool is_point_inside_bound(const Bounds<float3> &bb, const float3 &point)
{
  return (point[0] < bb.min[0]) || (point[1] < bb.min[1]) || (point[2] < bb.min[2]) ||
         (point[0] > bb.max[0]) || (point[1] > bb.max[1]) || (point[2] > bb.max[2]);
}

static bool is_face_group_to_remove_fn(ExtrudeMeshData *extrudata,
                                       BVHTree *trees[2],
                                       const float3 (*looptri_coords)[3],
                                       BMFace *const *ftable,
                                       Span<int> face_indices,
                                       int side)
{
  BMFace *f = ftable[face_indices[0]];
  if (side == 0) {
    if (BM_elem_flag_test(f, BM_ELEM_SELECT)) {
      /* Always remove the original selected side 0 mesh face. */
      return true;
    }
  }

  int island_index = 0;
  if (extrudata->cd_offset != -1) {
    island_index = std::max(BM_ELEM_CD_GET_INT(f, extrudata->cd_offset), 0);
  }
  const float3 &move_src = extrudata->island_centers[island_index];
  float3 ray_orig;
  BM_face_calc_point_in_face(f, ray_orig);
  float move_src_dist;
  float3 ray_dir = math::normalize_and_get_length(move_src - ray_orig, move_src_dist);

  BVHTreeRayHit hit = {0};
  hit.dist = BVH_RAYCAST_DIST_MAX;
  if (math::is_zero(ray_dir)) {
    ray_dir[0] = 1.0f;
  }
  else {
    if (side == 1) {
      /* Limit distances to intersections. */
      hit.dist = move_src_dist - RAYCAST_DEPTH_EPSILON;
    }
    else {
      Bounds<float3> bb;
      BLI_bvhtree_get_bounding_box(trees[1], bb.min, bb.max);
      if (is_point_inside_bound(bb, ray_orig)) {
        /* Quick test to see if the point is outside the mesh. */
        return false;
      }
      /* Move the ray out a little to identify coplanar cases for the "remove_coplanar" option. */
      ray_orig -= ray_dir * RAYCAST_DEPTH_EPSILON;
    }
  }

  RaycastUserData data = {looptri_coords};
  BLI_bvhtree_ray_cast(trees[!side], ray_orig, ray_dir, 0.0f, &hit, raycast_fn, &data);

  bool is_inside = false;
  if (!data.dists.is_empty()) {
    /* Remove hits with equal distances since they can come from the same face. */
    std::sort(data.dists.begin(), data.dists.end());
    int equal_dists_len = 0;
    float dist_prev = data.dists[0];
    for (float dist : data.dists.as_span().drop_front(1)) {
      if ((dist - dist_prev) <= RAYCAST_DEPTH_EPSILON) {
        equal_dists_len++;
      }
      dist_prev = dist;
    }
    int hits = data.dists.size() - equal_dists_len;
    is_inside = (hits & 1);
  }
  return side == 1 && extrudata->do_subtract ? !is_inside : is_inside;
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Extrude Boolean Apply
 * \{ */

static int bm_face_isect_pair_fn(BMFace *f, void *user_data)
{
  if (BM_elem_flag_test(f, BM_ELEM_HIDDEN)) {
    return -1;
  }
  if (BM_elem_flag_test(f, POINTER_AS_INT(user_data))) {
    return 1;
  }
  return 0;
};

static void extrude_boolean_apply_fn(void *userdata, bool is_cancel)
{
  ExtrudeMeshData *extrudata = static_cast<ExtrudeMeshData *>(userdata);
  BMEditMesh *em = BKE_editmesh_from_object(extrudata->obedit);
  BMIter iter;
  BMFace *f;
  if (is_cancel) {
    extrude_boolean_data_free(extrudata);

    /* Untag #BM_ELEM_HIDDEN. */
    BM_ITER_MESH (f, &iter, em->bm, BM_FACES_OF_MESH) {
      if (BM_elem_flag_test(f, BM_ELEM_TAG)) {
        BM_elem_flag_disable(f, BM_ELEM_HIDDEN | BM_ELEM_TAG);
      }
    }
    return;
  }

  if (extrudata->has_closed_geom) {
    auto remove_face_group_fn = [&](BVHTree *trees[2],
                                    const float3(*looptri_coords)[3],
                                    BMFace *const *ftable,
                                    Span<int> face_indices,
                                    int side) -> bool {
      return is_face_group_to_remove_fn(
          extrudata, trees, looptri_coords, ftable, face_indices, side);
    };
    bool has_isect = BM_mesh_intersect_v2(extrudata->bm,
                                          extrudata->corner_tris,
                                          bm_face_isect_pair_fn,
                                          POINTER_FROM_INT(MESH_OPERAND_TAG),
                                          false,
                                          false,
                                          true,
                                          true,
                                          true,
                                          extrudata->do_remove_coplanar,
                                          extrudata->doublimit,
                                          remove_face_group_fn);
    if (has_isect) {
      /* `corner_tris` is now invalid. */
      extrudata->corner_tris = {};
    }
  }
  if (extrudata->has_loose_edge || extrudata->has_opened_geom) {
    BMOperator weldop;
    BMOpSlot *slot_targetmap;
    BMO_op_init(extrudata->bm, &weldop, BMO_FLAG_DEFAULTS, "weld_verts");
    slot_targetmap = BMO_slot_get(weldop.slots_in, "targetmap");
    GHash *ghash_targetmap = BMO_SLOT_AS_GHASH(slot_targetmap);
    bool has_isect = BM_mesh_intersect_edges(
        extrudata->bm, MESH_OPERAND_TAG2, extrudata->doublimit, true, ghash_targetmap);
    if (has_isect) {
      BMO_op_exec(extrudata->bm, &weldop);
      /* `corner_tris` is now invalid. */
      extrudata->corner_tris = {};
    }
    BMO_op_finish(extrudata->bm, &weldop);
  }
  if (extrudata->has_opened_geom) {
    /* Intersect the rest of the geometry. */
    if (extrudata->corner_tris.is_empty()) {
      int corner_tris_len = poly_to_tri_count(extrudata->bm->totface, extrudata->bm->totloop);
      extrudata->corner_tris.reinitialize(corner_tris_len);
      BMeshCalcTessellation_Params params = {};
      params.face_normals = true;
      BM_mesh_calc_tessellation_ex(extrudata->bm, extrudata->corner_tris, &params);
    }
    bool has_isect = BM_mesh_intersect_v2(extrudata->bm,
                                          extrudata->corner_tris,
                                          bm_face_isect_pair_fn,
                                          POINTER_FROM_INT(MESH_OPERAND_TAG2),
                                          false,
                                          false,
                                          true,
                                          true,
                                          true,
                                          false,
                                          extrudata->doublimit,
                                          nullptr);
    if (has_isect) {
      /* `corner_tris` is now invalid. */
      extrudata->corner_tris = {};
    }
  }

  if (extrudata->cd_offset != -1) {
    BM_data_layer_free(extrudata->bm, &extrudata->bm->pdata, CD_ORIGINDEX);
  }

  if (em->bm != extrudata->bm) {
    BM_mesh_free(em->bm);
    em->bm = extrudata->bm;
  }
  em->looptris = std::move(extrudata->corner_tris);
  extrudata->bm = nullptr;

  BM_ITER_MESH (f, &iter, em->bm, BM_FACES_OF_MESH) {
    if (BM_elem_flag_test(f, BM_ELEM_SELECT)) {
      BM_face_select_set(em->bm, f, true);
    }
  }
  EDBM_selectmode_flush(em);

  EDBMUpdate_Params params = {};
  if (em->looptris.is_empty()) {
    params.calc_looptris = true;
    params.calc_normals = true;
    params.is_destructive = true;
  }
  EDBM_update(static_cast<Mesh *>(extrudata->obedit->data), &params);
  extrude_boolean_data_free(extrudata);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Extrude Boolean Operator
 * \{ */

static int mesh_extrude_boolean_exec(bContext *C, wmOperator *op)
{
  if (!op->opm) {
    BKE_report(op->reports, RPT_ERROR, "Operator needs to be called in a macro");
    return OPERATOR_CANCELLED;
  }

  Object *obedit = CTX_data_edit_object(C);
  if (!obedit || obedit->type != OB_MESH) {
    return OPERATOR_CANCELLED;
  }

  const float eps = RNA_float_get(op->ptr, "threshold");
  ExtrudeMeshData *extrudata = extrude_boolean_data_create(obedit, eps);
  if (!extrudata) {
    return OPERATOR_CANCELLED;
  }

  extrudata->do_subtract = RNA_boolean_get(op->ptr, "invert");
  extrudata->do_remove_coplanar = RNA_boolean_get(op->ptr, "remove_coplanar");
  if (!(op->flag & OP_IS_REPEAT)) {
    ARegion *region = CTX_wm_region(C);
    extrude_boolean_drawdata_create(region,
                                    extrudata->bm,
                                    extrudata->moving_verts,
                                    extrudata->corner_tris,
                                    obedit->object_to_world().ptr(),
                                    extrudata->edge_tag_len,
                                    extrudata->face_tag_len,
                                    extrudata->has_loose_edge,
                                    &extrudata->draw_data);

    /* Tag (not hide) selected faces with #BM_ELEM_HIDDEN. This helps with snapping. */
    BMEditMesh *em = BKE_editmesh_from_object(obedit);
    BMIter iter;
    BMFace *f;
    BM_ITER_MESH (f, &iter, em->bm, BM_FACES_OF_MESH) {
      if (BM_elem_flag_test(f, BM_ELEM_SELECT)) {
        BM_elem_flag_enable(f, BM_ELEM_HIDDEN | BM_ELEM_TAG);
      }
      else {
        BM_elem_flag_disable(f, BM_ELEM_TAG);
      }
    }
  }

  int transform_data_len = extrudata->moving_verts.size() / 2;
  bool ok = ED_transform_reserve_custom(
      obedit,
      transform_data_len,
      extrudata,
      [&](int index, TransDataBasic &r_td, float r_no[3]) {
        r_td.loc = extrudata->moving_verts[transform_data_len + index]->co;
        copy_v3_v3(r_td.iloc, r_td.loc);
        copy_v3_v3(r_td.center, r_td.loc);
        copy_v3_v3(r_no, extrudata->moving_verts[transform_data_len + index]->no);
      },
      extrude_boolean_recalc_data_fn,
      extrude_boolean_apply_fn);

  if (!ok) {
    extrude_boolean_data_free(extrudata);
    return OPERATOR_CANCELLED;
  }

  return OPERATOR_FINISHED;
}

void MESH_OT_extrude_boolean_intern(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Mesh Extrude Boolean Intern";
  ot->idname = "MESH_OT_extrude_boolean_intern";
  ot->description = "Extrude and do a boolean operation";

  ot->poll = ED_operator_editmesh_view3d;
  ot->exec = mesh_extrude_boolean_exec;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;

  /* props */
  RNA_def_float_distance(
      ot->srna, "threshold", 0.0001f, 0.0f, FLT_MAX, "Overlap Threshold", "", 0.0001f, 0.1f);
  RNA_def_boolean(ot->srna, "invert", false, "Invert", "Which boolean operation to apply");
  RNA_def_boolean(ot->srna, "remove_coplanar", true, "Remove coplanar faces", "");
}

/** \} */
