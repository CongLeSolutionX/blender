/* SPDX-FileCopyrightText: 2017 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include <stdio.h>

#include "BLI_string.h"

#include "BLI_listbase.h"
#include "BLI_math_geom.h"
#include "BLI_math_vector_types.hh"
#include "BKE_attribute.hh"
#include "BLI_utildefines.h"
#include "BLI_math_matrix.h"

#include "BLT_translation.hh"

#include "BKE_context.hh"
#include "BKE_scene.hh"
#include "BKE_gpencil_geom_legacy.h"
#include "BKE_gpencil_legacy.h"
#include "BKE_gpencil_modifier_legacy.h"
#include "BKE_curves.hh"
#include "BKE_deform.hh"
#include "BKE_geometry_set.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_instances.hh"
#include "BKE_lib_query.hh"
#include "BKE_modifier.hh"
#include "BKE_screen.hh"
#include "BKE_shrinkwrap.hh"
#include "DNA_defaults.h"
#include "DNA_gpencil_legacy_types.h"
#include "DNA_gpencil_modifier_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"
#include "DEG_depsgraph_query.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.hh"
#include "ED_grease_pencil.hh"
#include "DNA_customdata_types.h"


#include "MOD_util.hh"
#include "MOD_grease_pencil_util.hh"
#include "MOD_modifiertypes.hh"
#include "MOD_ui_common.hh"

#include "MEM_guardedalloc.h"

#include "WM_api.hh"

/* HEADER FROM MOD_surfacedeform.c */

#include "BLI_alloca.h"
#include "BLI_task.h"

#include "DNA_mesh_types.h"

#include "BKE_bvhutils.hh"
#include "BKE_editmesh.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_runtime.hh"
#include "BKE_mesh_wrapper.hh"



namespace blender {


struct SDefGPDeformData {
  const VArraySpan<int> *sdef_vert_idx_span;
  float (*targetCos)[3];
  float (*vertexCos)[3];
  const MDeformVert *dvert;
  int defgrp_index;
  bool invert_vgroup;
  float strength;
  bke::AttributeAccessor*  attributes;
  SDefGPVert *vert_binds;
};


struct GPencilSurDeformModifierData *get_original_modifier(Object *ob,
                                                           GPencilSurDeformModifierData *smd,
                                                           ModifierData *md)
{
  Object *object_orig = DEG_get_original_object(ob);
  GPencilSurDeformModifierData *smd_orig;
  ModifierData *md_orig;

  if (object_orig == ob) {
    smd_orig = smd;
    md_orig = md;
  }
  else {
    md_orig = BKE_modifiers_findby_name(object_orig, md->name);
    smd_orig = (GPencilSurDeformModifierData *)md_orig;
  }

  return smd_orig;
}


static void init_data(ModifierData *md)
{
  GPencilSurDeformModifierData *smd = reinterpret_cast<GPencilSurDeformModifierData *>(md);

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(smd, modifier));

  MEMCPY_STRUCT_AFTER(smd, DNA_struct_default_get(GPencilSurDeformModifierData), modifier);

  //modifier::greasepencil::init_influence_data(&smd->influence, false); TODO_INFLUENCE
}

static void required_data_mask(ModifierData *md, CustomData_MeshMasks *r_cddata_masks)
{
  GPencilSurDeformModifierData *smd = (GPencilSurDeformModifierData *)md;

  /* Ask for vertex groups if we need them. TODO_INFLUENCE 
  if (smd->defgrp_name[0] != '\0') {
    r_cddata_masks->vmask |= CD_MASK_MDEFORMVERT;
  }*/
}

static void free_data(ModifierData *md)
{
  auto *smd = reinterpret_cast<GPencilSurDeformModifierData *>(md);
  modifier::greasepencil::free_influence_data(&smd->influence);
  //TODO remove data from point attributes!!!
  for (int i = 0; i < smd->verts_array_tot; i++) {
    MEM_SAFE_FREE(smd->verts_array[i].binds_array);
  }
  MEM_SAFE_FREE(smd->verts_array);
  smd->verts_array_tot = 0;
  smd->verts_array_occupied = 0;
}

static void copy_data(const ModifierData *md, ModifierData *target, const int flag)
{
  const auto *smd = reinterpret_cast<const GPencilSurDeformModifierData *>(md);
  auto *tsmd = reinterpret_cast<GPencilSurDeformModifierData *>(target);

  

  BKE_modifier_copydata_generic(md, target, flag);
  modifier::greasepencil::copy_influence_data(&smd->influence, &tsmd->influence, flag);
  tsmd->verts_array = reinterpret_cast<SDefGPVert *>(
      MEM_dupallocN(smd->verts_array));
  for (int i = 0; i < smd->verts_array_tot; i++) {
    tsmd->verts_array[i].binds_array = reinterpret_cast<SDefGPBind *>(
        MEM_dupallocN(smd->verts_array[i].binds_array));
  }
 
  
}

static bool dependsOnTime(ModifierData *md)

{
  return true;
}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  GPencilSurDeformModifierData *smd = (GPencilSurDeformModifierData *)md;

  walk(user_data, ob, (ID **)&smd->target, IDWALK_NOP);
}



static void update_depsgraph(ModifierData *md, const ModifierUpdateDepsgraphContext *ctx)
{
  const auto *smd = reinterpret_cast<const GPencilSurDeformModifierData *>(md);
  if (smd->target != nullptr) {
    DEG_add_object_relation(
        ctx->node, smd->target, DEG_OB_COMP_GEOMETRY, "Surface Deform GP Modifier");
    DEG_add_object_relation(
        ctx->node, smd->target, DEG_OB_COMP_TRANSFORM, "Surface Deform GP Modifier");
  }
}


static void deformVert(void *__restrict userdata,
                       const int index,
                       const TaskParallelTLS *__restrict /*tls*/)
{
  const SDefGPDeformData *const data = (SDefGPDeformData *)userdata;

  // Construct attributes for each bind
  bke::AttributeAccessor *attributes = data->attributes;
  int sdbind_num = 0;
  if (auto sdbind_num_lookup = attributes->lookup<int>("sdef_binds_num", bke::AttrDomain::Point)) {
    sdbind_num = attributes->lookup<int>("sdef_binds_num", bke::AttrDomain::Point).varray.get(index);
  }
  if (sdbind_num < 1)
    return;


  SDefBind *sdbinds = new SDefBind[sdbind_num];
  for (int i = 0; i < sdbind_num; i++) {
    const std::string index_str = std::to_string(i);

     // verts_num
    if (auto verts_num_lookup = attributes->lookup<int>("sdef_verts_num_bind" + index_str,
                                                        bke::AttrDomain::Point))
    {
      sdbinds[i].verts_num = verts_num_lookup.varray.get(index);
    }
    else
      return;

    // vert_inds
    sdbinds[i].vert_inds = new uint[sdbinds[i].verts_num];
    for (int j = 0; j < sdbinds[i].verts_num; j++) {
      if (auto vert_inds_lookup = attributes->lookup<int>(
              "sdef_vert_inds_bind" + index_str + "_" + std::to_string(j), bke::AttrDomain::Point))
      {
        sdbinds[i].vert_inds[j] = vert_inds_lookup.varray.get(index);
      }
      else {
        return;
      }
    }

    // mode
    if (auto mode_lookup = attributes->lookup<int>("sdef_mode_bind" + index_str,
                                                   bke::AttrDomain::Point))
    {
      sdbinds[i].mode = mode_lookup.varray.get(index);
    }
    else
      return;
   
    // vert_weights
    
    sdbinds[i].vert_weights = new float[sdbinds[i].verts_num];
    for (int j = 0; j < sdbinds[i].verts_num; j++) {
      if (auto vert_weights_lookup = attributes->lookup<float>("sdef_vert_weights_bind" + index_str +
                                                                "_" + std::to_string(j), bke::AttrDomain::Point))
      {
        sdbinds[i].vert_weights[j] = vert_weights_lookup.varray.get(index);
      }
      else {
        return;
      }
    }


    // normal_dist
    if (auto normal_dist_lookup = attributes->lookup<float>("sdef_normal_dist_bind" + index_str,
                                                            bke::AttrDomain::Point))
    {
      sdbinds[i].normal_dist = normal_dist_lookup.varray.get(index);
    }
    else
      return;

    // influence
    if (auto influence_lookup = attributes->lookup<float>("sdef_influence_bind" + index_str,
                                                          bke::AttrDomain::Point))
    {
      sdbinds[i].influence = influence_lookup.varray.get(index);
    }
    else
      return;
  }

  // const SDefGPBind *sdbind = data->bind_verts[index].binds;
  
  unsigned int vertex_idx = index;  // ???????????? TODO: verify!!!!!!!!!!!
  // const unsigned int vertex_idx = data->bind_verts[index].vertex_idx;
  float *const vertexCos = data->vertexCos[index];
  // float *vertexCos = &(data->gps->points[vertex_idx].x); POSITIONS
  float norm[3], temp[3], offset[3];
  float tmp_mat_err[4][4] = {
      {1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 1.0, -1.0}, {0.0, -1.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};

  /* Retrieve the value of the weight vertex group if specified. */
  float weight = 1.0f;

  if (data->dvert && data->defgrp_index != -1) {
    weight = BKE_defvert_find_weight(&data->dvert[vertex_idx], data->defgrp_index);
  }
  /* Check if this vertex will be deformed. If it is not deformed we return and avoid
   * unnecessary calculations. */
  if (weight <= 0.0f) {
    return;
  }

  zero_v3(offset);

  /* Allocate a `coords_buffer` that fits all the temp-data. */
  int max_verts = 0;
  for (int j = 0; j < sdbind_num; j++) {
    max_verts = std::max(max_verts, int(sdbinds[j].verts_num));
  }

  blender::Array<blender::float3, 256> coords_buffer(max_verts);

  for (int j = 0; j < sdbind_num; j++) {
    for (int k = 0; k < sdbinds[j].verts_num; k++) {
      copy_v3_v3(coords_buffer[k], data->targetCos[sdbinds[j].vert_inds[k]]);
    }

     normal_poly_v3(
        norm, reinterpret_cast<const float(*)[3]>(coords_buffer.data()), sdbinds[j].verts_num);
    zero_v3(temp);

    switch (sdbinds[j].mode) {
      /* ---------- looptri mode ---------- */
      case MOD_SDEF_MODE_CORNER_TRIS: {
        madd_v3_v3fl(temp, data->targetCos[sdbinds[j].vert_inds[0]], sdbinds[j].vert_weights[0]);
        madd_v3_v3fl(temp, data->targetCos[sdbinds[j].vert_inds[1]], sdbinds[j].vert_weights[1]);
        madd_v3_v3fl(temp, data->targetCos[sdbinds[j].vert_inds[2]], sdbinds[j].vert_weights[2]);
        break;
      }

      /* ---------- ngon mode ---------- */
      case MOD_SDEF_MODE_NGONS: {
        for (int k = 0; k < sdbinds[j].verts_num; k++) {
          madd_v3_v3fl(temp, coords_buffer[k], sdbinds[j].vert_weights[k]);
        }
        break;
      }

      /* ---------- centroid mode ---------- */
      case MOD_SDEF_MODE_CENTROID: {
        float cent[3];
        mid_v3_v3_array(
            cent, reinterpret_cast<const float(*)[3]>(coords_buffer.data()), sdbinds[j].verts_num);

        madd_v3_v3fl(temp, data->targetCos[sdbinds[j].vert_inds[0]], sdbinds[j].vert_weights[0]);
        madd_v3_v3fl(temp, data->targetCos[sdbinds[j].vert_inds[1]], sdbinds[j].vert_weights[1]);
        madd_v3_v3fl(temp, cent, sdbinds[j].vert_weights[2]);
        break;
      }
    }

    /* Apply normal offset (generic for all modes) */
    madd_v3_v3fl(temp, norm, sdbinds[j].normal_dist);

    madd_v3_v3fl(offset, temp, sdbinds[j].influence);
  }

  /* Subtract the vertex coord to get the deformation offset. */
  sub_v3_v3(offset, vertexCos);

  /* Add the offset to start coord multiplied by the strength and weight values. */
  madd_v3_v3fl(vertexCos, offset, data->strength * weight);

  /*free array*/
  for (int i = 0; i < sdbind_num; i++) {
    delete sdbinds[i].vert_inds;
    delete sdbinds[i].vert_weights;
  }
  delete sdbinds;

}



static void surfacedeform_deform(ModifierData *md,
                                     Depsgraph *depsgraph,
                                     bke::greasepencil::Drawing &drawing,
                                     Object *ob)
{
  auto *smd = reinterpret_cast<GPencilSurDeformModifierData *>(md);
  bke::CurvesGeometry &strokes = drawing.strokes_for_write();
  const OffsetIndices<int> points_by_curve = strokes.points_by_curve();
  const Span<MDeformVert> dverts = strokes.deform_verts();
  const MutableSpan<float3> positions = strokes.positions_for_write();
  Mesh *target;
  uint target_verts_num, target_polys_num;
  uint verts_num = strokes.points_num();
  uint strokes_num = strokes.curves_num();
  GPencilSurDeformModifierData *smd_orig = reinterpret_cast<GPencilSurDeformModifierData *>(
      BKE_modifier_get_original(ob, md));
  Object *ob_orig = (Object *)DEG_get_original_id(&ob->id);

  ModifierData *md_orig = BKE_modifier_get_original(ob_orig, md);
  bke::AttributeAccessor attributes = strokes.attributes();
  
   /* if (md_orig->error)
  {
    MEM_freeN(md_orig->error);
    md_orig->error = NULL;
  }*/

  Object *ob_target = DEG_get_evaluated_object(depsgraph, smd->target);
  target = BKE_modifier_get_evaluated_mesh_from_evaluated_object(ob_target);

 
  if (!target) {
    BKE_modifier_set_error(ob_orig, md, "No valid target mesh");
    return;
  }
  target_verts_num = BKE_mesh_wrapper_vert_len(target);
  target_polys_num = BKE_mesh_wrapper_face_len(target);


  /*Exit evaluation if we are on a frame that is not bound.
  (? USELESS OR DANGEROUS)

  if (smd->layers->frames->frame_number != gpf->framenum)
  {
    end_stroke_evaluation(smd, gpf);
    return;
  }*/

  /* Strokes count on the deforming Frame. 
  if (!(BLI_listbase_is_empty(&gpf->strokes))) {
    uint tot_strokes_num = BLI_listbase_count(&gpf->strokes);
    if (smd->layers->frames->strokes_num != tot_strokes_num) {
      BKE_modifier_set_error(md_orig,
                                     "Strokes changed from %u to %u",
                                     smd->layers->frames->strokes_num,
                                     tot_strokes_num);
      bGPDstroke *gps_ptr_copy = gps;
      int i = 0;
      while (gps_ptr_copy->prev != NULL) {
        gps_ptr_copy = gps_ptr_copy->prev;
        i++;
      }
      if (i >= smd->layers->frames->strokes_num)
        return;
      // free_frame_b(smd_orig, smd, smd->layers, gpf->framenum);
      // return;
    }
  }*/

  /* Points count on the deforming Stroke. 
  if (smd->layers->frames->strokes->stroke_verts_num != gps->totpoints) {
    BKE_modifier_set_error(md_orig,
                                   "Stroke %u: Points changed from %i to %i",
                                   smd->layers->frames->strokes->stroke_idx,
                                   smd->layers->frames->strokes->stroke_verts_num,
                                   gps->totpoints);
    // return;
  }*/

  /* Geometry count on the target mesh. 
  if (smd->target_polys_num != target_polys_num && smd->target_verts_num == 0) {
    /* Change in the number of polygons does not really imply change in the vertex count, but
     * this is how the modifier worked before the vertex count was known. Follow the legacy
     * logic without requirement to re-bind the mesh. 
    BKE_modifier_set_error( ob_orig,
        md_orig, "Target polygons changed from %u to %u", smd->target_polys_num, target_polys_num);
    return;
  } 
  if (smd->target_verts_num != 0 && smd->target_verts_num != target_verts_num) {
    if (smd->target_verts_num > target_verts_num) {
      /* Number of vertices on the target did reduce. There is no usable recovery from this. 
      BKE_modifier_set_error(ob_orig,
                             md_orig,
                                     "Target vertices changed from %u to %u",
                                     smd->target_verts_num,
                                     target_verts_num);
      return;
}
   */

    /* Assume the increase in the vertex count means that the "new" vertices in the target mesh are
     * added after the original ones. This covers typical case when target was at the subdivision
     * level 0 and then subdivision was increased (i.e. for the render purposes).

    BKE_modifier_set_warning(ob,
                             md,
                             "Target vertices changed from %u to %u, continuing anyway",
                             smd->target_verts_num,
                             target_verts_num);

    
  }*/

  

  /* Early out if modifier would not affect input at all - still *after* the sanity checks
   * (and potential binding) above. */
  if (smd->strength == 0.0f) {
    return;
  }

  int defgrp_index = BKE_object_defgroup_name_index(ob, smd->defgrp_name);
 
  // MOD_get_vgroup(ob, mesh, smd->defgrp_name, &dvert, &defgrp_index);
  const bool invert_vgroup = (smd->flags & GP_MOD_SDEF_INVERT_VGROUP) != 0;

  /* Actual vertex location update starts here */

  SDefGPDeformData data{};
  data.targetCos = static_cast<float(*)[3]>(
      MEM_malloc_arrayN(target_verts_num, sizeof(float[3]), "SDefTargetVertArray"));
  data.dvert = dverts.data();
  data.defgrp_index = defgrp_index;
  data.invert_vgroup = invert_vgroup;
  data.strength = smd->strength;
  data.vert_binds = smd->verts_array;
  data.vertexCos = reinterpret_cast<float(*)[3]>(positions.data());
  data.attributes = &attributes;

  

  if (data.targetCos != NULL) {

    
    float tmp_mat[4][4] = {
        {1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 1.0, -1.0}, {0.0, -1.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
    BKE_mesh_wrapper_vert_coords_copy_with_mat4(
        target, data.targetCos, target_verts_num, smd->mat);

    TaskParallelSettings settings;
    BLI_parallel_range_settings_defaults(&settings);
    settings.use_threading = (verts_num > 10000);
    BLI_task_parallel_range(0, verts_num, &data, deformVert, &settings);

    MEM_freeN(data.targetCos);
  }

  //BKE_gpencil_stroke_geometry_update(ob->data, gps);
}

/* END MOD_surfacedeform.c FUNCTIONS */

/*Start binding*/
/* Bind result values */
enum {
  MOD_SDEF_BIND_RESULT_SUCCESS = 1,
  MOD_SDEF_BIND_RESULT_GENERIC_ERR = 0,
  MOD_SDEF_BIND_RESULT_MEM_ERR = -1,
  MOD_SDEF_BIND_RESULT_NONMANY_ERR = -2,
  MOD_SDEF_BIND_RESULT_CONCAVE_ERR = -3,
  MOD_SDEF_BIND_RESULT_OVERLAP_ERR = -4,
};

/* Infinite weight flags */
enum {
  MOD_SDEF_INFINITE_WEIGHT_ANGULAR = (1 << 0),
  MOD_SDEF_INFINITE_WEIGHT_DIST_PROJ = (1 << 1),
  MOD_SDEF_INFINITE_WEIGHT_DIST = (1 << 2),
};

/**
 * This represents the relationship between a point (a source coordinate)
 * and the face-corner it's being bound to (from the target mesh).
 *
 * \note Some of these values could be de-duplicated however these are only
 * needed once when running bind, so optimizing this structure isn't a priority.
 */
typedef struct SDefBindPoly {
  /** Coordinates copied directly from the modifiers input. */
  float (*coords)[3];
  /** Coordinates projected into 2D space using `normal`. */
  float (*coords_v2)[2];
  /** The point being queried projected into 2D space using `normal`. */
  float point_v2[2];
  float weight_angular;
  float weight_dist_proj;
  float weight_dist;
  float weight;
  /** Distances from the centroid to edges flanking the corner vertex, used to penalize
   *  small or long and narrow faces in favor of bigger and more square ones. */
  float scales[2];
  /** Distance weight from the corner vertex to the chord line, used to penalize
   *  cases with the three consecutive vertices being nearly in line. */
  float scale_mid;
  /** Center of `coords` */
  float centroid[3];
  /** Center of `coords_v2` */
  float centroid_v2[2];
  /**
   * The calculated normal of coords (could be shared between faces).
   */
  float normal[3];
  /** Vectors pointing from the centroid to the midpoints of the two edges
   *  flanking the corner vertex. */
  float cent_edgemid_vecs_v2[2][2];
  /** Angle between the cent_edgemid_vecs_v2 vectors. */
  float edgemid_angle;
  /** Angles between the centroid-to-point and cent_edgemid_vecs_v2 vectors.
   *  Positive values measured towards the corner; clamped non-negative. */
  float point_edgemid_angles[2];
  /** Angles between the centroid-to-corner and cent_edgemid_vecs_v2 vectors. */
  float corner_edgemid_angles[2];
  /** Weight of the bind mode based on the corner and two adjacent vertices,
   *  versus the one based on the centroid and the dominant edge. */
  float dominant_angle_weight;
  /** Index of the input polygon. */
  uint index;
  /** Number of vertices in this face. */
  uint verts_num;
  /**
   * This polygons loop-start.
   * \note that we could look this up from the polygon.
   */
  uint loopstart;
  uint edge_inds[2];
  uint edge_vert_inds[2];
  /** The index of this corner in the face (starting at zero). */
  uint corner_ind;
  uint dominant_edge;
  /** When true `point_v2` is inside `coords_v2`. */
  bool inside;
} SDefBindPoly;

typedef struct SDefBindWeightData {
  SDefBindPoly *bind_polys;
  uint faces_num;
  uint binds_num;
} SDefBindWeightData;

typedef struct SDefAdjacency {
  struct SDefAdjacency *next;
  uint index;
} SDefAdjacency;

typedef struct SDefAdjacencyArray {
  SDefAdjacency *first;
  uint num; /* Careful, this is twice the number of polygons (avoids an extra loop) */
} SDefAdjacencyArray;

/**
 * Polygons per edge (only 2, any more will exit calculation).
 */
typedef struct SDefEdgePolys {
  uint polys[2], num;
} SDefEdgePolys;

typedef struct SDefBindCalcData {
  BVHTreeFromMesh *treeData;
  const SDefAdjacencyArray *vert_edges;
  const SDefEdgePolys *edge_polys;
  blender::Span<blender::int2> edges;
  blender::OffsetIndices<int> polys;
  blender::Span<int> corner_verts;
  blender::Span<int> corner_edges;
  blender::Span<blender::int3> corner_tris;
  blender::Span<int> tri_faces;
  bke::MutableAttributeAccessor *attributes;
  /** Coordinates to bind to, transformed into local space (compatible with `vertexCos`). */
  float (*targetCos)[3];
  /** Coordinates to bind (reference to the modifiers input argument).
   * Coordinates are stored in the grease pencil vert for grease pencil, which is accessed from the
   * stroke, also keeps the referenc from the deformstroke inut argument.
   */
  float imat[4][4];
  float falloff;
  int success;
  /** Vertex group lookup data. */
  const MDeformVert *const dvert;
  int stroke_idx;
  const float (*positions)[3];
  MutableSpan<SDefGPVert> vert_binds;
  int defgrp_index;
  bool invert_vgroup;
  bool sparse_bind;
} SDefBindCalcData;

static void freeBindData(SDefBindWeightData *const bwdata)
{
  SDefBindPoly *bpoly = bwdata->bind_polys;

  if (bwdata->bind_polys) {
    for (int i = 0; i < bwdata->faces_num; bpoly++, i++) {
      MEM_SAFE_FREE(bpoly->coords);
      MEM_SAFE_FREE(bpoly->coords_v2);
    }

    MEM_freeN(bwdata->bind_polys);
  }

  MEM_freeN(bwdata);
}
BLI_INLINE uint nearestVert(SDefBindCalcData *const data, const float point_co[3])
{
  BVHTreeNearest nearest{};
  nearest.dist_sq = FLT_MAX;
  nearest.index = -1;

  float t_point[3];
  float max_dist = FLT_MAX;
  float dist;
  uint index = 0;

  mul_v3_m4v3(t_point, data->imat, point_co);

  BLI_bvhtree_find_nearest(
      data->treeData->tree, t_point, &nearest, data->treeData->nearest_callback, data->treeData);

  const blender::IndexRange face = data->polys[data->tri_faces[nearest.index]];

  for (int i = 0; i < face.size(); i++) {
    const int edge_i = data->corner_edges[face.start() + i];
    const blender::int2 &edge = data->edges[edge_i];
    dist = dist_squared_to_line_segment_v3(
        point_co, data->targetCos[edge[0]], data->targetCos[edge[1]]);

    if (dist < max_dist) {
      max_dist = dist;
      index = edge_i;
    }
  }

  const blender::int2 &edge = data->edges[index];
  if (len_squared_v3v3(point_co, data->targetCos[edge[0]]) <
      len_squared_v3v3(point_co, data->targetCos[edge[1]]))
  {
    return edge[0];
  }

  return edge[1];
}

BLI_INLINE int isPolyValid(const float coords[][2], const uint nr)
{
  float prev_co[2], prev_prev_co[2];
  float curr_vec[2], prev_vec[2];

  if (!is_poly_convex_v2(coords, nr)) {
    return MOD_SDEF_BIND_RESULT_CONCAVE_ERR;
  }

  copy_v2_v2(prev_prev_co, coords[nr - 2]);
  copy_v2_v2(prev_co, coords[nr - 1]);
  sub_v2_v2v2(prev_vec, prev_co, coords[nr - 2]);
  normalize_v2(prev_vec);

  for (int i = 0; i < nr; i++) {
    sub_v2_v2v2(curr_vec, coords[i], prev_co);

    /* Check overlap between directly adjacent vertices. */
    const float curr_len = normalize_v2(curr_vec);
    if (curr_len < FLT_EPSILON) {
      return MOD_SDEF_BIND_RESULT_OVERLAP_ERR;
    }

    /* Check overlap between vertices skipping one. */
    if (len_squared_v2v2(prev_prev_co, coords[i]) < FLT_EPSILON * FLT_EPSILON) {
      return MOD_SDEF_BIND_RESULT_OVERLAP_ERR;
    }

    /* Check for adjacent parallel edges. */
    if (1.0f - dot_v2v2(prev_vec, curr_vec) < FLT_EPSILON) {
      return MOD_SDEF_BIND_RESULT_CONCAVE_ERR;
    }

    copy_v2_v2(prev_prev_co, prev_co);
    copy_v2_v2(prev_co, coords[i]);
    copy_v2_v2(prev_vec, curr_vec);
  }

  return MOD_SDEF_BIND_RESULT_SUCCESS;
}

BLI_INLINE float computeAngularWeight(const float point_angle, const float edgemid_angle)
{
  return sinf(min_ff(point_angle / edgemid_angle, 1) * M_PI_2);
}

BLI_INLINE SDefBindWeightData *computeBindWeights(SDefBindCalcData *const data,
                                                  const float point_co[3])
{
  const uint nearest = nearestVert(data, point_co);
  const SDefAdjacency *const vert_edges = data->vert_edges[nearest].first;
  const SDefEdgePolys *const edge_polys = data->edge_polys;

  const SDefAdjacency *vedge;

  SDefBindWeightData *bwdata;
  SDefBindPoly *bpoly;

  const float world[3] = {0.0f, 0.0f, 1.0f};
  float avg_point_dist = 0.0f;
  float tot_weight = 0.0f;
  int inf_weight_flags = 0;

  bwdata = static_cast<SDefBindWeightData *>(MEM_callocN(sizeof(*bwdata), "SDefBindWeightData"));
  if (bwdata == nullptr) {
    data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
    return nullptr;
  }

  bwdata->faces_num = data->vert_edges[nearest].num / 2;

  bpoly = static_cast<SDefBindPoly *>(
      MEM_calloc_arrayN(bwdata->faces_num, sizeof(*bpoly), "SDefBindPoly"));
  if (bpoly == nullptr) {
    freeBindData(bwdata);
    data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
    return nullptr;
  }

  bwdata->bind_polys = bpoly;

  /* Loop over all adjacent edges,
   * and build the #SDefBindPoly data for each face adjacent to those. */
  for (vedge = vert_edges; vedge; vedge = vedge->next) {
    uint edge_ind = vedge->index;

    for (int i = 0; i < edge_polys[edge_ind].num; i++) {
      {
        bpoly = bwdata->bind_polys;

        for (int j = 0; j < bwdata->faces_num; bpoly++, j++) {
          /* If coords isn't allocated, we have reached the first uninitialized `bpoly`. */
          if ((bpoly->index == edge_polys[edge_ind].polys[i]) || (!bpoly->coords)) {
            break;
          }
        }
      }

      /* Check if face was already created by another edge or still has to be initialized */
      if (!bpoly->coords) {
        float angle;
        float axis[3];
        float tmp_vec_v2[2];
        int is_poly_valid;

        bpoly->index = edge_polys[edge_ind].polys[i];
        bpoly->coords = nullptr;
        bpoly->coords_v2 = nullptr;

        /* Copy face data */
        const blender::IndexRange face = data->polys[bpoly->index];

        bpoly->verts_num = face.size();
        bpoly->loopstart = face.start();

        bpoly->coords = static_cast<float(*)[3]>(
            MEM_malloc_arrayN(face.size(), sizeof(*bpoly->coords), "SDefBindPolyCoords"));
        if (bpoly->coords == nullptr) {
          freeBindData(bwdata);
          data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
          return nullptr;
        }

        bpoly->coords_v2 = static_cast<float(*)[2]>(
            MEM_malloc_arrayN(face.size(), sizeof(*bpoly->coords_v2), "SDefBindPolyCoords_v2"));
        if (bpoly->coords_v2 == nullptr) {
          freeBindData(bwdata);
          data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
          return nullptr;
        }

        for (int j = 0; j < face.size(); j++) {
          const int vert_i = data->corner_verts[face.start() + j];
          const int edge_i = data->corner_edges[face.start() + j];
          copy_v3_v3(bpoly->coords[j], data->targetCos[vert_i]);

          /* Find corner and edge indices within face loop array */
          if (vert_i == nearest) {
            bpoly->corner_ind = j;
            bpoly->edge_vert_inds[0] = (j == 0) ? (face.size() - 1) : (j - 1);
            bpoly->edge_vert_inds[1] = (j == face.size() - 1) ? (0) : (j + 1);

            bpoly->edge_inds[0] = data->corner_edges[face.start() + bpoly->edge_vert_inds[0]];
            bpoly->edge_inds[1] = edge_i;
          }
        }

        /* Compute polygons parametric data. */
        mid_v3_v3_array(bpoly->centroid, bpoly->coords, face.size());
        normal_poly_v3(bpoly->normal, bpoly->coords, face.size());

        /* Compute face skew angle and axis */
        angle = angle_normalized_v3v3(bpoly->normal, world);

        cross_v3_v3v3(axis, bpoly->normal, world);
        normalize_v3(axis);

        /* Map coords onto 2d normal plane. */
        map_to_plane_axis_angle_v2_v3v3fl(bpoly->point_v2, point_co, axis, angle);

        zero_v2(bpoly->centroid_v2);
        for (int j = 0; j < face.size(); j++) {
          map_to_plane_axis_angle_v2_v3v3fl(bpoly->coords_v2[j], bpoly->coords[j], axis, angle);
          madd_v2_v2fl(bpoly->centroid_v2, bpoly->coords_v2[j], 1.0f / face.size());
        }

        is_poly_valid = isPolyValid(bpoly->coords_v2, face.size());

        if (is_poly_valid != MOD_SDEF_BIND_RESULT_SUCCESS) {
          freeBindData(bwdata);
          data->success = is_poly_valid;
          return nullptr;
        }

        bpoly->inside = isect_point_poly_v2(bpoly->point_v2, bpoly->coords_v2, face.size());

        /* Initialize weight components */
        bpoly->weight_angular = 1.0f;
        bpoly->weight_dist_proj = len_v2v2(bpoly->centroid_v2, bpoly->point_v2);
        bpoly->weight_dist = len_v3v3(bpoly->centroid, point_co);

        avg_point_dist += bpoly->weight_dist;

        /* Common vertex coordinates. */
        const float *const vert0_v2 = bpoly->coords_v2[bpoly->edge_vert_inds[0]];
        const float *const vert1_v2 = bpoly->coords_v2[bpoly->edge_vert_inds[1]];
        const float *const corner_v2 = bpoly->coords_v2[bpoly->corner_ind];

        /* Compute centroid to mid-edge vectors */
        mid_v2_v2v2(bpoly->cent_edgemid_vecs_v2[0], vert0_v2, corner_v2);
        mid_v2_v2v2(bpoly->cent_edgemid_vecs_v2[1], vert1_v2, corner_v2);

        sub_v2_v2(bpoly->cent_edgemid_vecs_v2[0], bpoly->centroid_v2);
        sub_v2_v2(bpoly->cent_edgemid_vecs_v2[1], bpoly->centroid_v2);

        normalize_v2(bpoly->cent_edgemid_vecs_v2[0]);
        normalize_v2(bpoly->cent_edgemid_vecs_v2[1]);

        /* Compute face scales with respect to the two edges. */
        bpoly->scales[0] = dist_to_line_v2(bpoly->centroid_v2, vert0_v2, corner_v2);
        bpoly->scales[1] = dist_to_line_v2(bpoly->centroid_v2, vert1_v2, corner_v2);

        /* Compute the angle between the edge mid vectors. */
        bpoly->edgemid_angle = angle_normalized_v2v2(bpoly->cent_edgemid_vecs_v2[0],
                                                     bpoly->cent_edgemid_vecs_v2[1]);

        /* Compute the angles between the corner and the edge mid vectors. The angles
         * are computed signed in order to correctly clamp point_edgemid_angles later. */
        float corner_angles[2];

        sub_v2_v2v2(tmp_vec_v2, corner_v2, bpoly->centroid_v2);
        normalize_v2(tmp_vec_v2);

        corner_angles[0] = angle_signed_v2v2(tmp_vec_v2, bpoly->cent_edgemid_vecs_v2[0]);
        corner_angles[1] = angle_signed_v2v2(tmp_vec_v2, bpoly->cent_edgemid_vecs_v2[1]);

        bpoly->corner_edgemid_angles[0] = fabsf(corner_angles[0]);
        bpoly->corner_edgemid_angles[1] = fabsf(corner_angles[1]);

        /* Verify that the computed values are valid (the face isn't somehow
         * degenerate despite having passed isPolyValid). */
        if (bpoly->scales[0] < FLT_EPSILON || bpoly->scales[1] < FLT_EPSILON ||
            bpoly->edgemid_angle < FLT_EPSILON || bpoly->corner_edgemid_angles[0] < FLT_EPSILON ||
            bpoly->corner_edgemid_angles[1] < FLT_EPSILON)
        {
          freeBindData(bwdata);
          data->success = MOD_SDEF_BIND_RESULT_GENERIC_ERR;
          return nullptr;
        }

        /* Check for infinite weights, and compute angular data otherwise. */
        if (bpoly->weight_dist < FLT_EPSILON) {
          inf_weight_flags |= MOD_SDEF_INFINITE_WEIGHT_DIST_PROJ;
          inf_weight_flags |= MOD_SDEF_INFINITE_WEIGHT_DIST;
        }
        else if (bpoly->weight_dist_proj < FLT_EPSILON) {
          inf_weight_flags |= MOD_SDEF_INFINITE_WEIGHT_DIST_PROJ;
        }
        else {
          /* Compute angles between the point and the edge mid vectors. */
          float cent_point_vec[2], point_angles[2];

          sub_v2_v2v2(cent_point_vec, bpoly->point_v2, bpoly->centroid_v2);
          normalize_v2(cent_point_vec);

          point_angles[0] = angle_signed_v2v2(cent_point_vec, bpoly->cent_edgemid_vecs_v2[0]) *
                            signf(corner_angles[0]);
          point_angles[1] = angle_signed_v2v2(cent_point_vec, bpoly->cent_edgemid_vecs_v2[1]) *
                            signf(corner_angles[1]);

          if (point_angles[0] <= 0 && point_angles[1] <= 0) {
            /* If the point is outside the corner formed by the edge mid vectors,
             * choose to clamp the closest side and flip the other. */
            if (point_angles[0] < point_angles[1]) {
              point_angles[0] = bpoly->edgemid_angle - point_angles[1];
            }
            else {
              point_angles[1] = bpoly->edgemid_angle - point_angles[0];
            }
          }

          bpoly->point_edgemid_angles[0] = max_ff(0, point_angles[0]);
          bpoly->point_edgemid_angles[1] = max_ff(0, point_angles[1]);

          /* Compute the distance scale for the corner. The base value is the orthogonal
           * distance from the corner to the chord, scaled by `sqrt(2)` to preserve the old
           * values in case of a square grid. This doesn't use the centroid because the
           * corner_triS method only uses these three vertices. */
          bpoly->scale_mid = area_tri_v2(vert0_v2, corner_v2, vert1_v2) /
                             len_v2v2(vert0_v2, vert1_v2) * sqrtf(2);

          if (bpoly->inside) {
            /* When inside, interpolate to centroid-based scale close to the center. */
            float min_dist = min_ff(bpoly->scales[0], bpoly->scales[1]);

            bpoly->scale_mid = interpf(bpoly->scale_mid,
                                       (bpoly->scales[0] + bpoly->scales[1]) / 2,
                                       min_ff(bpoly->weight_dist_proj / min_dist, 1));
          }

          /* Verify that the additional computed values are valid. */
          if (bpoly->scale_mid < FLT_EPSILON ||
              bpoly->point_edgemid_angles[0] + bpoly->point_edgemid_angles[1] < FLT_EPSILON)
          {
            freeBindData(bwdata);
            data->success = MOD_SDEF_BIND_RESULT_GENERIC_ERR;
            return nullptr;
          }
        }
      }
    }
  }

  avg_point_dist /= bwdata->faces_num;

  /* If weights 1 and 2 are not infinite, loop over all adjacent edges again,
   * and build adjacency dependent angle data (depends on all polygons having been computed) */
  if (!inf_weight_flags) {
    for (vedge = vert_edges; vedge; vedge = vedge->next) {
      SDefBindPoly *bpolys[2];
      const SDefEdgePolys *epolys;
      float ang_weights[2];
      uint edge_ind = vedge->index;
      uint edge_on_poly[2];

      epolys = &edge_polys[edge_ind];

      /* Find bind polys corresponding to the edge's adjacent polys */
      bpoly = bwdata->bind_polys;

      for (int i = 0, j = 0; (i < bwdata->faces_num) && (j < epolys->num); bpoly++, i++) {
        if (ELEM(bpoly->index, epolys->polys[0], epolys->polys[1])) {
          bpolys[j] = bpoly;

          if (bpoly->edge_inds[0] == edge_ind) {
            edge_on_poly[j] = 0;
          }
          else {
            edge_on_poly[j] = 1;
          }

          j++;
        }
      }

      /* Compute angular weight component */
      if (epolys->num == 1) {
        ang_weights[0] = computeAngularWeight(bpolys[0]->point_edgemid_angles[edge_on_poly[0]],
                                              bpolys[0]->edgemid_angle);
        bpolys[0]->weight_angular *= ang_weights[0] * ang_weights[0];
      }
      else if (epolys->num == 2) {
        ang_weights[0] = computeAngularWeight(bpolys[0]->point_edgemid_angles[edge_on_poly[0]],
                                              bpolys[0]->edgemid_angle);
        ang_weights[1] = computeAngularWeight(bpolys[1]->point_edgemid_angles[edge_on_poly[1]],
                                              bpolys[1]->edgemid_angle);

        bpolys[0]->weight_angular *= ang_weights[0] * ang_weights[1];
        bpolys[1]->weight_angular *= ang_weights[0] * ang_weights[1];
      }
    }
  }

  /* Compute scaling and falloff:
   * - Scale all weights if no infinite weight is found.
   * - Scale only un-projected weight if projected weight is infinite.
   * - Scale none if both are infinite. */
  if (!inf_weight_flags) {
    bpoly = bwdata->bind_polys;

    for (int i = 0; i < bwdata->faces_num; bpoly++, i++) {
      float corner_angle_weights[2];
      float scale_weight, sqr, inv_sqr;

      corner_angle_weights[0] = bpoly->point_edgemid_angles[0] / bpoly->corner_edgemid_angles[0];
      corner_angle_weights[1] = bpoly->point_edgemid_angles[1] / bpoly->corner_edgemid_angles[1];

      if (isnan(corner_angle_weights[0]) || isnan(corner_angle_weights[1])) {
        freeBindData(bwdata);
        data->success = MOD_SDEF_BIND_RESULT_GENERIC_ERR;
        return nullptr;
      }

      /* Find which edge the point is closer to */
      if (corner_angle_weights[0] < corner_angle_weights[1]) {
        bpoly->dominant_edge = 0;
        bpoly->dominant_angle_weight = corner_angle_weights[0];
      }
      else {
        bpoly->dominant_edge = 1;
        bpoly->dominant_angle_weight = corner_angle_weights[1];
      }

      /* Check for invalid weights just in case computations fail. */
      if (bpoly->dominant_angle_weight < 0 || bpoly->dominant_angle_weight > 1) {
        freeBindData(bwdata);
        data->success = MOD_SDEF_BIND_RESULT_GENERIC_ERR;
        return nullptr;
      }

      bpoly->dominant_angle_weight = sinf(bpoly->dominant_angle_weight * M_PI_2);

      /* Compute quadratic angular scale interpolation weight */
      {
        const float edge_angle_a = bpoly->point_edgemid_angles[bpoly->dominant_edge];
        const float edge_angle_b = bpoly->point_edgemid_angles[!bpoly->dominant_edge];
        /* Clamp so skinny faces with near zero `edgemid_angle`
         * won't cause numeric problems. see #81988. */
        scale_weight = edge_angle_a / max_ff(edge_angle_a, bpoly->edgemid_angle);
        scale_weight /= scale_weight + (edge_angle_b / max_ff(edge_angle_b, bpoly->edgemid_angle));
      }

      sqr = scale_weight * scale_weight;
      inv_sqr = 1.0f - scale_weight;
      inv_sqr *= inv_sqr;
      scale_weight = sqr / (sqr + inv_sqr);

      BLI_assert(scale_weight >= 0 && scale_weight <= 1);

      /* Compute interpolated scale (no longer need the individual scales,
       * so simply storing the result over the scale in index zero) */
      bpoly->scales[0] = interpf(bpoly->scale_mid,
                                 interpf(bpoly->scales[!bpoly->dominant_edge],
                                         bpoly->scales[bpoly->dominant_edge],
                                         scale_weight),
                                 bpoly->dominant_angle_weight);

      /* Scale the point distance weights, and introduce falloff */
      bpoly->weight_dist_proj /= bpoly->scales[0];
      bpoly->weight_dist_proj = powf(bpoly->weight_dist_proj, data->falloff);

      bpoly->weight_dist /= avg_point_dist;
      bpoly->weight_dist = powf(bpoly->weight_dist, data->falloff);

      /* Re-check for infinite weights, now that all scalings and interpolations are computed */
      if (bpoly->weight_dist < FLT_EPSILON) {
        inf_weight_flags |= MOD_SDEF_INFINITE_WEIGHT_DIST_PROJ;
        inf_weight_flags |= MOD_SDEF_INFINITE_WEIGHT_DIST;
      }
      else if (bpoly->weight_dist_proj < FLT_EPSILON) {
        inf_weight_flags |= MOD_SDEF_INFINITE_WEIGHT_DIST_PROJ;
      }
      else if (bpoly->weight_angular < FLT_EPSILON) {
        inf_weight_flags |= MOD_SDEF_INFINITE_WEIGHT_ANGULAR;
      }
    }
  }
  else if (!(inf_weight_flags & MOD_SDEF_INFINITE_WEIGHT_DIST)) {
    bpoly = bwdata->bind_polys;

    for (int i = 0; i < bwdata->faces_num; bpoly++, i++) {
      /* Scale the point distance weight by average point distance, and introduce falloff */
      bpoly->weight_dist /= avg_point_dist;
      bpoly->weight_dist = powf(bpoly->weight_dist, data->falloff);

      /* Re-check for infinite weights, now that all scalings and interpolations are computed */
      if (bpoly->weight_dist < FLT_EPSILON) {
        inf_weight_flags |= MOD_SDEF_INFINITE_WEIGHT_DIST;
      }
    }
  }

  /* Final loop, to compute actual weights */
  bpoly = bwdata->bind_polys;

  for (int i = 0; i < bwdata->faces_num; bpoly++, i++) {
    /* Weight computation from components */
    if (inf_weight_flags & MOD_SDEF_INFINITE_WEIGHT_DIST) {
      bpoly->weight = bpoly->weight_dist < FLT_EPSILON ? 1.0f : 0.0f;
    }
    else if (inf_weight_flags & MOD_SDEF_INFINITE_WEIGHT_DIST_PROJ) {
      bpoly->weight = bpoly->weight_dist_proj < FLT_EPSILON ? 1.0f / bpoly->weight_dist : 0.0f;
    }
    else if (inf_weight_flags & MOD_SDEF_INFINITE_WEIGHT_ANGULAR) {
      bpoly->weight = bpoly->weight_angular < FLT_EPSILON ?
                          1.0f / bpoly->weight_dist_proj / bpoly->weight_dist :
                          0.0f;
    }
    else {
      bpoly->weight = 1.0f / bpoly->weight_angular / bpoly->weight_dist_proj / bpoly->weight_dist;
    }

    /* Apply after other kinds of scaling so the faces corner angle is always
     * scaled in a uniform way, preventing heavily sub-divided triangle fans
     * from having a lop-sided influence on the weighting, see #81988. */
    bpoly->weight *= bpoly->edgemid_angle / M_PI;

    tot_weight += bpoly->weight;
  }

  bpoly = bwdata->bind_polys;

  for (int i = 0; i < bwdata->faces_num; bpoly++, i++) {
    bpoly->weight /= tot_weight;

    /* Evaluate if this face is relevant to bind */
    /* Even though the weights should add up to 1.0,
     * the losses of weights smaller than epsilon here
     * should be negligible... */
    if (bpoly->weight >= FLT_EPSILON) {
      if (bpoly->inside) {
        bwdata->binds_num += 1;
      }
      else {
        if (bpoly->dominant_angle_weight < FLT_EPSILON ||
            1.0f - bpoly->dominant_angle_weight < FLT_EPSILON)
        {
          bwdata->binds_num += 1;
        }
        else {
          bwdata->binds_num += 2;
        }
      }
    }
  }

  return bwdata;
}
BLI_INLINE float computeNormalDisplacement(const float point_co[3],
                                           const float point_co_proj[3],
                                           const float normal[3])
{
  float disp_vec[3];
  float normal_dist;

  sub_v3_v3v3(disp_vec, point_co, point_co_proj);
  normal_dist = len_v3(disp_vec);

  if (dot_v3v3(disp_vec, normal) < 0) {
    normal_dist *= -1;
  }

  return normal_dist;
}


static void freeAdjacencyMap(SDefAdjacencyArray *const vert_edges,
                             SDefAdjacency *const adj_ref,
                             SDefEdgePolys *const edge_polys)
{
  MEM_freeN(edge_polys);

  MEM_freeN(adj_ref);

  MEM_freeN(vert_edges);
}

static int buildAdjacencyMap(const blender::OffsetIndices<int> polys,
                             const blender::Span<blender::int2> edges,
                             const blender::Span<int> corner_edges,
                             SDefAdjacencyArray *const vert_edges,
                             SDefAdjacency *adj,
                             SDefEdgePolys *const edge_polys)
{
  /* Find polygons adjacent to edges. */
  for (const int i : polys.index_range()) {
    for (const int edge_i : corner_edges.slice(polys[i])) {
      if (edge_polys[edge_i].num == 0) {
        edge_polys[edge_i].polys[0] = i;
        edge_polys[edge_i].polys[1] = -1;
        edge_polys[edge_i].num++;
      }
      else if (edge_polys[edge_i].num == 1) {
        edge_polys[edge_i].polys[1] = i;
        edge_polys[edge_i].num++;
      }
      else {
        return MOD_SDEF_BIND_RESULT_NONMANY_ERR;
      }
    }
  }

  /* Find edges adjacent to vertices */
  for (const int i : edges.index_range()) {
    const blender::int2 &edge = edges[i];
    adj->next = vert_edges[edge[0]].first;
    adj->index = i;
    vert_edges[edge[0]].first = adj;
    vert_edges[edge[0]].num += edge_polys[i].num;
    adj++;

    adj->next = vert_edges[edge[1]].first;
    adj->index = i;
    vert_edges[edge[1]].first = adj;
    vert_edges[edge[1]].num += edge_polys[i].num;
    adj++;
  }

  return MOD_SDEF_BIND_RESULT_SUCCESS;
}

BLI_INLINE void sortPolyVertsEdge(uint *indices,
                                  const int *const corner_verts,
                                  const int *const corner_edges,
                                  const uint edge,
                                  const uint num)
{
  bool found = false;

  for (int i = 0; i < num; i++) {
    if (corner_edges[i] == edge) {
      found = true;
    }
    if (found) {
      *indices = corner_verts[i];
      indices++;
    }
  }

  /* Fill in remaining vertex indices that occur before the edge */
  for (int i = 0; corner_edges[i] != edge; i++) {
    *indices = corner_verts[i];
    indices++;
  }
}

BLI_INLINE void sortPolyVertsTri(uint *indices,
                                 const int *const corner_verts,
                                 const uint loopstart,
                                 const uint num)
{
  for (int i = loopstart; i < num; i++) {
    *indices = corner_verts[i];
    indices++;
  }

  for (int i = 0; i < loopstart; i++) {
    *indices = corner_verts[i];
    indices++;
  }
}


static bool makeTreeData(Object *ob,
                        BVHTreeFromMesh *treeData,
                         Mesh *target,
                         GPencilSurDeformModifierData *smd_eval,
                         SDefAdjacencyArray *vert_edges,
                         SDefAdjacency *adj_array,
                         SDefEdgePolys *edge_polys,
                         uint target_verts_num)
{
  int adj_result;

  const blender::Span<blender::int2> edges = target->edges();
  const blender::OffsetIndices polys = target->faces();
  const blender::Span<int> corner_edges = target->corner_edges();
  uint tedges_num = target->edges_num;
  /*Empty edge polys array*/
  for (int ep = 0; ep < tedges_num; ep++) {

    edge_polys[ep].polys[0] = 0;
    edge_polys[ep].polys[1] = 0;
    edge_polys[ep].num = 0;
  }

  /*Empty vert edges array*/
  for (int ve = 0; ve < target_verts_num; ve++) {
    vert_edges[ve].first = NULL;
    vert_edges[ve].num = 0;
  }
  BKE_bvhtree_from_mesh_get(treeData, target, BVHTREE_FROM_CORNER_TRIS, 2);
  if (treeData->tree == NULL) {
    BKE_modifier_set_error(ob, (ModifierData *)smd_eval, "Out of memory");
    // freeAdjacencyMap(vert_edges, adj_array, edge_polys);
    return false;
  }

  // printf("mPoly: %p, mpoly->totloop: %d", mpoly, mpoly->totloop);

  adj_result = buildAdjacencyMap(polys, edges, corner_edges, vert_edges, adj_array, edge_polys);

  if (adj_result == MOD_SDEF_BIND_RESULT_NONMANY_ERR) {
    BKE_modifier_set_error(
        ob, (ModifierData *)smd_eval,
                                   "Target has edges with more than two polygons");
    // freeAdjacencyMap(vert_edges, adj_array, edge_polys);
    // free_bvhtree_from_mesh(&treeData);
    return false;
  }
  return true;
}



static void bindVert(void *__restrict userdata,
                     const int index,
                     const TaskParallelTLS *__restrict /*tls*/)
{
  SDefBindCalcData *const data = (SDefBindCalcData *)userdata;
  float point_co[3];
  float point_co_proj[3];

  SDefBindWeightData *bwdata;
  SDefBindPoly *bpoly;
  SDefGPBind *sdbind;


  if (data->success != MOD_SDEF_BIND_RESULT_SUCCESS) {
    // TODO empty binds 
    return;
  }

  copy_v3_v3(point_co, data->positions[index]);
  bwdata = computeBindWeights(data, point_co);

  if (bwdata == NULL) {
    data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
    freeBindData(bwdata);
    return;
  }
  SDefGPBind *binds = static_cast<SDefGPBind *>(
      MEM_calloc_arrayN(bwdata->binds_num, sizeof(SDefGPBind), "SDefVertBindData"));
  if (binds == NULL) {
    data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
    freeBindData(bwdata);
    return;
  }
  
 

  sdbind = binds;

  bpoly = bwdata->bind_polys;

  //for (SDefBindPoly bpoly : binds_span) {
  for (int i = 0; i < bwdata->binds_num; bpoly++) {
    if (bpoly->weight >= FLT_EPSILON) {
      if (bpoly->inside) {
        sdbind->influence = bpoly->weight;
        sdbind->verts_num = bpoly->verts_num;

        sdbind->mode = GP_MOD_SDEF_MODE_NGON;
        sdbind->vert_weights = static_cast<float *>(MEM_malloc_arrayN(
            bpoly->verts_num, sizeof(*sdbind->vert_weights), "SDefNgonVertWeights"));
        if (sdbind->vert_weights == nullptr) {
          data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
          return;
        }

        sdbind->vert_inds = static_cast<uint *>(
            MEM_malloc_arrayN(bpoly->verts_num, sizeof(*sdbind->vert_inds), "SDefNgonVertInds"));
        if (sdbind->vert_inds == nullptr) {
          data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
          return;
        }

        interp_weights_poly_v2(
            sdbind->vert_weights, bpoly->coords_v2, bpoly->verts_num, bpoly->point_v2);

        /* Re-project vert based on weights and original poly verts,
         * to reintroduce poly non-planarity */
        zero_v3(point_co_proj);
        for (int j = 0; j < bpoly->verts_num; j++) {
          const int vert_i = data->corner_verts[bpoly->loopstart + j];
          madd_v3_v3fl(point_co_proj, bpoly->coords[j], sdbind->vert_weights[j]);
          sdbind->vert_inds[j] = vert_i;
        }

        sdbind->normal_dist = computeNormalDisplacement(point_co, point_co_proj, bpoly->normal);

        sdbind++;
        i++;
      }
      else {
        float tmp_vec[3];
        float cent[3], norm[3];
        float v1[3], v2[3], v3[3];

        if (1.0f - bpoly->dominant_angle_weight >= FLT_EPSILON) {
          sdbind->influence = bpoly->weight * (1.0f - bpoly->dominant_angle_weight);
          sdbind->verts_num = bpoly->verts_num;

          sdbind->mode = GP_MOD_SDEF_MODE_CENTROID;
          sdbind->vert_weights = static_cast<float *>(
              MEM_malloc_arrayN(3, sizeof(*sdbind->vert_weights), "SDefCentVertWeights"));
          if (sdbind->vert_weights == nullptr) {
            data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
            return;
          }

          sdbind->vert_inds = static_cast<uint *>(
              MEM_malloc_arrayN(bpoly->verts_num, sizeof(*sdbind->vert_inds), "SDefCentVertInds"));
          if (sdbind->vert_inds == nullptr) {
            data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
            return;
          }

          sortPolyVertsEdge(sdbind->vert_inds,
                            &data->corner_verts[bpoly->loopstart],
                            &data->corner_edges[bpoly->loopstart],
                            bpoly->edge_inds[bpoly->dominant_edge],
                            bpoly->verts_num);

          copy_v3_v3(v1, data->targetCos[sdbind->vert_inds[0]]);
          copy_v3_v3(v2, data->targetCos[sdbind->vert_inds[1]]);
          copy_v3_v3(v3, bpoly->centroid);

          mid_v3_v3v3v3(cent, v1, v2, v3);
          normal_tri_v3(norm, v1, v2, v3);

          add_v3_v3v3(tmp_vec, point_co, bpoly->normal);

          /* We are sure the line is not parallel to the plane.
           * Checking return value just to avoid warning... */
          if (!isect_line_plane_v3(point_co_proj, point_co, tmp_vec, cent, norm)) {
            BLI_assert(false);
          }

          interp_weights_tri_v3(sdbind->vert_weights, v1, v2, v3, point_co_proj);

          sdbind->normal_dist = computeNormalDisplacement(point_co, point_co_proj, bpoly->normal);

          sdbind++;
          i++;
        }

        if (bpoly->dominant_angle_weight >= FLT_EPSILON) {
          sdbind->influence = bpoly->weight * bpoly->dominant_angle_weight;
          sdbind->verts_num = bpoly->verts_num;

          sdbind->mode = GP_MOD_SDEF_MODE_LOOPTRI;
          sdbind->vert_weights = static_cast<float *>(
              MEM_malloc_arrayN(3, sizeof(*sdbind->vert_weights), "SDefTriVertWeights"));
          if (sdbind->vert_weights == nullptr) {
            data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
            return;
          }

          sdbind->vert_inds = static_cast<uint *>(
              MEM_malloc_arrayN(bpoly->verts_num, sizeof(*sdbind->vert_inds), "SDefTriVertInds"));
          if (sdbind->vert_inds == nullptr) {
            data->success = MOD_SDEF_BIND_RESULT_MEM_ERR;
            return;
          }

          sortPolyVertsTri(sdbind->vert_inds,
                           &data->corner_verts[bpoly->loopstart],
                           bpoly->edge_vert_inds[0],
                           bpoly->verts_num);

          copy_v3_v3(v1, data->targetCos[sdbind->vert_inds[0]]);
          copy_v3_v3(v2, data->targetCos[sdbind->vert_inds[1]]);
          copy_v3_v3(v3, data->targetCos[sdbind->vert_inds[2]]);

          mid_v3_v3v3v3(cent, v1, v2, v3);
          normal_tri_v3(norm, v1, v2, v3);

          add_v3_v3v3(tmp_vec, point_co, bpoly->normal);

          /* We are sure the line is not parallel to the plane.
           * Checking return value just to avoid warning... */
          if (!isect_line_plane_v3(point_co_proj, point_co, tmp_vec, cent, norm)) {
            BLI_assert(false);
          }

          interp_weights_tri_v3(sdbind->vert_weights, v1, v2, v3, point_co_proj);

          sdbind->normal_dist = computeNormalDisplacement(point_co, point_co_proj, bpoly->normal);

          sdbind++;
          i++;
        }
      }
    }
  }

 sdbind = binds;
  
  bke::MutableAttributeAccessor *attributes = data->attributes;

  // binds_num
  bke::AttributeWriter<int> binds_num_attr_writer = attributes->lookup_or_add_for_write<int>(
      "sdef_binds_num", bke::AttrDomain::Point);
  binds_num_attr_writer.varray.set(index, bwdata->binds_num);
  binds_num_attr_writer.finish();

  std::string index_str;
  // Add attributes for each bind
  for (int i = 0; i < bwdata->binds_num; i++) {
    index_str = std::to_string(i);

    // vert_inds
    for (int vi = 0; vi < sdbind->verts_num; vi++) {
      std::string vi_str = std::to_string(vi);
      bke::AttributeWriter<int> vert_inds_attr_writer = attributes->lookup_or_add_for_write<int>(
          "sdef_vert_inds_bind" + index_str + "_" + vi_str, bke::AttrDomain::Point);
      vert_inds_attr_writer.varray.set(index, static_cast<int>(sdbind->vert_inds[vi]));
      vert_inds_attr_writer.finish();
    }
    MEM_SAFE_FREE(sdbind->vert_inds);

    // verts_num
    bke::AttributeWriter<int> verts_num_attr_writer = attributes->lookup_or_add_for_write<int>(
        "sdef_verts_num_bind" + index_str, bke::AttrDomain::Point);
    verts_num_attr_writer.varray.set(index, sdbind->verts_num);
    verts_num_attr_writer.finish();

    // mode
    bke::AttributeWriter<int> mode_attr_writer = attributes->lookup_or_add_for_write<int>(
        "sdef_mode_bind" + index_str, bke::AttrDomain::Point);
    mode_attr_writer.varray.set(index, sdbind->mode);
    mode_attr_writer.finish();

    // vert_weights
    
    for (int vi = 0; vi < sdbind->verts_num; vi++) {
      std::string vi_str = std::to_string(vi);
      bke::AttributeWriter<float> vert_weights_attr_writer =
          attributes->lookup_or_add_for_write<float>(
              "sdef_vert_weights_bind" + index_str + "_" + vi_str,
                                                      bke::AttrDomain::Point);
      vert_weights_attr_writer.varray.set(index, sdbind->vert_weights[vi]);
      vert_weights_attr_writer.finish();
    }
    MEM_SAFE_FREE(sdbind->vert_weights);
    

    // normal_dist
    bke::AttributeWriter<float> normal_dist_attr_writer =
        attributes->lookup_or_add_for_write<float>("sdef_normal_dist_bind" + index_str,
                                                  bke::AttrDomain::Point);
    normal_dist_attr_writer.varray.set(index, sdbind->normal_dist);
    normal_dist_attr_writer.finish();

    // influence
    bke::AttributeWriter<float> influence_attr_writer = attributes->lookup_or_add_for_write<float>(
        "sdef_influence_bind" + index_str, bke::AttrDomain::Point);
    influence_attr_writer.varray.set(index, sdbind->influence);
    influence_attr_writer.finish();

    sdbind++;
  }

  // free arrays
  MEM_SAFE_FREE(binds);
  freeBindData(bwdata);
}

bool unbind_drawing(ModifierData *md_eval,
                  Object *ob,
                  bke::greasepencil::Drawing *drawing,
                  Mesh *target)
{
  using namespace bke;
  MutableAttributeAccessor attributes = drawing->strokes_for_write().attributes_for_write();
  
  Vector<std::string> to_remove;
  attributes.foreach_attribute([&](const bke::AttributeIter &iter) {
    if (iter.name.startswith("sdef_")) {
      to_remove.append(iter.name);
    }
    
  });

  for (const std::string &iter : to_remove) {
    attributes.remove(iter);
  }

  return true;
  
}

static bool bind_drawing(ModifierData *md_eval,
                         Object *ob,
                         bke::greasepencil::Drawing *drawing,
                         Mesh *target)
{
  auto smd_eval = reinterpret_cast<GPencilSurDeformModifierData *>(md_eval);
  GPencilSurDeformModifierData *smd_orig = (GPencilSurDeformModifierData *)
      BKE_modifier_get_original(ob, md_eval);
  bke::CurvesGeometry &curves = drawing->strokes_for_write();
  const OffsetIndices<int> points_by_curve = curves.points_by_curve();
  const Span<MDeformVert> dverts = curves.deform_verts();
  const MutableSpan<float3> positions = curves.positions_for_write();
  uint verts_num = curves.points_num();
  if (verts_num < 1)
    return 0;

  SDefAdjacencyArray *vert_edges;
  SDefAdjacency *adj_array;
  SDefEdgePolys *edge_polys;

  uint tedges_num = target->edges_num;
  uint target_verts_num = BKE_mesh_wrapper_vert_len(target);
  uint target_polys_num = BKE_mesh_wrapper_face_len(target);
  const blender::Span<int> corner_verts = target->corner_verts();
  const blender::Span<int> corner_edges = target->corner_edges();

  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
  
 
  
  
    // Allocate / Reallocate verts.
  SDefGPVert *verts;
  if (smd_orig->verts_array == nullptr) {
    verts = static_cast<SDefGPVert *>(
        MEM_calloc_arrayN(verts_num, sizeof(SDefGPVert), "surdef GP verts"));
    for (int i = 0; i < verts_num; i++)
      verts[i].vertex_idx = -1;
     smd_orig->verts_array_tot = verts_num;
    smd_orig->verts_array_occupied = verts_num;
  }
  else
  {
    int n_of_verts_total = smd_orig->verts_array_occupied + verts_num;
    int n_of_verts_extra_allocated = n_of_verts_total - smd_orig->verts_array_tot;
    verts = static_cast<SDefGPVert *>(
        MEM_calloc_arrayN(n_of_verts_total, sizeof(SDefGPVert), "surdef GP verts"));
    memcpy(verts, smd_orig->verts_array, sizeof(SDefGPVert) * smd_orig->verts_array_tot);
    MEM_SAFE_FREE(smd_orig->verts_array);
    for (int i = 0; i < n_of_verts_extra_allocated; i++)
      verts[smd_orig->verts_array_tot + i].vertex_idx = -1;
    smd_orig->verts_array_tot = n_of_verts_total;
  }
  smd_orig->verts_array = verts;
  

  
  // TODO mask vertices by frame, mask, vertex group etc.*/
  

  vert_edges = static_cast<SDefAdjacencyArray *>(
      MEM_calloc_arrayN(target_verts_num, sizeof(*vert_edges), "SDefVertEdgeMap"));
  if (vert_edges == NULL) {
    BKE_modifier_set_error(ob, (ModifierData *)smd_eval, "Out of memory");
    return false;
  }

  adj_array = static_cast<SDefAdjacency *>(
      MEM_malloc_arrayN(tedges_num, 2 * sizeof(*adj_array), "SDefVertEdge"));
  if (adj_array == NULL) {
    BKE_modifier_set_error(ob, (ModifierData *)smd_eval, "Out of memory");
    MEM_freeN(vert_edges);
    return false;
  }

  edge_polys = static_cast<SDefEdgePolys *>(
      MEM_calloc_arrayN(tedges_num, sizeof(*edge_polys), "SDefEdgeFaceMap"));
  if (edge_polys == NULL) {
    BKE_modifier_set_error(ob, (ModifierData *)smd_eval, "Out of memory");
    MEM_freeN(vert_edges);
    MEM_freeN(adj_array);
    return false;
  }

  BVHTreeFromMesh treeData = {NULL};
  if (!makeTreeData(
          ob, &treeData,
                    target,
                    smd_eval,
                    vert_edges,
                    adj_array,
                    edge_polys,
                    target_verts_num))
    return false;

  SDefBindCalcData data{};
  data.treeData = &treeData;
  data.vert_edges = vert_edges;
  data.edge_polys = edge_polys;
  data.polys = target->faces();
  data.edges = target->edges();
  data.corner_verts = corner_verts;
  data.corner_edges = corner_edges;
  data.corner_tris = target->corner_tris();
  data.tri_faces = target->corner_tri_faces();
  data.targetCos = static_cast<float(*)[3]>(
      MEM_malloc_arrayN(target_verts_num, sizeof(float[3]), "SDefTargetBindVertArray"));
  data.positions = reinterpret_cast<float(*)[3]>(positions.data());
  data.vert_binds = smd_orig->verts();
  data.falloff = smd_orig->falloff;
  data.success = MOD_SDEF_BIND_RESULT_SUCCESS;
  data.attributes = &attributes;
  /*.dvert = dvert,
  .defgrp_index = defgrp_index,
  .invert_vgroup = invert_vgroup,
  .sparse_bind = sparse_bind,*/

  if (data.targetCos == NULL) {
    BKE_modifier_set_error(ob, md_eval, "Out of memory");
    BKE_modifier_get_info(static_cast<ModifierType>(md_eval->type))
        ->free_data((ModifierData *)smd_orig);
    return false;
  }

  invert_m4_m4(data.imat, smd_orig->mat);
  const blender::Span<blender::float3> mesh_positions = target->vert_positions();
  for (int i = 0; i < target_verts_num; i++) {
    mul_v3_m4v3(data.targetCos[i], smd_orig->mat, mesh_positions[i]);
  }

  TaskParallelSettings settings;
  BLI_parallel_range_settings_defaults(&settings);
  settings.use_threading = (verts_num > 10000);
  BLI_task_parallel_range(0, verts_num, &data, bindVert, &settings);
  
  MEM_freeN(data.targetCos);
  return true;
}

static void free_drawing_bind_data(ModifierData *md_orig, bke::greasepencil::Drawing *drawing)
{
  auto smd_orig = reinterpret_cast<GPencilSurDeformModifierData *>(md_orig);
  bke::CurvesGeometry &curves = drawing->strokes_for_write();
  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
  bke::SpanAttributeWriter<int> sdef_vert_idx_attr_writer =
      attributes.lookup_or_add_for_write_only_span<int>("sdef_vert_idx", bke::AttrDomain::Point);
  MutableSpan<int> sdef_vert_idx_span = sdef_vert_idx_attr_writer.span;
  for (int vert_binds_idx : sdef_vert_idx_span) {
    const SDefGPBind *databind = smd_orig->verts_array[vert_binds_idx].binds_array;
    MEM_SAFE_FREE(databind);
    smd_orig->verts_array[vert_binds_idx].vertex_idx = -1;
    smd_orig->verts_array_occupied--;
  }
  sdef_vert_idx_attr_writer.finish();

}

/*End binding*/
static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                bke::GeometrySet *geometry_set)
{
  using bke::greasepencil::Drawing;
  using bke::greasepencil::Layer;
  using modifier::greasepencil::LayerDrawingInfo;

  auto &smd = *reinterpret_cast<GPencilSurDeformModifierData *>(md);
  BLI_assert(smd.target != nullptr);

  GPencilSurDeformModifierData *smd_orig = reinterpret_cast<GPencilSurDeformModifierData *>(
      BKE_modifier_get_original(ctx->object, md));
  ModifierData *md_orig = (ModifierData *)smd_orig;
  
  if (!geometry_set->has_grease_pencil()) {
    return;
  }
  GreasePencil &grease_pencil = *geometry_set->get_grease_pencil_for_write();
  const int frame = grease_pencil.runtime->eval_frame;

  // TODO LAYER INFLUENCE PANEL
  IndexMaskMemory mask_memory;
  const IndexMask layer_mask = modifier::greasepencil::get_filtered_layer_mask(
      grease_pencil, smd.influence, mask_memory);

  Mesh *tg_mesh = reinterpret_cast<Mesh *>(smd_orig->target->data);
  const Vector<Drawing *> drawings = modifier::greasepencil::get_drawings_for_write(
      grease_pencil, layer_mask,  frame);
  threading::parallel_for_each(drawings, [&](Drawing *drawing) {
    surfacedeform_deform(md, ctx->depsgraph, *drawing, smd_orig->target);
  });
}


static void bake_modifier(
                         Depsgraph *depsgraph,
                         ModifierData *md,
                         Object *ob)
{
  return;  // generic_bake_deform_stroke(depsgraph, md, ob, false, deformStroke);
}

/* uses original modifer */
static bool is_disabled(const Scene *scene, ModifierData *md, bool use_render_params)
{
  GPencilSurDeformModifierData *smd = (GPencilSurDeformModifierData *)md;

  /* The object type check is only needed here in case we have a placeholder
   * object assigned (because the library containing the mesh is missing).
   *
   * In other cases it should be impossible to have a type mismatch.
   */
  return (smd->target == NULL || smd->target->type != OB_MESH) &&
         (smd->bound_flags == 0);
}


static void panel_draw(const bContext * /*C*/, Panel *panel)
{
  uiLayout *sub, *row, *col;
  uiLayout *layout = panel->layout;
  

  PointerRNA op_ptr_all;
  PointerRNA op_ptr_curr;
  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);

  PointerRNA target_ptr = RNA_pointer_get(ptr, "target");
  ModifierData *md = (ModifierData *)ptr->data;
  GPencilSurDeformModifierData *smd = (GPencilSurDeformModifierData *)md;

  // bool unbind_mode = RNA_boolean_get(ptr, "unbind_mode");
  bool bind_all_frames = (RNA_enum_get(ptr, "curr_frame_or_all_frames") ==
                          GP_MOD_SDEF_BIND_ALL_FRAMES);

  uiLayoutSetPropSep(layout, true);

  col = uiLayoutColumn(layout, false);
 // uiLayoutSetActive(col, !smd->layers);
  uiItemR(col, ptr, "target", UI_ITEM_NONE, NULL, ICON_NONE);  // TODO: disable layout if bound
  col = uiLayoutColumn(layout, false);
  uiItemR(col, ptr, "falloff", UI_ITEM_NONE, NULL, ICON_NONE);

  bool display_unbind = false;

  uiItemR(layout, ptr, "strength", UI_ITEM_NONE, NULL, ICON_NONE);
  row = uiLayoutRow(layout, true);
  modifier_vgroup_ui(layout, ptr, &ob_ptr, "vertex_group", "invert_vertex_group", nullptr);


  uiItemS(layout);

  col = uiLayoutColumn(layout, false);

  // BIND 
  row = uiLayoutRow(col, true);
  uiLayoutSetActive(col, !RNA_pointer_is_null(&target_ptr));
  uiItemFullO(row,
              "OBJECT_OT_gpencilsurdeform_bind",
              IFACE_("Bind All"),
              ICON_NONE,
              NULL,
              WM_OP_INVOKE_DEFAULT,
              UI_ITEM_NONE,
              &op_ptr_all);
  RNA_enum_set(&op_ptr_all, "curr_frame_or_all_frames", GP_MOD_SDEF_BIND_CURRENT_FRAME);
  uiItemFullO(row,
              "OBJECT_OT_gpencilsurdeform_bind",
              IFACE_("Bind Current Frame"),
              ICON_NONE,
              NULL,
              WM_OP_INVOKE_DEFAULT,
              UI_ITEM_NONE,
              &op_ptr_curr);
  RNA_enum_set(&op_ptr_curr, "curr_frame_or_all_frames", GP_MOD_SDEF_BIND_ALL_FRAMES);

  //UNBIND
  row = uiLayoutRow(col, true);
  uiLayoutSetActive(col, !RNA_pointer_is_null(&target_ptr));
  uiItemFullO(row,
              "OBJECT_OT_gpencilsurdeform_unbind",
              IFACE_("Unbind All"),
              ICON_NONE,
              NULL,
              WM_OP_INVOKE_DEFAULT,
              UI_ITEM_NONE,
              &op_ptr_all);
  RNA_enum_set(&op_ptr_all, "curr_frame_or_all_frames", GP_MOD_SDEF_BIND_CURRENT_FRAME);
  uiItemFullO(row,
              "OBJECT_OT_gpencilsurdeform_unbind",
              IFACE_("Unbind current Frame"),
              ICON_NONE,
              NULL,
              WM_OP_INVOKE_DEFAULT,
              UI_ITEM_NONE,
              &op_ptr_curr);
  RNA_enum_set(&op_ptr_curr, "curr_frame_or_all_frames", GP_MOD_SDEF_BIND_ALL_FRAMES);

  row = uiLayoutRow(col, true);

  modifier_panel_end(layout, ptr);
}

static void bake_panel_draw(const bContext *(C), Panel *panel)
{
  uiLayout *row, *col, *sub;
  uiLayout *layout = panel->layout;

  PointerRNA bake_op;

  int toggles_flag = UI_ITEM_R_TOGGLE | UI_ITEM_R_FORCE_BLANK_DECORATE;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, NULL);
  Scene *scene = CTX_data_scene(C);

  col = uiLayoutColumn(layout, false);

  uiLayoutSetPropSep(layout, true);

  row = uiLayoutRow(col, true);
  uiItemFullO(row,
              "OBJECT_OT_gpencilsurdeform_bake",
              IFACE_("Bake Current Frame"),
              ICON_NONE,
              NULL,
              WM_OP_INVOKE_DEFAULT,
              UI_ITEM_NONE,
              &bake_op);
  int current_frame_num = (int)BKE_scene_frame_get(scene);
  RNA_int_set(&bake_op, "frame_start", current_frame_num);
  RNA_int_set(&bake_op, "frame_end", current_frame_num);

  row = uiLayoutRow(col, true);
  uiItemFullO(row,
              "OBJECT_OT_gpencilsurdeform_bake",
              IFACE_("Bake Range"),
              ICON_NONE,
              NULL,
              WM_OP_INVOKE_DEFAULT,
              UI_ITEM_NONE,
              &bake_op);
  RNA_int_set(&bake_op, "frame_start", RNA_int_get(ptr, "bake_range_start"));
  RNA_int_set(&bake_op, "frame_end", RNA_int_get(ptr, "bake_range_end"));

  row = uiLayoutRow(col, true);
  uiItemL(row, "Range:", ICON_NONE);
  row = uiLayoutRow(col, true);
  uiItemR(row, ptr, "bake_range_start", UI_ITEM_NONE, "Start", ICON_NONE);
  sub = uiLayoutRow(row, true);
  uiItemR(sub, ptr, "bake_range_end", UI_ITEM_NONE, "End", ICON_NONE);
  sub = uiLayoutRow(row, true);
  uiItemFullO(sub,
              "OBJECT_OT_gpencilsurdeform_fillrange",
              IFACE_(""),
              ICON_PREVIEW_RANGE,
              NULL,
              WM_OP_INVOKE_DEFAULT,
              UI_ITEM_NONE,
              NULL);

  row = uiLayoutRow(col, true);
  uiItemL(row,
          "Note that baked frames are unbound. Please bind the freshly baked frame if desired",
          ICON_INFO);

  // gpencil_modifier_panel_end(layout, ptr);

  /*gpencil_modifier_masking_panel_draw(panel, true, true);*/
}

static void panel_register(ARegionType *region_type)
{
  PanelType *panel_type = modifier_panel_register(
      region_type, eModifierType_GPencilSurDeform, panel_draw);
  modifier_subpanel_register(region_type, "bake", "Bake", NULL, bake_panel_draw, panel_type);
}

static void blend_write(BlendWriter *writer, const ID *id_owner, const ModifierData *md){
  return;
}

static void blend_read(BlendDataReader *reader, ModifierData *md)
{
  return;
}
}  // namespace blender

ModifierTypeInfo modifierType_GPencilSurDeform = {
    /*idname*/ "GPencilSurDeform",
    /*name*/ N_("Surface Deform"),
    /*struct_name*/ "GPencilSurDeformModifierData",
    /*struct_size*/ sizeof(GPencilSurDeformModifierData),
    /*srna*/ &RNA_GPencilSurDeformModifier, 
    /*type*/ ModifierTypeType::OnlyDeform,
    /*flags*/ eModifierTypeFlag_AcceptsGreasePencil | eModifierTypeFlag_SupportsMapping,
    /*icon*/ ICON_MOD_MESHDEFORM,

    /*copy_data*/ blender::copy_data,

    /*deform_verts*/ nullptr,
    /*deform_matrices*/ nullptr,
    /*deform_verts_EM*/ nullptr,
    /*deform_matrices_EM*/ nullptr,
    /*modify_mesh*/ nullptr,
    /*modify_geometry_set*/ blender::modify_geometry_set,

    /*init_data*/ blender::init_data,
    /*required_data_mask*/ nullptr,
    /*free_data*/ blender::free_data,
    /*is_disabled*/ blender::is_disabled,
    /*update_depsgraph*/ blender::update_depsgraph,
    /*depends_on_time*/ nullptr,
    /*depends_on_normals*/ nullptr,
    /*foreach_ID_link*/ blender::foreach_ID_link,
    /*foreach_tex_link*/ nullptr,
    /*free_runtime_data*/ nullptr,
    /*panel_register*/ blender::panel_register,
    /*blend_write*/ blender::blend_write,
    /*blend_read*/ blender::blend_read,
};

/*
grease_pencil: *geometry_set->get_grease_pencil_for_write()
frames: vector containing the frames to bind.*/
bool GPencilSurDeformModifierData::bind_drawings(ModifierData *md_orig,
                                                                  const Depsgraph *depsgraph,
                                                         Scene *sc,
                                                        // Vector<int> frames,
                                                        Object *ob)
{ using namespace blender;
  using bke::greasepencil::Drawing;
  
  GPencilSurDeformModifierData *smd_orig = (GPencilSurDeformModifierData *)md_orig;
  // eValuate the 2 objectS (current and target) so it can be bound on deformations after the
  // modifiers
  Object *ob_target = DEG_get_evaluated_object(depsgraph, smd_orig->target);
  Object *ob_eval = DEG_get_evaluated_object(depsgraph, ob);
  Object *ob_orig = DEG_get_original_object(ob);
  Mesh *target = BKE_modifier_get_evaluated_mesh_from_evaluated_object(ob_target);
  GreasePencil &grease_pencil_orig = *reinterpret_cast<GreasePencil *>(ob_orig->data);
  GreasePencil &grease_pencil_eval = *reinterpret_cast<GreasePencil *>(ob_eval->data);
  IndexMaskMemory layer_mask_memory;
  const IndexMask layer_mask = blender::modifier::greasepencil::get_filtered_layer_mask(
      grease_pencil_orig, smd_orig->influence, layer_mask_memory);
  IndexMaskMemory frame_mask_memory;
  bool success = true;
  bool success_temp = false;
  
  if (smd_orig->bind_modes & GP_MOD_SDEF_UNBIND_MODE) {
    if (smd_orig->bind_modes & GP_MOD_SDEF_BIND_ALL_FRAMES) {
      // unselect all frames
      for (bke::greasepencil::Layer *layer : grease_pencil_orig.layers_for_write()) {
        blender::ed::greasepencil::select_all_frames(*layer, SELECT_SUBTRACT);
        for (auto [frame_number, frame] : layer->frames_for_write().items()) {
          // unbind drawing
          GreasePencilDrawingBase *drawing_base = grease_pencil_orig.drawings()[frame.drawing_index];
          if (drawing_base->type == GP_DRAWING)
          {
            GreasePencilDrawing *drawing = (reinterpret_cast<GreasePencilDrawing *>(drawing_base));
            success_temp = unbind_drawing(md_orig, ob_eval, &drawing->wrap(), target);
            if (!success_temp) {
              success = false;
              frame.flag &= ~GP_FRAME_SELECTED;
            }
            else
              frame.flag |= GP_FRAME_SELECTED;
          }
          
        }
      }


      
      DEG_id_tag_update(&grease_pencil_orig.id, ID_RECALC_GEOMETRY);
      
    }
    else if (smd_orig->bind_modes & GP_MOD_SDEF_BIND_CURRENT_FRAME) {
      Vector<GreasePencilFrame *> frames;

      const int frame = grease_pencil_eval.runtime->eval_frame;
    // unselect all frames
      for (bke::greasepencil::Layer *layer : grease_pencil_orig.layers_for_write()) {
        blender::ed::greasepencil::select_all_frames(*layer, SELECT_SUBTRACT);
        frames.append(layer->frame_at(frame));
      }
      const Vector<Drawing *> drawings = modifier::greasepencil::get_drawings_for_write(
          grease_pencil_orig, layer_mask, frame);
      // unbind drawwing
      for (Drawing *drawing : drawings) {
        success_temp = unbind_drawing(md_orig, ob_eval, drawing, target);
        if (!success_temp)
          success = false;
      }
      // select the frames whose type to change
      for (GreasePencilFrame * f : frames) {
        f->flag |= GP_FRAME_SELECTED;
      }

      DEG_id_tag_update(&grease_pencil_orig.id, ID_RECALC_GEOMETRY);
    }
    for (bke::greasepencil::Layer *layer : grease_pencil_orig.layers_for_write()) {
      blender::ed::greasepencil::set_selected_frames_type(*layer, BEZT_KEYTYPE_KEYFRAME);
    }
    return success;
  }

  if (smd_orig->bind_modes & GP_MOD_SDEF_BIND_CURRENT_FRAME) {

    Vector<GreasePencilFrame *> frames;
    const int frame = grease_pencil_eval.runtime->eval_frame;

    // unselect all frames
    for (bke::greasepencil::Layer *layer : grease_pencil_orig.layers_for_write()) {
      blender::ed::greasepencil::select_all_frames(*layer, SELECT_SUBTRACT);
      frames.append(layer->frame_at(frame));
    }

    const Vector<Drawing *> drawings = modifier::greasepencil::get_drawings_for_write(
        grease_pencil_orig, layer_mask, frame);
    // bind drawwing
    for (Drawing *drawing : drawings) {
      success_temp = bind_drawing(md_orig, ob_eval, drawing, target);
      if (!success_temp)
        success = false;
    }
    // select the frames whose type to change
    for (GreasePencilFrame *f : frames) {
      f->flag |= GP_FRAME_SELECTED;
    }
  }
  else if (smd_orig->bind_modes & GP_MOD_SDEF_BIND_ALL_FRAMES) {
    Array<Vector<ed::greasepencil::MutableDrawingInfo>> drawings_per_frame =
        ed::greasepencil::retrieve_editable_drawings_grouped_per_frame(*sc, grease_pencil_orig);

    /* const IndexMask full_mask = drawings_per_frame.index_range();
    IndexMask frame_mask = full_mask;
     IndexMask::from_predicate(
    full_mask, GrainSize(4096), memory, [&](const int64_t layer_i) {
      if (layer_name_filter) {
        const Layer &layer = *layers[layer_i];
        const bool match = (layer.name() == layer_name_filter.value());
        if (match == layer_filter_invert) {
          return false;
        }
      }
      if (layer_pass_filter) {
        const int layer_pass = layer_passes.get(layer_i);
        const bool match = (layer_pass == layer_pass_filter.value());
        if (match == layer_pass_filter_invert) {
          return false;
        }
      }
      return true;
    });*/
    
    // unselect all frames
    for (bke::greasepencil::Layer *layer : grease_pencil_orig.layers_for_write()) {
      blender::ed::greasepencil::select_all_frames(*layer, SELECT_SUBTRACT);
      for (auto [frame_number, frame] : layer->frames_for_write().items()) {
        // unbind drawing
        GreasePencilDrawingBase *drawing_base = grease_pencil_orig.drawings()[frame.drawing_index];
        if (drawing_base->type == GP_DRAWING)
        {
          GreasePencilDrawing *drawing = (reinterpret_cast<GreasePencilDrawing *>(drawing_base));
          success_temp = bind_drawing(md_orig, ob_eval, &drawing->wrap(), target);
          if (!success_temp) {
            success = false;
            frame.flag &= ~GP_FRAME_SELECTED;
          }
          else
            frame.flag |= GP_FRAME_SELECTED;
        }
      }
    }
    
  }
  for (bke::greasepencil::Layer *layer : grease_pencil_orig.layers_for_write()) {
    blender::ed::greasepencil::set_selected_frames_type(*layer, BEZT_KEYTYPE_SURDEFBOUND);
  }


  DEG_id_tag_update(&grease_pencil_orig.id, ID_RECALC_GEOMETRY);
  return success;
}
blender::Span<SDefGPVert> GPencilSurDeformModifierData::verts() const
{
  return {this->verts_array, this->verts_array_tot};
}

blender::MutableSpan<SDefGPVert> GPencilSurDeformModifierData::verts()
{
  return {this->verts_array, this->verts_array_tot};
}

blender::Span<SDefGPBind> SDefGPVert::binds() const
{
  return {this->binds_array, this->binds_num};
}

blender::MutableSpan<SDefGPBind> SDefGPVert::binds()
{
  return {this->binds_array, this->binds_num};
}
