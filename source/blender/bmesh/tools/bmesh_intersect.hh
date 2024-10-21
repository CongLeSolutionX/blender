/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bmesh
 */

enum {
  BMESH_ISECT_BOOLEAN_NONE = -1,
  /* aligned with BooleanModifierOp */
  BMESH_ISECT_BOOLEAN_ISECT = 0,
  BMESH_ISECT_BOOLEAN_UNION = 1,
  BMESH_ISECT_BOOLEAN_DIFFERENCE = 2,
};

/* `bmesh_intersect.cc` */

/**
 * Intersect tessellated faces
 * leaving the resulting edges tagged.
 *
 * \param test_fn: Return value: -1: skip, 0: tree_a, 1: tree_b (use_self == false)
 * \param boolean_mode: -1: no-boolean, 0: intersection... see #BMESH_ISECT_BOOLEAN_ISECT.
 * \return true if the mesh is changed (intersections cut or faces removed from boolean).
 */
bool BM_mesh_intersect(BMesh *bm,
                       blender::Span<std::array<BMLoop *, 3>> looptris,
                       int (*test_fn)(BMFace *f, void *user_data),
                       void *user_data,
                       bool use_self,
                       bool use_separate,
                       bool use_dissolve,
                       bool use_island_connect,
                       bool use_partial_connect,
                       bool use_edge_tag,
                       int boolean_mode,
                       float eps);

/* `bmesh_intersect_v2.cc` */

/**
 * Intersect tessellated faces, leaving the resulting edges tagged.
 *
 * \param test_fn: Function that determines which intersection tree a face belongs to.
 *                 Return values:
 *                 - -1: skip,
 *                 -  0: tree_a,
 *                 -  1: tree_b (use_self == false).
 * \param use_self: tree_a == tree_b.
 * \param use_separate: Separate the groups of faces delimited by the tagged edges.
 * \param use_dissolve: Remove verts created by intersecting triangles.
 * \param use_island_connect: Connects holes in the edge-net created on faces to split.
 * \param use_partial_connect: Enhances `use_island_connect` by supporting islands
 *                             connected by a single edge.
 *                             \note This option is quite slow, so avoid using it when possible.
 * \param use_remove_coplanar_faces: Removes coplanar faces in tree_a.
 * \param eps: The overlap threshold.
 * \param remove_face_group_fn: Callback to determine whether a group of faces
 *                              should be deleted based on boolean logic.
 *
 * \return True if the mesh is changed (e.g., intersections cut or faces removed).
 */
bool BM_mesh_intersect_v2(BMesh *bm,
                          blender::Span<std::array<BMLoop *, 3>> looptris,
                          int (*test_fn)(BMFace *f, void *user_data),
                          void *user_data,
                          bool use_self,
                          bool use_separate,
                          bool use_dissolve,
                          bool use_island_connect,
                          bool use_partial_connect,
                          bool use_remove_coplanar_faces,
                          float eps,
                          blender::FunctionRef<bool(struct BVHTree *trees[2],
                                                    const blender::float3 (*looptri_coords)[3],
                                                    BMFace *const *ftable,
                                                    blender::Span<int> face_indices,
                                                    int side)> remove_face_group_fn);
