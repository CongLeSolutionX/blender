/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bmesh
 *
 * Cut meshes along intersections.
 *
 * Boolean-like modeling operation (without calculating inside/outside).
 *
 * Supported:
 * - Concave faces.
 * - Non-planar faces.
 * - Custom-data (UVs etc).
 *
 * Unsupported:
 * - Intersecting between different meshes.
 */

#include "BLI_alloca.h"
#include "BLI_buffer.h"
#include "BLI_kdopbvh.h"
#include "BLI_linklist.h"
#include "BLI_math_geom.h"
#include "BLI_memarena.h"
#include "BLI_ordered_edge.hh"
#include "BLI_sort.h"

#include "bmesh.hh"

#include "tools/bmesh_edgesplit.hh"

#include "bmesh_intersect.hh" /* own include */

using namespace blender;

#ifndef NDEBUG
#  define DEBUG_ISECT
#endif

#define VERT_DISSOLVE_FLAG BM_ELEM_TAG_ALT
#define EDGE_ISECT_FLAG BM_ELEM_TAG

/* Prototype. */
static LinkNode *isectmap_node_new(struct IsectMap *s);

enum eIsectElemType {
  NONE_MAYBE_DISSOLVE = -2, /* Used for vertices created in cases of Edge x Edge intersections
                               where one of the edges is not a BMEdge. */
  NONE = -1,                /* Identifies triangles or edges that are not a BMEdges. */
  VERT = 0,
  EDGE = 1,
};

/**
 * Pair containing two intersecting elements (vertices or edges).
 */
struct IsectPair {
  union {
    void *ptr;
    BMVert *v;
    BMEdge *e;
  } elem_a;
  union {
    void *ptr;
    BMVert *v;
    BMEdge *e;
  } elem_b;
  short type_a;
  short type_b;
  union {
    struct {
      float lambda_a;
      float lambda_b;
      uint sort_vote; /* Unused. */
    };
    float3 co;
  };
};

/**
 * Edge formed by the intersection between two triangles effectively linking two #IsectPair pairs.
 */
struct IsectEdge {
  IsectPair *pair_a;
  IsectPair *pair_b;
  BMEdge *e = nullptr;

  IsectEdge(IsectPair *pair_1, IsectPair *pair_2)
  {
    /* The order of the pairs is normalized, meaning `pair_a` is always the smaller address to
     * ensure *consistency in comparisons and hashing.*/
    BLI_assert(pair_1 != pair_2);
    if (uint64_t(pair_1) < uint64_t(pair_2)) {
      this->pair_a = pair_1;
      this->pair_b = pair_2;
    }
    else {
      this->pair_a = pair_2;
      this->pair_b = pair_1;
    }
  }

  uint64_t hash() const
  {
    uint64_t k0 = uint64_t(this->pair_a);
    uint64_t k1 = uint64_t(this->pair_b);
    BLI_assert(k0 <= k1);
    return (k0 << 8) ^ k1;
  }

  bool operator==(const IsectEdge &other) const
  {
    /* Ignore the #BMEdge. */
    return this->pair_a == other.pair_a && this->pair_b == other.pair_b;
  }
};

struct IsectEdgeRef {
  IsectEdge *isect_edge;

  uint64_t hash() const
  {
    return this->isect_edge->hash();
  }

  static uint64_t hash_as(const IsectEdge *isect_edge)
  {
    return isect_edge->hash();
  }

  bool operator==(const IsectEdgeRef &other) const
  {
    return *this->isect_edge == *other.isect_edge;
  }
};

inline bool operator==(const IsectEdge *isect_edge, const IsectEdgeRef &ekey)
{
  return *isect_edge == *ekey.isect_edge;
}

/**
 * Stores data related to intersections on a face, including lists of coplanar faces and
 * intersecting edges. The `edge_ptrs_len` is maintained to allow efficient access to the list
 * length without recalculating it each time.
 */
struct FaceData {
  LinkNode *coplanar_faces; /* #BMFace. */
  LinkNode *edge_ptrs;      /* Pointer to #IsectEdge::e. */
  uint edge_ptrs_len;
  bool is_face_a;

  void add_edge_ptr(IsectMap *s, BMEdge **edge_ptr)
  {
    if (BLI_linklist_index(this->edge_ptrs, edge_ptr) == -1) {
      LinkNode *node = isectmap_node_new(s);
      node->next = this->edge_ptrs;
      node->link = (void *)edge_ptr;
      this->edge_ptrs = node;
      this->edge_ptrs_len++;
    }
  }

  void add_face_coplanar(IsectMap *s, BMFace *f_coplanar)
  {
    if (BLI_linklist_index(this->coplanar_faces, f_coplanar) == -1) {
      LinkNode *node = isectmap_node_new(s);
      node->next = this->coplanar_faces;
      node->link = f_coplanar;
      this->coplanar_faces = node;
    }
  }
};

/* As a customdata for some faces, it stores a #BMFace and a #FaceData that can be shared across
 * other #BMFaces. */
struct BMFaceFaceData {
  BMFace *f;
  FaceData *fdata;

  uint64_t hash() const
  {
    return uint64_t(this->f);
  }

  static uint64_t hash_as(const BMFace *f)
  {
    return uint64_t(f);
  }

  bool operator==(const BMFaceFaceData &other) const
  {
    return this->f == other.f;
  }
};

inline bool operator==(const BMFace *f, const BMFaceFaceData &fkey)
{
  return f == fkey.f;
}

/**
 * IsectMap: Main object for managing intersections in a mesh.
 *
 * This structure maps intersections between various elements in a mesh,
 * including vertices, edges, and faces, ensuring that unique intersections
 * are tracked.
 */
struct IsectMap {
  BMesh *bm;

  Map<int2, IsectPair *> vert_tri;         /* vertex-triangle cases. */
  Map<OrderedEdge, IsectPair *> vert_vert; /* vertex-vertex cases. */
  Map<int3, IsectPair *> vert_edge;        /* vertex-edge cases. */
  Map<int3, IsectPair *> edge_tri;         /* edge-triangle cases. */
  Map<int4, IsectPair *> edge_edge;        /* edge-edge cases. */

  /* Stores the intersection edges and the face datas.
   * NOTE: Use VectorSet because the insertion order is important for regression tests. */
  VectorSet<IsectEdgeRef> isect_edges;
  VectorSet<BMFaceFaceData> face_data;

  /* Memory management for #LinkNode, #IsectPair, #IsectEdge and #FaceData. */
  MemArena *mem_arena;
  LinkNode *removed_nodes;

  /* Precision settings for intersections. */
  float eps;
  float eps_sq;

  IsectMap(BMesh *bm, const float eps)
  {
    this->bm = bm;

    this->mem_arena = BLI_memarena_new(BLI_MEMARENA_STD_BUFSIZE, __func__);
    this->removed_nodes = nullptr;

    this->eps = eps;
    this->eps_sq = eps * eps;
  }

  ~IsectMap()
  {
    BLI_memarena_free(this->mem_arena);
  }

  /* #IsectPair creation. */

  IsectPair *lookup_or_add_vert_vert(BMVert *va, BMVert *vb)
  {
    OrderedEdge key_vert_vert = OrderedEdge(BM_elem_index_get(va), BM_elem_index_get(vb));
    return this->vert_vert.lookup_or_add_cb(key_vert_vert, [&]() {
      IsectPair *pair = (IsectPair *)BLI_memarena_alloc(this->mem_arena, sizeof(*pair));
      pair->elem_a.v = va;
      pair->type_a = VERT;
      if (va != vb) {
        pair->elem_b.v = vb;
        pair->type_b = VERT;
      }
      else {
        pair->elem_b.v = nullptr;
        pair->type_b = NONE;
      }
      return pair;
    });
  }

  IsectPair *lookup_or_add_vert_edge(BMVert *v, BMVert *ev_a, BMVert *ev_b, const float fac)
  {
    int3 key_vert_edge = {BM_elem_index_get(ev_a), BM_elem_index_get(ev_b), BM_elem_index_get(v)};
    if (key_vert_edge[0] > key_vert_edge[1]) {
      std::swap(key_vert_edge[0], key_vert_edge[1]);
    }
    return this->vert_edge.lookup_or_add_cb(key_vert_edge, [&]() {
      IsectPair *val = (IsectPair *)BLI_memarena_alloc(this->mem_arena, sizeof(*val));
      BMEdge *e = BM_edge_exists(ev_a, ev_b);
      val->elem_a.v = v;
      val->type_a = VERT;
      if (e) {
        val->elem_b.e = e;
        val->type_b = EDGE;
        val->lambda_b = e->v1 == ev_a ? fac : 1.0f - fac;
      }
      else {
        val->elem_b.ptr = v->co;
        val->type_b = NONE;
      }
      return val;
    });
  }

  IsectPair *lookup_or_add_vert_tri(BMVert *v, int loop_tri)
  {
    int2 key_vert_tri = {BM_elem_index_get(v), loop_tri};
    return this->vert_tri.lookup_or_add_cb(key_vert_tri, [&]() {
      IsectPair *val = (IsectPair *)BLI_memarena_alloc(this->mem_arena, sizeof(*val));
      val->elem_a.v = v;
      val->type_a = VERT;
      val->type_b = NONE;
      return val;
    });
  }

  IsectPair *lookup_or_add_edge_edge(BMVert *ea_va,
                                     BMVert *ea_vb,
                                     BMVert *eb_va,
                                     BMVert *eb_vb,
                                     const float fac_a,
                                     const float fac_b)
  {
    int4 key_edge_edge = {
        BM_elem_index_get(ea_va),
        BM_elem_index_get(ea_vb),
        BM_elem_index_get(eb_va),
        BM_elem_index_get(eb_vb),
    };
    if (key_edge_edge[0] > key_edge_edge[1]) {
      std::swap(key_edge_edge[0], key_edge_edge[1]);
    }
    if (key_edge_edge[2] > key_edge_edge[3]) {
      std::swap(key_edge_edge[2], key_edge_edge[3]);
    }
    if (key_edge_edge[0] > key_edge_edge[2]) {
      std::swap(key_edge_edge[0], key_edge_edge[2]);
      std::swap(key_edge_edge[1], key_edge_edge[3]);
    }
    return this->edge_edge.lookup_or_add_cb(key_edge_edge, [&]() {
      IsectPair *val = (IsectPair *)BLI_memarena_alloc(this->mem_arena, sizeof(*val));
      BMEdge *ea = BM_edge_exists(ea_va, ea_vb);
      BMEdge *eb = BM_edge_exists(eb_va, eb_vb);
      if (ea) {
        val->elem_a.e = ea;
        val->type_a = EDGE;
        val->lambda_a = ea->v1 == ea_va ? fac_a : 1.0f - fac_a;
      }
      else {
        val->elem_a.ptr = val->co;
        val->type_a = NONE;
      }

      if (eb) {
        val->elem_b.e = eb;
        val->type_b = EDGE;
        val->lambda_b = eb->v1 == eb_va ? fac_b : 1.0f - fac_b;
      }
      else {
        val->elem_b.ptr = val->co;
        val->type_b = NONE;
      }

      if (!ea && !eb) {
        float3 co_a, co_b;
        interp_v3_v3v3(co_a, ea_va->co, ea_vb->co, fac_a);
        interp_v3_v3v3(co_b, eb_va->co, eb_vb->co, fac_b);
        val->co = math::midpoint(co_a, co_b);
      }
      return val;
    });
  }

  IsectPair *lookup_or_add_edge_tri(BMVert *ev_a,
                                    BMVert *ev_b,
                                    const int loop_tri,
                                    const float fac)
  {
    int3 key_edge_tri = {BM_elem_index_get(ev_a), BM_elem_index_get(ev_b), loop_tri};
    if (key_edge_tri[0] > key_edge_tri[1]) {
      std::swap(key_edge_tri[0], key_edge_tri[1]);
    }
    return this->edge_tri.lookup_or_add_cb(key_edge_tri, [&]() {
      BMEdge *e = BM_edge_exists(ev_a, ev_b);
      IsectPair *val = (IsectPair *)BLI_memarena_alloc(this->mem_arena, sizeof(*val));
      if (e) {
        val->elem_a.e = e;
        val->type_a = EDGE;
        val->lambda_a = e->v1 == ev_a ? fac : 1.0f - fac;
      }
      else {
        val->elem_a.ptr = val->co;
        val->type_a = NONE;
        val->co = math::interpolate(float3(ev_a->co), float3(ev_b->co), fac);
      }

      val->elem_b.ptr = val->co;
      val->type_b = NONE;
      return val;
    });
  }

  /* #IsectEdge creation. */
  IsectEdge *lookup_or_add_isect_edge(IsectPair *pair_a, IsectPair *pair_b)
  {
    IsectEdge isect_edge(pair_a, pair_b);
    int eindex = this->isect_edges.index_of_try_as(&isect_edge);
    if (eindex >= 0) {
      return this->isect_edges[eindex].isect_edge;
    }
    IsectEdgeRef ekey;
    ekey.isect_edge = (IsectEdge *)BLI_memarena_calloc(this->mem_arena, sizeof(*ekey.isect_edge));
    *ekey.isect_edge = isect_edge;
    this->isect_edges.add_new(ekey);
    return ekey.isect_edge;
  }

  /* #FaceData creation. */
  FaceData *lookup_or_add_face_data(BMFace *f, const bool is_face_a)
  {
    int findex = this->face_data.index_of_try_as(f);
    if (findex >= 0) {
      return this->face_data[findex].fdata;
    }
    BMFaceFaceData fkey = {};
    fkey.f = f;
    fkey.fdata = (FaceData *)BLI_memarena_calloc(this->mem_arena, sizeof(*fkey.fdata));
    fkey.fdata->is_face_a = is_face_a;
    this->face_data.add_new(fkey);
    return fkey.fdata;
  }

  void add_face_data(BMFace *f, FaceData *fdada)
  {
    BLI_assert(this->face_data.index_of_try_as(f) == -1);
    BMFaceFaceData fkey = {};
    fkey.f = f;
    fkey.fdata = fdada;
    this->face_data.add_new(fkey);
  }

  /* Creates a new LinkNode reusing a removed node if available; otherwise, allocates a new one
   * from the memory arena. */
  LinkNode *node_new()
  {
    LinkNode *node = this->removed_nodes;
    if (node) {
      this->removed_nodes = node->next;
    }
    else {
      node = (LinkNode *)BLI_memarena_alloc(this->mem_arena, sizeof(*node));
    }
    return node;
  }

  /* Removes a LinkNode from a linked list and adds it to the list of removed nodes. */
  void node_remove(LinkNode **node_ptr, LinkNode *node)
  {
    *node_ptr = node->next;
    node->next = this->removed_nodes;
    this->removed_nodes = node;
  }

  /* Adds a face to the coplanar list of all faces that are coplanar with the given face. */
  void face_coplanar_add(BMFace *f, const FaceData *fdata)
  {
    for (LinkNode *node_f = fdata->coplanar_faces; node_f; node_f = node_f->next) {
      BMFace *f_coplanar = (BMFace *)node_f->link;
      const BMFaceFaceData *fkey_coplanar = this->face_data.lookup_key_ptr({f_coplanar});
      if (!fkey_coplanar || !fkey_coplanar->fdata) {
        continue;
      }

      BLI_assert(fdata->is_face_a != fkey_coplanar->fdata->is_face_a);
      fkey_coplanar->fdata->add_face_coplanar(this, f);
    }
  }

  /* Removes a face from the coplanar lists of all faces that are coplanar with it. */
  void face_coplanar_remove(BMFace *f, const FaceData *fdata)
  {
    /* Remove face from other lists. */
    for (LinkNode *node_f = fdata->coplanar_faces; node_f; node_f = node_f->next) {
      BMFace *f_coplanar = (BMFace *)node_f->link;
      const BMFaceFaceData *fkey_coplanar = this->face_data.lookup_key_ptr({f_coplanar});
      if (!fkey_coplanar || !fkey_coplanar->fdata) {
        continue;
      }

      BLI_assert(fdata->is_face_a != fkey_coplanar->fdata->is_face_a);
      LinkNode **node_ptr, *node;
      node_ptr = &fkey_coplanar->fdata->coplanar_faces;
      while ((node = *node_ptr)) {
        if (node->link == f) {
          this->node_remove(node_ptr, node);
          break;
        }
        node_ptr = &node->next;
      }
    }
#ifdef DEBUG_ISECT
    /* Debugging: Ensure consistency in the coplanar relationships. */
    for (const BMFaceFaceData &fkey : this->face_data) {
      BLI_assert(fkey.f != f || fdata == fkey.fdata);
      for (LinkNode *node_f = fkey.fdata->coplanar_faces; node_f; node_f = node_f->next) {
        BMFace *f_coplanar = (BMFace *)node_f->link;
        BLI_assert(f_coplanar != f);
      }
    }
#endif
  }
};

/* The #FaceData type needs this function to create nodes. */
static LinkNode *isectmap_node_new(struct IsectMap *s)
{
  return s->node_new();
}

/* -------------------------------------------------------------------- */
/** \name Triangle Triangle Intersections
 *
 * The `intersect_tri_tri` function identifies and populates intersection maps in the Main
 * #IsectMap object.
 * \{ */

/**
 * Precomputes values for intersection operations between the vertices and edges of two triangles,
 * avoiding repetitive calculations. This includes precomputed normals, tangents, edge directions,
 * and distances to improve efficiency.
 */
struct IsectTriTriPrecalc {
  struct TriPrecalc {
    float3 v[3];           /* Vertices of the triangle. */
    float3 e_dir[3];       /* Directions of each edge. */
    float e_dir_len_sq[3]; /* Squared lengths of each edge direction. */

    float3 normal;                /* Normal of the triangle. */
    float3 tangent[3];            /* Tangents for each edge, perpendicular to `normal`. */
    float dist_to_plane_other[3]; /* Distance of each vertex to the plane of the other triangle. */
    bool is_degenerate = false;
  } tri_[2];

  float3 dir_other[3][3];       /* Directions between each pair of vertices from tri_0 to tri_1. */
  float dir_other_len_sq[3][3]; /* Squared lengths of `dir_other` directions. */

  IsectTriTriPrecalc(float3 tri0[3], float3 tri1[3], float dist_min_sq)
  {
    std::copy(tri0, tri0 + 3, tri_[0].v);
    std::copy(tri1, tri1 + 3, tri_[1].v);
    for (int t : IndexRange(2)) {
      for (uint e = 2, e_next = 0; e_next < 3; e = e_next++) {
        tri_[t].e_dir[e] = tri_[t].v[e_next] - tri_[t].v[e];
        tri_[t].e_dir_len_sq[e] = math::length_squared(tri_[t].e_dir[e]);
      }
    }
    for (int v0 : IndexRange(3)) {
      for (int v1 : IndexRange(3)) {
        this->dir_other[v0][v1] = tri_[1].v[v1] - tri_[0].v[v0];
        this->dir_other_len_sq[v0][v1] = math::length_squared(this->dir_other[v0][v1]);
      }
    }

    tri_[0].normal = math::normal_tri(UNPACK3(tri_[0].v));
    tri_[1].normal = math::normal_tri(UNPACK3(tri_[1].v));

    for (int t : IndexRange(2)) {
      tri_[t].tangent[0] = math::cross(tri_[t].e_dir[0], tri_[t].normal);
      tri_[t].tangent[1] = math::cross(tri_[t].e_dir[1], tri_[t].normal);
      tri_[t].tangent[2] = math::cross(tri_[t].e_dir[2], tri_[t].normal);

      float3 &tri_other_normal = tri_[int(!t)].normal;
      for (int i : IndexRange(3)) {
        tri_[t].dist_to_plane_other[i] = math::dot(this->dir_other[i][i], tri_other_normal);
        if (tri_[t].e_dir_len_sq[i] < dist_min_sq) {
          tri_[t].is_degenerate = true;
        }
      }
    }
  }

  /* Calculates the squared distance from a point to a segment only if the projection of that
   * point lies within the segment bounds. If the projection is outside, returns false. */
  static bool dist_sq_vert_edge_impl(const TriPrecalc &tri_edge,
                                     const uint e,
                                     const float3 &e_v_dir,
                                     const float e_v_dir_len_sq,
                                     float &r_dist_sq,
                                     float &r_lambda)
  {
    float dot_vert_edge = math::dot(e_v_dir, tri_edge.e_dir[e]);
    if (dot_vert_edge <= 0.0f) {
      return false;
    }
    float e_dir_length_sq = tri_edge.e_dir_len_sq[e];
    if (dot_vert_edge >= e_dir_length_sq) {
      return false;
    }
    r_lambda = dot_vert_edge / e_dir_length_sq;
    /* Floating-point precision limitations can introduce slight inaccuracies in projection and
     * squared length calculations. These inaccuracies may cause calculated distances to be
     * marginally longer than expected. To address this and bias toward shorter distances,
     * `prog_length_sq` is adjusted upward slightly.
     * NOTE: Alternatively, `prog_length_sq` can be computed as follows:
     *
     * \code{.cc}
     * float prog_length_sq = math::square(dot_vert_edge) / e_dir_length_sq;
     * \endcode
     */
    float prog_length_sq = std::nextafter(math::square(r_lambda), FLT_MAX) *
                           std::nextafter(e_dir_length_sq, FLT_MAX);

    r_dist_sq = e_v_dir_len_sq - prog_length_sq;
    return true;
  }

  /* Determines whether a point is within the vertical prism extending from a triangle. */
  static bool is_point_on_tri_prism_impl(const TriPrecalc &tri, const float3 &point)
  {
    if (math::dot(point - tri.v[0], tri.tangent[0]) >= 0.0f) {
      return false;
    }
    if (math::dot(point - tri.v[1], tri.tangent[1]) >= 0.0f) {
      return false;
    }
    if (math::dot(point - tri.v[2], tri.tangent[2]) >= 0.0f) {
      return false;
    }
    return true;
  }

  /* Calculates the squared distance from a vertex to a triangle if the vertex lies within
   * the vertical prism of the triangle. If the triangle is degenerate or the vertex is
   * outside the prism, returns false. */
  static bool dist_sq_vert_tri_impl(const TriPrecalc &tri,
                                    const TriPrecalc &tri_vert,
                                    const uint v_other,
                                    float &r_dist_sq)
  {
    if (tri.is_degenerate) {
      return false;
    }
    const float3 &v = tri_vert.v[v_other];
    if (!is_point_on_tri_prism_impl(tri, v)) {
      return false;
    }
    float dist_v_to_plane = tri_vert.dist_to_plane_other[v_other];
    r_dist_sq = math::square(dist_v_to_plane);
    return true;
  }

  /* Determines if an edge intersects a triangle's plane within the triangle's prism. */
  static bool isect_edge_tri_impl(const TriPrecalc &tri,
                                  const TriPrecalc &tri_edge,
                                  const uint e,
                                  float &r_lambda)
  {
    if (tri.is_degenerate) {
      return false;
    }
    const int e_next = e == 2 ? 0 : e + 1;
    const float3 &v0 = tri_edge.v[e];
    const float3 &v1 = tri_edge.v[e_next];
    float dist_v0_to_plane = tri_edge.dist_to_plane_other[e];
    float dist_v1_to_plane = tri_edge.dist_to_plane_other[e_next];
    if ((dist_v0_to_plane < 0.0f) == (dist_v1_to_plane < 0.0f)) {
      return false;
    }
    r_lambda = dist_v0_to_plane / (dist_v0_to_plane - dist_v1_to_plane);
    float3 isect_point = math::interpolate(v0, v1, r_lambda);
    if (!is_point_on_tri_prism_impl(tri, isect_point)) {
      return false;
    }
    return true;
  }

  /* Determines the closest vertex on an edge to a given intersection point and
   * returns the index of this vertex if it falls within a specified minimum distance.
   * Returns -1 if neither vertex is within `dist_sq_min`. */
  int snap_isect_edge_on_vert(uint tri, uint e, uint e_next, float lambda, float dist_sq_min)
  {
    float e_len_sq = tri_[tri].e_dir_len_sq[e];
    if (lambda <= 0.5f) {
      float dist_sq = math::square(lambda) * e_len_sq;
      if (dist_sq <= dist_sq_min) {
        return int(e);
      }
    }
    else {
      float dist_sq = math::square(1.0f - lambda) * e_len_sq;
      if (dist_sq <= dist_sq_min) {
        return int(e_next);
      }
    }
    return -1;
  }

  float dist_sq_vert0_vert1(const uint v0, const uint v1)
  {
    return this->dir_other_len_sq[v0][v1];
  }

  bool dist_sq_vert0_edge1(const uint v0, const uint e1, float &r_dist_sq, float &r_lamda)
  {
    return dist_sq_vert_edge_impl(
        tri_[1], e1, -this->dir_other[v0][e1], this->dir_other_len_sq[v0][e1], r_dist_sq, r_lamda);
  }

  bool dist_sq_vert1_edge0(const uint v1, const uint e0, float &r_dist_sq, float &r_lamda)
  {
    return dist_sq_vert_edge_impl(
        tri_[0], e0, this->dir_other[e0][v1], this->dir_other_len_sq[e0][v1], r_dist_sq, r_lamda);
  }

  bool dist_sq_edge0_edge1(
      uint e0, uint e1, float &r_dist_sq, float &r_lambda_a, float &r_lambda_b)
  {
    float a_len_sq = tri_[0].e_dir_len_sq[e0];
    float b_len_sq = tri_[1].e_dir_len_sq[e1];
    if (a_len_sq <= FLT_EPSILON || b_len_sq <= FLT_EPSILON) {
      /* Degenerate. */
      return false;
    }

    const float3 &dir_a = tri_[0].e_dir[e0];
    const float3 &dir_b = tri_[1].e_dir[e1];
    float3 cross = math::cross(dir_a, dir_b);
    float cross_len_sq = math::length_squared(cross);
    if (UNLIKELY(cross_len_sq < FLT_EPSILON)) {
      /* Parallel lines. */
      return false;
    }

    const float3 dir_e1_e0 = -this->dir_other[e0][e1];
    float dot = math::dot(dir_e1_e0, cross);
    r_dist_sq = math::square(dot) / cross_len_sq;

    float3 c = cross - dir_e1_e0;
    float3 cray0 = math::cross(c, dir_b);
    float dot_cross_a = dot_v3v3(cray0, cross);
    if (!IN_RANGE(dot_cross_a, 0.0f, cross_len_sq)) {
      return false;
    }
    float3 cray1 = math::cross(c, dir_a);
    float dot_cross_b = dot_v3v3(cray1, cross);
    if (!IN_RANGE(dot_cross_b, 0.0f, cross_len_sq)) {
      return false;
    }
    r_lambda_a = dot_cross_a / cross_len_sq;
    r_lambda_b = dot_cross_b / cross_len_sq;
    return true;
  }

  bool dist_sq_vert0_tri1(uint v, float &dist_sq)
  {
    return dist_sq_vert_tri_impl(tri_[1], tri_[0], v, dist_sq);
  }

  bool dist_sq_vert1_tri0(uint v, float &dist_sq)
  {
    return dist_sq_vert_tri_impl(tri_[0], tri_[1], v, dist_sq);
  }

  bool isect_edge0_on_tri1(uint e, float &r_lambda)
  {
    return isect_edge_tri_impl(tri_[1], tri_[0], e, r_lambda);
  }

  bool isect_edge1_on_tri0(uint e, float &r_lambda)
  {
    return isect_edge_tri_impl(tri_[0], tri_[1], e, r_lambda);
  }

  bool is_any_degenerate(float dist_sq_min)
  {
    if (tri_[0].is_degenerate || tri_[1].is_degenerate) {
      return true;
    }

    /* #TriPrecalc::is_degenerate does not consider cases where the vert is on the opposite edge.
     */
    for (uint t : IndexRange(2)) {
      for (uint e_prev = 2, e = 0; e < 3; e_prev = e++) {
        float dist_sq, lambda;
        if (dist_sq_vert_edge_impl(tri_[t],
                                   e,
                                   -tri_[t].e_dir[e_prev],
                                   tri_[t].e_dir_len_sq[e_prev],
                                   dist_sq,
                                   lambda) &&
            dist_sq <= dist_sq_min)
        {
          return true;
        }
      }
    }
    return false;
  }
};

enum eIsectTriTriMode {
  NO_ISECT = 0,
  ISECT,
  COPLANAR,
};

#define PAIRS_LEN_MAX 6
static eIsectTriTriMode intersect_tri_tri_impl(IsectMap *s,
                                               BMVert *fv_a[3],
                                               BMVert *fv_b[3],
                                               int a_index,
                                               int b_index,
                                               IsectPair *r_pairs[PAIRS_LEN_MAX][2],
                                               int *r_edge_size)
{
  float3 tri_a[3] = {UNPACK3_EX(, fv_a, ->co)};
  float3 tri_b[3] = {UNPACK3_EX(, fv_b, ->co)};

  IsectTriTriPrecalc tri_precalc(tri_a, tri_b, s->eps_sq);

  struct IsectResult {
    struct Elem {
      eIsectElemType type;
      uint index;
      float lambda;
    } elem[2];
    float dist_sq;
    IsectResult(IsectResult::Elem elem_a, IsectResult::Elem elem_b, float dist_sq = 0.0f)
        : dist_sq(dist_sq)
    {
      this->elem[0] = elem_a;
      this->elem[1] = elem_b;
    }
  };

  Vector<IsectResult, 9> pairs;

  bool is_va_on_tri[3] = {false, false, false};
  bool is_vb_on_tri[3] = {false, false, false};
  bool is_va_on_vb[3][3] = {{false, false, false}, {false, false, false}, {false, false, false}};
  bool is_va_on_eb[3][3] = {{false, false, false}, {false, false, false}, {false, false, false}};
  bool is_vb_on_ea[3][3] = {{false, false, false}, {false, false, false}, {false, false, false}};

  int ea_isect_len[3] = {0, 0, 0};
  int eb_isect_len[3] = {0, 0, 0};

  float dist_sq_min = s->eps_sq;
  float dist_sq_test = 0.0f;

  auto update_distance = [&](float dist_sq) {
    if (pairs.size() <= PAIRS_LEN_MAX) {
      dist_sq_test = std::max(dist_sq_test, dist_sq);
    }
    else {
      dist_sq_min = std::min(dist_sq_min, dist_sq);
    }
  };
  auto remove_vert_edge_pairs = [&](uint v, bool is_b) {
    /* `pairs.size()` can change in the loop. */
    for (int i = 0; i < pairs.size(); i++) {
      IsectResult::Elem *elem = pairs[i].elem;
      if (elem[is_b].type == VERT && elem[is_b].index == v && elem[!is_b].type == EDGE) {
        if (is_b) {
          uint e = elem[0].index;
          BLI_assert(is_vb_on_ea[v][e]);
          is_vb_on_ea[v][e] = false;
          ea_isect_len[e]--;
        }
        else {
          uint e = elem[1].index;
          BLI_assert(is_va_on_eb[v][e]);
          is_va_on_eb[v][e] = false;
          eb_isect_len[e]--;
        }
        pairs.remove_and_reorder(i);
      }
    }
  };

  /* Vert x Vert. */
  for (uint ia = 0; ia < 3; ia++) {
    for (uint ib = 0; ib < 3; ib++) {
      float dist_sq;
      if ((dist_sq = tri_precalc.dist_sq_vert0_vert1(ia, ib)) <= dist_sq_min) {
        is_va_on_tri[ia] = true;
        is_vb_on_tri[ib] = true;
        is_va_on_vb[ia][ib] = true;
        pairs.append(IsectResult({VERT, ia}, {VERT, ib}, dist_sq));
        update_distance(dist_sq);
      }
    }
  }
  if (pairs.size() >= PAIRS_LEN_MAX) {
    goto return_best;
  }

  /* Vert x Edge. */
  for (uint ia = 0; ia < 3; ia++) {
    if (is_va_on_tri[ia]) {
      continue;
    }
    for (uint ib = 2, ib_next = 0; ib_next < 3; ib = ib_next++) {
      float dist_sq, lambda;
      if (tri_precalc.dist_sq_vert0_edge1(ia, ib, dist_sq, lambda) && dist_sq <= dist_sq_min) {
        update_distance(dist_sq);
        is_va_on_tri[ia] = true;
        int vb = tri_precalc.snap_isect_edge_on_vert(1, ib, ib_next, lambda, s->eps_sq);
        if (vb != -1) {
          is_va_on_tri[ia] = true;
          is_vb_on_tri[vb] = true;
          is_va_on_vb[ia][vb] = true;

          /* Remove Vert x Edge cases for this vertex. */
          remove_vert_edge_pairs(ia, false);
          remove_vert_edge_pairs(vb, true);
          pairs.append(IsectResult({VERT, ia}, {VERT, uint(vb)}, dist_sq));

          /* Break to avoid adding a Vert x Edge Case. */
          break;
        }
        else {
          is_va_on_eb[ia][ib] = true;
          eb_isect_len[ib]++;
          pairs.append(IsectResult({VERT, ia}, {EDGE, ib, lambda}, dist_sq));
        }
      }
    }
  }
  for (uint ib = 0; ib < 3; ib++) {
    if (is_vb_on_tri[ib]) {
      continue;
    }
    for (uint ia = 2, ia_next = 0; ia_next < 3; ia = ia_next++) {
      float dist_sq, lambda;
      if (tri_precalc.dist_sq_vert1_edge0(ib, ia, dist_sq, lambda) && dist_sq <= dist_sq_min) {
        update_distance(dist_sq);
        is_vb_on_tri[ib] = true;
        int va = tri_precalc.snap_isect_edge_on_vert(0, ia, ia_next, lambda, s->eps_sq);
        if (va != -1) {
          is_va_on_tri[va] = true;
          is_vb_on_tri[ib] = true;
          is_va_on_vb[va][ib] = true;

          /* Remove Vert x Edge cases for this vertex. */
          remove_vert_edge_pairs(va, false);
          remove_vert_edge_pairs(ib, true);

          /* Break to avoid adding a Vert x Edge Case. */
          pairs.append(IsectResult({VERT, uint(va)}, {VERT, ib}, dist_sq));
          break;
        }
        else {
          is_vb_on_ea[ib][ia] = true;
          ea_isect_len[ia]++;
          pairs.append(IsectResult({EDGE, ia, lambda}, {VERT, ib}, dist_sq));
        }
      }
    }
  }
  if (pairs.size() >= PAIRS_LEN_MAX) {
    goto return_best;
  }

  /* Edge x Edge. */

  for (uint ia = 2, ia_next = 0; ia_next < 3; ia = ia_next++) {
    if (ea_isect_len[ia] >= 2 || (ea_isect_len[ia] && (is_va_on_tri[ia] || is_va_on_tri[ia_next])))
    {
      /* The edge already has two or more intersections. */
      continue;
    }
    for (uint ib = 2, ib_next = 0; ib_next < 3; ib = ib_next++) {
      if (eb_isect_len[ib] >= 2 ||
          (eb_isect_len[ib] && (is_vb_on_tri[ib] || is_vb_on_tri[ib_next])))
      {
        /* The edge already has two or more intersections. */
        continue;
      }
      if (is_va_on_eb[ia][ib] || is_va_on_eb[ia_next][ib] || is_vb_on_ea[ib][ia] ||
          is_vb_on_ea[ib_next][ia] || is_va_on_vb[ia][ib] || is_va_on_vb[ia][ib_next] ||
          is_va_on_vb[ia_next][ib] || is_va_on_vb[ia_next][ib_next])
      {
        /* Avoid repeated pairs. */
        continue;
      }
      float dist_sq, lambda_a, lambda_b;
      if (tri_precalc.dist_sq_edge0_edge1(ia, ib, dist_sq, lambda_a, lambda_b) &&
          dist_sq <= dist_sq_min)
      {
        update_distance(dist_sq);
        int va = tri_precalc.snap_isect_edge_on_vert(0, ia, ia_next, lambda_a, s->eps_sq);
        int vb = tri_precalc.snap_isect_edge_on_vert(1, ib, ib_next, lambda_b, s->eps_sq);
        if (va != -1 && vb != -1) {
          if (is_va_on_vb[va][vb]) {
            /* Avoid repeated pairs. */
            continue;
          }
          is_va_on_tri[va] = true;
          is_vb_on_tri[vb] = true;
          is_va_on_vb[va][vb] = true;

          /* Remove Vert x Edge cases for this vertex. */
          remove_vert_edge_pairs(va, false);
          remove_vert_edge_pairs(vb, true);
          pairs.append(IsectResult({VERT, uint(va)}, {VERT, uint(vb)}, dist_sq));
        }
        else if (va != -1) {
          is_va_on_tri[va] = true;
          is_va_on_eb[va][ib] = true;
          eb_isect_len[ib]++;
          pairs.append(IsectResult({VERT, uint(va)}, {EDGE, ib, lambda_b}, dist_sq));
        }
        else if (vb != -1) {
          is_vb_on_tri[vb] = true;
          is_vb_on_ea[vb][ia] = true;
          ea_isect_len[ia]++;
          pairs.append(IsectResult({EDGE, ia, lambda_a}, {VERT, uint(vb)}, dist_sq));
        }
        else {
          ea_isect_len[ia]++;
          eb_isect_len[ib]++;
          pairs.append(IsectResult({EDGE, ia, lambda_a}, {EDGE, ib, lambda_b}, dist_sq));
        }
      }
    }
  }
  if (pairs.size() >= PAIRS_LEN_MAX) {
    goto return_best;
  }

  /* Vert x Triangle. */

  for (uint ia_prev = 2, ia = 0; ia < 3; ia_prev = ia++) {
    if (is_va_on_tri[ia] || ea_isect_len[ia_prev] >= 2 || ea_isect_len[ia] >= 2) {
      continue;
    }
    float dist_sq;
    if (tri_precalc.dist_sq_vert0_tri1(ia, dist_sq) && dist_sq <= dist_sq_min) {
      is_va_on_tri[ia] = true;
      pairs.append(IsectResult({VERT, ia}, {NONE, uint(b_index)}, dist_sq));
      update_distance(dist_sq);
    }
  }
  for (uint ib_prev = 2, ib = 0; ib < 3; ib_prev = ib++) {
    if (is_vb_on_tri[ib] || eb_isect_len[ib_prev] >= 2 || eb_isect_len[ib] >= 2) {
      continue;
    }
    float dist_sq;
    if (tri_precalc.dist_sq_vert1_tri0(ib, dist_sq) && dist_sq <= dist_sq_min) {
      is_vb_on_tri[ib] = true;
      pairs.append(IsectResult({NONE, uint(a_index)}, {VERT, ib}, dist_sq));
      update_distance(dist_sq);
    }
  }
  if (pairs.size() >= 2) {
    /* Here it is not #PAIRS_LEN_MAX because if there are already 2 or more intersections, there
     * should not be any edge x tri intersections. */
    goto return_best;
  }

  /* Edge x Triangle. */

  for (uint ia = 2, ia_next = 0; ia_next < 3; ia = ia_next++) {
    if (is_va_on_tri[ia] || is_va_on_tri[ia_next] || ea_isect_len[ia]) {
      continue;
    }
    float lambda;
    if (tri_precalc.isect_edge0_on_tri1(ia, lambda)) {
      ea_isect_len[ia]++;
      pairs.append(IsectResult({EDGE, ia, lambda}, {NONE, uint(b_index)}));
    }
  }
  for (uint ib = 2, ib_next = 0; ib_next < 3; ib = ib_next++) {
    if (is_vb_on_tri[ib] || is_vb_on_tri[ib_next] || eb_isect_len[ib]) {
      continue;
    }
    float lambda;
    if (tri_precalc.isect_edge1_on_tri0(ib, lambda)) {
      eb_isect_len[ib]++;
      pairs.append(IsectResult({NONE, uint(a_index)}, {EDGE, ib, lambda}));
    }
  }

return_best:
  BLI_assert(pairs.size() <= 9);
  Vector<IsectResult, PAIRS_LEN_MAX> best_pairs;

  /* Take the best pairs with the best distances. Also, if the same vertex intersects more than
   * once, choose only the pair with the shortest distance. */
  for (int i = 0; i < pairs.size(); i++) {
    const IsectResult &pair = pairs[i];
    if (pair.dist_sq > dist_sq_min) {
      continue;
    }
    if (!ELEM(VERT, pair.elem[0].type, pair.elem[1].type)) {
      best_pairs.append(pair);
      continue;
    }

    int i_best = i;
    for (int i_test = i + 1; i_test < pairs.size(); i_test++) {
      /* Find pairs with the same vertex. */
      const IsectResult &pair_best = pairs[i_best];
      const IsectResult &pair_test = pairs[i_test];
      if (pair_test.dist_sq > dist_sq_min) {
        continue;
      }
      if (!ELEM(VERT, pair_test.elem[0].type, pair_test.elem[1].type)) {
        continue;
      }
      if ((pair_test.elem[0].type == VERT && pair_best.elem[0].type == VERT &&
           pair_test.elem[0].index == pair_best.elem[0].index) ||
          (pair_test.elem[1].type == VERT && pair_best.elem[1].type == VERT &&
           pair_test.elem[1].index == pair_best.elem[1].index))
      {
        if (pair_test.dist_sq < pair_best.dist_sq) {
          i_best = i_test;
        }
        else {
          /* Remove `pair_test` so we won't test it again. */
          pairs.remove_and_reorder(i_test);
          i_test--;
        }
      }
    }
    best_pairs.append(pairs[i_best]);
    if (i != i_best) {
      /* No need to test this pair again. */
      pairs.remove_and_reorder(i_best);
    }
  }
  if (best_pairs.size() < 2) {
    return NO_ISECT;
  }

  /* Sort to keep pairs with the same edge in the same isect edge. */
  std::sort(
      best_pairs.begin(), best_pairs.end(), [](const IsectResult &pa, const IsectResult &pb) {
        if (pa.elem[0].type == EDGE && pb.elem[0].type == EDGE) {
          return pa.elem[0].index < pb.elem[0].index;
        }
        if (pa.elem[1].type == EDGE && pb.elem[1].type == EDGE) {
          return pa.elem[1].index < pb.elem[1].index;
        }
        if (pa.elem[0].type == EDGE) {
          return true;
        }
        if (pb.elem[0].type == EDGE) {
          return false;
        }
        if (pa.elem[1].type == EDGE) {
          return true;
        }
        if (pb.elem[1].type == EDGE) {
          return false;
        }
        return false;
      });

  int edge, edge_prev = 0;
  for (edge = 0; edge < best_pairs.size(); edge_prev = edge++) {
    const IsectResult &pair = best_pairs[edge];
    if (edge != 0) {
      r_pairs[edge][0] = r_pairs[edge_prev][1];
    }
    if (pair.elem[0].type == VERT && pair.elem[1].type == VERT) {
      r_pairs[edge][1] = s->lookup_or_add_vert_vert(fv_a[pair.elem[0].index],
                                                    fv_b[pair.elem[1].index]);
    }
    else if (pair.elem[0].type == VERT && pair.elem[1].type == EDGE) {
      uint e = pair.elem[1].index;
      uint e_next = e == 2 ? 0 : e + 1;
      r_pairs[edge][1] = s->lookup_or_add_vert_edge(
          fv_a[pair.elem[0].index], fv_b[e], fv_b[e_next], pair.elem[1].lambda);
    }
    else if (pair.elem[0].type == EDGE && pair.elem[1].type == VERT) {
      uint e = pair.elem[0].index;
      uint e_next = e == 2 ? 0 : e + 1;
      r_pairs[edge][1] = s->lookup_or_add_vert_edge(
          fv_b[pair.elem[1].index], fv_a[e], fv_a[e_next], pair.elem[0].lambda);
    }
    else if (pair.elem[0].type == EDGE && pair.elem[1].type == EDGE) {
      uint ea = pair.elem[0].index;
      uint ea_next = ea == 2 ? 0 : ea + 1;
      uint eb = pair.elem[1].index;
      uint eb_next = eb == 2 ? 0 : eb + 1;
      r_pairs[edge][1] = s->lookup_or_add_edge_edge(fv_a[ea],
                                                    fv_a[ea_next],
                                                    fv_b[eb],
                                                    fv_b[eb_next],
                                                    pair.elem[0].lambda,
                                                    pair.elem[1].lambda);
    }
    else if (pair.elem[0].type == VERT) {
      r_pairs[edge][1] = s->lookup_or_add_vert_tri(fv_a[pair.elem[0].index], pair.elem[1].index);
    }
    else if (pair.elem[1].type == VERT) {
      r_pairs[edge][1] = s->lookup_or_add_vert_tri(fv_b[pair.elem[1].index], pair.elem[0].index);
    }
    else if (pair.elem[0].type == EDGE) {
      uint e = pair.elem[0].index;
      uint e_next = e == 2 ? 0 : e + 1;
      r_pairs[edge][1] = s->lookup_or_add_edge_tri(
          fv_a[e], fv_a[e_next], pair.elem[1].index, pair.elem[0].lambda);
    }
    else {
      BLI_assert(pair.elem[1].type == EDGE);
      uint e = pair.elem[1].index;
      uint e_next = e == 2 ? 0 : e + 1;
      r_pairs[edge][1] = s->lookup_or_add_edge_tri(
          fv_b[e], fv_b[e_next], pair.elem[0].index, pair.elem[1].lambda);
    }
  }

  r_pairs[0][0] = r_pairs[edge_prev][1];
  if (best_pairs.size() == 2 || tri_precalc.is_any_degenerate(s->eps_sq)) {
    *r_edge_size = 1;
    return ISECT;
  }
  *r_edge_size = edge;
  return COPLANAR;
}

static void intersect_tri_tri(
    IsectMap *s, int a_index, int b_index, std::array<BMLoop *, 3> a, std::array<BMLoop *, 3> b)
{
  BMFace *f_a = a[0]->f;
  BMFace *f_b = b[0]->f;
  if (UNLIKELY(BM_face_share_edge_check(f_a, f_b))) {
    return;
  }

  BMVert *fv_a[3] = {UNPACK3_EX(, a, ->v)};
  BMVert *fv_b[3] = {UNPACK3_EX(, b, ->v)};

  IsectPair *pairs[PAIRS_LEN_MAX][2] = {nullptr};
  int edge_size;
  eIsectTriTriMode isect = intersect_tri_tri_impl(
      s, fv_a, fv_b, a_index, b_index, pairs, &edge_size);

  if (isect == NO_ISECT) {
    return;
  }

  FaceData *f_a_data = s->lookup_or_add_face_data(f_a, true);
  FaceData *f_b_data = s->lookup_or_add_face_data(f_b, false);

  if (isect == COPLANAR) {
    /* Add coplanar faces. */
    BLI_assert(f_a_data->is_face_a != f_b_data->is_face_a);
    f_a_data->add_face_coplanar(s, f_b);
    f_b_data->add_face_coplanar(s, f_a);
  }
  else {
    for (int edge = 0; edge < edge_size; edge++) {
      IsectEdge *isect_edge = s->lookup_or_add_isect_edge(UNPACK2(pairs[edge]));
      f_a_data->add_edge_ptr(s, &isect_edge->e);
      f_b_data->add_edge_ptr(s, &isect_edge->e);
    }
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name BMesh Manipulation.
 *
 * Algorithm for: Split edges > Merge vertices > Split faces > Identify and remove coplanar faces.
 * \{ */

/*** Utils. ***/

/* Merges two vertices into one, removing any existing edge between them if present, and splits the
 * associated face as a result of the merge operation. */
static void merge_verts_remove_edge_split_face(IsectMap *s, BMVert *v_dst, BMVert *v_src)
{
  BLI_assert(v_src->e && v_dst->e);

  /* Capture all loop pairs per Face. */
  struct SplitFace {
    BMFace *f;
    FaceData *fdata;
    BMLoop *l_a;
    BMLoop *l_b;
  };

  BMIter iter;
  BMLoop *l_a, *l_b;
  BLI_buffer_declare_static(SplitFace, split_faces, BLI_BUFFER_NOP, 2);
  BM_ITER_ELEM (l_a, &iter, v_src, BM_LOOPS_OF_VERT) {
    BMFace *f = l_a->f;
    l_b = BM_face_vert_share_loop(f, v_dst);
    if (l_b) {
      const BMFaceFaceData *fkey = s->face_data.lookup_key_ptr({f});
      SplitFace f_item;
      f_item.f = f;
      f_item.fdata = fkey ? fkey->fdata : nullptr;
      f_item.l_a = l_a;
      f_item.l_b = l_b;
      BLI_buffer_append(&split_faces, SplitFace, f_item);
    }
  }

  /* Split Faces. */
  BMEdge *e_kill = BM_edge_exists(v_dst, v_src);
  if (split_faces.count) {
    for (int i : IndexRange(split_faces.count)) {
      SplitFace &f_item = static_cast<SplitFace *>(split_faces.data)[i];
      FaceData *fdata = f_item.fdata;
      if (BM_loop_is_adjacent(f_item.l_a, f_item.l_b)) {
        continue;
      }
      BMLoop *l_del;
      BMFace *f_new = BM_face_split(
          s->bm, f_item.f, f_item.l_a, f_item.l_b, &l_del, nullptr, false);
      if (e_kill) {
        BM_edge_splice(s->bm, e_kill, l_del->e);
      }
      else {
        e_kill = l_del->e;
      }
      if (fdata && f_new->len > 3) {
        s->add_face_data(f_new, fdata);
        s->face_coplanar_add(f_new, fdata);
      }
      SplitFace f_item_new;
      f_item_new.f = f_new;
      f_item_new.fdata = fdata;
      f_item_new.l_a = nullptr;
      f_item_new.l_b = nullptr;
      BLI_buffer_append(&split_faces, SplitFace, f_item_new);
    }
  }

  /* Merge Vertices (Removing the Edge). */
  if (e_kill) {
    bmesh_kernel_join_vert_kill_edge(s->bm, e_kill, v_src, true, false, false);
    for (const SplitFace &f_item :
         Span(static_cast<SplitFace *>(split_faces.data), split_faces.count))
    {
      if (f_item.f->len < 3) {
        if (f_item.fdata) {
          s->face_coplanar_remove(f_item.f, f_item.fdata);
          s->face_data.remove_as(f_item.f);
#ifdef DEBUG_ISECT
          printf("Removed Face Key: %p\n", f_item.f);
#endif
        }
        BM_face_kill(s->bm, f_item.f);
      }
    }
  }
  else {
    BM_vert_splice(s->bm, v_dst, v_src);
  }
  BLI_buffer_free(&split_faces);
}

/*** Split and Merge Edge x Edge cases. ***/

static int sort_cmp_by_lambda_fn(const void *a, const void *b, void *data)
{
  const BMEdge *e = static_cast<const BMEdge *>(data);
  const IsectPair *pair_a = *(IsectPair **)a;
  const IsectPair *pair_b = *(IsectPair **)b;

  float lambda_a = (pair_a->elem_b.e == e) ? pair_a->lambda_b : pair_a->lambda_a;
  float lambda_b = (pair_b->elem_b.e == e) ? pair_b->lambda_b : pair_b->lambda_a;

  if (lambda_a > lambda_b) {
    return 1;
  }
  if (lambda_a < lambda_b) {
    return -1;
  }

  /* Equal lambda values. They should be merged to avoid duplicate vertices, but this is rare and
   * not invalid. */
  return 0;
}

struct IsectEdgeDataIter {
  IsectPair **edge_pair_table;
  BMEdge **edge_affected;

  uint edge_pair_table_len;
  uint edge_in_pair_len;
  uint edge_affected_len;
};

static void split_edge_data_iter_set(IsectEdgeDataIter *isect_data, IsectPair *pair)
{
  if (ELEM(EDGE, pair->type_a, pair->type_b)) {
    isect_data->edge_pair_table[isect_data->edge_pair_table_len++] = pair;
  }
  if (pair->type_a == EDGE) {
    BMEdge *e = pair->elem_a.e;
    int cuts_len = BM_elem_index_get(e);
    BM_elem_index_set(e, cuts_len + 1);
    isect_data->edge_in_pair_len++;
    if (cuts_len == 0) {
      isect_data->edge_affected[isect_data->edge_affected_len++] = e;
    }
  }
  if (pair->type_b == EDGE) {
    BMEdge *e = pair->elem_b.e;
    int cuts_len = BM_elem_index_get(e);
    BM_elem_index_set(e, cuts_len + 1);
    isect_data->edge_in_pair_len++;
    if (cuts_len == 0) {
      isect_data->edge_affected[isect_data->edge_affected_len++] = e;
    }
  }
}

static void split_and_merge_edge_edge_cases(IsectMap *s)
{
  BMesh *bm = s->bm;
  BMIter iter;
  BMEdge *e;
  BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
    BM_elem_index_set(e, 0);
    BM_elem_flag_disable(e, EDGE_ISECT_FLAG);
  }
  bm->elem_index_dirty |= BM_EDGE;

  uint edge_pair_len_alloc = s->vert_edge.size() + s->edge_tri.size() + s->edge_edge.size();

  IsectEdgeDataIter isect_data = {nullptr};
  isect_data.edge_pair_table = (IsectPair **)MEM_mallocN(sizeof(IsectPair *) * edge_pair_len_alloc,
                                                         __func__);
  isect_data.edge_affected = (BMEdge **)MEM_mallocN(
      sizeof(BMEdge *) * (edge_pair_len_alloc + s->edge_edge.size()), __func__);

  for (IsectPair *pair : s->vert_edge.values()) {
    split_edge_data_iter_set(&isect_data, pair);
  }
  for (IsectPair *pair : s->edge_tri.values()) {
    split_edge_data_iter_set(&isect_data, pair);
  }
  for (IsectPair *pair : s->edge_edge.values()) {
    split_edge_data_iter_set(&isect_data, pair);
  }

  int offset = 0;
  for (uint i = 0; i < isect_data.edge_affected_len; i++) {
    e = isect_data.edge_affected[i];
    int len = BM_elem_index_get(e);
    BM_elem_index_set(e, offset);
    offset += len;
  }

  IsectPair **edge_pair = (IsectPair **)MEM_mallocN(
      sizeof(*edge_pair) * isect_data.edge_in_pair_len, __func__);
  for (uint i = 0; i < isect_data.edge_pair_table_len; i++) {
    IsectPair *pair = isect_data.edge_pair_table[i];
    if (pair->type_a == EDGE) {
      e = pair->elem_a.e;
      uint slot = BM_elem_index_get(e);
      BM_elem_index_set(e, slot + 1);
      edge_pair[slot] = pair;
    }
    if (pair->type_b == EDGE) {
      e = pair->elem_b.e;
      uint slot = BM_elem_index_get(e);
      BM_elem_index_set(e, slot + 1);
      edge_pair[slot] = pair;
    }
  }

  MEM_freeN(isect_data.edge_pair_table);

  offset = 0;
  for (uint i = 0; i < isect_data.edge_affected_len; i++) {
    e = isect_data.edge_affected[i];
    int end = BM_elem_index_get(e);
    int len = end - offset;
    if (len > 1) {
      BLI_qsort_r(&edge_pair[offset], len, sizeof(*edge_pair), sort_cmp_by_lambda_fn, e);
    }

    float lambda, lambda_prev = 0.0f;
    do {
      IsectPair *pair = edge_pair[offset];
      float pair_lambda = (pair->elem_b.e == e) ? pair->lambda_b : pair->lambda_a;

      lambda = (pair_lambda - lambda_prev) / (1.0f - lambda_prev);
      lambda_prev = pair_lambda;

      BMEdge *e_new;
      BMVert *v_new = BM_edge_split(bm, e, e->v1, &e_new, lambda);
      BLI_assert(e_new->v2 == v_new);
      BLI_assert(e->v1 == v_new);

      if (pair->elem_b.e == e) {
        /* It would be a swap, but the values ​​of element A will be replaced anyway. */
        pair->elem_b.ptr = pair->elem_a.ptr;
        pair->type_b = pair->type_a;
        pair->lambda_b = pair->lambda_a;
      }

      pair->type_a = VERT;
      if (pair->type_b == VERT) {
        BMEdge *e_dst_a = BM_edge_exists(e_new->v1, pair->elem_b.v);
        BMEdge *e_dst_b = BM_edge_exists(pair->elem_b.v, e->v2);
        merge_verts_remove_edge_split_face(s, pair->elem_b.v, v_new);
        if (e_dst_a) {
          BM_edge_splice(bm, e_dst_a, e_new);
        }
        if (e_dst_b) {
          BM_edge_splice(bm, e, e_dst_b);
        }

        pair->elem_a.v = pair->elem_b.v;
#ifdef DEBUG_ISECT
        pair->elem_b.v = nullptr;
#endif
        pair->type_b = NONE;
      }
      else {
        BLI_assert(ELEM(pair->type_b, NONE_MAYBE_DISSOLVE, NONE, EDGE));
        pair->elem_a.v = v_new;
        if (pair->type_b == NONE) {
          pair->type_b = NONE_MAYBE_DISSOLVE;
        }
      }
    } while (++offset != end);
  }

  MEM_freeN(isect_data.edge_affected);
  MEM_freeN(edge_pair);
}

/*** Merge Vert x Vert cases. ***/

static BMVert *vert_find_merge_dst(IsectPair **vert_pair_table, BMVert *v)
{
  BMVert *v_iter = v;
  int pair_i = BM_elem_index_get(v);
  while (pair_i >= 0) {
    v_iter = vert_pair_table[pair_i]->elem_a.v;
    pair_i = BM_elem_index_get(v_iter);
    BLI_assert(v_iter != v);
  }
  return v_iter;
}

/**
 * Merges \a v_src into \a v_dst, removing \a v_src.
 */
static void bm_vert_splice_and_remove_doubles(IsectMap *s, BMVert *v_dst, BMVert *v_src)
{
  struct EdgeSpliceLink {
    EdgeSpliceLink *next;
    BMEdge *e_dst;
    BMEdge *e_src;
  } *edge_splice_link = nullptr;

  BMIter iter;
  BMEdge *e;
  BM_ITER_ELEM (e, &iter, v_dst, BM_EDGES_OF_VERT) {
    BMVert *v_other = BM_edge_other_vert(e, v_dst);
    if (v_other == v_src) {
      continue;
    }
    BMEdge *e_src = BM_edge_exists(v_other, v_src);
    if (e_src) {
      EdgeSpliceLink *next = edge_splice_link;
      edge_splice_link = (EdgeSpliceLink *)alloca(sizeof(*edge_splice_link));
      edge_splice_link->next = next;
      edge_splice_link->e_dst = e;
      edge_splice_link->e_src = e_src;
    }
  }

  merge_verts_remove_edge_split_face(s, v_dst, v_src);
  while (edge_splice_link) {
    BM_edge_splice(s->bm, edge_splice_link->e_dst, edge_splice_link->e_src);
    edge_splice_link = edge_splice_link->next;
  }
}

static void merge_vert_vert_cases(IsectMap *s)
{
  BMesh *bm = s->bm;
  BMIter iter;
  BMVert *v;
  BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
    BM_elem_index_set(v, -1);
    BM_elem_flag_disable(v, VERT_DISSOLVE_FLAG);
  }
  bm->elem_index_dirty |= BM_VERT;

  for (IsectPair *pair : s->edge_edge.values()) {
    if (pair->type_b == NONE_MAYBE_DISSOLVE) {
      BM_elem_flag_enable(pair->elem_a.v, VERT_DISSOLVE_FLAG);
    }
  }

  if (s->vert_vert.is_empty()) {
    return;
  }

  IsectPair **vert_pair_table = (IsectPair **)MEM_mallocN(
      sizeof(*vert_pair_table) * s->vert_vert.size(), __func__);

  uint vert_pair_table_len = 0;
  for (IsectPair *pair : s->vert_vert.values()) {
    BLI_assert((pair->type_a == VERT) && (ELEM(pair->type_b, VERT, NONE)));
    vert_pair_table[vert_pair_table_len++] = pair;
#ifdef DEBUG_ISECT
    /* Make sure no element in elem_a is in elem_b if elem_b is not NONE. */
    if (pair->type_b == VERT) {
      BM_elem_index_set(pair->elem_b.v, -2);
      BLI_assert(BM_elem_index_get(pair->elem_a.v) == -1);
    }
#endif
  }
  BLI_assert(vert_pair_table_len == (uint)s->vert_vert.size());

#ifdef DEBUG_ISECT
  for (IsectPair *pair : s->edge_edge.values()) {
    BLI_assert(pair->type_b != VERT);
  }
#endif

  /**
   * Rearrange merged vert chain to find the final vert for each group.
   * elem_a          : | 2 3 3 2 3 5 3 | -> | 2 3 3 3 (3) 5 5 | -> | 5 5 5 5 (5) 5 5 |
   * elem_b (deleted): | 0 0 1 4 4 4 6 | -> | 0 2 1 4 (3) 3 6 | -> | 0 2 1 4 (X) 3 6 |
   */
  for (uint i = 0; i < vert_pair_table_len; i++) {
    IsectPair *pair = vert_pair_table[i];
    if (pair->type_b == NONE) {
      continue;
    }
    BMVert *v_a = pair->elem_a.v;
    BMVert *v_b = pair->elem_b.v;
    int v_a_index = BM_elem_index_get(v_a);
    int v_b_index = BM_elem_index_get(v_b);
    if (v_a_index >= 0) {
      /* `v_a` will already be deleted. Replace with the target. */
      BMVert *v_dst = vert_pair_table[v_a_index]->elem_a.v;
      pair->elem_a.v = vert_find_merge_dst(vert_pair_table, v_dst);
    }
    if (v_b_index >= 0) {
      /* `v_b` will already be deleted. Replace with the target. */
      BMVert *v_dst = vert_pair_table[v_b_index]->elem_a.v;
      pair->elem_b.v = vert_find_merge_dst(vert_pair_table, v_dst);
      if (pair->elem_b.v == pair->elem_a.v) {
        /* It is already the target, ignore merge. */
        pair->elem_b.v = nullptr;
      }
      else {
        BM_elem_index_set(pair->elem_b.v, i);
      }
    }
    BM_elem_index_set(v_b, i);
  }

  /* Avoid referencing vertices that will be deleted. */
  for (IsectPair *pair : Span(vert_pair_table, vert_pair_table_len)) {
    pair->elem_a.v = vert_find_merge_dst(vert_pair_table, pair->elem_a.v);
    BLI_assert(pair->elem_a.v != pair->elem_b.v);
  }
  for (IsectPair *pair : s->vert_tri.values()) {
    BLI_assert(pair->type_a == VERT);
    pair->elem_a.v = vert_find_merge_dst(vert_pair_table, pair->elem_a.v);
  }
  for (IsectPair *pair : s->vert_edge.values()) {
    BLI_assert(pair->type_a == VERT);
    pair->elem_a.v = vert_find_merge_dst(vert_pair_table, pair->elem_a.v);
  }

  /* Now merge verts. */
  for (IsectPair *pair : Span(vert_pair_table, vert_pair_table_len)) {
    if (pair->elem_b.v) {
      bm_vert_splice_and_remove_doubles(s, pair->elem_a.v, pair->elem_b.v);
    }
#ifdef DEBUG_ISECT
    pair->elem_b.v = nullptr;
#endif
  }

  MEM_freeN(vert_pair_table);
}

/*** Split Faces. ***/

static bool prepare_edgenet_for_face(BMesh *bm,
                                     BMFace *f,
                                     BMEdge **edge_net_init,
                                     uint edge_net_init_len,
                                     bool fix_intersections,
                                     bool use_island_connect,
                                     bool use_partial_connect,
                                     MemArena *mem_arena,
                                     BMEdge ***r_edge_net_new,
                                     uint *r_edge_net_new_len)
{
  bool ok = false;

  if (fix_intersections) {
    /* This is a rare case, but faces from the same operand mesh can intersect and create edgenets
     * with edges that also intersect. To improve the result, split the edges that intersect. */
    struct IsectEdge {
      struct Pair {
        int e;
        float lambda;
      } pair[2];
    };
    Vector<IsectEdge> edges_pair;

    const float3 &f_no = f->no;
    for (int ia : IndexRange(edge_net_init_len - 1)) {
      BMEdge *ea = edge_net_init[ia];
      const float3 &ea_v1 = ea->v1->co;
      const float3 &ea_v2 = ea->v2->co;
      float3 ea_tangent = math::cross(ea_v2 - ea_v1, f_no);
      if (math::is_zero(ea_tangent)) {
        continue;
      }

      int ia_next = ia + 1;
      for (int ib : IndexRange(ia_next, edge_net_init_len - ia_next)) {
        BMEdge *eb = edge_net_init[ib];
        if (BM_edge_share_vert(ea, eb)) {
          continue;
        }
        const float3 &eb_v1 = eb->v1->co;
        const float3 &eb_v2 = eb->v2->co;
        float dist_eb1 = math::dot(eb_v1 - ea_v1, ea_tangent);
        float dist_eb2 = math::dot(eb_v2 - ea_v1, ea_tangent);
        if ((dist_eb1 >= 0.0f) == (dist_eb2 >= 0.0f)) {
          continue;
        }
        float3 eb_tangent = math::cross(eb_v2 - eb_v1, f_no);
        float dist_ea1 = math::dot(ea_v1 - eb_v1, eb_tangent);
        float dist_ea2 = math::dot(ea_v2 - eb_v1, eb_tangent);
        if ((dist_ea1 >= 0.0f) == (dist_ea2 >= 0.0f)) {
          continue;
        }
        float lambda_a = dist_ea1 / (dist_ea1 - dist_ea2);
        float lambda_b = dist_eb1 / (dist_eb1 - dist_eb2);
        edges_pair.append({IsectEdge::Pair{ia, lambda_a}, {ib, lambda_b}});
      }
    }
    if (edges_pair.size()) {
      uint edge_net_new_len = edge_net_init_len + edges_pair.size();
      BMEdge **edge_net_new = static_cast<BMEdge **>(
          BLI_memarena_alloc(mem_arena, sizeof(*edge_net_new) * edge_net_new_len));
      BMEdge **edge_net_new_iter = edge_net_new;
      IsectEdge::Pair *edges_flat_iter = reinterpret_cast<IsectEdge::Pair *>(edges_pair.data());
      for (int i : IndexRange(edge_net_init_len)) {
        if (i == edges_flat_iter->e) {
          edges_flat_iter++;
          continue;
        }
        *edge_net_new_iter++ = edge_net_init[i];
      }

      for (IsectEdge &edge_pair : edges_pair) {
        BMEdge *ea = edge_net_init[edge_pair.pair[0].e];
        BMEdge *eb = edge_net_init[edge_pair.pair[1].e];
        float3 co = math::interpolate(
            float3(ea->v1->co), float3(ea->v2->co), edge_pair.pair[0].lambda);
        BMVert *v_new = BM_vert_create(bm, co, nullptr, BM_CREATE_NOP);
        *edge_net_new_iter++ = BM_edge_create(bm, ea->v1, v_new, ea, BM_CREATE_NOP);
        *edge_net_new_iter++ = BM_edge_create(bm, v_new, ea->v2, ea, BM_CREATE_NOP);
        *edge_net_new_iter++ = BM_edge_create(bm, eb->v1, v_new, eb, BM_CREATE_NOP);
        *edge_net_new_iter++ = BM_edge_create(bm, v_new, eb->v2, eb, BM_CREATE_NOP);
      }
      *r_edge_net_new = edge_net_init = edge_net_new;
      *r_edge_net_new_len = edge_net_init_len = edge_net_new_len;
      ok = true;
    }
  }

  if (use_island_connect) {
    if (BM_face_split_edgenet_connect_islands(bm,
                                              f,
                                              edge_net_init,
                                              edge_net_init_len,
                                              use_partial_connect,
                                              mem_arena,
                                              r_edge_net_new,
                                              r_edge_net_new_len))
    {
      ok = true;
    }
  }

  return ok;
}

/* Splits a face into multiple sub-faces using a list of edges in `fdata`.
 * The split faces are returned in `r_face_arr`. */
static bool split_face_with_edgenet(BMesh *bm,
                                    BMFace *f,
                                    FaceData *fdata,
                                    bool use_island_connect,
                                    bool use_partial_connect,
                                    MemArena *mem_arena_edgenet,
                                    Vector<BMFace *> *r_face_arr)
{
  uint edge_arr_len = fdata->edge_ptrs_len;
  BMEdge **edge_arr = BLI_array_alloca(edge_arr, edge_arr_len);
  LinkNode *node;
  BLI_assert(f->head.htype == BM_FACE);

  uint i = 0;
  for (node = fdata->edge_ptrs; node; node = node->next) {
    edge_arr[i++] = *(BMEdge *const *)node->link;
  }
  BLI_assert(i == fdata->edge_ptrs_len);

  uint edge_arr_holes_len;
  BMEdge **edge_arr_holes;
  BM_face_calc_normal(f, f->no);
  if (prepare_edgenet_for_face(bm,
                               f,
                               edge_arr,
                               edge_arr_len,
                               true,
                               use_island_connect,
                               use_partial_connect,
                               mem_arena_edgenet,
                               &edge_arr_holes,
                               &edge_arr_holes_len))
  {
    /* Owned by the arena. */
    edge_arr_len = edge_arr_holes_len;
    edge_arr = edge_arr_holes;
  }

  return BM_face_split_edgenet(bm, f, edge_arr, (int)edge_arr_len, r_face_arr) &&
         !r_face_arr->is_empty();
}

static void split_faces_and_tag_edges(IsectMap *s,
                                      const bool use_island_connect,
                                      const bool use_partial_connect)
{
  BMesh *bm = s->bm;

  /* Convert #IsectPairs to #BMEdge. */
  for (const IsectEdgeRef &ekey : s->isect_edges.as_span()) {
    IsectEdge *isect_edge = ekey.isect_edge;
    if (isect_edge->pair_a->elem_a.ptr == isect_edge->pair_b->elem_a.ptr) {
      /* It shouldn't happen. */
      isect_edge->e = nullptr;
      continue;
    }
    if (isect_edge->pair_a->type_a != VERT) {
      BMVert *v = BM_vert_create(bm, isect_edge->pair_a->co, nullptr, BM_CREATE_NOP);
      isect_edge->pair_a->elem_a.v = v;
      isect_edge->pair_a->type_a = VERT;
      BM_elem_flag_enable(v, VERT_DISSOLVE_FLAG);
    }
    if (isect_edge->pair_b->type_a != VERT) {
      BMVert *v = BM_vert_create(bm, isect_edge->pair_b->co, nullptr, BM_CREATE_NOP);
      isect_edge->pair_b->elem_a.v = v;
      isect_edge->pair_b->type_a = VERT;
      BM_elem_flag_enable(v, VERT_DISSOLVE_FLAG);
    }
    BMVert *v_a = isect_edge->pair_a->elem_a.v;
    BMVert *v_b = isect_edge->pair_b->elem_a.v;
    BMEdge *e = BM_edge_exists(v_a, v_b);
    if (e) {
      /* Pass. */
    }
    else {
      e = BM_edge_create(bm, v_a, v_b, nullptr, BM_CREATE_NOP);
      /* For faster detection of `BM_edge_in_face`. */
      BM_elem_index_set(e, -2);

      /* Do not dissolve vertices of edges created from edge-edge intersections.
       * This is not an absolute rule, but in most cases, it is better to keep these vertices, as
       * these edges often form a hole in a face with a well-defined contour. */
      if (isect_edge->pair_a->type_b == NONE_MAYBE_DISSOLVE) {
        BM_elem_flag_disable(isect_edge->pair_a->elem_a.v, VERT_DISSOLVE_FLAG);
      }
      if (isect_edge->pair_b->type_b == NONE_MAYBE_DISSOLVE) {
        BM_elem_flag_disable(isect_edge->pair_b->elem_a.v, VERT_DISSOLVE_FLAG);
      }
    }
    BM_elem_flag_enable(e, EDGE_ISECT_FLAG);
    isect_edge->e = e;
  }

  /* Ressolve mutable access. */
  BMFaceFaceData *fkey_array = (BMFaceFaceData *)BLI_array_alloca(fkey_array, s->face_data.size());

  uint fdata_len = 0;
  for (const BMFaceFaceData &fkey : s->face_data) {
    BLI_assert((uint)BLI_linklist_count(fkey.fdata->edge_ptrs) == fkey.fdata->edge_ptrs_len);
    if (fkey.fdata->edge_ptrs_len == 0) {
      continue;
    }
    /* Copy. */
    fkey_array[fdata_len++] = fkey;
  }

  if (fdata_len == 0) {
    return;
  }

  MemArena *mem_arena_edgenet = BLI_memarena_new(BLI_MEMARENA_STD_BUFSIZE, __func__);

  for (uint i = 0; i < fdata_len; i++) {
    const BMFaceFaceData &fkey = fkey_array[i];
    BMFace *f = fkey.f;
    FaceData *fdata = fkey.fdata;
    LinkNode **node_ptr, *node, *node_next;
    if (fdata->edge_ptrs_len == 0) {
      /* Can it happen? */
      continue;
    }

    /* Remove nullptr. */
    node_ptr = &fdata->edge_ptrs;
    node = *node_ptr;
    do {
      node_next = node->next;
      BMEdge *e = *(BMEdge **)node->link;
      if (!e) {
        /* Remove node. */
        s->node_remove(node_ptr, node);
        fdata->edge_ptrs_len--;
        continue;
      }
      node_ptr = &node->next;
    } while ((node = node_next));

    if (fdata->edge_ptrs_len == 0) {
      continue;
    }

    /* Remove duplicated edges.
     * It should never happen, but it happens because of the imprecision that causes pairs that
     * should reference EDGEs to reference TRIS. */
    node_ptr = &fdata->edge_ptrs;
    node = *node_ptr;
    do {
      node_next = node->next;
      BMEdge *e = *(BMEdge **)node->link;
      if (!BM_elem_flag_test(e, EDGE_ISECT_FLAG)) {
        /* Remove edge. */
        s->node_remove(node_ptr, node);
        fdata->edge_ptrs_len--;
        continue;
      }
      BM_elem_flag_disable(e, EDGE_ISECT_FLAG);
      node_ptr = &node->next;
    } while ((node = node_next));

    /* Remove edges in face. */
    node_ptr = &fdata->edge_ptrs;
    node = *node_ptr;
    do {
      node_next = node->next;
      BMEdge *e = *(BMEdge **)node->link;
      BM_elem_flag_enable(e, EDGE_ISECT_FLAG);
      if ((BM_elem_index_get(e) != -2) && BM_edge_in_face(e, f)) {
        /* Remove edge. */
        s->node_remove(node_ptr, node);
        fdata->edge_ptrs_len--;
        continue;
      }
      node_ptr = &node->next;
    } while ((node = node_next));

    BLI_assert((uint)BLI_linklist_count(fdata->edge_ptrs) == fdata->edge_ptrs_len);
    if (fdata->edge_ptrs_len == 0) {
      continue;
    }

    Vector<BMFace *> face_arr;
    if (split_face_with_edgenet(
            bm, f, fdata, use_island_connect, use_partial_connect, mem_arena_edgenet, &face_arr))
    {
      if (fdata->coplanar_faces) {
        for (BMFace *f_new : face_arr) {
          if (f_new == f) {
            continue;
          }

          s->add_face_data(f_new, fdata);
          s->face_coplanar_add(f_new, fdata);
        }
      }
    }
    BLI_memarena_clear(mem_arena_edgenet);
  }
  BLI_memarena_free(mem_arena_edgenet);
}

/*** Resolve Coplanar Cases. ***/

static bool is_loop_inside_face(BMLoop *la, BMFace *fb, const float eps_sq)
{
  BMFace *fa = la->f;
  BMVert *va = la->v;
  const float3 &va_co = va->co;
  const float3 &fa_no = fa->no;
  const float3 &fb_no = fb->no;

  auto is_on_same_face_side = [&](const BMLoop *lb) {
    /* Check if the edges of \la point inwards on the face \fb. */
    const float3 &va_prev_co = la->prev->v->co;
    const float3 &va_next_co = la->next->v->co;

    const float3 &vb_co = lb->v->co;
    const float3 &vb_next_co = lb->next->v->co;
    float3 tan_b = math::cross(vb_co - vb_next_co, fb_no);
    if ((math::dot(tan_b, va_prev_co - va_co) >= FLT_EPSILON) ||
        (math::dot(tan_b, va_next_co - va_co) >= FLT_EPSILON))
    {
      return true;
    }
    else {
      /* They can be aligned. Check the tangent. */
      float3 tan_a_prev = math::cross(va_co - va_prev_co, fa_no);
      float3 tan_a = math::cross(va_co - vb_next_co, fa_no);
      if (math::dot(tan_b, tan_a_prev) > 0.0f || math::dot(tan_b, tan_a) > 0.0f) {
        return true;
      }
    }
    return false;
  };

  /* First check if the face shares the same vertex. */
  BMLoop *lb = BM_face_vert_share_loop(fb, va);
  if (lb) {
    /* If they share an edge, check the normal. */
    if (lb->prev->v == la->prev->v || lb->next->v == la->next->v) {
      return math::dot(fa_no, fb_no) > 0.0f;
    }
    if (lb->prev->v == la->next->v || lb->next->v == la->prev->v) {
      return math::dot(fa_no, fb_no) < 0.0f;
    }
    return is_on_same_face_side(lb) || is_on_same_face_side(lb->prev);
  }

  /* Check if the vertex intersects any edge. */
  BMLoop *l_first = lb = fb->l_first;
  do {
    const float3 &vb_co = lb->v->co;
    const float3 &vb_next_co = lb->next->v->co;
    const float dist_sq = dist_squared_to_line_segment_v3(va_co, vb_co, vb_next_co);
    if (dist_sq <= eps_sq) {
      return is_on_same_face_side(lb);
    }
  } while ((lb = lb->next) != l_first);

  return BM_face_point_inside_test(fb, va_co);
}

static void resolve_coplanar_faces(IsectMap *s, const bool kill_face_a, const bool kill_face_b)
{
  for (const BMFaceFaceData &fkey : s->face_data) {
    if (fkey.fdata->coplanar_faces == nullptr) {
      continue;
    }
    BM_face_calc_normal(fkey.f, fkey.f->no);
  }

  /* Check if the face is overlapped by coplanar faces.
   * NOTE: This check is not precise, it only tests if all corners of a face are "inside" any of
   * the coplanar faces. */
  for (const BMFaceFaceData &fkey : s->face_data) {
    if (fkey.fdata->coplanar_faces == nullptr) {
      continue;
    }
    if (fkey.fdata->is_face_a) {
      if (!kill_face_a) {
        /* Skip if face A should not be killed. */
        continue;
      }
    }
    else {
      if (!kill_face_b) {
        /* Skip if face B should not be killed. */
        continue;
      }
    }
    bool is_overlap = true;
    BMLoop *l_iter = fkey.f->l_first;
    BMLoop *l_first = l_iter;
    do {
      LinkNode *node = fkey.fdata->coplanar_faces;
      do {
        BMFace *f_coplanar = (BMFace *)node->link;
        if (is_loop_inside_face(l_iter, f_coplanar, s->eps_sq)) {
          break;
        }
      } while ((node = node->next));
      if (!node) {
        /* No overlap found for this corner. */
        is_overlap = false;
        break;
      }
    } while ((l_iter = l_iter->next) != l_first);

    if (is_overlap) {
      /* Postpone deleting the face, as we will still access it. */
      fkey.f->mat_nr = -1;
    }
  }

  /* Remove faces marked for deletion. */
  for (const BMFaceFaceData &fkey : s->face_data) {
    if (fkey.f->mat_nr == -1) {
      BM_face_kill_loose(s->bm, fkey.f);
    }
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Main API.
 * \{ */

static bool bm_mesh_intersect(BMesh *bm,
                              const Span<std::array<BMLoop *, 3>> looptris,
                              BVHTreeOverlap *overlap,
                              const uint tree_overlap_tot,
                              const bool use_island_connect,
                              const bool use_partial_connect,
                              const bool kill_coplanar_a,
                              const bool kill_coplanar_b,
                              const float eps)
{
  BM_mesh_elem_index_ensure(bm, BM_VERT);

  IsectMap s(bm, eps);
  for (uint i = 0; i < tree_overlap_tot; i++) {
    intersect_tri_tri(&s,
                      overlap[i].indexA,
                      overlap[i].indexB,
                      looptris[overlap[i].indexA],
                      looptris[overlap[i].indexB]);
  }

  split_and_merge_edge_edge_cases(&s);
  merge_vert_vert_cases(&s);
  split_faces_and_tag_edges(&s, use_island_connect, use_partial_connect);
  resolve_coplanar_faces(&s, kill_coplanar_a, kill_coplanar_b);

  /* Use to check if we made any changes. */
  const bool has_edit_boolean = !s.face_data.is_empty();

  /* It's unlikely the selection history is useful at this point,
   * if this is not called this array would need to be validated, see: #86799. */
  BM_select_history_clear(bm);

  return has_edit_boolean;
}

struct LoopFilterWrap {
  int (*test_fn)(BMFace *f, void *user_data);
  void *user_data;
};

static bool bm_loop_filter_fn(const BMLoop *l, void *user_data)
{
  if (BM_elem_flag_test(l->e, EDGE_ISECT_FLAG)) {
    return false;
  }

  if (l->radial_next != l) {
    LoopFilterWrap *data = (LoopFilterWrap *)user_data;
    BMLoop *l_iter = l->radial_next;
    const int face_side = data->test_fn(l->f, data->user_data);
    do {
      const int face_side_other = data->test_fn(l_iter->f, data->user_data);
      if (UNLIKELY(face_side_other == -1)) {
        /* pass */
      }
      else if (face_side_other != face_side) {
        return false;
      }
    } while ((l_iter = l_iter->radial_next) != l);
    return true;
  }
  return false;
}

bool BM_mesh_intersect_v2(BMesh *bm,
                          const Span<std::array<BMLoop *, 3>> looptris,
                          int (*test_fn)(BMFace *f, void *user_data),
                          void *user_data,
                          const bool use_self,
                          const bool use_separate,
                          const bool use_dissolve,
                          const bool use_island_connect,
                          const bool use_partial_connect,
                          const bool use_remove_coplanar_faces,
                          const float eps,
                          FunctionRef<bool(BVHTree *trees[2],
                                           const float3 (*looptri_coords)[3],
                                           BMFace *const *ftable,
                                           Span<int> face_indices,
                                           int side)> remove_face_group_fn)
{
  BVHTree *tree_a, *tree_b;
  uint tree_overlap_tot;
  BVHTreeOverlap *overlap;

  /* use to check if we made any changes */
  bool has_edit_isect = false, has_edit_boolean = false;

  /* Needed for boolean, since cutting up faces moves the loops within the face. */
  float3(*looptri_coords)[3] = nullptr;

  if (remove_face_group_fn) {
    /* Keep original geometry for ray-cast callbacks. */
    float3(*cos)[3] = (float3(*)[3])MEM_mallocN((size_t)looptris.size() * sizeof(*cos), __func__);
    for (int i = 0; i < looptris.size(); i++) {
      copy_v3_v3(cos[i][0], looptris[i][0]->v->co);
      copy_v3_v3(cos[i][1], looptris[i][1]->v->co);
      copy_v3_v3(cos[i][2], looptris[i][2]->v->co);
    }
    looptri_coords = cos;
  }

  {
    tree_a = BLI_bvhtree_new(looptris.size(), eps, 8, 8);
    for (int i = 0; i < looptris.size(); i++) {
      if (test_fn(looptris[i][0]->f, user_data) == 0) {
        const float t_cos[3][3] = {
            {UNPACK3(looptris[i][0]->v->co)},
            {UNPACK3(looptris[i][1]->v->co)},
            {UNPACK3(looptris[i][2]->v->co)},
        };

        BLI_bvhtree_insert(tree_a, i, (const float *)t_cos, 3);
      }
    }
    BLI_bvhtree_balance(tree_a);
  }

  if (use_self == false) {
    tree_b = BLI_bvhtree_new(looptris.size(), eps, 8, 8);
    for (int i = 0; i < looptris.size(); i++) {
      if (test_fn(looptris[i][0]->f, user_data) == 1) {
        const float t_cos[3][3] = {
            {UNPACK3(looptris[i][0]->v->co)},
            {UNPACK3(looptris[i][1]->v->co)},
            {UNPACK3(looptris[i][2]->v->co)},
        };

        BLI_bvhtree_insert(tree_b, i, (const float *)t_cos, 3);
      }
    }
    BLI_bvhtree_balance(tree_b);
  }
  else {
    tree_b = tree_a;
  }

  int flag = BVH_OVERLAP_USE_THREADING | BVH_OVERLAP_RETURN_PAIRS;
#ifndef NDEBUG
  /* The overlap result must match that obtained in Release to succeed
   * in the `bmesh_boolean` test. */
  if (looptris.size() < 1024) {
    flag &= ~BVH_OVERLAP_USE_THREADING;
  }
#endif
  overlap = BLI_bvhtree_overlap_ex(tree_a, tree_b, &tree_overlap_tot, nullptr, nullptr, 0, flag);

  if (overlap) {
    has_edit_isect = bm_mesh_intersect(bm,
                                       looptris,
                                       overlap,
                                       tree_overlap_tot,
                                       use_island_connect,
                                       use_partial_connect,
                                       use_remove_coplanar_faces,
                                       true,
                                       eps);
    MEM_freeN(overlap);
  }

  if (use_separate) {
    BM_mesh_edgesplit(bm, false, true, false);
  }

  if (remove_face_group_fn) {
    BVHTree *tree_pair[2] = {tree_a, tree_b};

    /* group vars */
    int *groups_array;
    int(*group_index)[2];
    int group_tot;
    int i;
    BMFace **ftable;

    BM_mesh_elem_table_ensure(bm, BM_FACE);
    ftable = bm->ftable;

    /* wrap the face-test callback to make it into an edge-loop delimiter */
    LoopFilterWrap user_data_wrap{};
    user_data_wrap.test_fn = test_fn;
    user_data_wrap.user_data = user_data;

    groups_array = static_cast<int *>(
        MEM_mallocN(sizeof(*groups_array) * size_t(bm->totface), __func__));
    group_tot = BM_mesh_calc_face_groups(
        bm, groups_array, &group_index, bm_loop_filter_fn, nullptr, &user_data_wrap, 0, BM_EDGE);

    /* Check if island is inside/outside */
    for (i = 0; i < group_tot; i++) {
      int fg = group_index[i][0];
      int fg_size = group_index[i][1];
      Span<int> face_indices(groups_array + fg, fg_size);

      /* For now assume this is an OK face to test with (not degenerate!) */
      BMFace *f = ftable[face_indices[0]];
      int side = test_fn(f, user_data);
      if (side == -1) {
        continue;
      }
      BLI_assert(ELEM(side, 0, 1));
      bool do_remove = remove_face_group_fn(tree_pair, looptri_coords, ftable, face_indices, side);
      if (do_remove) {
        for (int f_index : face_indices) {
          /* Postpone killing the face since we can still access. */
          // BM_face_kill_loose(bm, ftable[groups_array[fg]]);
          ftable[f_index]->mat_nr = -1;
        }
      }

      has_edit_boolean |= do_remove;
    }

    MEM_freeN(groups_array);
    MEM_freeN(group_index);

    {
      int tot = bm->totface;
      for (i = 0; i < tot; i++) {
        if (ftable[i]->mat_nr == -1) {
          BM_face_kill_loose(bm, ftable[i]);
        }
      }
    }
  }

  /* We have dissolve code above, this is alternative logic,
   * we need to do it after the boolean is executed. */
  if (use_dissolve) {
    BMVert *v, *v_next;
    BMIter iter;
    BM_ITER_MESH_MUTABLE (v, v_next, &iter, bm, BM_VERTS_OF_MESH) {
      if (!BM_elem_flag_test(v, VERT_DISSOLVE_FLAG)) {
        continue;
      }
      if (BM_vert_is_edge_pair(v)) {
        /* we won't create degenerate faces from this */
        bool ok = true;

        /* would we create a 2-sided-face?
         * if so, don't dissolve this since we may */
        if (v->e->l) {
          BMLoop *l_iter = v->e->l;
          do {
            if (l_iter->f->len == 3) {
              ok = false;
              break;
            }
          } while ((l_iter = l_iter->radial_next) != v->e->l);
        }

        if (ok) {
          BM_vert_collapse_edge(bm, v->e, v, true, false, false);
        }
      }
    }
  }

  MEM_SAFE_FREE(looptri_coords);
  BLI_bvhtree_free(tree_a);
  if (tree_a != tree_b) {
    BLI_bvhtree_free(tree_b);
  }

  return (has_edit_isect || has_edit_boolean);
}

/** \} */
