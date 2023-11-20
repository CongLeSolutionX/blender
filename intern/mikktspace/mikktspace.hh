/* SPDX-FileCopyrightText: 2011 Morten S. Mikkelsen
 * SPDX-FileCopyrightText: 2022 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

/** \file
 * \ingroup mikktspace
 */

#include <algorithm>
#include <cassert>
#include <chrono>
#include <memory>
#include <unordered_map>
#ifdef WITH_TBB
#  include <tbb/parallel_for.h>
#endif

#include "atomic_ops.h"
#include "mikk_atomic_hash_set.hh"
#include "mikk_float3.hh"
#include "mikk_simd.hh"
#include "mikk_util.hh"

namespace mikk {
struct TSpace {
  float3 tangent;
  uint8_t counter;
  bool orientPreserving;
  uint8_t duplicate;
  uint8_t padding;

  void zero()
  {
    tangent = float3(1.0f, 0.0f, 0.0f);
    counter = 0;
    orientPreserving = false;
    duplicate = padding = 0;
  }

  void accumulateGroup(bool groupOrient, float3 tang)
  {
    assert(counter < 2);

    if (counter == 0) {
      tangent = tang;
    }
    else if (tangent == tang) {
      // this if is important. Due to floating point precision
      // averaging when ts0==ts1 will cause a slight difference
      // which results in tangent space splits later on, so do nothing
    }
    else {
      // this is a pathology (counter is not 0 only if it is a corner of a quad which we
      // subdivided, and in that case, 99% of the time, the two corners are in the same group and
      // so have identical parameters.) But it does happen. Orientations are not guaranteed to
      // match either; we keep the orientation of the last group to visit here.
      tangent = (tangent + tang).normalize();
    }

    counter++;
    orientPreserving = groupOrient;
  }
};

template<typename Mesh> struct Mikktspace {
  static constexpr uint UNSET_ENTRY = 0xffffffffu;
  static constexpr uint chunk_size = 4096;

  struct Triangle {
    /* Index of the face that this triangle belongs to. */
    uint faceIdx;
    /* Index of the first of this triangle's vertices' TSpaces. */
    uint tSpaceIdx;
    // index of this triangle in the list.
    uint triIdx;

    /* Stores mapping from this triangle's vertices to the original
     * face's vertices (relevant for quads). */
    std::array<uint8_t, 3> faceVertex;
    // flags
    struct {
      bool markDegenerate : 1;
      bool quadOneDegenTri : 1;
      bool orientPreserving : 1;
      uint8_t padding : 5;
    };

    // we don't want initialization to run at allocation time (single threaded),
    // so there is no constructor and no default values for any members.

    void construct(uint faceIdx_, uint tSpaceIdx_)
    {
      faceIdx = faceIdx_;
      tSpaceIdx = tSpaceIdx_, markDegenerate = false;
      quadOneDegenTri = false;
      orientPreserving = false;
      padding = 0;
    }

    void setVertices(Mikktspace<Mesh> *parent, uint t, uint8_t i0, uint8_t i1, uint8_t i2)
    {
      faceVertex[0] = i0;
      faceVertex[1] = i1;
      faceVertex[2] = i2;
      triIdx = t;
      parent->get_vertex(t, 0) = pack_index(faceIdx, i0);
      parent->get_vertex(t, 1) = pack_index(faceIdx, i1);
      parent->get_vertex(t, 2) = pack_index(faceIdx, i2);
    }
  };

  struct indexed_map {
    /****
     *
     *   This is a computationally cheap substitute for std::multimap<uint,uint> for indexed data.
     *
     *   It contains up to index_dim elements. Each element has two properties:
     *   * 'key' (possibly duplicate, arbitrary 32 bit)
     *   * 'index' (guaranteed to be unique and less than index_dim)
     *   After construction (which is done serially but in O(N) time), we can, quickly and in
     * parallel, walk through all elements with this 'key'
     *
     *   We use it in buildNeighbors() to quickly find all faces owning the same vertex,
     *   in build4RuleGroups_parallel() to iterate over vertexes, and in degenEpilogue()
     *   to find non-degenerate triangles owning vertexes of degenerate triangles.
     *
     * **/

    // Each element holds either 0 (no entry) or 1 + index of the last inserted element with this
    // hash
    std::unique_ptr<uint[]> hashes;
    uint hashes_size = 0, max_hashes_size = 0;

    // Each element holds a pointer at the next element with the same hash
    std::unique_ptr<uint[]> memory;
    uint memory_size = 0;
    uint chunk = 4096;

    static inline uint power_of_two(uint x)
    {
      uint y = 1;
      while (y < x)
        y *= 2;
      return y;
    }

    inline void build(uint index_dim, uint hash_divisor)
    {
      max_hashes_size = hashes_size = power_of_two(index_dim);
      hashes = std::unique_ptr<uint[]>(new uint[hashes_size]);
      if (hashes_size > (1u << hash_divisor))
        hashes_size /= (1u << hash_divisor);
      memory_size = index_dim;
      memory = std::unique_ptr<uint[]>(new uint[memory_size]);
      clear();
    }

    inline void set_divisor(int div)
    {
      hashes_size = max_hashes_size;
      if (hashes_size > (1u << div))
        hashes_size /= (1u << div);
    }

    inline void clear()
    {
#ifdef WITH_TBB
      if (hashes_size > chunk) {
        tbb::parallel_for(0, int(hashes_size / chunk), [&](int t) {
          memset(hashes.get() + uint(t) * chunk, 0, chunk * sizeof(uint));
        });
      }
      else
#endif
        memset(hashes.get(), 0, hashes_size * sizeof(uint));
    }

    inline bool insert(uint index, uint key)
    {
      uint hash = key & uint(hashes_size - 1);
      // Point memory[index] at the last entry with this hash, forming a linked list
      uint last_val;
      last_val = memory[index] = hashes[hash];
      // Point 'hashes' at this element
      hashes[hash] = index + 1;
      return last_val == 0;
    }

    //  may be executed by multiple threads in parallel, and may be used recursively
    //  guaranteed to return all entries with the right key, and possibly some others;
    //  the user must check
    template<typename T> inline void iterate(uint key, T func) const
    {
      uint hash = uint(key) & uint(hashes_size - 1);
      uint it = hashes[hash];
      while (it) {
        it -= 1;
        if (!func(it))
          break;
        it = memory[it];
      }
    }
  };

  Mesh &mesh;

  std::unique_ptr<Triangle[]> triangles;
  std::unique_ptr<std::array<uint, 3>[]> tri_vertices;
  std::unique_ptr<std::array<uint, 3>[]> tri_neighbors;
  std::unique_ptr<float3[]> tri_tangent;
  std::unique_ptr<std::array<uint, 3>[]> tri_groups;
  std::unique_ptr<uint8_t[]> groupWithAny;
  std::unique_ptr<float3[]> corner_tangents;
  std::unique_ptr<uint8_t[]> groupOrientPreserving;
  std::unique_ptr<uint[]> store_addresses;

  uint8_t get_any(uint t)
  {
    return groupWithAny[t];
  }

  void set_any(uint t, uint8_t v)
  {
    groupWithAny[t] = v;
  }

  float3 &get_tangent(uint t)
  {
    return tri_tangent[t];
  }

  uint &get_group(uint t, uint i)
  {
    return tri_groups[t][i];
  }

  uint &get_neighbor(uint t, uint i)
  {
    return tri_neighbors[t][i];
  }

  uint &get_vertex(uint t, uint i)
  {
    return tri_vertices[t][i];
  }

  float3 &get_corner_tangent(uint gid)
  {
    return corner_tangents[gid];
  }

  void zero_tspaces()
  {
    runParallel(0u, nrTSpaces, [&](uint t) { tSpaces[t].zero(); });
  }

  std::unique_ptr<TSpace[]> tSpaces;
  uint groupCount = 0;
  std::vector<uint> triangle_offsets, acc_nrTSpaces;

  indexed_map vert_set;

  uint nrTSpaces, nrFaces, nrTriangles, totalTriangles;

  int nrThreads;
  bool isParallel;

 public:
  bool print_extra = false;
  bool profile = false;
  bool trace_on = false;
  bool allow_avx2 = true;
  bool allow_parallel = true;

  uint grain = 256;

  Mikktspace(Mesh &mesh_) : mesh(mesh_) {}

  // The process may be broadly described as follows.
  // We receive a list of faces with 3 float3 coordinate attributes per corner,
  // and no other topology information.
  // We ignore all faces with more than 4 corners, and split any face with 4 corners
  // into two triangles.
  // We make a list of 'physical vertexes' by fusing all triangle corners where
  // all three attributes exactly match (generateSharedVerticesIndexList()).
  // We identify degenerate triangles (triangles with zero-length edges) and exclude them
  // from most subsequent operations (degenPrologue()).
  // We use convoluted math logic to set binary flags 'orientPreserving' and 'groupWithAny'
  // to either 0 or 1 on the remaining triangles (initTriangle()).
  // We recreate the topology of the mesh. To that end, for every edge of a triangle, we
  // identify a 'neighbor' which shares this edge (pair of 'physical vertexes'), so that
  // each triangle has at most one neighbor along that edge, and the relationship is
  // symmetric (buildNeighbors()).
  // We subdivide triangle corners meeting on any 'physical vertex' into connected 'groups'
  // based on topology and binary flags (build4RuleGroups()).
  // We calculate a float3 'tangent' for each group by adding up contributions from each triangle
  // corner in it, and then assign the result to each face corner, with special handling
  // for the situation where we split some corner into 2 tris and they end up in different groups
  // (generateTSpaces()).
  // We set tangents for degenerate triangles (degenEpilogue()).
  // Finally, we copy everything back to the caller.

  void genTangSpace()
  {
    nrFaces = uint(mesh.GetNumFaces());

#ifdef WITH_TBB
    nrThreads = tbb::this_task_arena::max_concurrency();
    isParallel = allow_parallel && (nrThreads > 1) && (nrFaces > 1000);
#else
    nrThreads = 1;
    isParallel = false;
#endif

    // make an initial triangle --> face index list
    generateInitialVerticesIndexList();

    if (nrTriangles == 0) {
      return;
    }
    // make a welded index list of identical positions and attributes (pos, norm, texc)
    generateSharedVerticesIndexList();
    // mark all triangle pairs that belong to a quad with only one
    // good triangle. These need special treatment in degenEpilogue().
    // Additionally, move all good triangles to the start of
    // triangles[] without changing order and
    // put the degenerate triangles last.
    degenPrologue();

    if (nrTriangles == 0) {
      // No point in building tangents if there are no non-degenerate triangles, so just zero them
      allocate(tSpaces, nrTSpaces);
      zero_tspaces();
    }
    else {
      // evaluate triangle level attributes and neighbor list
      initTriangle();

      // match up edge pairs
      buildNeighbors();

      // based on the 4 rules, identify groups based on connectivity
      build4RuleGroups();

      // make tspaces, each group is split up into subgroups.
      // Finally a tangent space is made for every resulting subgroup
      generateTSpaces();

      // degenerate quads with one good triangle will be fixed by copying a space from
      // the good triangle to the coinciding vertex.
      // all other degenerate triangles will just copy a space from any good triangle
      // with the same welded index in vertices[].
      degenEpilogue();
    }
    if (isParallel) {
      uint max_store_address = 0;
      for (uint i = 0; i < nrTSpaces; i++)
        max_store_address = std::max(max_store_address, store_addresses[i]);
      uint8_t *p = new uint8_t[max_store_address + 1];
      memset(p, 0, (max_store_address + 1));
      for (int i = int(nrTSpaces) - 1; i >= 0; i--) {
        if (p[store_addresses[size_t(i)]] != 0)
          tSpaces[size_t(i)].duplicate = 1;
        p[store_addresses[size_t(i)]] = 1;
      }
      delete[] p;
      runParallel(0, nrTSpaces, [&](uint f) {
        const auto &tSpace = tSpaces[f];
        if (!tSpace.duplicate)
          mesh.SetTangentSpaceDirect(store_addresses[f], tSpace.tangent, tSpace.orientPreserving);
      });
    }
    else {
      runSerial(0, nrTSpaces, [&](uint f) {
        const auto &tSpace = tSpaces[f];
        mesh.SetTangentSpaceDirect(store_addresses[f], tSpace.tangent, tSpace.orientPreserving);
      });
    }

    everything.reset();
    triangles.reset();
  }

 protected:
  template<typename F> void runParallel(uint start, uint end, F func, int grain_size = -1)
  {
#ifdef WITH_TBB
    if (isParallel) {
      auto body = [&](const tbb::blocked_range<uint> &r) {
        for (uint x = r.begin(); x < r.end(); x++)
          func(x);
      };

      uint active_grain = (grain_size <= 0) ? grain : uint(grain_size);
      tbb::parallel_for(
          tbb::blocked_range<uint>(start, end, active_grain), body, tbb::simple_partitioner());
    }
    else
#endif
    {
      for (uint i = start; i < end; i++) {
        func(i);
      }
    }
  }
  template<typename F> void runSerial(uint start, uint end, F func)
  {
    for (uint i = start; i < end; i++) {
      func(i);
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  struct float3x3 {
    float3 data[3];
  };

  std::unique_ptr<float3x3[]> everything;

  const float3 &getPosition(uint vertexID)
  {
    return *(float3 *)&everything[vertexID].data[0];
  }

  const float3 &getNormal(uint vertexID)
  {
    return *(float3 *)&everything[vertexID].data[2];
  }

  const float3 &getTexCoord(uint vertexID)
  {
    return *(float3 *)&everything[vertexID].data[1];
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  template<typename T>
  void allocate(std::unique_ptr<T[]> &v, uint n, bool do_memset = false, int val = 0)
  {
    v.reset(new T[n]);
    if (do_memset)
      memset(v.get(), val, n * sizeof(T));
  }

  void generateInitialVerticesIndexList()
  {
    nrTriangles = 0;
    triangle_offsets.resize(2 * nrFaces);
    acc_nrTSpaces.resize(nrFaces);

    nrTSpaces = 0;
    for (uint f = 0; f < nrFaces; f++) {
      triangle_offsets[2 * f] = nrTriangles;
      acc_nrTSpaces[f] = nrTSpaces;
      const uint verts = mesh.GetNumVerticesOfFace(f);
      if (verts == 3) {
        nrTriangles += 1;
        nrTSpaces += 3;
      }
      else if (verts == 4) {
        triangle_offsets[2 * f + 1] = nrTriangles + 1;
        nrTriangles += 2;
        nrTSpaces += 4;
      }
    }
    allocate(store_addresses, nrTSpaces);
    allocate(triangles, nrTriangles);
    allocate(tri_tangent, nrTriangles);
    allocate(tri_vertices, nrTriangles);
    allocate(tri_neighbors, nrTriangles);
    allocate(tri_groups, nrTriangles);
    allocate(groupWithAny, nrTriangles);
    allocate(corner_tangents, nrTriangles * 3);
    allocate(everything, nrFaces * 4);

    runParallel(
        0,
        (nrTriangles + chunk_size - 1) / chunk_size,
        [&](uint t) {
          uint chunk = std::min(chunk_size, nrTriangles - t * chunk_size);
          memset(tri_tangent.get() + t * chunk_size, 0, chunk * sizeof(tri_tangent[0]));
          memset(tri_neighbors.get() + t * chunk_size, -1, chunk * sizeof(tri_neighbors[0]));
          memset(tri_groups.get() + t * chunk_size, -1, chunk * sizeof(tri_groups[0]));
          memset(groupWithAny.get() + t * chunk_size, 1, chunk * sizeof(groupWithAny[0]));
          memset(
              corner_tangents.get() + t * chunk_size, 0, chunk * 3 * sizeof(corner_tangents[0]));
        },
        1);

    runParallel(0, nrFaces, [&](uint f) {
      const uint verts = mesh.GetNumVerticesOfFace(f);
      if (verts != 3 && verts != 4)
        return;
      uint tspace = acc_nrTSpaces[f];
      uint tA = triangle_offsets[2 * f];
      Triangle &triA = triangles[tA];
      triA.construct(f, tspace);

      float3 p[4][3];
      for (uint v = 0; v < verts; v++) {
        uint li = mesh.GetStoreIndex(f, v);
        store_addresses[tspace + v] = li;
        p[v][0] = mesh.GetPositionDirect(li);
        p[v][1] = mesh.GetTexCoordDirect(li);
        p[v][2] = mesh.GetNormalDirect((int)f, li);
      }
      memcpy(&everything[f * 4], &p[0][0], 4 * 3 * sizeof(float3));

      if (verts == 3) {
        triA.setVertices(this, tA, 0, 1, 2);
      }
      else {
        uint tB = triangle_offsets[2 * f] + 1;
        Triangle &triB = triangles[tB];
        triB.construct(f, tspace);

        // need an order independent way to evaluate
        // tspace on quads. This is done by splitting
        // along the shortest diagonal.
        float distSQ_02 = (p[2][1] - p[0][1]).length_squared();
        float distSQ_13 = (p[3][1] - p[1][1]).length_squared();
        bool quadDiagIs_02;
        if (distSQ_02 != distSQ_13) {
          quadDiagIs_02 = (distSQ_02 < distSQ_13);
        }
        else {
          distSQ_02 = (p[2][0] - p[0][0]).length_squared();
          distSQ_13 = (p[3][0] - p[1][0]).length_squared();
          quadDiagIs_02 = !(distSQ_13 < distSQ_02);
        }

        if (quadDiagIs_02) {
          triA.setVertices(this, tA, 0, 1, 2);
          triB.setVertices(this, tB, 0, 2, 3);
        }
        else {
          triA.setVertices(this, tA, 0, 1, 3);
          triB.setVertices(this, tB, 1, 2, 3);
        }
      }
    });

    // Higher values of hash_divisor mean more hits for each hash, but less memory usage.
    // The value of 2 or 3 seems to be optimal.
    vert_set.build(nrTriangles * 4, 3u);
  }

  static inline uint quick_hash(uint a, uint b)
  {
    return a ^ ((b - a) * 337);
  }

  // For step 5 (buildNeighbors()), it maps from a physical edge to its triangle corners
  // For steps 6 (build4RuleGroups()) and 9 (degenEpilogue()), we need  a map from a physical
  // vertex to triangle corners we can't use one in place of the other, so we have no choice but to
  // rebuild In steps 6 and 9, it is not always needed, so we don't construct it until it's called
  // for
  void construct_vert_set_on_edges()
  {
    for (int t = (int)nrTriangles - 1; t >= 0; t--) {
      // for (uint j = 0; j < 3; j++)
      for (int j = 2; j >= 0; j--) {
        uint v1 = get_vertex((uint)t, (uint)j),
             v2 = get_vertex((uint)t, (uint)(j == 2 ? 0 : j + 1));
        uint vmax = std::max(v1, v2), vmin = std::min(v1, v2);
        uint h = quick_hash(vmin, vmax);
        //        uint h = hash_uint3(vmax, vmin, 0);
        vert_set.insert((uint)t * 4 + (uint)j, h);
      }
    }
  }

  void construct_vert_set()
  {
    if (vert_set_constructed)
      return;
    vert_set.clear();
    for (int t = (int)nrTriangles - 1; t >= 0; t--)
      for (uint j = 0; j < 3; j++)
        vert_set.insert(uint(t) * 4 + j, get_vertex((uint)t, j));
    vert_set_constructed = true;
  }
  bool vert_set_constructed = false;

  struct VertexHash {
    Mikktspace<Mesh> *mikk;
    inline uint operator()(const uint &k) const
    {
      return hash_float3x3(mikk->getPosition(k), mikk->getNormal(k), mikk->getTexCoord(k));
    }
  };

  struct VertexEqual {
    Mikktspace<Mesh> *mikk;
    inline bool operator()(const uint &kA, const uint &kB) const
    {
      return mikk->everything[kA].data[0] == mikk->everything[kB].data[0] &&
             mikk->everything[kA].data[1] == mikk->everything[kB].data[1] &&
             mikk->everything[kA].data[2] == mikk->everything[kB].data[2];
    }
  };

  /* Merge identical vertices.
   * To find vertices with identical position, normal and texcoord, we calculate a hash of the 9
   * values. Then, by sorting based on that hash, identical elements (having identical hashes) will
   * be moved next to each other. Since there might be hash collisions, the elements of each block
   * are then compared with each other and duplicates are merged.
   */
  template<bool isAtomic> void generateSharedVerticesIndexList_impl()
  {
    uint numVertices = nrTriangles * 3;
    AtomicHashSet<uint, isAtomic, VertexHash, VertexEqual> set(numVertices, {this}, {this});
    runParallel(0u, nrTriangles, [&](uint t) {
      for (uint i = 0; i < 3; i++) {
        auto res = set.emplace(get_vertex(t, i));
        if (!res.second) {
          get_vertex(t, i) = res.first;
        }
      }
    });
  }
  void generateSharedVerticesIndexList()
  {
    if (isParallel) {
      generateSharedVerticesIndexList_impl<true>();
    }
    else {
      generateSharedVerticesIndexList_impl<false>();
    }
  }

  /////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////// Degenerate triangles ////////////////////////////////////
  void degenPrologue()
  {
    // Mark all degenerate triangles
    totalTriangles = nrTriangles;
    std::atomic<uint> degenTriangles(0);
    runParallel(0u, totalTriangles, [&](uint t) {
      const float3 p0 = getPosition(get_vertex(t, 0));
      const float3 p1 = getPosition(get_vertex(t, 1));
      const float3 p2 = getPosition(get_vertex(t, 2));
      if (p0 == p1 || p0 == p2 || p1 == p2)  // degenerate
      {
        triangles[t].markDegenerate = true;
        degenTriangles.fetch_add(1);
      }
    });
    nrTriangles -= degenTriangles.load();

    if (totalTriangles == nrTriangles) {
      return;
    }

    // locate quads with only one good triangle
    runParallel(0u, totalTriangles - 1, [&](uint t) {
      Triangle &triangleA = triangles[t], &triangleB = triangles[t + 1];
      if (triangleA.faceIdx != triangleB.faceIdx) {
        /* Individual triangle, skip. */
        return;
      }
      if (triangleA.markDegenerate != triangleB.markDegenerate) {
        triangleA.quadOneDegenTri = true;
        triangleB.quadOneDegenTri = true;
      }
    });

    std::stable_partition(triangles.get(),
                          triangles.get() + totalTriangles,
                          [](const Triangle &tri) { return !tri.markDegenerate; });

    // stable_partition has rearranged triangles, but not tri_vertices, which are now decoupled
    // from it. We also need to fix the mapping from faces to triangles (needed to ensure
    // correctness of operation of generateTSpaces in multithreaded scenario)
    auto new_tri_vertices = std::unique_ptr<std::array<uint, 3>[]>(
        new std::array<uint, 3>[totalTriangles]);
    runParallel(0u, totalTriangles, [&](uint t) {
      auto old_tri = tri_vertices[triangles[t].triIdx];
      new_tri_vertices[t] = old_tri;
      triangles[t].triIdx = t;
      auto face = triangles[t].faceIdx;
      if (triangles[t].faceVertex[1] == 2)
        triangle_offsets[face * 2 + 1] = t;
      else
        triangle_offsets[face * 2 + 0] = t;
    });
    std::swap(tri_vertices, new_tri_vertices);
  }

  void degenEpilogue()
  {
    if (nrTriangles == totalTriangles) {
      return;
    }

    construct_vert_set();

    // deal with degenerate triangles
    // punishment for degenerate triangles is O(nrTriangles) extra memory.
    for (uint t = nrTriangles; t < totalTriangles; t++) {
      // degenerate triangles on a quad with one good triangle are skipped
      // here but processed in the next loop
      if (triangles[t].quadOneDegenTri) {
        continue;
      }

      for (uint i = 0; i < 3; i++) {
        uint tSrc, iSrc;
        uint entry = 0;
        bool found = false;
        uint vert = get_vertex(t, i);
        vert_set.iterate(vert, [&](uint id) {
          if (get_vertex(id >> 2, id & 3) == vert) {
            found = true;
            entry = id;
            return false;
          }
          return true;
        });

        if (!found)
          continue;
        // return found;
        unpack_index(tSrc, iSrc, entry);

        const uint iSrcVert = triangles[tSrc].faceVertex[iSrc];
        const uint iSrcOffs = triangles[tSrc].tSpaceIdx;
        const uint iDstVert = triangles[t].faceVertex[i];
        const uint iDstOffs = triangles[t].tSpaceIdx;
        // copy tspace
        tSpaces[iDstOffs + iDstVert] = tSpaces[iSrcOffs + iSrcVert];
      }
    }

    // deal with degenerate quads with one good triangle
    runParallel(0, nrTriangles, [&](uint t) {
      // this triangle belongs to a quad where the
      // other triangle is degenerate
      if (!triangles[t].quadOneDegenTri) {
        return;
      }
      uint vertFlag = (1u << triangles[t].faceVertex[0]) | (1u << triangles[t].faceVertex[1]) |
                      (1u << triangles[t].faceVertex[2]);
      uint missingFaceVertex = 0;
      if ((vertFlag & 2) == 0) {
        missingFaceVertex = 1;
      }
      else if ((vertFlag & 4) == 0) {
        missingFaceVertex = 2;
      }
      else if ((vertFlag & 8) == 0) {
        missingFaceVertex = 3;
      }

      uint faceIdx = triangles[t].faceIdx;
      float3 dstP = mesh.GetPosition(faceIdx, missingFaceVertex);
      bool found = false;
      for (uint i = 0; i < 3; i++) {
        const uint faceVertex = triangles[t].faceVertex[i];
        const float3 srcP = mesh.GetPosition(faceIdx, faceVertex);
        if (srcP == dstP) {
          const uint offset = triangles[t].tSpaceIdx;
          tSpaces[offset + missingFaceVertex] = tSpaces[offset + faceVertex];
          found = true;
          break;
        }
      }
      assert(found);
      (void)found;
    });
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  // returns the texture area times 2
  float calcTexArea(uint tri)
  {
    const float3 t1 = getTexCoord(get_vertex(tri, 0));
    const float3 t2 = getTexCoord(get_vertex(tri, 1));
    const float3 t3 = getTexCoord(get_vertex(tri, 2));

    const float t21x = t2.x - t1.x;
    const float t21y = t2.y - t1.y;
    const float t31x = t3.x - t1.x;
    const float t31y = t3.y - t1.y;

    const float signedAreaSTx2 = t21x * t31y - t21y * t31x;
    return fabsf(signedAreaSTx2);
  }

  bool test_avx2() const
  {
    if (!allow_avx2)
      return false;
    bool has_avx2 = false;
#ifdef _MSC_VER
    std::array<int, 4> cpui;
    __cpuid(cpui.data(), 0);
    if (cpui[0] >= 7) {
      __cpuidex(cpui.data(), 7, 0);
      has_avx2 = (cpui[1] >> 5) & 1;
    }
#elif defined(__GNUC__)
    has_avx2 = __builtin_cpu_supports("avx2");
#endif
    return has_avx2;
  }

#ifdef __GNUC__
#  pragma GCC push_options
#  pragma GCC target("avx2", "sse4.1")
#endif
  void initTriangleAVX2()
  {
    // evaluate first order derivatives
    runParallel(0u, nrTriangles, [&](uint t) {
      Triangle &triangle = triangles[t];
      const v4sf v1 = to_float4(getPosition(get_vertex(t, 0)));
      const v4sf v2 = to_float4(getPosition(get_vertex(t, 1)));
      const v4sf v3 = to_float4(getPosition(get_vertex(t, 2)));
      float t1[4] = {getTexCoord(get_vertex(t, 0)).x,
                     getTexCoord(get_vertex(t, 0)).y,
                     getTexCoord(get_vertex(t, 0)).x,
                     getTexCoord(get_vertex(t, 0)).y};
      float t23[4] = {getTexCoord(get_vertex(t, 1)).x,
                      getTexCoord(get_vertex(t, 1)).y,
                      getTexCoord(get_vertex(t, 2)).x,
                      getTexCoord(get_vertex(t, 2)).y};
      v4sf vt1 = *(v4sf *)t1, vt23 = *(v4sf *)t23;
      v4sf vt = vt23 - vt1;
      const v4sf d1 = v2 - v1, d2 = v3 - v1;
      v4sf temp = vt * __builtin_ia32_shufps(vt, vt, 11);
      temp = temp - __builtin_ia32_shufps(temp, temp, 1);
      const float signedAreaSTx2 = extract(temp, 0);

      v4sf vOs = d1 * bcast<3>(vt) - d2 * bcast<1>(vt);  // eq 18
      v4sf vOt = d2 * bcast<0>(vt) - d1 * bcast<2>(vt);  // eq 19

      triangle.orientPreserving = (signedAreaSTx2 > 0);

      if (not_zero(signedAreaSTx2)) {
        const float lenOs2 = length_squared(vOs);
        const float lenOs = length(vOs);
        const float lenOt2 = length_squared(vOt);
        const float fS = triangle.orientPreserving ? 1.0f : (-1.0f);
        // NOTE: not_zero(lenOs) and not_zero(lenOs2) are not equivalent!
        // If lenOs2 is a denormal float (in 10^-45 to 10^-39 range), not_zero(lenOs2) is false,
        // but not_zero(lenOs) is true
        if (not_zero(lenOs2)) {
          get_tangent(t) = to_float3(vOs * (fS / lenOs));
        }
        // if this is a good triangle
        if (not_zero(lenOs2) && not_zero(lenOt2)) {
          set_any(t, 0);
        }
      }
    });

    // force otherwise healthy quads to a fixed orientation
    runParallel(0u, nrTriangles - 1, [&](uint t) {
      Triangle &triangleA = triangles[t], &triangleB = triangles[t + 1];
      if (triangleA.faceIdx != triangleB.faceIdx) {
        // this is not a quad
        return;
      }

      // bad triangles should already have been removed by
      // degenPrologue(), but just in case check that neither are degenerate
      if (!(triangleA.markDegenerate || triangleB.markDegenerate)) {
        // if this happens the quad has extremely bad mapping!!
        if (triangleA.orientPreserving != triangleB.orientPreserving) {
          bool chooseOrientFirstTri = false;
          if (get_any(t + 1)) {
            chooseOrientFirstTri = true;
          }
          else if (calcTexArea(t) >= calcTexArea(t + 1)) {
            chooseOrientFirstTri = true;
          }

          // force match
          const uint t0 = chooseOrientFirstTri ? t : (t + 1);
          const uint t1 = chooseOrientFirstTri ? (t + 1) : t;
          triangles[t1].orientPreserving = triangles[t0].orientPreserving;
        }
      }
    });
  }
#ifdef __GNUC__
#  pragma GCC pop_options
#endif

  void initTriangle()
  {
    if (test_avx2()) {
      initTriangleAVX2();
      return;
    }
    // evaluate first order derivatives
    runParallel(0u, nrTriangles, [&](uint t) {
      Triangle &triangle = triangles[t];
      // initial values
      const float3 v1 = getPosition(get_vertex(t, 0));
      const float3 v2 = getPosition(get_vertex(t, 1));
      const float3 v3 = getPosition(get_vertex(t, 2));
      const float3 t1 = getTexCoord(get_vertex(t, 0));
      const float3 t2 = getTexCoord(get_vertex(t, 1));
      const float3 t3 = getTexCoord(get_vertex(t, 2));

      const float t21x = t2.x - t1.x;
      const float t21y = t2.y - t1.y;
      const float t31x = t3.x - t1.x;
      const float t31y = t3.y - t1.y;
      const float3 d1 = v2 - v1, d2 = v3 - v1;

      const float signedAreaSTx2 = t21x * t31y - t21y * t31x;
      float3 vOs = (t31y * d1) - (t21y * d2);   // eq 18
      float3 vOt = (-t31x * d1) + (t21x * d2);  // eq 19

      triangle.orientPreserving = (signedAreaSTx2 > 0);

      if (not_zero(signedAreaSTx2)) {
        const float lenOs2 = vOs.length_squared();
        const float lenOt2 = vOt.length_squared();
        const float fS = triangle.orientPreserving ? 1.0f : (-1.0f);
        if (not_zero(lenOs2)) {
          get_tangent(t) = vOs * (fS / sqrtf(lenOs2));
        }
        // if this is a good triangle
        if (not_zero(lenOs2) && not_zero(lenOt2)) {
          set_any(t, 0);
        }
      }
    });

    // force otherwise healthy quads to a fixed orientation
    runParallel(0u, nrTriangles - 1, [&](uint t) {
      Triangle &triangleA = triangles[t], &triangleB = triangles[t + 1];
      if (triangleA.faceIdx != triangleB.faceIdx) {
        // this is not a quad
        return;
      }

      // bad triangles should already have been removed by
      // degenPrologue(), but just in case check that neither are degenerate
      if (!(triangleA.markDegenerate || triangleB.markDegenerate)) {
        // if this happens the quad has extremely bad mapping!!
        if (triangleA.orientPreserving != triangleB.orientPreserving) {
          bool chooseOrientFirstTri = false;
          if (get_any(t + 1)) {
            chooseOrientFirstTri = true;
          }
          else if (calcTexArea(t) >= calcTexArea(t + 1)) {
            chooseOrientFirstTri = true;
          }

          // force match
          const uint t0 = chooseOrientFirstTri ? t : (t + 1);
          const uint t1 = chooseOrientFirstTri ? (t + 1) : t;
          triangles[t1].orientPreserving = triangles[t0].orientPreserving;
        }
      }
    });
  }

  bool buildNeighbors_fallback(uint v, uint id)
  {
    bool painted = false;
    uint t = id >> 2, i = id & 3;
    vert_set.iterate(v, [&](uint id2) {
      // We walk each list until we find ourselves, then abort. Otherwise, each pairing would be
      // discovered twice (by both triangles), which is wasteful.
      uint k = id2 >> 2, j = id2 & 3;
      if (id == id2) {
        return false;
      }
      if (get_neighbor(k, j) != UNSET_ENTRY)
        return true;
      if (get_vertex(t, i) == get_vertex(k, j == 2 ? 0 : j + 1) &&
          get_vertex(t, i == 2 ? 0 : i + 1) == get_vertex(k, j))
      {
        get_neighbor(t, i) = k * 4 + j;
        get_neighbor(k, j) = t * 4 + i;
        painted = true;
        return false;
      }
      return true;
    });
    return painted;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////// Edges ///////////////////////////////////////////
  void buildNeighbors()
  {
    construct_vert_set_on_edges();

    // we iterate over a symmetric hash of two physical vertex indexes, to make sure that
    // all occurrences of any physical edge are processed by the same thread
    // (we need to make neighbor assignments atomically, and it's hard to do if multiple
    // threads could claim the same edge)
    runParallel(0, vert_set.hashes_size, [&](uint v) {
      uint history[32];
      int history_count = 0;

      vert_set.iterate(v, [&](uint id) {
        uint t = id >> 2, i = id & 3;
        bool painted = false;

        if (history_count <= 32) {
          for (int ii = 0; ii < history_count; ii++) {
            uint id2 = history[ii];
            if (id2 == UNSET_ENTRY)
              continue;
            uint k = id2 >> 2, j = id2 & 3;
            if (get_vertex(t, i) == get_vertex(k, j == 2 ? 0 : j + 1) &&
                get_vertex(t, i == 2 ? 0 : i + 1) == get_vertex(k, j))
            {
              // both tri_neighbors are guaranteed to be unset at this point
              get_neighbor(t, i) = k * 4 + j;
              get_neighbor(k, j) = t * 4 + i;
              history[ii] = UNSET_ENTRY;
              painted = true;
              break;
            }
          }
        }
        else
          painted = buildNeighbors_fallback(v, id);

        if (history_count < 32 && !painted)
          history[history_count++] = id;
        return true;
      });
    });
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  int assignRecur(
      uint /*vertRep*/, bool orient, uint t, uint i, uint groupId, uint tPrev, bool clockwise)
  {
    if (t == UNSET_ENTRY)
      return 0;
    int painted = 0;

    while (true) {
      i = (clockwise) ? i : (i == 2 ? 0 : i + 1);

      // early out
      if (get_group(t, i) != UNSET_ENTRY) {
        break;
      }

      Triangle &triangle = triangles[t];

      if (get_any(t)) {
        // first to group with a group-with-anything triangle
        // determines its orientation.
        // This is the only existing order dependency in the code!!
        if (get_group(t, 0) == UNSET_ENTRY && get_group(t, 1) == UNSET_ENTRY &&
            get_group(t, 2) == UNSET_ENTRY)
        {
          triangle.orientPreserving = orient;
        }
      }

      if (triangle.orientPreserving != orient)
        break;

      get_group(t, i) = groupId;
      painted++;

      const uint t_L = get_neighbor(t, i);
      const uint t_R = get_neighbor(t, i > 0 ? (i - 1) : 2);

      // unless buildNeighbors got somehow broken, either t_L or t_R is tPrev
      // (all connections are two-way; if A is the neighbor of B along some edge,
      // then B is the neighbor of A along the same edge)
      if ((t_L >> 2) != tPrev && t_L != UNSET_ENTRY) {
        tPrev = t;
        t = t_L >> 2;
        i = t_L & 3;
      }
      else if ((t_R >> 2) != tPrev && t_R != UNSET_ENTRY) {
        tPrev = t;
        t = t_R >> 2;
        i = t_R & 3;
      }
      else
        break;
    }
    return painted;
  }

  void build4RuleGroups_parallel()
  {
    groupCount = nrTriangles * 3;
    allocate(groupOrientPreserving, nrTriangles * 3);
    runParallel(0, vert_set.hashes_size, [&](uint h) {
      int verts = 0;
      auto label_func = [&](uint id) {
        uint t = id >> 2, i = id & 3;
        Triangle &triangle = triangles[t];
        if (get_any(t) || get_group(t, i) != UNSET_ENTRY) {
          return true;
        }
        uint vertRep = get_vertex(t, i);
        bool orient = triangle.orientPreserving;
        groupOrientPreserving[t * 3 + i] = orient;
        get_corner_tangent(t * 3 + i) = float3(0.0f, 0.0f, 0.0f);
        verts--;
        get_group(t, i) = t * 3 + i;
        const uint t_L = get_neighbor(t, i);
        if (t_L != UNSET_ENTRY)
          verts -= assignRecur(vertRep, orient, t_L >> 2, t_L & 3, t * 3 + i, t, false);
        const uint t_R = get_neighbor(t, i > 0 ? (i - 1) : 2);
        if (verts && t_R != UNSET_ENTRY)
          verts -= assignRecur(vertRep, orient, t_R >> 2, t_R & 3, t * 3 + i, t, true);
        return verts > 0;
      };
      vert_set.iterate(h, [&](uint) {
        verts++;
        return true;
      });
      vert_set.iterate(h, label_func);
    });
  }

  /*
      // We subdivide each set of triangle corners sharing the same physical vertex (tri_vertices
     value)
      // into one or more 'groups'. We will have at least one group per physical vertex and at most
     three
      // group per triangle.

      // Subdivision is done according to the following rules:
      // 1. triangles within one group must form a continuous surface
      // (each triangle is connected to the rest of the group along at least one of its edges).
      // 'Connectivity' is determined using triangle.neighbor entries, which means that any
     triangle may only
      // be connected to one other triangle along any given edge, even if we have complex topology,
     so they
      // necessarily form a simple linear chain rather than a graph.
      //
      // 2. Edge connections don't count if triangles have differing values of orientPreserving.
      //
      // 3. Some (pathological) triangles are marked 'groupWithAny'. They don't have a value of
     orientPreserving
      // at the outset, and are assigned the value of the first group (in triangle index order)
      // that comes across any of their corners.
      //
      // We can reliably apply rules 1 and 2 in parallel. Rule 3 is the one that induces complex
     nonlocal dependencies
      // and makes parallelization difficult.
      // Consider the example:

          A --- B --- C
           \ t / \ u /
            \ / v \ /
             D --- E
            / \ w / \
           / x \ / y \
          G --- F --- H

        Triangles t, u, and x are orientPreserving=1, groupWithAny=0, Triangle y is
     orientPreserving=0, groupWithAny=0. Triangles v and w are groupwithAny=1.

        Existing rules mean that assignments to vertex B depend on values of x and y. Specifically:

        If 'y' is lower than t, u, and x, triangle corners meeting at B are assigned to 3 separate
     groups. If either t, u, or x is less than y, triangle corners meeting at B are assigned to the
     same group.

        We can only resolve everything in parallel without going to great lengths if no two
     groupWithAny tris meet on the same physical vertex.

        In practice, it's not worth going even that far. If the mesh has more than a couple of
     groupWithAny tris, it usually has them in configurations that prevent parallelism.

        We quickly check if any groupWithAny tris are connected to the rest of the mesh, and, if we
     find them, we go down the fallback (non-parallel) route.
    */

  void build4RuleGroups()
  {
    uint ngroupWithAny = 0, isolated = 0;
    for (uint t = 0; t < nrTriangles; t++) {
      if (get_any(t)) {
        uint minNeighbor = nrTriangles + 1;
        for (uint i = 0; i < 3; i++) {
          uint tt = get_neighbor(t, i);
          if (tt == UNSET_ENTRY)
            continue;
          minNeighbor = 0;
          break;
        }
        if (minNeighbor < nrTriangles)
          ngroupWithAny++;
        else
          isolated++;
      }
      if (ngroupWithAny)
        break;
    }

    if (ngroupWithAny == 0) {
      construct_vert_set();
      build4RuleGroups_parallel();
      return;
    }

    allocate(groupOrientPreserving, nrTriangles * 3);
    groupCount = 0;
    for (uint t = 0; t < nrTriangles; t++) {
      Triangle &triangle = triangles[t];
      if (get_any(t))
        continue;
      for (uint i = 0; i < 3; i++) {
        if (get_group(t, i) != UNSET_ENTRY)
          continue;

        const uint newGroupId = groupCount++;
        get_group(t, i) = newGroupId;
        uint vertRep = get_vertex(t, i);
        bool orient = triangle.orientPreserving;
        groupOrientPreserving[newGroupId] = orient;
        get_corner_tangent(newGroupId) = float3(0.0f, 0.0f, 0.0f);
        const uint t_L = get_neighbor(t, i);
        if (t_L != UNSET_ENTRY)
          assignRecur(vertRep, orient, t_L >> 2, t_L & 3, newGroupId, t, false);
        const uint t_R = get_neighbor(t, i > 0 ? (i - 1) : 2);
        if (t_R != UNSET_ENTRY)
          assignRecur(vertRep, orient, t_R >> 2, t_R & 3, newGroupId, t, true);
      }
    }
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef __GNUC__
#  pragma GCC push_options
#  pragma GCC target("avx2", "sse4.1")
#endif
  // Benefits of AVX2 are somewhat limited, because, in full AVX2 mode and with large meshes, over
  // half of total time is spent waiting for memory transactions
  template<bool atomic> void accumulateTSpacesAVX2(uint t)
  {
    // only valid triangles get to add their contribution
    if (get_any(t))
      return;
    if (get_group(t, 0) == UNSET_ENTRY && get_group(t, 1) == UNSET_ENTRY &&
        get_group(t, 2) == UNSET_ENTRY)
      return;
    v4sf edges[3], n[3], p[3];
    for (uint i = 0; i < 3; i++) {
      n[i] = to_float4(getNormal(get_vertex(t, i)));
      p[i] = to_float4(getPosition(get_vertex(t, i)));
    }

    edges[0] = p[1] - p[0];
    edges[1] = p[2] - p[0];
    edges[2] = p[2] - p[1];

    v4sf tt = to_float4(get_tangent(t));

    // try not to mess with the following without checking generated assembly
    // (especially with MSVC, which sometimes gets "creative").
    //
    // There's also a risk of small changes due to rounding translating into large differences in
    // outputs, due to the fact that float3::normalize() is discontinuous at 0

    v4sf pp[3];
    v8sf pp80 = project2(from_v4sf(n[0]), from_2xv4sf(edges[0], edges[1]));
    v8sf pp81 = project2(from_v4sf(n[1]), from_2xv4sf(edges[2], edges[0]));
    v8sf pp82 = project2(from_v4sf(n[2]), from_2xv4sf(edges[1], edges[2]));
    v8sf pp83 = project2(from_2xv4sf(n[0], n[1]), from_v4sf(tt));

    pp[2] = project1(n[2], tt);

    v8sf len8 = inv_length8(pp80, pp81, pp82, pp83);

    pp80 = pp80 * unpack4x<0>(len8);
    pp81 = pp81 * unpack4x<2>(len8);
    pp82 = pp82 * unpack4x<4>(len8);
    float len[3] = {extract(len8, 6), extract(len8, 7), inv_length(pp[2])};

    v4sf fCos = {dot_inner(pp80), -dot_inner(pp81), dot_inner(pp82), 0.0};

    fCos = fast_acosf_4x(fCos);

    unpack(pp[0], pp[1], pp83);

    for (uint i = 0; i < 3; i++) {
      uint groupId = get_group(t, i);
      if (groupId != UNSET_ENTRY) {
        v4sf tangent = pp[i] * len[i] * extract(fCos, i);
        float3 &dest = get_corner_tangent(groupId);
        auto t3 = to_float3(tangent);
        if constexpr (atomic) {
          float_add_atomic(&dest.x, t3.x);
          float_add_atomic(&dest.y, t3.y);
          float_add_atomic(&dest.z, t3.z);
        }
        else {
          dest += t3;
        }
      }
    }
  }
#ifdef __GNUC__
#  pragma GCC pop_options
#endif

  template<bool atomic> void accumulateTSpaces(uint t)
  {
    // only valid triangles get to add their contribution
    if (get_any(t)) {
      return;
    }

    std::array<float3, 3> n, p;
    for (uint i = 0; i < 3; i++) {
      n[i] = getNormal(get_vertex(t, i));
      p[i] = getPosition(get_vertex(t, i));
    }

    std::array<float, 3> fCos = {dot(project(n[0], p[1] - p[0]), project(n[0], p[2] - p[0])),
                                 dot(project(n[1], p[2] - p[1]), project(n[1], p[0] - p[1])),
                                 dot(project(n[2], p[0] - p[2]), project(n[2], p[1] - p[2]))};

    for (uint i = 0; i < 3; i++) {
      uint groupId = get_group(t, i);
      if (groupId != UNSET_ENTRY) {
        float3 tangent = project(n[i], get_tangent(t)) *
                         fast_acosf(std::clamp(fCos[i], -1.0f, 1.0f));
        float3 &dest = get_corner_tangent(groupId);
        if constexpr (atomic) {
          float_add_atomic(&dest.x, tangent.x);
          float_add_atomic(&dest.y, tangent.y);
          float_add_atomic(&dest.z, tangent.z);
        }
        else {
          dest += tangent;
        }
      }
    }
  }

  void generateTSpaces()
  {
    if (test_avx2()) {
      if (isParallel) {
        runParallel(0u, nrTriangles, [&](uint t) { accumulateTSpacesAVX2<true>(t); });
      }
      else {
        for (uint t = 0; t < nrTriangles; t++) {
          accumulateTSpacesAVX2<false>(t);
        }
      }
    }
    else {
      if (isParallel) {
        runParallel(0u, nrTriangles, [&](uint t) { accumulateTSpaces<true>(t); });
      }
      else {
        for (uint t = 0; t < nrTriangles; t++) {
          accumulateTSpaces<false>(t);
        }
      }
    }
    runParallel(0, groupCount, [&](uint g) {
      float3 &tan = get_corner_tangent(g);
      tan = tan.normalize();
    });
    allocate(tSpaces, nrTSpaces);
    zero_tspaces();
    // A tSpace is only accessed twice if it is a vertex in a quad which is subdivided by a
    // 'virtual edge' into two triangles.

    // This loop can be rewritten a lot more explicitly (directly iterating over the 4 tspaces
    // making up a quad and accumulating them from either 1 or 2 groups each) but that appears
    // to offer no performance benefits.
    runParallel(0, nrFaces, [&](uint f) {
      const uint verts = mesh.GetNumVerticesOfFace(f);
      if (verts != 3 && verts != 4)
        return;
      uint ntri = (verts == 4 ? 2 : 1);
      for (uint nt = 0; nt < ntri; nt++) {
        uint t = triangle_offsets[2 * f + nt];
        Triangle &triangle = triangles[t];
        for (uint i = 0; i < 3; i++) {
          // output tspace
          const uint offset = triangle.tSpaceIdx;
          const uint faceVertex = triangle.faceVertex[i];

          uint groupId = get_group(t, i);
          if (groupId == UNSET_ENTRY) {
            continue;
          }
          bool orient = groupOrientPreserving[groupId];
          assert(triangle.orientPreserving == orient);
          tSpaces[offset + faceVertex].accumulateGroup(orient, get_corner_tangent(groupId));
        }
      }
    });
  }
};

}  // namespace mikk
