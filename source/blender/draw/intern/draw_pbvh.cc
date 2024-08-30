/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 *
 * bke::pbvh::Tree drawing.
 * Embeds GPU meshes inside of bke::pbvh::Tree nodes, used by mesh sculpt mode.
 */

#include <algorithm>
#include <climits>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "MEM_guardedalloc.h"

#include "BLI_bitmap.h"
#include "BLI_function_ref.hh"
#include "BLI_ghash.h"
#include "BLI_index_range.hh"
#include "BLI_map.hh"
#include "BLI_math_color.h"
#include "BLI_math_vector_types.hh"
#include "BLI_string.h"
#include "BLI_string_ref.hh"
#include "BLI_timeit.hh"
#include "BLI_utildefines.h"
#include "BLI_vector.hh"

#include "DNA_mesh_types.h"
#include "DNA_object_types.h"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_ccg.hh"
#include "BKE_customdata.hh"
#include "BKE_mesh.hh"
#include "BKE_paint.hh"
#include "BKE_pbvh_api.hh"
#include "BKE_subdiv_ccg.hh"

#include "DEG_depsgraph_query.hh"

#include "GPU_batch.hh"

#include "DRW_engine.hh"
#include "DRW_pbvh.hh"

#include "attribute_convert.hh"
#include "bmesh.hh"
#include "gpu_private.hh"

#define MAX_PBVH_BATCH_KEY 512
#define MAX_PBVH_VBOS 16

namespace blender {

template<> struct DefaultHash<draw::pbvh::AttributeRequest> {
  uint64_t operator()(const draw::pbvh::AttributeRequest &value) const
  {
    using namespace draw::pbvh;
    if (const CustomRequest *request_type = std::get_if<CustomRequest>(&value)) {
      return get_default_hash(*request_type);
    }
    const GenericRequest &attr = std::get<GenericRequest>(value);
    return get_default_hash(attr.name);
  }
};

}  // namespace blender

namespace blender::draw::pbvh {

uint64_t ViewportRequest::hash() const
{
  return get_default_hash(attributes, use_coarse_grids);
}

class DrawCache : public bke::pbvh::DrawCache {
 public:
  Vector<int> visible_tri_count;
  Vector<bool> use_flat_layout;
  Vector<int> material_indices;

  Vector<gpu::IndexBuf *> lines_ibos;
  Vector<gpu::IndexBuf *> lines_ibos_coarse;
  Vector<gpu::IndexBuf *> tris_ibos;
  Vector<gpu::IndexBuf *> tris_ibos_coarse;
  Map<AttributeRequest, Vector<gpu::VertBuf *>> attribute_vbos;

  Vector<gpu::Batch *> lines_batches;
  Vector<gpu::Batch *> lines_batches_coarse;
  Map<ViewportRequest, Vector<gpu::Batch *>> tris_batches;

  ~DrawCache() override;
};

DrawCache &ensure_draw_data(std::unique_ptr<bke::pbvh::DrawCache> &ptr)
{
  if (!ptr) {
    ptr = std::make_unique<DrawCache>();
  }
  return dynamic_cast<DrawCache &>(*ptr);
}

BLI_NOINLINE static void free_ibos(const MutableSpan<gpu::IndexBuf *> ibos,
                                   const IndexMask &nodes_to_update)
{
  IndexMaskMemory memory;
  const IndexMask mask = IndexMask::from_intersection(nodes_to_update, ibos.index_range(), memory);
  mask.foreach_index([&](const int i) { GPU_INDEXBUF_DISCARD_SAFE(ibos[i]); });
}

BLI_NOINLINE static void free_vbos(const MutableSpan<gpu::VertBuf *> vbos,
                                   const IndexMask &nodes_to_update)
{
  IndexMaskMemory memory;
  const IndexMask mask = IndexMask::from_intersection(nodes_to_update, vbos.index_range(), memory);
  mask.foreach_index([&](const int i) { GPU_VERTBUF_DISCARD_SAFE(vbos[i]); });
}

BLI_NOINLINE static void free_batches(const MutableSpan<gpu::Batch *> batches,
                                      const IndexMask &nodes_to_update)
{
  IndexMaskMemory memory;
  const IndexMask mask = IndexMask::from_intersection(
      nodes_to_update, batches.index_range(), memory);
  mask.foreach_index([&](const int i) { GPU_BATCH_DISCARD_SAFE(batches[i]); });
}

struct OrigMeshData {
  StringRef active_color;
  StringRef default_color;
  StringRef active_uv_map;
  StringRef default_uv_map;
  int face_set_default;
  int face_set_seed;
  bke::AttributeAccessor attributes;
  OrigMeshData(const Mesh &mesh)
      : active_color(mesh.active_color_attribute),
        default_color(mesh.default_color_attribute),
        active_uv_map(CustomData_get_active_layer_name(&mesh.corner_data, CD_PROP_FLOAT2)),
        default_uv_map(CustomData_get_render_layer_name(&mesh.corner_data, CD_PROP_FLOAT2)),
        face_set_default(mesh.face_sets_color_default),
        face_set_seed(mesh.face_sets_color_seed),
        attributes(mesh.attributes())
  {
  }
};

static const GPUVertFormat &position_format()
{
  static GPUVertFormat format{};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
  }
  return format;
}

static const GPUVertFormat &normal_format()
{
  static GPUVertFormat format{};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "nor", GPU_COMP_I16, 3, GPU_FETCH_INT_TO_FLOAT_UNIT);
  }
  return format;
}

static const GPUVertFormat &mask_format()
{
  static GPUVertFormat format{};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "msk", GPU_COMP_F32, 1, GPU_FETCH_FLOAT);
  }
  return format;
}

static const GPUVertFormat &face_set_format()
{
  static GPUVertFormat format{};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "fset", GPU_COMP_U8, 3, GPU_FETCH_INT_TO_FLOAT_UNIT);
  }
  return format;
}

static GPUVertFormat attribute_format(const OrigMeshData &orig_mesh_data,
                                      const StringRefNull name,
                                      const eCustomDataType data_type)
{
  GPUVertFormat format = draw::init_format_for_attribute(data_type, "data");

  bool is_render, is_active;
  const char *prefix = "a";

  if (CD_TYPE_AS_MASK(data_type) & CD_MASK_COLOR_ALL) {
    prefix = "c";
    is_active = orig_mesh_data.active_color == name;
    is_render = orig_mesh_data.default_color == name;
  }
  if (data_type == CD_PROP_FLOAT2) {
    prefix = "u";
    is_active = orig_mesh_data.active_uv_map == name;
    is_render = orig_mesh_data.default_uv_map == name;
  }

  DRW_cdlayer_attr_aliases_add(&format, prefix, data_type, name.c_str(), is_render, is_active);
  return format;
}

static GPUVertFormat format_for_request(const OrigMeshData &orig_mesh_data,
                                        const AttributeRequest &request)
{
  if (const CustomRequest *request_type = std::get_if<CustomRequest>(&request)) {
    switch (*request_type) {
      case CustomRequest::Position:
        return position_format();
      case CustomRequest::Normal:
        return normal_format();
      case CustomRequest::Mask:
        return mask_format();
      case CustomRequest::FaceSet:
        return face_set_format();
    }
  }
  else {
    const GenericRequest &attr = std::get<GenericRequest>(request);
    return attribute_format(orig_mesh_data, attr.name, attr.type);
  }
  BLI_assert_unreachable();
  return {};
}

static bool pbvh_attr_supported(const AttributeRequest &request)
{
  if (std::holds_alternative<CustomRequest>(request)) {
    return true;
  }
  const GenericRequest &attr = std::get<GenericRequest>(request);
  if (!ELEM(attr.domain, bke::AttrDomain::Point, bke::AttrDomain::Face, bke::AttrDomain::Corner)) {
    /* blender::bke::pbvh::Tree drawing does not support edge domain attributes. */
    return false;
  }
  bool type_supported = false;
  bke::attribute_math::convert_to_static_type(attr.type, [&](auto dummy) {
    using T = decltype(dummy);
    using Converter = AttributeConverter<T>;
    using VBOType = typename Converter::VBOType;
    if constexpr (!std::is_void_v<VBOType>) {
      type_supported = true;
    }
  });
  return type_supported;
}

inline short4 normal_float_to_short(const float3 &value)
{
  short3 result;
  normal_float_to_short_v3(result, value);
  return short4(result.x, result.y, result.z, 0);
}

template<typename T>
void extract_data_vert_mesh(const Span<int> corner_verts,
                            const Span<int3> corner_tris,
                            const Span<int> tri_faces,
                            const Span<bool> hide_poly,
                            const Span<T> attribute,
                            const Span<int> tris,
                            gpu::VertBuf &vbo)
{
  using Converter = AttributeConverter<T>;
  using VBOType = typename Converter::VBOType;
  VBOType *data = vbo.data<VBOType>().data();
  for (const int tri : tris) {
    if (!hide_poly.is_empty() && hide_poly[tri_faces[tri]]) {
      continue;
    }
    for (int i : IndexRange(3)) {
      const int vert = corner_verts[corner_tris[tri][i]];
      *data = Converter::convert(attribute[vert]);
      data++;
    }
  }
}

template<typename T>
void extract_data_face_mesh(const Span<int> tri_faces,
                            const Span<bool> hide_poly,
                            const Span<T> attribute,
                            const Span<int> tris,
                            gpu::VertBuf &vbo)
{
  using Converter = AttributeConverter<T>;
  using VBOType = typename Converter::VBOType;

  VBOType *data = vbo.data<VBOType>().data();
  for (const int tri : tris) {
    const int face = tri_faces[tri];
    if (!hide_poly.is_empty() && hide_poly[face]) {
      continue;
    }
    std::fill_n(data, 3, Converter::convert(attribute[face]));
    data += 3;
  }
}

template<typename T>
void extract_data_corner_mesh(const Span<int3> corner_tris,
                              const Span<int> tri_faces,
                              const Span<bool> hide_poly,
                              const Span<T> attribute,
                              const Span<int> tris,
                              gpu::VertBuf &vbo)
{
  using Converter = AttributeConverter<T>;
  using VBOType = typename Converter::VBOType;

  VBOType *data = vbo.data<VBOType>().data();
  for (const int tri : tris) {
    if (!hide_poly.is_empty() && hide_poly[tri_faces[tri]]) {
      continue;
    }
    for (int i : IndexRange(3)) {
      const int corner = corner_tris[tri][i];
      *data = Converter::convert(attribute[corner]);
      data++;
    }
  }
}

template<typename T> const T &bmesh_cd_vert_get(const BMVert &vert, const int offset)
{
  return *static_cast<const T *>(POINTER_OFFSET(vert.head.data, offset));
}

template<typename T> const T &bmesh_cd_loop_get(const BMLoop &loop, const int offset)
{
  return *static_cast<const T *>(POINTER_OFFSET(loop.head.data, offset));
}

template<typename T> const T &bmesh_cd_face_get(const BMFace &face, const int offset)
{
  return *static_cast<const T *>(POINTER_OFFSET(face.head.data, offset));
}

template<typename T>
void extract_data_vert_bmesh(const Set<BMFace *, 0> &faces, const int cd_offset, gpu::VertBuf &vbo)
{
  using Converter = AttributeConverter<T>;
  using VBOType = typename Converter::VBOType;
  VBOType *data = vbo.data<VBOType>().data();

  for (const BMFace *face : faces) {
    if (BM_elem_flag_test(face, BM_ELEM_HIDDEN)) {
      continue;
    }
    const BMLoop *l = face->l_first;
    *data = Converter::convert(bmesh_cd_vert_get<T>(*l->prev->v, cd_offset));
    data++;
    *data = Converter::convert(bmesh_cd_vert_get<T>(*l->v, cd_offset));
    data++;
    *data = Converter::convert(bmesh_cd_vert_get<T>(*l->next->v, cd_offset));
    data++;
  }
}

template<typename T>
void extract_data_face_bmesh(const Set<BMFace *, 0> &faces, const int cd_offset, gpu::VertBuf &vbo)
{
  using Converter = AttributeConverter<T>;
  using VBOType = typename Converter::VBOType;
  VBOType *data = vbo.data<VBOType>().data();

  for (const BMFace *face : faces) {
    if (BM_elem_flag_test(face, BM_ELEM_HIDDEN)) {
      continue;
    }
    std::fill_n(data, 3, Converter::convert(bmesh_cd_face_get<T>(*face, cd_offset)));
    data += 3;
  }
}

template<typename T>
void extract_data_corner_bmesh(const Set<BMFace *, 0> &faces,
                               const int cd_offset,
                               gpu::VertBuf &vbo)
{
  using Converter = AttributeConverter<T>;
  using VBOType = typename Converter::VBOType;
  VBOType *data = vbo.data<VBOType>().data();

  for (const BMFace *face : faces) {
    if (BM_elem_flag_test(face, BM_ELEM_HIDDEN)) {
      continue;
    }
    const BMLoop *l = face->l_first;
    *data = Converter::convert(bmesh_cd_loop_get<T>(*l->prev, cd_offset));
    data++;
    *data = Converter::convert(bmesh_cd_loop_get<T>(*l, cd_offset));
    data++;
    *data = Converter::convert(bmesh_cd_loop_get<T>(*l->next, cd_offset));
    data++;
  }
}

static const CustomData *get_cdata(const BMesh &bm, const bke::AttrDomain domain)
{
  switch (domain) {
    case bke::AttrDomain::Point:
      return &bm.vdata;
    case bke::AttrDomain::Corner:
      return &bm.ldata;
    case bke::AttrDomain::Face:
      return &bm.pdata;
    default:
      return nullptr;
  }
}

template<typename T> T fallback_value_for_fill()
{
  return T();
}

template<> ColorGeometry4f fallback_value_for_fill()
{
  return ColorGeometry4f(1.0f, 1.0f, 1.0f, 1.0f);
}

template<> ColorGeometry4b fallback_value_for_fill()
{
  return fallback_value_for_fill<ColorGeometry4f>().encode();
}

static int count_visible_tris_mesh(const Span<int> tris,
                                   const Span<int> tri_faces,
                                   const Span<bool> hide_poly)
{
  if (hide_poly.is_empty()) {
    return tris.size();
  }
  return std::count_if(
      tris.begin(), tris.end(), [&](const int tri) { return !hide_poly[tri_faces[tri]]; });
}

static int count_visible_tris_bmesh(const Set<BMFace *, 0> &faces)
{
  return std::count_if(faces.begin(), faces.end(), [&](const BMFace *face) {
    return !BM_elem_flag_test_bool(face, BM_ELEM_HIDDEN);
  });
}

static void remove_node_tags(bke::pbvh::Tree &pbvh, const IndexMask &node_mask)
{
  switch (pbvh.type()) {
    case bke::pbvh::Type::Mesh: {
      MutableSpan<bke::pbvh::MeshNode> nodes = pbvh.nodes<bke::pbvh::MeshNode>();
      node_mask.foreach_index([&](const int i) {
        nodes[i].flag_ &= ~(PBVH_UpdateDrawBuffers | PBVH_RebuildDrawBuffers);
      });
      break;
    }
    case bke::pbvh::Type::Grids: {
      MutableSpan<bke::pbvh::GridsNode> nodes = pbvh.nodes<bke::pbvh::GridsNode>();
      node_mask.foreach_index([&](const int i) {
        nodes[i].flag_ &= ~(PBVH_UpdateDrawBuffers | PBVH_RebuildDrawBuffers);
      });
      break;
    }
    case bke::pbvh::Type::BMesh: {
      MutableSpan<bke::pbvh::BMeshNode> nodes = pbvh.nodes<bke::pbvh::BMeshNode>();
      node_mask.foreach_index([&](const int i) {
        nodes[i].flag_ &= ~(PBVH_UpdateDrawBuffers | PBVH_RebuildDrawBuffers);
      });
      break;
    }
  }
}

static IndexMask calc_nodes_to_free_and_update_dyntopo_size(const Object &object,
                                                            const IndexMask &nodes_to_update,
                                                            IndexMaskMemory &memory,
                                                            DrawCache &draw_data)
{
  const bke::pbvh::Tree &pbvh = *object.sculpt->pbvh;
  switch (pbvh.type()) {
    case bke::pbvh::Type::Mesh: {
      const Span<bke::pbvh::MeshNode> nodes = pbvh.nodes<bke::pbvh::MeshNode>();
      return IndexMask::from_predicate(nodes_to_update, GrainSize(1024), memory, [&](const int i) {
        return nodes[i].flag_ & PBVH_RebuildDrawBuffers;
      });
    }
    case bke::pbvh::Type::Grids: {
      const Span<bke::pbvh::GridsNode> nodes = pbvh.nodes<bke::pbvh::GridsNode>();
      return IndexMask::from_predicate(nodes_to_update, GrainSize(1024), memory, [&](const int i) {
        return nodes[i].flag_ & PBVH_RebuildDrawBuffers;
      });
    }
    case bke::pbvh::Type::BMesh: {
      MutableSpan<int> node_visible_count = draw_data.visible_tri_count;
      const Span<bke::pbvh::BMeshNode> nodes = pbvh.nodes<bke::pbvh::BMeshNode>();
      return IndexMask::from_predicate(nodes_to_update, GrainSize(32), memory, [&](const int i) {
        if (nodes[i].flag_ & PBVH_RebuildDrawBuffers) {
          return true;
        }
        const int old_size = node_visible_count[i];
        node_visible_count[i] = count_visible_tris_bmesh(
            BKE_pbvh_bmesh_node_faces(&const_cast<bke::pbvh::BMeshNode &>(nodes[i])));
        if (old_size != node_visible_count[i]) {
          return true;
        }
        return false;
      });
    }
  }
  BLI_assert_unreachable();
  return {};
}

static void free_stale_node_data(const Object &object,
                                 const IndexMask &nodes_to_update,
                                 DrawCache &draw_data)
{
  IndexMaskMemory memory;
  const IndexMask nodes_to_free = calc_nodes_to_free_and_update_dyntopo_size(
      object, nodes_to_update, memory, draw_data);

  free_ibos(draw_data.lines_ibos, nodes_to_free);
  free_ibos(draw_data.lines_ibos_coarse, nodes_to_free);
  free_ibos(draw_data.tris_ibos, nodes_to_free);
  free_ibos(draw_data.tris_ibos_coarse, nodes_to_free);
  for (MutableSpan<gpu::VertBuf *> vbos : draw_data.attribute_vbos.values()) {
    free_vbos(vbos, nodes_to_free);
  }

  free_batches(draw_data.lines_batches, nodes_to_free);
  free_batches(draw_data.lines_batches_coarse, nodes_to_free);
  for (MutableSpan<gpu::Batch *> batches : draw_data.tris_batches.values()) {
    free_batches(batches, nodes_to_free);
  }
}

DrawCache::~DrawCache()
{
  free_ibos(this->lines_ibos, this->lines_ibos.index_range());
  free_ibos(this->lines_ibos_coarse, this->lines_ibos_coarse.index_range());
  free_ibos(this->tris_ibos, this->tris_ibos.index_range());
  free_ibos(this->tris_ibos_coarse, this->tris_ibos_coarse.index_range());
  for (MutableSpan<gpu::VertBuf *> vbos : this->attribute_vbos.values()) {
    free_vbos(vbos, vbos.index_range());
  }

  free_batches(this->lines_batches, this->lines_batches.index_range());
  free_batches(this->lines_batches_coarse, this->lines_batches_coarse.index_range());
  for (MutableSpan<gpu::Batch *> batches : this->tris_batches.values()) {
    free_batches(batches, batches.index_range());
  }
}

static void fill_vbo_normal_mesh(const Span<int> corner_verts,
                                 const Span<int3> corner_tris,
                                 const Span<int> tri_faces,
                                 const Span<bool> sharp_faces,
                                 const Span<bool> hide_poly,
                                 const Span<float3> vert_normals,
                                 const Span<float3> face_normals,
                                 const Span<int> tris,
                                 gpu::VertBuf &vert_buf)
{
  short4 *data = vert_buf.data<short4>().data();

  short4 face_no;
  int last_face = -1;
  for (const int tri : tris) {
    const int face = tri_faces[tri];
    if (!hide_poly.is_empty() && hide_poly[face]) {
      continue;
    }
    if (!sharp_faces.is_empty() && sharp_faces[face]) {
      if (face != last_face) {
        face_no = normal_float_to_short(face_normals[face]);
        last_face = face;
      }
      std::fill_n(data, 3, face_no);
      data += 3;
    }
    else {
      for (const int i : IndexRange(3)) {
        const int vert = corner_verts[corner_tris[tri][i]];
        *data = normal_float_to_short(vert_normals[vert]);
        data++;
      }
    }
  }
}

static void fill_vbo_mask_mesh(const Span<int> corner_verts,
                               const Span<int3> corner_tris,
                               const Span<int> tri_faces,
                               const Span<bool> hide_poly,
                               const Span<float> mask,
                               const Span<int> tris,
                               gpu::VertBuf &vbo)
{
  float *data = vbo.data<float>().data();
  for (const int tri : tris) {
    if (!hide_poly.is_empty() && hide_poly[tri_faces[tri]]) {
      continue;
    }
    for (int i : IndexRange(3)) {
      const int vert = corner_verts[corner_tris[tri][i]];
      *data = mask[vert];
      data++;
    }
  }
}

static void fill_vbo_face_set_mesh(const Span<int> tri_faces,
                                   const Span<bool> hide_poly,
                                   const Span<int> face_sets,
                                   const int color_default,
                                   const int color_seed,
                                   const Span<int> tris,
                                   gpu::VertBuf &vert_buf)
{
  uchar4 *data = vert_buf.data<uchar4>().data();
  int last_face = -1;
  uchar4 fset_color(UCHAR_MAX);
  for (const int tri : tris) {
    if (!hide_poly.is_empty() && hide_poly[tri_faces[tri]]) {
      continue;
    }
    const int face = tri_faces[tri];
    if (last_face != face) {
      last_face = face;

      const int id = face_sets[face];

      if (id != color_default) {
        BKE_paint_face_set_overlay_color_get(id, color_seed, fset_color);
      }
      else {
        /* Skip for the default color face set to render it white. */
        fset_color[0] = fset_color[1] = fset_color[2] = UCHAR_MAX;
      }
    }
    std::fill_n(data, 3, fset_color);
    data += 3;
  }
}

static void fill_vbo_attribute_mesh(const Span<int> corner_verts,
                                    const Span<int3> corner_tris,
                                    const Span<int> tri_faces,
                                    const Span<bool> hide_poly,
                                    const GSpan attribute,
                                    const bke::AttrDomain domain,
                                    const Span<int> tris,
                                    gpu::VertBuf &vert_buf)
{
  bke::attribute_math::convert_to_static_type(attribute.type(), [&](auto dummy) {
    using T = decltype(dummy);
    if constexpr (!std::is_void_v<typename AttributeConverter<T>::VBOType>) {
      switch (domain) {
        case bke::AttrDomain::Point:
          extract_data_vert_mesh<T>(corner_verts,
                                    corner_tris,
                                    tri_faces,
                                    hide_poly,
                                    attribute.typed<T>(),
                                    tris,
                                    vert_buf);
          break;
        case bke::AttrDomain::Face:
          extract_data_face_mesh<T>(tri_faces, hide_poly, attribute.typed<T>(), tris, vert_buf);
          break;
        case bke::AttrDomain::Corner:
          extract_data_corner_mesh<T>(
              corner_tris, tri_faces, hide_poly, attribute.typed<T>(), tris, vert_buf);
          break;
        default:
          BLI_assert_unreachable();
      }
    }
  });
}

static void fill_vbo_position_grids(const CCGKey &key,
                                    const Span<CCGElem *> grids,
                                    const bool use_flat_layout,
                                    const Span<int> grid_indices,
                                    gpu::VertBuf &vert_buf)
{
  float3 *data = vert_buf.data<float3>().data();
  if (use_flat_layout) {
    const int grid_size_1 = key.grid_size - 1;
    for (const int i : grid_indices.index_range()) {
      CCGElem *grid = grids[grid_indices[i]];
      for (int y = 0; y < grid_size_1; y++) {
        for (int x = 0; x < grid_size_1; x++) {
          *data = CCG_grid_elem_co(key, grid, x, y);
          data++;
          *data = CCG_grid_elem_co(key, grid, x + 1, y);
          data++;
          *data = CCG_grid_elem_co(key, grid, x + 1, y + 1);
          data++;
          *data = CCG_grid_elem_co(key, grid, x, y + 1);
          data++;
        }
      }
    }
  }
  else {
    for (const int i : grid_indices.index_range()) {
      CCGElem *grid = grids[grid_indices[i]];
      for (const int offset : IndexRange(key.grid_area)) {
        *data = CCG_elem_offset_co(key, grid, offset);
        data++;
      }
    }
  }
}

static void fill_vbo_normal_grids(const CCGKey &key,
                                  const Span<CCGElem *> grids,
                                  const Span<int> grid_to_face_map,
                                  const Span<bool> sharp_faces,
                                  const bool use_flat_layout,
                                  const Span<int> grid_indices,
                                  gpu::VertBuf &vert_buf)
{

  short4 *data = vert_buf.data<short4>().data();

  if (use_flat_layout) {
    const int grid_size_1 = key.grid_size - 1;
    for (const int i : grid_indices.index_range()) {
      const int grid_index = grid_indices[i];
      CCGElem *grid = grids[grid_index];
      if (!sharp_faces.is_empty() && sharp_faces[grid_to_face_map[grid_index]]) {
        for (int y = 0; y < grid_size_1; y++) {
          for (int x = 0; x < grid_size_1; x++) {
            float3 no;
            normal_quad_v3(no,
                           CCG_grid_elem_co(key, grid, x, y + 1),
                           CCG_grid_elem_co(key, grid, x + 1, y + 1),
                           CCG_grid_elem_co(key, grid, x + 1, y),
                           CCG_grid_elem_co(key, grid, x, y));
            std::fill_n(data, 4, normal_float_to_short(no));
            data += 4;
          }
        }
      }
      else {
        for (int y = 0; y < grid_size_1; y++) {
          for (int x = 0; x < grid_size_1; x++) {
            std::fill_n(data, 4, normal_float_to_short(CCG_grid_elem_no(key, grid, x, y)));
            data += 4;
          }
        }
      }
    }
  }
  else {
    /* The non-flat VBO layout does not support sharp faces. */
    for (const int i : grid_indices.index_range()) {
      CCGElem *grid = grids[grid_indices[i]];
      for (const int offset : IndexRange(key.grid_area)) {
        *data = normal_float_to_short(CCG_elem_offset_no(key, grid, offset));
        data++;
      }
    }
  }
}

static void fill_vbo_mask_grids(const CCGKey &key,
                                const Span<CCGElem *> grids,
                                const bool use_flat_layout,
                                const Span<int> grid_indices,
                                gpu::VertBuf &vert_buf)
{
  if (key.has_mask) {
    float *data = vert_buf.data<float>().data();
    if (use_flat_layout) {
      const int grid_size_1 = key.grid_size - 1;
      for (const int i : grid_indices.index_range()) {
        CCGElem *grid = grids[grid_indices[i]];
        for (int y = 0; y < grid_size_1; y++) {
          for (int x = 0; x < grid_size_1; x++) {
            *data = CCG_grid_elem_mask(key, grid, x, y);
            data++;
            *data = CCG_grid_elem_mask(key, grid, x + 1, y);
            data++;
            *data = CCG_grid_elem_mask(key, grid, x + 1, y + 1);
            data++;
            *data = CCG_grid_elem_mask(key, grid, x, y + 1);
            data++;
          }
        }
      }
    }
    else {
      for (const int i : grid_indices.index_range()) {
        CCGElem *grid = grids[grid_indices[i]];
        for (const int offset : IndexRange(key.grid_area)) {
          *data = CCG_elem_offset_mask(key, grid, offset);
          data++;
        }
      }
    }
  }
  else {
    vert_buf.data<float>().fill(0.0f);
  }
}

static void fill_vbo_face_set_grids(const CCGKey &key,
                                    const Span<int> grid_to_face_map,
                                    const Span<int> face_sets,
                                    const int color_default,
                                    const int color_seed,
                                    const bool use_flat_layout,
                                    const Span<int> grid_indices,
                                    gpu::VertBuf &vert_buf)
{
  const int verts_per_grid = use_flat_layout ? square_i(key.grid_size - 1) * 4 :
                                               square_i(key.grid_size);
  uchar4 *data = vert_buf.data<uchar4>().data();
  for (const int i : grid_indices.index_range()) {
    uchar4 color{UCHAR_MAX};
    const int fset = face_sets[grid_to_face_map[grid_indices[i]]];
    if (fset != color_default) {
      BKE_paint_face_set_overlay_color_get(fset, color_seed, color);
    }

    std::fill_n(data, verts_per_grid, color);
    data += verts_per_grid;
  }
}

static void fill_vbos_grids(const Object &object,
                            const OrigMeshData &orig_mesh_data,
                            const Span<bool> use_flat_layout,
                            const IndexMask &nodes_to_update,
                            const AttributeRequest &request,
                            const MutableSpan<gpu::VertBuf *> vbos)
{
  const SculptSession &ss = *object.sculpt;
  const Span<bke::pbvh::GridsNode> nodes = ss.pbvh->nodes<bke::pbvh::GridsNode>();
  const SubdivCCG &subdiv_ccg = *ss.subdiv_ccg;
  const CCGKey key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);
  const Span<CCGElem *> grids = subdiv_ccg.grids;

  if (const CustomRequest *request_type = std::get_if<CustomRequest>(&request)) {
    switch (*request_type) {
      case CustomRequest::Position: {
        nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
          fill_vbo_position_grids(
              key, grids, use_flat_layout[i], bke::pbvh::node_grid_indices(nodes[i]), *vbos[i]);
        });
        break;
      }
      case CustomRequest::Normal: {
        const Mesh &mesh = *static_cast<const Mesh *>(object.data);
        const Span<int> grid_to_face_map = subdiv_ccg.grid_to_face_map;
        const bke::AttributeAccessor attributes = mesh.attributes();
        const VArraySpan sharp_faces = *attributes.lookup<bool>("sharp_face",
                                                                bke::AttrDomain::Face);
        nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
          fill_vbo_normal_grids(key,
                                grids,
                                grid_to_face_map,
                                sharp_faces,
                                use_flat_layout[i],
                                bke::pbvh::node_grid_indices(nodes[i]),
                                *vbos[i]);
        });

        break;
      }
      case CustomRequest::Mask: {
        nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
          fill_vbo_mask_grids(
              key, grids, use_flat_layout[i], bke::pbvh::node_grid_indices(nodes[i]), *vbos[i]);
        });
        break;
      }
      case CustomRequest::FaceSet: {
        const int face_set_default = orig_mesh_data.face_set_default;
        const int face_set_seed = orig_mesh_data.face_set_seed;
        const Span<int> grid_to_face_map = subdiv_ccg.grid_to_face_map;
        const bke::AttributeAccessor attributes = orig_mesh_data.attributes;
        if (const VArray<int> face_sets = *attributes.lookup<int>(".sculpt_face_set",
                                                                  bke::AttrDomain::Face))
        {
          const VArraySpan<int> face_sets_span(face_sets);
          nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
            fill_vbo_face_set_grids(key,
                                    grid_to_face_map,
                                    face_sets_span,
                                    face_set_default,
                                    face_set_seed,
                                    use_flat_layout[i],
                                    bke::pbvh::node_grid_indices(nodes[i]),
                                    *vbos[i]);
          });
        }
        else {
          nodes_to_update.foreach_index(
              GrainSize(1), [&](const int i) { vbos[i]->data<uchar4>().fill(uchar4{UCHAR_MAX}); });
        }
        break;
      }
    }
  }
  else {
    const eCustomDataType type = std::get<GenericRequest>(request).type;
    nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
      bke::attribute_math::convert_to_static_type(type, [&](auto dummy) {
        using T = decltype(dummy);
        using Converter = AttributeConverter<T>;
        using VBOType = typename Converter::VBOType;
        if constexpr (!std::is_void_v<VBOType>) {
          vbos[i]->data<VBOType>().fill(Converter::convert(fallback_value_for_fill<T>()));
        }
      });
    });
  }
}

static void fill_vbos_mesh(const Object &object,
                           const OrigMeshData &orig_mesh_data,
                           const IndexMask &nodes_to_update,
                           const AttributeRequest &request,
                           const MutableSpan<gpu::VertBuf *> vbos)
{
  SculptSession &ss = *object.sculpt;
  const Span<bke::pbvh::MeshNode> nodes = ss.pbvh->nodes<bke::pbvh::MeshNode>();
  const Mesh &mesh = *static_cast<const Mesh *>(object.data);
  const Span<int> corner_verts = mesh.corner_verts();
  const Span<int3> corner_tris = mesh.corner_tris();
  const Span<int> tri_faces = mesh.corner_tri_faces();
  const bke::AttributeAccessor attributes = mesh.attributes();
  const VArraySpan hide_poly = *orig_mesh_data.attributes.lookup<bool>(".hide_poly",
                                                                       bke::AttrDomain::Face);

  if (const CustomRequest *request_type = std::get_if<CustomRequest>(&request)) {
    switch (*request_type) {
      case CustomRequest::Position: {
        const Span<float3> vert_positions = bke::pbvh::vert_positions_eval_from_eval(object);
        nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
          extract_data_vert_mesh<float3>(corner_verts,
                                         corner_tris,
                                         tri_faces,
                                         hide_poly,
                                         vert_positions,
                                         bke::pbvh::node_tri_indices(nodes[i]),
                                         *vbos[i]);
        });
        break;
      }
      case CustomRequest::Normal: {
        const Span<float3> vert_normals = bke::pbvh::vert_normals_eval_from_eval(object);
        const Span<float3> face_normals = bke::pbvh::face_normals_eval_from_eval(object);
        const VArraySpan sharp_faces = *attributes.lookup<bool>("sharp_face",
                                                                bke::AttrDomain::Face);
        nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
          fill_vbo_normal_mesh(corner_verts,
                               corner_tris,
                               tri_faces,
                               sharp_faces,
                               hide_poly,
                               vert_normals,
                               face_normals,
                               bke::pbvh::node_tri_indices(nodes[i]),
                               *vbos[i]);
        });
        break;
      }
      case CustomRequest::Mask: {
        const VArraySpan mask = *orig_mesh_data.attributes.lookup<float>(".sculpt_mask",
                                                                         bke::AttrDomain::Point);
        if (!mask.is_empty()) {
          nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
            fill_vbo_mask_mesh(corner_verts,
                               corner_tris,
                               tri_faces,
                               hide_poly,
                               mask,
                               bke::pbvh::node_tri_indices(nodes[i]),
                               *vbos[i]);
          });
        }
        else {
          nodes_to_update.foreach_index(GrainSize(64),
                                        [&](const int i) { vbos[i]->data<float>().fill(0.0f); });
        }
        break;
      }
      case CustomRequest::FaceSet: {
        const int face_set_default = orig_mesh_data.face_set_default;
        const int face_set_seed = orig_mesh_data.face_set_seed;
        const VArraySpan face_sets = *orig_mesh_data.attributes.lookup<int>(".sculpt_face_set",
                                                                            bke::AttrDomain::Face);
        if (!face_sets.is_empty()) {
          nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
            fill_vbo_face_set_mesh(tri_faces,
                                   hide_poly,
                                   face_sets,
                                   face_set_default,
                                   face_set_seed,
                                   bke::pbvh::node_tri_indices(nodes[i]),
                                   *vbos[i]);
          });
        }
        else {
          nodes_to_update.foreach_index(
              GrainSize(64), [&](const int i) { vbos[i]->data<uchar4>().fill(uchar4(255)); });
        }
        break;
      }
    }
  }
  else {
    const GenericRequest &attr = std::get<GenericRequest>(request);
    const StringRef name = attr.name;
    const bke::AttrDomain domain = attr.domain;
    const eCustomDataType data_type = attr.type;
    const GVArraySpan attribute = *attributes.lookup_or_default(name, domain, data_type);
    nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
      fill_vbo_attribute_mesh(corner_verts,
                              corner_tris,
                              tri_faces,
                              hide_poly,
                              attribute,
                              domain,
                              bke::pbvh::node_tri_indices(nodes[i]),
                              *vbos[i]);
    });
  }
}

static void fill_vbo_position_bmesh(const Set<BMFace *, 0> &faces, gpu::VertBuf &vbo)
{
  float3 *data = vbo.data<float3>().data();
  for (const BMFace *face : faces) {
    if (BM_elem_flag_test(face, BM_ELEM_HIDDEN)) {
      continue;
    }
    const BMLoop *l = face->l_first;
    *data = l->prev->v->co;
    data++;
    *data = l->v->co;
    data++;
    *data = l->next->v->co;
    data++;
  }
}

static void fill_vbo_normal_bmesh(const Set<BMFace *, 0> &faces, gpu::VertBuf &vbo)
{
  short4 *data = vbo.data<short4>().data();
  for (const BMFace *face : faces) {
    if (BM_elem_flag_test(face, BM_ELEM_HIDDEN)) {
      continue;
    }
    if (BM_elem_flag_test(face, BM_ELEM_SMOOTH)) {
      const BMLoop *l = face->l_first;
      *data = normal_float_to_short(l->prev->v->no);
      data++;
      *data = normal_float_to_short(l->v->no);
      data++;
      *data = normal_float_to_short(l->next->v->no);
      data++;
    }
    else {
      std::fill_n(data, 3, normal_float_to_short(face->no));
      data += 3;
    }
  }
}

static void fill_vbo_mask_bmesh(const Set<BMFace *, 0> &faces,
                                const int cd_offset,
                                gpu::VertBuf &vbo)
{
  float *data = vbo.data<float>().data();
  for (const BMFace *face : faces) {
    if (BM_elem_flag_test(face, BM_ELEM_HIDDEN)) {
      continue;
    }
    const BMLoop *l = face->l_first;
    *data = bmesh_cd_vert_get<float>(*l->prev->v, cd_offset);
    data++;
    *data = bmesh_cd_vert_get<float>(*l->v, cd_offset);
    data++;
    *data = bmesh_cd_vert_get<float>(*l->next->v, cd_offset);
    data++;
  }
}

static void fill_vbo_face_set_bmesh(const Set<BMFace *, 0> &faces,
                                    const int color_default,
                                    const int color_seed,
                                    const int offset,
                                    gpu::VertBuf &vbo)
{
  uchar4 *data = vbo.data<uchar4>().data();
  for (const BMFace *face : faces) {
    if (BM_elem_flag_test(face, BM_ELEM_HIDDEN)) {
      continue;
    }
    uchar4 color{UCHAR_MAX};
    const int fset = bmesh_cd_face_get<int>(*face, offset);
    if (fset != color_default) {
      BKE_paint_face_set_overlay_color_get(fset, color_seed, color);
    }
    std::fill_n(data, 3, color);
    data += 3;
  }
}

static void fill_vbo_attribute_bmesh(const Set<BMFace *, 0> &faces,
                                     const eCustomDataType data_type,
                                     const bke::AttrDomain domain,
                                     const int offset,
                                     gpu::VertBuf &vbo)
{
  bke::attribute_math::convert_to_static_type(data_type, [&](auto dummy) {
    using T = decltype(dummy);
    if constexpr (!std::is_void_v<typename AttributeConverter<T>::VBOType>) {
      switch (domain) {
        case bke::AttrDomain::Point:
          extract_data_vert_bmesh<T>(faces, offset, vbo);
          break;
        case bke::AttrDomain::Face:
          extract_data_face_bmesh<T>(faces, offset, vbo);
          break;
        case bke::AttrDomain::Corner:
          extract_data_corner_bmesh<T>(faces, offset, vbo);
          break;
        default:
          BLI_assert_unreachable();
      }
    }
  });
}

static void fill_vbos_bmesh(const Object &object,
                            const OrigMeshData &orig_mesh_data,
                            const IndexMask &nodes_to_update,
                            const AttributeRequest &request,
                            const MutableSpan<gpu::VertBuf *> vbos)
{
  const SculptSession &ss = *object.sculpt;
  const Span<bke::pbvh::BMeshNode> nodes = ss.pbvh->nodes<bke::pbvh::BMeshNode>();
  const BMesh &bm = *ss.bm;
  if (const CustomRequest *request_type = std::get_if<CustomRequest>(&request)) {
    switch (*request_type) {
      case CustomRequest::Position: {
        nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
          fill_vbo_position_bmesh(
              BKE_pbvh_bmesh_node_faces(&const_cast<bke::pbvh::BMeshNode &>(nodes[i])), *vbos[i]);
        });
        break;
      }
      case CustomRequest::Normal: {
        nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
          fill_vbo_normal_bmesh(
              BKE_pbvh_bmesh_node_faces(&const_cast<bke::pbvh::BMeshNode &>(nodes[i])), *vbos[i]);
        });
        break;
      }
      case CustomRequest::Mask: {
        const int cd_offset = CustomData_get_offset_named(
            &bm.vdata, CD_PROP_FLOAT, ".sculpt_mask");
        if (cd_offset != -1) {
          nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
            fill_vbo_mask_bmesh(
                BKE_pbvh_bmesh_node_faces(&const_cast<bke::pbvh::BMeshNode &>(nodes[i])),
                cd_offset,
                *vbos[i]);
          });
        }
        else {
          nodes_to_update.foreach_index(GrainSize(64),
                                        [&](const int i) { vbos[i]->data<float>().fill(0.0f); });
        }
        break;
      }
      case CustomRequest::FaceSet: {
        const int face_set_default = orig_mesh_data.face_set_default;
        const int face_set_seed = orig_mesh_data.face_set_seed;
        const int cd_offset = CustomData_get_offset_named(
            &bm.pdata, CD_PROP_INT32, ".sculpt_face_set");
        if (cd_offset != -1) {
          nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
            fill_vbo_face_set_bmesh(
                BKE_pbvh_bmesh_node_faces(&const_cast<bke::pbvh::BMeshNode &>(nodes[i])),
                face_set_default,
                face_set_seed,
                cd_offset,
                *vbos[i]);
          });
        }
        else {
          nodes_to_update.foreach_index(
              GrainSize(64), [&](const int i) { vbos[i]->data<uchar4>().fill(uchar4(255)); });
        }
        break;
      }
    }
  }
  else {
    const GenericRequest &attr = std::get<GenericRequest>(request);
    const bke::AttrDomain domain = attr.domain;
    const eCustomDataType data_type = attr.type;
    const CustomData &custom_data = *get_cdata(bm, domain);
    const int offset = CustomData_get_offset_named(&custom_data, data_type, attr.name);
    nodes_to_update.foreach_index(GrainSize(1), [&](const int i) {
      fill_vbo_attribute_bmesh(
          BKE_pbvh_bmesh_node_faces(&const_cast<bke::pbvh::BMeshNode &>(nodes[i])),
          data_type,
          domain,
          offset,
          *vbos[i]);
    });
  }
}

static gpu::IndexBuf *create_index_faces(const Span<int2> edges,
                                         const Span<int> corner_verts,
                                         const Span<int> corner_edges,
                                         const Span<int3> corner_tris,
                                         const Span<int> tri_faces,
                                         const Span<bool> hide_poly,
                                         const Span<int> tri_indices)
{
  /* Calculate number of edges. */
  int edge_count = 0;
  for (const int tri_i : tri_indices) {
    if (!hide_poly.is_empty() && hide_poly[tri_faces[tri_i]]) {
      continue;
    }
    const int3 real_edges = bke::mesh::corner_tri_get_real_edges(
        edges, corner_verts, corner_edges, corner_tris[tri_i]);
    if (real_edges[0] != -1) {
      edge_count++;
    }
    if (real_edges[1] != -1) {
      edge_count++;
    }
    if (real_edges[2] != -1) {
      edge_count++;
    }
  }

  GPUIndexBufBuilder builder;
  GPU_indexbuf_init(&builder, GPU_PRIM_LINES, edge_count, INT_MAX);
  MutableSpan<uint2> data = GPU_indexbuf_get_data(&builder).cast<uint2>();

  int edge_i = 0;
  int vert_i = 0;
  for (const int tri_i : tri_indices) {
    if (!hide_poly.is_empty() && hide_poly[tri_faces[tri_i]]) {
      continue;
    }

    const int3 real_edges = bke::mesh::corner_tri_get_real_edges(
        edges, corner_verts, corner_edges, corner_tris[tri_i]);

    if (real_edges[0] != -1) {
      data[edge_i] = uint2(vert_i, vert_i + 1);
      edge_i++;
    }
    if (real_edges[1] != -1) {
      data[edge_i] = uint2(vert_i + 1, vert_i + 2);
      edge_i++;
    }
    if (real_edges[2] != -1) {
      data[edge_i] = uint2(vert_i + 2, vert_i);
      edge_i++;
    }

    vert_i += 3;
  }

  gpu::IndexBuf *ibo = GPU_indexbuf_calloc();
  GPU_indexbuf_build_in_place_ex(&builder, 0, vert_i, false, ibo);
  return ibo;
}

static gpu::IndexBuf *create_index_bmesh(const Set<BMFace *, 0> &faces,
                                         const int visible_faces_num)
{
  GPUIndexBufBuilder elb_lines;
  GPU_indexbuf_init(&elb_lines, GPU_PRIM_LINES, visible_faces_num * 3, INT_MAX);

  int v_index = 0;

  for (const BMFace *face : faces) {
    if (BM_elem_flag_test(face, BM_ELEM_HIDDEN)) {
      continue;
    }

    GPU_indexbuf_add_line_verts(&elb_lines, v_index, v_index + 1);
    GPU_indexbuf_add_line_verts(&elb_lines, v_index + 1, v_index + 2);
    GPU_indexbuf_add_line_verts(&elb_lines, v_index + 2, v_index);

    v_index += 3;
  }

  return GPU_indexbuf_build(&elb_lines);
}

static void create_tri_index_grids(const Span<int> grid_indices,
                                   const BitGroupVector<> &grid_hidden,
                                   const int gridsize,
                                   const int skip,
                                   const int totgrid,
                                   GPUIndexBufBuilder &elb)
{
  uint offset = 0;
  const uint grid_vert_len = gridsize * gridsize;
  for (int i = 0; i < totgrid; i++, offset += grid_vert_len) {
    uint v0, v1, v2, v3;

    const BoundedBitSpan gh = grid_hidden.is_empty() ? BoundedBitSpan() :
                                                       grid_hidden[grid_indices[i]];

    for (int y = 0; y < gridsize - skip; y += skip) {
      for (int x = 0; x < gridsize - skip; x += skip) {
        /* Skip hidden grid face */
        if (!gh.is_empty() && paint_is_grid_face_hidden(gh, gridsize, x, y)) {
          continue;
        }
        /* Indices in a Clockwise QUAD disposition. */
        v0 = offset + CCG_grid_xy_to_index(gridsize, x, y);
        v1 = offset + CCG_grid_xy_to_index(gridsize, x + skip, y);
        v2 = offset + CCG_grid_xy_to_index(gridsize, x + skip, y + skip);
        v3 = offset + CCG_grid_xy_to_index(gridsize, x, y + skip);

        GPU_indexbuf_add_tri_verts(&elb, v0, v2, v1);
        GPU_indexbuf_add_tri_verts(&elb, v0, v3, v2);
      }
    }
  }
}

static void create_tri_index_grids_flat_layout(const Span<int> grid_indices,
                                               const BitGroupVector<> &grid_hidden,
                                               const int gridsize,
                                               const int skip,
                                               const int totgrid,
                                               GPUIndexBufBuilder &elb)
{
  uint offset = 0;
  const uint grid_vert_len = square_uint(gridsize - 1) * 4;

  for (int i = 0; i < totgrid; i++, offset += grid_vert_len) {
    const BoundedBitSpan gh = grid_hidden.is_empty() ? BoundedBitSpan() :
                                                       grid_hidden[grid_indices[i]];

    uint v0, v1, v2, v3;
    for (int y = 0; y < gridsize - skip; y += skip) {
      for (int x = 0; x < gridsize - skip; x += skip) {
        /* Skip hidden grid face */
        if (!gh.is_empty() && paint_is_grid_face_hidden(gh, gridsize, x, y)) {
          continue;
        }

        v0 = (y * (gridsize - 1) + x) * 4;

        if (skip > 1) {
          v1 = (y * (gridsize - 1) + x + skip - 1) * 4;
          v2 = ((y + skip - 1) * (gridsize - 1) + x + skip - 1) * 4;
          v3 = ((y + skip - 1) * (gridsize - 1) + x) * 4;
        }
        else {
          v1 = v2 = v3 = v0;
        }

        /* VBO data are in a Clockwise QUAD disposition.  Note
         * that vertices might be in different quads if we're
         * building a coarse index buffer.
         */
        v0 += offset;
        v1 += offset + 1;
        v2 += offset + 2;
        v3 += offset + 3;

        GPU_indexbuf_add_tri_verts(&elb, v0, v2, v1);
        GPU_indexbuf_add_tri_verts(&elb, v0, v3, v2);
      }
    }
  }
}

static void create_lines_index_grids(const Span<int> grid_indices,
                                     int display_gridsize,
                                     const BitGroupVector<> &grid_hidden,
                                     const int gridsize,
                                     const int skip,
                                     const int totgrid,
                                     GPUIndexBufBuilder &elb_lines)
{
  uint offset = 0;
  const uint grid_vert_len = gridsize * gridsize;
  for (int i = 0; i < totgrid; i++, offset += grid_vert_len) {
    uint v0, v1, v2, v3;
    bool grid_visible = false;

    const BoundedBitSpan gh = grid_hidden.is_empty() ? BoundedBitSpan() :
                                                       grid_hidden[grid_indices[i]];

    for (int y = 0; y < gridsize - skip; y += skip) {
      for (int x = 0; x < gridsize - skip; x += skip) {
        /* Skip hidden grid face */
        if (!gh.is_empty() && paint_is_grid_face_hidden(gh, gridsize, x, y)) {
          continue;
        }
        /* Indices in a Clockwise QUAD disposition. */
        v0 = offset + CCG_grid_xy_to_index(gridsize, x, y);
        v1 = offset + CCG_grid_xy_to_index(gridsize, x + skip, y);
        v2 = offset + CCG_grid_xy_to_index(gridsize, x + skip, y + skip);
        v3 = offset + CCG_grid_xy_to_index(gridsize, x, y + skip);

        GPU_indexbuf_add_line_verts(&elb_lines, v0, v1);
        GPU_indexbuf_add_line_verts(&elb_lines, v0, v3);

        if (y / skip + 2 == display_gridsize) {
          GPU_indexbuf_add_line_verts(&elb_lines, v2, v3);
        }
        grid_visible = true;
      }

      if (grid_visible) {
        GPU_indexbuf_add_line_verts(&elb_lines, v1, v2);
      }
    }
  }
}

static void create_lines_index_grids_flat_layout(const Span<int> grid_indices,
                                                 int display_gridsize,
                                                 const BitGroupVector<> &grid_hidden,
                                                 const int gridsize,
                                                 const int skip,
                                                 const int totgrid,
                                                 GPUIndexBufBuilder &elb_lines)
{
  uint offset = 0;
  const uint grid_vert_len = square_uint(gridsize - 1) * 4;

  for (int i = 0; i < totgrid; i++, offset += grid_vert_len) {
    bool grid_visible = false;
    const BoundedBitSpan gh = grid_hidden.is_empty() ? BoundedBitSpan() :
                                                       grid_hidden[grid_indices[i]];

    uint v0, v1, v2, v3;
    for (int y = 0; y < gridsize - skip; y += skip) {
      for (int x = 0; x < gridsize - skip; x += skip) {
        /* Skip hidden grid face */
        if (!gh.is_empty() && paint_is_grid_face_hidden(gh, gridsize, x, y)) {
          continue;
        }

        v0 = (y * (gridsize - 1) + x) * 4;

        if (skip > 1) {
          v1 = (y * (gridsize - 1) + x + skip - 1) * 4;
          v2 = ((y + skip - 1) * (gridsize - 1) + x + skip - 1) * 4;
          v3 = ((y + skip - 1) * (gridsize - 1) + x) * 4;
        }
        else {
          v1 = v2 = v3 = v0;
        }

        /* VBO data are in a Clockwise QUAD disposition.  Note
         * that vertices might be in different quads if we're
         * building a coarse index buffer.
         */
        v0 += offset;
        v1 += offset + 1;
        v2 += offset + 2;
        v3 += offset + 3;

        GPU_indexbuf_add_line_verts(&elb_lines, v0, v1);
        GPU_indexbuf_add_line_verts(&elb_lines, v0, v3);

        if (y / skip + 2 == display_gridsize) {
          GPU_indexbuf_add_line_verts(&elb_lines, v2, v3);
        }
        grid_visible = true;
      }

      if (grid_visible) {
        GPU_indexbuf_add_line_verts(&elb_lines, v1, v2);
      }
    }
  }
}

static void calc_material_indices(const Object &object,
                                  const IndexMask &nodes_to_update,
                                  MutableSpan<int> node_materials)
{
  const SculptSession &ss = *object.sculpt;
  const bke::pbvh::Tree &pbvh = *ss.pbvh;
  switch (pbvh.type()) {
    case bke::pbvh::Type::Mesh: {
      const Span<bke::pbvh::MeshNode> nodes = pbvh.nodes<bke::pbvh::MeshNode>();
      const Mesh &mesh = *static_cast<const Mesh *>(object.data);
      const Span<int> tri_faces = mesh.corner_tri_faces();
      const bke::AttributeAccessor attributes = mesh.attributes();
      const VArray material_indices = *attributes.lookup_or_default<int>(
          "material_index", bke::AttrDomain::Face, 0);
      nodes_to_update.foreach_index(GrainSize(64), [&](const int i) {
        const Span<int> tris = bke::pbvh::node_tri_indices(nodes[i]);
        if (tris.is_empty()) {
          node_materials[i] = 0;
        }
        else {
          node_materials[i] = material_indices[tri_faces[tris.first()]];
        }
      });
      break;
    }
    case bke::pbvh::Type::Grids: {
      const Span<bke::pbvh::GridsNode> nodes = pbvh.nodes<bke::pbvh::GridsNode>();
      const Mesh &mesh = *static_cast<const Mesh *>(object.data);
      const bke::AttributeAccessor attributes = mesh.attributes();
      const VArray material_indices = *attributes.lookup_or_default<int>(
          "material_index", bke::AttrDomain::Face, 0);
      const SubdivCCG &subdiv_ccg = *ss.subdiv_ccg;
      const Span<int> grid_faces = subdiv_ccg.grid_to_face_map;
      nodes_to_update.foreach_index(GrainSize(64), [&](const int i) {
        const Span<int> grids = bke::pbvh::node_grid_indices(nodes[i]);
        if (grids.is_empty()) {
          node_materials[i] = 0;
        }
        else {
          node_materials[i] = material_indices[grid_faces[grids.first()]];
        }
      });
      break;
    }
    case bke::pbvh::Type::BMesh:
      node_materials.fill(0);
      break;
  }
}

static void ensure_use_flat_layout_check(const Object &object, DrawCache &draw_data)
{
  const bke::pbvh::Tree &pbvh = *object.sculpt->pbvh;
  switch (pbvh.type()) {
    case bke::pbvh::Type::Mesh:
      break;
    case bke::pbvh::Type::Grids: {
      const Span<bke::pbvh::GridsNode> nodes = pbvh.nodes<bke::pbvh::GridsNode>();
      if (draw_data.use_flat_layout.size() == nodes.size()) {
        return;
      }
      draw_data.use_flat_layout.resize(nodes.size());
      const MutableSpan<bool> use_flat_layout = draw_data.use_flat_layout;

      const Mesh &mesh = *static_cast<const Mesh *>(object.data);
      const bke::AttributeAccessor attributes = mesh.attributes();
      const VArraySpan sharp_faces = *attributes.lookup<bool>("sharp_face", bke::AttrDomain::Face);
      if (sharp_faces.is_empty()) {
        use_flat_layout.fill(false);
      }
      else {
        const SubdivCCG &subdiv_ccg = *object.sculpt->subdiv_ccg;
        const Span<int> grid_to_face_map = subdiv_ccg.grid_to_face_map;
        threading::parallel_for(nodes.index_range(), 4, [&](const IndexRange range) {
          for (const int i : range) {
            const Span<int> grids = bke::pbvh::node_grid_indices(nodes[i]);
            use_flat_layout[i] = std::any_of(grids.begin(), grids.end(), [&](const int grid) {
              return sharp_faces[grid_to_face_map[grid]];
            });
          }
        });
      }
      break;
    }
    case bke::pbvh::Type::BMesh:
      break;
  }
}

static gpu::IndexBuf *create_tri_index_grids(const CCGKey &key,
                                             const BitGroupVector<> &grid_hidden,
                                             const bool do_coarse,
                                             const Span<int> grid_indices,
                                             const bool use_flat_layout)
{
  int gridsize = key.grid_size;
  int display_gridsize = gridsize;
  int totgrid = grid_indices.size();
  int skip = 1;

  const int display_level = do_coarse ? 0 : key.level;

  if (display_level < key.level) {
    display_gridsize = (1 << display_level) + 1;
    skip = 1 << (key.level - display_level - 1);
  }

  GPUIndexBufBuilder elb;

  uint visible_quad_len = bke::pbvh::count_grid_quads(
      grid_hidden, grid_indices, key.grid_size, display_gridsize);

  GPU_indexbuf_init(&elb, GPU_PRIM_TRIS, 2 * visible_quad_len, INT_MAX);

  if (use_flat_layout) {
    create_tri_index_grids_flat_layout(grid_indices, grid_hidden, gridsize, skip, totgrid, elb);
  }
  else {
    create_tri_index_grids(grid_indices, grid_hidden, gridsize, skip, totgrid, elb);
  }

  return GPU_indexbuf_build(&elb);
}

static gpu::IndexBuf *create_lines_index_grids(const CCGKey &key,
                                               const BitGroupVector<> &grid_hidden,
                                               const bool do_coarse,
                                               const Span<int> grid_indices,
                                               const bool use_flat_layout)
{
  int gridsize = key.grid_size;
  int display_gridsize = gridsize;
  int totgrid = grid_indices.size();
  int skip = 1;

  const int display_level = do_coarse ? 0 : key.level;

  if (display_level < key.level) {
    display_gridsize = (1 << display_level) + 1;
    skip = 1 << (key.level - display_level - 1);
  }

  GPUIndexBufBuilder elb;
  GPU_indexbuf_init(
      &elb, GPU_PRIM_LINES, 2 * totgrid * display_gridsize * (display_gridsize - 1), INT_MAX);

  if (use_flat_layout) {
    create_lines_index_grids_flat_layout(
        grid_indices, display_gridsize, grid_hidden, gridsize, skip, totgrid, elb);
  }
  else {
    create_lines_index_grids(
        grid_indices, display_gridsize, grid_hidden, gridsize, skip, totgrid, elb);
  }

  return GPU_indexbuf_build(&elb);
}

static Span<gpu::IndexBuf *> calc_lines_ibos(const Object &object,
                                             const OrigMeshData &orig_mesh_data,
                                             const IndexMask &nodes_to_update,
                                             const bool coarse,
                                             DrawCache &draw_data)
{
  const bke::pbvh::Tree &pbvh = *object.sculpt->pbvh;
  draw_data.tris_ibos.resize(pbvh.nodes_num(), nullptr);
  MutableSpan<gpu::IndexBuf *> ibos = coarse ? draw_data.tris_ibos_coarse : draw_data.tris_ibos;

  IndexMaskMemory memory;
  const IndexMask nodes_to_calculate = IndexMask::from_predicate(
      nodes_to_update, GrainSize(8196), memory, [&](const int i) { return !ibos[i]; });

  switch (pbvh.type()) {
    case bke::pbvh::Type::Mesh: {
      const Span<bke::pbvh::MeshNode> nodes = pbvh.nodes<bke::pbvh::MeshNode>();
      const Mesh &mesh = *static_cast<const Mesh *>(object.data);
      const Span<int2> edges = mesh.edges();
      const Span<int> corner_verts = mesh.corner_verts();
      const Span<int> corner_edges = mesh.corner_edges();
      const Span<int3> corner_tris = mesh.corner_tris();
      const Span<int> tri_faces = mesh.corner_tri_faces();
      const bke::AttributeAccessor attributes = orig_mesh_data.attributes;
      const VArraySpan hide_poly = *attributes.lookup<bool>(".hide_poly", bke::AttrDomain::Face);
      nodes_to_calculate.foreach_index(GrainSize(1), [&](const int i) {
        ibos[i] = create_index_faces(edges,
                                     corner_verts,
                                     corner_edges,
                                     corner_tris,
                                     tri_faces,
                                     hide_poly,
                                     bke::pbvh::node_tri_indices(nodes[i]));
      });
      break;
    }
    case bke::pbvh::Type::Grids: {
      const Span<bke::pbvh::GridsNode> nodes = pbvh.nodes<bke::pbvh::GridsNode>();
      nodes_to_calculate.foreach_index(GrainSize(1), [&](const int i) {
        const SubdivCCG &subdiv_ccg = *object.sculpt->subdiv_ccg;
        const CCGKey key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);
        ibos[i] = create_lines_index_grids(key,
                                           subdiv_ccg.grid_hidden,
                                           coarse,
                                           bke::pbvh::node_grid_indices(nodes[i]),
                                           draw_data.use_flat_layout[i]);
      });
      break;
    }
    case bke::pbvh::Type::BMesh: {
      const Span<bke::pbvh::BMeshNode> nodes = pbvh.nodes<bke::pbvh::BMeshNode>();
      nodes_to_calculate.foreach_index(GrainSize(1), [&](const int i) {
        const Set<BMFace *, 0> &faces = BKE_pbvh_bmesh_node_faces(
            &const_cast<bke::pbvh::BMeshNode &>(nodes[i]));
        const int visible_faces_num = count_visible_tris_bmesh(faces);  // TODO
        ibos[i] = create_index_bmesh(faces, visible_faces_num);
      });
      break;
    }
  }

  return draw_data.tris_ibos;
}

BLI_NOINLINE static void ensure_vbos_allocated_mesh(const Object &object,
                                                    const GPUVertFormat &format,
                                                    const IndexMask &nodes_to_update,
                                                    const MutableSpan<gpu::VertBuf *> vbos)
{
  const SculptSession &ss = *object.sculpt;
  const Span<bke::pbvh::MeshNode> nodes = ss.pbvh->nodes<bke::pbvh::MeshNode>();
  const Mesh &mesh = *static_cast<Mesh *>(object.data);
  const Span<int> tri_faces = mesh.corner_tri_faces();
  const bke::AttributeAccessor attributes = mesh.attributes();
  const VArraySpan hide_poly = *attributes.lookup<bool>(".hide_poly", bke::AttrDomain::Face);
  nodes_to_update.foreach_index(GrainSize(64), [&](const int i) {
    if (!vbos[i]) {
      vbos[i] = GPU_vertbuf_create_with_format(format);
    }
    const Span<int> tris = bke::pbvh::node_tri_indices(nodes[i]);
    const int verts_num = count_visible_tris_mesh(tris, tri_faces, hide_poly) * 3;

    if (vbos[i]->data<uchar>().data() == nullptr ||
        GPU_vertbuf_get_vertex_len(vbos[i]) != verts_num)
    {
      GPU_vertbuf_data_alloc(*vbos[i], verts_num);
    }
  });
}

BLI_NOINLINE static void ensure_vbos_allocated_grids(const Object &object,
                                                     const GPUVertFormat &format,
                                                     const Span<bool> use_flat_layout,
                                                     const IndexMask &nodes_to_update,
                                                     const MutableSpan<gpu::VertBuf *> vbos)
{
  const SculptSession &ss = *object.sculpt;
  const Span<bke::pbvh::GridsNode> nodes = ss.pbvh->nodes<bke::pbvh::GridsNode>();
  const SubdivCCG &subdiv_ccg = *ss.subdiv_ccg;
  const CCGKey key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);
  nodes_to_update.foreach_index(GrainSize(64), [&](const int i) {
    if (!vbos[i]) {
      vbos[i] = GPU_vertbuf_create_with_format(format);
    }
    const int verts_per_grid = use_flat_layout[i] ? square_i(key.grid_size - 1) * 4 :
                                                    square_i(key.grid_size);
    const int verts_num = bke::pbvh::node_grid_indices(nodes[i]).size() * verts_per_grid;
    if (vbos[i]->data<uchar>().data() == nullptr ||
        GPU_vertbuf_get_vertex_len(vbos[i]) != verts_num)
    {
      GPU_vertbuf_data_alloc(*vbos[i], verts_num);
    }
  });
}

BLI_NOINLINE static void ensure_vbos_allocated_bmesh(const Object &object,
                                                     const GPUVertFormat &format,
                                                     const IndexMask &nodes_to_update,
                                                     const MutableSpan<gpu::VertBuf *> vbos)
{
  SculptSession &ss = *object.sculpt;
  const Span<bke::pbvh::BMeshNode> nodes = ss.pbvh->nodes<bke::pbvh::BMeshNode>();

  nodes_to_update.foreach_index(GrainSize(64), [&](const int i) {
    if (!vbos[i]) {
      vbos[i] = GPU_vertbuf_create_with_format(format);
    }
    const Set<BMFace *, 0> &faces = BKE_pbvh_bmesh_node_faces(
        &const_cast<bke::pbvh::BMeshNode &>(nodes[i]));
    const int verts_num = count_visible_tris_bmesh(faces) * 3;
    if (vbos[i]->data<uchar>().data() == nullptr ||
        GPU_vertbuf_get_vertex_len(vbos[i]) != verts_num)
    {
      GPU_vertbuf_data_alloc(*vbos[i], verts_num);
    }
  });
}

static Span<gpu::VertBuf *> calc_vbos(const Object &object,
                                      const OrigMeshData &orig_mesh_data,
                                      const IndexMask &nodes_to_update,
                                      const AttributeRequest &attr,
                                      DrawCache &draw_data)
{
  if (!pbvh_attr_supported(attr)) {
    return {};
  }
  const bke::pbvh::Tree &pbvh = *object.sculpt->pbvh;
  Vector<gpu::VertBuf *> &vbos = draw_data.attribute_vbos.lookup_or_add_default(attr);
  vbos.resize(pbvh.nodes_num(), nullptr);

  const GPUVertFormat format = format_for_request(orig_mesh_data, attr);

  switch (pbvh.type()) {
    case bke::pbvh::Type::Mesh: {
      ensure_vbos_allocated_mesh(object, format, nodes_to_update, vbos);
      fill_vbos_mesh(object, orig_mesh_data, nodes_to_update, attr, vbos);
      break;
    }
    case bke::pbvh::Type::Grids: {
      ensure_vbos_allocated_grids(
          object, format, draw_data.use_flat_layout, nodes_to_update, vbos);
      fill_vbos_grids(
          object, orig_mesh_data, draw_data.use_flat_layout, nodes_to_update, attr, vbos);
      break;
    }
    case bke::pbvh::Type::BMesh: {
      ensure_vbos_allocated_bmesh(object, format, nodes_to_update, vbos);
      fill_vbos_bmesh(object, orig_mesh_data, nodes_to_update, attr, vbos);
      break;
    }
  }

  return vbos;
}

BLI_NOINLINE static Span<gpu::IndexBuf *> ensure_tris_ibos(const Object &object,
                                                           const IndexMask &nodes_to_update,
                                                           const bool coarse,
                                                           DrawCache &draw_data)
{
  const bke::pbvh::Tree &pbvh = *object.sculpt->pbvh;
  switch (pbvh.type()) {
    case bke::pbvh::Type::Mesh:
      break;
    case bke::pbvh::Type::Grids: {
      const Span<bke::pbvh::GridsNode> nodes = pbvh.nodes<bke::pbvh::GridsNode>();

      draw_data.tris_ibos.resize(nodes.size(), nullptr);
      const MutableSpan<gpu::IndexBuf *> ibos = coarse ? draw_data.tris_ibos_coarse :
                                                         draw_data.tris_ibos;

      IndexMaskMemory memory;
      const IndexMask nodes_to_calculate = IndexMask::from_predicate(
          nodes_to_update, GrainSize(8196), memory, [&](const int i) { return !ibos[i]; });

      const SubdivCCG &subdiv_ccg = *object.sculpt->subdiv_ccg;
      const CCGKey key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);

      nodes_to_calculate.foreach_index(GrainSize(1), [&](const int i) {
        ibos[i] = create_tri_index_grids(key,
                                         subdiv_ccg.grid_hidden,
                                         coarse,
                                         bke::pbvh::node_grid_indices(nodes[i]),
                                         draw_data.use_flat_layout[i]);
      });
      break;
    }
    case bke::pbvh::Type::BMesh:
      break;
  }
  return draw_data.tris_ibos;
}

BLI_NOINLINE static void flush_vbo_data(const Span<gpu::VertBuf *> vbos,
                                        const IndexMask &nodes_to_update)
{
  nodes_to_update.foreach_index([&](const int i) { GPU_vertbuf_use(vbos[i]); });
}

Span<gpu::Batch *> ensure_tris_batches(const Object &object,
                                       const ViewportRequest &request,
                                       const IndexMask &nodes_to_update,
                                       DrawCache &draw_data)
{
  const Object &object_orig = *DEG_get_original_object(&const_cast<Object &>(object));
  const OrigMeshData orig_mesh_data{*static_cast<const Mesh *>(object_orig.data)};

  const bke::pbvh::Tree &pbvh = *object.sculpt->pbvh;

  ensure_use_flat_layout_check(object, draw_data);

  Vector<gpu::Batch *> &batches = draw_data.tris_batches.lookup_or_add_default(request);
  batches.resize(pbvh.nodes_num(), nullptr);

  IndexMaskMemory memory;
  IndexMask node_mask;
  switch (pbvh.type()) {
    case bke::pbvh::Type::Mesh: {
      const Span<bke::pbvh::MeshNode> nodes = pbvh.nodes<bke::pbvh::MeshNode>();
      node_mask = IndexMask::from_predicate(
          nodes_to_update, GrainSize(1024), memory, [&](const int i) {
            return nodes[i].flag_ & PBVH_UpdateDrawBuffers || batches[i] == nullptr;
          });
      break;
    }
    case bke::pbvh::Type::Grids: {
      const Span<bke::pbvh::GridsNode> nodes = pbvh.nodes<bke::pbvh::GridsNode>();
      node_mask = IndexMask::from_predicate(
          nodes_to_update, GrainSize(1024), memory, [&](const int i) {
            return nodes[i].flag_ & PBVH_UpdateDrawBuffers || batches[i] == nullptr;
          });
      break;
    }
    case bke::pbvh::Type::BMesh: {
      const Span<bke::pbvh::BMeshNode> nodes = pbvh.nodes<bke::pbvh::BMeshNode>();
      node_mask = IndexMask::from_predicate(
          nodes_to_update, GrainSize(1024), memory, [&](const int i) {
            return nodes[i].flag_ & PBVH_UpdateDrawBuffers || batches[i] == nullptr;
          });
      break;
    }
  }

  const Span<gpu::IndexBuf *> ibos = ensure_tris_ibos(
      object, node_mask, request.use_coarse_grids, draw_data);

  for (const AttributeRequest &attr : request.attributes) {
    calc_vbos(object, orig_mesh_data, node_mask, attr, draw_data);
  }

  const IndexMask batches_to_create = IndexMask::from_predicate(
      node_mask, GrainSize(8196), memory, [&](const int i) { return !batches[i]; });
  batches_to_create.foreach_index(GrainSize(64), [&](const int i) {
    batches[i] = GPU_batch_create(GPU_PRIM_TRIS, nullptr, ibos.is_empty() ? nullptr : ibos[i]);
  });

  for (const AttributeRequest &attr : request.attributes) {
    Span<gpu::VertBuf *> vbos = draw_data.attribute_vbos.lookup(attr);
    if (vbos.is_empty()) {
      continue;
    }
    batches_to_create.foreach_index(
        GrainSize(64), [&](const int i) { GPU_batch_vertbuf_add(batches[i], vbos[i], false); });
  }

  for (const AttributeRequest &attr : request.attributes) {
    flush_vbo_data(draw_data.attribute_vbos.lookup(attr), node_mask);
  }

  remove_node_tags(const_cast<bke::pbvh::Tree &>(pbvh), node_mask);

  return batches;
}

Span<gpu::Batch *> ensure_lines_batches(const Object &object,
                                        const ViewportRequest &request,
                                        const IndexMask &nodes_to_update,
                                        DrawCache &draw_data)
{
  const Object &object_orig = *DEG_get_original_object(&const_cast<Object &>(object));
  const OrigMeshData orig_mesh_data(*static_cast<const Mesh *>(object_orig.data));

  const bke::pbvh::Tree &pbvh = *object.sculpt->pbvh;

  Vector<gpu::Batch *> &batches = request.use_coarse_grids ? draw_data.lines_batches_coarse :
                                                             draw_data.lines_batches;
  batches.resize(pbvh.nodes_num(), nullptr);

  free_stale_node_data(object, nodes_to_update, draw_data);

  IndexMaskMemory memory;
  IndexMask node_mask;
  switch (pbvh.type()) {
    case bke::pbvh::Type::Mesh: {
      const Span<bke::pbvh::MeshNode> nodes = pbvh.nodes<bke::pbvh::MeshNode>();
      node_mask = IndexMask::from_predicate(
          nodes_to_update, GrainSize(1024), memory, [&](const int i) {
            return nodes[i].flag_ & PBVH_UpdateDrawBuffers || batches[i] == nullptr;
          });
      break;
    }
    case bke::pbvh::Type::Grids: {
      const Span<bke::pbvh::GridsNode> nodes = pbvh.nodes<bke::pbvh::GridsNode>();
      node_mask = IndexMask::from_predicate(
          nodes_to_update, GrainSize(1024), memory, [&](const int i) {
            return nodes[i].flag_ & PBVH_UpdateDrawBuffers || batches[i] == nullptr;
          });
      break;
    }
    case bke::pbvh::Type::BMesh: {
      const Span<bke::pbvh::BMeshNode> nodes = pbvh.nodes<bke::pbvh::BMeshNode>();
      node_mask = IndexMask::from_predicate(
          nodes_to_update, GrainSize(1024), memory, [&](const int i) {
            return nodes[i].flag_ & PBVH_UpdateDrawBuffers || batches[i] == nullptr;
          });
      break;
    }
  }

  const Span<gpu::VertBuf *> position_vbos = calc_vbos(
      object, orig_mesh_data, node_mask, CustomRequest::Position, draw_data);
  const Span<gpu::IndexBuf *> ibos = calc_lines_ibos(
      object, orig_mesh_data, node_mask, request.use_coarse_grids, draw_data);

  node_mask.foreach_index(GrainSize(1), [&](const int i) {
    if (!batches[i]) {
      batches[i] = GPU_batch_create(GPU_PRIM_LINES, nullptr, ibos[i]);
      GPU_batch_vertbuf_add(batches[i], position_vbos[i], false);
    }
  });

  flush_vbo_data(position_vbos, node_mask);

  remove_node_tags(const_cast<bke::pbvh::Tree &>(pbvh), node_mask);

  return draw_data.lines_batches;
}

Span<int> ensure_material_indices(const Object &object, DrawCache &draw_data)
{
  const bke::pbvh::Tree &pbvh = *object.sculpt->pbvh;
  const int nodes_num = pbvh.nodes_num();
  if (draw_data.material_indices.size() != nodes_num) {
    // TODO: Allow to be empty when no material indices
    draw_data.material_indices.reinitialize(nodes_num);
    calc_material_indices(object, IndexRange(nodes_num), draw_data.material_indices);
  }
  return draw_data.material_indices;
}

}  // namespace blender::draw::pbvh
