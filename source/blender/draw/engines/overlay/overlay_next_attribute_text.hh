/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 */

#include "BLI_math_quaternion_types.hh"

#include "DNA_curve_types.h"

#include "BKE_attribute.hh"
#include "BKE_curves.hh"
#include "BKE_duplilist.hh"
#include "BKE_geometry_set.hh"
#include "BKE_instances.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"

#include "DRW_render.hh"

#include "UI_resources.hh"

#include "draw_manager_text.hh"

#include "overlay_next_base.hh"

namespace blender::draw::overlay {

/**
 * Displays geometry node viewer output.
 * Values are displayed as text on top of the active object.
 */
class AttributeTexts : Overlay {
 public:
  void begin_sync(Resources &res, const State &state) final
  {
    enabled_ = !res.is_selection() && state.show_attribute_viewer_text();
  }

  void object_sync(Manager & /*manager*/,
                   const ObjectRef &ob_ref,
                   Resources & /*res*/,
                   const State &state) final
  {
    if (!enabled_) {
      return;
    }

    const Object &object = *ob_ref.object;
    const DupliObject *dupli_object = ob_ref.dupli_object;
    const bool is_preview = dupli_object != nullptr &&
                            dupli_object->preview_base_geometry != nullptr;
    if (!is_preview) {
      return;
    }

    DRWTextStore *dt = state.dt;
    const float4x4 &object_to_world = object.object_to_world();

    if (dupli_object->preview_instance_index >= 0) {
      const bke::Instances *instances = dupli_object->preview_base_geometry->get_instances();
      if (instances->attributes().contains(".viewer")) {
        add_instance_attributes_to_text_cache(
            dt, instances->attributes(), object_to_world, dupli_object->preview_instance_index);

        return;
      }
    }

    switch (object.type) {
      case OB_MESH: {
        const Mesh *mesh = static_cast<Mesh *>(object.data);
        add_attributes_to_text_cache(dt, mesh->attributes(), object_to_world);
        break;
      }
      case OB_POINTCLOUD: {
        const PointCloud *pointcloud = static_cast<PointCloud *>(object.data);
        add_attributes_to_text_cache(dt, pointcloud->attributes(), object_to_world);
        break;
      }
      case OB_CURVES_LEGACY: {
        const Curve *curve = static_cast<const Curve *>(object.data);
        if (curve->curve_eval) {
          const bke::CurvesGeometry &curves = curve->curve_eval->geometry.wrap();
          add_attributes_to_text_cache(dt, curves.attributes(), object_to_world);
        }
        break;
      }
      case OB_CURVES: {
        const Curves *curves_id = static_cast<Curves *>(object.data);
        const bke::CurvesGeometry &curves = curves_id->geometry.wrap();
        add_attributes_to_text_cache(dt, curves.attributes(), object_to_world);
        break;
      }
    }
  }

 private:
  void add_attributes_to_text_cache(DRWTextStore *dt,
                                    bke::AttributeAccessor attribute_accessor,
                                    const float4x4 &object_to_world)
  {
    if (!attribute_accessor.contains(".viewer")) {
      return;
    }

    const bke::GAttributeReader attribute = attribute_accessor.lookup(".viewer");
    const VArraySpan<float3> positions = *attribute_accessor.lookup<float3>("position",
                                                                            attribute.domain);

    add_values_to_text_cache(dt, attribute.varray, positions, object_to_world);
  }

  void add_instance_attributes_to_text_cache(DRWTextStore *dt,
                                             bke::AttributeAccessor attribute_accessor,
                                             const float4x4 &object_to_world,
                                             int instance_index)
  {
    /* Data from instances are read as a single value from a given index. The data is converted
     * back to an array so one function can handle both instance and object data. */
    const GVArray attribute = attribute_accessor.lookup(".viewer").varray.slice(
        IndexRange(instance_index, 1));

    add_values_to_text_cache(dt, attribute, {float3(0, 0, 0)}, object_to_world);
  }

  void add_values_to_text_cache(DRWTextStore *dt,
                                const GVArray &values,
                                const Span<float3> positions,
                                const float4x4 &object_to_world)
  {
    uchar col[4];
    UI_GetThemeColor4ubv(TH_TEXT_HI, col);

    bke::attribute_math::convert_to_static_type(values.type(), [&](auto dummy) {
      using T = decltype(dummy);
      const VArray<T> &values_typed = values.typed<T>();
      for (const int i : values.index_range()) {
        const float3 position = math::transform_point(object_to_world, positions[i]);
        const T &value = values_typed[i];

        char numstr[64];
        size_t numstr_len = 0;
        if constexpr (std::is_same_v<T, bool>) {
          numstr_len = SNPRINTF_RLEN(numstr, "%s", value ? "True" : "False");
        }
        else if constexpr (std::is_same_v<T, int8_t>) {
          numstr_len = SNPRINTF_RLEN(numstr, "%d", int(value));
        }
        else if constexpr (std::is_same_v<T, int>) {
          numstr_len = SNPRINTF_RLEN(numstr, "%d", value);
        }
        else if constexpr (std::is_same_v<T, int2>) {
          numstr_len = SNPRINTF_RLEN(numstr, "(%d, %d)", value.x, value.y);
        }
        else if constexpr (std::is_same_v<T, float>) {
          numstr_len = SNPRINTF_RLEN(numstr, "%g", value);
        }
        else if constexpr (std::is_same_v<T, float2>) {
          numstr_len = SNPRINTF_RLEN(numstr, "(%g, %g)", value.x, value.y);
        }
        else if constexpr (std::is_same_v<T, float3>) {
          numstr_len = SNPRINTF_RLEN(numstr, "(%g, %g, %g)", value.x, value.y, value.z);
        }
        else if constexpr (std::is_same_v<T, ColorGeometry4b>) {
          const ColorGeometry4f color = value.decode();
          numstr_len = SNPRINTF_RLEN(
              numstr, "(%.3f, %.3f, %.3f, %.3f)", color.r, color.g, color.b, color.a);
        }
        else if constexpr (std::is_same_v<T, ColorGeometry4f>) {
          numstr_len = SNPRINTF_RLEN(
              numstr, "(%.3f, %.3f, %.3f, %.3f)", value.r, value.g, value.b, value.a);
        }
        else if constexpr (std::is_same_v<T, math::Quaternion>) {
          numstr_len = SNPRINTF_RLEN(
              numstr, "(%.3f, %.3f, %.3f, %.3f)", value.w, value.x, value.y, value.z);
        }
        else {
          BLI_assert_unreachable();
        }

        DRW_text_cache_add(
            dt, position, numstr, numstr_len, 0, 0, DRW_TEXT_CACHE_GLOBALSPACE, col, true, true);
      }
    });
  }
};

}  // namespace blender::draw::overlay
