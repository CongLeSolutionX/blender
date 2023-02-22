/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2020 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup editors
 */

#pragma once

#include "BKE_attribute.h"
#include "DNA_customdata_types.h"

#ifdef __cplusplus

#  include "BLI_generic_pointer.hh"
#  include "BLI_string_ref.hh"

struct PointerRNA;
struct PropertyRNA;

namespace blender::ed::geometry {

/* -------------------------------------------------------------------- */
/** \name Attribute Value RNA Property Helpers
 *
 * Functions to make it easier to register RNA properties for the various attribute types and
 * retrieve/set their values.
 * \{ */

StringRefNull rna_property_name_for_type(eCustomDataType type);
PropertyRNA *rna_property_for_type(PointerRNA &ptr, const eCustomDataType type);
void register_rna_properties_for_attribute_types(StructRNA &srna);
GPointer rna_property_for_attribute_type_retrieve_value(PointerRNA &ptr,
                                                        const eCustomDataType type,
                                                        void *buffer);
void rna_property_for_attribute_type_set_value(PointerRNA &ptr, PropertyRNA &prop, GPointer value);

/** \} */

}  // namespace blender::ed::geometry

#endif

#ifdef __cplusplus
extern "C" {
#endif

struct Mesh;
struct ReportList;

void ED_operatortypes_geometry(void);

/**
 * Convert an attribute with the given name to a new type and domain.
 * The attribute must already exist.
 *
 * \note Does not support meshes in edit mode.
 */
bool ED_geometry_attribute_convert(struct Mesh *mesh,
                                   const char *name,
                                   eCustomDataType dst_type,
                                   eAttrDomain dst_domain,
                                   ReportList *reports);
#ifdef __cplusplus
}
#endif
