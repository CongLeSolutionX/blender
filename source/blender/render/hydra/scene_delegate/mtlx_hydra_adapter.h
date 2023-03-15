/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/base/tf/token.h>
#include <pxr/pxr.h>

#include <string>

struct pxr::HdMaterialNetworkMap;

void HdMtlxConvertToMaterialNetworkMap(std::string const &mtlx_path,
                                       pxr::TfTokenVector const &shader_source_types,
                                       pxr::TfTokenVector const &render_contexts,
                                       pxr::HdMaterialNetworkMap *out);
