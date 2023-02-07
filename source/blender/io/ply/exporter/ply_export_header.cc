/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup ply
 */

#include "BKE_blender_version.h"
#include "BKE_customdata.h"

#include "IO_ply.h"
#include "ply_data.hh"
#include "ply_file_buffer.hh"

namespace blender::io::ply {

void write_header(std::unique_ptr<FileBuffer> &buffer,
                  std::unique_ptr<PlyData> &plyData,
                  const PLYExportParams &export_params)
{
  buffer->write_string("ply");

  StringRef format = export_params.ascii_format ? "ascii" : "binary_little_endian";
  buffer->write_string("format " + format + " 1.0");

  StringRef version = BKE_blender_version_string();
  buffer->write_string("comment Created in Blender version " + version);

  buffer->write_header_element("vertex", int32_t(plyData->vertices.size()));
  buffer->write_header_scalar_property("float", "x");
  buffer->write_header_scalar_property("float", "y");
  buffer->write_header_scalar_property("float", "z");

  if (!plyData->vertex_normals.is_empty()) {
    buffer->write_header_scalar_property("float", "nx");
    buffer->write_header_scalar_property("float", "ny");
    buffer->write_header_scalar_property("float", "nz");
  }

  if (!plyData->vertex_colors.is_empty()) {
    buffer->write_header_scalar_property("uchar", "red");
    buffer->write_header_scalar_property("uchar", "green");
    buffer->write_header_scalar_property("uchar", "blue");
    buffer->write_header_scalar_property("uchar", "alpha");
  }

  if (!plyData->UV_coordinates.is_empty()) {
    buffer->write_header_scalar_property("float", "s");
    buffer->write_header_scalar_property("float", "t");
  }

  if (!plyData->faces.is_empty()) {
    buffer->write_header_element("face", int32_t(plyData->faces.size()));
    buffer->write_header_list_property("uchar", "uint", "vertex_indices");
  }

  if (!plyData->edges.is_empty()) {
    buffer->write_header_element("edge", int32_t(plyData->edges.size()));
    buffer->write_header_scalar_property("int", "vertex1");
    buffer->write_header_scalar_property("int", "vertex2");
  }

  buffer->write_string("end_header");
  buffer->write_to_file();
}

}  // namespace blender::io::ply
