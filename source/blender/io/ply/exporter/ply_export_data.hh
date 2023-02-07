/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup ply
 */

#pragma once

#include "ply_data.hh"
#include "ply_file_buffer.hh"

namespace blender::io::ply {

void write_vertices(std::unique_ptr<FileBuffer> &buffer, std::unique_ptr<PlyData> &plyData);

void write_faces(std::unique_ptr<FileBuffer> &buffer, std::unique_ptr<PlyData> &plyData);

void write_edges(std::unique_ptr<FileBuffer> &buffer, std::unique_ptr<PlyData> &plyData);

}  // namespace blender::io::ply
