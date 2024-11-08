/* SPDX-FileCopyrightText: 2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "gpu_interface_info.hh"
#include "gpu_shader_create_info.hh"
GPU_SHADER_INTERFACE_INFO(gpu_node_socket_iface, "")
    .flat(Type::VEC4, "finalColor")
    .flat(Type::VEC4, "finalOutlineColor")
    .flat(Type::FLOAT, "finalDotRadius")
    .flat(Type::FLOAT, "finalOutlineThickness")
    .flat(Type::FLOAT, "AAsize")
    .flat(Type::VEC2, "extrusion")
    .flat(Type::INT, "finalShape")
    .smooth(Type::VEC2, "uv");

/* TODO(Leon): Share with C code. */
/* TODO(Leon): Tweak the instance count to test if there's a noticable sweet spot. */
#define MAX_SOCKET_PARAMETERS 4
#define MAX_SOCKET_INSTANCE 32

GPU_SHADER_CREATE_INFO(gpu_shader_2D_node_socket_shared)
    .define("MAX_PARAM", STRINGIFY(MAX_SOCKET_PARAMETERS))
    .push_constant(Type::MAT4, "ModelViewProjectionMatrix")
    .vertex_out(gpu_node_socket_iface)
    .fragment_out(0, Type::VEC4, "fragColor")
    .vertex_source("gpu_shader_2D_node_socket_vert.glsl")
    .fragment_source("gpu_shader_2D_node_socket_frag.glsl");

GPU_SHADER_CREATE_INFO(gpu_shader_2D_node_socket)
    .do_static_compilation(true)
    /* gl_InstanceID is supposed to be 0 if not drawing instances, but this seems
     * to be violated in some drivers. For example, macOS 10.15.4 and Intel Iris
     * causes #78307 when using gl_InstanceID outside of instance. */
    .define("widgetID", "0")
    .push_constant(Type::VEC4, "parameters", MAX_SOCKET_PARAMETERS)
    .additional_info("gpu_shader_2D_node_socket_shared");

GPU_SHADER_CREATE_INFO(gpu_shader_2D_node_socket_inst)
    .do_static_compilation(true)
    .define("widgetID", "gl_InstanceID")
    .push_constant(Type::VEC4, "parameters", (MAX_SOCKET_PARAMETERS * MAX_SOCKET_INSTANCE))
    .additional_info("gpu_shader_2D_node_socket_shared");
