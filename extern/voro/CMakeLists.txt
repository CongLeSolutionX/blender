# SPDX-FileCopyrightText: 2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later
set(INC
	PUBLIC src
)

set(INC_SYS
)

set(SRC
	intern/cell.cc
	intern/common.cc
	intern/container.cc
	intern/unitcell.cc
	intern/v_compute.cc
	intern/c_loops.cc
	intern/v_base.cc
	intern/wall.cc
	intern/pre_container.cc
	intern/container_prd.cc
)

set(LIB
)

blender_add_lib(voro "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
add_library(bf::extern::voro ALIAS voro)