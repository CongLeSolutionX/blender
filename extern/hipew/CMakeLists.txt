# SPDX-FileCopyrightText: 2021 Blender Foundation
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  .
  include
)

set(INC_SYS

)

set(SRC
  src/hipew.c
  include/hipew.h
  include/hipew6.h
  include/hipew5.h
)

set(LIB
)

if(HIPRT_INCLUDE_DIR)
  list(APPEND INC_SYS
    ${HIPRT_INCLUDE_DIR}
  )

  list(APPEND SRC
    src/hiprtew.cc

    include/hiprtew.h
    src/util.h
  )
endif()

if(HIP_VERSION_MAJOR EQUAL 6)
  add_definitions(-DWITH_HIP_SDK_6)
endif()

blender_add_lib(extern_hipew "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
