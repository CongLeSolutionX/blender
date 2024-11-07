# SPDX-FileCopyrightText: 2018-2022 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

# Build as a shared library so that the dependencies (Abseil and utf8_range)
# get all nicely packed into that one shared lib. Especially important for
# Abseil, as that's around ~80 little static libraries if we go that route.
#
# Note that the utf8_range dependency is bundled with the Protobuf sources
# themselves, and doesn't require anything extra on our side.

set(PROTOBUF_EXTRA_ARGS
  -DABSL_PROPAGATE_CXX_STD=ON
  -Dabsl_DIR=${LIBDIR}/abseil/lib/cmake/absl
  -Dprotobuf_ABSL_PROVIDER=package
  -Dprotobuf_BUILD_SHARED_LIBS=ON
  -Dprotobuf_BUILD_TESTS=OFF
)

ExternalProject_Add(external_protobuf
  URL file://${PACKAGE_DIR}/${PROTOBUF_FILE}
  DOWNLOAD_DIR ${DOWNLOAD_DIR}
  URL_HASH ${PROTOBUF_HASH_TYPE}=${PROTOBUF_HASH}
  PREFIX ${BUILD_DIR}/protobuf

  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${LIBDIR}/protobuf
    ${DEFAULT_CMAKE_FLAGS}
    ${PROTOBUF_EXTRA_ARGS}

  INSTALL_DIR ${LIBDIR}/protobuf
)

add_dependencies(
  external_protobuf
  external_abseil
)

# TODO: make sure the harvesting skips the static .a libraries. They are not
# necessary for Blender to build. The rest (including `protobuf/bin` and
# `protobuf/include` is necessary though.
