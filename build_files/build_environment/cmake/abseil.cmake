# SPDX-FileCopyrightText: 2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(ABSEIL_EXTRA_ARGS
  -DABSL_BUILD_MONOLITHIC_SHARED_LIBS=OFF
  -DABSL_PROPAGATE_CXX_STD=ON
  -DBUILD_SHARED_LIBS=OFF
)

ExternalProject_Add(external_abseil
  URL file://${PACKAGE_DIR}/${ABSEIL_FILE}
  DOWNLOAD_DIR ${DOWNLOAD_DIR}
  URL_HASH ${ABSEIL_HASH_TYPE}=${ABSEIL_HASH}
  PREFIX ${BUILD_DIR}/abseil

  # Install into ${LIBDIR}/protobuf so that the harvester for Protobuf picks
  # them up, and using them doesn't require any other include path than the
  # Protobuf one.
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${LIBDIR}/protobuf
    ${DEFAULT_CMAKE_FLAGS}
    ${ABSEIL_EXTRA_ARGS}

  INSTALL_DIR ${LIBDIR}/protobuf
)

# TODO: make sure the harvesting skips everything from this package. The header
# files will be harvested with the Protobuf files.
