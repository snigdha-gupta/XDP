# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2024 Advanced Micro Devices, Inc.  All rights reserved.
#

# ========================================================================
# This builds the AIE PC Plugin which can configure Perf Counter registers
# to count cycles between core function start and end.
# It is currently built on Client Windows Only.
# =========================================================================
# aie-codegen only for host/NPU; edge uses sysroot xaiengine (no extra includes)
if (TARGET aie_codegen)
  set(AIE_DRIVER_LIB aie_codegen)
  set(AIE_DRIVER_INCLUDE_DIRS)
elseif (DEFINED AIE_CODEGEN_DIR)
  set(AIE_DRIVER_LIB aie_codegen)
  set(AIE_DRIVER_INCLUDE_DIRS ${AIE_CODEGEN_DIR}/include ${AIE_CODEGEN_DIR}/include/aie_codegen_inc ${AIE_CODEGEN_DIR}/../aie-regdb/globalparams)
else()
  set(AIE_DRIVER_LIB xaiengine)
  set(AIE_DRIVER_INCLUDE_DIRS)
endif()

if (XDP_CLIENT_BUILD_CMAKE STREQUAL "yes")
  set(IMPL_DIR "${PROFILE_DIR}/plugin/aie_pc/clientDev")
endif()


file(GLOB XDP_AIE_PC_PLUGIN_FILES
  "${PROFILE_DIR}/plugin/aie_pc/*.h"
  "${PROFILE_DIR}/plugin/aie_pc/*.cpp"
  "${IMPL_DIR}/*.h"
  "${IMPL_DIR}/*.cpp"
)

file(GLOB XDP_DEVICE_COMMON_FILES
  "${PROFILE_DIR}/device/common/*.h"
  "${PROFILE_DIR}/device/common/*.cpp"
)

if (XDP_CLIENT_BUILD_CMAKE STREQUAL "yes")
  xrt_configure_version_file(xdp_aie_pc_plugin SHARED)
  add_library(xdp_aie_pc_plugin SHARED xdp_aie_pc_plugin-version.rc
    ${XDP_AIE_PC_PLUGIN_FILES} ${XDP_DEVICE_COMMON_FILES})
  add_dependencies(xdp_aie_pc_plugin xdp_core xrt_coreutil)
  target_link_libraries(xdp_aie_pc_plugin PRIVATE xdp_core xrt_coreutil ${AIE_DRIVER_LIB})
  target_compile_definitions(xdp_aie_pc_plugin PRIVATE XDP_CLIENT_BUILD=1 -DXAIE_FEATURE_MSVC)
  if (AIE_DRIVER_INCLUDE_DIRS)
    target_include_directories(xdp_aie_pc_plugin PRIVATE ${AIE_DRIVER_INCLUDE_DIRS})
  endif()
  if (TARGET aie_codegen OR DEFINED AIE_CODEGEN_DIR)
    target_compile_definitions(xdp_aie_pc_plugin PRIVATE XDP_USE_AIE_CODEGEN=1)
  endif()
  if (DEFINED AIE_CODEGEN_DIR)
    target_link_directories(xdp_aie_pc_plugin PRIVATE ${AIE_CODEGEN_DIR})
  endif()
  set_target_properties(xdp_aie_pc_plugin PROPERTIES VERSION ${XRT_VERSION_STRING} SOVERSION ${XRT_SOVERSION})

  install (TARGETS xdp_aie_pc_plugin
    RUNTIME DESTINATION ${XDP_PLUGIN_INSTALL_DIR} COMPONENT ${XRT_COMPONENT}
    LIBRARY DESTINATION ${XDP_PLUGIN_INSTALL_DIR} COMPONENT ${XRT_COMPONENT} ${XRT_NAMELINK_SKIP}
  )

# Else, on edge-aarch64 don't build at all

endif()
