# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2022-2025 Advanced Micro Devices, Inc.  All rights reserved.
#

# ====================================================================
# This builds the AIE Status plugin.  It is currently built on
# Edge and VE2 platforms that support AIE.
# ====================================================================
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

file(GLOB AIE_STATUS_PLUGIN_FILES
  "${PROFILE_DIR}/plugin/aie_status/*.h"
  "${PROFILE_DIR}/plugin/aie_status/*.cpp"
  "${PROFILE_DIR}/writer/aie_status/*.h"
  "${PROFILE_DIR}/writer/aie_status/*.cpp"
)

if (XDP_VE2_BUILD_CMAKE STREQUAL "yes")
  add_library(xdp_aie_status_plugin SHARED ${AIE_STATUS_PLUGIN_FILES})
  add_dependencies(xdp_aie_status_plugin xdp_core)
  target_link_libraries(xdp_aie_status_plugin PRIVATE xdp_core ${AIE_DRIVER_LIB})
  target_compile_definitions(xdp_aie_status_plugin PRIVATE XDP_VE2_BUILD=1 FAL_LINUX="on")
  if (AIE_DRIVER_INCLUDE_DIRS)
    target_include_directories(xdp_aie_status_plugin PRIVATE ${AIE_DRIVER_INCLUDE_DIRS})
  endif()
  target_include_directories(xdp_aie_status_plugin PRIVATE ${CMAKE_SOURCE_DIR}/src)
  if (TARGET aie_codegen OR DEFINED AIE_CODEGEN_DIR)
    target_compile_definitions(xdp_aie_status_plugin PRIVATE XDP_USE_AIE_CODEGEN=1)
  endif()
  if (DEFINED AIE_CODEGEN_DIR)
    target_link_directories(xdp_aie_status_plugin PRIVATE ${AIE_CODEGEN_DIR})
  endif()
  set_target_properties(xdp_aie_status_plugin PROPERTIES VERSION ${XRT_VERSION_STRING} SOVERSION ${XRT_SOVERSION})

  install (TARGETS xdp_aie_status_plugin
    RUNTIME DESTINATION ${XDP_PLUGIN_INSTALL_DIR} COMPONENT ${XRT_COMPONENT}
    LIBRARY DESTINATION ${XDP_PLUGIN_INSTALL_DIR} COMPONENT ${XRT_COMPONENT} ${XRT_NAMELINK_SKIP}
  )

elseif (DEFINED XRT_AIE_BUILD AND XRT_EDGE)
  add_library(xdp_aie_status_plugin SHARED ${AIE_STATUS_PLUGIN_FILES})
  add_dependencies(xdp_aie_status_plugin xdp_core)
  target_link_libraries(xdp_aie_status_plugin PRIVATE xdp_core ${AIE_DRIVER_LIB})
  if (AIE_DRIVER_INCLUDE_DIRS)
    target_include_directories(xdp_aie_status_plugin PRIVATE ${AIE_DRIVER_INCLUDE_DIRS})
  endif()
  if (TARGET aie_codegen OR DEFINED AIE_CODEGEN_DIR)
    target_compile_definitions(xdp_aie_status_plugin PRIVATE XDP_USE_AIE_CODEGEN=1)
  endif()
  if (DEFINED AIE_CODEGEN_DIR)
    target_link_directories(xdp_aie_status_plugin PRIVATE ${AIE_CODEGEN_DIR})
  endif()
  target_compile_definitions(xdp_aie_status_plugin PRIVATE FAL_LINUX="on")

  set_target_properties(xdp_aie_status_plugin PROPERTIES VERSION ${XRT_VERSION_STRING} SOVERSION ${XRT_SOVERSION})

  install (TARGETS xdp_aie_status_plugin
    RUNTIME DESTINATION ${XDP_PLUGIN_INSTALL_DIR} COMPONENT ${XRT_COMPONENT}
    LIBRARY DESTINATION ${XDP_PLUGIN_INSTALL_DIR} COMPONENT ${XRT_COMPONENT} ${XRT_NAMELINK_SKIP}
  )

endif()
