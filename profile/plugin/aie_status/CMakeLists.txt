# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2022-2025 Advanced Micro Devices, Inc.  All rights reserved.
#

# ====================================================================
# This builds the AIE Status plugin.  It is currently built on
# Edge and VE2 platforms that support AIE.
# ====================================================================
# Client: N/A. Edge flow: xaiengine; VE2 handled in its block below.

file(GLOB AIE_STATUS_PLUGIN_FILES
  "${PROFILE_DIR}/plugin/aie_status/*.h"
  "${PROFILE_DIR}/plugin/aie_status/*.cpp"
  "${PROFILE_DIR}/writer/aie_status/*.h"
  "${PROFILE_DIR}/writer/aie_status/*.cpp"
)

if (XDP_VE2_BUILD_CMAKE STREQUAL "yes")
  # VE2: driver lib TBD later
  if (XDP_USE_AIE_CODEGEN)
    set(AIE_DRIVER_LIB aie_codegen)
  else()
    set(AIE_DRIVER_LIB xaiengine)
  endif()
  add_library(xdp_aie_status_plugin SHARED ${AIE_STATUS_PLUGIN_FILES})
  add_dependencies(xdp_aie_status_plugin xdp_core)
  target_link_libraries(xdp_aie_status_plugin PRIVATE xdp_core ${AIE_DRIVER_LIB})
  target_compile_definitions(xdp_aie_status_plugin PRIVATE XDP_VE2_BUILD=1 FAL_LINUX="on")
  target_include_directories(xdp_aie_status_plugin PRIVATE ${CMAKE_SOURCE_DIR}/src)
  if (XDP_USE_AIE_CODEGEN)
    target_compile_definitions(xdp_aie_status_plugin PRIVATE XDP_USE_AIE_CODEGEN=${XDP_USE_AIE_CODEGEN})
  endif()
  set_target_properties(xdp_aie_status_plugin PROPERTIES VERSION ${XRT_VERSION_STRING} SOVERSION ${XRT_SOVERSION})

  install (TARGETS xdp_aie_status_plugin
    RUNTIME DESTINATION ${XDP_PLUGIN_INSTALL_DIR} COMPONENT ${XRT_COMPONENT}
    LIBRARY DESTINATION ${XDP_PLUGIN_INSTALL_DIR} COMPONENT ${XRT_COMPONENT} ${XRT_NAMELINK_SKIP}
  )

elseif (DEFINED XRT_AIE_BUILD AND XRT_EDGE)
  add_library(xdp_aie_status_plugin SHARED ${AIE_STATUS_PLUGIN_FILES})
  add_dependencies(xdp_aie_status_plugin xdp_core)
  target_link_libraries(xdp_aie_status_plugin PRIVATE xdp_core xaiengine)
  if (XDP_USE_AIE_CODEGEN)
    target_compile_definitions(xdp_aie_status_plugin PRIVATE XDP_USE_AIE_CODEGEN=${XDP_USE_AIE_CODEGEN})
  endif()
  target_compile_definitions(xdp_aie_status_plugin PRIVATE FAL_LINUX="on")

  set_target_properties(xdp_aie_status_plugin PROPERTIES VERSION ${XRT_VERSION_STRING} SOVERSION ${XRT_SOVERSION})

  install (TARGETS xdp_aie_status_plugin
    RUNTIME DESTINATION ${XDP_PLUGIN_INSTALL_DIR} COMPONENT ${XRT_COMPONENT}
    LIBRARY DESTINATION ${XDP_PLUGIN_INSTALL_DIR} COMPONENT ${XRT_COMPONENT} ${XRT_NAMELINK_SKIP}
  )

endif()
