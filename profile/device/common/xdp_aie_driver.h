// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2025 Advanced Micro Devices, Inc. All rights reserved.
//
// Single point for AIE driver includes. Include this header instead of aie_codegen.h or
// xaiengine.h directly. When XDP_USE_AIE_CODEGEN is defined (e.g. via CMake for host/NPU),
// pulls in aie_codegen.h and xaiegbl_params.h. Otherwise (e.g. edge with sysroot), pulls in
// xaiengine.h and xaiengine/xaiegbl_params.h

#ifndef XDP_AIE_DRIVER_H
#define XDP_AIE_DRIVER_H

#ifdef XDP_USE_AIE_CODEGEN
#include <aie_codegen.h>
#include <xaiegbl_params.h>
#else
extern "C" {
#include <xaiengine.h>
#include <xaiengine/xaiegbl_params.h>
}
#endif

#endif /* XDP_AIE_DRIVER_H */
