// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2025 Advanced Micro Devices, Inc. All rights reserved.
//
// Single point for AIE driver includes. Supports both aie-codegen and aie-rt.
// When building with aie-codegen, define XDP_USE_AIE_CODEGEN (e.g. via CMake).

#ifndef XDP_AIE_DRIVER_H
#define XDP_AIE_DRIVER_H

#ifdef XDP_USE_AIE_CODEGEN
#include <aie_codegen.h>
#include <xaiegbl_params.h>
#else
#include <xaiengine.h>
#include <xaiengine/xaiegbl_params.h>
#endif

#endif /* XDP_AIE_DRIVER_H */
