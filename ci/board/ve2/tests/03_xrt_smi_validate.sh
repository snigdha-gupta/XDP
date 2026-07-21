#!/bin/bash
set -euo pipefail

DEVICE="${XRT_VALIDATE_DEVICE:-0000:00:00.0}"

echo "Running: xrt-smi validate --device ${DEVICE}"
xrt-smi validate --device "${DEVICE}"
