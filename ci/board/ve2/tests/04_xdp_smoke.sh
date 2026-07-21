#!/bin/bash
set -euo pipefail

# Placeholder smoke test until a canonical compile directory is configured.
# Verifies XRT packages are installed after RPM upgrade.
echo "Running: XDP smoke test (package check)"

if rpm -qa | grep -q '^xrt'; then
  echo "xrt RPM package found"
  rpm -qa | grep '^xrt' || true
else
  echo "ERROR: no xrt RPM package found after install"
  exit 1
fi

if [ -d /opt/xilinx/xrt/lib ] || [ -d /usr/lib/xrt ]; then
  echo "XRT library path present on board"
else
  echo "WARNING: expected XRT library path not found (may still be valid for this rootfs)"
fi
