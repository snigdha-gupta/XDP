#!/bin/bash
# Placeholder for Edge board testing.
#
# Open questions before enabling board-test-edge:
# - Which runner label / board access method (SSH vs zboard)?
# - RPM path from edge build (XRT/build/versal/rpms/) vs target arch on board
# - Canonical test design path on NFS
#
# Expected flow once infra is confirmed:
#   1. Install xrt/zocl RPMs built by build-xrt-edge
#   2. Run xrt-smi or platform-specific smoke tests
#   3. Run XDP validation design

set -euo pipefail

echo "Edge board test not yet configured"
exit 1
