#!/bin/bash
set -euo pipefail

WORK_DIR="/mnt/test"
RPM_DIR="${WORK_DIR}/rpms"
TEST_DIR="${WORK_DIR}/tests"
SUMMARY="${WORK_DIR}/github_step_summary.md"

echo "################################ XDP VE2 Board Test ################################"
rm -f "${SUMMARY}"
touch "${SUMMARY}"
echo "## XDP VE2 Board Test" >> "${SUMMARY}"
echo "| Step | Status |" >> "${SUMMARY}"
echo "|------|--------|" >> "${SUMMARY}"

log_pass() { echo "| $1 | pass |" >> "${SUMMARY}"; }
log_fail() { echo "| $1 | fail |" >> "${SUMMARY}"; }

ve2_set_env() {
  echo 1 > /sys/module/rcupdate/parameters/rcu_cpu_stall_suppress
  export XRT_AIARM=true
  export XLNX_ENABLE_CACHE=0
  export XRT_ELF_FLOW=1
}

ve2_run_fpgautil() {
  local overlay_dir="${WORK_DIR}/overlay"
  local pdi_path="${overlay_dir}/vpl_gen_fixed_pld.pdi"
  local dtbo_path="${overlay_dir}/pl_aiarm.dtbo"

  if [ ! -d "${overlay_dir}" ]; then
    echo "WARN: ${overlay_dir} not found, skipping fpgautil"
    return 0
  fi

  if [ ! -f "${pdi_path}" ] || [ ! -f "${dtbo_path}" ]; then
    echo "WARN: overlay files not found under ${overlay_dir}, skipping fpgautil"
    return 0
  fi

  echo "Loading FPGA overlay"
  fpgautil -b "${pdi_path}" -o "${dtbo_path}" || echo "WARN: fpgautil returned non-zero (overlay may already be loaded)"
}

install_xrt_rpms() {
  echo "Installing XRT RPMs from ${RPM_DIR}"
  if ! ls "${RPM_DIR}"/xrt*.rpm >/dev/null 2>&1; then
    echo "ERROR: no xrt RPMs found in ${RPM_DIR}"
    log_fail "Install XRT RPMs"
    exit 1
  fi

  if command -v dnf >/dev/null 2>&1; then
    dnf install -y "${RPM_DIR}"/xrt*.rpm
  else
    rpm -Uvh --force "${RPM_DIR}"/xrt*.rpm
  fi
  log_pass "Install XRT RPMs"
}

run_unit_tests() {
  if [ ! -d "${TEST_DIR}" ]; then
    echo "ERROR: test directory not found: ${TEST_DIR}"
    log_fail "Unit tests"
    exit 1
  fi

  ve2_set_env
  ve2_run_fpgautil

  local test_script
  for test_script in "${TEST_DIR}"/*.sh; do
    [ -f "${test_script}" ] || continue
    local name
    name="$(basename "${test_script}")"
    echo "---- Running ${name} ----"
    if bash "${test_script}"; then
      log_pass "${name}"
    else
      log_fail "${name}"
      exit 1
    fi
  done
}

install_xrt_rpms
run_unit_tests

echo "All board tests passed"
cat "${SUMMARY}"
