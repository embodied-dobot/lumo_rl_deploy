#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build-real}"
DEPS_DIR="${LUMO_DEPS_ROOT:-}"
SDK_DIR="${LUMO_SDK_ROOT:-}"

if [[ "$(uname -m)" != "x86_64" ]]; then
  echo "The real-robot program must be built on an x86_64 Linux computer." >&2
  exit 1
fi
if [[ -z "${DEPS_DIR}" || ! -d "${DEPS_DIR}/include" || ! -d "${DEPS_DIR}/lib" ]]; then
  echo "Set LUMO_DEPS_ROOT to the x86_64 dependency prefix." >&2
  exit 1
fi
if [[ -z "${SDK_DIR}" || ! -f "${SDK_DIR}/include/dobot/sdk/session.hpp" ]]; then
  echo "Set LUMO_SDK_ROOT to the public dobot-sdk repository." >&2
  exit 1
fi
cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DLUMO_DEPS_ROOT="${DEPS_DIR}" \
  -DLUMO_SDK_ROOT="${SDK_DIR}" \
  -DLUMO_BUILD_SIM=OFF \
  -DLUMO_BUILD_REAL=ON
cmake --build "${BUILD_DIR}" --target lumo_real -j"$(nproc)"
"${BUILD_DIR}/lumo_real" --check
echo "Build complete. Start with scripts/run_real.sh [--model sr_3588|sr_thro] [--robot-ip ROBOT_IP]."
