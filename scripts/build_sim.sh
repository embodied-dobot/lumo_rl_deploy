#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build-sim}"
DEPS_DIR="${LUMO_DEPS_ROOT:-}"

if [[ -z "${DEPS_DIR}" ]]; then
  echo "Set LUMO_DEPS_ROOT to the x86_64 dependency prefix." >&2
  exit 1
fi

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DLUMO_DEPS_ROOT="${DEPS_DIR}" \
  -DLUMO_BUILD_SIM=ON \
  -DLUMO_BUILD_REAL=OFF
cmake --build "${BUILD_DIR}" -j"$(nproc)"
ctest --test-dir "${BUILD_DIR}" --output-on-failure

