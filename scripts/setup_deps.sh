#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if (( $# != 0 )); then
  echo "Usage: $0" >&2
  exit 1
fi
if [[ "$(uname -m)" != "x86_64" ]]; then
  echo "This project supports dependency setup only on x86_64 Linux." >&2
  exit 1
fi

EIGEN_VERSION="3.4.0"
ONNXRUNTIME_VERSION="1.20.1"
MUJOCO_VERSION="3.2.6"
GLFW_VERSION="3.4"

PREFIX="${ROOT_DIR}/.deps/linux-x64"
WORK_DIR="$(mktemp -d)"
trap 'rm -rf -- "${WORK_DIR}"' EXIT

for command in curl tar cmake; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "Missing command: ${command}" >&2
    echo "Install the Ubuntu 22.04 prerequisites listed in README.zh-CN.md." >&2
    exit 1
  fi
done

mkdir -p "${PREFIX}/include/eigen3" "${PREFIX}/lib"

download() {
  local url="$1"
  local output="$2"
  echo "Downloading ${url}"
  curl --fail --location --retry 3 --retry-all-errors \
    --output "${output}" "${url}"
}

download \
  "https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz" \
  "${WORK_DIR}/eigen.tar.gz"
tar -xzf "${WORK_DIR}/eigen.tar.gz" -C "${WORK_DIR}"
cp -a "${WORK_DIR}/eigen-${EIGEN_VERSION}/Eigen" "${PREFIX}/include/eigen3/"
cp -a "${WORK_DIR}/eigen-${EIGEN_VERSION}/unsupported" "${PREFIX}/include/eigen3/"

ONNXRUNTIME_PACKAGE="onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}"
download \
  "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/${ONNXRUNTIME_PACKAGE}.tgz" \
  "${WORK_DIR}/onnxruntime.tgz"
tar -xzf "${WORK_DIR}/onnxruntime.tgz" -C "${WORK_DIR}"
cp -a "${WORK_DIR}/${ONNXRUNTIME_PACKAGE}/include/." "${PREFIX}/include/"
cp -a "${WORK_DIR}/${ONNXRUNTIME_PACKAGE}/lib/." "${PREFIX}/lib/"

download \
  "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz" \
  "${WORK_DIR}/mujoco.tar.gz"
tar -xzf "${WORK_DIR}/mujoco.tar.gz" -C "${WORK_DIR}"
cp -a "${WORK_DIR}/mujoco-${MUJOCO_VERSION}/include/." "${PREFIX}/include/"
cp -a "${WORK_DIR}/mujoco-${MUJOCO_VERSION}/lib/." "${PREFIX}/lib/"

download \
  "https://github.com/glfw/glfw/archive/refs/tags/${GLFW_VERSION}.tar.gz" \
  "${WORK_DIR}/glfw.tar.gz"
tar -xzf "${WORK_DIR}/glfw.tar.gz" -C "${WORK_DIR}"
cmake -S "${WORK_DIR}/glfw-${GLFW_VERSION}" -B "${WORK_DIR}/glfw-build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DBUILD_SHARED_LIBS=ON \
  -DGLFW_BUILD_EXAMPLES=OFF \
  -DGLFW_BUILD_TESTS=OFF \
  -DGLFW_BUILD_DOCS=OFF \
  -DGLFW_BUILD_WAYLAND=OFF \
  -DGLFW_BUILD_X11=ON
cmake --build "${WORK_DIR}/glfw-build" -j"$(nproc)"
cmake --install "${WORK_DIR}/glfw-build"

echo
echo "Dependencies are ready: ${PREFIX}"
echo "Run: export LUMO_DEPS_ROOT='${PREFIX}'"
