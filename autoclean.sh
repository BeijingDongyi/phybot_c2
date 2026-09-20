#!/usr/bin/env bash
set -Eeuo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${PACKAGE_DIR}/build"

if [[ -d "${BUILD_DIR}" ]]; then
    find "${BUILD_DIR}" -mindepth 1 -maxdepth 1 -exec rm -rf {} +
    echo "Cleaned: ${BUILD_DIR}"
else
    echo "Build directory does not exist: ${BUILD_DIR}"
fi
