#!/usr/bin/env bash
set -Eeuo pipefail
PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"${PACKAGE_DIR}/check_compatibility.sh" --runtime
export LD_LIBRARY_PATH="${PACKAGE_DIR}/lib:${LD_LIBRARY_PATH:-}"
if [[ ! -d "${PACKAGE_DIR}/build" ]]; then
    echo "尚未编译，请先运行: ./autobuild.sh" >&2
    exit 2
fi
cd "${PACKAGE_DIR}/build"
if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <executable> [args...]" >&2
    printf 'Available executables:\n'
    find . -maxdepth 1 -type f -executable -printf '  %f\n'
    exit 2
fi
exec "./$@"
