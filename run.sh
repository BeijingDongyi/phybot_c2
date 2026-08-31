#!/usr/bin/env bash
set -Eeuo pipefail
PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LD_LIBRARY_PATH="${PACKAGE_DIR}/lib:${LD_LIBRARY_PATH:-}"
cd "${PACKAGE_DIR}/bin"
if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <executable> [args...]" >&2
    printf 'Available executables:\n'
    find . -maxdepth 1 -type f -executable -printf '  %f\n'
    exit 2
fi
exec "./$@"
