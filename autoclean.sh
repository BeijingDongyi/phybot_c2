#!/usr/bin/env bash
set -euo pipefail
BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Only remove this package's build directory; preserve libraries and prebuilt programs.
rm -rf -- "$BASE_DIR/build"
echo 'build 已清理；封装库和预编译程序已保留。'
