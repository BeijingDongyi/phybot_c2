#!/usr/bin/env bash
set -euo pipefail
BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
test -x "$BASE_DIR/build/Joystick_LCM"
echo '使用已封装的 Joystick_LCM。'
