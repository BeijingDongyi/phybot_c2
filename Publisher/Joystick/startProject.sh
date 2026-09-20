#!/usr/bin/env bash
set -euo pipefail
BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export LCM_DEFAULT_URL="${LCM_DEFAULT_URL:-udpm://239.255.76.67:7667?ttl=1}"
cd -- "$BASE_DIR"
exec ./build/Joystick_LCM "$@"
