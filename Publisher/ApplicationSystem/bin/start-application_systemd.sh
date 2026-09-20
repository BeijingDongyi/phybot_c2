#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PACKAGE_ROOT="$(cd -- "$APP_DIR/../.." && pwd)"
export QT_PLUGIN_PATH="$APP_DIR/dev/plugins"
export QT_QPA_PLATFORM_PLUGIN_PATH="$APP_DIR/dev/plugins/platforms"
export LD_LIBRARY_PATH="$APP_DIR/dev/lib:$APP_DIR/dev/runtime:$PACKAGE_ROOT/lcm_server/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
# Refresh only the two deployment paths; preserve device and application settings.
export PHYBOT_PACKAGE_ROOT="$PACKAGE_ROOT"
temp_config="$(mktemp "$SCRIPT_DIR/.config.ini.XXXXXX")"
trap 'rm -f -- "$temp_config"' EXIT
awk '
BEGIN { root = ENVIRON["PHYBOT_PACKAGE_ROOT"]; gsub(/\\/, "\\\\", root); gsub(/"/, "\\\"", root) }
/^robot_control_exe=/ { print "robot_control_exe=\"" root "/build/main\""; next }
/^robot_control_config=/ { print "robot_control_config=\"" root "/build\""; next }
{ print }
' "$SCRIPT_DIR/config.ini" > "$temp_config"
cat "$temp_config" > "$SCRIPT_DIR/config.ini"
rm -f -- "$temp_config"
trap - EXIT
cd -- "$SCRIPT_DIR"
applications=("$SCRIPT_DIR"/application_systemd.*)
if [[ ${#applications[@]} -ne 1 || ! -x ${applications[0]} ]]; then
    echo '错误：未找到 ApplicationSystem 可执行程序。' >&2
    exit 1
fi
exec "${applications[0]}" "$@"
