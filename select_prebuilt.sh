#!/usr/bin/env bash
set -euo pipefail
BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
imu="${1:-Hipnuc}"
mode="${2:-realrobot_mini}"
case "$imu" in Hipnuc|IMU) ;; *) echo 'IMU: Hipnuc 或 IMU' >&2; exit 2 ;; esac
case "$mode" in realrobot_mini|mujoco_sim_mini|Robot_test) ;; *) echo '环境无效' >&2; exit 2 ;; esac
source_dir="$BASE_DIR/prebuilt/$imu/$mode"
[[ -d "$source_dir" ]] || { echo "缺少 $source_dir" >&2; exit 1; }
mkdir -p "$BASE_DIR/build"
for name in main set_zero set_one_zero led_test led_off_test led_homing_test imu_test read_pos set_one_Enable; do
    rm -f -- "$BASE_DIR/build/$name"
done
cp -a -- "$source_dir/." "$BASE_DIR/build/"
printf '%s\n' "$mode $imu" > "$BASE_DIR/build/BUILD_SELECTION"
echo "已选择 $mode / $imu；运行目录为 build。"
