#!/usr/bin/env bash
set -euo pipefail
BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

IMU_TYPE=Hipnuc  #旧IMU
# IMU_TYPE=IMU      #新IMU
IMU_TYPE="${PHYBOT_IMU_TYPE:-$IMU_TYPE}"

selection="${1:-}"
if [[ -z "$selection" ]]; then
    echo '==================PHYBOT SOFTWARE=================='
    echo '1.realrobot_mini'
    echo '2.mujoco_sim_mini'
    echo '3.Robot_test'
    read -r -p 'enter control system number:' selection
fi
case "$selection" in
    1|realrobot_mini) mode=realrobot_mini ;;
    2|mujoco_sim_mini) mode=mujoco_sim_mini ;;
    3|Robot_test) mode=Robot_test ;;
    *) echo '请选择 1、2、3，或对应的环境名称。' >&2; exit 2 ;;
esac
case "$IMU_TYPE" in Hipnuc|IMU) ;; *) echo 'IMU_TYPE 必须为 Hipnuc 或 IMU' >&2; exit 2 ;; esac
if [[ -f "$BASE_DIR/build/CMakeCache.txt" ]]; then
    cached_root="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$BASE_DIR/build/CMakeCache.txt")"
    if [[ -n "$cached_root" && "$cached_root" != "$BASE_DIR" ]]; then
        echo "检测到工程路径已变化，清理旧构建缓存：$cached_root"
        cmake -E remove_directory "$BASE_DIR/build"
    fi
fi
cmake -S "$BASE_DIR" -B "$BASE_DIR/build" \
    -DWHICH_ENV="$mode" -DIMU_TYPE="$IMU_TYPE" -DCMAKE_BUILD_TYPE=Release
# Prevent an executable left by another mode from being started accidentally.
for name in main set_zero set_one_zero led_test led_off_test led_homing_test imu_test read_pos set_one_Enable; do
    rm -f -- "$BASE_DIR/build/$name"
done
cmake --build "$BASE_DIR/build" --parallel "${PHYBOT_JOBS:-4}"
printf '%s\n' "$mode $IMU_TYPE" > "$BASE_DIR/build/BUILD_SELECTION"
