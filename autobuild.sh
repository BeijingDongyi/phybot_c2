#!/usr/bin/env bash

set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BASE_DIR}/build"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 8)}"

print_menu() {
    echo "==================PHYBOT SOFTWARE=================="
    echo "1.realrobot_mini"
    echo "2.mujoco_sim_mini"
    echo "3.test(zero one_zero joy head one_Enable read_pos imu motor_control pd_control)"
    echo "4.motor_control"
    echo "5.pd_control"
}

select_test() {
    local test_name="${1:-}"
    case "${test_name}" in
        ""|0|all) echo "set_zero set_one_zero joy_test head_test set_one_Enable read_pos imu_test motor_control pd_control" ;;
        1|set_zero|zero) echo "set_zero" ;;
        2|set_one_zero|one_zero) echo "set_one_zero" ;;
        3|joy_test|joy) echo "joy_test" ;;
        4|head_test|head) echo "head_test" ;;
        5|set_one_Enable|one_Enable) echo "set_one_Enable" ;;
        6|read_pos) echo "read_pos" ;;
        7|imu_test|imu) echo "imu_test" ;;
        8|motor_control|motor) echo "motor_control" ;;
        9|pd_control|pd) echo "pd_control" ;;
        *)
            echo "!invalid test enter!" >&2
            exit 2
            ;;
    esac
}

mode="${1:-}"
shift || true

if [[ -z "${mode}" ]]; then
    print_menu
    read -r -p "enter control system number:" mode
fi

targets=()
case "${mode}" in
    1|realrobot_mini)
        targets=(realrobot_mini)
        ;;
    2|mujoco_sim_mini)
        targets=(mujoco_sim_mini)
        ;;
    3|test)
        if [[ $# -eq 0 ]]; then
            targets=(set_zero set_one_zero joy_test head_test set_one_Enable read_pos imu_test motor_control pd_control)
        else
            read -r -a targets <<< "$(select_test "$1")"
        fi
        ;;
    4|motor_control|motor)
        targets=(motor_control)
        ;;
    5|pd_control|pd)
        targets=(pd_control)
        ;;
    *)
        echo "!invalid enter!" >&2
        exit 2
        ;;
esac

cmake -S "${BASE_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_DIR}" --target "${targets[@]}" -j "${JOBS}"

echo "Build finished. Executables are in:"
echo "  ${BASE_DIR}/bin"
