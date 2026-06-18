#!/usr/bin/env bash

set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LD_LIBRARY_PATH="${BASE_DIR}/lib:${LD_LIBRARY_PATH:-}"

run_from_bin() {
    local exe="$1"
    shift || true
    if [[ ! -x "${BASE_DIR}/bin/${exe}" ]]; then
        echo "executable not found: ${BASE_DIR}/bin/${exe}" >&2
        echo "build it first, for example: ./autobuild.sh ${exe}" >&2
        exit 1
    fi
    cd "${BASE_DIR}/bin"
    exec "./${exe}" "$@"
}

mode="${1:-}"
shift || true

case "${mode}" in
    1|realrobot_mini) run_from_bin realrobot_mini "$@" ;;
    2|mujoco_sim_mini) run_from_bin mujoco_sim_mini "$@" ;;
    set_zero|zero) run_from_bin set_zero "$@" ;;
    set_one_zero|one_zero) run_from_bin set_one_zero "$@" ;;
    joy_test|joy) run_from_bin joy_test "$@" ;;
    head_test|head) run_from_bin head_test "$@" ;;
    set_one_Enable|one_Enable) run_from_bin set_one_Enable "$@" ;;
    read_pos) run_from_bin read_pos "$@" ;;
    imu_test|imu) run_from_bin imu_test "$@" ;;
    motor_control|motor) run_from_bin motor_control "$@" ;;
    pd_control|pd) run_from_bin pd_control "$@" ;;
    *)
        echo "usage: ./run.sh realrobot_mini|mujoco_sim_mini|set_zero|set_one_zero|joy_test|head_test|set_one_Enable|read_pos|imu_test|motor_control|pd_control [args...]" >&2
        exit 2
        ;;
esac
