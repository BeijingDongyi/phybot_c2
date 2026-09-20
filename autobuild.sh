#!/usr/bin/env bash
set -Eeuo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_PLATFORM=ubuntu20.04
DEFAULT_ENVIRONMENT=realrobot_mini
JOYSTICK_TYPE=Model_airplane

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    cat <<USAGE
Usage: ./autobuild.sh [realrobot_mini|mujoco_sim_mini|test] [Hipnuc|IMU]

Default environment: ${DEFAULT_ENVIRONMENT}
Joystick: ${JOYSTICK_TYPE}
Default IMU: Hipnuc (old)
IMU options: Hipnuc (old), IMU (new)
USAGE
    exit 0
fi
if (( $# > 2 )); then
    echo "Too many arguments; use --help" >&2
    exit 2
fi

control_system_enter="${1:-${BUILD_ENVIRONMENT:-}}"
if [[ -z "${control_system_enter}" ]]; then
    echo "================== PHYBOT SOFTWARE =================="
    echo "1. realrobot_mini"
    echo "2. mujoco_sim_mini"
    echo "3. test"
    read -r -p "Select environment [1-3, default 1]: " control_system_enter
    control_system_enter="${control_system_enter:-1}"
fi
case "${control_system_enter}" in
    1|realrobot_mini) control_system_enter=realrobot_mini ;;
    2|mujoco_sim_mini) control_system_enter=mujoco_sim_mini ;;
    3|test) control_system_enter=test ;;
    *)
        echo "Unsupported environment: ${control_system_enter}" >&2
        echo "Use 1-3, realrobot_mini, mujoco_sim_mini, or test." >&2
        exit 2
        ;;
esac

# Keep the same editable old/new default used by the source project. Running
# interactively asks for the final selection; arguments support automation.
IMU_TYPE=Hipnuc  # old IMU
# IMU_TYPE=IMU   # new IMU
selected_imu="${2:-${PHYBOT_IMU_TYPE:-}}"
if [[ -z "${selected_imu}" ]]; then
    echo "1. Hipnuc (old IMU)"
    echo "2. IMU (new IMU)"
    if [[ "${IMU_TYPE}" == IMU ]]; then
        default_imu_selection=2
    else
        default_imu_selection=1
    fi
    read -r -p "Select IMU [1-2, default ${default_imu_selection}]: " selected_imu
    selected_imu="${selected_imu:-${default_imu_selection}}"
fi
case "${selected_imu}" in
    1|Hipnuc) IMU_TYPE=Hipnuc ;;
    2|IMU) IMU_TYPE=IMU ;;
    *)
        echo "Unsupported IMU type: ${selected_imu}" >&2
        echo "Use 1-2, Hipnuc (old IMU), or IMU (new IMU)." >&2
        exit 2
        ;;
esac
case "${IMU_TYPE}" in
    Hipnuc|IMU) ;;
    *)
        echo "Unsupported IMU type: ${IMU_TYPE}" >&2
        echo "Use Hipnuc (old IMU) or IMU (new IMU)." >&2
        exit 2
        ;;
esac

"${PACKAGE_DIR}/check_compatibility.sh" --build
compiler_args=()
if [[ "${TARGET_PLATFORM}" == ubuntu20.04 ]]; then
    compiler_args=(-DCMAKE_C_COMPILER=/usr/bin/gcc-9 -DCMAKE_CXX_COMPILER=/usr/bin/g++-9)
fi

if [[ ! -f "${PACKAGE_DIR}/CMakeLists.txt" ]]; then
    echo "当前目录不包含 CMakeLists.txt，不能编译。" >&2
    echo "请直接运行: ./run.sh <executable> [args...]" >&2
    echo "查看可执行程序: ./run.sh" >&2
    exit 2
fi

export LD_LIBRARY_PATH="${PACKAGE_DIR}/ThirdParty/urdfdom/lib:${PACKAGE_DIR}/ThirdParty/boost/lib:${PACKAGE_DIR}/lib:${LD_LIBRARY_PATH:-}"

build_dir="${PACKAGE_DIR}/build"
mkdir -p "${build_dir}"
echo "${build_dir}"
cmake -S "${PACKAGE_DIR}" -B "${build_dir}" \
    -DWHICH_ENV="${control_system_enter}" \
    -DJOYSTICK_TYPE="${JOYSTICK_TYPE}" \
    -DIMU_TYPE="${IMU_TYPE}" \
    -DCMAKE_BUILD_TYPE=Release "${compiler_args[@]}"
cmake --build "${build_dir}" --parallel "${JOBS:-4}"

case "${control_system_enter}" in
    realrobot_mini|mujoco_sim_mini) expected_executable=main ;;
    test) expected_executable=set_zero ;;
esac
if [[ ! -x "${build_dir}/${expected_executable}" ]]; then
    echo "编译失败：未生成 ${build_dir}/${expected_executable}" >&2
    exit 1
fi
echo "编译完成，可执行文件已生成到: ${build_dir}"
find "${build_dir}" -maxdepth 1 -type f -executable -printf '  %f\n' | sort
