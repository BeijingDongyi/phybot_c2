#!/usr/bin/env bash
set -Eeuo pipefail
export LC_ALL=C
PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_PLATFORM=ubuntu20.04
mode="${1:---build}"
case "${mode}" in --abi-only|--build|--runtime) ;; *) echo "Usage: $0 [--abi-only|--build|--runtime]" >&2; exit 2 ;; esac
if [[ "${TARGET_PLATFORM}" == native ]]; then
    echo "Host-native SDK: no Ubuntu 20.04 compatibility guarantee."
    exit 0
fi
version_gt() {
    [[ "$1" != "$2" && "$(printf '%s\n%s\n' "$1" "$2" | sort -V | tail -n 1)" == "$1" ]]
}
if [[ "${mode}" != --abi-only ]]; then
    if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
        echo "This SDK requires Linux x86_64." >&2; exit 2
    fi
fi
if [[ "${mode}" == --runtime ]]; then exit 0; fi
if [[ "${mode}" == --build ]]; then
    host_release="$(. /etc/os-release; printf '%s:%s' "${ID}" "${VERSION_ID}")"
    if [[ "${host_release}" != ubuntu:20.04 ]]; then
        echo "Rebuild this target on Ubuntu 20.04, or regenerate with TARGET_PLATFORM=native." >&2
        exit 2
    fi
    for tool in cmake gcc-9 g++-9 make readelf; do
        command -v "${tool}" >/dev/null || { echo "Missing ${tool}; run ./install_dependencies.sh" >&2; exit 127; }
    done
    cmake_version="$(cmake --version | awk 'NR == 1 {print $3}')"
    if version_gt 3.16 "${cmake_version}"; then
        echo "CMake 3.16 or newer is required." >&2; exit 2
    fi
fi
command -v readelf >/dev/null || { echo "Install binutils to check ELF compatibility." >&2; exit 127; }
declare -A checked=()
failures=0
count=0
while IFS= read -r -d '' library; do
    resolved="$(readlink -f -- "${library}")" || resolved="${library}"
    [[ -z "${checked["${resolved}"]:-}" ]] || continue
    checked["${resolved}"]=1
    if ! header="$(readelf -Wh "${library}" 2>/dev/null)"; then
        echo "Invalid or broken shared library: ${library#"${PACKAGE_DIR}/"}" >&2
        failures=$((failures + 1)); continue
    fi
    if [[ "${header}" != *'ELF64'* || "${header}" != *'Advanced Micro Devices X86-64'* ]]; then
        echo "Not an x86_64 ELF library: ${library#"${PACKAGE_DIR}/"}" >&2
        failures=$((failures + 1)); continue
    fi
    versions="$(readelf -W --version-info "${library}" | awk '
        /Version needs section/ {needs=1; next}
        /^Version .* section/ {needs=0}
        needs {for (i=1; i<NF; i++) if ($i == "Name:") print $(i+1)}')"
    while IFS= read -r symbol; do
        case "${symbol}" in
            GLIBC_*) limit=2.31; required="${symbol#GLIBC_}" ;;
            GLIBCXX_*) limit=3.4.28; required="${symbol#GLIBCXX_}" ;;
            CXXABI_[0-9]*) limit=1.3.12; required="${symbol#CXXABI_}" ;;
            *) continue ;;
        esac
        if [[ ! "${required}" =~ ^[0-9]+(\.[0-9]+)*$ ]] || version_gt "${required}" "${limit}"; then
            echo "Ubuntu 20.04 ABI mismatch: ${library#"${PACKAGE_DIR}/"} requires ${symbol} (limit ${limit})" >&2
            failures=$((failures + 1))
        fi
    done <<< "${versions}"
    count=$((count + 1))
done < <(find "${PACKAGE_DIR}" \
    -type d \( -name build -o -name 'build-*' -o -name .git \) -prune -o \
    \( -type f -o -type l \) \( -name '*.so' -o -name '*.so.*' \) -print0)
if (( failures )); then
    echo "Replace/rebuild the reported libraries on Ubuntu 20.04; changing C++ flags cannot downgrade glibc requirements." >&2
    exit 1
fi
echo "Ubuntu 20.04 x86_64 ELF ABI check passed (${count} libraries)."
echo "This checks symbol-version requirements; final linking, system dependencies and hardware still require target testing."
