#!/usr/bin/env bash
set -Eeuo pipefail
host_release="$(. /etc/os-release; printf '%s:%s' "${ID}" "${VERSION_ID}")"
if [[ "${host_release}" != ubuntu:20.04 || "$(uname -m)" != x86_64 ]]; then
    echo "This dependency installer is for Ubuntu 20.04 x86_64." >&2; exit 2
fi
sudo_command=()
if (( EUID != 0 )); then sudo_command=(sudo); fi
"${sudo_command[@]}" apt-get update
"${sudo_command[@]}" apt-get install -y build-essential gcc-9 g++-9 cmake binutils \
    liburdfdom-dev libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev \
    libxcursor-dev libxxf86vm-dev libxi-dev
