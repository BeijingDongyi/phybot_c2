#!/usr/bin/env bash
set -euo pipefail
BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
echo '================== PHYBOT 启动工具 =================='
echo '0 -- 退出程序'
echo '1 -- 启动 Joystick 手柄 + 主程序 main'
echo '2 -- 启动 航模手柄 + 主程序 main'
echo '3 -- 启动 ApplicationSystem 应用'
read -r -p '请输入选择 (0/1/2/3): ' run_mode
case "$run_mode" in
    0) exit 0 ;;
    1|2)
        [[ -x "$BASE_DIR/build/main" ]] || { echo '请先运行 ./autobuild.sh 或 ./select_prebuilt.sh'; exit 1; }
        publisher=Joystick
        [[ "$run_mode" != 2 ]] || publisher=Joystick_Air_Plane
        gnome-terminal --geometry=80x24+1650+100 --tab -- bash -c \
            'cd -- "$1" && ./makeProject.sh && ./startProject.sh; exec bash' \
            phybot "$BASE_DIR/Publisher/$publisher"
        gnome-terminal --geometry=80x24+1650+600 --tab -- bash -c \
            'cd -- "$1" && ./main; exec bash' phybot "$BASE_DIR/build"
        ;;
    3)
        gnome-terminal --geometry=80x24+1650+100 --tab -- bash -c \
            'cd -- "$1" && ./start-application_systemd.sh; exec bash' \
            phybot "$BASE_DIR/Publisher/ApplicationSystem/bin"
        ;;
    *) echo '输入无效' >&2; exit 2 ;;
esac
