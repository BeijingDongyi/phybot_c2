#!/bin/bash

# 工程根目录（基于当前脚本路径向上回溯一级）
PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../" &>/dev/null && pwd)
echo "🔍 当前工程根目录是: ${PROJECT_ROOT}"

# service 文件位置（工程根目录）
SERVICE_FILE="${PROJECT_ROOT}/my_script_with_delay.service"

# 启动脚本路径
START_SCRIPT="${PROJECT_ROOT}/bin/start-application_systemd.sh"

# 替换 ExecStart= 后面的路径
sed -i "s|^ExecStart=.*|ExecStart=${START_SCRIPT}|" "${SERVICE_FILE}"

# 完成提示
echo "✅ 路径修改完成！"
echo "📄 已修改为: ExecStart=${START_SCRIPT}"


