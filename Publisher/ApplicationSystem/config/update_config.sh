#!/bin/bash

# 工程根目录（基于当前脚本路径向上回溯一级）
PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../" &>/dev/null && pwd)

# config.ini 路径（bin 目录下）
CONFIG_FILE="${PROJECT_ROOT}/bin/config.ini"

# 检查文件是否存在
if [ ! -f "$CONFIG_FILE" ]; then
    echo "❌ 错误：找不到配置文件 $CONFIG_FILE"
    exit 1
fi

echo "============================================="
echo "配置文件路径：$CONFIG_FILE"
echo "============================================="


# 输入 Phybot 工程根路径
echo -e "\n===== 请输入 Phybot 工程根路径 ====="
read -p "请输入路径：" BASE_PATH

EXE_PATH="${BASE_PATH}/build/main"
CONFIG_PATH="${BASE_PATH}/build"

# 替换配置
sed -i "s|^robot_control_exe=.*|robot_control_exe=\"${EXE_PATH}\"|" "$CONFIG_FILE"
sed -i "s|^robot_control_config=.*|robot_control_config=\"${CONFIG_PATH}\"|" "$CONFIG_FILE"


# 选择 joystick_type
echo -e "\n===== 选择手柄类型 ====="
echo "1 - 使用--普通手柄"
echo "2 - 使用--航模手柄"
read -p "请选择 [1/2]: " joy_type

if [ "$joy_type" = "1" ]; then
    sed -i "s|^joystick_type=.*|joystick_type=01|" "$CONFIG_FILE"
    sed -i "s|^joystick_file=.*|joystick_file=\"/dev/input/js0\"|" "$CONFIG_FILE"
elif [ "$joy_type" = "2" ]; then
    sed -i "s|^joystick_type=.*|joystick_type=02|" "$CONFIG_FILE"
    sed -i "s|^joystick_file=.*|joystick_file=\"/dev/ttyjoy\"|" "$CONFIG_FILE"
else
    echo "输入错误，保持默认不变"
fi


echo -e "\n============================================="
echo "✅ 所有配置修改完成！"
echo "📄 配置文件：$CONFIG_FILE"
echo "============================================="
