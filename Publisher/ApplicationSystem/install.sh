#!/bin/bash

# 工程根目录
PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)

# 定义脚本路径
UPDATE_CONFIG_SCRIPT="${PROJECT_ROOT}/config/update_config.sh"
UPDATE_SERVICE_SCRIPT="${PROJECT_ROOT}/config/update_service.sh"

# 检查脚本是否存在
check_script_exists() {
    if [ ! -f "$1" ]; then
        echo "❌ 错误：脚本文件 $1 不存在"
        exit 1
    fi
}

# 检查
check_script_exists "${UPDATE_CONFIG_SCRIPT}"
check_script_exists "${UPDATE_SERVICE_SCRIPT}"

# 执行 update_config.sh
bash "${UPDATE_CONFIG_SCRIPT}"
if [ $? -ne 0 ]; then
    echo "❌ update_config.sh 执行失败"
    exit 1
fi

# 执行 update_service.sh
bash "${UPDATE_SERVICE_SCRIPT}"
if [ $? -ne 0 ]; then
    echo "❌ update_service.sh 执行失败"
    exit 1
fi

# 完成提示
echo -e "\n============================================="
echo "🎉 所有配置脚本执行完成！"
echo "📌 工程根目录：${PROJECT_ROOT}"
echo "📌 已更新文件："
echo "   - ${PROJECT_ROOT}/bin/config.ini"
echo "   - ${PROJECT_ROOT}/my_script_with_delay.service"
echo "============================================="

