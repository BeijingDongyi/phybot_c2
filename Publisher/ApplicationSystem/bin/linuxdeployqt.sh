#!/bin/bash
# 一键打包（彻底解决 xcb 插件依赖，目标机无需安装 libxcb-cursor0）

# ===================== 配置项 =====================
QT_DIR="/usr/local/Qt6.9.3"  # 改为 Qt 根目录，不是 lib 目录
APP_NAME="application_systemd"
QMAKE_PATH="$QT_DIR/bin/qmake"
# ================================================

# 1. 强制清理并创建 lib 目录
if [ -f "./platforms" ]; then
    rm -f "./platforms"
fi
mkdir -p "./platforms"

# 2. 设置 Qt 环境
echo -e "\n===== 1. 设置 Qt 环境 ====="
export LD_LIBRARY_PATH="$QT_DIR/lib:$LD_LIBRARY_PATH"

# 3. 用 linuxdeployqt 打包 Qt 库和平台插件
echo -e "\n===== 2. 打包 Qt 依赖和平台插件 ====="
linuxdeployqt "$APP_NAME" -no-translations


# 4. 打包所有非 Qt 系统库（包括 xcb 插件的依赖）
echo -e "\n===== 3. 打包所有非 Qt 系统库 ====="
ldd ./$APP_NAME | awk '{print $3}' | \
    grep -v "^$QT_DIR" | \
    grep -v "^$" | \
    sort -u | while read lib; do
    if [ -f "$lib" ]; then
        cp -fv "$lib" "./platforms/"
        patchelf --set-rpath '$ORIGIN' "./platforms/$(basename "$lib")"
    fi
done

# 5. 关键修复：打包 xcb 插件自身的依赖
echo -e "\n===== 4. 打包 xcb 插件的依赖 ====="
XCB_PLUGIN="../lib/libQt6XcbQpa.so.6"
if [ -f "$XCB_PLUGIN" ]; then
    ldd "$XCB_PLUGIN" | awk '{print $3}' | \
        grep -v "^$QT_DIR" | \
        grep -v "^$" | \
        sort -u | while read lib; do
        if [ -f "$lib" ]; then
            cp -fv "$lib" "./platforms/"
            patchelf --set-rpath '$ORIGIN' "./platforms/$(basename "$lib")"
        fi
    done
else
    echo "警告：未找到 xcb 插件 $XCB_PLUGIN"
fi

# 6. 修复主程序 RPATH
echo -e "\n===== 5. 修正程序 RPATH ====="
patchelf --set-rpath '$ORIGIN/lib' ./$APP_NAME

# 7. 生成启动脚本（明确指定平台插件路径）
cat > ./start.sh << 'EOF'
#!/bin/bash
export QT_QPA_PLATFORM_PLUGIN_PATH=./platforms
export LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH:$QT_QPA_PLATFORM_PLUGIN_PATH
./BluetoothDemo
EOF
chmod +x ./start.sh

echo -e "\n✅ 打包完成！当前目录: $(pwd)"
echo "启动: ./start.sh"
