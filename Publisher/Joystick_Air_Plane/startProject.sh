#!/bin/bash
export LD_LIBRARY_PATH=./lcm/release/lib:$LD_LIBRARY_PATH
# export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=1

# 调试信息
echo ">>>> 程序启动 <<<<" 
# 执行程序
./build/Joystick_LCM

