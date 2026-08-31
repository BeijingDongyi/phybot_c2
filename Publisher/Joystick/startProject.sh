#!/bin/bash
export LD_LIBRARY_PATH=./lcm/release/lib:$LD_LIBRARY_PATH
export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=1

# 定义网卡变量，修改这里即可切换网卡
NIC="wlp0s20f3"

# 执行路由添加命令
# sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev $NIC

# 调试信息
echo ">>>> 程序启动 <<<<" 
# 执行程序
./build/Joystick_LCM

