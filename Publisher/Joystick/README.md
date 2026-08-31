# Joystick Publisher

2026.04.08 -- Version 1.0.0

## 运行流程

1. bash makeProject   编译程序
2. bash startProject  运行程序
3. 需要跨主机通信，在Joystick_Pub.cpp中更改LCM的定义方式(使用IP绑定);
4. "starProject.sh"文件中的NIC更改为自己的网卡名称

2026.06.11 -- Version 1.0.1

1. 如果有新网络增加需要增加键位，在 device/Joystick_int.h 中增加 joystick_mappings 映射表成员即可

