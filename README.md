# Phybot C2 封装交付包

目标平台：Ubuntu 20.04 x86_64。交付包已包含运行所需的模型、配置、动态库、封装静态库和默认程序。


## 使用方法

默认 `build/` 已选择旧 IMU（Hipnuc）真机程序，可直接执行：

```bash
./autorun.sh
```

切换预编译程序，不重新构建：

```bash
./select_prebuilt.sh Hipnuc realrobot_mini  # 旧 IMU
./select_prebuilt.sh IMU Robot_test         # 新 IMU
./select_prebuilt.sh IMU mujoco_sim_mini    # 新 IMU MuJoCo
```

如需修改允许公开的源码后重新构建，编辑 `autobuild.sh` 的 `IMU_TYPE`：

```bash
IMU_TYPE=Hipnuc  # 旧 IMU
# IMU_TYPE=IMU   # 新 IMU
./autobuild.sh 1 # 真机
./autobuild.sh 2 # MuJoCo
./autobuild.sh 3 # Robot_test
```

`autoclean.sh` 只会清理本包的 `build/`，不会删除封装库和 `prebuilt/`。

Ubuntu 20.04 重新编译需要 `build-essential` 和 `cmake`；运行时使用系统标准
`libglib2.0-0`。交付的预编译程序无需安装开发包。

真机运行前，请确认 `device/config/device.yaml`、`MotorList/config/`、IMU 串口权限、CAN 设备和急停状态。`autorun.sh` 使用 GNOME Terminal，MuJoCo 需要桌面图形环境。Gazebo 依赖独立的 ROS/catkin 工程，原项目未包含可独立封装的 Gazebo 运行环境；Webots 原项目未提供构建实现。

## 已验证

- Ubuntu 20.04 用户空间：GCC 9.4、glibc 2.31、CMake 3.16.3；
- 新旧 IMU × 真机、MuJoCo、Robot_test 共 6 种编译组合；
- 31 个交付程序动态库加载与重定位；
- 81 个有效 TorchScript 模型 CPU 加载；
- 104 个 ELF 文件的 GLIBC、GLIBCXX、CXXABI 版本不超出 Ubuntu 20.04 基线。

详细日志和机器可读报告位于 `compatibility/`。验证没有连接真机硬件，也没有执行电机动作。
