# PHYBOT C2 客户使用说明

适用环境：Ubuntu 20.04 x86_64。

## 编译

```bash
./autobuild.sh realrobot_mini
./autobuild.sh mujoco_sim_mini
./autobuild.sh test
```

执行以上命令后，根据提示选择 IMU：

- `Hipnuc`：旧 IMU
- `IMU`：新 IMU

也可以在命令中直接指定 IMU，避免交互选择：

```bash
./autobuild.sh realrobot_mini Hipnuc
./autobuild.sh realrobot_mini IMU
./autobuild.sh mujoco_sim_mini Hipnuc
./autobuild.sh test Hipnuc
```

切换真机、仿真或测试模式前，建议先清理原有编译结果，再重新编译。

## 运行

真机或 MuJoCo 仿真编译完成后：

```bash
./run.sh main
```

测试模式编译完成后，可运行：

```bash
./run.sh joy_test
./run.sh imu_test
./run.sh read_pos
./run.sh set_zero
./run.sh set_one_zero
./run.sh set_one_Enable
./run.sh head_test
```

查看当前 `build/` 目录中的可执行程序：

```bash
./run.sh
```

所有程序都应通过 `run.sh` 启动。该脚本会自动设置动态库路径，并从正确的工作目录运行程序。

## 清理编译产物

```bash
./autoclean.sh
```

## 首次安装

首次使用时安装编译和图形依赖：

```bash
./install_dependencies.sh
```

安装依赖需要网络连接和管理员权限。该脚本仅适用于 Ubuntu 20.04 x86_64。

如果脚本没有执行权限：

```bash
chmod +x install_dependencies.sh autobuild.sh autoclean.sh run.sh check_compatibility.sh
```

## 软件内容

| 路径 | 作用 |
|---|---|
| `CMakeLists.txt` | CMake 工程入口。 |
| `autobuild.sh` | 编译脚本，支持真机、MuJoCo 仿真和测试模式。 |
| `run.sh` | 运行脚本，自动设置动态库路径并启动程序。 |
| `autoclean.sh` | 清理 `build/` 中的编译产物。 |
| `install_dependencies.sh` | 安装 Ubuntu 20.04 所需系统依赖。 |
| `DataPackage/` | 公共数据接口和控制周期配置。 |
| `device/` | IMU 等设备接口和配置。 |
| `MotorList/` | 电机接口、通信参数和电机配置。 |
| `Model_airplane_Joystick/` | 航模手柄接口和配置。 |
| `StateMachine/` | 状态机定义和管理代码。 |
| `ZeroState/` | 归零状态和参数。 |
| `RL_deploy_cpg/` | 强化学习控制代码、模型和参数。 |
| `RobotStart/` | 真机、仿真和测试程序入口。 |
| `MujocoInterface/` | MuJoCo 仿真接口和配置。 |
| `RobotModel/` | 机器人 URDF、MuJoCo XML 和网格资源。 |
| `ThirdParty/` | 第三方头文件和运行依赖。 |
| `lib/` | Torch、MuJoCo、yaml-cpp、MotorDrive 等运行时动态库。 |

请保留软件的完整目录结构，不要单独移动可执行程序、配置文件、模型文件或动态库。

## 编译目标说明

| 命令 | 说明 |
|---|---|
| `./autobuild.sh realrobot_mini` | 编译 PHYBOT C2 真机程序，生成 `main`。 |
| `./autobuild.sh mujoco_sim_mini` | 编译 MuJoCo 仿真程序，生成 `main`。 |
| `./autobuild.sh test` | 编译归零、通信和硬件测试程序。 |
| `./autobuild.sh <模式> Hipnuc` | 使用旧 IMU 编译指定模式。 |
| `./autobuild.sh <模式> IMU` | 使用新 IMU 编译指定模式。 |

`autobuild.sh` 当前不支持单独编译某一个测试程序。执行 `./autobuild.sh test <IMU类型>` 会编译全部已配置的测试程序。

## 状态机说明

当前状态枚举位于 `StateMachine/include/fsmlist.h`：

```cpp
enum class State {
    IDLE,
    ZERO,
    RL_walk,
};
```

当前已注册的运行状态：

- `State::ZERO`：机器人归零状态。
- `State::RL_walk`：强化学习行走状态。

状态注册位于 `StateMachine/include/statemachinemanager.h`。修改状态枚举、注册关系或切换逻辑后，必须重新编译并完成仿真和真机安全验证。

## 测试程序说明

| 程序 | 作用 |
|---|---|
| `joy_test` | 读取航模手柄数据，用于检查设备连接和输入。 |
| `imu_test` | 读取 IMU 数据，用于检查 `/dev/ttyimu` 和波特率配置。 |
| `read_pos` | 读取并显示电机位置。 |
| `set_zero` | 按当前电机配置执行归零。 |
| `set_one_zero` | 对 `set_one_zero.yaml` 中指定的电机置零。 |
| `set_one_Enable` | 对 `set_one_Enable.yaml` 中指定的电机执行使能测试。 |
| `head_test` | 头部或相关执行器测试。 |

`RobotStart/test/pd_control.cpp`、`bms_test.cpp`、`led_test.cpp` 和 `led_off_test.cpp` 是参考测试源码，但当前 `autobuild.sh test` 不会生成对应的可执行程序。

## YAML 配置说明

| 配置文件 | 作用 |
|---|---|
| `DataPackage/config/datapackage.yaml` | 公共数据维度和控制周期。当前 `control_period` 为 0.002 秒，`actuatedDofNum` 为 21。 |
| `Model_airplane_Joystick/config/joystick.yaml` | 手柄设备、速度范围和速度变化率。默认设备为 `/dev/ttyjoy`。 |
| `MotorList/config/phybot_mini_1.yaml` | 第一块电机主板的网络参数和电机列表。 |
| `MotorList/config/phybot_mini_2.yaml` | 第二块电机主板的网络参数和电机列表。 |
| `MotorList/config/phybot_abszero.yaml` | 电机绝对零点偏置，数据长度应与电机数量一致。 |
| `MotorList/config/set_one_Enable.yaml` | `set_one_Enable` 使用的电机 ID 列表。 |
| `MotorList/config/set_one_zero.yaml` | `set_one_zero` 使用的电机 ID 列表。 |
| `MotorList/config/failed_motors.yaml` | 保存电机使能失败信息，通常由程序写入。 |
| `MotorList/config/phybot1.yaml`、`phybot2.yaml` | 其他机型或历史配置，当前 mini 主流程通常不使用。 |
| `MujocoInterface/config/mujoco_sim.yaml` | MuJoCo 模型路径、关节名称、基座名称和传感器名称。 |
| `RL_deploy_cpg/config/rl_params.yaml` | RL 模型路径、输入维度、PD 增益、动作缩放、力矩限制和默认关节位置。 |
| `ZeroState/config/ZERO.yaml` | 归零位置、控制增益和归零时间。 |
| `device/config/device.yaml` | IMU 串口配置，默认端口为 `/dev/ttyimu`，波特率为 921600。 |
| `RobotModel/*/config/joint_names_*.yaml` | 机器人模型的关节名称映射。 |

修改配置前建议先备份原文件：

```bash
cp -a MotorList/config MotorList/config.backup
cp -a device/config/device.yaml device/config/device.yaml.backup
cp -a Model_airplane_Joystick/config/joystick.yaml Model_airplane_Joystick/config/joystick.yaml.backup
```

## 电机网络配置字段

`phybot_mini_1.yaml` 和 `phybot_mini_2.yaml` 中常见字段如下：

| 字段 | 说明 |
|---|---|
| `Network.LocalIp` | 运行计算机的网卡 IP，需要与实际网卡一致。 |
| `Network.LocalPort` | 运行计算机使用的 UDP 端口。 |
| `Network.DestIp` | 电机主板 IP。 |
| `Network.DestPort` | 电机主板 UDP 端口。 |
| `CommunicationFre` | 通信频率。 |
| `Motor.Id` | 电机 ID。 |
| `Motor.Name` | 电机名称，用于确认程序索引和实际关节的对应关系。 |
| `ControlMode` | 电机控制模式。 |
| `FastMode` | 快速通信模式开关。 |
| `PDKp`、`PDKd` | 电机 PD 参数。 |
| `Direction` | 电机方向，会影响命令和反馈的符号。 |

## 常见修改位置

| 目标 | 修改位置 |
|---|---|
| 修改真机归零姿态 | `ZeroState/config/ZERO.yaml` 中的 `zero_joint_position`。 |
| 修改归零增益或时间 | `ZeroState/config/ZERO.yaml` 中的 `zero_p`、`zero_d` 和 `zero_totalTime`。 |
| 修改手柄设备名 | `Model_airplane_Joystick/config/joystick.yaml` 中的 `dev`。 |
| 修改手柄速度范围 | `Model_airplane_Joystick/config/joystick.yaml` 中的 `maxspeed_*` 和 `minspeed_*`。 |
| 修改电机 IP 或端口 | `MotorList/config/phybot_mini_1.yaml`、`phybot_mini_2.yaml` 中的 `Network`。 |
| 修改单电机置零列表 | `MotorList/config/set_one_zero.yaml` 中的 `set_zero_ID`。 |
| 修改单电机使能列表 | `MotorList/config/set_one_Enable.yaml` 中的 `set_Enable_ID`。 |
| 修改 RL 模型 | `RL_deploy_cpg/config/rl_params.yaml` 中的 `policy_path`。 |
| 修改 IMU 串口 | `device/config/device.yaml` 中的 `port_name` 和 `baud_rate`。 |
| 修改 MuJoCo 模型 | `MujocoInterface/config/mujoco_sim.yaml` 中的 `env_path`。 |

修改电机 ID、方向、关节顺序、归零位置、PD 参数或 RL 网络维度可能导致机器人动作异常。修改后应先进行配置检查和仿真验证，再进行真机测试。

## 电机顺序

当前控制数据按 21 个关节排列：

```text
左腿（由髋部到脚踝）
右腿（由髋部到脚踝）
腰部
左臂（由肩部到肘部）
右臂（由肩部到肘部）
```

具体电机 ID 和名称以 `MotorList/config/phybot_mini_1.yaml`、`phybot_mini_2.yaml` 为准；模型关节顺序以 `MujocoInterface/config/mujoco_sim.yaml` 为准。

## 设备权限

默认设备：

```text
IMU：  /dev/ttyimu
手柄： /dev/ttyjoy
```

检查设备：

```bash
ls -l /dev/ttyimu /dev/ttyjoy
```

如果设备存在但当前用户没有访问权限，可将用户加入串口设备所属用户组。Ubuntu 通常使用 `dialout` 组：

```bash
sudo usermod -aG dialout "$USER"
```

执行后注销并重新登录。如果设备使用其他用户组，以 `ls -l` 显示的实际用户组为准。

## 真机安全检查

启动真机程序前：

1. 将机器人放置在稳定、可支撑的位置，并确认急停开关可用。
2. 清空机器人运动范围，操作人员与机器人保持安全距离。
3. 检查电源、电机、网线、IMU 和手柄连接。
4. 确认编译时选择的 IMU 类型与实际硬件一致。
5. 核对本机网卡地址、电机主板地址和电机配置。
6. 先编译 `test` 模式，运行 `read_pos`、`imu_test` 和 `joy_test` 检查设备状态。
7. 测试完成后重新编译 `realrobot_mini`，确认手柄开关处于安全位置，再运行 `main`。

机器人运动期间不要修改配置或插拔设备。出现异常动作、通信中断或持续报错时，应立即急停并切断动力电源。

## 常见问题

| 问题 | 处理方法 |
|---|---|
| 系统版本不支持 | 使用 `cat /etc/os-release` 和 `uname -m` 确认系统为 Ubuntu 20.04 x86_64。 |
| 缺少编译工具或依赖 | 重新执行 `./install_dependencies.sh`。 |
| 提示尚未编译 | 先执行相应的 `./autobuild.sh` 命令。 |
| 找不到测试程序 | 执行 `./autoclean.sh`，然后运行 `./autobuild.sh test <IMU类型>`。 |
| 找不到配置、模型或动态库 | 保持目录完整，并通过 `./run.sh <程序名>` 启动。 |
| IMU 或手柄无法打开 | 检查设备名、线缆、配置和当前用户的设备访问权限。 |
| 真机通信异常 | 检查急停、电源、网线、本机 IP、主板 IP、端口和电机配置。 |

## 技术支持信息

反馈问题时，请提供：

- `PHYBOT_C2_UBUNTU20_1.0.version` 中的软件版本；
- 操作系统版本和 CPU 架构；
- 编译模式和 IMU 类型；
- 执行的完整命令；
- 终端中的完整错误信息；
- 问题出现前修改过的配置。
