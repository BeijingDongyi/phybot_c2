# Phybot C2 软件交付包使用说明

适用平台：Ubuntu 20.04 x86_64。

## 快速使用

### 编译

旧 IMU（Hipnuc，默认）：

```bash
./autobuild.sh realrobot_mini
./autobuild.sh mujoco_sim_mini
./autobuild.sh Robot_test
```

新 IMU：

```bash
PHYBOT_IMU_TYPE=IMU ./autobuild.sh realrobot_mini
PHYBOT_IMU_TYPE=IMU ./autobuild.sh mujoco_sim_mini
PHYBOT_IMU_TYPE=IMU ./autobuild.sh Robot_test
```

### 运行

真机程序和 MuJoCo 仿真程序统一通过启动菜单运行：

```bash
./autorun.sh
```

设备测试程序从 `build/` 目录直接运行：

```bash
cd build
./imu_test
./read_pos
./set_one_Enable
./set_one_zero
```

`set_one_Enable` 和 `set_one_zero` 会改变电机状态，仅限专业人员在确保机器人安全的情况下使用。

### 清理编译产物

```bash
./autoclean.sh
```

该命令只删除 `build/`，不会删除封装库和 `prebuilt/` 中的预编译程序。

## 交付内容

| 路径 | 作用 |
| --- | --- |
| `CMakeLists.txt` | 客户侧 CMake 工程入口。 |
| `autobuild.sh` | 编译脚本，支持真机、MuJoCo 和设备测试目标。 |
| `autorun.sh` | 启动脚本，用于启动手柄、主程序或 ApplicationSystem。 |
| `autoclean.sh` | 清理脚本，用于删除 `build/` 编译产物。 |
| `DataPackage/` | 数据接口头文件和全局数据配置。 |
| `device/` | IMU 设备接口、配置及相关源码。 |
| `MotorList/` | 电机接口头文件、网络与电机配置、底层封装库。 |
| `Joystick/` | 手柄接口头文件和配置。 |
| `StateMachine/` | 状态机源码、状态枚举和注册入口。 |
| `ZeroState/` | 归零状态源码和参数。 |
| `RL_deploy_*/` | 各动作的部署接口、模型和参数。 |
| `RobotStart/` | 真机、MuJoCo 和设备测试程序入口。 |
| `MujocoInterface/` | MuJoCo 仿真接口头文件和配置。 |
| `RobotModel/` | 机器人模型及仿真资源。 |
| `Publisher/` | Joystick、航模手柄和 ApplicationSystem 程序。 |
| `ThirdParty/` | 第三方头文件和运行依赖。 |
| `lib/` | 按 IMU 类型提供的封装静态库。 |
| `prebuilt/` | 新旧 IMU 对应的预编译程序。 |
| `compatibility/` | Ubuntu 20.04 兼容性检查记录。 |

## 编译环境

安装编译工具和运行依赖：

```bash
sudo apt update
sudo apt install build-essential cmake libglib2.0-0
```

如果脚本没有执行权限，可执行：

```bash
chmod +x autobuild.sh autorun.sh autoclean.sh
```

请在软件包根目录执行所有编译命令，并保持目录结构完整。

## 编译目标说明

| 命令 | 说明 | 主要输出 |
| --- | --- | --- |
| `./autobuild.sh realrobot_mini` | 编译旧 IMU 真机程序。 | `main`、`set_zero`、`set_one_zero` 和 LED 测试程序 |
| `./autobuild.sh mujoco_sim_mini` | 编译旧 IMU MuJoCo 仿真程序。 | `main` |
| `./autobuild.sh Robot_test` | 编译旧 IMU 设备测试程序。 | `imu_test`、`read_pos`、`set_one_Enable`、`set_one_zero` |
| `PHYBOT_IMU_TYPE=IMU ./autobuild.sh realrobot_mini` | 编译新 IMU 真机程序。 | 与旧 IMU 真机目标相同 |
| `PHYBOT_IMU_TYPE=IMU ./autobuild.sh mujoco_sim_mini` | 编译新 IMU MuJoCo 仿真程序。 | `main` |
| `PHYBOT_IMU_TYPE=IMU ./autobuild.sh Robot_test` | 编译新 IMU 设备测试程序。 | 与旧 IMU 测试目标相同 |

`Hipnuc` 表示旧 IMU，`IMU` 表示新 IMU。未设置 `PHYBOT_IMU_TYPE` 时，默认编译旧 IMU 版本。

目标名称区分大小写，其中设备测试目标必须写为 `Robot_test`。也可以使用数字参数：

```bash
./autobuild.sh 1  # realrobot_mini
./autobuild.sh 2  # mujoco_sim_mini
./autobuild.sh 3  # Robot_test
```

编译结果保存在 `build/`，当前编译目标和 IMU 类型记录在：

```bash
cat build/BUILD_SELECTION
```

默认使用 4 个并行任务。如需调整，可设置 `PHYBOT_JOBS`：

```bash
PHYBOT_JOBS=8 ./autobuild.sh realrobot_mini
```

## 启动方式

完成 `realrobot_mini` 或 `mujoco_sim_mini` 编译后运行：

```bash
./autorun.sh
```

启动菜单说明：

| 选项 | 功能 |
| --- | --- |
| `0` | 退出。 |
| `1` | 启动 Joystick 手柄程序和 `build/main`。 |
| `2` | 启动航模手柄程序和 `build/main`。 |
| `3` | 启动 ApplicationSystem。 |

选项 `1` 和 `2` 会分别打开控制器终端和主程序终端。停止程序时，在各终端按 `Ctrl+C`。

`Robot_test` 不生成 `build/main`，因此测试目标不能通过启动菜单的选项 `1` 或 `2` 运行。

## 状态机和手柄

状态枚举位于 `StateMachine/include/fsmlist.h`，状态注册入口位于 `StateMachine/include/statemachinemanager.h`。

当前注册的主要状态包括：

| 状态 | 功能 |
| --- | --- |
| `State::ZERO` | 零位状态 |
| `State::RL_walk` | 行走 |
| `State::RL_climb_up` | 爬起 |
| `State::RL_crane_down` | 趴下 |
| `State::RL_mimic` | 招手 |
| `State::RL_long_motion` | 连续动作 |
| `State::RL_forward_punch` | 打拳 |
| `State::RL_forward_kick` | 踢腿 |
| `State::RL_taichi` | 太极 |
| `State::RL_kongfu` | 功夫 |
| `State::RL_introduct` | 自我介绍 |
| `State::RL_dance` | 舞蹈 |
| `State::RL_ASE_*` | ASE 组合拳、侧踢、上勾拳和站立动作 |

手柄后端在 `Joystick/config/joystick_source.yaml` 中选择：

```yaml
joystick_backend: airplane
# joystick_backend: xbox
```

航模手柄设备和速度范围在 `Joystick/config/airplane_joystick.yaml` 中配置，默认串口为 `/dev/ttyjoy`。

## 测试程序说明

先编译测试目标：

```bash
./autobuild.sh Robot_test
```

如果使用新 IMU：

```bash
PHYBOT_IMU_TYPE=IMU ./autobuild.sh Robot_test
```

测试程序说明：

| 程序 | 作用 |
| --- | --- |
| `imu_test` | 读取 IMU 数据，用于检查 `/dev/ttyimu` 和波特率配置。 |
| `read_pos` | 读取电机位置。 |
| `set_one_Enable` | 对 `set_one_Enable.yaml` 中指定的电机执行使能测试。 |
| `set_one_zero` | 对 `set_one_zero.yaml` 中指定的电机设置零位。 |

真机构建还会生成以下维护程序：

| 程序 | 作用 |
| --- | --- |
| `set_zero` | 按真机配置执行归零。 |
| `set_one_zero` | 对指定电机设置零位。 |
| `led_test` | LED 测试。 |
| `led_off_test` | LED 关闭测试。 |
| `led_homing_test` | LED 归位流程测试。 |

> **警告：使能、归零和电机相关测试可能造成机器人运动。执行前必须稳定支撑机器人，清空周围区域，并确保急停可用。**

## YAML 配置说明

| 配置文件 | 作用 |
| --- | --- |
| `DataPackage/config/datapackage.yaml` | 全局数据维度和控制周期；当前控制周期为 0.002 秒，执行器自由度为 21。 |
| `Joystick/config/joystick_source.yaml` | 选择手柄后端，支持 `airplane` 和 `xbox`。 |
| `Joystick/config/airplane_joystick.yaml` | 航模手柄串口、加速度和速度范围。 |
| `MotorList/config/phybot_mini_1.yaml` | 第一块电机主板的网络和电机配置。 |
| `MotorList/config/phybot_mini_2.yaml` | 第二块电机主板的网络和电机配置。 |
| `MotorList/config/phybot_abszero.yaml` | 电机绝对零点偏置。 |
| `MotorList/config/set_one_Enable.yaml` | 单电机使能测试使用的电机列表。 |
| `MotorList/config/set_one_zero.yaml` | 单电机置零测试使用的电机列表。 |
| `MotorList/config/failed_motors.yaml` | 保存电机使能失败信息。 |
| `MujocoInterface/config/mujoco_sim.yaml` | MuJoCo 模型路径、关节名、基座名和传感器名。 |
| `RL_deploy_cpg/config/rl_params.yaml` | 行走策略模型路径、网络维度、PD 增益、动作缩放和力矩限制。 |
| `ZeroState/config/ZERO.yaml` | 站立或趴下零位、控制增益和归零时间。 |
| `device/config/device.yaml` | IMU 串口和波特率，默认 `/dev/ttyimu`、921600。 |
| `RobotModel/phybot_c2/config/` | 机器人模型关节名映射。 |

`phybot_mini_1.yaml` 和 `phybot_mini_2.yaml` 中的常用字段：

| 字段 | 说明 |
| --- | --- |
| `Network.LocalIp` | 本机网卡 IP，需要与实际网卡一致。 |
| `Network.LocalPort` | 本机 UDP 端口。 |
| `Network.DestIp` | 电机主板 IP。 |
| `Network.DestPort` | 电机主板 UDP 端口。 |
| `CommunicationFre` | 通信频率。 |
| `Motor.Id` | 电机 ID。 |
| `Motor.Name` | 电机名称。 |
| `ControlMode` | 电机控制模式。 |
| `FastMode` | 快速通信模式开关。 |
| `PDKp`、`PDKd` | 电机 PD 参数。 |
| `Direction` | 电机方向，影响命令和反馈符号。 |

## 常见修改位置

| 目标 | 修改位置 |
| --- | --- |
| 修改 IMU 串口或波特率 | `device/config/device.yaml` |
| 修改航模手柄设备名或速度范围 | `Joystick/config/airplane_joystick.yaml` |
| 切换手柄后端 | `Joystick/config/joystick_source.yaml` |
| 修改电机 IP、端口或电机参数 | `MotorList/config/phybot_mini_1.yaml`、`phybot_mini_2.yaml` |
| 修改单电机使能目标 | `MotorList/config/set_one_Enable.yaml` |
| 修改单电机置零目标 | `MotorList/config/set_one_zero.yaml` |
| 修改真机归零姿态和增益 | `ZeroState/config/ZERO.yaml` |
| 修改行走模型或控制参数 | `RL_deploy_cpg/config/rl_params.yaml` |
| 修改 MuJoCo 模型和传感器配置 | `MujocoInterface/config/mujoco_sim.yaml` |
| 查看或增加状态枚举 | `StateMachine/include/fsmlist.h` |
| 注册状态实现 | `StateMachine/include/statemachinemanager.h` |

修改源码后，重新运行对应的 `autobuild.sh` 命令即可。

## 关节顺序

控制向量和模型配置中的 21 个关节按以下顺序排列：

| 序号 | 关节 |
| --- | --- |
| 1–6 | 左腿：髋关节 Pitch、髋关节 Roll、髋关节 Yaw、膝关节、踝关节 Pitch、踝关节 Roll |
| 7–12 | 右腿：髋关节 Pitch、髋关节 Roll、髋关节 Yaw、膝关节、踝关节 Pitch、踝关节 Roll |
| 13 | 腰部 Yaw |
| 14–17 | 左臂：肩关节 Pitch、肩关节 Roll、肩关节 Yaw、肘关节 Pitch |
| 18–21 | 右臂：肩关节 Pitch、肩关节 Roll、肩关节 Yaw、肘关节 Pitch |

修改关节位置、PD 参数或动作参数时，必须保持数组顺序和长度一致。

## 真机运行前检查

1. 确认编译时选择的 IMU 类型与实际硬件一致。
2. 检查 `device/config/device.yaml` 中的串口和波特率。
3. 确认当前用户对 `/dev/ttyimu` 具有读写权限。
4. 检查两块电机主板的本机 IP、目标 IP 和端口配置。
5. 确认供电、网络、CAN 设备和控制手柄连接正常。
6. 稳定支撑机器人，清空周围区域，并确认急停开关可用。

串口检查命令：

```bash
ls -l /dev/ttyimu
test -r /dev/ttyimu && test -w /dev/ttyimu && echo "IMU 串口权限正常"
```

## 注意事项

- `autorun.sh` 使用 GNOME Terminal，MuJoCo 需要桌面图形环境和 OpenGL 支持。
- 不要将 `build/` 中的程序单独复制到其他位置，程序依赖包内的 `ThirdParty/`、`MotorList/lib/` 和 `lcm_server/lib/`。
- 切换 IMU 类型或运行目标时，应重新执行对应的 `autobuild.sh` 命令。
- 如果最后一次编译的是 `Robot_test`，`build/` 中不会生成 `main`。
- Gazebo 依赖独立的 ROS/catkin 工程，本交付包不包含可独立运行的 Gazebo 环境。
- 原项目未提供 Webots 构建实现。

## 兼容性说明

- 已完成新旧 IMU 与真机、MuJoCo、设备测试共 6 种组合的编译检查。
- 已完成交付程序的动态库加载、重定位和 Ubuntu 20.04 ABI 兼容性检查。
- 已完成有效 TorchScript 模型的 CPU 加载检查。
- 兼容性验证未连接真机硬件，也未执行电机动作。

详细验收记录位于 `compatibility/`。
