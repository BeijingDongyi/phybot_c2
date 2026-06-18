# Phybot C2 

本文档说明 Phybot C2 客户开发包的目录结构、构建运行方式、主要配置项，以及状态机、手柄、电机控制和 RL 部署相关的二次开发入口。

## 目录

- [适用范围](#适用范围)
- [快速开始](#快速开始)
- [构建目标](#构建目标)
- [运行目标](#运行目标)
- [目录结构](#目录结构)
- [状态机与手柄](#状态机与手柄)
- [测试程序](#测试程序)
- [配置文件](#配置文件)
- [常见修改位置](#常见修改位置)
- [安全注意事项](#安全注意事项)

## 适用范围

本开发包用于 Phybot C2/mini 机器人客户侧开发，包含：

- 客户可修改的状态机、归零状态、RL_walk、测试程序和配置文件。
- 真机、MuJoCo 仿真和测试程序的 CMake 构建入口。
- 闭源预编译库、目标文件、运行时动态库和模型资源。


如果仓库是从 GitHub 克隆得到的，请先执行：

```bash
git lfs install
git lfs pull
```

否则 `prebuilt/`、`lib/` 和模型文件可能只是 Git LFS 指针文件，编译时会出现 `file format not recognized` 等错误。

## 快速开始

构建真机程序：

```bash
./autobuild.sh realrobot_mini
```

运行真机程序：

```bash
./run.sh realrobot_mini
```

构建并运行 MuJoCo 仿真：

```bash
./autobuild.sh mujoco_sim_mini
./run.sh mujoco_sim_mini
```

构建全部测试程序：

```bash
./autobuild.sh test
```

清理构建产物：

```bash
./autoclean.sh
```

构建完成后，可执行文件输出到 `bin/`；中间构建产物输出到 `build/`。`run.sh` 会自动设置 `LD_LIBRARY_PATH`，并从 `bin/` 目录启动目标程序。

## 构建目标

| 命令 | 输出目标 | 说明 |
| --- | --- | --- |
| `./autobuild.sh realrobot_mini` | `bin/realrobot_mini` | 真机运行程序。 |
| `./autobuild.sh mujoco_sim_mini` | `bin/mujoco_sim_mini` | MuJoCo 仿真程序。 |
| `./autobuild.sh test` | 多个测试程序 | 构建全部测试目标。 |
| `./autobuild.sh test joy_test` | `bin/joy_test` | 只构建手柄测试。 |
| `./autobuild.sh test imu_test` | `bin/imu_test` | 只构建 IMU 测试。 |
| `./autobuild.sh motor_control` | `bin/motor_control` | 构建 21 电机归零示例。 |
| `./autobuild.sh pd_control` | `bin/pd_control` | 构建动态 PD 设置示例。 |

也可以通过环境变量调整并行构建线程数：

```bash
JOBS=8 ./autobuild.sh test
```

## 运行目标

| 命令 | 说明 |
| --- | --- |
| `./run.sh realrobot_mini` | 运行真机程序。 |
| `./run.sh mujoco_sim_mini` | 运行 MuJoCo 仿真程序。 |
| `./run.sh joy_test` | 运行手柄测试。 |
| `./run.sh imu_test` | 运行 IMU 测试。 |
| `./run.sh motor_control` | 运行 21 电机归零示例。 |
| `./run.sh pd_control 0 50 5` | 对 index 为 `0` 的电机设置 `PDKp=50`、`PDKd=5`。 |
| `./run.sh set_zero` | 按配置执行整机归零。 |
| `./run.sh set_one_zero` | 对配置文件指定电机置零。 |
| `./run.sh set_one_Enable` | 对配置文件指定电机使能。 |
| `./run.sh read_pos` | 读取电机位置。 |
| `./run.sh head_test` | 运行头部或相关执行器测试程序。 |

`run.sh` 也支持部分别名，例如 `joy`、`imu`、`motor`、`pd`、`zero`、`one_zero`。

## 目录结构

| 路径 | 说明 |
| --- | --- |
| `CMakeLists.txt` | 客户侧 CMake 工程入口。 |
| `autobuild.sh` | 构建脚本，封装常用 CMake 构建目标。 |
| `run.sh` | 运行脚本，负责设置动态库路径并启动 `bin/` 下的程序。 |
| `autoclean.sh` | 清理脚本，删除 `build/` 和 `bin/`。 |
| `DataPackage/` | 数据总线头文件和配置，状态机、手柄、电机和 RL 模块通过它交换数据。 |
| `StateMachine/` | 状态机源码、状态枚举、状态注册入口和自定义状态示例。 |
| `ZeroState/` | `ZERO` 状态源码和归零参数。 |
| `RL_deploy_cpg/` | `RL_walk` 源码、模型路径配置和部署参数。 |
| `RobotStart/test/` | 客户可参考或修改的测试程序源码。 |
| `MotorList/` | 电机列表头文件和 YAML 配置；底层实现通过库链接。 |
| `Model_airplane_Joystick/` | 飞机手柄头文件和配置；实现通过闭源库链接。 |
| `MujocoInterface/` | MuJoCo 仿真接口头文件和配置。 |
| `RobotModel/` | 机器人模型、URDF、XML、mesh 和关节名配置。 |
| `device/` | 设备接口源码和配置，目前主要包含 Hipnuc IMU 读取。 |
| `LowPassFilter/` | 低通滤波相关头文件。 |
| `ThirdParty/` | 第三方头文件和运行依赖。 |
| `lib/` | 运行时动态库，例如 Torch、MuJoCo、yaml-cpp、MotorDrive。 |
| `prebuilt/` | 闭源静态库和预编译目标文件。 |

## 状态机与手柄

当前状态枚举定义在 `StateMachine/include/fsmlist.h`：

```cpp
enum class State {
    IDLE,
    ZERO,
    RL_walk,
    SIMPLE_EXAMPLE,
};
```

状态注册入口位于 `StateMachine/include/state_registry_user.h`：

```cpp
#define PHYBOT_FOR_EACH_STATE(M) \
    M(ZeroState, State::ZERO) \
    M(rl_deploy_cpg, State::RL_walk) \
    M(SimpleStateExample, State::SIMPLE_EXAMPLE)
```

闭源手柄库内置了以下状态切换示例：

| 按键 | 目标状态 |
| --- | --- |
| `X` | `State::ZERO` |
| `B` | `State::RL_walk` |
| `A` | `State::SIMPLE_EXAMPLE` |

`SIMPLE_EXAMPLE` 是预留的客户自定义状态槽位。若要替换为客户自己的状态类，通常需要：

1. 在 `StateMachine/include/fsmlist.h` 中增加或复用状态枚举。
2. 在 `StateMachine/include/state_registry_user.h` 中包含客户状态类头文件。
3. 在 `PHYBOT_FOR_EACH_STATE(M)` 中注册状态类和状态枚举。

例如：

```cpp
#define PHYBOT_FOR_EACH_STATE(M) \
    M(ZeroState, State::ZERO) \
    M(rl_deploy_cpg, State::RL_walk) \
    M(MyCustomerState, State::SIMPLE_EXAMPLE)
```

如果需要在开放源码程序中接入自定义按键逻辑，可参考 `RobotStart/test/joy_test.cpp`。状态切换建议放在 `joystick.run()` 和 `joystick.SetDataToPackage(package)` 之间：

```cpp
joystick.GetDataFromPackage(package);
joystick.run();
joystick.set_next_state(State::SIMPLE_EXAMPLE);
joystick.SetDataToPackage(package);
```

公开的手柄接口包括：

| 接口 | 说明 |
| --- | --- |
| `get_button_state()` | 获取当前按键和摇杆原始状态。 |
| `get_pressed_buttons()` | 获取当前按下的按钮名称列表。 |
| `get_next_state()` | 获取下一状态枚举。 |
| `get_next_state_name()` | 获取下一状态名称。 |
| `set_next_state(State state)` | 设置下一状态。 |

## 测试程序

| 程序 | 说明 |
| --- | --- |
| `joy_test` | 打印手柄按键、摇杆通道、速度指令和状态切换结果。 |
| `imu_test` | 读取并打印 IMU 数据，用于检查串口和波特率配置。 |
| `motor_control` | 21 电机平滑归零示例，通过 `GetStatesToPackage` 读取位置，通过 `SetCommandsFromPackage` 下发命令。 |
| `pd_control` | 动态 PD 设置示例，先下发配置中的初始 PD，再使能电机，然后对指定电机执行 `SetPD` 和 `GetP/Getd`。 |
| `read_pos` | 读取电机位置。 |
| `set_zero` | 按配置执行归零。 |
| `set_one_zero` | 对 `set_one_zero.yaml` 中指定电机置零。 |
| `set_one_Enable` | 对 `set_one_Enable.yaml` 中指定电机使能。 |
| `head_test` | 头部或相关执行器测试程序。 |

### motor_control

客户通常修改 `RobotStart/test/motor_control.cpp` 中的 `zero_pos`：

```cpp
vector<float> zero_pos = {
    // 21 个关节目标初始位置，单位 rad
};
```

程序会检查每个目标值是否在 `[-1, 1]` 范围内，然后使用五阶插值平滑移动到目标位置，并打印电机 index、ID、名称、当前位置、命令位置和目标位置。

电机顺序为：

```text
左腿里到外 -> 右腿里到外 -> 腰 -> 左手里到外 -> 右手里到外
```

### pd_control

客户可以修改 `RobotStart/test/pd_control.cpp` 中的默认值：

```cpp
int target_index = 0;
float new_p = 50.0f;
float new_d = 5.0f;
```

也可以在运行时传参：

```bash
./run.sh pd_control 0 50 5
```

参数含义依次为：

| 参数 | 说明 |
| --- | --- |
| `0` | 电机 index。 |
| `50` | 新的 `PDKp`。 |
| `5` | 新的 `PDKd`。 |

## 配置文件

| 配置文件 | 说明 |
| --- | --- |
| `DataPackage/config/datapackage.yaml` | 全局数据维度和控制周期。当前 `control_period: 0.002`，`actuatedDofNum: 21`。 |
| `Model_airplane_Joystick/config/joystick.yaml` | 手柄串口、速度限制和速度变化率配置。默认设备为 `/dev/ttyjoy`。 |
| `device/config/device.yaml` | IMU 串口配置。默认 `port_name: /dev/ttyimu`，`baud_rate: 921600`。 |
| `MotorList/config/phybot_mini_1.yaml` | 第一块电机主板配置。 |
| `MotorList/config/phybot_mini_2.yaml` | 第二块电机主板配置。`realrobot_mini` 和 `motor_control` 使用这两个 mini 配置。 |
| `MotorList/config/phybot_abszero.yaml` | 电机绝对零点偏置，长度应与电机数量一致。 |
| `MotorList/config/set_one_Enable.yaml` | `set_one_Enable` 使用的电机 ID 列表。 |
| `MotorList/config/set_one_zero.yaml` | `set_one_zero` 使用的电机 ID 列表。 |
| `MotorList/config/failed_motors.yaml` | 电机使能失败时保存失败 ID，通常由程序写入。 |
| `MotorList/config/phybot1.yaml`、`MotorList/config/phybot2.yaml` | 其他机型或历史配置，mini 主流程通常不使用。 |
| `MujocoInterface/config/mujoco_sim.yaml` | MuJoCo 仿真配置，包含模型路径、关节名、基座名和传感器名。 |
| `RL_deploy_cpg/config/rl_params.yaml` | `RL_walk` 参数，包含模型路径、网络维度、PD 增益、动作缩放、力矩限制和默认关节位置。 |
| `ZeroState/config/ZERO.yaml` | `ZERO` 状态参数，包含归零目标、控制增益和归零时间。 |
| `RobotModel/*/config/joint_names_*.yaml` | 机器人模型关节名映射，主要供模型、仿真或可视化使用。 |

### 电机主板配置字段

`MotorList/config/phybot_mini_1.yaml` 和 `MotorList/config/phybot_mini_2.yaml` 中常见字段如下：

| 字段 | 说明 |
| --- | --- |
| `Network.LocalIp` | 本机网卡 IP，需要与实际网卡一致。 |
| `Network.LocalPort` | 本机 UDP 端口。 |
| `Network.DestIp` | 电机主板 IP。 |
| `Network.DestPort` | 电机主板 UDP 端口。 |
| `CommunicationFre` | 通信频率。 |
| `Motor.Id` | 电机 ID，测试程序打印时也会显示。 |
| `Motor.Name` | 电机名称，用于确认 index 与实际关节的对应关系。 |
| `ControlMode` | 电机控制模式。 |
| `FastMode` | 快速通信模式开关。 |
| `PDKp`、`PDKd` | 电机 PD 参数。 |
| `Direction` | 电机方向，影响命令和反馈符号。 |

## 常见修改位置

| 目标 | 修改位置 |
| --- | --- |
| 修改 21 电机初始姿态 | `RobotStart/test/motor_control.cpp` 中的 `zero_pos`。 |
| 动态调整电机 PD | `RobotStart/test/pd_control.cpp` 中的 `target_index`、`new_p`、`new_d`，或运行时传参。 |
| 修改真机归零姿态 | `ZeroState/config/ZERO.yaml` 中的 `zero_joint_position`。 |
| 修改手柄设备名 | `Model_airplane_Joystick/config/joystick.yaml` 中的 `dev`。 |
| 修改 IMU 串口 | `device/config/device.yaml` 中的 `port_name` 和 `baud_rate`。 |
| 修改电机 IP 或端口 | `MotorList/config/phybot_mini_1.yaml`、`MotorList/config/phybot_mini_2.yaml` 中的 `Network`。 |
| 修改 RL 模型 | `RL_deploy_cpg/config/rl_params.yaml` 中的 `policy_path`。 |
| 接入客户状态 | `StateMachine/include/fsmlist.h`、`StateMachine/include/state_registry_user.h`、`StateMachine/examples/`。 |

## 安全注意事项

- 真机运行前，确认机器人有可靠支撑，急停可用，人员远离运动范围。
- 修改 `zero_pos`、`zero_joint_position`、PD 参数或 RL 模型后，先在低风险环境验证，再上真机。
- 修改电机网络配置前，确认本机网卡 IP、目标板 IP 和端口与现场硬件一致。
- `motor_control` 会使能电机并下发位置命令，运行前确认 21 个目标位置与实际关节顺序一致。
- `pd_control` 会实际修改目标电机 PD 参数，运行前确认电机 index、ID 和名称。