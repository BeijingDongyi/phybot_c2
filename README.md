# Phybot 客户开发包交付说明

## 快速使用

编译：

```bash
./autobuild.sh realrobot_mini
./autobuild.sh mujoco_sim_mini
./autobuild.sh test
./autobuild.sh test joy_test
./autobuild.sh motor_control
./autobuild.sh pd_control
```

运行：

```bash
./run.sh realrobot_mini
./run.sh mujoco_sim_mini
./run.sh joy_test
./run.sh imu_test
./run.sh motor_control
./run.sh pd_control 0 50 5
```

清理编译产物：

```bash
./autoclean.sh
```

## 交付内容

| 路径 | 作用 |
| --- | --- |
| `CMakeLists.txt` | 我们提供的客户侧 CMake 工程入口。 |
| `autobuild.sh` | 我们提供的编译脚本，支持真机、仿真和测试目标。 |
| `run.sh` | 我们提供的运行脚本，会自动设置 `LD_LIBRARY_PATH` 并从 `bin/` 启动程序。 |
| `autoclean.sh` | 我们提供的清理脚本，用于删除 `build/`、`bin/` 编译产物。 |
| `DataPackage/` | 数据总线头文件和配置，状态机、手柄、电机、RL 之间通过它交换数据。 |
| `device/` | 设备接口源码，目前主要包含 Hipnuc IMU 读取相关代码。 |
| `MotorList/` | 电机列表头文件和配置。电机底层实现不开放，通过库链接。 |
| `Model_airplane_Joystick/` | 我们只开放飞机手柄头文件和配置。`src/` 不交付，实际实现链接 `prebuilt/lib/libphybot_joystick_closed.a`。 |
| `StateMachine/` | 我们保留状态机源码，客户可以注册自己的状态。 |
| `ZeroState/` | 归零状态源码和参数。 |
| `RL_deploy_cpg/` | 我们开放 `RL_walk` 源码、模型和参数。其他 RL 状态不交付源码。 |
| `RobotStart/test/` | 我们开放的测试程序源码，包括 `imu_test.cpp`、`motor_control.cpp`、`pd_control.cpp`、`joy_test.cpp`。 |
| `MujocoInterface/` | MuJoCo 仿真接口头文件和配置。 |
| `RobotModel/` | 仿真和模型资源。 |
| `ThirdParty/` | 第三方头文件和运行依赖。 |
| `lib/` | 运行时动态库，如 Torch、MuJoCo、yaml-cpp、MotorDrive 等。 |
| `prebuilt/` | 闭源静态库和预编译目标文件。 |

## 编译目标说明

| 命令 | 说明 |
| --- | --- |
| `./autobuild.sh realrobot_mini` | 编译真机运行程序 `realrobot_mini`。 |
| `./autobuild.sh mujoco_sim_mini` | 编译 MuJoCo 仿真程序 `mujoco_sim_mini`。 |
| `./autobuild.sh test` | 编译所有测试程序。 |
| `./autobuild.sh test joy_test` | 只编译手柄测试。 |
| `./autobuild.sh test imu_test` | 只编译 IMU 测试。 |
| `./autobuild.sh motor_control` | 编译 21 电机归零示例。 |
| `./autobuild.sh pd_control` | 编译动态 PD 设置示例。 |

## 状态机和手柄

我们当前已经注册了三个状态：

- `State::ZERO`
- `State::RL_walk`
- `State::SIMPLE_EXAMPLE`

我们把状态枚举放在 `StateMachine/include/fsmlist.h`，把状态注册入口放在 `StateMachine/include/state_registry_user.h`。

我们在闭源手柄库中内置了以下切换示例：

```cpp
X -> State::ZERO
B -> State::RL_walk
A -> State::SIMPLE_EXAMPLE
```

`SIMPLE_EXAMPLE` 是我们给客户预留的自定义状态模板。最简单的接入方式是保留 `State::SIMPLE_EXAMPLE` 这个状态槽位，然后把 `state_registry_user.h` 中的实现类替换为客户自己的类：

```cpp
#define PHYBOT_FOR_EACH_STATE(M) \
    M(ZeroState, State::ZERO) \
    M(rl_deploy_cpg, State::RL_walk) \
    M(MyCustomerState, State::SIMPLE_EXAMPLE)
```

这样客户在真机程序里仍然可以用 A 键切入自己的状态。

如果客户要在开放源码程序中自行接入新的按键逻辑，我们提供了 `RobotStart/test/joy_test.cpp` 作为参考：

```cpp
joystick.GetDataFromPackage(package);
joystick.run();
joystick.set_next_state(State::SIMPLE_EXAMPLE);
joystick.SetDataToPackage(package);
```

我们公开的手柄接口包括：

- `get_button_state()`
- `get_pressed_buttons()`
- `get_next_state()`
- `get_next_state_name()`
- `set_next_state(State state)`

## 测试程序说明

| 程序 | 作用 |
| --- | --- |
| `joy_test` | 打印当前按下的手柄按键、摇杆通道、速度指令和状态切换结果。 |
| `imu_test` | 读取并打印 IMU 数据，用于检查 `/dev/ttyimu` 和波特率配置。 |
| `motor_control` | 21 电机归零示例。通过 `GetStatesToPackage` 读位置，通过 `SetCommandsFromPackage` 下发命令。 |
| `pd_control` | 动态 PD 设置示例。先按配置里的初始化 PD 下发，再使能，再对一个电机 `SetPD` 并 `GetP/Getd` 打印确认。 |
| `read_pos` | 读取电机位置。 |
| `set_zero` | 按配置执行归零。 |
| `set_one_zero` | 对 `set_one_zero.yaml` 中指定电机置零。 |
| `set_one_Enable` | 对 `set_one_Enable.yaml` 中指定电机使能。 |
| `head_test` | 头部/相关执行器测试程序。 |

客户在 `motor_control.cpp` 中主要需要修改：

```cpp
vector<float> zero_pos = {
    // 21 个关节目标初始位置，单位 rad
};
```

我们在程序里检查每个值是否在 `[-1, 1]`，然后用五阶插值平滑移动到该位置，并打印每个电机的当前位置、命令位置和目标位置。

客户在 `pd_control.cpp` 中主要修改：

```cpp
int target_index = 0;
float new_p = 50.0f;
float new_d = 5.0f;
```

也可以直接运行时传入：

```bash
./run.sh pd_control 0 50 5
```

## YAML 配置说明

我们随包提供以下主要 YAML 配置文件：

| 配置文件 | 作用 |
| --- | --- |
| `DataPackage/config/datapackage.yaml` | 全局数据维度和控制周期。`control_period` 当前为 `0.002` 秒，`actuatedDofNum` 当前为 `21`。 |
| `Model_airplane_Joystick/config/joystick.yaml` | 手柄设备和速度限制。`dev` 是手柄串口设备，默认 `/dev/ttyjoy`；`acc_max`、`angacc_max` 控制速度变化率；`maxspeed_x/y/yaw` 和 `minspeed_x/y/yaw` 控制手柄速度范围。 |
| `MotorList/config/phybot_mini_1.yaml` | 第一块电机主板配置，包含本机 IP、端口、目标板 IP、通信频率和电机列表。 |
| `MotorList/config/phybot_mini_2.yaml` | 第二块电机主板配置，字段含义同上。`realrobot_mini` 和 `motor_control` 使用这两个 mini 配置。 |
| `MotorList/config/phybot_abszero.yaml` | 电机绝对零点偏置，长度应与电机数一致。 |
| `MotorList/config/set_one_Enable.yaml` | `set_one_Enable` 使用的电机 ID 列表。 |
| `MotorList/config/set_one_zero.yaml` | `set_one_zero` 使用的电机 ID 列表。 |
| `MotorList/config/failed_motors.yaml` | 电机使能失败时保存失败 ID，通常由程序写入。 |
| `MotorList/config/phybot1.yaml`、`phybot2.yaml` | 我们保留的其他机型/历史配置，当前 mini 主流程一般不使用。 |
| `MujocoInterface/config/mujoco_sim.yaml` | MuJoCo 仿真配置，包含 `env_path`、关节名列表、基座名和传感器名。 |
| `RL_deploy_cpg/config/rl_params.yaml` | `RL_walk` 参数。包含 `policy_path`、网络输入维度、PD 增益、动作缩放、力矩限制、默认关节位置等。 |
| `ZeroState/config/ZERO.yaml` | `ZERO` 状态参数。`zero_joint_position` 是 21 关节归零目标；`zero_p`、`zero_d` 是控制增益；`zero_totalTime` 是归零时间。 |
| `device/config/device.yaml` | IMU 串口配置，默认 `port_name: /dev/ttyimu`，`baud_rate: 921600`。 |
| `RobotModel/*/config/joint_names_*.yaml` | 机器人模型关节名映射，主要供模型、仿真或可视化资源使用。 |

`phybot_mini_1.yaml` 和 `phybot_mini_2.yaml` 中常见字段：

| 字段 | 说明 |
| --- | --- |
| `Network.LocalIp` | 本机网卡 IP，需要与实际网卡一致。 |
| `Network.LocalPort` | 本机 UDP 端口。 |
| `Network.DestIp` | 电机主板 IP。 |
| `Network.DestPort` | 电机主板 UDP 端口。 |
| `CommunicationFre` | 通信频率。 |
| `Motor.Id` | 电机 ID，测试程序打印时也会显示。 |
| `Motor.Name` | 电机名称，便于客户确认 index 和实际关节的关系。 |
| `ControlMode` | 电机控制模式。 |
| `FastMode` | 快速通信模式开关。 |
| `PDKp`、`PDKd` | 电机 PD 参数。 |
| `Direction` | 电机方向，影响命令和反馈符号。 |

## 常见修改位置

| 目标 | 修改位置 |
| --- | --- |
| 修改 21 电机初始姿态 | `RobotStart/test/motor_control.cpp` 中的 `zero_pos`。 |
| 动态调整电机 PD | `RobotStart/test/pd_control.cpp` 中的 `target_index`、`new_p`、`new_d`。 |
| 修改真机归零姿态 | `ZeroState/config/ZERO.yaml` 中的 `zero_joint_position`。 |
| 修改手柄设备名 | `Model_airplane_Joystick/config/joystick.yaml` 中的 `dev`。 |
| 修改电机 IP 或端口 | `MotorList/config/phybot_mini_1.yaml`、`phybot_mini_2.yaml` 中的 `Network`。 |
| 修改 RL 模型 | `RL_deploy_cpg/config/rl_params.yaml` 中的 `policy_path`。 |
| 接入客户状态 | `StateMachine/include/fsmlist.h`、`StateMachine/include/state_registry_user.h`、`StateMachine/examples/`。 |

# 电机顺序
- 左腿里到外 右腿里到外 腰 左手里到外 右手里到外
