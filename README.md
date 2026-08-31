# PHYBOT_C2_SDK_1.1

Phybot C2 客户交付软件包。

## 1. 软件信息

- 软件包名称：PHYBOT_C2_SDK_1.1
- 运行环境：all
- 手柄类型：Model_airplane
- 保留源码包：无
- 架构：Linux x86_64
- 编译类型：Release
- 交付内容：可执行二进制、运行时动态库、配置文件、模型文件和数据文件

本软件包不包含 C/C++ 源文件、头文件、CMake 工程文件或静态库。

## 2. 目录说明

```text
PHYBOT_C2_SDK_1.1/
├── bin/                    # 可执行程序
│   ├── realrobot_mini_main # 真机环境主程序
│   ├── mujoco_sim_mini_main# MuJoCo 仿真主程序
│   └── test_*              # 测试程序
├── lib/                    # 程序运行所需的动态库
├── RL_deploy_*/            # RL 配置、模型和运行数据
├── MotorList/config/       # 电机配置
├── device/config/          # IMU 等设备配置
├── ZeroState/config/       # 归零配置
├── DataPackage/config/     # 数据包配置
├── RobotModel/phybot_c2/   # MuJoCo XML、URDF 和 STL 模型资源
├── PHYBOT_C2_SDK_1.1.version # 软件版本标识
├── README.md               # 本说明文件
└── run.sh                  # 推荐启动脚本
```

## 3. 运行方式

请在目标 Linux 系统上解压整个目录，不要单独复制 `bin/` 中的程序。

```bash
cd PHYBOT_C2_SDK_1.1
chmod +x run.sh
./run.sh realrobot_mini_main
```

查看可用程序：

```bash
./run.sh
```

`run.sh` 会自动设置本包的动态库路径，并从 `bin/` 目录启动程序，保证现有配置中的相对路径能够正常工作。

### 3.1 程序清单

| 程序 | 用途 | 备注 |
|---|---|---|
| `realrobot_mini_main` | C2 小型真机控制 | 连接真实电机、IMU 和手柄前使用 |
| `mujoco_sim_mini_main` | MuJoCo 仿真 | 使用 `RobotModel/phybot_c2/xml/phybot_c2.xml` |
| `test_set_zero` | 指定电机归零 | 先检查 `MotorList/config/set_one_zero.yaml` |
| `test_set_one_zero` | 单个电机归零 | 仅在明确电机 ID 后使用 |
| `test_set_one_Enable` | 单个电机使能测试 | 用于排查电机在线和使能问题 |
| `test_read_pos` | 读取电机位置 | 不执行运动控制 |
| `test_joy_test` | 手柄输入测试 | 用于确认手柄映射和输入值 |
| `test_head_test` | 头部关节测试 | 仅在确认机械安全后运行 |
| `test_imu_test` | IMU 通信测试 | 用于检查串口和 IMU 数据 |

不同程序名称前的环境前缀用于避免多个环境中的 `main` 文件相互覆盖。

### 3.2 推荐启动顺序

真机首次启动建议按以下顺序进行：

1. 检查机器人处于安全姿态，急停可用，电机周围没有障碍物；
2. 检查 `device/config/device.yaml` 中的 IMU 串口和波特率；
3. 检查 `MotorList/config/` 中的电机数量、CAN 配置和机器人型号；
4. 如需单独归零，先运行测试程序确认通信，再执行归零程序；
5. 确认手柄拨杆处于安全位置后，再启动 `realrobot_mini_main`。

## 4. 配置文件

常用配置位置如下：

- 电机配置：`MotorList/config/`
- IMU 配置：`device/config/device.yaml`
- RL 参数：各 `RL_deploy_*/config/rl_params.yaml`
- RL 模型：各 `RL_deploy_*/model/`
- 归零配置：`ZeroState/config/ZERO.yaml`

修改配置前请备份原文件。模型文件通常不能直接用文本编辑器修改。

### 4.1 手柄配置

本包构建时使用的手柄类型是 `Model_airplane`。手柄速度和按键映射位于：

- 航模手柄：`Model_airplane_Joystick/config/joystick.yaml`
- 普通手柄：`Joystick/config/joystick.yaml`

如果需要切换手柄类型，必须重新编译交付包，不能只替换配置文件：

```bash
./package_release.sh all Joystick PHYBOT_C2_SDK_1.2 ./deliveries
```

### 4.2 RL 模型和参数

每个 RL 包通常包含以下内容：

```text
RL_deploy_xxx/
├── config/rl_params.yaml  # 观测、动作、频率等参数
├── model/                 # TorchScript 或策略模型
└── data/                  # 动作轨迹或辅助数据
```

修改 `rl_params.yaml` 前需要确认参数与模型训练时的网络结构一致。随意修改观测维度、动作维度、关节顺序或控制频率，可能导致模型加载失败或机器人动作异常。

### 4.3 可扩展功能包

新增 LED、蜂鸣器、传感器或其他通用功能时，建议使用以下目录约定：

```text
Feature_led/
├── include/                # 头文件
├── src/                    # C/C++ 源文件
├── config/                 # 运行配置，可选
├── resources/              # 固件、字库或其他运行资源，可选
└── assets/                 # 图片、表格等运行资源，可选
```

工程会自动发现 `Feature_*` 和 `RL_deploy_*` 目录中的 `src/*.c`、`src/*.cc`、`src/*.cpp` 和 `src/*.cxx` 并编译进公共库。交付脚本会自动复制这些目录中的 `config/`、`model/`、`data/`、`resources/` 和 `assets/`，不会复制 `src/`、`include/` 或构建文件。

因此，新增一个 LED 功能包通常不需要修改 `package_release.sh`。只需：

1. 创建 `Feature_led/include` 和 `Feature_led/src`；
2. 在现有主程序或状态机中注册并调用 LED 类；
3. 如有配置或资源，放入 `Feature_led/config` 或 `Feature_led/resources`；
4. 重新生成包并检查 `bin/` 中的程序。

如果 LED 使用独立动态库，可将运行时 `.so` 放在 `Feature_led/lib/`，脚本会自动收集到交付包的 `lib/`。

### 4.4 MuJoCo 模型

MuJoCo 主程序读取：

```text
RobotModel/phybot_c2/xml/phybot_c2.xml
```

该 XML 依赖同目录下的 STL 网格文件，因此 `RobotModel/phybot_c2/xml/` 和 `RobotModel/phybot_c2/new_meshes/` 必须保持原有相对目录结构，不能只复制 XML 文件。

## 5. 日志、权限和运行目录

程序必须通过 `run.sh` 启动，或者确保当前工作目录等同于包内的 `bin/` 目录。直接从其他目录运行二进制可能导致 `../MotorList/config/...` 等相对路径找不到。

日志和运行过程中生成的数据可能写入对应 RL 模块的 `logs/` 或 `data/` 目录。客户部署时建议给整个包目录保留写权限，或者将日志目录单独映射到可写磁盘。

常用检查命令：

```bash
pwd
find ./bin -maxdepth 1 -type f -executable -printf '%f\\n'
df -h .
ls -l /dev/tty* 2>/dev/null
```

## 6. 依赖和硬件要求

- 操作系统：Linux x86_64；
- 编译器和 CMake：仅源码编译时需要，客户运行二进制时不需要；
- 动态库：已随包放入 `lib/`；
- MuJoCo：使用包内的 MuJoCo 动态库和机器人模型；
- 真机运行：需要正确连接电机、CAN 设备、IMU 和对应手柄；
- 串口权限：当前用户需要有访问 IMU 串口设备的权限；
- 本交付包不包含 Gazebo 和 ROS 运行环境。

## 7. 配置修改和备份建议

建议在修改前建立版本备份：

```bash
cp -a MotorList/config MotorList/config.backup.$(date +%Y%m%d_%H%M%S)
cp -a device/config/device.yaml device/config/device.yaml.backup
```

不建议直接修改以下内容：

- RL 模型文件（`.pt`）；
- MuJoCo XML 中的网格路径；
- 电机 ID、方向和关节顺序；
- 观测/动作维度以及控制频率。

上述内容需要由熟悉机器人控制和模型训练的工程师修改并重新验证。

## 8. 新版本交付流程

源码有更新或新增/删除 RL 包后，在源码工程根目录重新执行：

```bash
./package_release.sh all Model_airplane PHYBOT_C2_SDK_1.1 ./deliveries
```

脚本会自动：

1. 分别编译真机、MuJoCo 和 test 环境；
2. 自动发现 `RL_deploy_*` 目录中的源文件；
3. 收集运行所需的配置、模型和动态库；
4. 为同名程序增加环境前缀，避免多个 `main` 相互覆盖；
5. 删除可执行文件中的非必要符号；
6. 生成新的客户交付包和 README；
7. 检查交付包中只有指定功能包可以保留源码。

如果只需要某一个环境，可以指定环境名称：

```bash
./package_release.sh mujoco_sim_mini Model_airplane PHYBOT_C2_SDK_1.1 ./deliveries
```

如果希望某个功能包在交付包中保留源码，可以在第五个参数中使用逗号分隔的目录名。该功能包仍会正常编译，但它的源码和头文件也会进入交付包：

```bash
./package_release.sh all Model_airplane PHYBOT_C2_SDK_1.1 ./deliveries Feature_led,RL_deploy_crane_down
```

如果不需要保留任何源码，传入 `none` 或省略第五个参数：

```bash
./package_release.sh all Model_airplane PHYBOT_C2_SDK_1.1 ./deliveries none
```

本封装脚本不包含 Gazebo 功能。Gazebo 相关源码和 ROS 依赖不会进入客户交付包。

`RL_deploy_cpg_run` 仍是 CPG 的备用实现，默认不会参与编译，以避免和 `RL_deploy_cpg` 产生重复符号。第五个参数只控制交付包是否保留源码，不改变这个 CMake 编译规则。

如果新 RL 包需要通过手柄或状态机触发，仍需在状态机的状态枚举、注册表和控制逻辑中完成业务注册。

## 9. 安全说明

本包不直接交付源码，但 Linux ELF 二进制仍可能被专业工具逆向分析。建议同时采取客户授权、设备绑定、版本水印、文件完整性校验和许可证校验等措施。

## 10. 故障排查

检查动态库依赖：

```bash
LD_LIBRARY_PATH=./lib ldd ./bin/realrobot_mini_main
```

常见问题：

### 程序提示动态库 `not found`

确认使用 `run.sh` 启动，并检查：

```bash
LD_LIBRARY_PATH=./lib ldd ./bin/realrobot_mini_main | grep 'not found'
```

如果仍有缺失，通常是交付包不完整，或者 `bin/`、`lib/` 被单独移动。

### 程序提示配置或模型文件不存在

确认当前目录是包根目录，并通过 `run.sh` 启动。检查目标文件：

```bash
test -f MotorList/config/phybot_mini_1.yaml && echo OK
test -f RobotModel/phybot_c2/xml/phybot_c2.xml && echo OK
test -d RobotModel/phybot_c2/new_meshes && echo OK
```

### 真机电机无法使能

依次检查电源、CAN 接线、设备权限、MotorList 配置、电机 ID 和急停状态。不要在未确认电机状态时重复执行使能或运动程序。

### IMU 数据为 `nan` 或无数据

检查 `device/config/device.yaml` 的串口名、波特率和设备权限，确认没有其他程序占用该串口，再运行 `test_imu_test`。

### MuJoCo 找不到网格

确认 `RobotModel/phybot_c2/xml/phybot_c2.xml` 和 `RobotModel/phybot_c2/new_meshes/` 均存在，并且没有改变目录层级。

## 11. 交付边界

本包用于客户运行和现场配置，不提供源码级二次开发接口。新增 RL 包、修改状态机、修改电机控制逻辑或更换模型，都应由交付方重新编译并生成新的版本包。

如需问题定位，请同时提供软件包版本文件、启动命令、终端完整日志、修改过的配置文件名以及硬件连接状态。
