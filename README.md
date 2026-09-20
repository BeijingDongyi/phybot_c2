# PHYBOT_C2_UBUNTU20_1.0

Phybot C2 客户交付软件包。

## 1. 软件信息

- 软件包名称：PHYBOT_C2_UBUNTU20_1.0
- 交付目标：ubuntu20.04（ubuntu20.04 表示 Ubuntu 20.04 x86_64）
- 运行环境：all
- 手柄类型：Model_airplane
- 保留源码包：device,RobotStart,ZeroState,RL_deploy_cpg,StateMachine
- 保留头文件包：LowPassFilter,Model_airplane_Joystick,MotorList,MujocoInterface,DataPackage,LED
- 架构：Linux x86_64
- 编译类型：Release
- 交付内容：源码、头文件、CMake 工程、运行时动态库、配置文件、模型文件、数据文件和编译/清理脚本

本软件包是需要在目标机上编译的 SDK 包，不预先交付 `bin/` 可执行文件。包含编译所需的 C/C++ 源文件、头文件、CMake 工程文件和 ThirdParty 依赖。

## 2. 目录说明

```text
PHYBOT_C2_UBUNTU20_1.0/
├── CMakeLists.txt          # CMake 工程入口
├── build/                  # 运行 autobuild.sh 后生成
├── lib/                    # 程序运行所需的动态库
├── ThirdParty/             # 编译和运行依赖
├── StateMachine/           # 状态机源码
├── RobotStart/             # 真机、仿真和测试入口源码
├── RL_deploy_*/            # RL 配置、模型和运行数据
├── MotorList/config/       # 电机配置
├── device/config/          # IMU 等设备配置
├── ZeroState/config/       # 归零配置
├── DataPackage/config/     # 数据包配置
├── RobotModel/phybot_c2/   # MuJoCo XML、URDF 和 STL 模型资源
├── autobuild.sh            # 非交互编译入口
├── autoclean.sh            # 清理 build 目录
├── PHYBOT_C2_UBUNTU20_1.0.version # 软件版本标识
├── README.md               # 本说明文件
└── run.sh                  # 推荐启动脚本
```

## 3. 运行方式

请在目标 Linux 系统上解压整个目录，首先执行编译脚本：

```bash
cd PHYBOT_C2_UBUNTU20_1.0
./install_dependencies.sh  # Ubuntu 20.04 首次使用时安装编译和图形依赖
./autobuild.sh              # 交互选择环境和新/旧 IMU
```

`autobuild.sh` 不传参数时会依次让用户手动选择运行环境和 IMU 类型：`Hipnuc` 是旧 IMU，`IMU` 是新 IMU。需要自动构建时，第一个参数可传 `realrobot_mini`、`mujoco_sim_mini` 或 `test`，第二个参数可传 `Hipnuc` 或 `IMU`；也可以通过 `BUILD_ENVIRONMENT` 和 `PHYBOT_IMU_TYPE` 环境变量选择。编译结果会生成在 `build/` 目录。例如真机或 MuJoCo 环境编译后：

```bash
./run.sh main
```

查看可用程序：

```bash
./run.sh
```

`run.sh` 会自动设置本包的动态库路径，并从 `build/` 目录启动程序，保证现有配置中的相对路径能够正常工作。

清理编译目录：

```bash
./autoclean.sh
```

如果打包命令第六个参数指定了“只保留头文件功能包”，这些包会递归保留 `include/`、`config/`、`model/`、`data/`、`resources/`、`assets/`、`lib/` 和必要的父目录，其他内容会被移除。第五个参数指定的源码目录及其子目录优先完整保留。编译时会链接 `prebuilt/<IMU类型>/<环境>/` 下的预编译静态库，因此 `autobuild.sh` 仍可使用；这类头文件包本身不能在交付包内修改源码后重新编译。

### 3.1 程序清单

| 程序 | 用途 | 备注 |
|---|---|---|
| `main` | 真机或 MuJoCo 主程序 | 具体类型取决于 `autobuild.sh` 中的选择 |
| `set_zero` | 指定电机归零 | 先检查 `MotorList/config/set_one_zero.yaml` |
| `set_one_zero` | 单个电机归零 | 仅在明确电机 ID 后使用 |
| `set_one_Enable` | 单个电机使能测试 | 用于排查电机在线和使能问题 |
| `read_pos` | 读取电机位置 | 不执行运动控制 |
| `joy_test` | 手柄输入测试 | 用于确认手柄映射和输入值 |
| `head_test` | 头部关节测试 | 仅在确认机械安全后运行 |
| `imu_test` | IMU 通信测试 | 用于检查串口和 IMU 数据 |

### 3.2 推荐启动顺序

真机首次启动建议按以下顺序进行：

1. 检查机器人处于安全姿态，急停可用，电机周围没有障碍物；
2. 检查 `device/config/device.yaml` 中的 IMU 串口和波特率；
3. 检查 `MotorList/config/` 中的电机数量、CAN 配置和机器人型号；
4. 如需单独归零，先运行测试程序确认通信，再执行归零程序；
5. 确认手柄拨杆处于安全位置后，再启动 `main`。

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

交付脚本会递归复制源码和运行资源，并保持相对路径；任意层级的 `build/`、`build-*/`、`.git/`、`.vscode/` 和 `Publisher/` 会被排除。哪些源码参与编译仍由原工程 `CMakeLists.txt` 决定，新增或嵌套的功能包需要在原工程中正确注册。

因此，新增一个 LED 功能包通常不需要修改 `package_release.sh`。只需：

1. 创建 `Feature_led/include` 和 `Feature_led/src`；
2. 在现有主程序或状态机中注册并调用 LED 类；
3. 如有配置或资源，放入 `Feature_led/config` 或 `Feature_led/resources`；
4. 在原工程 CMake 中添加所需源码和头文件搜索路径，再重新生成包，运行 `autobuild.sh` 后检查 `build/` 中的程序。

如果 LED 使用独立动态库，可将运行时 `.so` 放在 `Feature_led/lib/`，脚本会递归收集任意层级 `lib/` 下的 `.so` 和 `.so.*` 到交付包的 `lib/`。

### 4.4 MuJoCo 模型

MuJoCo 主程序读取：

```text
RobotModel/phybot_c2/xml/phybot_c2.xml
```

该 XML 依赖同目录下的 STL 网格文件，因此 `RobotModel/phybot_c2/xml/` 和 `RobotModel/phybot_c2/new_meshes/` 必须保持原有相对目录结构，不能只复制 XML 文件。

## 5. 日志、权限和运行目录

程序必须通过 `run.sh` 启动，或者确保当前工作目录等同于包内的 `build/` 目录。直接从其他目录运行二进制可能导致 `../MotorList/config/...` 等相对路径找不到。

日志和运行过程中生成的数据可能写入对应 RL 模块的 `logs/` 或 `data/` 目录。客户部署时建议给整个包目录保留写权限，或者将日志目录单独映射到可写磁盘。

常用检查命令：

```bash
pwd
find ./build -maxdepth 1 -type f -executable -printf '%f\\n'
df -h .
ls -l /dev/tty* 2>/dev/null
```

## 6. 依赖和硬件要求

- 默认目标系统：Ubuntu 20.04 x86_64；
- 编译工具：GCC/G++ 9、CMake 3.16+、C++17；交付 SDK 需要先在目标机上编译；
- `install_dependencies.sh` 安装 Ubuntu 20.04 编译、OpenGL 和 X11 依赖；
- `check_compatibility.sh --abi-only` 检查库架构及 GLIBC/GLIBCXX/CXXABI 版本要求；
- `build-info.txt` 记录封装目标和构建环境，`compatibility-report.txt` 记录 ABI 检查结果；
- 动态库：已随包放入 `lib/`；
- MuJoCo：使用包内的 MuJoCo 动态库和机器人模型；
- 真机运行：需要正确连接电机、CAN 设备、IMU 和对应手柄；
- 串口权限：当前用户需要有访问 IMU 串口设备的权限；
- 本交付包不包含 Gazebo 和 ROS 运行环境。

Ubuntu 20.04 混合 SDK 的预编译库必须在 Ubuntu 20.04 上使用 GCC 9 生成。在 Ubuntu 22.04/24.04 上封装时，可在原封装命令前添加 `PACKAGE_USE_DOCKER=1`，脚本会使用 Ubuntu 20.04 容器。仅更换编译器或 C++ 标准不能降低现有二进制的 glibc 依赖版本。

默认检查基线为 GLIBC 2.31、GLIBCXX 3.4.28、CXXABI 1.3.12。检测到超出基线的动态库会阻止发布，应替换为适配 Ubuntu 20.04 的库或在该系统重新编译。ABI 检查不代表所有系统依赖、CPU 指令集和硬件已完成验证。

如果显式使用 `TARGET_PLATFORM=native`，包沿用宿主机工具链，不保证 Ubuntu 20.04 兼容性；依赖安装脚本仍仅用于 Ubuntu 20.04。

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

1. 在纯源码模式跳过编译；指定头文件包时，按所选环境分别构建旧 Hipnuc 和新 IMU 静态库；
2. 递归复制源码工程、配置、模型和动态库，过滤旧构建目录；
3. 按源码包优先的规则裁剪头文件包；
4. 生成客户 SDK、编译/清理/运行脚本、版本文件和 README，不交付预编译可执行程序；
5. 全部生成成功后替换同名旧交付包。

交付包会保留可用的 `autobuild.sh` 和 `autoclean.sh`。`autobuild.sh` 使用包内 `CMakeLists.txt` 配置并编译到包内 `build/` 目录；`autoclean.sh` 只清理包内 `build/`。

如果只需要某一个环境，可以指定环境名称：

```bash
./package_release.sh mujoco_sim_mini Model_airplane PHYBOT_C2_SDK_1.1 ./deliveries
```

当前脚本默认交付完整可编译源码工程。如果第六个参数指定头文件包，会生成“源码包 + 头文件包 + 预编译库”的混合 SDK：

`Publisher/` 是独立的内部源码仓库，始终不会复制到客户交付包；依赖它的原工程 `autorun.sh` 也不会进入交付包。

```bash
./package_release.sh all Model_airplane PHYBOT_C2_SDK_1.1 ./deliveries device,RobotStart,ZeroState,RL_deploy_cpg,StateMachine LowPassFilter,Model_airplane_Joystick,MotorList,MujocoInterface,DataPackage
```

第五、六个参数均支持逗号分隔的多层相对目录，例如 `modules/sensors/imu,modules/control/filter`，路径相对于源码工程根目录。第五个参数中的包及其子目录保留完整源码；第六个参数中的包递归保留头文件和运行资源，编译时链接 `prebuilt/<IMU类型>/<环境>/` 下的静态库。未指定的包仍按原逻辑保留完整内容，`none` 表示空列表，不表示删除全部源码。

可选的第七个参数（或 `PROJECT_DIR` 环境变量）可以指定任意层级的源码工程根目录，原来的六参数命令仍可使用。混合 SDK 仍依赖原工程的 `phybot_common`、`phybot_joystick` 及对应环境静态库目标。

本封装脚本不包含 Gazebo 功能。Gazebo 相关源码和 ROS 依赖不会进入客户交付包。

`RL_deploy_cpg_run` 仍是 CPG 的备用实现，默认不会参与编译，以避免和 `RL_deploy_cpg` 产生重复符号。第五个参数只控制交付包是否保留源码，不改变这个 CMake 编译规则。

如果新 RL 包需要通过手柄或状态机触发，仍需在状态机的状态枚举、注册表和控制逻辑中完成业务注册。

## 9. 客户二次开发步骤

### 9.1 新增机器人状态机

1. 在软件包根目录新建状态功能目录，并建立 `include/`、`src/` 目录；需要模型、配置或动作数据时，再建立 `model/`、`config/`、`data/` 目录。
2. 在 `include/` 中创建状态类头文件，在 `src/` 中创建实现文件。
3. 状态类需要提供 `GetDataFromPackage`、`Step`、`SetDataToPackage` 和 `Exit` 接口，分别用于读取公共数据、执行状态逻辑、写回控制结果和退出状态。
4. 编辑 `StateMachine/include/fsmlist.h`，在 `State` 枚举末尾追加新状态，不要调整已有状态的顺序。
5. 编辑 `StateMachine/include/statemachinemanager.h`，引入新状态头文件，并在 `StateRegistry::registerAllStates()` 中注册新状态类和对应枚举。
6. 编辑 `StateMachine/include/statemacine.h`，在 `toString()` 中增加新状态名称。
7. 编辑根目录 `CMakeLists.txt`，把新状态源文件加入 `main` 目标，并把新状态的 `include/` 目录加入 `main` 的头文件搜索路径。
8. 如果测试程序需要调用新状态，也要把相同的源文件和头文件路径加入对应测试目标。

### 9.2 增加航模手柄按键映射

1. 编辑 `Model_airplane_Joystick/include/joystick_int.h`，确认目标按键的 SBUS 通道号和 `xbox_map_t` 数据字段；使用新通道时，增加通道定义和数据字段。
2. 编辑 `Model_airplane_Joystick/src/joystick_int.cpp`，在 `Serial_map_read()` 中读取目标通道，并设置按键松开、按下或多挡开关位置对应的字段值。
3. 在 `Joystick::get_state_change()` 中增加按键到新 `State` 枚举的映射。
4. 检查映射判断顺序。多个按键同时触发时，排在前面的条件优先执行，`ZERO` 等安全状态应放在普通动作状态之前。
5. 一个按键只映射一个状态，避免同一按键同时修改多个 `NextState`。
6. 执行 `./autoclean.sh`，再执行 `./autobuild.sh`，选择 `test` 环境和实际使用的 IMU。
7. 编译完成后执行 `./run.sh joy_test`，逐个操作航模手柄按键，确认状态与映射一致。
8. 手柄测试通过后，重新构建 `realrobot_mini` 或 `mujoco_sim_mini`，运行 `main` 验证完整的状态进入、运行和退出流程。

## 10. 安全说明

纯源码模式会交付完整源码；混合模式仅裁剪指定头文件包，其他源码仍保留。预编译静态库仍可能被逆向分析，交付前请确认源码保留列表符合交付范围。

## 11. 故障排查

检查动态库依赖：

```bash
LD_LIBRARY_PATH=./lib ldd ./build/main
```

常见问题：

### 程序提示动态库 `not found`

确认使用 `run.sh` 启动，并检查：

```bash
LD_LIBRARY_PATH=./lib ldd ./build/main | grep 'not found'
```

如果仍有缺失，通常是交付包不完整，或者 `build/`、`lib/` 被单独移动。

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

## 12. 交付边界

本包用于客户运行、现场配置和重新编译。新增 RL 包、修改状态机、修改电机控制逻辑或更换模型后，应重新执行 `autobuild.sh` 并进行实机或仿真验证。

如需问题定位，请同时提供软件包版本文件、启动命令、终端完整日志、修改过的配置文件名以及硬件连接状态。
