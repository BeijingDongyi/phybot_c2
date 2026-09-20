# 封装清单

| 内容 | 位置 | 说明 |
| --- | --- | --- |
| 完整源码 | `ZeroState/`、`StateMachine/`、`RobotStart/`、`device/` | 按交付要求完整保留 |
| 旧 IMU 封装库 | `lib/Hipnuc/` | Hipnuc 选择时自动链接 |
| 新 IMU 封装库 | `lib/IMU/` | IMU 选择时自动链接 |
| 六种预编译组合 | `prebuilt/<IMU>/<环境>/` | 新旧 IMU × 真机、MuJoCo、测试 |
| 默认启动程序 | `build/` | Hipnuc / realrobot_mini |
| 手柄与应用程序 | `Publisher/` | 无需客户侧源码编译 |
| 模型、配置和资源 | 各 `RL_deploy_*`、`MotorList/`、`RobotModel/` | 完整保留运行所需文件 |
| Ubuntu 20.04 验收记录 | `compatibility/` | ABI、链接、构建和模型加载结果 |

未交付非保留模块的 C/C++/CUDA/Python 实现源码、Git 历史、对象文件及构建缓存。
