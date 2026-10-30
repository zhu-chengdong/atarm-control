# ATArm Control

农业张拉整体机械臂的私有研发仓库。当前代码为 2026-10-30 **v4 + 参数核查 R1**，包括三维位置闭环、伸缩/弯曲协调、在线雅可比更新、外部视觉桥接、A1 四通道执行器接口及默认禁用的实机启动准备。

当前已完成模拟与伪串口验证，真实机械臂尚未接入本版三维控制。默认现场参数继续留空，历史方向/增益冲突不会自动取值。

## 阅读入口

| 内容 | 文档 |
|---|---|
| 硬件与软件结构、原始控制及当前缺口 | [系统与当前状态](docs/SYSTEM.md) |
| 逆运动学、视觉闭环、在线 J 与伸缩/弯曲分配 | [控制方法](docs/CONTROL.md) |
| 目标、回执、状态、参考确认及视觉消息契约 | [ROS 接口](docs/ROS_INTERFACES.md) |
| 无设备模拟、检查和 Linux/ROS 复现 | [离线复现](docs/OFFLINE_GUIDE.md) |
| R1 参数、冲突、工作站部署与到场顺序 | [实机准备](docs/COMMISSIONING.md) |
| 哪些结果已经验证、哪些仍须实测 | [验证与剩余事项](docs/VALIDATION.md) |
| 原始来源、版本、许可状态及未上传内容 | [来源与版本](docs/PROVENANCE.md) |

## 先运行纯离线示例

在仓库根目录使用 Python 3.10+，无需 ROS、SDK 加载或串口。输出目录必须尚不存在。

```bash
python -B tools/run_cartesian_sim.py --strategy alternating --output outputs/demo_alternating
python -B tools/run_cartesian_sim.py --strategy coordinated --output outputs/demo_coordinated
```

运行本仓库的四组本地检查及上述两种模拟策略，使用新的输出目录。

```bash
python -B tools/verify_repository.py --out outputs/repository_check_01
```

## 目录

```text
atarm_controller/       控制核心、任务层、ROS 节点、视觉及执行器接口
config/                纯模拟配置、v4 空模板和 R1 参数审阅模板
native/                C++ A1 协议适配器及带原许可的固定 Unitree SDK
tests/                 数值、接口、ROS 和 PTY 检查
tools/                 模拟、独立原生构建、宿主 check/run、仓库验证
docs/                  当前技术说明与分阶段历史报告
evidence/              历史机器可读结果及本次整理后复验
reference/             原始源码快照，保留来源前缀 468fcfd39fa6
provenance/            来源逐文件哈希、旧版清单与排除项
```

仓库根目录仍是 ROS 2 的 `atarm_controller` 包。当前复现采用 Python 模块启动；尚未验证 colcon 安装打包。旧 `arm_controller`/`end_effector_controller` 和 `reference/` 中入口按历史代码保留，不属于默认离线命令。
