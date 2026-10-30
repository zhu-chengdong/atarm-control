# 三维位置控制的ROS2模拟节点接入与联调

2026年10月30日完成。沿用Mark工作站 `elecpc-hppc` 既有固定ROS2 Jazzy容器，未在用户电脑安装ROS或下载镜像。原三维离线版本和历史控制版本全部保留，PPT未修改。

## 已接入的节点

新增 `cartesian_sim_node` 包装上一版 `CartesianController.move_position`。ROS回调只受理任务或取消，独立 `cartesian-sim-owner` 线程执行运动。20Hz状态回调只读取精简缓存，不复制完整控制历史，也不访问电机。主节点明确使用 `SingleThreadedExecutor`，长动作不会占住回调。

原 `arm_controller_node.py`、原电机/串口适配器及三维控制算法未改。继承副本中只有 `setup.py` 增加模拟节点入口；新增模块与测试均在v2独立副本。当前没有接入真实机械臂或外部SLAM位姿话题。

## ROS接口

| 相对名称 | 类型 | 作用 |
|---|---|---|
| `atarm/cartesian_sim/target` | `std_msgs/String` | JSON三维目标 |
| `atarm/cartesian_sim/ack` | `std_msgs/String` | 按request_id关联的受理/拒绝回执 |
| `atarm/cartesian_sim/state` | `std_msgs/String` | 20Hz任务、参考系和合成观测状态 |
| `atarm/cartesian_sim/cancel` | `std_srvs/Trigger` | 请求取消当前任务 |
| `atarm/cartesian_sim/reset` | `std_srvs/Trigger` | 空闲时显式重置模拟机构，更新地图会话 |

受理成功只表示任务已接受，`complete`保持false。最终结果由状态中的 `job.state`、`job.success` 和 `job.complete` 给出。重复ID、忙碌请求、非法数字、错误参考系以及未显式复位的故障后请求均被拒绝。重置后的旧地图目标被拒绝。

目标格式如下。

```json
{"request_id":"goal_001","position_m":[0.3124208,0.03748,0.1949592],"parent_frame":"sim_base","child_frame":"camera_optical","map_epoch":"sim_map_0"}
```

相机和电机来自节点进程内的独立合成机构。DDS实际承载目标、回执、状态、取消和重置通信；没有把内部合成反馈说成真实SLAM输入。`capture_s`属于合成单调时钟，`published_ros_stamp_ns`仅为ROS状态发布时间，未冒充相机曝光时间。

## 实际验证

6项本地任务/并发检查和上一版22项数值控制回归通过。最终 `ros05` 在实际rclpy、DDS及独立控制节点/客户端进程中完成8类用例。各节点均正常退出，工作线程已结束，未加载原生串口或机械臂控制模块。

| 用例 | 结果 | 独立模拟真值误差/mm | 用例内RUNNING状态消息数 |
|---|---|---:|---:|
| complete | SUCCEEDED | 0.205 | 34 |
| coordinated | SUCCEEDED | 0.200 | 10 |
| cancel | CANCELLED | — | 2 |
| tracking_lost | FAULT | — | 2 |
| frozen_pose | FAULT | — | 3 |
| map_reset | FAULT | — | 2 |
| unreachable | FAULT | — | 43 |
| shutdown | CLEAN_SHUTDOWN | — | 2 |

`complete`用例还验证了重复目标、重叠目标、运行中重置拒绝、NaN拒绝回执、显式重置、旧地图拒绝及重置后再次到位。该用例包含两次成功运动，状态消息数不是单次运动的统计。

取消用例从客户端发出服务请求至模拟停止函数执行约0.483毫秒，只是本次软件/RPC链路结果，不是实际电机、机械制动或承重保持的时间。故障与取消都未报告任务完成；退出用例确认模拟停止、工作线程结束和进程返回码为0。

容器使用固定镜像 `ros@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617`，无外部网络、无设备映射、只读源码和根文件系统，丢弃额外能力，只写任务专属结果目录。8个上传文件共约16KB压缩包，不含指南、凭据、原始录包或硬件驱动。容器已停止并清理，没有修改工作站系统软件。

## 保留的调试证据

`ros01`保留测试子进程覆盖PYTHONPATH而丢失ROS环境的启动失败。`ros02_startup_check`已能运动到位，但发现非法数字请求的拒绝回执未保留请求编号；现已修复关联。`ros03`暴露对快速故障也要求至少3条20Hz运行状态的错误测试条件，并在退出清理中重复发送中断信号。测试现按完整运动和快速退出分别检查，退出信号只发送一次。`ros04`及最终 `ros05` 的8项全部通过，最终源码与上传哈希按 `ros05` 核对。

## 后续工作

软件接入已经推进到ROS目标/状态/取消接口。外部SLAM位姿桥接到三维控制器、真实执行器适配和相应消息级模拟还可继续离线开发，当前没有声称这些已接入。

实机仍需确认新供电、基本运动、四路电机对应与方向、行程和张力约束、参考坐标及初始雅可比、三个方向的独立性、实际视觉时效，以及独立参照下的到位精度与机械停止/保持方式。沿用Mark已明确的优先级，先验供电与基本动作，再推进实机位置控制。

## 入口与证据

在已有ROS2 Jazzy环境中，可从开发副本用 `python3 -m atarm_controller.cartesian_sim_node` 启动。`setup.py`已增加 `cartesian_sim` 入口，但没有运行colcon安装验证，不能据此声称已验证 `ros2 run` 安装部署。

开发副本为 `01_Source/development/atarm_controller_cartesian_ros_sim_20261030_v2`。结果位于 `04_Test_Results/cartesian_ros_sim_20261030_v2/ros05`，每例保存DDS状态、回执、进程日志与退出核查。`ros05_environment.json`保存实际容器设置，`payload_manifest.json`保存限定上传清单。

原始控制、归档和v1开发副本共1284份受保护文件SHA-256全部一致。详见 `preservation_verification.json`、`ROS_SOURCE_PROVENANCE.json` 和 `delivery_manifest.json`。
