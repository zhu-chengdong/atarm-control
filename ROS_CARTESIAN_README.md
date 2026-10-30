# ROS2三维控制模拟入口

本目录是v1离线三维控制的独立派生副本。新增模拟专用ROS节点、异步任务接口和DDS测试，原硬件控制节点保持不变。当前仍只使用合成相机和电机，未接外部SLAM与物理串口。

正式技术说明与接口维护在[ROS接入与验证报告](docs/reports/cartesian_ros_sim_20261030_v2/ROS接入与验证报告.md)。原v1数值说明作为历史阶段保留。

在已经source ROS2 Jazzy的环境中执行 `python3 -m atarm_controller.cartesian_sim_node`。目标、回执、状态、取消与重置均位于 `atarm/cartesian_sim/` 命名空间。原有硬件节点不是本次入口。
