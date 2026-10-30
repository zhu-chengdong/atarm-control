# 离线三维目标控制试验

本目录由2026-10-28的ROS接入副本复制，复制来的文件保持不变。新增入口为 `atarm_controller.cartesian_control.CartesianController.move_position(PositionGoal)`，当前只提供合成机构I/O，不可直接上机。

完整说明、结果和必须实机确认的项目统一维护在[实现与验证报告](docs/reports/cartesian_sim_20261030_v1/实现与验证报告.md)。来源清单见 `SOURCE_PROVENANCE.json`。

在本目录使用现有Python运行，不需要安装或启动ROS。

```powershell
python -B tools/run_cartesian_sim.py --target 0.3124208 0.03748 0.1949592 --strategy alternating --output <新的结果目录>
python -B tests/test_cartesian_offline.py <新的验证目录>
```

`alternating` 为同一三维目标下分时执行弯曲和伸缩小步；`coordinated` 为模拟中的同步分配。输出误差来自模拟真值，不代表机械臂实际精度。原ROS节点尚未接入此新增模块。
