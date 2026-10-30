# 验证证据与剩余工作

| 阶段 | 已有实际结果 | 本仓库证据 |
|---|---|---|
| v1 纯三维控制 | 22 个核心检查及交替/协调模拟 | evidence/cartesian_sim_20261030_v1/ |
| v2 ROS 模拟 | 8 类 DDS 场景 | evidence/cartesian_ros_sim_20261030_v2/ros05/results.json |
| v4 单元与 Linux | 14+12+22+6 单元检查及 4 项 Linux/PTy 检查，共 58 项 | evidence/commissioning_entry_20261030_v4/validation01/ |
| v4 ROS/A1/PTy | 18 类完整场景全部通过 | evidence/commissioning_entry_20261030_v4/validation01/ros_matrix/ |
| R1 工作站复查 | 原样、仅开放确认标记、误用历史文件均被拒绝 | evidence/commissioning_parameters_sync_20261030_r1/ |
| 本次仓库整理 | 复制后四组检查和两种纯模拟策略 | evidence/repository_preparation_20261030/ |

历史结果与本轮复验分别保存。完整串口十六进制、逐帧 DDS/事务日志、大型原始数据继续保留在本机/工作站原实验目录；仓库保存汇总、各场景 case.json 及来源哈希，不将省略的原始证据写成已经上传。

必须现场核实的内容为供电、展开/保持/收回可靠性、独立停止与承重支撑、USB/串口/ID 与安装方向、零位/行程/半径、协议增益和限制、视觉米制尺度/外参/捕获时效、初始三维 J 及独立参照下的到位精度/重复性。展开与弯曲索的张力耦合也仍需实测。
