# ROS 2 接口

当前外部视觉/执行器节点为 [cartesian_fd_node.py](../atarm_controller/cartesian_fd_node.py)。下表为相对 ROS 名称，部署时遵守命名空间和显式 remap。

| 名称 | 类型 | 契约 |
|---|---|---|
| atarm/cartesian/target | std_msgs/String | JSON，包含 request_id、position_m、parent_frame、child_frame、map_epoch |
| atarm/cartesian/ack | std_msgs/String | 受理或拒绝；accepted 不表示完成 |
| atarm/cartesian/state | std_msgs/String | 20 Hz 缓存状态；最终 SUCCEEDED 且 success/complete=true 才表示完成 |
| atarm/cartesian/confirm_reference | std_srvs/Trigger | 提交参考检查，需等待稳定多帧后进入 READY，不归零、不回家 |
| atarm/cartesian/cancel | std_srvs/Trigger | 请求取消；故障/取消后需要检查并显式重新建立会话 |
| atarm/camera/pose | geometry_msgs/PoseStamped | 米制位置、有效姿态、源父帧、原始捕获时间 |
| atarm/camera/source_status | std_msgs/String | 同一 stamp_ns，frame_id、tracking_ok、metric、map_epoch、child_frame |

合成验证目标示例，不适用于未经标定的实机。

```json
{"request_id":"goal_001","position_m":[0.3124208,0.03748,0.1949592],"parent_frame":"control_base","child_frame":"tcp","map_epoch":"validation_map_0"}
```

视觉消息必须按同一源时间戳配对，外参使用 T_base_source · T_source_camera · T_camera_control。保留原始捕获时间，不把发布时间改作曝光时间。非米制、失跟、陈旧/未来帧、回放、错误坐标、地图变化等均拒绝。默认的纯模拟数值阈值与实机安装参数含义应分别查配置，不能据模拟配置补齐 R1。

`cartesian_sim_node` 是另一个纯合成节点，使用 atarm/cartesian_sim 下的 target/ack/state/cancel/reset。FD 节点不提供自动清除实机故障的 reset。详见[历史 v3 接口报告](reports/cartesian_fd_20261030_v3/执行器与视觉接入验证报告.md)。
