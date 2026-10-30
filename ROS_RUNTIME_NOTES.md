# ROS2位姿接入与异步取消联调

2026-10-28。新副本work/atarm_controller_ros_integration_20261028。原代码及此前20个副本共21版本哈希不变；已有生产模块只改arm_controller_node.py，控制核心、雅可比、运动保护和原生串口源码/二进制未改。

## 实测结果

8项真实ROS联调全部通过，另17项纯适配层与并发检查通过。完整展开—弯曲—归中—收回完成，运动期间收到465帧有效源位姿，最大接收间隔20.09毫秒。展开、弯曲、收回均可以在运动期间通过ROS服务取消；从客户端发出取消请求到原生串口首个确认停机请求及四电机全部有效回执的耗时如下表。计时逐一关联停止回执的transport_frame_id，普通零力矩被动轮询不算停机。

| 用例 | 最终状态 | 运动中收到位姿帧 | 最大接收间隔/ms | 首个停止请求/ms | 四电机停止回复/ms |
|---|---|---:|---:|---:|---:|
| complete | SUCCEEDED | 465 | 20.09 | — | — |
| cancel_deploy | CANCELLED | 24 | 20.02 | 2.88 | 3.87 |
| cancel_bend | CANCELLED | 181 | 19.98 | 3.46 | 5.28 |
| cancel_retract | CANCELLED | 295 | 20.07 | 2.96 | 4.09 |
| freeze | FAILED | 24 | 10.26 | 150.1 | 152.36 |
| tracking_lost | FAILED | 25 | 10.07 | 12.38 | 13.52 |
| map_reset | FAILED | 25 | 19.98 | 11.1 | 12.1 |
| wrong_frame | FAILED | 24 | 20.03 | 11.23 | 12.09 |

后三类来源错误和位姿冻结均进入失败状态、四电机请求零力矩，没有误报流程完成。取消后直接重跑被原控制核心拦住，重叠任务在ROS受理层返回BUSY。原生请求和回复分别独立复算CRC，7个停止用例共28份停止回执验证有效。这里是伪串口软件命令/回复延迟，不是机械臂物理制动时间。

## 修改内容

新增ros_pose_bridge.py，将geometry_msgs/PoseStamped与同一捕获时间的std_msgs/String源状态配对。源状态必须包含stamp_ns、源frame_id、tracking_ok、map_epoch、metric及child_frame。缺失配对不生成观测；重复源帧不冒充新帧；时间回退、未来/陈旧时间、非米制尺度、几何非法、跟踪丢失和地图/参考系变化均拒绝。ROS捕获时间经同钟域偏移转换到控制器单调时钟，没有把消息到达时间直接当捕获时间。输入必须已转换到目标使用的同一坐标系，不自动猜测TF或单目尺度。源错误锁存后需要明确重建位姿源会话，不自动恢复运动。

新增ros_motion_worker.py。只有一个线程负责动作、空闲电机轮询、停止和关闭串口。ROS回调只提交一个任务或设置取消标志；状态发布读取精简缓存，不读串口，不反复复制完整历史。运动等待能被取消事件打断，正常结束与取消在短锁内提交，停止I/O放在锁外。任务异常和节点退出也发送四电机零力矩请求。完整控制历史不进入高频ROS状态消息。

新增ros_control_config.py和仅供模拟的JSON配置。目标、帧名/地图会话、局部雅可比、相机与电机限值、恢复试运动及暂停预算通过control_config_path显式载入。没有自动生成真实机械臂标定，也没有启用自动打开真实串口。

ROS接口保留run_camera_cycle、deploy、retract、恢复及取消服务名。Trigger响应success现在表示请求已受理，message是含accepted、job_id、complete=false的JSON；它不表示机械运动完成。最终结果从atarm/arm_status/controller_state的ros_job读取，运动成功须state=SUCCEEDED且complete=true。恢复成功的complete仍为false，表示允许继续任务。

默认位姿话题为atarm/camera/pose，配对源状态为atarm/camera/source_status。真实SLAM的输出需先满足上述位姿、米制尺度、帧名、时间戳和跟踪状态契约；本轮发布器仅来自独立模拟物理状态。

## 真实ROS与对照证据

本机WSL没有安装ROS2。此次SSH到Mark工作站，在[官方ROS2镜像](https://hub.docker.com/_/ros/tags?name=jazzy)中使用真实rclpy、DDS、消息和SingleThreadedExecutor运行，位姿发布器PID与控制器不同。镜像固定为ros@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617。容器无外部网络、不挂载宿主设备、丢弃额外权限，以工作站普通用户写专属结果目录。没有在宿主系统安装ROS、没有连接机械臂。测试后容器已退出；镜像保留供复现。

[ROS官方回调组说明](https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/nodes/Working-with-nodes/Using-callback-groups.rst)解释了默认互斥回调组的串行约束。本轮用原服务回调函数快照和350毫秒受控阻塞操作，在真实ROS中复现取消回调延迟320.43毫秒；期间发布35帧、处理0帧。它隔离验证原回调阻塞，不是原机械臂流程的制动性能对照。原真实流程的探索性对照在初始位姿确认阶段就失败，证据另保留。

最初还暴露了新状态发布复制完整历史导致的负担，以及测试诊断列表边写边增长的问题，已改为精简状态及冻结诊断快照；早期失败日志和线程栈均保留。最终ROS矩阵为ros_matrix03；ros_matrix01/02属于之前实现或计时核对版本，不能与最终结果混称同一版本。

未运行colcon安装测试，也未接真实SLAM输出、真实相机或真机串口。之前98项核心模拟保护证据保留，本轮未重跑整个旧套件；相关核心源代码和原生库逐字节未变。当前交付证明真实ROS通信与单线程调度下的模拟闭环和取消可用。

## 交付文件

核心证据为ros_matrix03/results.json及各case.json、adapter_worker_checks01/results.json、original_executor_red01/results.json、ros_comparison.csv、execution_scope.json、remote_environment.json、changes_from_confirm_budget.patch及delivery_manifest.json。remote_run.py记录隔离容器运行方式，输入只打包代码、原生库、测试及配置目录，未上传资料指南或凭据。
