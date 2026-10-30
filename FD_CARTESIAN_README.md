# 三维控制的外部视觉与A1描述符接口

2026-10-30 v3，基于v2完整复制后独立开发。新接口使用真实A1协议的原生收发层，已在Mark工作站固定ROS2容器中通过18类外部视觉/伪串口用例；40项本地接口与回归检查通过。

完整接口、复现命令、测试范围和仍需现场确认的事项统一见[执行器与视觉接入验证报告](docs/reports/cartesian_fd_20261030_v3/执行器与视觉接入验证报告.md)。

新增执行器适配为`actuator_io.py`，视觉变换为`external_vision.py`，任务管理为`cartesian_fd_runtime.py`，ROS节点为`cartesian_fd_node.py`。原模拟节点、原硬件节点、原生库和v1/v2保留。只有本副本的`cartesian_control.py`扩展了显式FD接口契约，`setup.py`增加入口。

当前CLI只接受调用者提供的PTY描述符，原生库未启用物理UART。`config/offline_fd_validation.json`全部为合成参数，不能作为实机标定。没有连接真实机械臂、相机或底盘，没有修改PPT。
