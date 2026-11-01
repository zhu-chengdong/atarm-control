# 代码来源与公开状态核查

本页记录 2026-11-01 对仓库主分支的来源核查。仓库目前公开可访问；公开状态、文件复制记录和 Git 提交者身份均不单独证明版权归属或再分发授权。

## 来源范围

| 路径 | 已有证据与边界 |
| --- | --- |
| [reference/original_468fcfd39fa6/](../reference/original_468fcfd39fa6/) | 据 [来源与版本](PROVENANCE.md)，这是王天元原始代码的可用快照。原压缩包给出的前缀不是完整上游 Git 历史；原始代码的公开与再分发授权待相关权利人确认。 |
| [arm_controller_lib.py](../atarm_controller/arm_controller_lib.py)、[arm_controller_node.py](../atarm_controller/arm_controller_node.py) | 与原始快照对应文件不同，原有控制与后续扩展存在于当前文件中。应结合原始快照、开发记录和实际贡献确认来源及授权。 |
| [end_effector_controller.py](../atarm_controller/end_effector_controller.py)、[end_effector_controller.launch.py](../launch/end_effector_controller.launch.py) | 与原始快照对应文件的 Git 内容哈希一致；公开及再分发授权待确认。 |
| [cartesian_control.py](../atarm_controller/cartesian_control.py)、[cartesian_math.py](../atarm_controller/cartesian_math.py)、[cartesian_simulation.py](../atarm_controller/cartesian_simulation.py)、[cartesian_jobs.py](../atarm_controller/cartesian_jobs.py) | 三维控制、数值计算、模拟和任务层的后续开发模块。它们与其他项目模块存在依赖，具体贡献者、代码来源及许可仍需核对。 |
| [external_vision.py](../atarm_controller/external_vision.py)、[ros_pose_bridge.py](../atarm_controller/ros_pose_bridge.py)、[visual_servo.py](../atarm_controller/visual_servo.py)、[online_jacobian.py](../atarm_controller/online_jacobian.py) | 涉及位姿接入、视觉伺服和在线雅可比等功能。逐文件贡献和授权应结合开发记录确认。 |
| [native/a1_serial.cpp](../native/a1_serial.cpp)、[serial_adapter.py](../atarm_controller/serial_adapter.py) | 项目接口使用 Unitree SDK；适配器自身的来源和许可应与 vendor 组件分别确认。 |
| [tests/original_node_snapshot.py](../tests/original_node_snapshot.py) | 历史节点快照，其内容哈希不同于 reference 目录下原始节点，也不同于当前节点；不能仅根据文件名认定版本或作者。 |
| [config/](../config/)、[docs/](.)、[evidence/](../evidence/) | 包含配置、技术说明与离线验证记录；其中项目参数和研究材料的公开范围需结合项目约定确认。 |
| [native/vendor/](../native/vendor/) 与 `test/test_*.py` 中已标注的模板 | 已识别的第三方许可见 [第三方组件与许可](../THIRD_PARTY_NOTICES.md)。 |

[provenance/source_map.json](../provenance/source_map.json) 与版本来源文件记录复制路径和哈希，有助于追踪材料，但不构成作者或许可证明。GitHub 初始导入将不同来源的文件集中提交，提交者姓名也不能用来判断逐文件作者。

## 当前许可状态

项目根目录尚无统一许可证；[package.xml](../package.xml) 和 [setup.py](../setup.py) 中的许可仍为 TODO。第三方组件按其各自声明处理，不自动覆盖其他文件。原始代码、衍生代码以及后续开发模块的权利和授权尚待确认；在确认之前，勿将本仓库整体描述为已按某一开源许可证发布。

## 验证状态

本仓库记录了模拟和伪串口验证。按照 [验证与剩余事项](VALIDATION.md)，实际机械臂尚未接入本版三维控制；模拟结果不代表实机精度。
