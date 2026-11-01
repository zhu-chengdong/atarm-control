# 第三方组件与许可说明

本文件记录已识别的第三方组件，不为本项目其余代码授予许可，也不代表依赖清单已经完整。

## 仓库内包含的组件

### Unitree actuator SDK

- 范围：`native/vendor/include/` 和 `native/vendor/lib/libUnitreeMotorSDK_Linux64.so`。
- 上游：[Unitree actuator SDK](https://github.com/unitreerobotics/unitree_actuator_sdk)。
- 许可：BSD 3-Clause；版权声明、许可条件和免责声明保存在 [native/vendor/LICENSE](native/vendor/LICENSE)。
- 二进制再分发时应随附该版权声明、许可条件及免责声明；不得未经许可使用权利人或贡献者名称为衍生产品背书。
- `native/a1_serial.cpp`、`native/a1_serial.h` 及项目接口不属于上述 vendor 目录；其具体来源和许可需分别确认。

### ROS 测试模板

下列文件及其在 `reference/original_468fcfd39fa6/test/` 中的对应快照包含 Open Source Robotics Foundation, Inc. 版权声明，文件头注明 Apache License 2.0：

- `test/test_copyright.py`
- `test/test_flake8.py`
- `test/test_pep257.py`

许可文本见 [licenses/Apache-2.0.txt](licenses/Apache-2.0.txt)。原文件头中的版权和许可声明予以保留。本说明不将 Apache 2.0 扩展到其他项目文件。

## 外部依赖

- `atarm_controller/end_effector_controller.py` 引用 `dynamixel_sdk`；[ROBOTIS Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) 上游仓库标示 Apache 2.0。本仓库未在上述 vendor 路径中包含该 SDK，实际部署版本应另行核对。
- ROS 2 相关模块在 `package.xml` 中声明；运行环境的 ROS 发行版、各包版本和许可需要按实际安装核对。
- Python 标准库、setuptools 和测试工具等依赖的版本与许可应在打包或正式发布时一并核对。

如发现未列出的第三方源码、二进制或其他材料，应保留原声明并补充其来源和许可。
