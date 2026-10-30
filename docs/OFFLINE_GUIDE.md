# 离线复现

纯 Python 模拟和四组单元检查使用 README 的命令即可，不连接设备。Python 3.10+ 为源码语法下限；当前整理副本的具体运行版本和结果见 evidence/repository_preparation_20261030。

Linux x86_64 原生构建需要 CMake、C++17 编译器及仓库中固定的 Unitree SDK。构建目录必须是新目录；物理 UART 能力默认 OFF。

```bash
python3 -B tools/build_commissioning_native.py --mode offline --output outputs/native_offline_01
```

以下为已有 ROS 2 Jazzy 环境中的 PTY 整链路复现。原生矩阵从 native/build 读取默认离线库，先按其预期布局构建。

```bash
cmake -S native -B native/build -DATARM_ENABLE_PHYSICAL_UART=OFF
cmake --build native/build --target a1_serial_adapter -j2
python3 -B tests/ros_fd_matrix.py --case all --out outputs/ros_pty_01
```

测试程序会创建两路自己的 PTY 和独立位姿源，通过 rclpy/DDS 运行控制节点、客户端和真实 A1 编解码。它不发现 /dev/ttyUSB 等实物设备。

Mark 工作站已缓存 ROS 镜像 ros@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617。已有完整容器隔离与检查命令见[历史验证记录](reports/commissioning_entry_20261030_v4/离线验证记录.md)。本轮上传不重新安装 ROS 或构建/连接硬件。

仓库没有声称 colcon 安装或 ros2 run 打包已经验证；使用已验证的模块/测试入口。保留的旧报告含当时绝对路径，当前阅读入口和仓库相对路径优先。
