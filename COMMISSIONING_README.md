# 实机启动入口与待确认参数模板

2026-10-30 v4，保留 v3 后独立派生。默认不连接任何实机。

- [启动与切换说明](docs/reports/commissioning_entry_20261030_v4/启动与切换说明.md)
- [现场参数确认表](docs/reports/commissioning_entry_20261030_v4/现场参数确认表.md)
- [离线验证记录](docs/reports/commissioning_entry_20261030_v4/离线验证记录.md)
- [部署模板](config/commissioning/site.template.json)与[标定模板](config/commissioning/session.template.json)

Mark 工作站宿主使用 `tools/commissioning_container.py check` 做不映射设备的配置检查。`run` 必须显式提供 `--connect-hardware`、完整实机参数、现场确认和新日志目录，才会准备两路白名单 UART。

保留 `cartesian_fd_node --offline-pty`。实机能力库与原离线库分开构建、分开保存；默认旧库不变。物理 UART 仅具备软件准备，实际设备、供电、标定与三维到位仍待现场验证，本轮未运行实机 run 入口。
