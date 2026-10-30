# v4 参数审阅版 R1

2026-10-30。原空模板和控制代码保留，本目录是依据原源码与历史资料新增的参数补充版本。

- [site.template.json](site.template.json)与[session.template.json](session.template.json)为本次审阅模板，新增15个具有明确SDK/软件证据的字段。未知安装参数仍为空，实机开关及确认保持关闭。
- [historical_defaults.reference.json](historical_defaults.reference.json)保存历史端口、ID、方向、半径、波特率、增益/单位换算、几何和冲突，仅供人工复核，不是启动配置，不会自动合并。
- [parameter_evidence.json](parameter_evidence.json)逐项记录26组判断、33条来源锚点、行号与哈希。当前实物没有新增已验证值。
- [核查与回填报告](../../../docs/reports/commissioning_parameters_20261030_r1/参数核查与回填报告.md)列出已回填值和来源范围。
- [剩余现场事项](../../../docs/reports/commissioning_parameters_20261030_r1/剩余现场确认事项.md)区分问Mark、现场核对与必须实测。

本目录留在本地v4工作副本，尚未复制到工作站。到场使用时将本目录作为复制模板的来源，保持 `session_file` 与现场副本中的文件名一致，继续使用v4的check/run入口。不要直接运行原始源码读取参数，也不要将历史参考文件当作site配置。
