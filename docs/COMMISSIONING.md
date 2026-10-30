# R1 参数与现场启动准备

当前审阅模板为 [site.template.json](../config/commissioning/source_audit_20261030_r1/site.template.json) 和 [session.template.json](../config/commissioning/source_audit_20261030_r1/session.template.json)。原 v4 空模板另行保留。R1 只回填 15 个有依据的 SDK/软件字段，26 组判断与 33 条来源锚点见 [parameter_evidence.json](../config/commissioning/source_audit_20261030_r1/parameter_evidence.json)。

已回填值包括两路 8N1/无流控配置、固定 A1 SDK 减速比 9.1、两项视觉话题和已实现停止策略。它们各自的适用范围见证据，不能推导当前安装已核实。

历史 ID、26 mm 卷筒半径、4 M SDK 默认速率及旧增益仅保存在 [historical_defaults.reference.json](../config/commissioning/source_audit_20261030_r1/historical_defaults.reference.json)。展开方向 −1/+1、增益倍率 3/1、弯曲上限实现 60°/注释 30° 存在冲突，执行模板继续留空。剩余项目见[现场确认事项](reports/commissioning_parameters_20261030_r1/剩余现场确认事项.md)。

R1 已同步到 Mark 工作站独立发布目录 /home/zcd/codex-runs/atarm-commissioning-entry-20261030-01a1831d/releases/source_audit_20261030_r1，见[本次同步回执](reports/commissioning_parameters_sync_20261030_r1/同步与预检回执.md)。

检查只能使用 check。使用新目录复制 site.template.json 为 site.json，并复制其 session_file 所指向的 session.template.json 后，可运行以下命令；变量由操作者指向实际仓库、配置和构建目录。

```bash
python3 -B tools/commissioning_container.py check --project "$ARM_PROJECT" --site-dir "$SITE_DIR" --native-dir "$NATIVE_DIR"
```

宿主 check 为固定镜像配置无网络、无设备映射和只读挂载，不加载原生库或打开串口。R1 原样和仅在测试副本打开确认标记时均返回 2、ready_for_run=false，97 项参数路径问题继续阻止通过；历史参考文件不能作为 site 配置。冲突以未确定值拦截，检查器不会自动裁决历史记录。

到实验室后按[实机流程](reports/hardware_commissioning_20261030_v1/实机测试流程.md)逐级完成供电/支撑、基本动作、映射和参考、视觉/时钟、初始 J 和小幅三维到位，再决定是否进入实际 run。真实启动还要求显式连接参数、两路白名单身份、完整参数、能力库哈希和现场确认；本轮未执行。
