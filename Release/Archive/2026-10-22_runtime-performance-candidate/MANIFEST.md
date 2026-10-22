# 运行资源统计与耗时收敛本地候选

日期：2026-10-22。候选：runtime-performance-20261022。状态：软件构建完成，实板待验证；本地独立分支，未合入develop、未推送或打tag。

| 项目 | 实际记录 |
|---|---|
| 分支 / 生产提交 | codex/fix-runtime-performance / `02b352482b02c81b92eac8c0a44cf7705f2fe015` |
| develop基线 | `4d1886b0f7b4ada7dd93808e5767de7c8f0a8dfc`；已包含姿态仪灰显修复 |
| 构建 | 2026-10-22T11:10:25.3503948+08:00 — 2026-10-22T11:11:03.8615025+08:00；Rebuild All，退出码0，0 Error(s), 0 Warning(s)，日志36 s |
| 工程 / 工具链 | MDK-ARM/formal_framework.uvprojx / Target 1 / ARMCC5.06 u5 build528 / DFP2.17.0 |
| Code / RO / RW / ZI | 339604 / 111008 / 1980 / 109716 B |
| 主SRAM | 使用111696 B、空余19376 B；相比灰显版增加8 B |
| Flash装载 | 450740 B；相比灰显版增加192 B，按链接器压缩装载口径 |
| HEX数据范围 | [0x8000000, 0x806e0b4)；450740 B；逐条校验通过，不占配置sector11 |
| 输入 | 1021项构建前后和归档时原始SHA一致，生产提交内容经换行归一化逐项一致，ZIP逐文件SHA一致 |
| 输入清单SHA-256 | `bc1a739e89c65d6c8149da88c5c44b990e975cfce2f67747e9a6d67e314e04f7` |
| 源码ZIP SHA-256 | `3ba85012c32de2701cc19b672f7df4d22c8abc8d8754899f572a3b7f9acd7c89` |

## 同次产物 SHA-256

| 文件 | SHA-256 |
|---|---|
| formal_framework.hex | `0e46fca1dc42473ea5219052f492e19c01bf9ba9aec4df298f7d7484fce81765` |
| formal_framework.axf | `8f862f539e7f7595324a147b78ad519b7137c18af83b3bd1b233ed9370d1e45d` |
| formal_framework.htm | `1d67d9b422579b31b92dc3abe647dd627629cae1f2336fcd96357ca7ed541b11` |
| formal_framework.map | `142295364fcd2194ba95490a73174de3e7045152bac85243e4671c6bdd5c6653` |
| keil-rebuild.log | `2d5af21cf6dee65776b84b42889c62a497ce59144c43eace4a189dca814dc6a3` |

Latest与日期归档保存同次HEX/AXF/MAP和build.log（即上述keil-rebuild.log）。完整HTM、原始输入清单、生产源码ZIP、身份JSON与原日志保存在本工作区build/runtime_performance_20261022/artifacts/，并复制到主工作区同路径。旧灰显归档保留。

本次缩短离线IMU地址探测、批量写入FSMC像素，修正LVGL采样峰值口径并增加sensor/comm单轮经过时间最大值。未改变任务优先级/周期、堆池、16行绘图缓冲、页面操作、UART6协议与上报规则。未新增任务/堆分配。

独立只读评审未发现本次差异的阻断问题；记录既有IMU重初始化稳定帧计数与连续有效帧规范不符的风险，未在本分支改动该发布路径。未新增/运行host测试，未烧录或完成空载/满载计时、热插拔、串口/RF/CSV及长稳验收。不能据构建声称61.56 ms显示尖峰、203 ms通信间隔或75.21%堆占用已解决。M5/M6及M5-R01继续待验。
