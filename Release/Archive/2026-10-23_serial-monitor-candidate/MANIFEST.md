# Serial 通用串口监视本地候选

日期：2026-10-23。候选：serial-monitor-20261023。状态：软件构建完成，实板待验证；独立分支，未合入develop、未推送或打tag。

| 项目 | 实际记录 |
|---|---|
| 分支 / 生产提交 | codex/serial-monitor / `d6515d7551ae7600e23b17da7320c0c54b4ca45a` |
| develop基线 | `5e1b2e266f899de5f4915615bf9f41a5c30d7edc` |
| Rebuild All | 2026-10-23T09:57:10.7083548+08:00 — 2026-10-23T09:57:52.9121560+08:00，退出码0，0 Error(s), 0 Warning(s)，日志00:00:40 |
| 工具链 / 工程 | ARMCC5.06 u5 build528 / MDK-ARM/formal_framework.uvprojx / Target 1 |
| Code / RO / RW / ZI | 341196 / 111084 / 1988 / 109844 B |
| 主SRAM | 111832 B、余19240 B；基线+136 B |
| Flash装载 | 452408 B；基线+1132 B，压缩装载口径 |
| HEX范围 | [0x08000000, 0x0806E738)；不占配置sector11 |
| 生产输入 | 1023项，构建前后/归档SHA不变；清单SHA `068971c3ea9caf3ea7ebe46100517e24d305ace5aeff8b7142aa6bf0804ada87` |
| 源码ZIP | `4c83f69eb04128cc93415a68cf2b05dec67a810dacd64a8d7ce86cccda5b182a`；生产输入逐项SHA与清单一致 |

## 同次产物SHA-256

| 文件 | SHA-256 |
|---|---|
| formal_framework.hex | `31e851c1e3e17eab7e70c4567ec336ac65c001d77445cb7da22d1bbcb5f96cd3` |
| formal_framework.axf | `9c70860e3dae07f8996957355d61184b9833020725119932e3b01e22742e9c09` |
| formal_framework.htm | `9c3076e0a8c7ea5dd482095fa741fc6d0c9435190751bf4bd4579c0e713b50c9` |
| formal_framework.map | `061c6f970a0bc2ba2a625a73e564f587204b6e81fe5e0f80565fe612fe386a10` |
| keil-rebuild.log | `af9e1c31137d0163bcdd31c6604b4e6ff6f43ac48d5ac098ef595fc52d183354` |

Latest与日期归档保存HEX/AXF/MAP和build.log（对应keil-rebuild.log）；完整HTM、源码输入ZIP、输入清单及身份JSON在两个工作区build/serial_monitor_20261023/artifacts中。

完整显示UART6配置与九项独立累计统计，Serial名称保持，Clear stats仅改变显示基线；Frames复用有界捕获。H1/B1及现有通信协议、编码/调度/实验事务保持。没有新增FreeRTOS任务、业务动态分配、队列、大缓存或改变任务栈；页面继续使用固定LVGL池，新增端口标题标签的池内占用待实屏采样。代码审查及构建完成，不代替实屏、真实收发、栈水位/性能和长稳验收；未新增或运行host测试，M5/M6保留待验。
