# 2026-11-03 项目归档候选

状态：预发布开发归档；触摸故障未定位，M5/M6完整验收及M7长稳未完成，不是V1.0稳定版。

| 项目 | 记录 |
|---|---|
| 归档分支 | `codex/project-archive-20261103`；基于`367ac0507714335dbbed99c730c530e6bb8f9977`，归档改动仅资料、索引和进度记录 |
| 功能来源 | `codex/buzzer-startup-audition`，含Serial通用监视、显示/性能修正及PE5/TIM9一次经典启动音 |
| develop基线 | `5e1b2e266f899de5f4915615bf9f41a5c30d7edc`；本次不合入未完成验收的Serial/启动音候选 |
| 标签 | `v1.0.0-pre.archive.20261103` |
| 工程 / Target | `MDK-ARM/formal_framework.uvprojx` / `Target 1` |
| 工具链 | Keil MDK5 / ARMCC5.06 update5 build528 / STM32F4xx_DFP2.17.0 / C99 |
| 实际Rebuild All | 2026-11-03 17:21:58—17:22:44 UTC+8；退出码0，`0 Error(s), 0 Warning(s)`；日志41 s |
| Code / RO / RW / ZI | 342608 / 111120 / 1996 / 109948 B |
| 主SRAM静态使用 / 余量 | 111944 / 19128 B；包含固定堆池，不等于实时空闲堆或栈水位 |
| Flash实际装载 | 453856 B；压缩装载口径 |
| HEX范围 | `[0x08000000, 0x0806ECE0)`；连续，无重叠，校验通过，不写sector11配置区 |
| 生产输入 | 1027项，构建前后SHA不变；源码ZIP逐项核对一致 |
| HEX SHA-256 | `3355692eaeea435d03429fa18247c645b3d1d5c3bd91da9a7afeee055d918506`，与2026-10-27试听候选一致 |

## 固件获取

[GitHub预发布附件](https://github.com/oran9eLi/Formal_Framework/releases/tag/v1.0.0-pre.archive.20261103)：

- `cns-f407-classic-startup-20261103.hex`：常规候选。
- `cns-f407-classic-startup-20261103.zip`：同次HEX/AXF/MAP/HTM、日志、输入清单、完整生产源码ZIP、build-evidence及启动音许可证说明。
- `cns-f407-gt911-diagnostic-20261103.hex` / `.zip`：另一个触摸诊断候选，不能与常规候选混用性能数据。诊断包附独立输入源码、构建证据及现场结果补充。
- `cns-f407-serial-monitor-20261023.zip`：Serial独立候选历史构建；不是11月3日的新构建。
- `cns-f407-numeric-control-20261023.zip`：此前显示修复对照固件历史构建；用户回刷下载/校验成功，触摸仍未恢复。
- `SHA256SUMS.txt` / `ASSETS.json`：附件大小与SHA-256。

本地同次常规HEX/AXF/MAP/build.log存于本目录及`Release/Latest/`，二进制不进Git。源码与固件状态分别记录，GitHub源代码自动ZIP不等于诊断配置快照。

## GT911诊断候选

基于367ac05，仅在`Debug/Inc/debug_config.h`打开`DEBUG_SELFTEST_ENABLE`与`DEBUG_SELFTEST_GT911_ENABLE`，未改GPIO、I2C、复位时序或触摸算法。16:29:34—16:30:18 UTC+8已实际Keil Rebuild All 0/0；Flash实际装载454392 B，主SRAM静态111944 B。

诊断HEX SHA-256：`8a31498536744e6a0c3406a47ed1da18c7f1d140a901a51062ae94ece19615df`。UART1为115200、8N1、无流控。诊断启用既有DebugTask（384 words栈），不能作为常规性能样本。

用户烧录后持续输出`addr=0x00 result=1 err=65 init_err=0 cfg_ver=0 cfg_ok=0 int=0 scl=1 sda=1 status=0x00 cfg=00000000`。65即0x41，表示读取寄存器的设备写地址阶段未收到ACK；没有逐地址探测或波形证据，不能断言两个地址都NACK或认定硬件损坏。

## 验证边界

外接HW-508模块已有启动音试听反馈；扩展板蜂鸣器本体因5V-BEC供电问题仍需核对。RGB与UART1 CLI未实现。当前屏幕可显示、运行时间增加，但触摸不可用；完全断电、断开扩展板、回刷旧显示修复版、重插外部屏幕排线均无恢复。尚缺供电测量与信号波形，问题保持未解决。统一host回归、完整数据/RF/CSV/保护/资源验收和长稳均未因本次归档新增通过结论。
