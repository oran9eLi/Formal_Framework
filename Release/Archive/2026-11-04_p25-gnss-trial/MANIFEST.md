# 2026-11-04 北征P25 Pro定位功能试接候选

状态：本地开发候选；软件验证完成，实板待验证。对应V1-GNSS-T01、V1-E36，罗盘不接，不构成V1.0稳定版或整机验收通过。

## 固件选择

- [诊断版 HEX](cns-f407-p25-gnss-diagnostic-20261104.hex)：短时试接，每秒从USART1输出GNSS-RX、GNSS-FIX和TIME；方便触摸不可用时取证。
- [普通版 HEX](cns-f407-p25-gnss-normal-20261104.hex)：关闭周期GNSS诊断，当前工作区默认配置；试接确认后使用。
- [接线、日志判断与恢复步骤](../../../Development_Guide/Change_History/2026-11-04_北征P25Pro_GNSS试接.md)。

两版均使用USART2 PA2 TX / PA3 RX、115200 8N1、1024 B DMA，复用NMEA链路，不解析UBX定位，不发送猜测的芯片配置。原电机人工锁、临时关闭电压门禁和仅第二组供电监测保留；电流计问题按用户要求暂留，未改标定。P25商家资料列5 V、115200、10 Hz，但本批UART电平、出厂报文与实际定位仍待实物验证。

## 实际构建

唯一工程`MDK-ARM/formal_framework.uvprojx`，Target 1，Rebuild All。实际Keil MDK5.39 / ARMCC5.06 update7 build960 / C99；本机DFP3.1.1，工程元数据u5/DFP2.17.0未改，HAL/CMSIS来自工程目录。用户点击Windows管理员确认后实际完成构建，未用此前候选代替。

| 项目 | 普通版 | 诊断版 |
|---|---|---|
| 实际构建时间UTC+8 | 2026-11-04 18:28:55—18:29:09 | 2026-11-04 18:28:40—18:28:55 |
| Keil结果 / 日志耗时 | 0 Error(s), 0 Warning(s) / 12 s | 0 Error(s), 0 Warning(s) / 12 s |
| DEBUG_GNSS_MONITOR_ENABLE | 0 | 1 |
| Code / RO / RW / ZI，B | 342016 / 111020 / 1960 / 110632 | 342952 / 111456 / 1960 / 110632 |
| 主SRAM静态使用 / 余量，B | 112592 / 18480 | 112592 / 18480 |
| GNSS DMA缓冲 | 0x20006200，1024 B，主SRAM | 0x20006200，1024 B，主SRAM |
| Flash实际装载，B | 453164 | 454532 |
| HEX连续地址范围 | [0x08000000, 0x0806EA2C) | [0x08000000, 0x0806EF84) |
| 额外Debug任务堆估算，B | 0 | 1648 |
| HEX SHA-256 | 2f5369ff44b6cdb927154d064d6ef45a23396c7a6fc0424fe01f82b469783d4e | 407d33a53368e1d8c9ba0a8b4f6c294e27548f042c9f6ebaf81993d2329ab4bb |

普通版较上一电机锁候选主SRAM静态增加600 B，Flash增加620 B。静态余量不等于实时堆/任务栈水位。诊断版沿用384 word任务栈，预计额外占用1648 B RTOS堆；沿历史75.21%峰值估算可达约79.2%，不能作为75%预算、满载或长稳达标样本；只用于短时排查，没有扩大堆或任务栈。

## 输入与证据

本目录没有Git提交身份，当前工作区没有.git；以输入清单中的路径、大小和SHA标识源码。每版1027项C/H/S及唯一uvprojx，在构建前后均无变化；当前恢复的普通配置与普通版输入一致。每版源码ZIP均逐项核对SHA，诊断版使用该次构建直接保存的Debug头文件，两个配置快照各自独立。

每个variant子目录保存同次HEX、AXF、MAP、HTM、rebuild.log、inputs_before/after.json、debug_config.build.h和production_source.zip。输入范围含Core/Bsp/Sensor/Framework/Business/Display/Debug/Storage/Third_Party的C/H/S及uvprojx，也含未编入第三方实现，不是完整文档/硬件资料备份。HEX全部记录校验、无重叠和地址范围检查通过，不写sector11配置区。

`keil_matrix.json`记录构建时间、退出码、输入和产物SHA；`build_evidence.json`记录资源、DMA地址及HEX检查。34组针对性host回归全部通过、编译日志无警告，覆盖原供电/电机锁及GNSS采集、时效、多星座、连续DMA、溢出、恢复和诊断；命令见`verification/host_command.txt`，结果见`host_final_summary.log`及`verification/`。`host_tests_source.zip`保存对应测试源码与脚本，`SHA256SUMS.txt`覆盖归档产物和证据。

独立只读复核未发现本轮新增P1/P2。仍需保证相邻两次NDTR推进间隔小于整圈约88.9 ms，依赖HT/TC时最坏中断延迟小于半圈约44.4 ms；主机模拟不能验证真实时延。GSV分类仍限既有GPS/BDS；GSA辅助维度不等于完整融合解算。尚未烧录或取得P25真实串口流、定位、断流恢复、屏幕、UART6、运行资源或长稳证据。M5/M6、触摸及其他既有问题不关闭；未更新Latest、提交、推送或建立标签。
