# 2026-11-05 GNSS本机UART2硬件回环自测

> 已撤回：用户2026-11-05要求删除，当前代码已完整移除；本目录仅保留历史证据，不再作为可推荐候选。当前无回环固件见[移除回环候选](../2026-11-05_gnss-no-loopback/MANIFEST.md)。

状态：软件验证完成，物理回环待执行；V1-GNSS-T04、V1-GNSS-R01、V1-E40。该专用固件用于验证开发板收发路径，不是GPS定位验收。

- [回环自测HEX](cns-f407-gnss-uart2-loopback-20261105.hex)：先断电、拔掉GPS模块插头，再用跳线短接开发板PA2与PA3。调试串口USART1 115200/8N1，观察GNSS-LOOP。
- [普通HEX](cns-f407-gnss-normal-20261105.hex)：关闭测试发送与周期诊断。测试结束后断电移除PA2/PA3短接线，才能恢复GPS连接和普通/配置诊断固件。
- [完整步骤与判定](../../../Development_Guide/Change_History/2026-11-05_GNSS本机串口回环自测.md)。

回环版每秒由sensor唯一owner实际发送18B固定非NMEA标记，HAL timeout5ms；只有原DMA接收、原Sensor消费的真实字节完整匹配才增加matched。发送成功不当作接收成功，不向RX软件注入数据，不生成位置或时间。回环时nmea=0、fix=0属于预期。GPS输出必须先隔离，避免两个TX输出互相驱动。

## 构建证据

唯一MDK-ARM/formal_framework.uvprojx / Target 1，普通与回环两版实际Rebuild All均0 Error(s), 0 Warning(s)，各13s。Keil MDK5.39 / ARMCC5.06 u7 b960 / C99；本机DFP3.1.1，工程u5/DFP2.17.0元数据未改，HAL/CMSIS来自项目目录。本次沿用调用者权限兼容子进程启动，不改系统设置。

| 项目 | 普通版 | 回环自测版 |
|---|---|---|
| LOOPBACK / GNSS监视 | 0 / 0 | 1 / 1 |
| Code / RO / RW / ZI，B | 342736 / 111020 / 1992 / 110720 | 346328 / 112128 / 1996 / 110716 |
| 主SRAM静态使用 / 余量，B | 112712 / 18360 | 112712 / 18360 |
| DMA缓冲 | 0x20006230，1024B，主SRAM | 0x20006230，1024B，主SRAM |
| Flash装载，B | 453884 | 458588 |
| HEX连续范围 | [0x08000000,0x0806ECFC) | [0x08000000,0x0806FF5C) |
| HEX SHA-256 | 5c2448804c2819c50efdd514c865817beb0fd39b6fbeb26b838ac5be87c48cc3 | a522be719895b636ba66add40d77baabe40f44b3f2740152b00ee921a8ca2ae9 |

39组host全部通过且编译日志无警告，包含普通零TX、owner限速、HAL状态、marker匹配/噪声/跨块/重同步及无假定位。构造测试不替代实际跳线、引脚、电气和中断时序。回环通过只证明本机发送→引脚→DMA→任务消费与内容匹配；收发共用时钟，不能证明外部GPS真实波特率、输出或兼容，低速标记也不覆盖HT/TC满负荷验收。

当前无.git，以每版1027项C/H/S及唯一uvprojx清单SHA标识。每版输入构建前后不变，与各自production_source.zip逐项一致；回环ZIP使用该次直接保存的Debug及Framework配置头，当前恢复普通配置。每个子目录HEX/AXF/MAP/HTM/rebuild.log来自各自同次构建。keil_matrix.json、build_evidence.json、host_tests_source.zip、verification/与SHA256SUMS.txt记录完整证据。

主SRAM比前配置候选增加32B；普通Flash增加84B。诊断沿用384word Debug栈及1648B额外堆估算，只用于短时自测，资源与长稳未实测。HEX记录、范围和无重叠检查通过，不写sector11。未烧录/执行物理回环、未关闭GPS收数故障或M5/M6；Latest与历史产物保留，未提交、推送或新增标签。
