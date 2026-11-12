# 2026-11-05 GNSS串口接收恢复修复候选

状态：软件验证完成，现场待新版复测。对应V1-GNSS-T02、V1-GNSS-R01、V1-E38。用户已确认接线无问题，本轮以代码为排查前提；不能把构造恢复测试当作现场根因已确认或GPS已收数。

- [新版诊断 HEX](cns-f407-gnss-rx-fix-diagnostic-20261105.hex)：短时排查，USART1每秒GNSS-HW、GNSS-SW、GNSS-RX、GNSS-FIX及TIME，115200 8N1。
- [普通 HEX](cns-f407-gnss-rx-fix-normal-20261105.hex)：默认关闭周期Debug，当前工作区配置。
- [代码问题、修复与日志说明](../../../Development_Guide/Change_History/2026-11-05_GNSS串口接收恢复修复.md)。

## 已修复行为

上电累计诊断不再被通用软初始化清零；parser reset不丢失传输恢复待办或0ms限速事实。UART错误中断保存原始证据后关闭自有IDLE/错误源，由sensor限速完整停止、DeInit、Init并重新挂接DMA，处理HAL异步ABORT/Lock残留；DMA原点只在实际重置后归零，连续失败重试不重复计字节。MSP只设IRQ优先级，接收完整启动后才启用IRQ。新只读硬件/软件诊断不消费字节或读取DR，不改变NMEA解析边界。

保持USART2 PA2/PA3、115200 8N1、1024B DMA及NMEA试接，罗盘不接，不发送猜测配置。前序人工电机锁、电压门禁临时旁路和第二组供电采样保留；电流计问题仍暂留。

## 构建与资源

唯一工程MDK-ARM/formal_framework.uvprojx，Target 1，2026-11-05实际Rebuild All。Keil MDK5.39 / ARMCC5.06 update7 build960 / C99；工程u5/DFP2.17.0元数据未改，本机DFP3.1.1，HAL/CMSIS来自项目目录；不宣称复现历史工具环境。

| 项目 | 普通版 | 诊断版 |
|---|---|---|
| Keil结果 / 日志耗时 | 0 Error(s), 0 Warning(s) / 12s | 0 Error(s), 0 Warning(s) / 13s |
| DEBUG_GNSS_MONITOR_ENABLE | 0 | 1 |
| Code / RO / RW / ZI，B | 342656 / 111020 / 1960 / 110720 | 344264 / 111612 / 1960 / 110720 |
| 主SRAM静态使用 / 余量，B | 112680 / 18392 | 112680 / 18392 |
| GNSS DMA地址 / 容量 | 0x20006210 / 1024B | 0x20006210 / 1024B |
| Flash实际装载，B | 453800 | 456000 |
| HEX连续范围 | [0x08000000, 0x0806ECA8) | [0x08000000, 0x0806F540) |
| HEX SHA-256 | d26a49334926fee763f4ef7f0fd5e39c84768aeaf9b889f1a28840f0e3c60105 | b0d1f157bdb6eea1e0dd44ce4c4d3b8c2f9e958c13e9fb22f49da6783a846205 |

普通版比上一P25候选静态主SRAM增加88B、Flash增加636B；DMA地址在可用主SRAM，HEX记录校验、无重叠及范围检查通过，不覆盖sector11。静态余量不能替代运行堆或栈水位。诊断任务沿用384word栈及预计1648B额外堆，仅作短时排查，不能作为75%资源目标、满载或长稳通过样本；未扩大任务栈或堆。

## 源码与验证证据

9个生产文件调整，当前无.git，不声明提交或分支身份。每版1027项C/H/S及唯一uvprojx输入在构建前后不变，并与各自production_source.zip逐项SHA一致。两版仅Debug配置开关不同；诊断ZIP使用同次构建直接保存的Debug头，当前目录恢复普通配置且与普通输入一致。清单包含未编入的第三方实现，不是全项目资料备份。

各variant子目录中的HEX/AXF/MAP/HTM/rebuild.log来自各自同一次构建；keil_matrix.json记录UTC时间、退出码、输入及产物SHA，build_evidence.json记录资源及HEX检查。host_tests_source.zip、host_final_summary.log及verification/保存36组host回归的源码、命令、编译与运行记录，编译日志无警告。SHA256SUMS.txt覆盖固件和证据文件；具名HEX与子目录HEX逐项一致。

测试包括原GPS/供电/电机锁回归，以及Sensor跨初始化统计、待恢复工作保留、0ms/回绕限速、模拟HAL锁定ABORT、错误+IDLE中断隔离、五阶段恢复失败重试、连续停止失败不虚增字节、接收成功前IRQ关闭及getter不读DR。交叉复核发现的问题已修正。模拟不能代替真实IRQ延迟、UART波形、实际模块输出或现场定位。

尚未烧录新版或取得新版实板日志，V1-GNSS-R01根因未闭合；M5/M6、触摸及长稳不因此通过。未更新Latest、提交、合并、推送或新增标签。
