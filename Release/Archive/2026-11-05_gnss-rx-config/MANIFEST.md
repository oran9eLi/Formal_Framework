# 2026-11-05 GNSS运行配置只读诊断候选

状态：软件验证完成，运行配置待现场读回；V1-GNSS-T03、V1-GNSS-R01、V1-E39。用户新日志显示接收已编程启动但未见DMA搬运，根因仍未确认，GPS尚未调通。

- [配置诊断 HEX](cns-f407-gnss-rx-config-diagnostic-20261105.hex)：USART1 115200/8N1，新增GNSS-CFG、GNSS-DMA、GNSS-REG；短时试接使用。
- [普通 HEX](cns-f407-gnss-rx-config-normal-20261105.hex)：默认关闭周期诊断，与上一恢复修复版HEX完全相同。
- [配置对照与诊断边界](../../../Development_Guide/Change_History/2026-11-05_GNSS接收配置只读诊断.md)。

## 范围

5个生产文件只新增BSP只读配置快照、Platform Adapter映射和Debug短报告。读取GPIO/RCC使能、UART BRR/CR1/2/3、DMA CR/PAR/M0AR/NDTR及实际期望地址；不读USART SR/DR，不清标志、不改队列、配置或恢复策略。保持USART2 PA2/PA3、115200、1024B DMA和NMEA试接；罗盘不接，原电机锁/电压门禁旁路/第二组供电不改。

本工程HAL整数BRR宏在PCLK1=42MHz/115200时为0x16C；该派生值不是线速实测。GPIO IDR是瞬间采样，高电平不证明没有数据活动；DMA可在采样期间运行，跨行字段不作为严格同时值。

## 实际构建

唯一工程MDK-ARM/formal_framework.uvprojx、Target 1、2026-11-05两次Rebuild All，各12s，退出码0、0 Error(s), 0 Warning(s)。实际Keil MDK5.39 / ARMCC5.06 update7 build960 / C99；本机DFP3.1.1，工程元数据u5/DFP2.17.0未改，HAL/CMSIS来自工程目录。

本次Windows管理员启动被系统取消后，成功采用调用者权限子进程兼容环境RunAsInvoker启动同一UV4工程；仅对子进程设环境，没有更改注册表或系统设置，也不请求额外Windows权限提升。构建UTC时间、源码与产物SHA见keil_matrix.json。

| 项目 | 普通版 | 诊断版 |
|---|---|---|
| Code / RO / RW / ZI，B | 342656 / 111020 / 1960 / 110720 | 345648 / 112008 / 1960 / 110720 |
| 主SRAM静态使用 / 余量，B | 112680 / 18392 | 112680 / 18392 |
| Flash实际装载，B | 453800 | 457784 |
| GNSS DMA地址 / 大小 | 0x20006210 / 1024B | 0x20006210 / 1024B |
| HEX连续范围 | [0x08000000, 0x0806ECA8) | [0x08000000, 0x0806FC38) |
| HEX SHA-256 | d26a49334926fee763f4ef7f0fd5e39c84768aeaf9b889f1a28840f0e3c60105 | 4e05d8e5009ccc7349c00cb4f73a5e0615532f7fedc81caf1afbd4e5d2716c73 |

普通版与前版相同；诊断版Flash增加1784B、静态SRAM无增加。诊断沿用384word任务栈及额外1648B堆估算，不作为75%资源目标、满载或长稳达标样本，运行堆/栈水位待测。HEX记录校验、无重叠及范围检查通过，不写sector11。

## 输入与验证

当前无.git，不声明提交身份。每版1027项C/H/S及唯一uvprojx输入构建前后不变，并与各自production_source.zip逐项SHA核对；诊断ZIP使用构建时直接保存的Debug配置，当前工作区恢复普通配置。清单包含未编入第三方实现，不是整个项目资料备份。

variant子目录中的HEX/AXF/MAP/HTM/rebuild.log来自各自同次构建，附inputs_before/after.json、debug_config.build.h和production_source.zip。37组host全部通过、编译日志无警告，含BSP配置快照及Adapter/Debug真实映射、正常/错误配置、零配置、HAL BRR运算、DR探针、寄存器/队列不变与极限短行；源与命令在host_tests_source.zip、verification/及host_final_summary.log。build_evidence.json记录资源/DMA/HEX检查，SHA256SUMS.txt覆盖产物和证据。

独立只读复核未发现新增P1/P2。尚未烧录本版、取得现场配置值或确认GPS收数；R01不关闭，M5/M6、触摸与长稳状态不因此改变。未更新Latest、提交、推送或新增标签。
