# 2026-11-04 电机人工锁候选

状态：本地开发候选，软件验证完成，实板待验证；任务V1-MOTOR-T01、V1-PWR-T02，证据V1-E35。

用户报告PA4采样针脚对GND约12 V，已提示立即断电、断开信号并核查PC1。恢复实板实验前必须先核对真实点位、电流计输出及分压；不能用关闭电压门禁或修改系数代替电气处理。未确定ADC损坏或读数异常的唯一根因。

| 项目 | 记录 |
|---|---|
| 功能 | 原电压门禁算法/阈值保留，默认POWER_GATE=0；新增电机页第4个Lock/Unlock按钮和实际锁状态，修复Details±1%入口 |
| 默认采样组 | 保持仅第二组PA4/PC1，第一组PA5/PC0停用，ADC1内部VREFINT保留；未改分压或电流标定 |
| 人工锁 | 开机及异常锁定，显式解锁清旧目标并确认最小PWM，再等待2500 ms预备；不自动非零油门 |
| 输出事实 | 上锁清目标并撤收；连续写入/撤收失败不发布伪造的零输出，旧确认快照自然过期 |
| 工程 / Target | MDK-ARM/formal_framework.uvprojx / Target 1 / Rebuild All |
| 实际工具链 | Keil MDK5.39 / ARMCC5.06 update7 build960 / C99；本机DFP3.1.1，工程元数据u5/DFP2.17.0未改，HAL/CMSIS来自工程目录 |
| 最终默认构建 | 2026-11-04 17:24:29—17:24:43 UTC+8，Keil日志11 s；退出0，0 Error(s), 0 Warning(s) |
| 开关验证 | POWER_GATE=1及默认0两次Rebuild All均0错误0警告；最后默认0，当前源码与最终输入一致 |
| 主机回归 | GCC15.2.0，26组全部通过，编译日志无警告；命令见verification/host_command.txt |
| 生产输入 | 1027项C/H/S及唯一uvprojx，每次构建前后不变，恢复后的当前目录及production_source.zip逐项SHA一致；当前目录没有.git，不能声明提交或分支身份 |
| 本轮范围 | 较上一轮单路供电候选11个生产文件调整；不覆盖旧候选身份 |
| Code / RO / RW / ZI | 341400 / 111020 / 1956 / 110036 B |
| 主SRAM静态使用 / 余量 | 111992 / 19080 B；较上一轮+48 B，不代表运行堆峰值或栈水位 |
| Flash实际装载 | 452544 B，压缩装载口径 |
| HEX检查 | 全部记录校验通过，无重叠，连续范围[0x08000000, 0x0806E7C0)，不覆盖sector11配置区 |
| HEX SHA-256 | 3cfd631440c51b03b2e1a5727a546c0c9e3e20a28b6adc66ecf2481d8e4a2ddc |
| 输入清单SHA-256 | 45843220ae7a06abb9d2f0ac3d0684db9e346b97c022c03e8ceb99903aa1afcc |
| 源码ZIP SHA-256 | d43c636e419782d3106f7c33b732728f52724fcf1cd6f8f2ce5bb0fad4281332 |

[formal_framework.hex](formal_framework.hex)为默认门禁关闭固件。HEX、AXF、MAP、HTM、rebuild.log均取自最后同一次默认构建。SHA256SUMS.txt记录产物与证据哈希，keil_matrix.json及verification/power_gate_on保存恢复电压门禁的构建证据。需要恢复原保护时将PX4LITE_CONTROL_POWER_GATE_ENABLE设1后重新构建。

production_source.zip包含Core/Bsp/Sensor/Framework/Business/Display/Debug/Storage/Third_Party的C/H/S及uvprojx，含部分未编入实现，不是全项目资料备份。host_tests_source.zip保存Tests源码、桩和脚本；host_final_summary.log与verification/保存26组编译/运行日志。

新增锁状态枚举6及日志尾部三个事件，旧编号与topic布局保持；原196日志编号不变，当前目录总数199。若外部文字字典需展示新增事件，按尾部编号更新。

本轮只完成软件实现、审查、主机与Keil验证。没有烧录、实测电压/电流标定、实际电机输出、实屏/触摸、保护波形、运行水位或长稳结论。原M5/M6及触摸故障保持；未更新Latest、合入、推送或建立标签。测量调查及实现细节见[技术说明](../../../Development_Guide/Change_History/2026-11-04_电机手动锁与临时停用电压门禁.md)。
