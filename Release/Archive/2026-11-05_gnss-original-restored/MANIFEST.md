# 2026-11-05 恢复P25适配前原始GNSS候选

状态：本地回滚候选，V1-GNSS-T06、V1-E42、台账R46。用户明确取消本轮P25实现，要求恢复首次适配P25之前的原始GNSS。源码恢复、主回归及Keil验证已完成；GPS现场收数问题V1-GNSS-R01仍未解决，没有新增烧录或实板通过结论。

- [本次重建HEX](cns-f407-gnss-original-restored-20261105.hex)：恢复原始USART2 PA2 TX / PA3 RX、9600 8N1、512B循环DMA及原NMEA处理。
- [回滚技术记录](../../../Development_Guide/Change_History/2026-11-05_恢复P25适配前原始GNSS.md)：范围、证据与验证边界。
- [构建及产物检查](build_evidence.json)、[产物SHA清单](SHA256SUMS.txt)。

## 原始基线与恢复范围

基线为[2026-11-04电机人工锁候选](../2026-11-04_motor-manual-lock/MANIFEST.md)的[production_source.zip](../2026-11-04_motor-manual-lock/production_source.zip)，SHA-256为`d43c636e419782d3106f7c33b732728f52724fcf1cd6f8f2ce5bb0fad4281332`。归档1027项生产输入与首次P25实施前备份逐项相同；首次P25 CHANGESET的before SHA也与此基线对应，不是仅按文件日期推断。

本轮恢复13个生产文件，当前全部1027项生产输入与该基线逐字节相同；Keil构建前后输入不变。首次P25试接、多星座GSA聚合、严格校验调整、512B单轮处理预算，以及之后的接收恢复、累计诊断和CFG读回改动均已退回原实现。旧代码注释为ATGM336H，不据此猜测原来或当前实物模块型号。基线已有的原句捕获回调及默认关闭的旧GNSS监视保留。

恢复范围不影响前序单路供电、电机人工锁、显示及业务功能：默认仍为仅第二组PA4/PC1、第一组停用、ADC1内部参考保留；电机人工锁保留，电压门禁算法与阈值保留且开关默认0。未调整电流计标定。

测试侧恢复runner、原DMA和measurement测试，删除6个P25阶段新增测试。estimator和TX两份已有测试保留已确认的类型/依赖及Battery2兼容夹具，不改变GNSS断言或固件；Tests目录不作为1027项生产一致性的组成部分。恢复前完整备份及逐项结果见[restore_results.json](../../../build/20261105_gnss_restore_original/restore_results.json)。

## 本次实际构建与产物身份

唯一入口`MDK-ARM/formal_framework.uvprojx`，Target 1，Rebuild All。2026-11-05 **18:50:50—18:51:09 UTC+8**，Keil日志13s；退出码0，**0 Error(s), 0 Warning(s)**。日志确认ARMCC5.06 update7 build960，工程沿用C99及原构建配置。

| 项目 | 本次结果 |
|---|---|
| Code / RO / RW / ZI | 341400 / 111020 / 1956 / 110036 B |
| 主SRAM静态使用 / 余量 | 111992 / 19080 B |
| GNSS DMA地址 / 大小 | 0x200061A8 / 512 B，位于DMA可用主SRAM |
| Flash实际装载 | 452544 B，压缩装载口径 |
| HEX范围 | [0x08000000, 0x0806E7C0) |
| HEX检查 | 记录校验通过、无重叠、不写sector11 |
| HEX SHA-256 | 3cfd631440c51b03b2e1a5727a546c0c9e3e20a28b6adc66ecf2481d8e4a2ddc |
| 本次源码ZIP SHA-256 | 15c0792a571ca5b850c251d8dbb72494c95292314008093c76776c1029127ade |

新HEX与原电机人工锁候选HEX逐字节相同。本次`production_source.zip`是当前输入重新打包，ZIP容器SHA与原基线ZIP不同，内部1027项文件内容相同；两者身份分别保留。HEX、AXF、MAP、HTM与`rebuild.log`来自本次同次构建，`inputs_before.json`、`inputs_after.json`及源码ZIP供复核。当前无.git，不声明提交或分支身份。

## 主机验证及限制

GCC15.2.0共尝试19组：**18组主回归通过，另1组旧UART6测试链接失败**，编译日志无warning。不能称19组全部通过。

18组覆盖5项既有GNSS回归，以及单路供电、电机锁、显示、MAVLink、CSV、Debug所有权和fatal撤收等13项保护回归。额外`rpi_baud_switch`因原夹具缺`Px4Lite_PlatformGetMs`桩而链接失败；该测试及对应生产文件与pre-P25基线相同，本次未扩大修复。保留真实失败日志，不计为通过。详细命令、组别、测试前后1027项输入检查及失败说明见[test_review.json](../../../build/20261105_gnss_restore_original/test_review.json)，归档`verification/`与`host_tests_source.zip`保存本轮证据及实际测试源码。

本次只关闭用户要求的回滚任务。未烧录、未确认P25在9600配置下输出、未取得GPS定位/断流恢复或整机长稳结论；M5/M6、触摸与电流计问题不因此关闭。旧P25及诊断候选仅作历史，原归档MANIFEST和SHA不改。Latest未更新，未提交、合并、推送或新增标签；文档收尾不改变固件输入，不重复构建。
