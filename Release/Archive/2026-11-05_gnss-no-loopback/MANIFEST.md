# 2026-11-05 移除GNSS回环自测

用户明确取消回环，已从当前工程完整删除。本候选恢复为加回环前的GPS USART2 115200/NMEA接收、恢复和只读配置诊断；不包含测试发送、原始观察器、标记匹配、开关或GNSS-LOOP报告。V1-GNSS-T05、V1-E41；GPS收数故障R01仍进行中。

- [无回环普通HEX](cns-f407-gnss-normal-no-loopback-20261105.hex)
- [无回环配置诊断HEX](cns-f407-gnss-diagnostic-no-loopback-20261105.hex)
- [移除说明](../../../Development_Guide/Change_History/2026-11-05_移除GNSS回环自测.md)

8个生产文件与回环前备份逐项SHA一致；1027项生产输入完全匹配R43。三个旧测试和runner已恢复，新增raw TX测试已删除，全部当前生产/测试目录检索无回环标识。历史回环候选仅保留证据并已标记撤回。

## 实际验证

14组相关host回归全部通过、编译日志无警告。普通/诊断两版均从唯一工程MDK-ARM/formal_framework.uvprojx的Target 1实际Rebuild All，0 Error(s), 0 Warning(s)，各12s。实际ARMCC5.06 update7 build960/C99、Keil MDK5.39；本机DFP3.1.1，工程工具/Pack元数据未改，HAL/CMSIS来自项目目录。采用已验证的调用者权限兼容子进程启动，不改系统设置。

| 项目 | 普通版 | 配置诊断版 |
|---|---|---|
| HEX SHA-256 | d26a49334926fee763f4ef7f0fd5e39c84768aeaf9b889f1a28840f0e3c60105 | 4e05d8e5009ccc7349c00cb4f73a5e0615532f7fedc81caf1afbd4e5d2716c73 |
| Code / RO / RW / ZI，B | 342656 / 111020 / 1960 / 110720 | 345648 / 112008 / 1960 / 110720 |
| 主SRAM静态使用 / 余量，B | 112680 / 18392 | 112680 / 18392 |
| DMA地址 / 容量 | 0x20006210 / 1024B | 0x20006210 / 1024B |
| Flash装载，B | 453800 | 457784 |

两版HEX均与加回环前R43完全相同。每版1027项输入构建前后不变，当前工作区恢复普通配置并与普通版输入一致；各自源码ZIP逐项验证，HEX记录/范围/无重叠检查通过、不写sector11。各子目录HEX/AXF/MAP/HTM/log来自同次构建；完整证据见keil_matrix.json、build_evidence.json、verification/、host_tests_source.zip及SHA256SUMS.txt。

本次只关闭回环删除任务，不据此宣称GPS已调通、接线有错或驱动完全正确。尚未烧录本次构建或新增实板结论，M5/M6与既有问题保留。配置诊断沿用额外1648B堆估算，仅短时使用，资源/长稳未实测。Latest及历史版本保留，未提交、推送或新增标签。
