# M5 教学操作与诊断预发布存档

| 项目 | 记录 |
|---|---|
| 生产提交 / 分支 | c4d3eeec139a154e847a254f43e053f63e3b52f1 / codex/v1-m5；后续文档提交不改变固件身份 |
| 标签源码 / GitHub | a99e35a68fc2fd4843e0ae9ade81f8af275a176e；[v1.0.0-pre.m5.20261014](https://github.com/oran9eLi/Formal_Framework/releases/tag/v1.0.0-pre.m5.20261014)，分支/注释标签及四附件已推送并回下载一致 |
| 性质 | M5预发布，待M5/M6联合验收；未合入develop，不是稳定版或M6 rc |
| 构建 | 2026-10-14 13:58:58—13:59:47（UTC+8），Rebuild All；退出码0，0 Error(s), 0 Warning(s)，Keil日志41 s |
| 工程 / Target | MDK-ARM/formal_framework.uvprojx / Target 1 |
| 芯片 / 编译器 / Pack | STM32F407ZGTx / ARMCC5.06 update5 build528 / Keil.STM32F4xx_DFP.2.17.0 |
| Code / RO / RW / ZI | 338264 / 111004 / 1968 / 124488 B |
| 主SRAM用量 / 余量 | 126456 / 4616 B；较M4增加5256 B≤5632 B，余量≥4096 B |
| 既有堆 / DMA | heap_4 40 KiB主SRAM、LVGL 40 KiB CCM不变；DMA仍为主SRAM |
| 应用 / 配置区 | IROM上限0xE0000；sector11 0x080E0000..0x080FFFFF保存Node ID |
| HEX数据范围 | 0x08000000..0x0806DB73，449396 B；逐条校验和通过，不覆盖配置区 |
| 生产输入 | 1021个原始输入，构建前后无变化；与当前生产提交无差异，源码ZIP逐文件SHA一致 |

## 同次产物 SHA-256

| 文件 | SHA-256 |
|---|---|
| formal_framework.hex | 459e577d6e32cabf231e1aa167f871997d89011a2fcfee7a934c9ded410f86e5 |
| formal_framework.axf | 23145aaa3402eaa92c0605021cc851faf976fc83f6da376cc70bd4ddd3e3ec90 |
| formal_framework.map | 9406f3a766f9f6c7ee7b3b6726ead21386dec8c397f840002f6af889cfaae140 |
| build.log | 666241da808fb411feb42f4554f3f2580384154d48c3e0749903212a99cee424 |

生产清单SHA-256：babe2c122b02ebedb1f6014912dc0444d18614f2efc92139827c1d16576a1a16。原始输入ZIP SHA-256：d0b8919ea632a1d7da33f2bd766e040b4c2bdd52b9f35af9884916bd332b7566。同次HTM、输入、ZIP、构建与回下载证据保留在build/github_m5_20261014/；两个Release副本已核对。本机uvoptx/Reports保留。

软件范围：详情、独立Messages/Raw/Decoded、NMEA/Serial、两路Observe/Drift、手动Record、常驻资源统计及课堂打印配置；一次审查六项重要问题已修正。用户单机交互观察良好，实际烧录SHA与逐项结果待登记；本次未新增/运行host回归、实板/RF/运行资源或长稳。HEX与2026-10-13候选相同，AXF/MAP/log来自本次重新构建。

软件构建不构成实板放行。C01—C16、运行堆/栈、完整显示耗时、最重组合及24小时证据仍待补。烧录只擦应用使用扇区；整片擦除会清空Node ID配置区。使用本次HEX并记录SHA及板号后按验证记录执行。

详见[M5归档记录](../V1.0_M5_GitHub归档记录.md)、[M6准备](../V1.0_M6_准备与联合验收方案.md)及[联合矩阵](../V1.0_M5_M6_联合验收矩阵.md)。同次长期归档为[2026-10-14 M5标签](../Archive/2026-10-14_v1.0.0-pre.m5.20261014/MANIFEST.md)；[原M5候选](../Archive/2026-10-13_v1-m5-candidate/MANIFEST.md)与前序M4产物保留。
