# M5 教学操作与诊断开发候选

| 项目 | 记录 |
|---|---|
| 生产提交 / 分支 | c4d3eeec139a154e847a254f43e053f63e3b52f1 / codex/v1-m5；后续文档提交不改变固件身份 |
| 性质 | 本地开发候选，M5待验证；未合入develop、未推送、未打M5标签或建立GitHub Release |
| 构建 | 2026-10-13 08:58:27—08:59:01（UTC+8），Rebuild All；退出码0，0 Error(s), 0 Warning(s)，Keil日志32 s |
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
| formal_framework.axf | 0787cbea0ee418515b8934934e322f81e7444f82949ee5c6ee825bb81753423e |
| formal_framework.map | 9406f3a766f9f6c7ee7b3b6726ead21386dec8c397f840002f6af889cfaae140 |
| build.log | df9d03aec5f168637f1fd9ab9a015b3ad3975a88701749ba7008220b9dc932b7 |

生产清单SHA-256：fa8c9920ef3fa0031e65d6c023a891d04c1e31de0aedac2cc395b6b66e8ef180。原始输入ZIP SHA-256：a6ebe889b605c2ad7e37837ac25cf1cb62cd8760917f45b247a84fd69d664d9b。同次HTM、原始输入、ZIP和构建证据保留在build/v1_m5/final/artifacts/；两个Release副本逐文件SHA已核对。Keil本机uvoptx和用户Reports保留，不参与生产提交。

软件范围：详情、独立Messages/Raw/Decoded、NMEA/Serial、两路Observe/Drift、手动Record、常驻资源统计及课堂打印配置；一次独立审查的六项重要问题已作源码修正，并由本次构建覆盖。未新增/运行host回归，未连接PC串口、射频或烧录；离线13类样例生成与Python语法检查成功。

软件构建不构成实板放行。C01—C16、运行堆/栈、完整显示耗时、最重组合及24小时证据仍待补。烧录只擦应用使用扇区；整片擦除会清空Node ID配置区。使用本次HEX并记录SHA及板号后按验证记录执行。

详见[M5验证记录](../../V1.0_M5_验证记录.md)。
