# M5 预发布标签同次固件归档

日期：2026-10-14。标签 `v1.0.0-pre.m5.20261014`，分支 `codex/v1-m5`，源码 `a99e35a68fc2fd4843e0ae9ade81f8af275a176e`；生产实现 `c4d3eeec139a154e847a254f43e053f63e3b52f1`，1021 个输入一致。

Keil Rebuild All：2026-10-14 13:58:58—13:59:47 UTC+8，Target1 / ARMCC5.06 u5 build528 / DFP2.17.0；退出码 0，0 Error(s), 0 Warning(s)，日志 41 s。Code/RO/RW/ZI 为 338264/111004/1968/124488 B，主 SRAM 126456 B，余 4616 B。

| 文件 | SHA-256 |
|---|---|
| formal_framework.hex | 459e577d6e32cabf231e1aa167f871997d89011a2fcfee7a934c9ded410f86e5 |
| formal_framework.axf | 23145aaa3402eaa92c0605021cc851faf976fc83f6da376cc70bd4ddd3e3ec90 |
| formal_framework.map | 9406f3a766f9f6c7ee7b3b6726ead21386dec8c397f840002f6af889cfaae140 |
| build.log | 666241da808fb411feb42f4554f3f2580384154d48c3e0749903212a99cee424 |

HEX 与 2026-10-13 M5 候选相同，范围 0x08000000..0x0806DB73，不使用 sector11。本文件夹使用本次同一构建的 HEX/AXF/MAP/log。

源码、标签及四个[GitHub 附件](https://github.com/oran9eLi/Formal_Framework/releases/tag/v1.0.0-pre.m5.20261014)已保存，回下载 SHA 相同。详见[M5 GitHub 归档记录](../../V1.0_M5_GitHub归档记录.md)。原始证据在 build/github_m5_20261014/。

当前为 M5 预发布；用户单机交互观察良好，M5/M6 正式联合验收、运行资源和 M7 长稳仍待执行。未合入 develop，不是稳定版或 M6 rc。烧录只擦应用使用扇区。
