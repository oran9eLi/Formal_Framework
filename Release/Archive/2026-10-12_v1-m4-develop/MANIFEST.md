# 最新全量构建固件：M4 开发候选

| 项目 | 记录 |
|---|---|
| 构建时间 | 2026-10-12 22:57:25—22:58:01（UTC+8） |
| 工程 / Target | MDK-ARM/formal_framework.uvprojx / Target 1 |
| 芯片 | STM32F407ZGTx |
| 编译器 / Pack | ARMCC5.06 update5 build528 / Keil.STM32F4xx_DFP.2.17.0 |
| 构建 | Rebuild All，退出码0，0 Error(s), 0 Warning(s)，35 s |
| 源码身份 | M4实现f3039004193026debaf7c39dc8e1665bc16ef5a4；实现与文档96b99656b74802457e7588e9e2a18c8929e3d95f已合入并推送develop，远端SHA已核对 |
| 发布性质 | 已推送源码的M4开发候选；固件附件仅本地归档，没有M4标签，不是稳定版或发布rc |
| Code / RO / RW / ZI | 309408 / 105828 / 1728 / 119488 B |
| 主SRAM用量 / 余量 | 121216 / 9856 B；heap_4固定40 KiB已在其中，运行峰值待测 |
| CCM | LVGL固定40 KiB |
| 应用IROM / 配置区 | 应用上限0xE0000；sector11 0x080E0000..0x080FFFFF保存Node ID |
| HEX数据范围 | 0x08000000..0x08065673，不覆盖配置区 |

## 同次文件与 SHA-256

| 文件 | SHA-256 |
|---|---|
| formal_framework.hex（可烧录） | a778892f5440b909ce978e34092c112e56ea4e9f96f1a59b134519c687d0f0a0 |
| formal_framework.axf（调试） | 98075bb633ce5cb1b3e465b4b6bb9827f285522dadc9f24c73996193534dae25 |
| formal_framework.map（资源） | 8c4d77970d68c2603f0dc540d0324b219d53ddc2e3820b8caa8314db6952230b |
| build.log | 8adf94ea3f74c9b354ecbbe49adc814c64268ce61adfbfa54bcd4b416ce2a35c |

构建前后1011个生产输入无变化，并与此前M4输入逐文件一致。原始清单SHA-256为65232adef9c0f242377e02143f324cb29d6a4f355ee0ef9f69c4c0c5cc19f83a；本次同次产物、输入ZIP及构建证据位于build/v1_m4_merge/artifacts/。此前M4文件及原MANIFEST保留于../Archive/2026-10-12_v1-m4-implementation/；原M3保留于../Archive/2026-10-12_v1.0.0-pre.m3.20261012/，其GitHub存档继续有效。

五项主要审查问题已修正，旧Debug分消息统计续M5。2026-10-12用户确认首轮交互未发现问题、LoRa离线改参拒绝符合预期，接受本轮交互验收（V1-E08）；因接线问题LoRa未插接，实际无线收发、在线配置及其他必需矩阵仍待验证。本轮没有host回归、完整实板验收、运行资源或24小时长稳证据。反馈按对话关联本候选，实际烧录SHA未独立核对；此次文档补记不改变固件。详细记录见[M4验证记录](../V1.0_M4_验证记录.md)。

首次上板先核对E22课堂预设，尤其空速9.6 kbit/s；Profile mismatch需厂商工具手动修正。烧录只擦固件使用的扇区，整片擦除会清空Node ID保存区，见[实验说明](../../Development_Guide/2026-10-12_M4手动LoRa实验与配置说明.md)。
