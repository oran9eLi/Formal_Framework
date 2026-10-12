# 最新全量构建固件：M4 Node ID 保存修正候选

| 项目 | 记录 |
|---|---|
| 构建时间 | 2026-10-13 00:34:32—00:35:05（UTC+8） |
| 工程 / Target | MDK-ARM/formal_framework.uvprojx / Target 1 |
| 芯片 | STM32F407ZGTx |
| 编译器 / Pack | ARMCC5.06 update5 build528 / Keil.STM32F4xx_DFP.2.17.0 |
| 构建 | Rebuild All，退出码0，0 Error(s), 0 Warning(s)，32 s |
| 源码身份 | 生产提交06e6a211519e181627f798ef78497e6b15db763b；1011个原始输入与该提交的Git过滤后blob及构建ZIP核对一致；后续文档提交不改变固件身份 |
| 发布性质 | 已推送的M4 Node ID保存修正预发布源码标签v1.0.0-pre.m4.20261013，标签指向06e6a21；不是稳定版或发布rc，未新建GitHub Release/固件附件 |
| 本次修改 | Save Node解除动力电源与稳定采样窗口依赖，保留停机互锁；Power gate改Motor lock，保存超时归Error |
| Code / RO / RW / ZI | 309160 / 105828 / 1712 / 119488 B |
| 主SRAM用量 / 余量 | 121200 / 9872 B；heap_4固定40 KiB已在其中，运行峰值待测 |
| CCM | LVGL固定40 KiB |
| 应用IROM / 配置区 | 应用上限0xE0000；sector11 0x080E0000..0x080FFFFF保存Node ID |
| HEX数据范围 | 0x08000000..0x0806557B，不覆盖配置区 |

## 同次文件与 SHA-256

| 文件 | SHA-256 |
|---|---|
| formal_framework.hex（可烧录） | 17cc496d54cf06e64cf527d6b03fcae36a8118f29319e5062bc308b3f611afb5 |
| formal_framework.axf（调试） | 3a4466d4ca7315c5ec6927fe6f6819b83461e01116255b839ec926a8777c816b |
| formal_framework.map（资源） | cbb909d33e890a3e1bd3009fed77fa47974fb04a2ebd97b45edd3191e5b2ee43 |
| build.log | e224f4e9c8899f0426deb7ed25bbac40c542240dd1b7645f0da3f47627f91afb |

构建前后1011个生产输入无变化；当前文件和原始输入ZIP逐文件核对一致。清单SHA-256为d061af4bb19a6edb97476db88c1f5f1c4a3899c60b46b470a5ee2f40172b094c，ZIP SHA-256为e9b429710dd019bcb632f0d62cd41dd52877daa411dcd05174beae392c56cb1f。完整证据位于build/v1_m4_node_save_publish/artifacts/。

同次文件长期保留于[2026-10-13标签归档](../Archive/2026-10-13_v1.0.0-pre.m4.20261013/MANIFEST.md)，实际Git推送/标签核对见[归档记录](../V1.0_M4_标签归档记录.md)。原develop的M4文件及原MANIFEST保留于../Archive/2026-10-12_v1-m4-develop/；此前M4实施存档和M3 GitHub存档保留。2026-10-13用户确认本次保存修正验证成功（V1-E12），对应Save Node与重启核对；烧录SHA未独立核对。其余M4无线链路/异常/资源/长稳继续待验，本次未新增或运行host回归。详见[M4验证记录](../V1.0_M4_验证记录.md)。

首次补验可不接LoRa和动力电源，主控室内正常供电、控制输出停止后Save Node并重启核对。主控供电符合原Flash编程条件是运行前提，本次未新增VDD检测。烧录采用仅擦固件使用扇区，整片擦除会清空Node ID保留区，见[配置说明](../../Development_Guide/2026-10-12_M4手动LoRa实验与配置说明.md)。
