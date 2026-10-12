# 最新全量构建固件（开发存档）

| 项目 | 记录 |
|---|---|
| 构建日期 | 2026-10-12（UTC+8） |
| 工程 / Target | `MDK-ARM/formal_framework.uvprojx` / `Target 1` |
| 芯片 | STM32F407ZGTx |
| 实际编译器 | ARM Compiler 5.06 update 5 (build 528)，由 `armcc --vsn` 核对 |
| 工程 Pack | Keil.STM32F4xx_DFP.2.17.0 |
| 构建方式 / 结果 | Rebuild All / 退出码 0 / 0 Error(s), 0 Warning(s) / 32 s |
| 固件对应源码 | `5978dc6347ba3970cf4689a859e5d004001b1ff4` |
| 标签 | `v1.0.0-pre.m3.20261012` |
| 发布性质 | GitHub Pre-release 开发存档，未建立稳定版或发布 rc |
| Code / RO / RW / ZI | 300440 / 106016 / 1688 / 121240 B |
| 主 SRAM 静态用量 / 余量 | 122928 / 8144 B；不是运行时堆峰值 |

[GitHub 版本与固件下载](https://github.com/oran9eLi/Formal_Framework/releases/tag/v1.0.0-pre.m3.20261012)。[归档记录](../V1.0_M3_GitHub归档记录.md)包含回下载核对结果与验证范围。

## 同次构建产物

- `formal_framework.hex`：可烧录固件。
- `formal_framework.axf`：调试文件。
- `formal_framework.map`：链接与资源记录。
- `build.log`：本次全量构建日志。

上述四个二进制/日志文件按仓库规则保留在本地、不提交到 Git；GitHub Release 提供 HEX 和完整构建 ZIP。1003 个生产输入构建前后不变，并已核对与 Git 内容一致。

## SHA-256

```text
formal_framework.hex 0032581ae03f962fa76594b0b62e04ffb99b8195d812c5e1668f03849502bd19
formal_framework.axf 2f6f230fec09011ba5b33030efd67c02c0dfdf0fc0976ce289be241f5882ad89
formal_framework.map fde13ec6ca0f36778e61eec5d05020fa606d419183e897d0a292724480673f49
build.log a1394e2ec189fec7ec0f77e12fc999b0c9cc6e260aebb3f9f5de25d790d6e3c7
```

HEX 与 2026-10-03 的 M3 HEX 相同。此文件在固件归档后仅补记文档；M4 仍为设计准备。本次没有重新进行 host 回归、实板逐项验收、运行资源实测或 24 小时长稳，不能据此关闭任务或宣布稳定版。
