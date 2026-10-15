# V1.0 SRAM 与显示刷新优化预发布存档

| 项目 | 记录 |
|---|---|
| 代码 / 注释标签 | `da6f6e5b1322d0b9a882d0d09d52962395014fd7` / `v1.0.0-pre.opt.20261015` |
| 优化实现 | `3a9ee21a8c5ea33231298e5030ff1942d667fe68`，合入最新 develop；生产源码与构建原始输入逐文件核对 |
| 版本入口 | [GitHub 预发布](https://github.com/oran9eLi/Formal_Framework/releases/tag/v1.0.0-pre.opt.20261015)，实板待测，不是稳定版或 M6 rc |
| 构建 | 2026-10-15 22:37:56—22:39:22 UTC+8，Rebuild All，退出码 0，0 Error(s), 0 Warning(s)，日志 72 s |
| 工程 / 工具链 | MDK-ARM/formal_framework.uvprojx / Target 1 / ARMCC 5.06 update5 build528 / DFP2.17.0 |
| Code / RO / RW / ZI | 338788 / 111008 / 1980 / 109708 B |
| 主 SRAM | 使用 111688 B，空余 19384 B（18.93 KiB）；较 M5 净释放 14768 B |
| Flash 装载 | 449924 B，较 M5 增加 528 B |
| 既有内存池 | FreeRTOS heap_4 40 KiB 主 SRAM、LVGL 40 KiB CCM；DMA 缓冲仍为主 SRAM |
| HEX 范围 | [0x08000000, 0x0806DD84)，数据 449924 B；逐条校验通过，不进入 sector11 |
| 生产输入 | 1021 项构建前后及打包时原始 SHA 一致，与源码提交的换行归一化内容逐项一致；原始输入 ZIP 已逐项核对 |

## GitHub 附件

| 文件 | 字节数 | SHA-256 |
|---|---:|---|
| Formal_Framework-v1.0.0-pre.opt.20261015-build.zip | 9386831 | `2be20ae29c9e98f4a0bc7d67ab4ed8966fce95cf23bfe9b8ca0afb75c5c15302` |
| formal_framework.hex | 1265574 | `d90a754525756fd8aaf47fa58ea07f31340beaec2739ab3907cba02033413f95` |
| MANIFEST.json | 63139 | `d5cc43b4e0a0d24d4cb7d074610bf110c981108f090396074fbc12f36b98c288` |
| SHA256SUMS.txt | 287 | `4f7f881ca78ca6275826bc899ddc1105f81a1743c9298ddb857e6acd47d8d9c9` |

构建 ZIP 包含同次 HEX、AXF、MAP、HTM、构建日志、前后生产输入清单、原始输入 ZIP、工程配置、工具链及身份。独立 HEX 与优化分支此前交付的 HEX 一致。

同次 AXF SHA-256：`61ab009ee1b415b70aa370301380067303637796545ed5d37c9166b33640a508`。
MAP SHA-256：`a284dd74a60c1dbaead227aded383c13874ed5560851d7f78e85dcbda30ccf76`。
日志 SHA-256：`e4ad6f5fb54e75d08dfe54c025d09bcd20b1dbc40db7694965ac1243d66ed6a9`。
输入清单 SHA-256：`262c127524945afdafe85ea4ebfc4b0e4d2e9c3c823484b8ae632de0c23ae568`。

## 范围与验收

四项 SRAM 精简与显示重复刷新收敛已实现，原功能、交互、有效性、Freeze 和异步反馈规则保留。UART1 CLI 尚未实现，UART6 保持原流程。独立审查无 Critical/Important，一项 Comm Busy 标题重复刷新 Minor 留待后续，不影响反馈正确性。

本次未新增或运行主机测试、烧录、实屏/计时、串口/RF、运行资源或长稳。用户后续测试，M5/M6、M5-R01 和前序必需待验不关闭。烧录保留 sector11，整片擦除会清空 Node ID 配置。

构建与附件核对证据保存在本次集成工作区 `build/opt_release_20261015/`。本机原工作区的 AGENTS、uvoptx、Reports、tmp 及文档草稿保留。

详见 [合入与归档记录](../../V1.0_SRAM与显示精简_合入与归档记录.md)、[技术记录](../../../Development_Guide/2026-10-15_SRAM与显示刷新精简说明.md)。

四个GitHub附件已在草稿及公开发布后分别回下载，字节数和SHA-256全部一致。Release为已发布Pre-release（非Draft、非Latest），目标提交与标签源码一致。
