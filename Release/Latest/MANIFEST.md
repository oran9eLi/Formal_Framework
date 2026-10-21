# 姿态仪无数据灰显本地候选

日期：2026-10-21。候选：attitude-no-data-20261021。状态：本地软件构建完成，实板待验证，未推送/发布；原优化标签保持不变。

| 项目 | 实际记录 |
|---|---|
| 分支 / 生产提交 | codex/fix-attitude-no-data / `579a3d2e77060e7de94320791c981fa46f6c9f91` |
| develop基线 | `22e37356b822039690604c685aabc53d78a1af36` |
| 构建 | 2026-10-21T19:41:02.7109956+08:00 — 2026-10-21T19:41:47.5187363+08:00；Keil Rebuild All，退出码0，0 Error(s), 0 Warning(s)，日志37s |
| 工程 / 工具链 | MDK-ARM/formal_framework.uvprojx / Target1 / ARMCC5.06 u5 build528 / DFP2.17.0 |
| Code / RO / RW / ZI | 339412 / 111008 / 1980 / 109708 B |
| 主SRAM | 使用111688 B、余19384 B，与优化基线一致 |
| Flash装载 | 450548 B，比优化基线增加624 B |
| HEX范围 | [0x8000000, 0x806dff4)；450548 B；逐条校验和/地址核对，不占Node ID sector11 |
| 输入 | 1021项构建前后及归档时原始SHA一致、与生产提交内容逐项一致；原始输入ZIP逐文件核对 |
| 源码ZIP SHA-256 | `4c919e2af69de9e1dee23b14c14a335dc851a0f91aa0f624e18d9e52b9a88f3c` |

## 同次产物 SHA-256

| 文件 | SHA-256 |
|---|---|
| formal_framework.hex | `833407579aa9046420508cb61220732b1b56b8ea3ae93382572fb8ee88eebcf5` |
| formal_framework.axf | `a1215f4ce3d933e50115dd5d49d27b1bc735c42dfe598c6f3e6b03f585904af4` |
| formal_framework.htm | `7aa201a132de3c2bbfe5392ec426d2836e636f8b011c21c0f59c41dbb6d0a6b8` |
| formal_framework.map | `e3fa36b32db6d5de3e1cfc98d474f5b9e5f1f363b9cc980a7274f6edb5c9af77` |
| keil-rebuild.log | `ccbb9665cb0de0845f9184038356023bd54ec24805f326b5a11b99316784c9fb` |

无有效姿态时保留灰色仪表和NO DATA，角度仍为N/A；恢复新鲜有效姿态后回到原彩色绘制。原Freeze/Resume观察规则、Observe曲线、IMU采集/解算/控制/记录均不修改。没有新增对象、持久缓冲、任务或堆分配；运行栈和灰盘实际绘制耗时尚未实测。

未新增/运行host测试，未烧录或完成实屏、数据恢复、组合资源及长稳验收；M5/M6与M5-R01保持待验。软件审查无阻断不构成实板通过。详见执行台账V1-E24及2026-10-21变更记录。
