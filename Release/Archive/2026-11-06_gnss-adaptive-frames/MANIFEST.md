# GNSS报文页自适应NMEA与UBX候选

日期2026-11-06，执行台账R53、V1-E49。配置`gnss_adaptive_frames_raw_debug`，基于[固定115200原帧观察版](../2026-11-06_p25-fixed115200/raw-debug/MANIFEST.md)。32组选定host及实际Keil全量构建已通过；**本候选未烧录或取得实屏、资源和长稳验收**，不关闭M5/M6，不作为稳定版发布。

- [本次HEX](cns-f407-gnss-adaptive-frames-20261106.hex)、[使用指南](USER_GUIDE.md)、[构建证据](build_evidence.json)。
- [开发日志](../../../Development_Guide/Change_History/2026-11-06_GNSS报文页自适应NMEA与UBX.md)、[8项生产改动](changed_production.json)、[源码ZIP](production_source.zip)。
- [host结果](verification/host_results.json)、[UI复核](verification/display_review.json)、[集成复核](verification/integration_review.json)、[实际成功构建](verification/keil_run2_result.json)。

## 已实现行为

定位页入口为Frames >，复用旧NMEA页面和既有Messages/Raw两模式。默认Auto RX依据最近3s完整且校验正确的报文显示NMEA、UBX或Mixed；无有效接收Waiting，过期Stale。识别是常驻接收事实，不依赖Start、Filter或会话；暂停只停捕获记录，实时RX标题继续，选中帧仍是独立历史。

Start手动捕获，Filter轮转Auto/NMEA全/UBX全/GGA/RMC/GSA/GSV/PVT/DOP/SAT，改变后建立新的暂停session，需再次Start。消息按协议元数据命名，Select经App API取稳定副本；NMEA ASCII+HEX，UBX完整HEX分页和class/id/len/checksum，Display不解析payload。

保持4帧槽、16汇总键，UBX完整帧上限392 B、NMEA原行128 B；超长不截断，前置Driver拒绝的超长也不会进入capture drop计数。BAD可捕获但不刷新Auto识别或授权测量，Count/平均Hz是捕获活动，不等于定位Hz或精度。

本机GNSS仍固定115200，近期PVT约3s优先于NMEA测量；捕获和识别独立进行。正常八项测量、Details、电源/电机和Debug原帧串口保留，无新增任务、控件池或运行期heap分配。相对基线8个生产文件变化，无新增生产源文件。

## 实际构建及资源

唯一入口`MDK-ARM/formal_framework.uvprojx`、Target 1，ARMCC5.06 update7 build960/C99。run1在沙箱环境未找到ARMCC，没有形成该候选的有效通过证据；run2通过已授权的沙箱外RunAsInvoker方式执行，无UAC弹窗、未烧录板卡，实际Rebuild All **0 Error(s), 0 Warning(s)**。时间为2026-11-06 **14:23:32—14:23:54 UTC+8**（UTC06:23:32—06:23:54），日志19s。1030项输入构建前后不变，归档使用run2同次HEX/AXF/MAP/HTM/日志，不复用run1或旧HEX。失败环境记录见[aborted_sandbox_build.json](verification/aborted_sandbox_build.json)。

| 项目 | 结果 |
|---|---|
| Code / RO / RW / ZI | 352040 / 111476 / 1972 / 113956 B |
| 主SRAM使用 / 余量 | 115928 / 15144 B；较原帧观察基线增加560 B |
| RX DMA / 静态TX | 0x20006C70 / 1024 B；0x20007070 / 96 B，均在主SRAM |
| Flash装载 / HEX范围 | 463648 B；[0x08000000, 0x08071320) |
| HEX检查 | 记录校验通过、无重叠、sector11未写 |
| HEX SHA-256 | d3a58c60bc47391557c51bf5a26e276e726d88cb8d2d2f433bf1e1e711c37766 |
| 源码ZIP SHA-256 | c4799b0fbf87b5ff98f66a37d424fcf2e292a2e9e5c17a64e5d24a973bf3bd02 |

32组选定host全部PASS，不代表全库所有测试或实屏验收。保留Debug观察配置的既有RTOS堆估算79.22%，高于75%目标；本轮未取得新堆峰值、栈水位或实时性实测，不能声称资源或长稳达标，仅用于观察验证。

Latest未更新，旧归档和普通清屏版保留；当前无.git，不声明提交、推送或标签。文档收尾不改变固件输入，不重复构建；最终SHA清单由主代理统一生成。
