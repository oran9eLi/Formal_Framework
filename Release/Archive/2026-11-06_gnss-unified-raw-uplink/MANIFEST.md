# GNSS统一出口与独立原帧MAVLink上报候选

最终构建补记：保留当前工程的设备包3.1.1更新后，于2026-11-06 16:26:53—16:27:16 UTC+8再次Rebuild All，0错误0警告、日志21s；1034项当前工程输入一致，HEX仍为`142625cf096ab67b9a16e79473a05bfb8b7ee8ac8c0a1ce948279682decf0958`。当前源码ZIP SHA-256为`c00f832a9879501b652f6d2e66fbabb459183cea696d80b38a74168429226753`；源码与固件已按本次构建匹配。


日期2026-11-06，执行台账R55、V1-E51；配置`gnss_unified_and_raw_uplink`。先完成R54 / V1-E50的typed统一出口，再独立增加RAW通道，未将typed工程量冒充原始字节。本候选38组选定C host、31项Python及实际Keil全量构建通过；**未烧录或取得本轮UART6/GPS/Pi实板、资源和长稳验收，不是稳定版，不关闭M5/M6。**

- [本次HEX](cns-f407-gnss-unified-raw-20261106.hex)、[构建证据](build_evidence.json)、[11项生产差异](changed_production.json)、[生产源码ZIP](production_source.zip)、[host源码ZIP](host_tests_source.zip)。
- [typed字段与UART6完整目录](../../../Development_Guide/2026-11-06_GNSS统一出口与树莓派MAVLink对接.md)、[独立RAW对接说明](../../../Development_Guide/2026-11-06_GPS原始帧MAVLink转发与树莓派对接.md)、[开发日志](../../../Development_Guide/Change_History/2026-11-06_GNSS统一出口与原始帧MAVLink上报.md)。
- [完整host结果](verification/host_results.json)、[RAW队列/调度复核](verification/raw_review.json)、[Sensor转发复核](verification/hooks_review.json)、[实际Keil结果](verification/keil_run1_result.json)、[重构建日志](rebuild.log)。

## 行为与边界

UART6启动115200 8N1，MAVLink2标准TUNNEL385 / CRC-extra147。本地typed v1的0x8011 NAV、0x8012 DOP、0x8013 COUNTS、0x8014 LINK保持六包合同、来源和独立时效，未知/NoFix/STALE不伪造工程值；RTC系统时间不代填GNSS UTC，NMEA没有iTOW时不假0。旧遥测、专属目录、RemoteID、COMMAND_ACK和H1/B1兼容保留，旧GNSS消息不与新权威域混用。

独立0x8020 RAW_FRAME、0x8021 RAW_STATS转发完整NMEA/UBX原wire，GPS坏校验照传、不重算修正；整帧CRC16-CCITT-FALSE与每片MAVLink CRC独立核对。RAW不是标准GPS_RAW_INT(24)，不进入typed导航缓存，Pi事件`usable_as_navigation_update`恒false，源校验通过也不证明定位有效。

RAW静态4槽、最大1024B完整wire、40B数据分片、最多26片；NMEA仍128B，UBX payload1016B。屏幕和USART1 Debug旧392B观察缓存独立保留。超长、半帧、噪声/重同步只能记源拒绝事件；队满丢新、活跃帧固定，每片最多3次/50ms重试，耗尽或活跃超过2s中止整帧。统计与数据有界交替，和旧遥测/专属/typed四路公平，不承诺无损全量；序号缺口、source_reject或本机UART成功均不证明物理丢包率或Pi接收成功。无新增任务或运行期heap分配；电源/电机既有功能保留。

## 实际构建、输入与校验

唯一入口`MDK-ARM/formal_framework.uvprojx`、Target 1，ARMCC5.06 update7 build960 / C99，实际Rebuild All **0 Error(s), 0 Warning(s)**。UTC07:53:07—07:53:30，即2026-11-06 **15:53:07—15:53:30 UTC+8**，日志21s。1034项生产输入构建前后不变，归档保存同次HEX/AXF/MAP/HTM和日志；阶段二相对typed阶段一11项生产变化，其中新增Framework RAW头/源文件。

| 项目 | 结果 |
|---|---|
| Code / RO / RW / ZI | 360760 / 111492 / 1988 / 120404 B |
| 主SRAM静态 / 余量 | 122392 / 8680 B；较阶段一增加5648 B，较自适应报文页增加6464 B |
| GNSS RX DMA / 静态TX | 0x200085B0 / 1024 B；0x200089B0 / 96 B，均位于主SRAM |
| Flash装载 / HEX范围 | 472384 B；[0x08000000, 0x08073540) |
| HEX检查 | 记录校验通过、无重叠、sector11未写 |
| HEX SHA-256 | 142625cf096ab67b9a16e79473a05bfb8b7ee8ac8c0a1ce948279682decf0958 |
| 生产源码ZIP SHA-256 | c00f832a9879501b652f6d2e66fbabb459183cea696d80b38a74168429226753 |
| host源码ZIP SHA-256 | fc588ca4490027892d9096a7238674e276ab4137ac3641a41f4fc5fdffaa77e1 |

38组选定C host完整重跑通过，31项Python通过（原typed19+RAW12），真实C六包typed及RAW三片+统计黄金与Python交叉通过，非全库或实板验收声明。首次host仅测试helper旧392B限制阻碍合法长帧，修测试后重跑全部38组，生产未因此变更。

阶段一R54 / V1-E50保留在[build交付目录](../../../build/20261106_gnss_unified_uplink/delivery/)：35组C、19项Python、Keil0/0、日志24s；HEX SHA `05a54692aae5df225c1da484b880b49a7ac07fcee032f88589f77c2779570e9c`。本包是其最终集成后继，不复用阶段一HEX作为新产物。

既有Debug观察配置RTOS堆估算79.22%高于75%目标，未取得新栈水位/堆峰值/WCET和板上资源证据，资源门禁未关闭。定位精度、时间对齐、断流热插拔、GPS/Pi持续收发与长稳仍待验，不因host/构建通过整体关闭M5/M6。旧归档保留，Latest未更新，当前无.git提交或标签声明。本文收尾仅改文档，不重复构建；最终SHA256SUMS由主代理统一生成。
