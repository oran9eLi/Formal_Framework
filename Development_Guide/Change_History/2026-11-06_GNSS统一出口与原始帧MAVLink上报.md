# GNSS统一出口与原始帧MAVLink上报

最终构建补记：保留当前工程的设备包3.1.1更新后，于2026-11-06 16:26:53—16:27:16 UTC+8再次Rebuild All，0错误0警告、日志21s；1034项当前工程输入一致，HEX仍为`142625cf096ab67b9a16e79473a05bfb8b7ee8ac8c0a1ce948279682decf0958`。当前源码ZIP SHA-256为`c00f832a9879501b652f6d2e66fbabb459183cea696d80b38a74168429226753`；源码与固件已按本次构建匹配。


日期：2026-11-06。执行台账先R54 / V1-E50（typed统一出口），再R55 / V1-E51（独立RAW最终集成）。用途是记录两阶段真实实现、候选身份与验证边界；上位机详细字段分别见[typed对接](../2026-11-06_GNSS统一出口与树莓派MAVLink对接.md)和[RAW对接](../2026-11-06_GPS原始帧MAVLink转发与树莓派对接.md)。

## 阶段一：NMEA/UBX归一化上报

用户要求同一套上位机接口读取NMEA与UBX事实，先实现标准MAVLink2 TUNNEL385 / CRC147中的本地typed v1：0x8011 NAV64、0x8012 DOP64两片、0x8013 COUNTS32、0x8014 LINK64两片，共六包约1Hz。GNSS基础域、七项DOP及GPS/北斗严格NMEA计数分别保留来源/有效性/年龄，不用ACK续旧坐标；无fix状态、已知0和未知分开，UBX总used不猜星座拆分。

同组冻结bundle_id、snapshot_uptime_ms与观测，实际编码/重试时重算age/valid/stale；Pi六包同源同组齐全后发布，重复不能使数据返老，断流本地老化后发失效事件。无同步时钟不保证消除网络单向延迟；RTC系统时间不补GNSS UTC，RTC源sync_age=0无seen位时保持歧义，未知来源/epoch不编造真值。旧UART6遥测、专属目录、RemoteID、ACK、H1/B1保留，旧GNSS字段标legacy，不与新权威值拼用。

R54 / V1-E50的软件交付保存在[build/delivery](../../build/20261106_gnss_unified_uplink/delivery/)；35组选定C host、19项Python和真实C六包黄金交叉通过。唯一Keil工程Target 1实际Rebuild All **0错误0警告**、日志24s，15:04:16—15:04:43 UTC+8；1032项输入一致，主SRAM116744B、余14328B。阶段一HEX `cns-f407-gnss-unified-20261106.hex`，SHA-256 `05a54692aae5df225c1da484b880b49a7ac07fcee032f88589f77c2779570e9c`。完成阶段一后才开始RAW，不把前一观察版串口采样当此新通道已实现。

## 阶段二：独立原wire通道

新增0x8020 RAW_FRAME与0x8021 RAW_STATS，外层仍标准TUNNEL385；先按payload_type分流，不套typed头或source枚举。原wire包含NMEA实际行终止与UBX实收CK_A/CK_B，源BAD照传，CRC16-CCITT-FALSE核对完整封装字节，不修源校验。Python只确认源checksum事实，`usable_as_navigation_update=false`，不以RAW事件直接更新导航；标准GPS_RAW_INT24依旧是结构化旧消息。

4个静态FIFO槽，NMEA≤128B、UBX≤1024B（1016B payload），每片40B、最多26片。屏幕/USART1 Debug原392B观察缓存独立不扩容。队满丢新且活跃帧固定；每片最多3次、失败间隔50ms，耗尽或活跃超过2s中止整帧；owner维护超时不被H1/B1/ACK优先分支绕过。统计每次尝试后给已就绪数据下一RAW机会，四路机会轮转保留旧流与typed，不能承诺无损全量。帧序号在源拒绝/队满等input_event也递增，缺口不直接等于GPS物理丢帧。

Adapter是Sensor原wire唯一tee入口，Comm独占发送；没有新任务或运行期动态分配。固定GNSS 115200、测量源优先级、正常UI、Debug观察、电源和电机保护保留。最终相对阶段一11项生产变更（含新RAW头/源），见[清单](../../Release/Archive/2026-11-06_gnss-unified-raw-uplink/changed_production.json)。

## 最终验证与归档

最终38组选定C host完整重跑全部PASS；首轮只有测试helper仍按392B限制，妨碍合法长帧用例，修测试后重跑38组，生产未因此改变。Python31项PASS（原typed19+RAW12），覆盖RAW真实C三片+统计黄金、源BAD字节保真、64星NAV-SAT784B、1024B/26片、乱序/重试/冲突/CRC/超时、来源隔离和typed失效不受RAW续期；黄金是host合成，不是GPS现场记录。

唯一入口`MDK-ARM/formal_framework.uvprojx`、Target 1，ARMCC5.06 update7 build960 / C99，最终Rebuild All **0 Error(s), 0 Warning(s)**，日志21s；2026-11-06 UTC07:53:07—07:53:30，即**15:53:07—15:53:30 UTC+8**。1034项输入前后不变，保存同次HEX/AXF/MAP/HTM/日志与源码，不复用旧HEX。

Code/RO/RW/ZI为360760/111492/1988/120404B；主SRAM122392B、余8680B，比阶段一+5648B。RX DMA0x200085B0/1024B、TX0x200089B0/96B均在主SRAM。Flash装载472384B，HEX范围[0x08000000, 0x08073540)，记录校验、无重叠与sector11未写通过。

最终归档见[MANIFEST](../../Release/Archive/2026-11-06_gnss-unified-raw-uplink/MANIFEST.md)及[构建证据](../../Release/Archive/2026-11-06_gnss-unified-raw-uplink/build_evidence.json)。HEX `cns-f407-gnss-unified-raw-20261106.hex`，SHA-256 `142625cf096ab67b9a16e79473a05bfb8b7ee8ac8c0a1ce948279682decf0958`；源码ZIP SHA `c00f832a9879501b652f6d2e66fbabb459183cea696d80b38a74168429226753`。

## 验收边界

新最终包未烧录/实板验收，不因有原帧或host通过推断定位、精度、Pi接收成功、全量无损、时钟对齐或长稳。typed不含完整逐星数组；RAW保留NAV-SAT不等于已在业务实现星图。RTK/罗盘/PPS不新增，商品名不作为芯片识别证据。既有Debug堆估算79.22%超过75%目标，栈水位/堆峰值/实时性和主SRAM余量需实测；M5/M6不整体关闭。旧归档与阶段一build证据保留，Latest未更新，无.git提交/标签声明。此收尾只改文档，不重复Keil构建。

## R57/E53：恢复双流

按用户最新要求，解析GPS与原始帧同时发Pi，parsed/typed恢复默认1，raw1/stats0。真实6秒取得6个解析六包组、30个完整原帧；旧GPS及RID镜像恢复，其它功能保留。40组回归、Keil0/0/33s，接口文档仍一份，新固件未实板。
