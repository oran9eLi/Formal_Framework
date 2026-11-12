# GNSS报文页自适应NMEA与UBX

日期：2026-11-06。执行台账R53、证据V1-E49。基于固定115200原帧观察配置，将原NMEA报文页扩展为GNSS Frames；软件验证完成，本候选尚未烧录或取得实屏通过结论。

## 行为与边界

- 定位页入口NMEA >改为Frames >，复用页面编号、Messages/Raw两模式和现有控件池。
- Auto RX按最近3s的完整CRC正确报文显示NMEA/UBX/Mixed，Waiting/Stale明确；ready位区分合法0ms。识别不依Start、Filter或session，Pause只冻结记录，不冻结真实RX标题。
- Start手动保存；Filter保留原1GGA/2RMC/3GSA/4GSV编号，增加NMEA全、UBX全、PVT/DOP/SAT，按0/5/6/1/2/3/4/7/8/9轮转。改变后换新暂停session，需显式Start，旧token隔离。
- Summary依据协议/class/id或NMEA句型命名；Select经App取稳定原帧，NMEA ASCII+HEX、UBX完整HEX分页及class/id/len/checksum。选帧标history，与实时RX协议分开，Display不解析payload。
- 4帧槽/16汇总键，UBX完整wire392 B、NMEA原行128 B；超长不截断。前置Driver拒绝的超长不计capture drop，不能用捕获统计代表所有UART异常。BAD可保存但不续协议识别或授权测量，平均帧率不是定位Hz。

固定115200、PVT近期约3s测量优先级保持；原帧捕获与识别不受该测量选源门控。正常八项测量、Details、电源/电机和Debug原帧串口保留；不新增任务、控件池或运行期动态分配。UI年龄使用显示周期时基，仅短窗口前向快照夹0，真实陈旧年龄保留。

本轮8项生产变化涉及Sensor原帧回调、Framework捕获/Adapter、App包装和LVGL页面，未新增生产源文件，见[完整清单](../../Release/Archive/2026-11-06_gnss-adaptive-frames/changed_production.json)。

## 实际验证与归档

32组选定host统一PASS，覆盖双协议捕获、筛选/暂停/会话、392 B完整分页、协议识别0ms/回绕/陈旧边界及原测量/电源/电机保护；不称全库所有测试或实屏通过。

run1沙箱执行未找到ARMCC；不将其旧产物作为本次通过证据。run2通过已授权沙箱外RunAsInvoker方式，未触发UAC、未烧录，唯一工程`MDK-ARM/formal_framework.uvprojx`的Target 1实际Rebuild All完成，ARMCC5.06 update7 build960/C99，**0错误0警告**。2026-11-06 **14:23:32—14:23:54 UTC+8**，日志19s；1030项生产输入构建前后不变。

Code/RO/RW/ZI为352040/111476/1972/113956 B；主SRAM115928 B、余15144 B，比原帧观察基线增加560 B。RX DMA0x20006C70/1024 B、TX0x20007070/96 B位于主SRAM；Flash装载463648 B，HEX范围[0x08000000, 0x08071320)，校验、无重叠及sector11未写检查通过。

[归档MANIFEST](../../Release/Archive/2026-11-06_gnss-adaptive-frames/MANIFEST.md)及[使用指南](../../Release/Archive/2026-11-06_gnss-adaptive-frames/USER_GUIDE.md)保存同次run2固件、源码与验证。HEX为`cns-f407-gnss-adaptive-frames-20261106.hex`，SHA-256：`d3a58c60bc47391557c51bf5a26e276e726d88cb8d2d2f433bf1e1e711c37766`；源码ZIP SHA：`c4799b0fbf87b5ff98f66a37d424fcf2e292a2e9e5c17a64e5d24a973bf3bd02`。

## 未完成项

本候选未烧录或取得新实屏操作、栈水位、堆峰值和时序实测。Debug观察配置既有堆估算79.22%，超过75%目标，不能作为资源达标或长稳候选；仅短时观察，M5/M6和定位精度、断流恢复、资源长稳待验项不关闭。

旧普通版与观察版归档保留，Latest未更新，当前无.git提交/发布声明。本文收尾只写文档，不修改生产输入，不重复Keil构建。
