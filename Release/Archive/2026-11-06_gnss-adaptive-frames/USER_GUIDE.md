# GNSS Frames查看指南

对应执行台账R53、证据V1-E49。[本候选](MANIFEST.md)已通过32组选定host及Keil全量0错误0警告；尚未烧录或取得本轮实屏验收，不能据软件通过认定定位精度、运行资源或长稳通过。

## 操作顺序

1. 打开**定位/GNSS → Frames >**，进入GNSS Frames。
2. 默认`Auto RX`根据最近3s收到的完整、校验正确报文显示`NMEA`、`UBX`或`Mixed`。这个识别不需要Start，也不受Filter影响；无接收显示Waiting，过期明确显示Stale。
3. 按**Start/Pause**开始手动捕获。暂停只停止保存记录，真实接收和标题识别继续，已有记录保持。
4. **Filter >**切换Auto、NMEA全、UBX全或GGA/RMC/GSA/GSV/PVT/DOP/SAT。切换会创建新的暂停session，需要再次Start；旧session选帧不混入新会话。
5. **Messages**中用Next选择消息，再按**Select**进入Raw；Raw中的Next翻页，View切换Messages/Raw。

消息列表使用NMEA句型或UBX NAV-PVT、NAV-DOP、NAV-SAT、MON-VER、ACK-ACK/NAK名称。NMEA Raw提供ASCII及HEX；UBX Raw只显示完整HEX分页，并列出class/id、总长度和checksum。校验为BAD的捕获保留原字节，但不刷新Auto RX识别，也不能当有效定位数据。

标题是当前RX协议；`Selected`是选中的历史帧。即使标题后来变成UBX，已选NMEA仍按NMEA展示，不被重新标记或改写。

## 容量和计数

- 共4个原帧槽、16个消息汇总键；满槽覆盖属于诊断缓存行为，不等于GPS传输丢帧。
- 完整UBX wire上限392 B，NMEA原始行上限128 B，保留原校验和CR/LF。超长不会截断成完整帧；在前置Driver已经拒绝的超长数据不会进入本捕获的drop计数。
- 捕获可以保存BAD帧。Count、消息平均Hz和覆盖数描述捕获活动，不是有效定位次数、定位Hz或精度。
- 消息记录不保证无损全量保存；选中后是稳定副本，可完整翻页查看。

## 原功能与验证边界

GNSS本机串口保持115200。测量路径近期完整PVT优先，约3s内不让兼容NMEA覆盖其测量；原帧捕获和协议识别独立进行，Mixed不表示两份坐标同时融合。

电源第二组PA4/PC1、人工锁、原电机页面及USART1 Debug串口保留。本轮复用已有任务和静态区，不新增任务或heap分配；此前原帧观察配置的RTOS堆79.22%只是预算估算，超过75%目标，不能当本轮资源实测或长稳达标。若使用观察配置，仅短时查看，资源仍待验证。

本包固件为[cns-f407-gnss-adaptive-frames-20261106.hex](cns-f407-gnss-adaptive-frames-20261106.hex)，SHA-256：`d3a58c60bc47391557c51bf5a26e276e726d88cb8d2d2f433bf1e1e711c37766`。2026-11-06 14:23:32—14:23:54 UTC+8实际Keil Rebuild All完成，日志19s；1030项输入构建前后不变，主SRAM115928 B、余15144 B，较原帧观察基线增加560 B。详见[构建证据](build_evidence.json)。

本版保留Debug原帧观察配置和串口输出，尚无新栈水位/堆峰值/时序实测，因此仅作观察候选，不作为资源达标或稳定版放行。下一步按同候选检查实屏Start/Pause、Filter、选帧分页、协议切换与原功能回归；M5/M6及长稳待验项保持。
