# GNSS MAVLink完整交付：板端与树莓派对齐

当前实现使用标准MAVLink2 **TUNNEL(msgid385，CRC_EXTRA147)**，按payload_type分流；无需新增MAVLink方言。实际发送是二进制，不是UART1 Debug的ASCII HEX文本。此前CNS自定义消息提案未采用。

| payload_type | 当前内容 |
|---|---|
| 0x8011 | NAV64，NMEA/UBX归一化位置、速度、海拔、UTC、状态及来源/有效位/年龄 |
| 0x8012 | DOP64两包，七项reported DOP与可用标志/来源/年龄/iTOW |
| 0x8013 | COUNTS32，NMEA来源四项GPS/北斗报文计数；未提供明确未知 |
| 0x8014 | LINK64两包，GPS USART2链路/配置、系统UTC及RTC/GNSS来源 |
| 0x8020 | 独立GPS完整原wire分片；每片最多40B，最多1024B/26片 |
| 0x8021 | 独立原帧队列/拒绝/发送完成/丢弃统计 |

以上是TUNNEL.payload_type，不是六种MAVLink msgid。旧电源、电机、环境、RemoteID、告警等消息仍保留；全部目录与字段在对接MD。旧GPS_RAW_INT24不是GPS原始串口字节，不投入原帧组装，也不与新权威数据交叉续期。

## Pi先离线核对

解压后在本目录运行，Python3.10+；离线仅标准库：

```text
python Tools/gnss_uart6_decoder.py --hex @Tools/Fixtures/typed_c_golden.hex
python Tools/gnss_uart6_decoder.py --hex @Tools/Fixtures/raw_c_golden.hex
python Tools/smoke_check.py
```

黄金帧来自真实C编码器的确定性host测试，不是现场GPS测量，不据此推断模块RTK能力、定位精度或实际帧频率。

## 连接实际串口

USART6：PC6 TX→Pi RX，PC7 RX←Pi TX，GND共地，兼容3.3V TTL。上电115200/8N1/无流控；设备文件按实际接法选择。GPS输入是另一个USART2，Debug文本是USART1。下例只接收，不发命令；串口模式可选安装pyserial：

```text
python Tools/gnss_uart6_decoder.py --port /dev/ttyUSB0 --baud 115200
```

真实端口也可能是Pi GPIO UART或其他USB编号，需按实际设备配置。既有B1实验可临时切USART6到57600，接收器需同步切速，GPS USART2仍115200。

## 上位机事件分流

`gnss_bundle`用于完整结构化快照；必须处理`gnss_metadata_update`和`gnss_invalidated`，按有效位/来源/年龄管理当前值，未知或失效为null。`gnss_raw_frame`仅原帧审计，`bytes_hex`重建为原GPS wire；`usable_as_navigation_update`恒false，不把源校验正确等同定位有效。`gnss_raw_stats`单独展示队列事实，不能续期typed数据。

已有pymavlink循环可将`bytes(msg.get_msgbuf())`作为完整MAVLink wire传给`Decoder.feed()`。若只拿`msg.payload`，它已是TUNNEL内部opaque数组；不要把它误传给期望带TUNNEL5B头的`decode_tunnel_payload()`。本文内层偏移从opaque数组首字节算，GPS原wire只由0x8020的data片段拼出。

Pi/服务器转发保留原MAVLink字节及源sysid/compid/seq/CRC；不要以Pi身份重新编码冒充源设备。当前没有唯一boot ID/端到端ACK保证，网络单向延迟未被消除；原帧队列4槽有界、源NMEA128/UBX1024，不承诺无损全量。原GPS坏校验会原样传，源校验、封装CRC16和MAVLink CRC是三件事。

## 板端代码与验证

外层完整交付包的firmware中包含最终HEX和production_source.zip。源码解压后唯一入口为MDK-ARM/formal_framework.uvprojx，Keil MDK5/ARMCC5.06/C99；代码已在当前工作区修改，非仅文档设计。

最终38组选定C回归、31项Python、C/Python黄金帧对齐通过；Keil全量0错误0警告。当前软件未烧录到实板、未验证真实Pi接收/吞吐/精度/栈/长稳。主SRAM静态122392B余8680B；保留Debug观察配置，原RTOS堆预算约79.22%超过75%目标，资源达标未关闭，仅作观察验证。
