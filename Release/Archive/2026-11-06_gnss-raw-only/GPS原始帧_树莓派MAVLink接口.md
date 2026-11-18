# GPS原始帧：树莓派MAVLink接口

2026-11-06。当前接口：固件已按raw-only配置。Pi接收GPS完整原始报文，STM32不向Pi发送GPS解析结果；本文只说明MAVLink封装及原字节恢复。

## 1. 串口与外层格式

UART6：PC6为STM32 TX、PC7为RX，**115200、8N1**。使用MAVLink2标准消息 **TUNNEL，msgid=385，CRC-extra=147**；sysid为设备Node ID（1—250），compid=193。接收端使用标准MAVLink库校验帧后再读TUNNEL。

| TUNNEL载荷偏移 | 类型 | 字段 | 当前值/含义 |
|---:|---|---|---|
| 0 | u16 | payload_type | **0x8020**，GPS原帧分片 |
| 2 | u8 | target_system | 0 |
| 3 | u8 | target_component | 0 |
| 4 | u8 | payload_length | 内层实际长度：24B头＋1—40B数据，即25—64B |
| 5 | u8[128] | payload | 内层内容，未使用尾部补0 |

所有多字节整数均为**小端**。MAVLink2可省略尾部0，标准库先补齐TUNNEL字段，再按payload_length取数据；不得对GPS原字节执行rstrip。若外层有签名，帧尾还含13B签名；CRC通过不代表已完成签名认证。

本候选Pi端停发解析型0x8011—0x8014及旧GPS_RAW_INT24、GLOBAL_POSITION_INT33、GNSS_SAT、GNSSUTC、BAROALT、含GPS数据的RemoteID镜像；**0x8021原帧统计默认关闭**。其它模块既有MAVLink消息可能仍在同一串口，Pi仅接收385/0x8020，其余可忽略。

## 2. 0x8020内层字段

**下表偏移0是TUNNEL.payload数组起点，不是MAVLink整帧起点；GPS原字节从内层偏移24开始。**

| 偏移 | 类型 | 字段 | 含义/规则 |
|---:|---|---|---|
| 0 | u8 | version | 固定1 |
| 1 | u8 | protocol | 1=NMEA，2=UBX |
| 2 | u8 | flags | bit0=源GPS校验有效；bit1=附带完整原帧CRC16，必须1；其余0 |
| 3 | u8 | header_length | 固定24 |
| 4 | u32 | frame_id | 原帧事件序号，同一帧各片一致；回绕0合法 |
| 8 | u32 | received_uptime_ms | STM32完成该帧消费/组帧的开机毫秒，0合法；不是GPS UTC或PPS时刻 |
| 12 | u16 | frame_length | 完整GPS原wire总字节数，含原协议头/校验/行尾 |
| 14 | u16 | fragment_offset | 本片在完整原wire中的偏移，必须=40×fragment_index |
| 16 | u8 | fragment_index | 片序号，从0开始 |
| 17 | u8 | fragment_count | 总片数，等于ceil(frame_length/40)，最多26 |
| 18 | u8 | data_length | 本片原字节数，等于min(40, frame_length-fragment_offset) |
| 19 | u8 | nmea_type | 0其他、1GGA、2RMC、3GSA、4GSV；UBX时为0 |
| 20 | u8 | ubx_class | UBX class；NMEA时为0 |
| 21 | u8 | ubx_id | UBX message id；NMEA时为0 |
| 22 | u16 | wire_crc16 | 对完整原wire计算的额外CRC16，不替代GPS原校验 |
| 24 | u8[data_length] | data | 本片原字节，不含额外补齐或终止符 |

flags.bit0仅报告原GPS校验事实。BAD源校验仍可传输，原NMEA校验或UBX CK_A/CK_B保持原样，不修正；校验正确也不等于定位有效。Pi仅恢复原wire，不在本接口中解析经纬度、卫星、DOP等GPS字段。

## 3. 拼片与校验

完整帧上限：**NMEA 128B、UBX 1024B**。每片最多40B，1024B对应26片（前25片各40B、最后24B）。NMEA保留实际`$`、正文、校验及行尾，收到CRLF就保留CRLF；UBX保留`B5 62`、class/id、长度、payload及实收CK_A/CK_B。超过上限不截短转发；噪声、半帧和超长候选不是完整帧，通道不保证整个UART字节流无损。

Pi按以下规则恢复：

1. 先验证每个MAVLink2帧的CRC147，筛选msgid385、payload_type0x8020及源sysid/compid。
2. 检查version=1、header_length=24、flags保留位为0、bit1为1，内层长度必须=24＋data_length。
3. 按`(sysid, compid, frame_id, received_uptime_ms)`建立组；同组protocol、frame_length、片总数、nmea_type、class/id、flags与wire_crc16必须一致，冲突则丢弃该组。
4. 检查index/count/offset/本片长度，按offset放入缓存。允许乱序和完全相同的重复片，禁止重叠冲突或空洞；重复片不延长组装期限。
5. 收齐所有片后，严格恢复frame_length个原字节，核对完整wire_crc16；通过才交给记录/转发程序，GPS原校验BAD也保持原字节。

额外CRC为**CRC16-CCITT-FALSE**：poly=0x1021、init=0xFFFF、refin=false、refout=false、xorout=0，覆盖完整GPS原wire；ASCII `123456789`检查值为0x29B1。它与每片MAVLink CRC是两层独立校验。

建议组装缓存有界，首片起5s未齐则丢弃；完成后短期去重。frame_id在源拒绝/队满时也推进，缺口不直接等于物理丢帧；设备重启可能复用序号与开机时间，连接/日志会话应另作区分。STM32队列4槽、队满丢新，不能承诺无损全量。

## 4. Pi接收伪代码

以下只拆封装，不解析GPS正文；`decode_raw_header`和`assembler.add`表示按上述表与规则实现的辅助函数，不是pymavlink现成接口。

```python
# mav为已打开的115200、8N1 MAVLink2接收连接
while True:
    msg = mav.recv_match(blocking=True)  # 标准库已校验MAVLink CRC
    if msg is None or msg.get_msgId() != 385:
        continue
    if msg.payload_type != 0x8020 or msg.get_srcComponent() != 193:
        continue
    inner = bytes(msg.payload)[:msg.payload_length]
    h, chunk = decode_raw_header(inner)  # 仅解析24B封装头并校验长度
    key = (msg.get_srcSystem(), msg.get_srcComponent(),
           h.frame_id, h.received_uptime_ms)
    wire = assembler.add(key, h, chunk)  # 元数据一致、齐片、CRC16通过
    if wire is not None:
        save_or_forward(wire)           # bytes原样；不修改GPS校验/CRLF
```

只接收不需要给GPS发配置命令，也不需要根据商品名猜芯片。完成的wire是GPS原始报文，MAVLink头、TUNNEL头及24B分片头都不属于GPS字节。
