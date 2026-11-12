# GPS解析与原始帧：树莓派MAVLink接口

2026-11-09。当前候选默认同时发送**解析数据＋GPS原始帧**（parsed policy=1、typed=1、raw=1、stats=0）。箱体实验功能新增本地授时/逐星显示及UBX NAV-SAT的1Hz输出；下述消息ID、载荷布局及有效性规则保持。Pi使用0x8011—0x8014作为解析业务权威来源，0x8020用于保留GPS原wire，不用RAW覆盖解析业务值。

## 1. 串口、外层与消息分类

UART6：PC6=STM32 TX、PC7=RX，**115200、8N1**。MAVLink2标准TUNNEL **msgid=385、CRC-extra=147**；sysid=设备Node ID（1—250），compid=193，target_system/target_component=0/0。所有多字节数均为**小端**，i32为有符号二补码。

| TUNNEL载荷偏移 | 类型 | 字段/含义 |
|---:|---|---|
| 0 | u16 | payload_type：下表本地类型，不是MAVLink消息号 |
| 2 / 3 | u8 / u8 | target_system / target_component，当前0/0 |
| 4 | u8 | payload_length：内层实际长度 |
| 5 | u8[128] | payload：内层数据，其余尾部0 |

**下文偏移0均为TUNNEL.payload数组起点，不是MAVLink整帧起点。** 标准库验MAVLink CRC后先补齐可能被MAVLink2裁剪的尾零，再按payload_length取内层；不能rstrip GPS原字节。签名帧另有13B签名，无密钥时不能把CRC通过当已验签。

| payload_type | 内容 | 内层长度/分包 |
|---|---|---|
| 0x8011 | NAV，位置/速度/UTC/状态/HDOP | 64B，1包，kind=1 |
| 0x8012 | 七项DOP | 64B，2包，kind=2 |
| 0x8013 | GPS/北斗分类计数 | 32B，1包，kind=3 |
| 0x8014 | LINK，接收/配置/系统时间事实 | 64B，2包，kind=4 |
| 0x8020 | 完整NMEA/UBX原wire的分片 | 24B头＋1—40B原字节 |

0x8021统计默认关闭。旧GPS_RAW_INT24、GLOBAL_POSITION_INT33、NAMED_VALUE_INT252的GNSS_SAT/GNSSUTC/BAROALT恢复兼容发送，GPS相关RID12901/12904镜像也可能出现；**这些旧消息不与新权威字段混用或续期**。旧GPS24.time_usec是源位置开机微秒；旧GNSSUTC.time_boot_ms实际是UTC日内秒、来源可为RTC，不是开机毫秒；旧GNSS_SAT不能当UBX真实星座分类计数。其它模块消息可同串口出现，完整旧目录不在本文重复。

## 2. 解析包公共头、来源和有效性

| 内层偏移 | 类型 | 字段/含义 |
|---:|---|---|
| 0 / 1 | u8 / u8 | version=1 / kind=1—4 |
| 2 / 3 | u8 / u8 | part_index从0开始 / part_count（见上表） |
| 4 | u32 | bundle_id：同组六包一致，非唯一boot ID |
| 8 | u32 | snapshot_uptime_ms：整组冻结的本机开机毫秒，0合法 |

约1Hz冻结一组；按(sysid, compid, bundle_id, snapshot_uptime_ms)收齐NAV＋DOP两片＋COUNT＋LINK两片后替换完整组，不能跨组/跨源拼接。新1s周期会取消未发完的旧组，未齐不能永久等候或与新组补齐；重启/回绕需清理旧缓存。

mask位=1才授权对应含义：present=0未知；present=1且valid=0不可用于业务；stale=1为旧值。无效/未知工程值输出null，不能把wire的0占位当真值。**已授权的0坐标、0卫星、午夜0秒及0ms都合法**；新状态不能复活旧坐标。age16：0xFFFF未知，0xFFFE表示≥65534ms，其他为毫秒；valid要求age≤1500ms。Pi还需加本地等待年龄，断流后主动失效；编码时有效不保证网络到达时仍新鲜。

年龄按**各片本次编码时刻**重算，header冻结时刻不变，不能由snapshot减age反推源时刻；各域不保证同物理历元。source枚举：0未知、1 UBX_PVT、2 NMEA_GGA、3 NMEA_RMC、4 NMEA_GSA、5 NMEA_GSV、6 UBX_DOP；不是“最后收到的协议”。所有保留位/保留字节为0。

## 3. 0x8011 NAV64

mask bits0—5依次为position、velocity、time、altitude、navigation_status、hdop。sources_pack中前五域依次各占3bit（位0—14），bit15保留；HDOP单独source。

| 偏移 | 类型 | 字段/单位或含义 |
|---:|---|---|
| 12 / 13 | u8 / u8 | valid_mask / stale_mask |
| 14 | u16 | sources_pack，前五域来源 |
| 16 / 17 / 18 | u8各1 | fix_quality_gga / fix_dimension / satellites_used（总used颗数） |
| 19 | u8 | hdop_source |
| 20 / 24 | i32各1 | latitude_e7 / longitude_e7，度×10⁷，南/西负 |
| 28 | i32 | altitude_msl_mm，海拔毫米 |
| 32 | u32 | ground_speed_cms，水平地速cm/s |
| 36 | u16 | course_cdeg，航迹角0.01°，有效范围0—35999 |
| 38 | u16 | hdop_x100，因子×100 |
| 40 / 44 | u32各1 | utc_date_yyyymmdd / utc_second_of_day（0—86399s） |
| 48 / 50 / 52 / 54 / 56 / 58 | u16各1 | position / velocity / time / altitude / status / hdop各自age_ms |
| 60 | u16 | link_age_ms：距最后有效协议帧年龄，不是测量年龄 |
| 62 / 63 | u8 / u8 | present_mask / raw_status_type |

位置/高度/状态来源只0/1/2，速度/UTC只0/1/3；HDOP只0/2/4/6。坐标有效范围纬±90°、经±180°。UTC有效日期2000—2099年；NAV UTC仅GPS测量时间，不用LINK RTC填补。

fix_quality_gga：0无fix、1普通GNSS、2差分、3PPS、4RTK fixed、5RTK float、6估算/DR、7手工、8模拟（枚举不代表本机具备对应硬件能力）；普通位置仅按valid位授权。fix_dimension=0未知、2二维、3三维。raw_status_type按状态source解释：PVT为fixType 0无fix/1仅DR/2二维/3三维/4 GNSS+DR/5仅时间；GGA为原质量码0—8。状态valid且质量0是合法NoFix，satellites_used=0也是已知0；不能因此授权位置。

## 4. 0x8012 DOP64，两片

偏移12为u8 entry_count：part0=4（id0—3），part1=3（id4—6）；13—15保留0。条目从16开始，每项12B，下面偏移为**条目内偏移**；未用尾部为0。

| 条目偏移 | 类型 | 字段/含义 |
|---:|---|---|
| 0 / 1 / 2 / 3 | u8各1 | id / source / flags / reserved=0 |
| 4 | u16 | reported_value_x100：源报告DOP×100；present时1—65534 |
| 6 | u16 | age_ms，遵循age16规则 |
| 8 | u32 | itow_ms：GPS周内毫秒，仅itow_known有效，范围0—604799999 |

id0—6=G/P/T/V/H/N/E DOP。flags bit0 present、bit1 valid、bit2 stale、bit3 itow_known，其余0。reported是审计值，**valid=0时可用DOP仍为null**，不借全局fix把无解句的DOP变有效；DOP因子不是米级精度承诺。NMEA无iTOW，bit3=0时wire零为占位，业务epoch=null，不假GPS周起点。合法source：G/T/N/E只0/6，P只0/1/4/6，V只0/4/6，H只0/2/4/6。

## 5. 0x8013 COUNTS32

| 偏移 | 类型 | 字段/含义 |
|---:|---|---|
| 12 / 13 / 14 / 15 | u8各1 | present_mask / valid_mask / stale_mask / reserved=0 |
| 16 / 17 / 18 / 19 | u8各1 | gps_visible / bds_visible / gps_used / bds_used，颗数 |
| 20 / 21 / 22 / 23 | u8各1 | 上述四项各自source |
| 24 / 26 / 28 / 30 | u16各1 | 上述四项各自age_ms |

mask bits0—3依次对应这四项。visible来源只0/5GSV，used只0/4GSA；只采用明确GPS/北斗NMEA事实。只有UBX总used时分类未知，不按总数拆分；present/valid授权下0是真实已知0。

## 6. 0x8014 LINK64，两片

part0：下面计数均为GNSS **USART2**接收事实，不是UART6发往Pi的接收率或丢包统计。

| 偏移 | 类型 | 字段/单位或含义 |
|---:|---|---|
| 12 | u8 | diagnostic_flags，定义见下 |
| 13 / 14 | u8 / u8 | receiver_protocol / receiver_state |
| 15 / 16 / 17 | u8各1 | config_step / config_total / config_confirmed（0/1） |
| 18 / 20 | u16 / u32 | protocol_version_x100（如3410=34.10）/ gnss_baud_bps |
| 24 / 28 / 32 | u32各1 | rx_bytes（Driver消费字节）/ valid_nmea / valid_ubx（有效协议帧数） |
| 36 / 40 | u32各1 | checksum_errors / framing_resets，事件次数 |
| 44 / 48 / 52 | u32各1 | rx_drop_bytes / uart_errors / dma_errors |
| 56 / 60 | u32各1 | last_byte_age_ms / last_frame_age_ms；FFFFFFFF未知，0合法 |

flags bit0 Driver新鲜可用、bit1 transport可用、bit2见过字节、bit3见过有效帧、bit4见过版本、bit5 UART/DMA初始化完成、bit6 TX忙，bit7保留。Driver状态/计数按bit0授权，BSP计数按bit1，版本另需bit4；缺授权为未知，不能把默认0当实测。protocol=0未知/1NMEA/2UBX；state=0探测/1版本/2配置/3接收/4未确认/5IO错误/255未知。Receiving或config_confirmed不证明已定位。

part1：

| 偏移 | 类型 | 字段/单位或含义 |
|---:|---|---|
| 12 / 13 / 14 / 15 | u8各1 | time_available / system_time_source / system_time_sync_state / reserved=0 |
| 16 | u32 | time_sync_age_s，距最近GNSS校准秒数，注意下述0歧义 |
| 20 / 24 / 28 / 32 | u32各1 | tx_commands / ack_count / nak_count / config_timeout_count，Driver次数 |
| 36 / 40 / 44 / 48 | u32各1 | tx_packets / tx_failures / recoveries / transport_rx_bytes，USART2事实 |
| 52 / 54 | u16 / u16 | dma_ndtr_bytes / queued_bytes |
| 56 / 60 | u32各1 | system_utc_date_yyyymmdd / system_utc_second_of_day |

time_available=0时本系统时间业务字段全部null；=1才可用，C端系统时间副本门限2500ms。source=0未知/1RTC/2GNSS；sync_state=0无效/1RTC有效/2已GNSS校准/3过期。RTC且sync_age_s=0可能从未校准，也可能刚校准不足1s，wire无seen位，保留报告0但可用同步年龄为未知；不能据此声称刚GPS授时。LINK系统UTC绝不代填NAV GNSS UTC。所有累计计数u32自然回绕；无同步时钟不能消除单向网络延迟。

## 7. 0x8020 原帧分片

RAW有独立frame_id＋received_uptime_ms，不与parsed bundle混组，也不覆盖解析业务。以下偏移从本RAW内层起点算；protocol 1=NMEA、2=UBX，**不同于解析source枚举**。

| 偏移 | 类型 | 字段/含义 |
|---:|---|---|
| 0 / 1 / 2 / 3 | u8各1 | version=1 / protocol / flags / header_length=24 |
| 4 / 8 | u32各1 | frame_id / received_uptime_ms（Sensor完整消费时刻，非UTC/PPS，0合法） |
| 12 / 14 | u16 / u16 | frame_length / fragment_offset=40×index |
| 16 / 17 / 18 | u8各1 | fragment_index从0 / fragment_count=ceil(length/40) / data_length=1—40 |
| 19 | u8 | nmea_type：0其他、1GGA、2RMC、3GSA、4GSV；UBX为0 |
| 20 / 21 | u8 / u8 | ubx_class / ubx_id；NMEA为0 |
| 22 | u16 | wire_crc16：完整GPS原wire的CRC16-CCITT-FALSE |
| 24 | u8[data_length] | 原字节，本片内层长度=24＋data_length |

flags bit0源GPS校验有效、bit1额外CRC16存在（必须1），其余0。源BAD也保留实收NMEA校验/UBX CK_A/CK_B，不修字节；源校验正确不等于导航有效。NMEA保留实际$、校验与行尾，收到CRLF就保留CRLF；UBX包含B5 62、class/id、长度、payload和原校验。

NMEA最大128B，UBX完整wire最大1024B，每片≤40B，最多26片（1024B末片24B），末片长度=min(40,total-offset)。按源sysid/compid＋frame_id＋received_ms组装，全组protocol/长度/总片数/flags/类型/CRC16一致；检查固定offset、片序号、长度，允许乱序与相同重复，冲突/空洞拒绝。收齐后校验整帧CRC16：poly1021/initFFFF/refin=false/refout=false/xorout0，ASCII“123456789”=29B1；与MAVLink CRC147独立。通过后交出完整原bytes，不额外解析GPS正文。

建议首片5s未齐丢弃、缓存有界并短期去重；4槽发送队列满时丢新，超长不截断，噪声/半帧不构成完整wire，不保证全UART无损。frame_id在拒绝/队满时也推进，缺口不等于物理丢帧；重启可能复用序号/时刻。

## 8. Pi分流伪代码

下面只表示处理顺序，辅助函数按本表实现；解析业务必须在断流/等待后重新老化，不可永久保留最后有效坐标。

```python
while True:
    msg = mav.recv_match(blocking=True)  # 标准MAVLink2库先验CRC
    if msg is None or msg.get_msgId() != 385 or msg.get_srcComponent() != 193:
        continue
    inner = bytes(msg.payload)[:msg.payload_length]
    if msg.payload_type in (0x8011, 0x8012, 0x8013, 0x8014):
        typed_assembler.add(msg.get_srcSystem(), inner, msg.payload_type)
        # 同源同bundle六包齐全；按mask/source/age发布，未知值null
    elif msg.payload_type == 0x8020:
        raw_assembler.add(msg.get_srcSystem(), inner)
        # 同原帧齐片＋CRC16通过后保存完整bytes，不更新parsed导航
```

## 9. 箱体实验新增的原帧来源

识别为支持的UBX接收机后，课堂配置申请NAV-PVT、NAV-DOP、NAV-SAT各1Hz；USART2/6仍为115200。输出以模块实际接收事实为准，配置确认不代替收到逐星帧。NMEA接收保留，不新增上位机命令或载荷类型。

NAV-SAT经既有`0x8020`发送，protocol=2、class=0x01、id=0x35。64条卫星记录的完整UBX帧为784B，对应20片。先重组原帧并验原UBX校验；其UBX payload长为`8+12×numSvs`：

| payload内位置 | 内容 |
|---|---|
| 0—3 / 4 / 5 | u32 iTOW（GPS周内ms）/ u8 version=1 / u8 numSvs |
| 6—7 | 保留字节 |
| 第n项起点`8+12n`后0 / 1 / 2 | u8 gnssId / svId / cno（dB-Hz，0保留为源报告值） |
| 第n项后3 / 4—5 | i8仰角 / i16方位角，单位度；仰角超±90则两角未知，方位另需0..360 |
| 第n项后6—7 / 8—11 | i16伪距残差（0.1m）/ u32 flags，低3bit为qualityInd，bit3为svUsed |

本机LCD只保存最多64项并分页显示；原帧上传仍按原字节，不能把LCD截断当作原帧截断。NAV-SAT是已知可见或跟踪列表，不等于全部已锁定；svUsed不授权位置。`0x8013`仍只按原NMEA来源语义报告分类计数，未把UBX列表混入旧字段。屏幕Hold只定格当前5项，不停止接收、解析或上报。
