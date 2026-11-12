/**
 * @file test_gnss_unified_uplink.c
 * @brief 真实GNSS规范化编码与MAVLink UART6调度回归，不连接串口或硬件。
 * @details 确定性Framework副本覆盖无定位、过期、来源、零值、冻结、重试和三路公平。
 */
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "../../Framework/Src/px4lite_gnss_uplink.c"
#include "../../Framework/Src/px4lite_mavlink_tx.c"

static Px4Lite_SensorGnss_t source_gnss;
static Px4Lite_GnssDiagnostics_t source_diag;
static Px4Lite_TimeSnapshot_t source_time;
static Px4Lite_Result_t source_gnss_result=PX4LITE_OK,source_diag_result=PX4LITE_OK,source_time_result=PX4LITE_OK;
static uint32_t clock_ms,gnss_reads,diag_reads,time_reads,uart_calls,lora_calls;
static uint8_t uart_bytes[280];static uint16_t uart_length;
static Px4Lite_Result_t uart_result=PX4LITE_OK;
static uint8_t b1_pending,h1_pending,advance_during_copy;
static const uint8_t reply_bytes[14]={1U,4U};

Px4Lite_Result_t Px4Lite_CopyGnss(Px4Lite_SensorGnss_t *out) { gnss_reads++;*out=source_gnss;return source_gnss_result; }
Px4Lite_Result_t Px4Lite_GnssCopyDiagnostics(Px4Lite_GnssDiagnostics_t *out) { diag_reads++;*out=source_diag;return source_diag_result; }
Px4Lite_Result_t Px4Lite_CopyTime(Px4Lite_TimeSnapshot_t *out) { time_reads++;*out=source_time;if(advance_during_copy){clock_ms++;}return source_time_result; }
uint32_t Px4Lite_PlatformGetMs(void) { return clock_ms; }
uint8_t Px4Lite_IdentityGetMavlinkSystemId(void) { return 42U; }
uint8_t Px4Lite_IdentityGetNodeId(void) { return 42U; }
Px4Lite_Result_t Px4Lite_RpiMavlinkSend(const uint8_t *bytes,uint16_t length)
{ assert(length<=sizeof(uart_bytes));memcpy(uart_bytes,bytes,length);uart_length=length;uart_calls++;return uart_result; }
Px4Lite_Result_t Px4Lite_LoRaSend(const uint8_t *bytes,uint16_t length)
{ (void)bytes;(void)length;lora_calls++;return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_RpiMavlinkInit(void) { return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_RpiH1EchoInit(void) { return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_RpiBaudSwitchInit(void) { return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_RpiBaudSwitchPeekResponse(Px4Lite_RpiBaudSwitchResponseView_t *out)
{ if(!b1_pending){return PX4LITE_IDLE;}memset(out,0,sizeof(*out));out->payload=reply_bytes;out->payload_type=0x8003U;out->payload_length=14U;return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_RpiBaudSwitchResponseSent(void) { b1_pending=0U;return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_RpiH1EchoPeekResponse(Px4Lite_RpiH1EchoResponseView_t *out)
{ if(!h1_pending){return PX4LITE_IDLE;}memset(out,0,sizeof(*out));out->payload=reply_bytes;out->payload_type=0x8003U;out->payload_length=2U;return PX4LITE_OK; }
void Px4Lite_RpiH1EchoResponseSent(void) { h1_pending=0U; }
void Px4Lite_LoRaGetDebugInfo(Px4Lite_CommDebugInfo_t *out) { memset(out,0,sizeof(*out)); }
Px4Lite_State_t Px4Lite_LoRaGetState(uint32_t now) { (void)now;return PX4LITE_STATE_OFFLINE; }
uint8_t Px4Lite_LoRaIsPresent(void) { return 0U; }
void Px4Lite_RemoteIdTxGetStats(Px4Lite_RemoteIdTxStats_t *out) { memset(out,0,sizeof(*out)); }
Px4Lite_Result_t Px4Lite_CopyAlarmRecord(uint16_t index,Px4Lite_AlarmRecord_t *out) { (void)index;(void)out;return PX4LITE_NOT_READY; }
uint16_t Px4Lite_LocalMsgLogCopy(Px4Lite_LogEntry_t *out,uint16_t capacity,uint32_t *version,uint16_t *last)
{ (void)out;(void)capacity;if(version){*version=0U;}if(last){*last=0U;}return 0U; }
void Px4Lite_LoraLabInit(uint32_t now) { (void)now; }

/** @brief 其他真实遥测编码器无数据替身，确保调度不需要硬件或伪造电机输出。 */
#define EMPTY_COPY(name,type) Px4Lite_Result_t name(type *out) { (void)out;return PX4LITE_NOT_READY; }
EMPTY_COPY(Px4Lite_CopyNavigation,Px4Lite_VehicleNavigation_t)
EMPTY_COPY(Px4Lite_CopyBaro,Px4Lite_SensorBaro_t)
EMPTY_COPY(Px4Lite_CopyBattery,Px4Lite_BatteryStatus_t)
EMPTY_COPY(Px4Lite_CopyBattery2,Px4Lite_BatteryStatus_t)
EMPTY_COPY(Px4Lite_CopyMotor,Px4Lite_MotorOutputs_t)
EMPTY_COPY(Px4Lite_CopyHealth,Px4Lite_SystemHealth_t)

/** @brief 测试端逐字节解码，不依赖被测编码器的内部整数字节写入函数。 */
static uint16_t Read16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8U)); }
static uint32_t Read32(const uint8_t *p) { return (uint32_t)p[0]|((uint32_t)p[1]<<8U)|((uint32_t)p[2]<<16U)|((uint32_t)p[3]<<24U); }
/** @brief 单次Peek必须只有一次源复制，重复读取和重试不得读取新源。 */
static Px4Lite_GnssUplinkView_t Peek(uint32_t now)
{ Px4Lite_GnssUplinkView_t v;clock_ms=now;assert(Px4Lite_GnssUplinkPeek(now,&v)==PX4LITE_OK);assert(v.length<=64U && v.length<=128U);return v; }
/** @brief 为完整黄金向量建立混合来源、合法零值和独立RTC系统时间。 */
static void Sources(uint32_t now)
{
  uint8_t i;
  memset(&source_gnss,0,sizeof(source_gnss));memset(&source_diag,0,sizeof(source_diag));memset(&source_time,0,sizeof(source_time));
  source_gnss_result=source_diag_result=source_time_result=PX4LITE_OK;
  source_gnss.header.valid=1U;source_gnss.header.sample_time_ms=now;source_gnss.valid_mask=31U;
  source_gnss.position_source=1U;source_gnss.velocity_source=3U;source_gnss.time_source=1U;source_gnss.altitude_source=2U;source_gnss.status_source=1U;
  source_gnss.position_update_ms=source_gnss.velocity_update_ms=source_gnss.time_update_ms=source_gnss.altitude_update_ms=source_gnss.status_update_ms=now;
  source_gnss.fix_type=4U;source_gnss.fix_dimension=3U;source_gnss.satellites_used=12U;source_gnss.raw_status_type=4U;
  source_gnss.dimension_source=1U;source_gnss.dimension_update_ms=now;
  source_gnss.latitude_e7=-123456789L;source_gnss.longitude_e7=987654321L;source_gnss.altitude_mm=-1234L;source_gnss.ground_speed_cms=456U;source_gnss.heading_deg100=12345U;
  source_gnss.utc_date=261106U;source_gnss.utc_sec=45296U;source_gnss.dop_present_mask=127U;source_gnss.dop_solution_mask=127U;source_gnss.dop_itow_known_mask=127U;
  for(i=0U;i<7U;i++){source_gnss.dop_source[i]=6U;source_gnss.dop_x100[i]=(uint16_t)(101U+i);source_gnss.dop_update_ms[i]=now;source_gnss.dop_itow_ms[i]=123456U;}
  source_gnss.dop_source[4]=4U; /* NMEA GSA没有UBX iTOW，不能凭known垃圾位造历元。 */
  source_gnss.counts_present_mask=15U;source_gnss.gps_visible=0U;source_gnss.bds_visible=14U;source_gnss.gps_used=0U;source_gnss.bds_used=8U;
  for(i=0U;i<4U;i++){source_gnss.counts_source[i]=(i<2U)?5U:4U;source_gnss.counts_update_ms[i]=now;}
  source_diag.available=1U;source_diag.transport_available=1U;source_diag.measured_ms=now;source_diag.rx_seen=1U;source_diag.frame_seen=1U;source_diag.version_seen=1U;source_diag.initialized=1U;source_diag.tx_busy=1U;
  source_diag.protocol=2U;source_diag.state=3U;source_diag.config_step=3U;source_diag.config_total=3U;source_diag.config_confirmed=1U;source_diag.protocol_version_x100=3410U;
  source_diag.baud_bps=source_diag.transport_baud_bps=115200U;source_diag.rx_bytes=12345U;source_diag.valid_nmea=7U;source_diag.valid_ubx=89U;source_diag.checksum_errors=2U;source_diag.framing_resets=3U;
  source_diag.rx_drop_bytes=4U;source_diag.uart_errors=5U;source_diag.dma_errors=6U;source_diag.last_rx_ms=now;source_diag.last_frame_ms=now;
  source_diag.tx_commands=10U;source_diag.ack_count=3U;source_diag.nak_count=1U;source_diag.timeout_count=2U;source_diag.tx_packets=11U;source_diag.tx_failures=12U;source_diag.recoveries=13U;source_diag.transport_rx_bytes=12349U;source_diag.ndtr=694U;source_diag.queued_bytes=0U;
  source_time.header.valid=1U;source_time.header.sample_time_ms=now;source_time.utc_date_ymd=20261106U;source_time.utc_time_hhmmss=123456U;source_time.source=PX4LITE_TIME_SOURCE_RTC;source_time.sync_state=PX4LITE_TIME_SYNC_RTC_VALID;source_time.sync_age_s=42U;
  clock_ms=now;gnss_reads=diag_reads=time_reads=uart_calls=lora_calls=0U;uart_result=PX4LITE_OK;b1_pending=h1_pending=advance_during_copy=0U;
  Px4Lite_GnssUplinkInit(now);
}
/** @brief 完整线帧由真实MAVLink解析器验CRC，允许合法的MAVLink2尾零裁剪。 */
static mavlink_message_t DecodeWire(void)
{
  mavlink_message_t rx,buffer; mavlink_status_t status,rxstatus;uint16_t i;uint8_t frames=0U;
  memset(&buffer,0,sizeof(buffer));memset(&status,0,sizeof(status));memset(&rxstatus,0,sizeof(rxstatus));memset(&rx,0,sizeof(rx));
  for(i=0U;i<uart_length;i++){if(mavlink_frame_char_buffer(&buffer,&status,uart_bytes[i],&rx,&rxstatus)==MAVLINK_FRAMING_OK){frames++;}}
  assert(frames==1U && rx.magic==MAVLINK_STX && rx.sysid==42U && rx.compid==193U);return rx;
}
/** @brief 输出六个固定完整MAVLink黄金帧，供Pi实现做跨语言校验。 */
static void TestGoldenWire(void)
{
  uint8_t part;uint16_t i;uint32_t seq; mavlink_message_t rx;mavlink_tunnel_t t;
  Sources(1000U);s_rpi_tx_seq=0U;mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq=0U;
  for(part=0U;part<6U;part++){
    clock_ms=1000U+10U*part;seq=mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq;
    assert(MavTx_SendRpiGnssUplink(clock_ms)==PX4LITE_OK);rx=DecodeWire();assert(rx.msgid==385U && rx.seq==part);
    assert(mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq==seq); /* 不耗LoRa序号。 */
    mavlink_msg_tunnel_decode(&rx,&t);assert(t.payload_length<=64U && t.target_system==0U && t.target_component==0U);
    assert(Read32(&t.payload[4])==1U && Read32(&t.payload[8])==1000U);
    assert(uart_length<=81U && (uart_length+13U)*10000UL<57600UL*20UL); /* 最坏签名也<20ms。 */
    if(part==0U){assert(t.payload_type==0x8011U && t.payload_length==64U && t.payload[12]==63U);assert(Read16(&t.payload[14])==(1U|(3U<<3U)|(1U<<6U)|(2U<<9U)|(1U<<12U)));assert((int32_t)Read32(&t.payload[20])==-123456789L);assert(Read32(&t.payload[40])==20261106U && Read32(&t.payload[44])==45296U);}
    if(part==1U){assert(t.payload_type==0x8012U && t.payload[12]==4U && t.payload[18]==11U);}
    if(part==2U){assert(t.payload[2]==1U && t.payload[12]==3U && t.payload[18]==3U && Read32(&t.payload[24])==0U);}
    if(part==3U){assert(t.payload_type==0x8013U && t.payload_length==32U && t.payload[12]==15U && t.payload[13]==15U && t.payload[16]==0U);}
    if(part==4U){assert(t.payload_type==0x8014U && t.payload[12]==127U && t.payload[17]==1U && Read32(&t.payload[20])==115200U);}
    if(part==5U){assert(t.payload[12]==1U && t.payload[13]==PX4LITE_TIME_SOURCE_RTC && Read32(&t.payload[56])==20261106U && Read32(&t.payload[60])==45296U);}
    printf("GOLDEN_FRAME_%u ",(unsigned)part);for(i=0U;i<uart_length;i++){printf("%02X",uart_bytes[i]);}puts("");
  }
  assert(gnss_reads==1U && diag_reads==1U && time_reads==1U && lora_calls==0U);
}
/** @brief 无定位/0颗为合法状态，分类未知不冒充0；DOP保留reported但不可用。 */
static void TestNoFixUnknown(void)
{
  Px4Lite_GnssUplinkView_t v;Sources(0U);source_gnss.valid_mask=16U;source_gnss.fix_type=0U;source_gnss.satellites_used=0U;source_gnss.raw_status_type=0U;source_gnss.counts_present_mask=0U;
  v=Peek(0U);assert(v.payload[12]==16U && v.payload[18]==0U && (v.payload[62]&16U));assert(Read16(&v.payload[56])==0U && Read32(&v.payload[20])==0U);
  Px4Lite_GnssUplinkResult(PX4LITE_OK,0U);v=Peek(0U);assert(v.payload[18]==9U && Read16(&v.payload[20])==101U);
  Px4Lite_GnssUplinkResult(PX4LITE_OK,0U);v=Peek(0U);assert(v.payload[18]==1U && Read32(&v.payload[24])==0U);
  Px4Lite_GnssUplinkResult(PX4LITE_OK,0U);v=Peek(0U);assert(v.payload[12]==0U && v.payload[13]==0U && Read16(&v.payload[24])==65535U);
  Sources(0U);source_gnss_result=PX4LITE_NOT_READY;source_diag_result=PX4LITE_NOT_READY;memset(&source_diag,0,sizeof(source_diag));source_time_result=PX4LITE_NOT_READY;
  v=Peek(0U);assert(v.payload[12]==0U && v.payload[62]==0U && Read16(&v.payload[48])==65535U && Read16(&v.payload[60])==65535U);
}
/** @brief 过期/未来/回绕/午夜逐域判定；系统RTC不能填入NAV UTC。 */
static void TestAgesTimeAndRanges(void)
{
  Px4Lite_GnssUplinkView_t v;uint8_t i;
  Sources(0U);Px4Lite_GnssUplinkInit(3000U);v=Peek(3000U);assert(v.payload[12]==0U && v.payload[13]==63U && Read16(&v.payload[48])==3000U);assert(Read32(&v.payload[40])==0U);
  for(i=0U;i<4U;i++){Px4Lite_GnssUplinkResult(PX4LITE_OK,3000U);v=Peek(3000U);}assert(!(v.payload[12]&1U) && (v.payload[12]&2U) && v.payload[14]==255U && v.payload[17]==0U);
  Px4Lite_GnssUplinkResult(PX4LITE_OK,3000U);v=Peek(3000U);assert(v.payload[12]==0U && Read32(&v.payload[56])==0U);
  Sources(101U);Px4Lite_GnssUplinkInit(100U);v=Peek(100U);assert(v.payload[12]==0U && Read16(&v.payload[48])==65535U);
  Sources(UINT32_MAX-10U);Px4Lite_GnssUplinkInit(10U);v=Peek(10U);assert(v.payload[12]==63U && Read16(&v.payload[48])==21U);
  Sources(0U);source_gnss.utc_sec=0U;v=Peek(0U);assert((v.payload[12]&4U) && Read32(&v.payload[44])==0U);
  Sources(0U);source_gnss.utc_date=260230U;source_gnss.position_source=7U;source_gnss.dimension_source=0U;source_gnss.dop_x100[4]=0U;v=Peek(0U);assert(!(v.payload[12]&5U) && !(v.payload[12]&32U) && v.payload[17]==0U && (Read16(&v.payload[14])&7U)==0U);
  Sources(0U);Px4Lite_GnssUplinkInit(70000U);v=Peek(70000U);assert(Read16(&v.payload[48])==65534U);
  Sources(1U);Px4Lite_GnssUplinkInit(0U);advance_during_copy=1U;v=Peek(0U);assert(v.payload[12]==63U && Read32(&v.payload[8])==1U && Read16(&v.payload[48])==0U);
  Sources(0U);source_time.header.sample_time_ms=1U;source_diag.transport_available=0U;source_diag.rx_seen=source_diag.frame_seen=0U;
  for(i=0U;i<4U;i++){v=Peek(0U);Px4Lite_GnssUplinkResult(PX4LITE_OK,0U);}v=Peek(0U);assert(!(v.payload[12]&2U) && Read32(&v.payload[56])==UINT32_MAX && Read32(&v.payload[60])==UINT32_MAX);
  Px4Lite_GnssUplinkResult(PX4LITE_OK,0U);v=Peek(0U);assert(!v.payload[12]);
}
/** @brief 单包重试固定内容且有上限；新组主动替换旧组，输出窗口不能被结果重放推进。 */
static void TestFreezeRetryAndReplacement(void)
{
  Px4Lite_GnssUplinkView_t v;uint8_t before[64];uint32_t reads;
  Sources(0U);v=Peek(0U);memcpy(before,v.payload,64U);reads=gnss_reads;source_gnss.latitude_e7=1;
  Px4Lite_GnssUplinkResult(PX4LITE_IO_ERROR,0U);assert(Px4Lite_GnssUplinkPeek(49U,&v)==PX4LITE_IDLE);v=Peek(50U);assert(memcmp(before,v.payload,12U)==0 && memcmp(&before[20],&v.payload[20],28U)==0 && Read16(&v.payload[48])==50U && gnss_reads==reads);
  Px4Lite_GnssUplinkResult(PX4LITE_BUSY,50U);v=Peek(100U);Px4Lite_GnssUplinkResult(PX4LITE_IO_ERROR,100U);v=Peek(100U);assert(v.frame_index==1U);
  Px4Lite_GnssUplinkResult(PX4LITE_OK,100U);Px4Lite_GnssUplinkResult(PX4LITE_OK,100U);v=Peek(100U);assert(v.frame_index==2U);
  v=Peek(1000U);assert(v.frame_index==0U && Read32(&v.payload[4])==2U && gnss_reads==reads+1U);
  assert(Px4Lite_GnssUplinkPeek(1000U,0)==PX4LITE_INVALID_PARAM);
}
/** @brief 组内年龄继续流逝；1499/1500/1501ms不能用冻结时刻续期有效位。 */
static void TestEncodingAgeBoundary(void)
{
  Px4Lite_GnssUplinkView_t v;uint8_t i;
  Sources(0U);Px4Lite_GnssUplinkInit(1499U);v=Peek(1499U);assert(v.payload[12]==63U && Read32(&v.payload[8])==1499U);
  source_gnss.latitude_e7=1;v=Peek(1500U);assert(v.payload[12]==63U && Read16(&v.payload[48])==1500U && (int32_t)Read32(&v.payload[20])==-123456789L);
  v=Peek(1501U);assert(v.payload[12]==0U && v.payload[13]==63U && Read16(&v.payload[48])==1501U && Read32(&v.payload[8])==1499U && Read32(&v.payload[20])==0U);
  Px4Lite_GnssUplinkResult(PX4LITE_OK,1501U);v=Peek(1501U);assert(v.payload[18]==13U && Read16(&v.payload[22])==1501U);
  Sources(0U);Px4Lite_GnssUplinkInit(2499U);v=Peek(2499U);
  for(i=0U;i<5U;i++){Px4Lite_GnssUplinkResult(PX4LITE_OK,2499U);v=Peek(2499U);}
  assert(v.payload[12]==1U);v=Peek(2500U);assert(v.payload[12]==1U);v=Peek(2501U);assert(v.payload[12]==0U);
  /* 没有POSITION的纯GSV/GSA缓存仍可报告分类计数与DOP reported事实。 */
  Sources(0U);source_gnss.header.valid=0U;source_gnss.valid_mask=0U;
  v=Peek(0U);Px4Lite_GnssUplinkResult(PX4LITE_OK,0U);v=Peek(0U);assert(v.payload[18]==9U);
  Px4Lite_GnssUplinkResult(PX4LITE_OK,0U);v=Peek(0U);Px4Lite_GnssUplinkResult(PX4LITE_OK,0U);v=Peek(0U);assert(v.payload[12]==15U && v.payload[13]==15U && v.payload[16]==0U);
  assert((64U+5U+12U+13U)*10000UL<57600UL*20UL); /* 最坏未裁剪且带签名的理论线时。 */
  /* 全局仍有PVT位置，GSA1的reported HDOP不能借用这个位置获得解质量授权。 */
  Sources(0U);source_gnss.dop_solution_mask&=(uint8_t)~(1U<<4U);source_gnss.dop_x100[4]=9999U;
  v=Peek(0U);assert((v.payload[12]&1U) && !(v.payload[12]&32U) && (v.payload[62]&32U));
  Px4Lite_GnssUplinkResult(PX4LITE_OK,0U);v=Peek(0U);Px4Lite_GnssUplinkResult(PX4LITE_OK,0U);v=Peek(0U);assert(v.payload[18]==1U && Read16(&v.payload[20])==9999U);
  Sources(0U);source_gnss.dop_solution_mask=0U;v=Peek(0U);assert(!(v.payload[12]&32U));Px4Lite_GnssUplinkResult(PX4LITE_OK,0U);v=Peek(0U);assert(v.payload[18]==9U);
}
/** @brief 三路持续到期时均获机会；B1/H1/ACK优先且每次最多一次实际UART提交。 */
static void TestSchedulerFairnessAndPriority(void)
{
  uint32_t cycle,before,gnss_count=0U,old_count=0U,telemetry_count=0U;uint8_t i;
  mavlink_message_t rx;mavlink_tunnel_t t;
  Sources(0U);source_gnss_result=PX4LITE_NOT_READY;assert(Px4Lite_MavlinkTxInit(0U)==PX4LITE_OK);s_tx_enabled=0U;
  for(cycle=0U;cycle<60U;cycle++) {
    clock_ms=cycle*10U;for(i=0U;i<MAV_TX_RPI_TELEM_COUNT;i++){s_rpi_telem_next_ms[i]=0U;}
    for(i=0U;i<MAV_TX_RPI_CATALOG_COUNT;i++){*s_mav_tx_rpi_catalog[i].next_ms=0U;}
    before=uart_calls;(void)Px4Lite_MavlinkTxRunRpi(clock_ms);assert(uart_calls==before+1U);rx=DecodeWire();
    if(rx.msgid==0U){telemetry_count++;}else if(rx.msgid==385U){mavlink_msg_tunnel_decode(&rx,&t);if(t.payload_type>=0x8011U && t.payload_type<=0x8014U){gnss_count++;}else{old_count++;}}else{old_count++;}
  }
  assert(gnss_count==6U && old_count>6U && telemetry_count>6U && lora_calls==0U);
  b1_pending=h1_pending=1U;Px4Lite_MavlinkQueueCommandAck(400U,0U,9U,1U,PX4LITE_MAVLINK_LINK_RPI);
  for(i=0U;i<3U;i++){before=uart_calls;(void)Px4Lite_MavlinkTxRunRpi(clock_ms);assert(uart_calls==before+1U);rx=DecodeWire();assert(rx.msgid==(i==2U?77U:385U));if(i<2U){mavlink_msg_tunnel_decode(&rx,&t);assert(t.payload_type==0x8003U);}}
}
int main(void)
{
  TestGoldenWire();TestNoFixUnknown();TestAgesTimeAndRanges();TestFreezeRetryAndReplacement();TestEncodingAgeBoundary();TestSchedulerFairnessAndPriority();
  puts("GNSS unified uplink: PASS (6 wire vectors / nofix / flags / source / age / frozen retry / fair one-submit UART6)");return 0;
}
