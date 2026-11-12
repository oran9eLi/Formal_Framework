/**
 * @file test_gnss_raw_uplink.c
 * @brief 真实RAW分片/统计/四路UART6调度回归；无硬件串口或软件注入接收。
 * @details 独立Framework事实桩，覆盖默认Pi仅原帧与显式兼容配置，不修改typed模块。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static unsigned raw_critical_depth;
static void (*raw_exit_hook)(void);
void RawTestEnter(void) { raw_critical_depth++; }
void RawTestExit(void)
{ void (*hook)(void);assert(raw_critical_depth>0U);raw_critical_depth--;hook=raw_exit_hook;if(!raw_critical_depth && hook){raw_exit_hook=0;hook();} }
#define taskENTER_CRITICAL() RawTestEnter()
#define taskEXIT_CRITICAL() RawTestExit()
typedef int BaseType_t;
typedef void *TaskHandle_t;
#include "../../Framework/Src/px4lite_gnss_uplink.c"
#if PX4LITE_ENABLE_RPI_BMI088_UPLINK
/* 默认共存夹具使用真实BMI邮箱/编码器；GPS专项由runner显式关闭该通道。 */
#define TEST_GNSS_BMI088_IMPLEMENTATION_INCLUDED 1
#include "../../Framework/Src/px4lite_bmi088_uplink.c"
#endif
#include "../../Framework/Src/px4lite_mavlink_tx.c"
#include "../../Business/Src/app_data_api.c"
static Px4Lite_SensorGnss_t source_gnss;
static Px4Lite_GnssDiagnostics_t source_diag;
static Px4Lite_TimeSnapshot_t source_time;
static Px4Lite_Result_t source_gnss_result=PX4LITE_OK,source_diag_result=PX4LITE_OK,source_time_result=PX4LITE_OK;
static uint32_t clock_ms,gnss_reads,diag_reads,time_reads,uart_calls,lora_calls;
static uint8_t uart_bytes[280];static uint16_t uart_length;
static Px4Lite_Result_t uart_result=PX4LITE_OK;
static uint8_t b1_pending,h1_pending,advance_during_copy;
static const uint8_t reply_bytes[14]={1U,4U};

#if PX4LITE_ENABLE_RPI_BMI088_UPLINK
/** @brief 可由专项用例注入异常诊断，默认代表当前已识别的在线I2C器件。 */
static Px4Lite_ImuDiagnostics_t test_bmi_diag={
  .reinit_count=1U,.driver=PX4LITE_IMU_DRIVER_BMI088,.bus=1U,
  .accel_chip_id=0x1EU,.gyro_chip_id=0x0FU,
  .accel_addr_7bit=0x18U,.gyro_addr_7bit=0x68U,.device_ready=1U
};
static Px4Lite_Result_t test_bmi_diag_result=PX4LITE_OK;
static Px4Lite_State_t test_bmi_module_state=PX4LITE_STATE_ONLINE;
/** @brief BMI设备只读事实桩，不产生样本或执行硬件读取；采样由独立Publish注入。 */
Px4Lite_Result_t Px4Lite_ImuCopyDiagnostics(Px4Lite_ImuDiagnostics_t *out)
{
  *out=test_bmi_diag;
  return test_bmi_diag_result;
}
/** @brief 独立提供已在线模块状态，不以发送计数冒充健康判定。 */
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t id,Px4Lite_ModuleStatus_t *out)
{
  assert(id==PX4LITE_MODULE_IMU);
  memset(out,0,sizeof(*out));out->state=test_bmi_module_state;
  return PX4LITE_OK;
}
#endif

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

/** @brief 可独立提供真实工程量的topic桩，默认关闭，仅用于验证保留的非GPS遥测。 */
static uint8_t extra_ready;
static Px4Lite_VehicleNavigation_t extra_nav;
static Px4Lite_SensorBaro_t extra_baro;
static Px4Lite_BatteryStatus_t extra_battery;
static Px4Lite_MotorOutputs_t extra_motor;
static Px4Lite_SystemHealth_t extra_health;
#define EXTRA_COPY(name,type,source) Px4Lite_Result_t name(type *out) { if(!extra_ready){return PX4LITE_NOT_READY;}*out=source;return PX4LITE_OK; }
EXTRA_COPY(Px4Lite_CopyNavigation,Px4Lite_VehicleNavigation_t,extra_nav)
EXTRA_COPY(Px4Lite_CopyBaro,Px4Lite_SensorBaro_t,extra_baro)
EXTRA_COPY(Px4Lite_CopyBattery2,Px4Lite_BatteryStatus_t,extra_battery)
EXTRA_COPY(Px4Lite_CopyMotor,Px4Lite_MotorOutputs_t,extra_motor)
EXTRA_COPY(Px4Lite_CopyHealth,Px4Lite_SystemHealth_t,extra_health)
Px4Lite_Result_t Px4Lite_CopyBattery(Px4Lite_BatteryStatus_t *out) { (void)out;return PX4LITE_NOT_READY; }
static uint16_t Read16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8U)); }
static uint32_t Read32(const uint8_t *p) { return (uint32_t)p[0]|((uint32_t)p[1]<<8U)|((uint32_t)p[2]<<16U)|((uint32_t)p[3]<<24U); }

/** @brief 独立Framework事实夹具，保留协议源/时刻，不访问真实硬件。 */
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
  extra_ready=0U;
}
/** @brief 实际MAVLink解析器验证完整wire及CRC。 */
static mavlink_message_t DecodeWire(void)
{
  mavlink_message_t rx,buffer; mavlink_status_t status,rxstatus;uint16_t i;uint8_t frames=0U;
  memset(&buffer,0,sizeof(buffer));memset(&status,0,sizeof(status));memset(&rxstatus,0,sizeof(rxstatus));memset(&rx,0,sizeof(rx));
  for(i=0U;i<uart_length;i++){if(mavlink_frame_char_buffer(&buffer,&status,uart_bytes[i],&rx,&rxstatus)==MAVLINK_FRAMING_OK){frames++;}}
  assert(frames==1U && rx.magic==MAVLINK_STX && rx.sysid==42U && rx.compid==193U);return rx;
}
#include "../../Framework/Src/px4lite_gnss_raw_uplink.c"

#if PX4LITE_ENABLE_RPI_GNSS_RAW_UPLINK
static uint8_t raw_wire[1024],raw_original[1024],raw_reassembled[1024];
static const uint8_t raw_short[]={'$','\n'};
/** @brief 独立参考CRC与公开标准向量共同核验，不使用被测CRC计算结果当期望值。 */
static uint16_t RefCrc(const uint8_t *p,uint16_t length)
{
  uint32_t crc=65535U;uint16_t i;uint8_t bit;
  for(i=0U;i<length;i++){crc^=(uint32_t)p[i]*256U;for(bit=0U;bit<8U;bit++){crc=(crc&32768U)?(crc*2U)^4129U:crc*2U;crc&=65535U;}}
  return (uint16_t)crc;
}
/** @brief 构造合法长度的完整UBX原wire，可保留一个真实错误的原CK_B供BAD路径验证。 */
static void MakeRaw(uint16_t length,uint8_t bad)
{
  uint16_t i;uint8_t a=0U,b=0U;assert(length>=8U && length<=sizeof(raw_wire));
  for(i=0U;i<length;i++){raw_wire[i]=(uint8_t)(i*17U);}
  raw_wire[0]=0xB5U;raw_wire[1]=0x62U;raw_wire[2]=1U;raw_wire[3]=7U;raw_wire[4]=(uint8_t)(length-8U);raw_wire[5]=(uint8_t)((length-8U)>>8U);
  for(i=2U;i<length-2U;i++){a=(uint8_t)(a+raw_wire[i]);b=(uint8_t)(b+a);}raw_wire[length-2U]=a;raw_wire[length-1U]=(uint8_t)(b^(bad?128U:0U));memcpy(raw_original,raw_wire,length);
}
/** @brief 只设测试时钟并取得一个有界RAW包，借用指针不跨下一次状态调用使用。 */
static Px4Lite_GnssRawUplinkView_t RawPeekAt(uint32_t now)
{ Px4Lite_GnssRawUplinkView_t v;clock_ms=now;assert(Px4Lite_GnssRawUplinkPeek(now,&v)==PX4LITE_OK);assert(v.length<=64U);return v; }
/** @brief 初始化RAW独立状态，不能清除typed或生产测量。 */
static void ResetRaw(uint32_t now)
{ clock_ms=now;raw_exit_hook=0;Px4Lite_GnssRawUplinkInit(now);uart_result=PX4LITE_OK; }
/** @brief 完整真实MAVLink线帧黄金向量，包括BAD原CRC与新增wire CRC16元数据。 */
static void TestRawGoldenWire(void)
{
  uint8_t part;uint16_t i;uint32_t seq;mavlink_message_t rx;mavlink_tunnel_t t;
  assert(RawCrc16((const uint8_t *)"123456789",9U)==0x29B1U && RefCrc((const uint8_t *)"123456789",9U)==0x29B1U);
  ResetRaw(1000U);MakeRaw(100U,1U);Px4Lite_GnssRawEnqueue(raw_wire,100U,2U,0U,1U,7U,0U,1000U);
  memset(raw_wire,0,sizeof(raw_wire));memset(raw_reassembled,0,sizeof(raw_reassembled));s_rpi_tx_seq=0U;mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq=0U;
  for(part=0U;part<3U;part++){
    clock_ms=1000U+part*10U;seq=mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq;
    assert(MavTx_SendRpiGnssRawUplink(clock_ms)==PX4LITE_OK);assert(mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq==seq);rx=DecodeWire();mavlink_msg_tunnel_decode(&rx,&t);
    assert(rx.msgid==385U && rx.seq==part && t.payload_type==0x8020U && t.payload_length==(part==2U?44U:64U));
    assert(t.payload[0]==1U && t.payload[1]==2U && t.payload[2]==2U && t.payload[3]==24U && Read32(&t.payload[4])==1U && Read32(&t.payload[8])==1000U);
    assert(Read16(&t.payload[12])==100U && Read16(&t.payload[14])==part*40U && t.payload[16]==part && t.payload[17]==3U && t.payload[19]==0U && t.payload[20]==1U && t.payload[21]==7U);
    assert(Read16(&t.payload[22])==RefCrc(raw_original,100U));memcpy(&raw_reassembled[Read16(&t.payload[14])],&t.payload[24],t.payload[18]);
    printf("RAW_GOLDEN_FRAME_%u ",(unsigned)part);for(i=0U;i<uart_length;i++){printf("%02X",uart_bytes[i]);}puts("");
  }
  assert(memcmp(raw_original,raw_reassembled,100U)==0 && s_raw.counters.completed_frames==1U && s_raw.counters.completed_parts==3U);
#if PX4LITE_RPI_GNSS_RAW_STATS_ENABLE
  clock_ms=2000U;assert(MavTx_SendRpiGnssRawUplink(clock_ms)==PX4LITE_OK);rx=DecodeWire();mavlink_msg_tunnel_decode(&rx,&t);
  assert(t.payload_type==0x8021U && t.payload_length==64U && t.payload[1]==0x21U && Read32(&t.payload[12])==1U && Read32(&t.payload[20])==1U && Read32(&t.payload[36])==3U);
  assert(t.payload[44]==0U && t.payload[45]==4U && t.payload[46]==0U && t.payload[47]==1U && Read32(&t.payload[48])==1000U && Read32(&t.payload[52])==980U && Read32(&t.payload[56])==UINT32_MAX && Read16(&t.payload[60])==1024U);
  printf("RAW_GOLDEN_STATS ");for(i=0U;i<uart_length;i++){printf("%02X",uart_bytes[i]);}puts("");
#else
  {uint32_t before=uart_calls;clock_ms=2000U;assert(MavTx_SendRpiGnssRawUplink(clock_ms)==PX4LITE_IDLE && uart_calls==before);}
#endif
}
/** @brief 有效NMEA保留美元号、原星号校验及CRLF，独立protocol/type不混为typed来源。 */
static void TestCompleteNmea(void)
{
  static const uint8_t nmea[]="$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
  uint8_t part,checksum=0U;uint16_t i,length=(uint16_t)(sizeof(nmea)-1U);mavlink_message_t rx;mavlink_tunnel_t t;
  for(i=1U;nmea[i]!='*';i++){checksum^=nmea[i];}assert(checksum==0x47U);
  ResetRaw(0U);Px4Lite_GnssRawEnqueue(nmea,length,1U,1U,0U,0U,1U,0U);memset(raw_reassembled,0,sizeof(raw_reassembled));
  for(part=0U;part<2U;part++){clock_ms=part;assert(MavTx_SendRpiGnssRawUplink(clock_ms)==PX4LITE_OK);rx=DecodeWire();mavlink_msg_tunnel_decode(&rx,&t);assert(t.payload[1]==1U && t.payload[2]==3U && t.payload[19]==1U && t.payload[20]==0U && t.payload[21]==0U && Read16(&t.payload[22])==RefCrc(nmea,length));memcpy(&raw_reassembled[Read16(&t.payload[14])],&t.payload[24],t.payload[18]);}
  assert(memcmp(raw_reassembled,nmea,length)==0 && raw_reassembled[length-2U]=='\r' && raw_reassembled[length-1U]=='\n');
}
/** @brief 模拟Sensor已预留但尚未READY时Comm读取，不能读取半个frame。 */
static void ReadDuringWriting(void)
{ Px4Lite_GnssRawUplinkView_t v;assert(!raw_critical_depth);assert(Px4Lite_GnssRawUplinkPeek(clock_ms,&v)==PX4LITE_IDLE); }
/** @brief 模拟pin后被Sensor抢占并填满其余槽；不能改写pin槽或让CRC读到别帧。 */
static void FillDuringPin(void)
{ uint8_t i;assert(!raw_critical_depth && s_raw.active);for(i=0U;i<4U;i++){Px4Lite_GnssRawEnqueue(raw_short,2U,1U,0U,0U,0U,0U,0U);} }
/** @brief 4槽溢出、活跃pin、最大1024B二十六片与释放后复用空槽均保真。 */
static void TestRingPinAndMaximum(void)
{
  Px4Lite_GnssRawUplinkView_t v;uint8_t part;uint16_t crc;
  ResetRaw(0U);MakeRaw(1024U,0U);raw_exit_hook=ReadDuringWriting;Px4Lite_GnssRawEnqueue(raw_wire,1024U,2U,0U,1U,7U,1U,0U);
  raw_exit_hook=FillDuringPin;v=RawPeekAt(0U);assert(s_raw.depth==4U && s_raw.counters.full_drops==1U && s_raw.counters.enqueued_frames==4U);crc=RefCrc(raw_original,1024U);
  for(part=0U;part<26U;part++){
    if(part){v=RawPeekAt(part);}assert(v.payload_type==0x8020U && Read32(&v.payload[4])==1U && v.payload[2]==3U && v.payload[17]==26U && v.payload[16]==part && Read16(&v.payload[22])==crc);
    assert(v.length==(part==25U?48U:64U));memcpy(&raw_reassembled[part*40U],&v.payload[24],v.payload[18]);Px4Lite_GnssRawUplinkResult(PX4LITE_OK,part);
  }
  assert(memcmp(raw_original,raw_reassembled,1024U)==0 && s_raw.depth==3U && s_raw.counters.completed_frames==1U);
  Px4Lite_GnssRawEnqueue(raw_short,2U,1U,0U,0U,0U,0U,30U);assert(s_raw.depth==4U && s_raw.counters.input_events==6U);
#if PX4LITE_RPI_GNSS_RAW_STATS_ENABLE
  v=RawPeekAt(1000U);assert(v.payload_type==0x8021U && Read32(&v.payload[12])==6U && Read32(&v.payload[16])==5U && Read32(&v.payload[24])==1U && v.payload[44]==4U);
#else
  assert(s_raw.counters.input_events==6U && s_raw.counters.enqueued_frames==5U && s_raw.counters.full_drops==1U);
#endif
}
/** @brief 拒绝元数据/截断/超长与源丢弃均计独立事件；短坏NMEA完整边界不丢。 */
static void TestRejectAndShortNmea(void)
{
  Px4Lite_GnssRawUplinkView_t v;ResetRaw(0U);MakeRaw(100U,0U);
  Px4Lite_GnssRawEnqueue(0,0U,1U,0U,0U,0U,0U,0U);Px4Lite_GnssRawEnqueue(raw_wire,1025U,2U,0U,1U,7U,1U,0U);Px4Lite_GnssRawEnqueue(raw_wire,100U,0U,0U,1U,7U,1U,0U);
  Px4Lite_GnssRawEnqueue(raw_wire,99U,2U,0U,1U,7U,1U,0U);Px4Lite_GnssRawEnqueue(raw_wire,100U,2U,1U,1U,7U,1U,0U);Px4Lite_GnssRawSourceReject(2U,65543U,1U,0U);
  Px4Lite_GnssRawEnqueue(raw_short,2U,1U,0U,0U,0U,0U,0U);v=RawPeekAt(0U);assert(v.length==26U && v.payload[1]==1U && v.payload[2]==2U && Read32(&v.payload[4])==7U && memcmp(&v.payload[24],raw_short,2U)==0);
  Px4Lite_GnssRawUplinkResult(PX4LITE_OK,0U);
#if PX4LITE_RPI_GNSS_RAW_STATS_ENABLE
  v=RawPeekAt(1000U);assert(Read32(&v.payload[12])==7U && Read32(&v.payload[28])==6U && Read32(&v.payload[16])==1U && Read32(&v.payload[20])==1U);
#else
  assert(s_raw.counters.input_events==7U && s_raw.counters.source_rejects==6U && s_raw.counters.enqueued_frames==1U && s_raw.counters.completed_frames==1U);
#endif
}
/** @brief 同片有界重试不跳片；三次失败释放整帧，随后新帧从offset0开始。 */
static void TestRetryAbortAndTimeout(void)
{
  Px4Lite_GnssRawUplinkView_t v;uint8_t first[64];ResetRaw(0U);MakeRaw(100U,0U);Px4Lite_GnssRawEnqueue(raw_wire,100U,2U,0U,1U,7U,1U,0U);
  v=RawPeekAt(0U);memcpy(first,v.payload,64U);Px4Lite_GnssRawUplinkResult(PX4LITE_IO_ERROR,0U);clock_ms=49U;assert(Px4Lite_GnssRawUplinkPeek(49U,&v)==PX4LITE_IDLE);
  v=RawPeekAt(50U);assert(memcmp(first,v.payload,64U)==0);Px4Lite_GnssRawUplinkResult(PX4LITE_BUSY,50U);v=RawPeekAt(100U);Px4Lite_GnssRawUplinkResult(PX4LITE_IO_ERROR,100U);Px4Lite_GnssRawUplinkResult(PX4LITE_OK,100U);
  assert(s_raw.counters.aborted_frames==1U && s_raw.counters.completed_parts==0U && s_raw.counters.error_attempts==3U && s_raw.depth==0U);
  Px4Lite_GnssRawEnqueue(raw_short,2U,1U,0U,0U,0U,0U,100U);v=RawPeekAt(100U);assert(Read32(&v.payload[4])==2U && Read16(&v.payload[14])==0U);Px4Lite_GnssRawUplinkResult(PX4LITE_OK,100U);
  ResetRaw(UINT32_MAX-1000U);MakeRaw(392U,0U);Px4Lite_GnssRawEnqueue(raw_wire,392U,2U,0U,1U,7U,1U,UINT32_MAX-1000U);v=RawPeekAt(UINT32_MAX-1000U);Px4Lite_GnssRawUplinkResult(PX4LITE_OK,UINT32_MAX-1000U);
  clock_ms=1000U;
#if PX4LITE_RPI_GNSS_RAW_STATS_ENABLE
  v=RawPeekAt(1000U);assert(v.payload_type==0x8021U && Read32(&v.payload[32])==1U && v.payload[44]==0U && Read32(&v.payload[48])==2001U);
#else
  assert(Px4Lite_GnssRawUplinkPeek(1000U,&v)==PX4LITE_IDLE && s_raw.counters.aborted_frames==1U && s_raw.depth==0U);
#endif
}
/** @brief Stats失败等待时RAW仍有机会；旧queued帧允许以原时刻发送，不冒充新定位。 */
static void TestStatsAndOldQueue(void)
{
  Px4Lite_GnssRawUplinkView_t v;
#if PX4LITE_RPI_GNSS_RAW_STATS_ENABLE
  ResetRaw(0U);MakeRaw(100U,0U);Px4Lite_GnssRawEnqueue(raw_wire,100U,2U,0U,1U,7U,1U,0U);
  v=RawPeekAt(1000U);assert(v.payload_type==0x8021U);Px4Lite_GnssRawUplinkResult(PX4LITE_IO_ERROR,1000U);
  v=RawPeekAt(1001U);assert(v.payload_type==0x8020U);Px4Lite_GnssRawUplinkResult(PX4LITE_OK,1001U);
  v=RawPeekAt(1050U);assert(v.payload_type==0x8021U);Px4Lite_GnssRawUplinkResult(PX4LITE_IO_ERROR,1050U);v=RawPeekAt(1051U);assert(v.payload_type==0x8020U);Px4Lite_GnssRawUplinkResult(PX4LITE_OK,1051U);
  v=RawPeekAt(1100U);assert(v.payload_type==0x8021U);Px4Lite_GnssRawUplinkResult(PX4LITE_IO_ERROR,1100U);v=RawPeekAt(1101U);assert(v.payload_type==0x8020U);Px4Lite_GnssRawUplinkResult(PX4LITE_OK,1101U);
  assert(!s_raw.stats_pending && s_raw.counters.completed_frames==1U && s_raw.counters.error_attempts==3U);
#endif
  ResetRaw(0U);Px4Lite_GnssRawEnqueue(raw_short,2U,1U,0U,0U,0U,0U,0U);
#if PX4LITE_RPI_GNSS_RAW_STATS_ENABLE
  v=RawPeekAt(200000U);assert(v.payload_type==0x8021U && Read32(&v.payload[56])==200000U);Px4Lite_GnssRawUplinkResult(PX4LITE_OK,200000U);
#endif
  v=RawPeekAt(200000U);assert(v.payload_type==0x8020U && Read32(&v.payload[8])==0U);Px4Lite_GnssRawUplinkResult(PX4LITE_OK,200000U);
  ResetRaw(0U);s_raw.frame_id=UINT32_MAX;Px4Lite_GnssRawEnqueue(raw_short,2U,1U,0U,0U,0U,0U,0U);v=RawPeekAt(0U);assert(Read32(&v.payload[4])==0U);Px4Lite_GnssRawUplinkResult(PX4LITE_OK,0U);
#if PX4LITE_RPI_GNSS_RAW_STATS_ENABLE
  v=RawPeekAt(1000U);assert(Read32(&v.payload[52])==1000U); /* 完成时刻0合法。 */
  ResetRaw(0U);Px4Lite_GnssRawEnqueue(raw_short,2U,1U,0U,0U,0U,0U,2000U);v=RawPeekAt(1000U);assert(Read32(&v.payload[48])==UINT32_MAX && Read32(&v.payload[56])==UINT32_MAX);
  /* RAW机会稀疏且stats一直失败时，也必须在两次stats尝试间让就绪数据一次。 */
  ResetRaw(0U);Px4Lite_GnssRawEnqueue(raw_short,2U,1U,0U,0U,0U,0U,0U);v=RawPeekAt(1000U);assert(v.payload_type==0x8021U);Px4Lite_GnssRawUplinkResult(PX4LITE_IO_ERROR,1000U);
  v=RawPeekAt(2000U);assert(v.payload_type==0x8020U);Px4Lite_GnssRawUplinkResult(PX4LITE_OK,2000U);assert(s_raw.counters.completed_frames==1U);
#else
  assert(RawAge(1000U,s_raw.last_completed_ms,s_raw.completed_seen)==1000U);
#endif
}
/** @brief RAW持续队满且有UART失败时，typed/旧目录仍推进；每call至多一次UART提交。 */
static void TestFourLaneFairness(void)
{
  uint32_t cycle,before,typed=0U,raw=0U,old=0U,telemetry=0U;uint8_t i;mavlink_message_t rx;mavlink_tunnel_t t;
  Sources(0U);assert(Px4Lite_MavlinkTxInit(0U)==PX4LITE_OK);MakeRaw(1024U,0U);s_tx_enabled=0U;
  for(cycle=0U;cycle<300U;cycle++){
    clock_ms=cycle*10U;Px4Lite_GnssRawEnqueue(raw_wire,1024U,2U,0U,1U,7U,1U,clock_ms);
    for(i=0U;i<MAV_TX_RPI_TELEM_COUNT;i++){s_rpi_telem_next_ms[i]=0U;}for(i=0U;i<MAV_TX_RPI_CATALOG_COUNT;i++){*s_mav_tx_rpi_catalog[i].next_ms=0U;}
    uart_result=(cycle%7U==0U)?PX4LITE_IO_ERROR:PX4LITE_OK;before=uart_calls;(void)Px4Lite_MavlinkTxRunRpi(clock_ms);assert(uart_calls==before+1U);rx=DecodeWire();
    if(rx.msgid==0U){telemetry++;}else if(rx.msgid==385U){mavlink_msg_tunnel_decode(&rx,&t);if(t.payload_type>=0x8011U && t.payload_type<=0x8014U){if(uart_result==PX4LITE_OK){typed++;}}else if(t.payload_type==0x8020U || t.payload_type==0x8021U){raw++;}else{old++;}}else{old++;}
  }
  assert(raw>12U && telemetry>12U && old>12U && s_raw.counters.full_drops>0U && s_raw.counters.error_attempts>0U && lora_calls==0U);
#if PX4LITE_RPI_GNSS_PARSED_TX_ENABLE && PX4LITE_ENABLE_RPI_GNSS_UPLINK
  assert(typed>=12U);
#else
  assert(typed==0U);
#endif
  uart_result=PX4LITE_OK;b1_pending=h1_pending=1U;Px4Lite_MavlinkQueueCommandAck(400U,0U,9U,1U,PX4LITE_MAVLINK_LINK_RPI);
  for(i=0U;i<3U;i++){before=uart_calls;(void)Px4Lite_MavlinkTxRunRpi(clock_ms);assert(uart_calls==before+1U);rx=DecodeWire();assert(rx.msgid==(i==2U?77U:385U));if(i<2U){mavlink_msg_tunnel_decode(&rx,&t);assert(t.payload_type==0x8003U);}}
}
/** @brief 连续最高优先H1/ACK流量期间也回收超2s活跃槽，维护不能偷偷发送RAW或双计中止。 */
static void TestTimeoutDuringPriorityReplies(void)
{
  Px4Lite_GnssRawUplinkView_t v;uint8_t i;uint32_t before;mavlink_message_t rx;mavlink_tunnel_t t;
  Sources(0U);assert(Px4Lite_MavlinkTxInit(0U)==PX4LITE_OK);MakeRaw(1024U,0U);Px4Lite_GnssRawEnqueue(raw_wire,1024U,2U,0U,1U,7U,1U,0U);
  v=RawPeekAt(0U);assert(v.payload_type==0x8020U);Px4Lite_GnssRawUplinkResult(PX4LITE_OK,0U);Px4Lite_MavlinkQueueCommandAck(400U,0U,9U,1U,PX4LITE_MAVLINK_LINK_RPI);
  for(i=0U;i<3U;i++){clock_ms=(i==0U)?1999U:2001U+(i-1U)*10U;h1_pending=1U;before=uart_calls;(void)Px4Lite_MavlinkTxRunRpi(clock_ms);assert(uart_calls==before+1U);rx=DecodeWire();mavlink_msg_tunnel_decode(&rx,&t);assert(rx.msgid==385U && t.payload_type==0x8003U);assert(s_raw.counters.aborted_frames==(i==0U?0U:1U));}
  assert(s_raw.depth==0U && s_raw.counters.completed_frames==0U && s_raw.counters.completed_parts==1U);before=uart_calls;(void)Px4Lite_MavlinkTxRunRpi(clock_ms);assert(uart_calls==before+1U && DecodeWire().msgid==77U && s_raw.counters.aborted_frames==1U);
}
/** @brief 连续提供独立本地事实，让真实旧编码器有数据，避免靠空topic掩盖误停消息。 */
#if !PX4LITE_RPI_GNSS_RAW_STATS_ENABLE
static void RefreshLocalFacts(uint32_t now)
{
  uint8_t i;extra_ready=1U;source_gnss.header.valid=1U;source_gnss.header.sequence++;
  source_gnss.header.sample_time_ms=source_gnss.last_rx_ms=source_gnss.position_update_ms=source_gnss.velocity_update_ms=source_gnss.time_update_ms=source_gnss.altitude_update_ms=source_gnss.status_update_ms=now;source_gnss.hdop_x100=105U;
  source_time.header.sample_time_ms=now;source_diag.measured_ms=now;
  memset(&extra_nav,0,sizeof(extra_nav));extra_nav.header.valid=1U;extra_nav.header.sequence=source_gnss.header.sequence;extra_nav.header.sample_time_ms=now;extra_nav.attitude_update_ms=now;
  extra_nav.valid_mask=PX4LITE_NAV_VALID_ATTITUDE|PX4LITE_NAV_VALID_POSITION|PX4LITE_NAV_VALID_ALTITUDE|PX4LITE_NAV_VALID_VELOCITY|PX4LITE_NAV_VALID_TIME;
  extra_nav.latitude_e7=source_gnss.latitude_e7;extra_nav.longitude_e7=source_gnss.longitude_e7;extra_nav.fused_altitude_mm=source_gnss.altitude_mm;extra_nav.gnss_fix_type=4U;extra_nav.roll_deg100=123U;
  memset(&extra_baro,0,sizeof(extra_baro));extra_baro.header.valid=1U;extra_baro.header.sequence=source_gnss.header.sequence;extra_baro.header.sample_time_ms=now;extra_baro.pressure_pa=100000.0f;extra_baro.temperature_c=25.0f;extra_baro.relative_humidity_pct=50.0f;
  memset(&extra_battery,0,sizeof(extra_battery));extra_battery.header=extra_baro.header;extra_battery.voltage_mv=12000U;extra_battery.current_ma=1000;extra_battery.current_valid=1U;
  memset(&extra_motor,0,sizeof(extra_motor));extra_motor.header=extra_baro.header;for(i=0U;i<4U;i++){extra_motor.pulse_us[i]=1000U;}
  memset(&extra_health,0,sizeof(extra_health));extra_health.header=extra_baro.header;for(i=0U;i<PX4LITE_MODULE_COUNT;i++){extra_health.module_state[i]=PX4LITE_STATE_ONLINE;}
}
#endif
#if !PX4LITE_RPI_GNSS_PARSED_TX_ENABLE
/** @brief 最后出口门禁阻止误开typed/直接镜像旁路；不改调用者即将发往UART4的原帧。 */
static void TestParsedPolicyAndIdentityMirrors(void)
{
  static const uint32_t ids[]={24U,33U,12901U,12904U};static const char *names[]={"GNSS_SAT","GNSSUTC","BAROALT"};
  uint8_t i;uint32_t before;uint8_t rpi_seq; mavlink_message_t msg,original; mavlink_named_value_int_t named;mavlink_tunnel_t tunnel;
  for(i=0U;i<4U;i++){memset(&msg,0,sizeof(msg));msg.msgid=ids[i];original=msg;before=uart_calls;rpi_seq=s_rpi_tx_seq;assert(Px4Lite_MavlinkTxMirrorToRpi(&msg)==PX4LITE_IDLE && uart_calls==before && s_rpi_tx_seq==rpi_seq && memcmp(&msg,&original,sizeof(msg))==0);}
  for(i=0U;i<3U;i++){uint8_t lora_seq;memset(&named,0,sizeof(named));memcpy(named.name,names[i],strlen(names[i]));mavlink_msg_named_value_int_encode(42U,192U,&msg,&named);original=msg;before=uart_calls;assert(Px4Lite_MavlinkTxMirrorToRpi(&msg)==PX4LITE_IDLE && uart_calls==before && memcmp(&msg,&original,sizeof(msg))==0);
    lora_seq=mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq;mavlink_msg_named_value_int_encode_chan(42U,193U,MAVLINK_COMM_0,&s_message,&named);assert(MavTx_SendRpiExclusive()==PX4LITE_IDLE && uart_calls==before && mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq==lora_seq);}
  for(i=0U;i<4U;i++){memset(&tunnel,0,sizeof(tunnel));tunnel.payload_type=(uint16_t)(0x8011U+i);mavlink_msg_tunnel_encode(42U,193U,&msg,&tunnel);before=uart_calls;assert(Px4Lite_MavlinkTxMirrorToRpi(&msg)==PX4LITE_IDLE && uart_calls==before);}
  before=uart_calls;assert(MavTx_SendRpiGnssUplink(clock_ms)==PX4LITE_IDLE && uart_calls==before);
#if !PX4LITE_RPI_GNSS_RAW_STATS_ENABLE
  memset(&tunnel,0,sizeof(tunnel));tunnel.payload_type=0x8021U;mavlink_msg_tunnel_encode(42U,193U,&msg,&tunnel);before=uart_calls;assert(Px4Lite_MavlinkTxMirrorToRpi(&msg)==PX4LITE_IDLE && uart_calls==before);
#endif
  for(i=0U;i<3U;i++){
    if(i==0U){mavlink_open_drone_id_basic_id_t p;memset(&p,0,sizeof(p));mavlink_msg_open_drone_id_basic_id_encode(42U,192U,&msg,&p);}
    else if(i==1U){mavlink_open_drone_id_self_id_t p;memset(&p,0,sizeof(p));mavlink_msg_open_drone_id_self_id_encode(42U,192U,&msg,&p);}
    else{mavlink_open_drone_id_operator_id_t p;memset(&p,0,sizeof(p));mavlink_msg_open_drone_id_operator_id_encode(42U,192U,&msg,&p);}
    original=msg;before=uart_calls;assert(Px4Lite_MavlinkTxMirrorToRpi(&msg)==PX4LITE_OK && uart_calls==before+1U && memcmp(&msg,&original,sizeof(msg))==0);assert(DecodeWire().msgid==original.msgid);
  }
}
#endif
#if PX4LITE_RPI_GNSS_PARSED_TX_ENABLE
/** @brief 默认恢复LOCATION/SYSTEM镜像，且不改调用者继续发往硬件RemoteID的原消息。 */
static void TestParsedMirrorsRestored(void)
{
  mavlink_message_t msg,original;uint8_t i;uint32_t before;
  for(i=0U;i<2U;i++){
    if(i==0U){mavlink_open_drone_id_location_t p;memset(&p,0,sizeof(p));p.latitude=-123456789L;p.longitude=987654321L;mavlink_msg_open_drone_id_location_encode(42U,192U,&msg,&p);}
    else{mavlink_open_drone_id_system_t p;memset(&p,0,sizeof(p));mavlink_msg_open_drone_id_system_encode(42U,192U,&msg,&p);}
    original=msg;before=uart_calls;assert(Px4Lite_MavlinkTxMirrorToRpi(&msg)==PX4LITE_OK && uart_calls==before+1U && memcmp(&msg,&original,sizeof(msg))==0);assert(DecodeWire().msgid==(i==0U?12901U:12904U));
  }
#if !PX4LITE_RPI_GNSS_RAW_STATS_ENABLE
  {mavlink_tunnel_t packet;memset(&packet,0,sizeof(packet));packet.payload_type=0x8021U;mavlink_msg_tunnel_encode(42U,193U,&msg,&packet);before=uart_calls;assert(Px4Lite_MavlinkTxMirrorToRpi(&msg)==PX4LITE_IDLE && uart_calls==before);}
#endif
}
#endif
#if !PX4LITE_RPI_GNSS_RAW_STATS_ENABLE
/** @brief 六秒真实RunRpi验证默认解析+RAW并行及显式raw-only策略；统计仍关闭。 */
static void TestDefaultGpsStream(void)
{
  uint32_t cycle,before,frames=0U,current_id=0U,offset=0U,seen=0U;uint8_t assembled[100];mavlink_message_t rx;mavlink_tunnel_t t;mavlink_named_value_int_t named;
  uint32_t parsed_seen=0U,typed_id=0U,typed_ms=0U,typed_groups=0U;uint8_t typed_mask=0U;
  App_GnssDetails_t local_gnss;App_NavigationSnapshot_t local_nav;Px4Lite_TimeSnapshot_t local_time;
  Sources(0U);assert(Px4Lite_MavlinkTxInit(0U)==PX4LITE_OK);MakeRaw(100U,1U);
  for(cycle=0U;cycle<600U;cycle++){
    clock_ms=cycle*10U;RefreshLocalFacts(clock_ms);
    if(cycle%20U==0U){Px4Lite_GnssRawEnqueue(raw_wire,100U,2U,0U,1U,7U,0U,clock_ms);}
    before=uart_calls;(void)Px4Lite_MavlinkTxRunRpi(clock_ms);assert(uart_calls<=before+1U);if(uart_calls==before){continue;}
    rx=DecodeWire();
    if(!PX4LITE_RPI_GNSS_PARSED_TX_ENABLE){assert(rx.msgid!=24U && rx.msgid!=33U && rx.msgid!=12901U && rx.msgid!=12904U);}
    if(rx.msgid==24U){parsed_seen|=1U;}if(rx.msgid==33U){parsed_seen|=2U;}
    if(rx.msgid==0U){seen|=1U;}if(rx.msgid==30U){seen|=2U;}if(rx.msgid==147U){mavlink_battery_status_t battery;mavlink_msg_battery_status_decode(&rx,&battery);assert(battery.id==1U && battery.voltages[0]==12000U);seen|=4U;}
    if(rx.msgid==29U){seen|=8U;}if(rx.msgid==36U){seen|=16U;}if(rx.msgid==1U){seen|=32U;}
    if(rx.msgid==252U){mavlink_msg_named_value_int_decode(&rx,&named);if(memcmp(named.name,"GNSS_SAT",8U)==0){parsed_seen|=4U;}if(memcmp(named.name,"GNSSUTC",7U)==0){parsed_seen|=8U;}if(memcmp(named.name,"BAROALT",7U)==0){parsed_seen|=16U;}if(memcmp(named.name,"MOTOR",5U)==0){seen|=64U;}}
    if(rx.msgid==385U){mavlink_msg_tunnel_decode(&rx,&t);assert(t.payload_type!=0x8021U);
      if(t.payload_type>=0x8011U && t.payload_type<=0x8014U){uint32_t id=Read32(&t.payload[4]);uint8_t bit;assert(PX4LITE_RPI_GNSS_PARSED_TX_ENABLE && PX4LITE_ENABLE_RPI_GNSS_UPLINK);if(id!=typed_id){typed_id=id;typed_ms=Read32(&t.payload[8]);typed_mask=0U;}assert(t.payload[0]==1U && Read32(&t.payload[8])==typed_ms && t.payload[3]==((t.payload_type==0x8012U || t.payload_type==0x8014U)?2U:1U));if(t.payload_type==0x8011U){assert((t.payload[12]&5U)==5U && Read32(&t.payload[40])==20261106U && Read32(&t.payload[44])==45296U);}bit=(t.payload_type==0x8011U)?0U:(t.payload_type==0x8012U)?(uint8_t)(1U+t.payload[2]):(t.payload_type==0x8013U)?3U:(uint8_t)(4U+t.payload[2]);assert(bit<6U);typed_mask|=(uint8_t)(1U<<bit);if(typed_mask==63U){typed_groups++;typed_mask=0U;}}
      if(t.payload_type==0x8001U){seen|=128U;}
      if(t.payload_type==0x8020U){uint32_t id=Read32(&t.payload[4]);uint16_t part_offset=Read16(&t.payload[14]);if(!part_offset){current_id=id;offset=0U;}assert(id==current_id && part_offset==offset && Read16(&t.payload[12])==100U && t.payload[2]==2U && Read16(&t.payload[22])==RefCrc(raw_original,100U));memcpy(&assembled[offset],&t.payload[24],t.payload[18]);offset+=t.payload[18];if(offset==100U){assert(memcmp(assembled,raw_original,100U)==0);frames++;}}
    }
  }
  assert(seen==255U && frames>=20U && s_raw.counters.completed_frames==frames && lora_calls==0U);
  if(PX4LITE_RPI_GNSS_PARSED_TX_ENABLE){assert(parsed_seen==31U);if(PX4LITE_ENABLE_RPI_GNSS_UPLINK){assert(typed_groups>=1U);}else{assert(typed_groups==0U);}}
  else{assert(parsed_seen==0U && typed_groups==0U);}
  before=gnss_reads;assert(App_CopyGnssDetails(&local_gnss,clock_ms)==PX4LITE_OK && gnss_reads>before && local_gnss.latitude_e7==-123456789L);assert(App_CopyNavigation(&local_nav,clock_ms)==PX4LITE_OK);assert(Px4Lite_CopyTime(&local_time)==PX4LITE_OK && local_time.utc_time_hhmmss==123456U);
  s_send_target=MAV_TX_TARGET_LORA;before=lora_calls;assert(MavTx_SendGpsRaw(clock_ms)==PX4LITE_OK && s_message.msgid==24U && lora_calls==before+1U);assert(MavTx_SendGnssDetail(clock_ms)==PX4LITE_OK);assert(MavTx_SendDateTime(clock_ms)==PX4LITE_OK);assert(MavTx_SendPosition(clock_ms)==PX4LITE_OK);
  printf("GPS stream: PASS (600 RunRpi calls/6s; parsed=%u, %u typed6-part groups, %u complete original BAD frames; no8021; other telemetry/App/LoRa/RTC retained)\n",(unsigned)PX4LITE_RPI_GNSS_PARSED_TX_ENABLE,(unsigned)typed_groups,(unsigned)frames);
}
#endif
#endif
int main(void)
{
#if PX4LITE_ENABLE_RPI_GNSS_RAW_UPLINK
  TestRawGoldenWire();TestCompleteNmea();TestRingPinAndMaximum();TestRejectAndShortNmea();TestRetryAbortAndTimeout();TestStatsAndOldQueue();TestFourLaneFairness();TestTimeoutDuringPriorityReplies();
#if !PX4LITE_RPI_GNSS_PARSED_TX_ENABLE
  TestParsedPolicyAndIdentityMirrors();
#else
  TestParsedMirrorsRestored();
#endif
#if !PX4LITE_RPI_GNSS_RAW_STATS_ENABLE
  TestDefaultGpsStream();
#endif
  printf("RAW static state=%uB slot=%uB\n",(unsigned)sizeof(s_raw),(unsigned)sizeof(RawSlot_t));
  puts("GNSS RAW uplink: PASS (original wire / CRC29B1 / 4 slots / pinned / 26 parts / reject / retries / timeout / stats / 4 lanes)");
#else
  Sources(0U);s_send_target=MAV_TX_TARGET_RPI;assert(MavTx_SendHeartbeat(0U)==PX4LITE_OK && DecodeWire().msgid==0U);assert(Read16(reply_bytes)==0x0401U && Read32(reply_bytes)==0x0401U);
  {Px4Lite_GnssRawUplinkView_t v;uint32_t before=uart_calls;Px4Lite_GnssRawUplinkInit(0U);Px4Lite_GnssRawEnqueue((const uint8_t *)"$\n",2U,1U,0U,0U,0U,0U,0U);Px4Lite_GnssRawSourceReject(0U,0U,2U,0U);Px4Lite_GnssRawUplinkPollTimeout(10000U);assert(Px4Lite_GnssRawUplinkPeek(10000U,&v)==PX4LITE_IDLE);Px4Lite_GnssRawUplinkResult(PX4LITE_OK,10000U);assert(MavTx_SendRpiGnssRawUplink(10000U)==PX4LITE_IDLE && uart_calls==before);}
  puts("GNSS RAW uplink disabled: PASS (no queue/data/stats/UART traffic)");
#endif
  assert(raw_critical_depth==0U);return 0;
}
