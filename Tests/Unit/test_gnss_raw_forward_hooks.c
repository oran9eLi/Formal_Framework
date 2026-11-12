/**
 * @file test_gnss_raw_forward_hooks.c
 * @brief 使用真实Sensor、Adapter tee及屏幕捕获验证独立RAW来源事件与原字节。
 * @details RAW队列入口为计数/复制替身；队列编码由独立Framework回归验证，不产生实板伪数据。
 */
#include <stdint.h>
#include <assert.h>
static uint32_t s_driver_critical_depth;
#define BSP_CRITICAL_H
static uint32_t BSP_Critical_Enter(void) { return s_driver_critical_depth++; }
static void BSP_Critical_Exit(uint32_t saved) { assert(s_driver_critical_depth==saved+1U);s_driver_critical_depth=saved; }

#define Sensor_GNSS_Service FixtureAdapterSensorService
#define Sensor_GNSS_CopySnapshot FixtureAdapterCopySnapshot
#define Sensor_GNSS_RequestReinit FixtureAdapterRequestReinit
#define main FixtureStatusDomainMain
#include "test_gnss_status_domain.c"
#undef main
#undef Sensor_GNSS_Service
#undef Sensor_GNSS_CopySnapshot
#undef Sensor_GNSS_RequestReinit
#include "../../Sensor/Src/sensor_nmea.c"
#include "../../Sensor/Src/sensor_ubx.c"
#include "../../Sensor/Src/sensor_gnss.c"
#include "../../Framework/Src/px4lite_diagnostic_capture.c"

static uint8_t s_input[2048],s_forwarded[GNSS_RAW_FORWARD_MAX_BYTES];
static uint16_t s_input_size,s_forwarded_length;
static uint32_t s_forward_count,s_reject_count,s_reject_length,s_reject_ms,s_forward_ms,s_drop;
static uint8_t s_forward_protocol,s_forward_crc,s_forward_class,s_forward_id,s_forward_type;
static uint8_t s_reject_protocol,s_reject_reason,s_recover_request;
static uint32_t s_raw_init_calls;

/** @brief 接收机输入固定115200；不产生发送回环或真实硬件访问。 */
uint16_t BSP_GNSS_GetRxCount(void) { return s_input_size; }
uint16_t BSP_GNSS_GetRxData(uint8_t *out,uint16_t capacity)
{ uint16_t n=s_input_size<capacity?s_input_size:capacity;memcpy(out,s_input,n);memmove(s_input,s_input+n,s_input_size-n);s_input_size=(uint16_t)(s_input_size-n);return n; }
uint32_t BSP_GNSS_GetDropCount(void) { return s_drop; }
void BSP_GNSS_RxIdleCallback(uint16_t unused) { (void)unused; }
void BSP_GNSS_RequestRecoverRx(void) { s_recover_request=1U; }
uint8_t BSP_GNSS_ConsumeRecoverRxRequest(void) { uint8_t p=s_recover_request;s_recover_request=0U;return p; }
BSP_Status_t BSP_GNSS_RecoverRx(void) { s_input_size=0U;return BSP_STATUS_OK; }
BSP_Status_t BSP_GNSS_Transmit(const uint8_t *p,uint16_t n) { assert(p!=NULL && n>0U);return BSP_STATUS_OK; }
/** @brief 独立TX复位不消费或清空待转发的原始接收字节。 */
BSP_Status_t BSP_GNSS_AbortTx(void) { return BSP_STATUS_OK; }
BSP_Status_t BSP_GNSS_SetBaudRate(uint32_t baud) { assert(baud==115200U);return BSP_STATUS_OK; }
BSP_Status_t BSP_GNSS_CopyTransportInfo(BSP_GNSS_TransportInfo_t *out)
{ memset(out,0,sizeof(*out));out->baud_bps=115200U;out->initialized=1U;return BSP_STATUS_OK; }
void Lora_E22_SetDiagnosticCallbacks(Lora_DiagnosticRx_t rx,Lora_DiagnosticTx_t tx) { (void)rx;(void)tx; }

/** @brief 只复制已收到原帧，断言回调在Driver临界区外；不解码或重算CRC。 */
void Px4Lite_GnssRawEnqueue(const uint8_t *bytes,uint16_t length,uint8_t protocol,uint8_t type,uint8_t cls,uint8_t id,uint8_t checksum,uint32_t now)
{
  assert(s_driver_critical_depth==0U && bytes!=NULL && length>0U && length<=sizeof(s_forwarded));
  memcpy(s_forwarded,bytes,length);s_forwarded_length=length;s_forward_count++;
  s_forward_protocol=protocol;s_forward_crc=checksum;s_forward_class=cls;s_forward_id=id;s_forward_type=type;s_forward_ms=now;
}
/** @brief 源事件单独计数，不将候选长度当作完整线帧或物理丢帧。 */
void Px4Lite_GnssRawSourceReject(uint8_t protocol,uint32_t length,uint8_t reason,uint32_t now)
{
  assert(s_driver_critical_depth==0U);s_reject_count++;s_reject_protocol=protocol;s_reject_length=length;s_reject_reason=reason;s_reject_ms=now;
}
void Px4Lite_GnssRawUplinkInit(uint32_t now) { (void)now;s_raw_init_calls++; }

/** @brief 主机冷启动准备；真实PlatformInit注册所有tee，RAW队列Init由Comm/MavTx所有者负责。 */
static void ColdForward(void)
{
  s_input_size=0U;s_forward_count=0U;s_reject_count=0U;s_raw_init_calls=0U;s_drop=0U;s_recover_request=0U;
  s_line_pos=0U;s_ubx_candidate_bytes=0U;memset(&s_ubx,0,sizeof(s_ubx));memset(s_pending_raw_reject,0,sizeof(s_pending_raw_reject));
  s_driver_started=0U;s_driver_critical_depth=0U;assert(Sensor_GNSS_Init()==GNSS_RESULT_OK);
  assert(Px4Lite_PlatformInit()==PX4LITE_OK && s_raw_init_calls==0U);
  assert(GNSS_RAW_FORWARD_MAX_BYTES==PX4LITE_GNSS_RAW_UPLINK_FRAME_MAX && GNSS_RAW_MAX_BYTES==392U);
}
static void FeedBytes(const uint8_t *bytes,uint16_t length,uint32_t now)
{
  assert(s_input_size+length<=sizeof(s_input));memcpy(s_input+s_input_size,bytes,length);s_input_size=(uint16_t)(s_input_size+length);
  assert(Sensor_GNSS_Service(now)==GNSS_RESULT_OK);
  while(s_input_size) { now+=10U;assert(Sensor_GNSS_Service(now)==GNSS_RESULT_OK); }
}
static void SameWire(const uint8_t *bytes,uint16_t length,uint8_t protocol,uint8_t checksum)
{ assert(s_forwarded_length==length && s_forward_protocol==protocol && s_forward_crc==checksum && !memcmp(s_forwarded,bytes,length)); }

/** @brief 暂停/过滤及近期PVT只限制屏幕/typed，不能挡任一协议完整原帧，BAD与短缺校验也保真。 */
static void TestCompleteTeeAndUiIndependence(void)
{
  static const uint8_t nmea[]="$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
  uint8_t payload[92]={0},wire[100],bad_nmea[sizeof(nmea)-1U];
  Gnss_Snapshot_t snapshot;Px4Lite_CaptureStatus_t status;
  ColdForward();s_status.source=PX4LITE_CAPTURE_GNSS;s_status.running=0U;s_status.filter=PX4LITE_CAPTURE_FILTER_GGA;
  memcpy(payload,nmea,sizeof(nmea)-1U);payload[20]=0U;payload[21]=0U;payload[23]=0U; /* UBX内部$..LF不能另造NMEA事件。 */
  assert(Ubx_BuildFrame(1U,7U,payload,sizeof(payload),wire,sizeof(wire))==sizeof(wire));
  FeedBytes(wire,sizeof(wire),0U);SameWire(wire,sizeof(wire),2U,1U);
  assert(s_forward_count==1U && s_forward_ms==0U && s_forward_class==1U && s_forward_id==7U);
  FeedBytes(nmea,sizeof(nmea)-1U,10U);SameWire(nmea,sizeof(nmea)-1U,1U,1U);
  assert(s_forward_count==2U && s_forward_type==1U);
  assert(Sensor_GNSS_CopySnapshot(&snapshot)==GNSS_RESULT_NO_FIX && !(snapshot.valid_mask&GNSS_VALID_POSITION));
  (void)Px4Lite_CaptureCopyStatus(&status);assert(status.count==0U && (status.gnss_seen_mask&3U)==3U);
  s_status.running=1U;s_status.filter=PX4LITE_CAPTURE_FILTER_UBX;
  memcpy(bad_nmea,nmea,sizeof(bad_nmea));bad_nmea[sizeof(bad_nmea)-4U]='0';bad_nmea[sizeof(bad_nmea)-3U]='0';
  FeedBytes(bad_nmea,sizeof(bad_nmea),20U);SameWire(bad_nmea,sizeof(bad_nmea),1U,0U);assert(s_forward_count==3U && s_status.count==0U);
  wire[98]^=0x55U;wire[99]=0x00U;FeedBytes(wire,sizeof(wire),30U);SameWire(wire,sizeof(wire),2U,0U);
  assert(s_forward_count==4U && s_status.count==1U && s_reject_count==0U);
  FeedBytes((const uint8_t*)"$\n",2U,40U);SameWire((const uint8_t*)"$\n",2U,1U,0U);assert(s_forward_count==5U);
}

/** @brief 超长UBX好/坏CRC均一次源拒绝；NMEA NULL旧回调只屏幕，不重复RAW输入事件。 */
static void TestOversizeRejectOnce(void)
{
  uint8_t payload[UBX_PAYLOAD_CAPACITY+1U]={0},wire[UBX_PAYLOAD_CAPACITY+9U],line[180];
  ColdForward();s_status.source=PX4LITE_CAPTURE_GNSS;s_status.running=1U;s_status.filter=PX4LITE_CAPTURE_FILTER_ALL;
  payload[100]='$';payload[101]='X';payload[102]='\n';
  assert(Ubx_BuildFrame(0x99U,1U,payload,sizeof(payload),wire,sizeof(wire))==sizeof(wire));
  FeedBytes(wire,sizeof(wire),0U);assert(s_reject_count==1U && s_reject_length==1025U && s_reject_protocol==2U && s_reject_reason==GNSS_RAW_REJECT_OVERSIZE && s_forward_count==0U);
  wire[sizeof(wire)-1U]^=1U;FeedBytes(wire,sizeof(wire),30U);assert(s_reject_count==2U && s_forward_count==0U);
  memset(line,'1',sizeof(line));line[0]='$';line[178]='\r';line[179]='\n';FeedBytes(line,sizeof(line),60U);
  assert(s_reject_count==3U && s_reject_protocol==1U && s_reject_length==129U && s_forward_count==0U && s_status.capture_drops==1U);
}

/** @brief 32/64/84颗NAV-SAT完整400/784/1024B保真转发，旧392B镜像不被前缀截断替换。 */
static void TestLongSatWireAndLegacyMirror(void)
{
  static const uint8_t counts[3]={32U,64U,84U};
  uint8_t payload[UBX_PAYLOAD_CAPACITY],wire[GNSS_RAW_FORWARD_MAX_BYTES],small[8],mirror[GNSS_RAW_MAX_BYTES];
  Gnss_RawFrameInfo_t info;uint16_t length;unsigned i,j;
  ColdForward();s_status.source=PX4LITE_CAPTURE_GNSS;s_status.running=1U;s_status.filter=PX4LITE_CAPTURE_FILTER_UBX;
  assert(Ubx_BuildFrame(0x99U,1U,NULL,0U,small,sizeof(small))==sizeof(small));FeedBytes(small,sizeof(small),0U);
  assert(Sensor_GNSS_CopyRawFrame(2U,mirror,sizeof(mirror),&info)==GNSS_RESULT_OK && info.length==8U && info.sequence==0U);
  for(i=0U;i<3U;i++) {
    length=(uint16_t)(8U+12U*counts[i]);memset(payload,0,sizeof(payload));payload[4]=1U;payload[5]=counts[i];
    for(j=8U;j<length;j++) { payload[j]=(uint8_t)(j*17U); }
    assert(Ubx_BuildFrame(1U,0x35U,payload,length,wire,sizeof(wire))==(uint16_t)(length+8U));
    length=(uint16_t)(length+8U);FeedBytes(wire,length,100U+i*100U);SameWire(wire,length,2U,1U);
    assert(s_forward_count==i+2U && s_reject_count==0U && s_forward_class==1U && s_forward_id==0x35U);
    assert(Sensor_GNSS_CopyRawFrame(2U,mirror,sizeof(mirror),&info)==GNSS_RESULT_OK && info.length==8U && info.sequence==0U && !memcmp(mirror,small,sizeof(small)));
  }
  assert(s_status.capture_drops==3U); /* 屏幕上限392，独立RAW上限1024，两个状态不能混淆。 */
}

/** @brief 最大u16声明长度不能越过owner半帧期限或伪造成完整RAW；codec全长度另有独立覆盖。 */
static void TestUbx65535Length(void)
{
  static const uint8_t header[6]={0xB5U,0x62U,0x99U,1U,0xFFU,0xFFU};
  uint8_t zeros[256]={0},ck[2];uint32_t remaining=65535U,now=0U;uint16_t n;unsigned i;
  ColdForward();ck[0]=0U;ck[1]=0U;
  for(i=2U;i<6U;i++){ck[0]=(uint8_t)(ck[0]+header[i]);ck[1]=(uint8_t)(ck[1]+ck[0]);}
  FeedBytes(header,sizeof(header),now);
  while(remaining){n=remaining>sizeof(zeros)?sizeof(zeros):(uint16_t)remaining;for(i=0U;i<n;i++){ck[1]=(uint8_t)(ck[1]+ck[0]);}now+=10U;FeedBytes(zeros,n,now);remaining-=n;}
  FeedBytes(ck,sizeof(ck),now+10U);
  assert(s_reject_count==1U && s_reject_reason==GNSS_RAW_REJECT_FRAMING &&
      s_reject_ms==500U && s_reject_length==6U+49U*256U && s_reject_protocol==2U && s_forward_count==0U);
  assert(!Ubx_IsReceiving(&s_ubx));
  {
    static const uint8_t complete[]="$GPGSV,1,1,00*79\r\n";
    FeedBytes(complete,sizeof(complete)-1U,now+20U);
    assert(s_forward_count==1U && s_reject_count==1U);
    SameWire(complete,sizeof(complete)-1U,1U,1U);
  }
}

/** @brief 半帧重同步、丢字节、成功恢复与无时钟warmInit均有依据才记一次；0ms/wrap合法。 */
static void TestFramingBoundaries(void)
{
  static const uint8_t partial[]="$GPGGA,123";
  static const uint8_t complete[]="$GPGSV,1,1,00*79\r\n";
  uint8_t ubx_half[3]={0xB5U,0x62U,1U};
  ColdForward();FeedBytes(partial,sizeof(partial)-1U,UINT32_MAX-9U);
  assert(Sensor_GNSS_Init()==GNSS_RESULT_OK && s_reject_count==0U);
  assert(Sensor_GNSS_Service(0U)==GNSS_RESULT_OK);
  assert(s_reject_count==1U && s_reject_protocol==1U && s_reject_length==sizeof(partial)-1U && s_reject_ms==0U);
  FeedBytes(partial,sizeof(partial)-1U,10U);FeedBytes(complete,sizeof(complete)-1U,20U);
  assert(s_reject_count==2U && s_reject_reason==GNSS_RAW_REJECT_FRAMING && s_forward_count==1U);
  FeedBytes(partial,sizeof(partial)-1U,30U);s_drop++;FeedBytes(complete,sizeof(complete)-1U,40U);
  assert(s_reject_count==3U && s_forward_count==2U);
  FeedBytes(ubx_half,sizeof(ubx_half),50U);s_recover_request=1U;assert(Sensor_GNSS_Service(60U)==GNSS_RESULT_OK);
  assert(s_reject_count==4U && s_reject_protocol==2U && s_reject_length==3U && s_reject_ms==60U);
  assert(Sensor_GNSS_Service(70U)==GNSS_RESULT_OK && s_reject_count==4U);
}

int main(void)
{
  TestCompleteTeeAndUiIndependence();TestOversizeRejectOnce();TestLongSatWireAndLegacyMirror();TestUbx65535Length();TestFramingBoundaries();
  puts("GNSS RAW forwarding hooks passed: realSensor+Adapter,UIpause/filter,BAD,bounded-oversize,reject-once,resync,0ms/wrap");return 0;
}
