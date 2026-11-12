/**
 * @file test_gnss_bringup.c
 * @brief 主机验证真实GNSS驱动的固定115200、版本门控、配置确认及双协议测量转换。
 * @details BSP为可失败的独立队列/发送替身，输入全是测试向量，不写入实板或伪造生产定位。
 */
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
typedef struct { int unused; } UART_HandleTypeDef;
#define BSP_CRITICAL_H
static uint32_t BSP_Critical_Enter(void) { return 0U; }
static void BSP_Critical_Exit(uint32_t saved) { (void)saved; }
#include "../../Sensor/Src/sensor_nmea.c"
#include "../../Sensor/Src/sensor_ubx.c"
#include "../../Sensor/Src/sensor_gnss.c"

static uint8_t input[4096], commands[64][96];
static uint16_t input_size, sizes[64];
static uint32_t current_baud, commands_count, requested_recoveries, actual_recoveries, fake_drop;
static uint32_t baud_set_calls;
static uint8_t initialized, busy, io_fail;
static uint32_t aborted_tx_count;
static uint8_t abort_tx_fail;

uint16_t BSP_GNSS_GetRxCount(void) { return input_size; }
uint16_t BSP_GNSS_GetRxData(uint8_t *out, uint16_t capacity)
{ uint16_t n=input_size<capacity?input_size:capacity; memcpy(out,input,n); memmove(input,input+n,input_size-n); input_size=(uint16_t)(input_size-n); return n; }
uint32_t BSP_GNSS_GetDropCount(void) { return fake_drop; }
void BSP_GNSS_RxIdleCallback(uint16_t unused) { (void)unused; }
void BSP_GNSS_RequestRecoverRx(void) { requested_recoveries=1U; }
uint8_t BSP_GNSS_ConsumeRecoverRxRequest(void) { uint8_t value=(uint8_t)requested_recoveries; requested_recoveries=0U; return value; }
BSP_Status_t BSP_GNSS_RecoverRx(void) { actual_recoveries++; if(io_fail) { return BSP_STATUS_ERROR; } initialized=1U; busy=0U; return BSP_STATUS_OK; }
/** @brief 发送独立撤销保留接收队列，硬件撤销失败可单独注入。 */
BSP_Status_t BSP_GNSS_AbortTx(void)
{ aborted_tx_count++;if(abort_tx_fail) { return BSP_STATUS_ERROR; }busy=0U;return BSP_STATUS_OK; }
BSP_Status_t BSP_GNSS_SetBaudRate(uint32_t baud)
{ assert(baud==115200U);baud_set_calls++;if(busy) { return BSP_STATUS_BUSY; } if(io_fail) { return BSP_STATUS_ERROR; } current_baud=baud; input_size=0U; return BSP_STATUS_OK; }
BSP_Status_t BSP_GNSS_CopyTransportInfo(BSP_GNSS_TransportInfo_t *out)
{ memset(out,0,sizeof(*out)); out->initialized=initialized; out->tx_busy=busy; out->baud_bps=current_baud; out->rx_drop_bytes=fake_drop; return BSP_STATUS_OK; }
BSP_Status_t BSP_GNSS_Transmit(const uint8_t *bytes, uint16_t length)
{
  if(busy) { return BSP_STATUS_BUSY; } if(io_fail) { return BSP_STATUS_ERROR; }
  assert(length<=96U && commands_count<64U); memcpy(commands[commands_count],bytes,length); sizes[commands_count]=length; commands_count++; return BSP_STATUS_OK;
}

/** @brief 每个场景显式模拟一次新的上电；正常warm Init测试不使用此入口。 */
static void Cold(void)
{
  input_size=0U; commands_count=0U; requested_recoveries=0U; actual_recoveries=0U;
  current_baud=115200U; initialized=1U; busy=0U; io_fail=0U; fake_drop=0U;
  baud_set_calls=0U;
  aborted_tx_count=0U;abort_tx_fail=0U;
  s_driver_started=0U; s_diag_ready=0U;
  assert(Sensor_GNSS_Init()==GNSS_RESULT_OK);
}
static void Step(uint32_t now) { assert(Sensor_GNSS_Service(now)==GNSS_RESULT_OK); }
static Gnss_Diagnostics_t Diag(void) { Gnss_Diagnostics_t out; assert(Sensor_GNSS_CopyDiagnostics(&out)==GNSS_RESULT_OK); return out; }
static void Raw(const uint8_t *bytes, uint16_t n) { assert(input_size+n<=sizeof(input)); memcpy(input+input_size,bytes,n); input_size=(uint16_t)(input_size+n); }
static void Frame(uint8_t c, uint8_t id, const uint8_t *p, uint16_t n, uint32_t now)
{ uint8_t packet[512]; uint16_t size=Ubx_BuildFrame(c,id,p,n,packet,sizeof(packet)); assert(size); Raw(packet,size); Step(now); }
static void Sentence(const char *body, uint32_t now)
{ char line[128]; uint8_t checksum=0U; const char *p; int n; for(p=body;*p;p++) { checksum^=(uint8_t)*p; } n=snprintf(line,sizeof(line),"$%s*%02X\r\n",body,checksum); assert(n>0 && n<128); Raw((const uint8_t*)line,(uint16_t)n); Step(now); }

/** @brief 标准M10 MON-VER测试向量，扩展按原厂固定30B排列。 */
static void Version(uint16_t version, uint32_t now)
{
  uint8_t p[70]; memset(p,0,sizeof(p)); memcpy(p,"SPG 5.10",8U); memcpy(p+30,"000A0000",8U);
  (void)snprintf((char*)p+40,30U,"PROTVER=%u.%02u",(unsigned)(version/100U),(unsigned)(version%100U));
  Frame(0x0AU,0x04U,p,sizeof(p),now);
}
static void Ack(uint8_t id, uint8_t accepted, uint32_t now) { uint8_t p[2]={0x06U,0U}; p[1]=id; Frame(0x05U,accepted?1U:0U,p,2U,now); }
/** @brief 构造独立字段值，检查单位、有效位及GGA质量码转换。 */
static void Pvt(uint8_t kind, uint8_t flags, uint8_t invalid_llh, uint32_t itow, uint32_t now)
{
  uint8_t p[92]; memset(p,0,sizeof(p));
  Gnss_Put32(p,itow); Gnss_Put16(p+4,2026U); p[6]=11U;p[7]=6U;p[8]=12U;p[9]=34U;p[10]=56U;p[11]=3U;
  p[20]=kind;p[21]=flags;p[23]=12U;Gnss_Put32(p+24,1135000000UL);Gnss_Put32(p+28,225000000UL);
  Gnss_Put32(p+36,42500U);Gnss_Put32(p+60,12340U);Gnss_Put32(p+64,9000000U);Gnss_Put16(p+76,9999U);p[78]=invalid_llh;
  Frame(0x01U,0x07U,p,sizeof(p),now);
}

/** @brief 构造无UTC的PVT状态测试向量，卫星原值与定位可用性分别指定。 */
static void PvtStatus(uint8_t kind, uint8_t flags, uint8_t satellites, uint32_t now)
{
  uint8_t p[92];memset(p,0,sizeof(p));p[20]=kind;p[21]=flags;p[23]=satellites;
  Frame(0x01U,0x07U,p,sizeof(p),now);
}

/** @brief 无定位PVT也提供真实0/非0卫星数；ACK/版本/其它句不续状态时间，非法类型撤有效位。 */
static void TestPvtIndependentNavigationStatus(void)
{
  Gnss_Snapshot_t s;
  Cold();PvtStatus(0U,0U,0U,0U);
  assert(Sensor_GNSS_CopySnapshot(&s)==GNSS_RESULT_NO_FIX);
  assert(s.valid_mask==GNSS_VALID_NAV_STATUS && s.status_update_ms==0U && s.satellites==0U);
  PvtStatus(0U,0U,2U,20U);(void)Sensor_GNSS_CopySnapshot(&s);
  assert(s.valid_mask==GNSS_VALID_NAV_STATUS && s.status_update_ms==20U && s.satellites==2U && !s.fix_valid);
  Ack(0x8AU,1U,30U);Version(3410U,40U);(void)Sensor_GNSS_CopySnapshot(&s);
  assert(s.last_rx_ms==40U && s.status_update_ms==20U && s.satellites==2U);
  Sentence("GPGGA,123519.00,,,,,0,00,99.9,,,,,,",50U);(void)Sensor_GNSS_CopySnapshot(&s);
  assert(s.status_update_ms==20U && s.satellites==2U); /* 近期PVT隔离NMEA测量。 */
  Sentence("GPGSV,1,1,08",3030U);Sentence("GPGSA,A,1,,,,,,,,,,,,,1.0,1.0,1.0",3040U);
  Sentence("GPRMC,123519.00,V,,,,,,,061126,,,N",3050U);(void)Sensor_GNSS_CopySnapshot(&s);
  assert(s.status_update_ms==20U && s.satellites==2U && (s.valid_mask&GNSS_VALID_NAV_STATUS));
  PvtStatus(255U,0U,22U,3060U);(void)Sensor_GNSS_CopySnapshot(&s);
  assert(!(s.valid_mask&GNSS_VALID_NAV_STATUS) && s.status_update_ms==20U);

  Cold();PvtStatus(0U,0U,9U,UINT32_MAX-9U);PvtStatus(0U,0U,3U,0U);
  (void)Sensor_GNSS_CopySnapshot(&s);
  assert((s.valid_mask&GNSS_VALID_NAV_STATUS) && s.status_update_ms==0U && s.satellites==3U);
}

/** @brief GGA状态独立校验，真实少卫星/缺坐标不变为假0，畸形整数不能借截断获得有效性。 */
static void TestGgaIndependentNavigationStatus(void)
{
  static const char * const bad_quality[]={"","-1","+1","1.0","9","1000"};
  static const char * const bad_satellites[]={"","-1","2.0","256","999","1234","00008x","0000256"};
  Gnss_Snapshot_t s;char body[128];unsigned j;
  Cold();Sentence("GPGGA,123519.00,,,,,0,00,99.9,,,,,,",0U);
  assert(Sensor_GNSS_CopySnapshot(&s)==GNSS_RESULT_NO_FIX && s.valid_mask==GNSS_VALID_NAV_STATUS);
  assert(s.status_update_ms==0U && s.satellites==0U);
  Sentence("GPGGA,123519.00,,,,,0,02,99.9,,,,,,",10U);(void)Sensor_GNSS_CopySnapshot(&s);
  assert(s.status_update_ms==10U && s.satellites==2U && !s.fix_valid);
  Sentence("GPGGA,123519.00,2230.0000,N,11330.0000,E,1,00002,1.0,25.0,M,0.0,M,,",20U);
  assert(Sensor_GNSS_CopySnapshot(&s)==GNSS_RESULT_NO_FIX && s.satellites==2U && s.status_update_ms==20U);
  Sentence("GPGGA,123519.00,,N,11330.0000,E,1,08,1.0,25.0,M,0.0,M,,",30U);
  assert(Sensor_GNSS_CopySnapshot(&s)==GNSS_RESULT_NO_FIX && s.satellites==8U && s.status_update_ms==30U);
  for(j=0U;j<sizeof(bad_quality)/sizeof(bad_quality[0]);++j) {
    (void)snprintf(body,sizeof(body),"GPGGA,123519.00,2230.0000,N,11330.0000,E,%s,08,1.0,25.0,M,0.0,M,,",bad_quality[j]);
    Sentence(body,40U+j);(void)Sensor_GNSS_CopySnapshot(&s);
    assert(!(s.valid_mask&GNSS_VALID_NAV_STATUS) && s.status_update_ms==30U && !s.fix_valid);
  }
  Sentence("GPGGA,123519.00,2230.0000,N,11330.0000,E,8,255,1.0,25.0,M,0.0,M,,",60U);
  /* 模拟质量码8保留报文状态和星数，但不能授权为真实GNSS位置。 */
  assert(Sensor_GNSS_CopySnapshot(&s)==GNSS_RESULT_NO_FIX && s.satellites==255U && s.status_update_ms==60U);
  assert((s.valid_mask&GNSS_VALID_NAV_STATUS)!=0U && (s.valid_mask&GNSS_VALID_POSITION)==0U && s.raw_status_type==8U);
  for(j=0U;j<sizeof(bad_satellites)/sizeof(bad_satellites[0]);++j) {
    (void)snprintf(body,sizeof(body),"GPGGA,123519.00,2230.0000,N,11330.0000,E,1,%s,1.0,25.0,M,0.0,M,,",bad_satellites[j]);
    Sentence(body,70U+j);(void)Sensor_GNSS_CopySnapshot(&s);
    assert(!(s.valid_mask&GNSS_VALID_NAV_STATUS) && s.status_update_ms==60U && !s.fix_valid);
  }
  Sentence("GPGGA,123519.00,,,,,0,02,99.9,,,,,,",100U);
  Sentence("GPRMC,123519.00,A,,,,,1.0,90.0,061126,,,A",110U);
  Sentence("GPRMC,123519.00,V,,,,,,,061126,,,N",120U);(void)Sensor_GNSS_CopySnapshot(&s);
  assert(s.status_update_ms==100U && s.satellites==2U && (s.valid_mask&GNSS_VALID_NAV_STATUS));
}

/** @brief 无数据、warm Init及毫秒回绕均保持115200，不为探测重启DMA。 */
static void TestFixedBaudAndWarmInit(void)
{
  Gnss_Diagnostics_t d;
  Cold(); assert(Sensor_GNSS_CopyDiagnostics(&d)==GNSS_RESULT_NO_DATA);
  Step(0U); d=Diag(); assert(d.measured_ms==0U && d.service_count==1U && d.rx_bytes==0U && !d.rx_seen);
  assert(commands_count==1U && sizes[0]==8U && commands[0][2]==0x0AU && commands[0][3]==0x04U);
  Sensor_GNSS_RequestReinit();Step(1000U);Step(3500U);d=Diag();
  assert(current_baud==115200U && d.baud_changes==0U && d.service_count==3U);
  Step(7000U);assert(current_baud==115200U && baud_set_calls==0U && actual_recoveries==0U);
  Cold();Step(UINT32_MAX-100U);Step(3400U);assert(current_baud==115200U && baud_set_calls==0U);
}

/** @brief 每2秒错误恢复及warm Init均不切换速率，不伪造接收或协议事实。 */
static void TestFixedBaudSurvivesRecovery(void)
{
  uint32_t now;
  Cold();Step(0U);
  for(now=10U;now<=21000U;now+=10U) {
    if(now%2000U==1000U) { initialized=0U;requested_recoveries=1U; }
    if(now==3600U) { Sensor_GNSS_RequestReinit(); }
    Step(now);
    assert(Diag().baud_changes==0U && current_baud==115200U && baud_set_calls==0U);
  }
  assert(actual_recoveries==11U && Diag().rx_bytes==0U && Diag().valid_nmea==0U && Diag().valid_ubx==0U);
}

/** @brief 高频错误受2秒恢复限速，成功或待恢复时均只保持固定速率。 */
static void TestFrequentErrorsKeepFixedBaud(void)
{
  uint32_t now;
  Gnss_Result_t result;
  Cold();Step(0U);
  for(now=100U;now<=12100U;now+=100U) {
    initialized=0U;requested_recoveries=1U;
    result=Sensor_GNSS_Service(now);
    assert(result==(((now-100U)%2000U==0U)?GNSS_RESULT_OK:GNSS_RESULT_IO_ERROR));
    assert(current_baud==115200U && baud_set_calls==0U);
  }
  assert(actual_recoveries==7U && Diag().baud_changes==0U);
}

/** @brief 恢复失败、0ms首次重试及毫秒回绕均不改变固定115200。 */
static void TestFixedBaudRecoveryFailureZeroAndWrap(void)
{
  Cold();initialized=0U;requested_recoveries=1U;Step(0U);
  Step(3499U);assert(Diag().baud_changes==0U);
  Step(3500U);assert(Diag().baud_changes==0U && current_baud==115200U && baud_set_calls==0U);

  Cold();Step(0U);initialized=0U;requested_recoveries=1U;io_fail=1U;
  assert(Sensor_GNSS_Service(0U)==GNSS_RESULT_IO_ERROR && actual_recoveries==1U);
  assert(Sensor_GNSS_Service(1999U)==GNSS_RESULT_IO_ERROR && actual_recoveries==1U);
  io_fail=0U;Step(2000U);assert(actual_recoveries==2U);
  Step(3499U);assert(Diag().baud_changes==0U);
  Step(3500U);assert(Diag().baud_changes==0U && current_baud==115200U && baud_set_calls==0U);

  Cold();initialized=0U;requested_recoveries=1U;Step(UINT32_MAX-999U);
  initialized=0U;requested_recoveries=1U;
  assert(Sensor_GNSS_Service(0U)==GNSS_RESULT_IO_ERROR && actual_recoveries==1U);
  Step(1000U);assert(actual_recoveries==2U);
  Step(2499U);assert(Diag().baud_changes==0U);
  Step(2500U);assert(Diag().baud_changes==0U && current_baud==115200U && baud_set_calls==0U);
}

/** @brief 三次无应答后退避10秒再查询；期间一直接收并支持模块后来输出有效数据。 */
static void TestNoDataProbeBackoffAndHotplug(void)
{
  uint32_t now;
  Gnss_Snapshot_t snapshot;
  Cold();Step(0U);Step(1100U);Step(2200U);assert(commands_count==3U);
  Sensor_GNSS_RequestReinit();Step(5000U);Step(12199U);assert(commands_count==3U);
  Step(12200U);assert(commands_count==4U);
  Step(13300U);Step(14400U);assert(commands_count==6U);
  for(now=14410U;now<=60000U;now+=10U) { Step(now); }
  assert(commands_count>6U && current_baud==115200U && baud_set_calls==0U && actual_recoveries>0U);
  Sentence("GPGGA,123519.00,2230.0000,N,11330.0000,E,1,08,1.0,25.0,M,0.0,M,,",60010U);
  assert(Diag().valid_nmea==1U && Diag().rx_bytes!=0U && Diag().baud_changes==0U);
  assert(Sensor_GNSS_CopySnapshot(&snapshot)==GNSS_RESULT_OK && snapshot.latitude_deg_e7==225000000);

  Cold();Step(UINT32_MAX-2199U);Step(UINT32_MAX-1099U);Step(0U);assert(commands_count==3U);
  Step(9999U);assert(commands_count==3U);
  Step(10000U);assert(commands_count==4U && current_baud==115200U && baud_set_calls==0U);
}

/** @brief 异常本机速率只纠正到115200，BUSY不提交协议命令，IO失败仍走owner恢复。 */
static void TestFixedBaudCorrectionBusyAndError(void)
{
  Cold();current_baud=9600U;busy=1U;Step(0U);Step(1000U);
  assert(current_baud==9600U && commands_count==0U && Diag().baud_changes==0U);
  busy=0U;Step(1010U);assert(current_baud==115200U && Diag().baud_changes==1U);
  Step(1020U);assert(commands_count==1U);

  Cold();current_baud=57600U;io_fail=1U;
  assert(Sensor_GNSS_Service(0U)==GNSS_RESULT_IO_ERROR && requested_recoveries);
  assert(Sensor_GNSS_Service(10U)==GNSS_RESULT_IO_ERROR && actual_recoveries==1U);
  io_fail=0U;Step(2010U);Step(2020U);
  assert(actual_recoveries==2U && current_baud==115200U && commands_count==1U);
}

static void TestNmeaPassiveAndBinaryIsolation(void)
{
  Gnss_Snapshot_t s; uint8_t p[100]; uint32_t before;
  Cold();Step(0U);
  Sentence("GPGGA,123519.00,2230.0000,N,11330.0000,E,1,08,1.0,25.0,M,0.0,M,,",10U);
  assert(Sensor_GNSS_CopySnapshot(&s)==GNSS_RESULT_OK && s.latitude_deg_e7==225000000);
  Sentence("GPGSA,A,1,,,,,,,,,,,,,1.0,1.0,1.0",20U);
  assert(Sensor_GNSS_CopySnapshot(&s)==GNSS_RESULT_OK && (s.valid_mask&GNSS_VALID_POSITION));
  Step(1210U);Step(2410U);Step(3610U);assert(Diag().state==GNSS_LINK_RECEIVING && !Diag().config_confirmed);
  before=Diag().valid_nmea;
  memset(p,0,sizeof(p));
  { static const char fake[]="$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n"; memcpy(p,fake,sizeof(fake)-1U); }
  Frame(0x99U,0x99U,p,sizeof(p),3620U);assert(Diag().valid_nmea==before);
}

/** @brief 已收到有效报文后错误恢复或断流均不切档，恢复数据仍能正常解析。 */
static void TestFixedRateAfterLockAndLoss(void)
{
  Cold();Step(0U);
  Sentence("GPGGA,123519.00,2230.0000,N,11330.0000,E,1,08,1.0,25.0,M,0.0,M,,",10U);
  Step(1210U);Step(2410U);Step(3610U);
  assert(Diag().state==GNSS_LINK_RECEIVING && current_baud==115200U);
  initialized=0U;requested_recoveries=1U;Step(50000U);
  assert(Diag().state==GNSS_LINK_SCAN && Diag().baud_changes==0U);
  initialized=0U;requested_recoveries=1U;Step(52000U);
  Step(53499U);assert(Diag().baud_changes==0U);
  Step(53500U);assert(Diag().baud_changes==0U && current_baud==115200U && baud_set_calls==0U);
  Sentence("GPGGA,123519.00,2230.0000,N,11330.0000,E,1,08,1.0,25.0,M,0.0,M,,",53510U);
  assert(Diag().valid_nmea==2U);

  Cold();Step(0U);
  Sentence("GPGGA,123519.00,2230.0000,N,11330.0000,E,1,08,1.0,25.0,M,0.0,M,,",10U);
  Step(6011U);assert(Diag().state==GNSS_LINK_SCAN && Diag().baud_changes==0U);
  Step(9510U);assert(Diag().baud_changes==0U);
  Step(9511U);assert(Diag().baud_changes==0U && current_baud==115200U && baud_set_calls==0U);
}

static void TestPvtAndDop(void)
{
  Gnss_Snapshot_t s; uint8_t p[18];
  Cold();Step(0U);Pvt(3U,1U,0U,1000U,10U);
  assert(Sensor_GNSS_CopySnapshot(&s)==GNSS_RESULT_OK);
  assert(s.latitude_deg_e7==225000000 && s.longitude_deg_e7==1135000000 && s.altitude_mm==42500);
  assert(s.speed_cms==1234U && s.heading_deg100==9000U && s.utc_date==261106U && s.utc_sec==45296U);
  assert(s.fix_quality==1U && s.fix_dimension==3U && s.hdop_cm==0U);
  memset(p,0,sizeof(p));Gnss_Put32(p,999U);Gnss_Put16(p+12,88U);Frame(1U,4U,p,18U,20U);
  (void)Sensor_GNSS_CopySnapshot(&s);assert(s.hdop_cm==0U);
  Gnss_Put32(p,1000U);Frame(1U,4U,p,18U,30U);(void)Sensor_GNSS_CopySnapshot(&s);assert(s.hdop_cm==88U);
  Pvt(2U,1U,0U,2000U,40U);(void)Sensor_GNSS_CopySnapshot(&s);
  assert(s.fix_quality==1U && s.fix_dimension==2U && !(s.valid_mask&GNSS_VALID_ALTITUDE));
  Version(3410U,45U);
  Pvt(3U,0x81U,0U,3000U,50U);(void)Sensor_GNSS_CopySnapshot(&s);assert(s.fix_quality==4U);
  Pvt(3U,1U,1U,4000U,60U);assert(Sensor_GNSS_CopySnapshot(&s)==GNSS_RESULT_NO_FIX && (s.valid_mask&GNSS_VALID_TIME));
  Pvt(3U,1U,0U,5000U,70U);
  Sentence("GPRMC,123519.00,V,,,,,,,061126,,,N",80U);
  assert(Sensor_GNSS_CopySnapshot(&s)==GNSS_RESULT_OK && s.position_update_ms==70U);
}

static void TestConfigurationConfirmed(void)
{
  uint32_t count; static const uint8_t pvt_key[5]={0x07U,0x00U,0x91U,0x20U,0x01U};
  static const uint8_t sat_key[5]={0x16U,0x00U,0x91U,0x20U,0x01U};
  Cold();Step(0U);Version(3410U,10U);
  assert(Diag().state==GNSS_LINK_CONFIGURE && Diag().config_step==0U && Diag().config_total==4U && !Diag().config_confirmed);
  assert(sizes[commands_count-1U]==17U && commands[commands_count-1U][3]==0x8AU && memcmp(commands[commands_count-1U]+10,pvt_key,5U)==0);
  Ack(1U,1U,20U);assert(Diag().config_step==0U);
  Ack(0x8AU,1U,30U);assert(Diag().config_step==1U && !Diag().config_confirmed);
  Ack(0x8AU,1U,40U);assert(Diag().config_step==1U && Diag().ack_count==1U);
  count=commands_count;Step(1200U);assert(commands_count==count);Step(1210U);assert(commands_count==count+1U);
  Ack(0x8AU,1U,1220U);Step(2410U);
  assert(sizes[commands_count-1U]==17U && !memcmp(commands[commands_count-1U]+10,sat_key,5U));
  Ack(0x8AU,1U,2420U);Step(3610U);
  assert(sizes[commands_count-1U]==29U && commands[commands_count-1U][14]==0xE8U && commands[commands_count-1U][15]==0x03U);
  Pvt(3U,1U,0U,900U,3615U);Ack(0x8AU,1U,3620U);Step(3625U);
  assert(Diag().ack_count==4U && Diag().config_step==4U && !Diag().config_confirmed);
  Pvt(3U,0U,0U,1000U,3630U);
  assert(Diag().config_confirmed && Diag().state==GNSS_LINK_RECEIVING);
  assert(s_snapshot.fix_valid==0U); /* 接入确认不等于定位成功。 */
}

static void TestNakTimeoutAndIoRecovery(void)
{
  uint32_t count;
  Cold();Step(0U);Version(3410U,10U);Ack(0x8AU,0U,20U);
  assert(Diag().nak_count==1U && Diag().state==GNSS_LINK_UNCONFIRMED && !Diag().config_confirmed);
  Cold();Step(0U);Version(3410U,10U);Step(1210U);Step(2410U);Step(3610U);
  assert(Diag().timeout_count==3U && Diag().state==GNSS_LINK_UNCONFIRMED && !Diag().config_confirmed);
  Cold();Step(0U);busy=1U;
  assert(Sensor_GNSS_Service(2000U)==GNSS_RESULT_OK && !requested_recoveries && !actual_recoveries);
  assert(aborted_tx_count==1U && !busy && Diag().state==GNSS_LINK_UNCONFIRMED);
  Step(2010U);assert(actual_recoveries==0U && !busy && aborted_tx_count==1U);
  Cold();Step(0U);busy=1U;abort_tx_fail=1U;
  assert(Sensor_GNSS_Service(2000U)==GNSS_RESULT_IO_ERROR && requested_recoveries && aborted_tx_count==1U);
  Cold();initialized=0U;assert(Sensor_GNSS_Service(0U)==GNSS_RESULT_IO_ERROR);
  io_fail=1U;assert(Sensor_GNSS_Service(10U)==GNSS_RESULT_IO_ERROR);count=actual_recoveries;
  assert(Sensor_GNSS_Service(100U)==GNSS_RESULT_IO_ERROR && actual_recoveries==count);
  io_fail=0U;Step(2010U);assert(actual_recoveries==count+1U);
}

static void TestLegacyConfigurationAndBudget(void)
{
  uint8_t bytes[600]; uint16_t n;
  Cold();Step(0U);Version(1800U,10U);
  assert(commands[commands_count-1U][3]==1U && sizes[commands_count-1U]==16U);
  assert(commands[commands_count-1U][6]==1U && commands[commands_count-1U][7]==7U && commands[commands_count-1U][9]==1U);
  Ack(1U,1U,30U);Step(1210U);Ack(1U,1U,1220U);Step(2410U);
  assert(commands[commands_count-1U][3]==1U && commands[commands_count-1U][7]==0x35U && commands[commands_count-1U][9]==1U);
  Ack(1U,1U,2420U);Step(3610U);
  assert(commands[commands_count-1U][3]==8U && commands[commands_count-1U][6]==0xE8U && commands[commands_count-1U][7]==0x03U);
  Cold();memset(bytes,'x',sizeof(bytes));Raw(bytes,sizeof(bytes));Step(0U);assert(Diag().rx_bytes==512U && input_size==88U);
  Step(10U);assert(Diag().rx_bytes==600U && input_size==0U);
  n=Ubx_BuildFrame(0x01U,0x04U,bytes,18U,bytes+100,100U);assert(n==26U);bytes[100+n-1U]^=1U;
  Raw(bytes+100,n);Step(20U);assert(Diag().checksum_errors==1U);
}

/** @brief 版本畸形不得触发写配置；旧84B的保留字段不得伪造RTK或撤销定位。 */
static void TestVersionGateAndLegacyPvt(void)
{
  uint8_t p[92]; uint32_t count; Gnss_Snapshot_t s;
  Cold();Step(0U);count=commands_count;
  memset(p,0,sizeof(p));memcpy(p,"SPG 5.10",8U);memcpy(p+30,"000A0000",8U);
  Frame(0x0AU,4U,p,39U,10U);Frame(0x0AU,4U,p,41U,20U);
  assert(!Diag().version_seen && commands_count==count);
  Frame(0x0AU,4U,p,40U,30U);assert(Diag().state==GNSS_LINK_UNCONFIRMED && !Diag().config_confirmed && commands_count==count);
  Cold();Step(0U);memcpy(p+40,"PROTVER=34.1x",13U);Frame(0x0AU,4U,p,70U,10U);
  assert(Diag().protocol_version_x100==0U && Diag().state==GNSS_LINK_UNCONFIRMED && commands_count==1U);
  Cold();Step(0U);memset(p,0,sizeof(p));Gnss_Put32(p,1000U);p[20]=3U;p[21]=0x81U;p[23]=8U;
  Gnss_Put32(p+24,1135000000UL);Gnss_Put32(p+28,225000000UL);p[78]=1U;
  Frame(1U,7U,p,84U,10U);assert(Sensor_GNSS_CopySnapshot(&s)==GNSS_RESULT_OK && s.fix_quality==1U);
  Cold();Step(0U);busy=1U;
  Sentence("GPGGA,123519.00,2230.0000,N,11330.0000,E,1,08,1.0,25.0,M,0.0,M,,",10U);
  assert(Sensor_GNSS_Service(2000U)==GNSS_RESULT_OK && !requested_recoveries && !actual_recoveries);
  assert(aborted_tx_count==1U && !busy && Diag().state==GNSS_LINK_UNCONFIRMED);
}

int main(void)
{
  TestPvtIndependentNavigationStatus();TestGgaIndependentNavigationStatus();
  TestFixedBaudAndWarmInit();TestFixedBaudSurvivesRecovery();TestFrequentErrorsKeepFixedBaud();
  TestFixedBaudRecoveryFailureZeroAndWrap();TestNoDataProbeBackoffAndHotplug();TestFixedBaudCorrectionBusyAndError();
  TestNmeaPassiveAndBinaryIsolation();TestFixedRateAfterLockAndLoss();TestPvtAndDop();
  TestConfigurationConfirmed();TestNakTimeoutAndIoRecovery();TestLegacyConfigurationAndBudget();TestVersionGateAndLegacyPvt();
  puts("GNSS bringup state, dual protocol and measurement tests passed");return 0;
}
