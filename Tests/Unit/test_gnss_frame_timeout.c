/**
 * @file test_gnss_frame_timeout.c
 * @brief 真实GNSS/UBX解析器的绝对半帧期限、及时重同步与原始拒绝回归。
 * @details 仅使用主机BSP队列，验证真实500ms时间/回绕/有效帧隔离，不替代实板时序验收。
 */
#define main ExistingGnssBringupMain
#include "test_gnss_bringup.c"
#undef main

/** @brief 已观察的有依据RAW拒绝，不把超时片段构造成完整原帧。 */
static struct {
  uint32_t count;    /**< 回调次数。 */
  uint32_t bytes;    /**< 最近拒绝片段真实消费字节数，B。 */
  uint32_t time_ms;  /**< 最近拒绝时刻，真实ms。 */
  uint8_t protocol;  /**< 最近拒绝协议编号。 */
  uint8_t reason;    /**< 最近拒绝原因。 */
} s_timeout_reject;
static const char s_nofix_gga[]="GPGGA,123519.00,,,,,0,00,99.9,,,,,,";

/** @brief 只记录来源拒绝，不改变输入或调用恢复入口。 */
static void TimeoutReject(uint8_t protocol,uint32_t length,uint8_t reason,uint32_t now)
{
  s_timeout_reject.count++;s_timeout_reject.protocol=protocol;s_timeout_reject.bytes=length;
  s_timeout_reject.reason=reason;s_timeout_reject.time_ms=now;
}

/** @brief 为独立场景清理旧候选来源事件后模拟冷启动，0ms首字节合法。 */
static void TimeoutCold(void)
{
  Gnss_SetRawRejectCallback(NULL);Gnss_ResetFraming(0U,1U);Cold();
  memset(&s_timeout_reject,0,sizeof(s_timeout_reject));Gnss_SetRawRejectCallback(TimeoutReject);
}

/** @brief 读取原生快照，不触发新接收。 */
static Gnss_Snapshot_t TimeoutSnapshot(void)
{ Gnss_Snapshot_t snapshot;(void)Sensor_GNSS_CopySnapshot(&snapshot);return snapshot; }

/** @brief 只到绝对500ms边界才拒绝一次，不更新最后有效帧/测量时刻或重建UART。 */
static void TestPoisonHeaderBoundaryAndRecovery(void)
{
  const uint8_t header[]={0xB5U,0x62U,0x01U,0x35U,0xFFU,0xFFU};
  Gnss_Snapshot_t old,now;uint32_t resets;
  TimeoutCold();Sentence(s_nofix_gga,100U);old=TimeoutSnapshot();
  Raw(header,sizeof(header));Step(110U);resets=Diag().framing_resets;
  Step(609U);assert(Ubx_IsReceiving(&s_ubx) && !s_timeout_reject.count);
  Step(610U);assert(!Ubx_IsReceiving(&s_ubx) && s_timeout_reject.count==1U);
  assert(s_timeout_reject.protocol==GNSS_PROTOCOL_UBX && s_timeout_reject.bytes==sizeof(header));
  assert(s_timeout_reject.reason==GNSS_RAW_REJECT_FRAMING && s_timeout_reject.time_ms==610U);
  now=TimeoutSnapshot();assert(now.rx_sequence==old.rx_sequence && now.last_rx_ms==100U);
  assert(now.position_update_ms==old.position_update_ms && now.status_update_ms==old.status_update_ms);
  assert(Diag().framing_resets==resets+1U && !actual_recoveries && !requested_recoveries);
  Sentence(s_nofix_gga,610U);now=TimeoutSnapshot();assert(now.rx_sequence==old.rx_sequence+1U && now.last_rx_ms==610U);
  Step(1110U);assert(s_timeout_reject.count==1U && !s_ubx_candidate_timed);
}

/** @brief 所有头/载荷/两校验截断位置均覆盖，未完成数据不能续有效心跳。 */
static void TestEveryPartialStage(void)
{
  uint8_t payload[92]={0U},packet[100];
  static const uint16_t lengths[]={1U,2U,3U,4U,5U,6U,7U,36U,98U,99U};
  unsigned i;
  assert(Ubx_BuildFrame(1U,7U,payload,sizeof(payload),packet,sizeof(packet))==sizeof(packet));
  for(i=0U;i<sizeof(lengths)/sizeof(lengths[0]);i++) {
    TimeoutCold();Raw(packet,lengths[i]);Step(0U);assert(Ubx_IsReceiving(&s_ubx));
    Step(499U);assert(!s_timeout_reject.count);
    Step(500U);assert(s_timeout_reject.count==1U && s_timeout_reject.bytes==lengths[i]);
    assert(TimeoutSnapshot().rx_sequence==0U && !s_ubx_candidate_timed);
    Sentence(s_nofix_gga,501U);assert(TimeoutSnapshot().last_rx_ms==501U && Diag().valid_nmea==1U);
  }
}

/** @brief 后续字节不能延长寿命，合法0ms起点与49.7天回绕均按无符号差处理。 */
static void TestAbsoluteAgeAndWrap(void)
{
  const uint8_t header[]={0xB5U,0x62U,0x01U,0x35U,0xFFU,0xFFU};
  const uint8_t payload[]={0U,0U,0U};
  uint32_t start=UINT32_MAX-100U;
  TimeoutCold();Raw(header,sizeof(header));Step(0U);
  Raw(payload,sizeof(payload));Step(100U);Raw(payload,sizeof(payload));Step(200U);
  Raw(payload,sizeof(payload));Step(499U);assert(!s_timeout_reject.count && s_ubx_candidate_started_ms==0U);
  Step(500U);assert(s_timeout_reject.count==1U && s_timeout_reject.bytes==15U);
  TimeoutCold();Raw(header,sizeof(header));Step(start);
  Step(start+499U);assert(!s_timeout_reject.count && Ubx_IsReceiving(&s_ubx));
  Step(start+500U);assert(s_timeout_reject.count==1U && !Ubx_IsReceiving(&s_ubx));
  Sentence(s_nofix_gga,start+501U);assert(TimeoutSnapshot().last_rx_ms==start+501U);
}

/** @brief 最大1024B合法帧按115200满线约90ms分批接收，载荷内伪NMEA/UBX不得重同步。 */
static void TestMaximumWireRateAndEmbeddedFrames(void)
{
  uint8_t payload[UBX_PAYLOAD_CAPACITY],wire[GNSS_RAW_FORWARD_MAX_BYTES];
  uint8_t fake_ubx[8];uint16_t at=0U,full;uint32_t time=0U;
  const char embedded[]="$GPGGA,123519.00,,,,,0,00,99.9,,,,,,*52\r\n";
  TimeoutCold();memset(payload,0xA7,sizeof(payload));
  memcpy(payload+20U,embedded,sizeof(embedded)-1U);
  assert(Ubx_BuildFrame(0x99U,0x77U,NULL,0U,fake_ubx,sizeof(fake_ubx))==sizeof(fake_ubx));
  memcpy(payload+150U,fake_ubx,sizeof(fake_ubx));
  full=Ubx_BuildFrame(0x99U,0x88U,payload,sizeof(payload),wire,sizeof(wire));assert(full==1024U);
  while(at<full) {
    uint16_t n=(uint16_t)(full-at);if(n>115U) { n=115U; }
    Raw(wire+at,n);Step(time);at=(uint16_t)(at+n);time+=10U;
  }
  assert(Diag().valid_ubx==1U && Diag().valid_nmea==0U && !s_timeout_reject.count);
  assert(!Ubx_IsReceiving(&s_ubx) && !s_ubx_candidate_timed && !actual_recoveries);
  Step(1000U);assert(!s_timeout_reject.count);
}

/** @brief 原完整超容量帧仍原样隔离到校验后拒绝，不在载荷内扫描伪报文。 */
static void TestCompleteOversizeStillIsolated(void)
{
  uint8_t payload[1040],wire[1048];uint16_t at=0U;uint32_t time=0U;
  TimeoutCold();memset(payload,0x4AU,sizeof(payload));
  assert(Ubx_BuildFrame(0x99U,0x88U,payload,sizeof(payload),wire,sizeof(wire))==sizeof(wire));
  while(at<sizeof(wire)) {
    uint16_t n=(uint16_t)(sizeof(wire)-at);if(n>115U) { n=115U; }
    Raw(wire+at,n);Step(time);at=(uint16_t)(at+n);time+=10U;
  }
  assert(s_timeout_reject.count==1U && s_timeout_reject.reason==GNSS_RAW_REJECT_OVERSIZE);
  assert(s_timeout_reject.bytes==sizeof(wire) && !Ubx_IsReceiving(&s_ubx));
  Sentence(s_nofix_gga,time);assert(Diag().valid_nmea==1U && s_timeout_reject.count==1U);
}

/** @brief 500ms之前完整帧成功，坏CK_B保留B5时下一前缀获得独立新寿命。 */
static void TestCompleteAndCrcPrefixLifetime(void)
{
  uint8_t payload[92]={0U},wire[100];
  TimeoutCold();assert(Ubx_BuildFrame(1U,7U,payload,sizeof(payload),wire,sizeof(wire))==sizeof(wire));
  Raw(wire,99U);Step(0U);Raw(wire+99U,1U);Step(499U);
  assert(Diag().valid_ubx==1U && !s_ubx_candidate_timed && !s_timeout_reject.count);
  Step(999U);assert(!s_timeout_reject.count);
  TimeoutCold();Raw(wire,99U);Step(0U);wire[99U]=0xB5U;Raw(wire+99U,1U);Step(400U);
  assert(Diag().checksum_errors==1U && Ubx_IsReceiving(&s_ubx) && s_ubx_candidate_started_ms==400U);
  Step(899U);assert(!s_timeout_reject.count);
  Step(900U);assert(s_timeout_reject.count==1U && s_timeout_reject.bytes==1U);
}

/** @brief warm Init及硬件恢复撤销计时，来源事件只报告一次，不把旧时标带入新帧。 */
static void TestLifecycleResetAndBackToBack(void)
{
  uint8_t payload[92]={0U},wire[100];uint32_t resets;
  TimeoutCold();Raw((const uint8_t *)"\xB5\x62",2U);Step(100U);
  assert(Sensor_GNSS_Init()==GNSS_RESULT_OK && !s_ubx_candidate_timed);
  Step(700U);assert(s_timeout_reject.count==1U && s_timeout_reject.time_ms==700U);
  Step(800U);assert(s_timeout_reject.count==1U);
  Raw((const uint8_t *)"\xB5\x62",2U);Step(900U);requested_recoveries=1U;Step(910U);
  assert(!s_ubx_candidate_timed && actual_recoveries==1U && s_timeout_reject.count==2U);
  TimeoutCold();assert(Ubx_BuildFrame(1U,7U,payload,sizeof(payload),wire,sizeof(wire))==sizeof(wire));
  Raw(wire,sizeof(wire));Raw(wire,sizeof(wire));Raw(wire,sizeof(wire));Step(10U);
  assert(Diag().valid_ubx==3U && TimeoutSnapshot().rx_sequence==3U && !s_ubx_candidate_timed);
  resets=Diag().framing_resets;Step(1000U);
  assert(!s_timeout_reject.count && Diag().framing_resets==resets && !actual_recoveries);
}

/** @brief 运行半帧期限与恢复全部场景，不将通过结果描述成线路故障已排除。 */
int main(void)
{
  TestPoisonHeaderBoundaryAndRecovery();TestEveryPartialStage();TestAbsoluteAgeAndWrap();
  TestMaximumWireRateAndEmbeddedFrames();TestCompleteOversizeStillIsolated();
  TestCompleteAndCrcPrefixLifetime();TestLifecycleResetAndBackToBack();
  puts("GNSS UBX lifetime: absolute500ms / malformed length recovery / 1024B wire-rate / binary isolation / CRC / raw reject / wrap / lifecycle PASS");
  return 0;
}
