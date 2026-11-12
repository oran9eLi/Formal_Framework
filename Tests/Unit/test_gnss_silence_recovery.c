/**
 * @file test_gnss_silence_recovery.c
 * @brief 真实GNSS owner的原始字节静默自愈、限速、协议退避及恢复事实回归。
 * @details 使用独立BSP队列/故障注入，不伪造实板数据；不改变Health两秒离线策略。
 */
#define main ExistingBringupMain
#include "test_gnss_bringup.c"
#undef main

static const char s_silence_nofix[]="GPGGA,123519.00,,,,,0,00,99.9,,,,,,";

/** @brief 冷启动无字节可有限重建，原3次1100ms查询/10秒退避不被6秒恢复打断。 */
static void TestColdSilentLimitedRebuild(void)
{
  uint32_t now;Gnss_Snapshot_t snapshot;
  Cold();Step(0U);
  for(now=10U;now<=30000U;now+=10U) {
    Step(now);
    if(now==6000U) { assert(requested_recoveries==1U && actual_recoveries==0U); }
    if(now==6010U) { assert(!requested_recoveries && actual_recoveries==1U); }
  }
  assert(actual_recoveries==4U && commands_count==9U && current_baud==115200U && !baud_set_calls);
  assert(Diag().rx_bytes==0U && !Diag().rx_seen && !Diag().valid_nmea && !Diag().valid_ubx);
  assert(Sensor_GNSS_CopySnapshot(&snapshot)==GNSS_RESULT_NO_DATA && snapshot.rx_sequence==0U);
}

/** @brief 1Hz合法输入即使无定位也不重建；截止时刻刚到的新字节先消费，避免误请求。 */
static void TestOneHzNoFixAndBoundaryData(void)
{
  uint32_t now;Gnss_Snapshot_t snapshot;
  Cold();Step(0U);Sentence(s_silence_nofix,6000U);
  assert(!requested_recoveries && !actual_recoveries && !s_silence_recovery_pending);
  Cold();
  for(now=0U;now<=30000U;now+=10U) {
    if(now%1000U==0U) { Sentence(s_silence_nofix,now); }
    else { Step(now); }
  }
  assert(!actual_recoveries && !requested_recoveries && Diag().valid_nmea==31U);
  assert(Sensor_GNSS_CopySnapshot(&snapshot)==GNSS_RESULT_NO_FIX && snapshot.last_rx_ms==30000U);
}

/** @brief 原始字节有进展时不因CRC失败或不支持内容而触发硬件静默重建。 */
static void TestBadCrcAndRawGarbageStillProgress(void)
{
  uint8_t payload[92]={0U},wire[100],garbage=0x55U;uint32_t now;
  Cold();assert(Ubx_BuildFrame(1U,7U,payload,sizeof(payload),wire,sizeof(wire))==sizeof(wire));wire[99]^=1U;
  for(now=0U;now<=30000U;now+=10U) {
    if(now%1000U==0U) { Raw(wire,sizeof(wire)); }
    Step(now);
  }
  assert(!actual_recoveries && !requested_recoveries && Diag().checksum_errors==31U && !Diag().valid_ubx);
  Cold();
  for(now=0U;now<=30000U;now+=10U) { if(now%100U==0U) { Raw(&garbage,1U); }Step(now); }
  assert(!actual_recoveries && !requested_recoveries && Diag().rx_bytes==301U && !Diag().valid_nmea && !Diag().valid_ubx);
}

/** @brief Health反复请求warm解析复位不能一直把静默观察拨回起点。 */
static void TestWarmInitDoesNotDelaySilence(void)
{
  uint32_t now;
  Cold();Step(0U);
  for(now=10U;now<=20000U;now+=10U) {
    if(now%100U==0U) { Sensor_GNSS_RequestReinit(); }
    Step(now);
  }
  assert(actual_recoveries==3U && commands_count==6U && !Diag().valid_nmea);
}

/** @brief 曾收到数据后静默自愈不伪造新序号/接收时刻，重新输出才推进事实。 */
static void TestSilentThenNewData(void)
{
  Gnss_Snapshot_t before,after;
  Cold();Sentence(s_silence_nofix,100U);
  assert(Sensor_GNSS_CopySnapshot(&before)==GNSS_RESULT_NO_FIX);
  Step(6100U);assert(requested_recoveries && !actual_recoveries);
  Step(6110U);assert(actual_recoveries==1U);
  assert(Sensor_GNSS_CopySnapshot(&after)==GNSS_RESULT_NO_FIX);
  assert(after.rx_sequence==before.rx_sequence && after.last_rx_ms==100U);
  Sentence(s_silence_nofix,7000U);
  assert(Sensor_GNSS_CopySnapshot(&after)==GNSS_RESULT_NO_FIX);
  assert(after.rx_sequence==before.rx_sequence+1U && after.last_rx_ms==7000U && actual_recoveries==1U);
}

/** @brief 静默恢复失败后仍按2秒无限重试，真正新数据恢复后才推进有效RX。 */
static void TestFailedRecoveryBackoffAndRejoin(void)
{
  uint32_t now;Gnss_Snapshot_t snapshot;
  Cold();Step(0U);Step(6000U);assert(requested_recoveries && !actual_recoveries);
  io_fail=1U;assert(Sensor_GNSS_Service(6010U)==GNSS_RESULT_IO_ERROR && actual_recoveries==1U);
  for(now=6020U;now<=10010U;now+=10U) {
    if(now%100U==0U) { Sensor_GNSS_RequestReinit(); }
    assert(Sensor_GNSS_Service(now)==GNSS_RESULT_IO_ERROR);
    if(now<8010U) { assert(actual_recoveries==1U); }
    else if(now<10010U) { assert(actual_recoveries==2U); }
    else { assert(actual_recoveries==3U); }
  }
  assert(Sensor_GNSS_CopySnapshot(&snapshot)==GNSS_RESULT_NO_DATA);
  io_fail=0U;Step(12010U);assert(actual_recoveries==4U && !s_recover_rx_pending);
  Sentence(s_silence_nofix,12020U);
  assert(Sensor_GNSS_CopySnapshot(&snapshot)==GNSS_RESULT_NO_FIX && snapshot.last_rx_ms==12020U);
  for(now=13020U;now<=20020U;now+=1000U) { Sentence(s_silence_nofix,now); }
  assert(actual_recoveries==4U && !requested_recoveries && current_baud==115200U);
}

/** @brief 静默请求与真实硬件错误碰撞必须走原恢复身份；中断未完TX则撤销旧ACK确认。 */
static void TestRealFaultAndInterruptedTxIdentity(void)
{
  Cold();Step(0U);Step(6000U);assert(s_silence_recovery_pending);
  s_diag_work.version_seen=1U;s_diag_work.config_confirmed=1U;initialized=0U;
  Step(6010U);assert(actual_recoveries==1U && !Diag().version_seen && !Diag().config_confirmed);
  Cold();Step(0U);Step(6000U);busy=1U;s_link.awaiting_ack=1U;s_link.waiting_nav=1U;
  s_diag_work.config_confirmed=1U;Step(6010U);
  assert(actual_recoveries==1U && !busy && !s_link.awaiting_ack && !s_link.waiting_nav && !s_link.tx_watchdog_seen);
  assert(Diag().state==GNSS_LINK_UNCONFIRMED && !Diag().config_confirmed && s_link.stage_ms==6010U);
}

/** @brief 0ms起点及49.7天回绕均按绝对6秒间隔触发，下拍才执行。 */
static void TestZeroEpochAndWrap(void)
{
  Cold();Step(0U);Step(5999U);assert(!requested_recoveries);
  Step(6000U);assert(requested_recoveries && !actual_recoveries);Step(6010U);assert(actual_recoveries==1U);
  Cold();Step(UINT32_MAX-3000U);Step(2998U);assert(!requested_recoveries);
  Step(2999U);assert(requested_recoveries && !actual_recoveries);Step(3009U);assert(actual_recoveries==1U);
  Step(9008U);assert(!requested_recoveries);Step(9009U);assert(requested_recoveries && actual_recoveries==1U);
  Step(9019U);assert(actual_recoveries==2U && current_baud==115200U);
}

/** @brief 执行静默与恢复边界全部场景，不将主机通过视作实板链路稳定性通过。 */
int main(void)
{
  TestColdSilentLimitedRebuild();TestOneHzNoFixAndBoundaryData();TestBadCrcAndRawGarbageStillProgress();
  TestWarmInitDoesNotDelaySilence();TestSilentThenNewData();TestFailedRecoveryBackoffAndRejoin();TestRealFaultAndInterruptedTxIdentity();
  TestZeroEpochAndWrap();
  puts("GNSS silence recovery: raw-only6s / next-owner rebuild / 30s bounded / probe backoff / warm Init / 1Hz+CRC noise / 2s retry / rejoin / hardware identity / wrap PASS");
  return 0;
}
