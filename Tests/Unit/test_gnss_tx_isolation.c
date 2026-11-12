/**
 * @file test_gnss_tx_isolation.c
 * @brief 真实GNSS状态机的TX故障独立恢复、半帧保留和异常退避回归。
 * @details 复用独立BSP队列，不伪造硬件在线；真正RX故障仍请求原恢复。
 */
#define main ExistingBringupMain
#include "test_gnss_bringup.c"
#undef main

/** @brief TX卡住同轮仍可接受NoFix PVT，既不丢有效接收事实也不重建RX。 */
static void TestValidRxDuringTxTimeout(void)
{
  uint8_t payload[92]={0U},wire[100];Gnss_Snapshot_t snapshot;
  Cold();Step(0U);busy=1U;
  assert(Ubx_BuildFrame(1U,7U,payload,sizeof(payload),wire,sizeof(wire))==sizeof(wire));
  Raw(wire,sizeof(wire));Step(2000U);
  assert(Sensor_GNSS_CopySnapshot(&snapshot)==GNSS_RESULT_NO_FIX);
  assert(snapshot.rx_sequence==1U && snapshot.last_rx_ms==2000U);
  assert(aborted_tx_count==1U && !busy && !requested_recoveries && !actual_recoveries);
  assert(Diag().state==GNSS_LINK_UNCONFIRMED && Diag().timeout_count==1U);
  Step(2010U);assert(aborted_tx_count==1U && Diag().timeout_count==1U);
}

/** @brief 发送超时不能丢弃正在接收的合法UBX半帧或重置其500ms起点。 */
static void TestInFlightRxPreserved(void)
{
  uint8_t payload[92]={0U},wire[100];uint32_t resets,start;
  Cold();Step(0U);busy=1U;
  assert(Ubx_BuildFrame(1U,7U,payload,sizeof(payload),wire,sizeof(wire))==sizeof(wire));
  Raw(wire,30U);Step(1990U);assert(Ubx_IsReceiving(&s_ubx));
  resets=Diag().framing_resets;start=s_ubx_candidate_started_ms;
  Step(2000U);assert(Ubx_IsReceiving(&s_ubx) && s_ubx_candidate_started_ms==start);
  assert(Diag().framing_resets==resets && !requested_recoveries && !actual_recoveries);
  Raw(wire+30U,70U);Step(2010U);
  assert(Diag().valid_ubx==1U && !Ubx_IsReceiving(&s_ubx));
}

/** @brief 失败提交的10秒退避不能被6秒无帧探测绕过，RX仍可在期间继续工作。 */
static void TestSubmissionFailureBackoff(void)
{
  Gnss_Snapshot_t snapshot;
  uint32_t now;static const uint8_t noise=0x55U;
  Cold();io_fail=1U;Step(0U);
  assert(aborted_tx_count==1U && Diag().state==GNSS_LINK_UNCONFIRMED && !requested_recoveries);
  Sentence("GPGGA,123519.00,,,,,0,00,99.9,,,,,,",1000U);
  assert(Sensor_GNSS_CopySnapshot(&snapshot)==GNSS_RESULT_NO_FIX && snapshot.last_rx_ms==1000U);
  /* 保持原始RX活动而不给有效帧，单独验证TX退避，不混入6秒全静默重建。 */
  for(now=2000U;now<=9000U;now+=1000U) { Raw(&noise,1U);Step(now); }
  Step(9999U);assert(aborted_tx_count==1U && !requested_recoveries);
  Step(10000U);assert(aborted_tx_count==2U && !requested_recoveries);
  Step(10010U);
  for(now=11000U;now<=19000U;now+=1000U) { Raw(&noise,1U);Step(now); }
  Step(19999U);assert(aborted_tx_count==2U);
  io_fail=0U;Step(20000U);
  assert(commands_count==1U && !actual_recoveries && current_baud==115200U);
}

/** @brief 撤销失败保持IO_ERROR，已收有效帧不被抹掉供Adapter独立复制。 */
static void TestAbortFailureAndStaleAck(void)
{
  uint8_t payload[92]={0U},wire[100];Gnss_Snapshot_t snapshot;
  Cold();Step(0U);busy=1U;abort_tx_fail=1U;
  assert(Ubx_BuildFrame(1U,7U,payload,sizeof(payload),wire,sizeof(wire))==sizeof(wire));
  Raw(wire,sizeof(wire));assert(Sensor_GNSS_Service(2000U)==GNSS_RESULT_IO_ERROR);
  assert(Sensor_GNSS_CopySnapshot(&snapshot)==GNSS_RESULT_NO_FIX && snapshot.last_rx_ms==2000U);
  assert(requested_recoveries && Diag().state==GNSS_LINK_IO_ERROR);
  Cold();Step(0U);Version(3410U,10U);busy=1U;Step(20U);
  assert(s_link.awaiting_ack);
  Step(2020U);assert(!s_link.awaiting_ack && !s_link.waiting_nav && !Diag().config_confirmed);
  Ack(0x8AU,1U,2030U);
  assert(Diag().state==GNSS_LINK_UNCONFIRMED && Diag().config_step==0U && !Diag().config_confirmed);
}

/** @brief 执行TX隔离场景，不延长Health门限或制造测量有效位。 */
int main(void)
{
  TestValidRxDuringTxTimeout();TestInFlightRxPreserved();TestSubmissionFailureBackoff();TestAbortFailureAndStaleAck();
  puts("GNSS TX isolation: valid RX / in-flight frame / 10s backoff / abort failure / stale ACK PASS");
  return 0;
}
