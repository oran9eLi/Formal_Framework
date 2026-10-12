/** @file test_remote_slots.c
 * @brief 第九节点在临界区入口抢占时仍不串号，覆盖计时回绕淘汰。
 */
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
static void Enter(void);
static void Exit(void);
#define taskENTER_CRITICAL() Enter()
#define taskEXIT_CRITICAL() Exit()
#include "../../Framework/Src/px4lite_remote_telemetry.c"
static unsigned depth;
static uint8_t inject;
static uint8_t inject_exit;
static uint32_t clock_ms;
uint8_t Px4Lite_IdentityGetNodeId(void) { return 250U; }
uint32_t Px4Lite_PlatformGetMs(void) { return clock_ms; }
static void Enter(void)
{
  if (depth == 0U && inject != 0U) {
    Px4Lite_RemoteTelemetry_t next;
    inject = 0U; memset(&next, 0, sizeof(next));
    next.system_id = 9U; next.last_rx_ms = clock_ms;
    assert(Px4Lite_RemoteTelemetryCommitNode(9U, &next, clock_ms) == PX4LITE_OK);
  }
  ++depth;
}
static void Exit(void)
{
  assert(depth != 0U); --depth;
  if (depth == 0U && inject_exit != 0U) {
    inject_exit = 0U; inject = 1U; Enter(); Exit();
  }
}
int main(void)
{
  Px4Lite_RemoteTelemetry_t next, result;
  Px4Lite_Result_t rc; unsigned i;
  Px4Lite_RemoteNodeStatus_t statuses[8]; uint8_t count;
  clock_ms = 100U; Px4Lite_RemoteTelemetryInit(clock_ms);
  for (i = 1U; i <= 8U; ++i) {
    memset(&next, 0, sizeof(next)); next.system_id = (uint8_t)i; next.last_rx_ms = i;
    assert(Px4Lite_RemoteTelemetryCommitNode((uint8_t)i, &next, clock_ms) == PX4LITE_OK);
  }
  inject = 1U;
  rc = Px4Lite_RemoteTelemetryCopyNode(1U, &result);
  assert(rc == PX4LITE_NOT_READY || result.system_id == 1U);
  Px4Lite_RemoteTelemetryInit(clock_ms);
  for(i=1U;i<=8U;++i) {
    memset(&next,0,sizeof(next)); next.system_id=(uint8_t)i; next.last_rx_ms=i;
    assert(Px4Lite_RemoteTelemetryCommitNode((uint8_t)i,&next,clock_ms)==PX4LITE_OK);
  }
  inject_exit=1U;
  assert(Px4Lite_CopyRemoteNodeStatuses(statuses,8U,&count,clock_ms)==PX4LITE_OK && count==8U);
  for(i=0U;i<count;++i) { assert(statuses[i].node_id==statuses[i].system_id); }
  Px4Lite_RemoteTelemetryInit(clock_ms);
  for(i=1U;i<=8U;++i) {
    memset(&next,0,sizeof(next)); next.system_id=(uint8_t)i;
    next.last_rx_ms=(i==1U)?(UINT32_MAX-10U):i;
    assert(Px4Lite_RemoteTelemetryCommitNode((uint8_t)i,&next,clock_ms)==PX4LITE_OK);
  }
  next.system_id=9U; next.last_rx_ms=clock_ms;
  assert(Px4Lite_RemoteTelemetryCommitNode(9U,&next,clock_ms)==PX4LITE_OK);
  assert(Px4Lite_RemoteTelemetryCopyNode(1U,&result)==PX4LITE_NOT_READY);
  for(i=2U;i<=9U;++i) { assert(Px4Lite_RemoteTelemetryCopyNode((uint8_t)i,&result)==PX4LITE_OK && result.system_id==i); }
  next.system_id=9U; next.last_rx_ms=0U; next.valid_mask=PX4LITE_REMOTE_VALID_NAVIGATION;
  next.navigation_valid_mask=PX4LITE_NAV_VALID_POSITION; next.position_update_ms=0U;
  assert(Px4Lite_RemoteTelemetryCommitNode(9U,&next,0U)==PX4LITE_OK);
  assert(Px4Lite_SelectRemoteNode(9U)==PX4LITE_OK);
  clock_ms=PX4LITE_REMOTE_DATA_STALE_MS+1U;
  assert(Px4Lite_CopyRemoteTelemetry(&result)==PX4LITE_OK);
  assert((result.navigation_stale_mask&PX4LITE_NAV_VALID_POSITION)!=0U);
  puts("remote slot tests passed"); return 0;
}
