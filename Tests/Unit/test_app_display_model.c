/**
 * @file test_app_display_model.c
 * @brief 验证统一显示取数接口的模式选源、远端归一、新鲜度口径与本机白名单。
 *
 * 用 stub 替换 app_data_api 的取数函数，隔离测试 app_display_model 的解析逻辑。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_display_model.h"

/* ---- 受测桩状态 ---- */
static Px4Lite_RemoteMode_t s_mode;
static Px4Lite_RemoteTelemetrySnapshot_t s_remote;
static Px4Lite_Result_t s_remote_result;
static App_NavigationSnapshot_t s_local_nav;
static Px4Lite_Result_t s_local_nav_result;
static Px4Lite_CommDebugInfo_t s_comm;
static Px4Lite_ModuleStatus_t s_lora_status;
static Px4Lite_Result_t s_lora_status_result;

/* ---- app_data_api 桩 ---- */
Px4Lite_RemoteMode_t App_GetRemoteDisplayMode(void) { return s_mode; }

Px4Lite_Result_t App_CopyRemoteTelemetry(Px4Lite_RemoteTelemetrySnapshot_t *out, uint32_t now_ms)
{
  (void)now_ms;
  *out = s_remote;
  return s_remote_result;
}

Px4Lite_Result_t App_CopyNavigation(App_NavigationSnapshot_t *out, uint32_t now_ms)
{
  (void)now_ms;
  *out = s_local_nav;
  return s_local_nav_result;
}

Px4Lite_Result_t App_CopyEnvironment(App_EnvironmentSnapshot_t *out, uint32_t now_ms) { (void)now_ms; memset(out, 0, sizeof(*out)); return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopySystem(App_SystemSnapshot_t *out, uint32_t now_ms) { (void)now_ms; memset(out, 0, sizeof(*out)); return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopyAlarm(App_AlarmSnapshot_t *out, uint32_t now_ms) { (void)now_ms; memset(out, 0, sizeof(*out)); return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopyMotor(App_MotorSnapshot_t *out, uint32_t now_ms) { (void)now_ms; memset(out, 0, sizeof(*out)); return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopyDateTime(App_DateTimeSnapshot_t *out, uint32_t now_ms) { (void)now_ms; memset(out, 0, sizeof(*out)); return PX4LITE_NOT_READY; }

void App_GetCommStats(Px4Lite_CommDebugInfo_t *out) { *out = s_comm; }

Px4Lite_Result_t App_GetModuleStatus(Px4Lite_ModuleId_t module_id, Px4Lite_ModuleStatus_t *out)
{
  (void)module_id;
  *out = s_lora_status;
  return s_lora_status_result;
}

/* ---- 断言辅助 ---- */
static int ExpectU32(const char *name, uint32_t actual, uint32_t expected)
{
  if (actual != expected) {
    printf("FAIL %s actual=%lu expected=%lu\n", name, (unsigned long)actual, (unsigned long)expected);
    return 1;
  }
  return 0;
}

static int ExpectI32(const char *name, int32_t actual, int32_t expected)
{
  if (actual != expected) {
    printf("FAIL %s actual=%ld expected=%ld\n", name, (long)actual, (long)expected);
    return 1;
  }
  return 0;
}

static void ResetState(void)
{
  s_mode = PX4LITE_REMOTE_MODE_LOCAL;
  memset(&s_remote, 0, sizeof(s_remote));
  s_remote_result = PX4LITE_NOT_READY;
  memset(&s_local_nav, 0, sizeof(s_local_nav));
  s_local_nav_result = PX4LITE_NOT_READY;
  memset(&s_comm, 0, sizeof(s_comm));
  memset(&s_lora_status, 0, sizeof(s_lora_status));
  s_lora_status_result = PX4LITE_NOT_READY;
}

static int TestLocalModeUsesLocalSource(void)
{
  App_NavigationSnapshot_t nav;
  int failures = 0;

  ResetState();
  s_mode = PX4LITE_REMOTE_MODE_LOCAL;
  s_local_nav.latitude_e7 = 111111;
  s_local_nav_result      = PX4LITE_OK;

  failures += ExpectU32("local nav result", App_GetDisplayNavigation(&nav, 1000U), PX4LITE_OK);
  failures += ExpectI32("local nav routed to local", nav.latitude_e7, 111111);
  return failures;
}

static int TestRemoteModeNormalizesFreshRemote(void)
{
  App_NavigationSnapshot_t nav;
  int failures = 0;

  ResetState();
  s_mode                  = PX4LITE_REMOTE_MODE_REMOTE;
  s_remote_result         = PX4LITE_OK;
  s_remote.valid_mask     = PX4LITE_REMOTE_VALID_NAVIGATION | PX4LITE_REMOTE_VALID_ATTITUDE;
  s_remote.stale_mask     = 0U;
  s_remote.latitude_e7    = 222222;
  s_remote.yaw_deg100     = 9000;
  /* 本机源给不同值，确保不是误取本机。 */
  s_local_nav.latitude_e7 = 999999;
  s_local_nav_result      = PX4LITE_OK;

  failures += ExpectU32("remote fresh result", App_GetDisplayNavigation(&nav, 2000U), PX4LITE_OK);
  failures += ExpectI32("remote nav routed to remote", nav.latitude_e7, 222222);
  failures += ExpectI32("remote yaw normalized", nav.yaw_deg100, 9000);
  failures += ExpectU32("remote nav valid bit set", (nav.valid_mask & PX4LITE_NAV_VALID_POSITION) != 0U, 1U);
  failures += ExpectU32("remote att valid bit set", (nav.valid_mask & PX4LITE_NAV_VALID_ATTITUDE) != 0U, 1U);
  return failures;
}

static int TestRemoteStaleKeepsValueAndReportsStale(void)
{
  App_NavigationSnapshot_t nav;
  int failures = 0;

  ResetState();
  s_mode               = PX4LITE_REMOTE_MODE_REMOTE;
  s_remote_result      = PX4LITE_OK;
  s_remote.valid_mask  = PX4LITE_REMOTE_VALID_NAVIGATION;
  s_remote.stale_mask  = PX4LITE_REMOTE_VALID_NAVIGATION;
  s_remote.latitude_e7 = 333333;

  failures += ExpectU32("remote stale result", App_GetDisplayNavigation(&nav, 3000U), PX4LITE_STALE);
  failures += ExpectI32("remote stale keeps last value", nav.latitude_e7, 333333);
  return failures;
}

static int TestRemoteNeverReceivedIsNotReady(void)
{
  App_NavigationSnapshot_t nav;
  int failures = 0;

  ResetState();
  s_mode              = PX4LITE_REMOTE_MODE_REMOTE;
  s_remote_result     = PX4LITE_OK;
  s_remote.valid_mask = PX4LITE_REMOTE_VALID_ENVIRONMENT; /* 有遥测但无导航/姿态 */

  failures += ExpectU32("remote nav never received", App_GetDisplayNavigation(&nav, 4000U), PX4LITE_NOT_READY);
  failures += ExpectI32("not-ready clears output", nav.latitude_e7, 0);
  return failures;
}

static int TestLinkStatusStaysLocalInRemote(void)
{
  App_DisplayLinkStatus_t link;
  int failures = 0;

  ResetState();
  s_mode                   = PX4LITE_REMOTE_MODE_REMOTE;
  s_comm.tx_frame_count    = 111U;
  s_comm.rx_frame_count    = 222U;
  s_comm.rx_loss_permille  = 33U;
  s_lora_status.state      = PX4LITE_STATE_ONLINE;
  s_lora_status_result     = PX4LITE_OK;

  App_GetDisplayLinkStatus(&link, 5000U);
  failures += ExpectU32("link tx local", link.tx_frame_count, 111U);
  failures += ExpectU32("link rx local", link.rx_frame_count, 222U);
  failures += ExpectU32("link loss local", link.loss_permille, 33U);
  failures += ExpectU32("link lora state local", link.lora_state, (uint32_t)PX4LITE_STATE_ONLINE);
  return failures;
}

static int TestMessageLogNotReadyInPhaseA(void)
{
  App_DisplayLogSnapshot_t log;
  int failures = 0;

  ResetState();
  s_mode = PX4LITE_REMOTE_MODE_REMOTE;
  failures += ExpectU32("message log phase A not ready", App_GetDisplayMessageLog(&log, 6000U), PX4LITE_NOT_READY);
  failures += ExpectU32("message log count zero", log.count, 0U);
  return failures;
}

int main(void)
{
  int failures = 0;

  failures += TestLocalModeUsesLocalSource();
  failures += TestRemoteModeNormalizesFreshRemote();
  failures += TestRemoteStaleKeepsValueAndReportsStale();
  failures += TestRemoteNeverReceivedIsNotReady();
  failures += TestLinkStatusStaysLocalInRemote();
  failures += TestMessageLogNotReadyInPhaseA();

  if (failures != 0) {
    printf("app display model tests failed: %d\n", failures);
    return 1;
  }
  printf("app display model tests passed\n");
  return 0;
}
