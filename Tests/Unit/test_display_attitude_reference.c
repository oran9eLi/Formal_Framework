/** @file test_display_attitude_reference.c
 * @brief 真实显示选源接口只对本机观察值扣除基准，源快照保持不变。
 */
#include <assert.h>
#include <stdio.h>
#include "../../Business/Src/app_display_model.c"
static App_NavigationSnapshot_t source;
static App_AttitudeStatus_t status;
static Px4Lite_RemoteMode_t mode;
Px4Lite_RemoteMode_t App_GetRemoteDisplayMode(void) { return mode; }
Px4Lite_Result_t App_CopyNavigation(App_NavigationSnapshot_t *out, uint32_t now_ms)
{ (void)now_ms; *out = source; return PX4LITE_OK; }
Px4Lite_Result_t App_CopyAttitudeStatus(App_AttitudeStatus_t *out) { *out = status; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_CopyRemoteTelemetry(Px4Lite_RemoteTelemetry_t *out)
{
  memset(out, 0, sizeof(*out)); out->header.valid = 1U; out->header.sample_time_ms = 1000U;
  out->valid_mask = PX4LITE_REMOTE_VALID_ATTITUDE; out->roll_deg100 = 5000;
  return PX4LITE_OK;
}
int main(void)
{
  App_NavigationSnapshot_t result;
  mode = PX4LITE_REMOTE_MODE_LOCAL;
  memset(&source, 0, sizeof(source)); source.roll_deg100 = 3000; source.yaw_deg100 = -17900;
  source.valid_mask = PX4LITE_NAV_VALID_ATTITUDE;
  status.reference_active = 1U; status.offset_deg100[0] = 3000; status.offset_deg100[2] = 17900;
  assert(App_GetDisplayNavigation(&result, 1000U) == PX4LITE_OK);
  assert(result.roll_deg100 == 0 && result.yaw_deg100 == 200);
  assert(source.roll_deg100 == 3000 && source.yaw_deg100 == -17900);
  source.roll_deg100 = -17900; status.offset_deg100[0] = 17900;
  assert(App_GetDisplayNavigation(&result, 1000U) == PX4LITE_OK && result.roll_deg100 == 200);
  source.roll_deg100 = 17900; status.offset_deg100[0] = -17900;
  assert(App_GetDisplayNavigation(&result, 1000U) == PX4LITE_OK && result.roll_deg100 == -200);
  source.valid_mask = PX4LITE_NAV_VALID_POSITION;
  source.roll_deg100 = 0; source.yaw_deg100 = 0;
  assert(App_GetDisplayNavigation(&result, 1000U) == PX4LITE_OK);
  assert(result.roll_deg100 == 0 && result.yaw_deg100 == 0);
  source.valid_mask = PX4LITE_NAV_VALID_ATTITUDE; source.roll_deg100 = 3000;
  status.reference_active = 0U;
  assert(App_GetDisplayNavigation(&result, 1000U) == PX4LITE_OK && result.roll_deg100 == 3000);
  status.reference_active = 1U; mode = PX4LITE_REMOTE_MODE_REMOTE;
  assert(App_GetDisplayNavigation(&result, 1000U) == PX4LITE_OK && result.roll_deg100 == 5000);
  puts("display attitude reference tests passed"); return 0;
}
