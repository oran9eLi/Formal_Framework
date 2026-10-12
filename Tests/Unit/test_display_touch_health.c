/** @file test_display_touch_health.c @brief 触摸降级不等同显示主体停更，恢复后清除故障。 */
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
typedef int BaseType_t;
typedef uint32_t TickType_t;
#include "../../Framework/Src/px4lite_modules.c"
#include "../../Business/Src/business_platform_adapter.c"
static Display_RuntimeStatus_t display_runtime;
void Display_GetRuntimeStatus(Display_RuntimeStatus_t *out) { *out=display_runtime; }
int main(void)
{
  Px4Lite_ModuleStatus_t status;
  Px4Lite_SetExternalModuleState(PX4LITE_MODULE_DISPLAY, PX4LITE_STATE_ONLINE, 0U, 100U);
  Px4Lite_SetExternalModuleState(PX4LITE_MODULE_DISPLAY, PX4LITE_STATE_DEGRADED, PX4LITE_FAULT_DISPLAY_TOUCH, 5000U);
  assert(Px4Lite_GetModuleStatus(PX4LITE_MODULE_DISPLAY,&status)==PX4LITE_OK);
  assert(status.state==PX4LITE_STATE_DEGRADED && status.fault_code==PX4LITE_FAULT_DISPLAY_TOUCH);
  assert(status.last_valid_ms==5000U); /* LCD仍在工作，不能按100ms旧心跳请求整屏恢复。 */
  Px4Lite_SetExternalModuleState(PX4LITE_MODULE_DISPLAY, PX4LITE_STATE_DEGRADED, PX4LITE_FAULT_DISPLAY_REFRESH, 5100U);
  assert(Px4Lite_GetModuleStatus(PX4LITE_MODULE_DISPLAY,&status)==PX4LITE_OK && status.last_valid_ms==5000U);
  Px4Lite_SetExternalModuleState(PX4LITE_MODULE_DISPLAY, PX4LITE_STATE_ONLINE, 0U, 5200U);
  assert(Px4Lite_GetModuleStatus(PX4LITE_MODULE_DISPLAY,&status)==PX4LITE_OK);
  assert(status.fault_code==0U && status.last_valid_ms==5200U && status.recovery_count==1U);
  display_runtime.display_ready=1U;
  Business_DisplayReportResult(BUSINESS_SERVICE_OK,6000U);
  assert(Px4Lite_GetModuleStatus(PX4LITE_MODULE_DISPLAY,&status)==PX4LITE_OK);
  assert(status.state==PX4LITE_STATE_DEGRADED && status.fault_code==PX4LITE_FAULT_DISPLAY_TOUCH && status.last_valid_ms==6000U);
  display_runtime.touch_ready=1U;
  Business_DisplayReportResult(BUSINESS_SERVICE_OK,6100U);
  assert(Px4Lite_GetModuleStatus(PX4LITE_MODULE_DISPLAY,&status)==PX4LITE_OK && status.state==PX4LITE_STATE_ONLINE);
  Business_DisplayReportResult(BUSINESS_SERVICE_ERROR,6200U);
  assert(Px4Lite_GetModuleStatus(PX4LITE_MODULE_DISPLAY,&status)==PX4LITE_OK && status.fault_code==PX4LITE_FAULT_DISPLAY_REFRESH);
  puts("display touch independent health passed"); return 0;
}
