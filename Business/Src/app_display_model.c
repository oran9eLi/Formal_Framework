/**
 * @file app_display_model.c
 * @brief Business 层本机显示模型；所有主页面固定消费 App 本机快照。
 * @details 对端选择只作用于通信诊断，不再覆盖本机数据、告警或日志。
 *          相对姿态零位仅用于显示，不修改测量 topic、控制或记录数据。
 */
#include "app_display_model.h"
#include "app_message_log.h"
#include <string.h>

/** @brief 复制本机导航，并应用仅供观察的相对姿态参考。 */
Px4Lite_Result_t App_GetDisplayNavigation(App_NavigationSnapshot_t *out, uint32_t now_ms)
{
  App_AttitudeStatus_t status;
  Px4Lite_Result_t result;
  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  result = App_CopyNavigation(out, now_ms);
  if ((result == PX4LITE_OK) && ((out->valid_mask & PX4LITE_NAV_VALID_ATTITUDE) != 0U) &&
      (App_CopyAttitudeStatus(&status) == PX4LITE_OK) && (status.reference_active != 0U)) {
    out->roll_deg100 -= status.offset_deg100[0];
    if (out->roll_deg100 >= 18000) { out->roll_deg100 -= 36000; }
    if (out->roll_deg100 < -18000) { out->roll_deg100 += 36000; }
    out->pitch_deg100 -= status.offset_deg100[1];
    out->yaw_deg100 -= status.offset_deg100[2];
    if (out->yaw_deg100 >= 18000) { out->yaw_deg100 -= 36000; }
    if (out->yaw_deg100 < -18000) { out->yaw_deg100 += 36000; }
  }
  return result;
}

/** @brief 复制本机环境与电源，保留 App 的逐字段有效性。 */
Px4Lite_Result_t App_GetDisplayEnvironment(App_EnvironmentSnapshot_t *out, uint32_t now_ms)
{
  return App_CopyEnvironment(out, now_ms);
}

/** @brief 复制本机模块状态，不使用对端心跳替代设备状态。 */
Px4Lite_Result_t App_GetDisplaySystem(App_SystemSnapshot_t *out, uint32_t now_ms)
{
  return App_CopySystem(out, now_ms);
}

/** @brief 复制本机活动告警；失败返回值不等于没有告警。 */
Px4Lite_Result_t App_GetDisplayAlarm(App_AlarmSnapshot_t *out, uint32_t now_ms)
{
  return App_CopyAlarm(out, now_ms);
}

/** @brief 复制本机目标与实际 PWM，不从远端百分比推算输出事实。 */
Px4Lite_Result_t App_GetDisplayMotor(App_MotorSnapshot_t *out, uint32_t now_ms)
{
  return App_CopyMotor(out, now_ms);
}

/** @brief 复制本机统一日期时间；Uptime 另由毫秒时基提供。 */
Px4Lite_Result_t App_GetDisplayDateTime(App_DateTimeSnapshot_t *out, uint32_t now_ms)
{
  return App_CopyDateTime(out, now_ms);
}

/** @brief 复制独立于显示运行的本机事件历史。 */
Px4Lite_Result_t App_GetDisplayMessageLog(App_DisplayLogSnapshot_t *out, uint32_t now_ms)
{
  (void)now_ms;
  return App_MessageLogCopy(out);
}

/** @brief 复制本机链路统计，切页和对端选择不改变收发开关。 */
void App_GetDisplayLinkStatus(App_DisplayLinkStatus_t *out, uint32_t now_ms)
{
  Px4Lite_CommDebugInfo_t comm;
  Px4Lite_ModuleStatus_t lora_status;

  (void)now_ms;
  if (out == 0) { return; }

  memset(out, 0, sizeof(*out));
  App_GetCommStats(&comm);
  out->tx_frame_count = comm.tx_frame_count;
  out->rx_frame_count = comm.rx_frame_count;
  out->loss_permille  = (uint16_t)comm.rx_loss_rate_x10; /* 0.1% 单位，与旧 permille 同义。 */
  out->lora_state     = (App_GetModuleStatus(PX4LITE_MODULE_LORA, &lora_status) == PX4LITE_OK)
                          ? (uint8_t)lora_status.state
                          : (uint8_t)PX4LITE_STATE_OFFLINE;
}

/** @brief 按域 getter 返回值置位 ready/stale 位图。 */
static void AppDisplay_MarkDomain(App_DisplayModel_t *out, App_DisplayDomain_t domain, Px4Lite_Result_t res)
{
  if (res == PX4LITE_OK) {
    out->ready_mask |= (1UL << (uint32_t)domain);
  } else if (res == PX4LITE_STALE) {
    out->stale_mask |= (1UL << (uint32_t)domain);
  }
}

/** @brief 顺序复制本机各域并汇总有效位；调用者提供固定存储，不跨域宣称原子采样。 */
void App_BuildDisplayModel(App_DisplayModel_t *out, uint32_t now_ms)
{
  if (out == 0) { return; }

  memset(out, 0, sizeof(*out));
  out->mode = App_GetRemoteDisplayMode();

  AppDisplay_MarkDomain(out, APP_DISPLAY_DOMAIN_NAVIGATION, App_GetDisplayNavigation(&out->navigation, now_ms));
  AppDisplay_MarkDomain(out, APP_DISPLAY_DOMAIN_ENVIRONMENT, App_GetDisplayEnvironment(&out->environment, now_ms));
  AppDisplay_MarkDomain(out, APP_DISPLAY_DOMAIN_SYSTEM, App_GetDisplaySystem(&out->system, now_ms));
  AppDisplay_MarkDomain(out, APP_DISPLAY_DOMAIN_ALARM, App_GetDisplayAlarm(&out->alarm, now_ms));
  AppDisplay_MarkDomain(out, APP_DISPLAY_DOMAIN_MOTOR, App_GetDisplayMotor(&out->motor, now_ms));
  AppDisplay_MarkDomain(out, APP_DISPLAY_DOMAIN_DATETIME, App_GetDisplayDateTime(&out->date_time, now_ms));
  AppDisplay_MarkDomain(out, APP_DISPLAY_DOMAIN_MESSAGE_LOG, App_GetDisplayMessageLog(&out->message_log, now_ms));

  App_GetDisplayLinkStatus(&out->link, now_ms);
}
