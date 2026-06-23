/**
 * @file app_data_api.c
 * @brief 实现 Business 层只读数据 API 的新鲜度检查和一致性复制。
 *
 * @details
 * 本文件是 Business 消费 Framework 数据的唯一入口实现。上层业务不得直接读取
 * Framework topic、BSP DMA 缓冲或驱动私有变量。
 */

#include "app_data_api.h"

#include <string.h>
#include "px4lite_alarm.h"
#include "px4lite_control.h"
#include "px4lite_modules.h"
#include "px4lite_topics.h"

/**
 * @brief 从已复制的系统快照中汇总活动模块故障。
 *
 * @param[in,out] snapshot 系统快照，允许为 NULL。
 */
static void App_SummarizeSystemFaults(App_SystemSnapshot_t *snapshot)
{
  uint32_t i;
  uint8_t best_severity = 0U;
  uint16_t best_fault   = 0U;
  uint16_t active_count = 0U;

  if (snapshot == 0) { return; }

  snapshot->highest_source_id = (uint16_t)PX4LITE_MODULE_COUNT;
  for (i = 0U; i < (uint32_t)PX4LITE_MODULE_COUNT; ++i) {
    const Px4Lite_ModuleStatus_t *module = &snapshot->modules[i];

    if (module->fault_code == 0U) { continue; }

    active_count++;
    if ((best_fault == 0U) || (module->severity > best_severity)) {
      best_fault                  = module->fault_code;
      best_severity               = module->severity;
      snapshot->highest_source_id = (uint16_t)i;
    }
  }

  snapshot->active_alarm_count = active_count;
  snapshot->highest_fault_code = best_fault;
  snapshot->highest_severity   = best_severity;
}

/**
 * @brief 优先使用告警快照填充系统告警汇总，缺失时回退到模块状态汇总。
 *
 * @param[in,out] out 系统快照，不能为 NULL。
 * @param[in] now_ms 当前系统毫秒时间。
 */
static void App_ApplyAlarmSummary(App_SystemSnapshot_t *out, uint32_t now_ms)
{
  Px4Lite_AlarmSnapshot_t alarm;

  if ((Px4Lite_CopyAlarmSnapshot(&alarm) == PX4LITE_OK) && (Px4Lite_IsFresh(&alarm.header, now_ms, APP_ALARM_MAX_AGE_MS) != 0U)) {
    out->active_alarm_count = alarm.active_count;
    out->highest_fault_code = alarm.highest_fault_code;
    out->highest_source_id  = alarm.highest_source_id;
    out->highest_severity   = (uint8_t)alarm.highest_severity;
  } else {
    App_SummarizeSystemFaults(out);
  }
}

/**
 * @brief 为应用消费者复制一份新鲜的导航快照。
 */
Px4Lite_Result_t App_CopyNavigation(App_NavigationSnapshot_t *out, uint32_t now_ms)
{
  Px4Lite_VehicleNavigation_t source;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }

  memset(out, 0, sizeof(*out));
  if (Px4Lite_CopyNavigation(&source) != PX4LITE_OK) { return PX4LITE_NOT_READY; }

  if (Px4Lite_IsFresh(&source.header, now_ms, APP_NAVIGATION_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }

  out->header             = source.header;
  out->valid_mask         = source.valid_mask;
  out->gnss_utc_sec       = source.gnss_utc_sec;
  out->gnss_utc_date      = source.gnss_utc_date;
  out->latitude_e7        = source.latitude_e7;
  out->longitude_e7       = source.longitude_e7;
  out->altitude_mm        = source.fused_altitude_mm;
  out->velocity_north_cms = source.velocity_north_cms;
  out->velocity_east_cms  = source.velocity_east_cms;
  out->velocity_down_cms  = source.velocity_down_cms;
  out->roll_deg100        = source.roll_deg100;
  out->pitch_deg100       = source.pitch_deg100;
  out->yaw_deg100         = source.yaw_deg100;
  out->roll_rate_dps100   = source.roll_rate_dps100;
  out->pitch_rate_dps100  = source.pitch_rate_dps100;
  out->yaw_rate_dps100    = source.yaw_rate_dps100;
  out->hdop_x100          = source.hdop_x100;
  out->satellites_used    = source.satellites_used;
  out->gnss_fix_type      = source.gnss_fix_type;
  out->navigation_quality = source.navigation_quality;
  return PX4LITE_OK;
}

/**
 * @brief 复制一份版本一致的系统健康和模块状态快照。
 */
Px4Lite_Result_t App_CopySystem(App_SystemSnapshot_t *out, uint32_t now_ms)
{
  Px4Lite_SystemHealth_t health;
  uint32_t version_before;
  uint32_t version_after;
  uint32_t attempt;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }

  memset(out, 0, sizeof(*out));
  for (attempt = 0U; attempt < APP_STATUS_COPY_RETRY_MAX; ++attempt) {
    if (Px4Lite_CopyHealth(&health) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
    if (Px4Lite_IsFresh(&health.header, now_ms, APP_SYSTEM_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }

    version_before = health.status_version;
    if (Px4Lite_CopyModuleStatuses(out->modules, PX4LITE_MODULE_COUNT, &version_after) != PX4LITE_OK) { return PX4LITE_IO_ERROR; }

    if ((version_before == version_after) && (version_after == Px4Lite_GetStatusVersion())) {
      out->header              = health.header;
      out->blocking_fault_mask = health.blocking_fault_mask;
      out->warning_fault_mask  = health.warning_fault_mask;
      out->not_ready_mask      = health.not_ready_mask;
      out->status_version      = version_after;
      out->system_ready        = ((health.blocking_fault_mask == 0U) && (health.not_ready_mask == 0U)) ? 1U : 0U;
      App_ApplyAlarmSummary(out, now_ms);
      return PX4LITE_OK;
    }
  }

  memset(out, 0, sizeof(*out));
  return PX4LITE_BUSY;
}

/**
 * @brief 复制一份新鲜的活动告警快照。
 */
Px4Lite_Result_t App_CopyAlarm(App_AlarmSnapshot_t *out, uint32_t now_ms)
{
  Px4Lite_AlarmSnapshot_t source;
  uint16_t i;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }

  memset(out, 0, sizeof(*out));
  if (Px4Lite_CopyAlarmSnapshot(&source) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&source.header, now_ms, APP_ALARM_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }

  out->header             = source.header;
  out->active_count       = source.active_count;
  out->highest_fault_code = source.highest_fault_code;
  out->highest_source_id  = source.highest_source_id;
  out->highest_severity   = (uint8_t)source.highest_severity;

  for (i = 0U; i < (uint16_t)PX4LITE_MODULE_COUNT; ++i) {
    out->records[i].source_id  = source.records[i].source_id;
    out->records[i].fault_code = source.records[i].fault_code;
    out->records[i].severity   = (uint8_t)source.records[i].severity;
    out->records[i].active     = source.records[i].active;
    out->records[i].raised_ms  = source.records[i].raised_ms;
    out->records[i].updated_ms = source.records[i].updated_ms;
    out->records[i].detail     = source.records[i].detail;
  }

  return PX4LITE_OK;
}

/**
 * @brief 复制一份新鲜的环境快照。
 */
Px4Lite_Result_t App_CopyEnvironment(App_EnvironmentSnapshot_t *out, uint32_t now_ms)
{
  Px4Lite_SensorBaro_t baro;
  Px4Lite_BatteryStatus_t battery;
  uint8_t copied = 0U;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }

  memset(out, 0, sizeof(*out));
  if ((Px4Lite_CopyBaro(&baro) == PX4LITE_OK) && (Px4Lite_IsFresh(&baro.header, now_ms, APP_ENVIRONMENT_MAX_AGE_MS) != 0U)) {
    out->header                = baro.header;
    out->pressure_pa           = baro.pressure_pa;
    out->temperature_c         = baro.temperature_c;
    out->relative_humidity_pct = baro.relative_humidity_pct;
    copied                     = 1U;
  }

  if ((Px4Lite_CopyBattery(&battery) == PX4LITE_OK) && (Px4Lite_IsFresh(&battery.header, now_ms, APP_ENVIRONMENT_MAX_AGE_MS) != 0U)) {
    if (copied == 0U) { out->header = battery.header; }
    out->voltage_mv      = battery.voltage_mv;
    out->battery_percent = battery.percent;
    out->low_voltage     = battery.low_voltage;
    copied               = 1U;
  }

  return (copied != 0U) ? PX4LITE_OK : PX4LITE_NOT_READY;
}

Px4Lite_Result_t App_CopyMotor(App_MotorSnapshot_t *out, uint32_t now_ms)
{
  Px4Lite_MotorOutputs_t source;
  uint8_t i;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }

  memset(out, 0, sizeof(*out));
  if (Px4Lite_CopyMotor(&source) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&source.header, now_ms, APP_MOTOR_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }

  out->header      = source.header;
  out->run_state   = source.run_state;
  out->speed_level = source.speed_level;
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    out->duty_percent[i] = source.duty_percent[i];
  }
  return PX4LITE_OK;
}

Px4Lite_Result_t App_SetMotorThrottlePercent(uint8_t motor_index, uint8_t throttle_percent)
{
  if (Px4Lite_RemoteTelemetryGetMode() == PX4LITE_REMOTE_MODE_REMOTE) { return PX4LITE_NOT_READY; }
  return Px4Lite_ControlSetMotorThrottlePercent(motor_index, throttle_percent);
}

Px4Lite_Result_t App_CopyDateTime(App_DateTimeSnapshot_t *out, uint32_t now_ms)
{
  Px4Lite_TimeSnapshot_t source;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }

  memset(out, 0, sizeof(*out));
  if (Px4Lite_CopyTime(&source) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&source.header, now_ms, APP_SYSTEM_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }

  out->header            = source.header;
  out->utc_date_ymd      = source.utc_date_ymd;
  out->utc_time_hhmmss   = source.utc_time_hhmmss;
  out->local_date_ymd    = source.local_date_ymd;
  out->local_time_hhmmss = source.local_time_hhmmss;
  out->last_sync_ms      = source.last_sync_ms;
  out->sync_age_s        = source.sync_age_s;
  out->source            = source.source;
  out->sync_state        = source.sync_state;
  return PX4LITE_OK;
}

/**
 * @brief 复制指定 Framework 模块的最新状态。
 */
Px4Lite_Result_t App_GetModuleStatus(Px4Lite_ModuleId_t module_id, Px4Lite_ModuleStatus_t *out)
{
  return Px4Lite_GetModuleStatus(module_id, out);
}

/**
 * @brief 复制最新 LoRa/MAVLink 通信统计。
 */
void App_GetCommStats(Px4Lite_CommDebugInfo_t *out)
{
  Px4Lite_GetCommDebugInfo(out);
}

Px4Lite_RemoteMode_t App_GetRemoteDisplayMode(void)
{
  return Px4Lite_RemoteTelemetryGetMode();
}

Px4Lite_Result_t App_CopyRemoteTelemetry(Px4Lite_RemoteTelemetrySnapshot_t *out, uint32_t now_ms)
{
  return Px4Lite_RemoteTelemetryCopySnapshot(out, now_ms);
}
