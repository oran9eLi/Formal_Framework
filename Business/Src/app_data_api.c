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
#include "px4lite_faults.h"
#include "px4lite_mavlink_rx.h"
#include "px4lite_modules.h"
#include "px4lite_topics.h"

/*
 * Display 聚合视图只由 biz_display 周期任务调用。以下 scratch 使用静态存储，
 * 避免把 System/Alarm 等较大快照压到任务栈上。
 */
static App_NavigationSnapshot_t s_display_navigation_scratch;
static App_SystemSnapshot_t s_display_system_scratch;
static App_EnvironmentSnapshot_t s_display_environment_scratch;
static App_DateTimeSnapshot_t s_display_date_time_scratch;
static App_MotorSnapshot_t s_display_motor_scratch;
static Px4Lite_AlarmSnapshot_t s_display_alarm_scratch;
static Px4Lite_RemoteTelemetry_t s_display_remote_scratch;

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
  uint32_t publish_time_ms;
  uint16_t active_count;
  uint16_t highest_fault_code;
  uint16_t highest_source_id;
  Px4Lite_AlarmSeverity_t highest_severity;

  if ((Px4Lite_CopyAlarmSummary(&publish_time_ms, 0, &active_count, &highest_fault_code, &highest_source_id, &highest_severity) == PX4LITE_OK) && ((uint32_t)(now_ms - publish_time_ms) <= APP_ALARM_MAX_AGE_MS)) {
    out->active_alarm_count = active_count;
    out->highest_fault_code = highest_fault_code;
    out->highest_source_id  = highest_source_id;
    out->highest_severity   = (uint8_t)highest_severity;
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
  Px4Lite_AlarmRecord_t source_record;
  uint32_t publish_time_ms;
  uint32_t sequence;
  Px4Lite_AlarmSeverity_t highest_severity;
  uint16_t i;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }

  memset(out, 0, sizeof(*out));
  if (Px4Lite_CopyAlarmSummary(&publish_time_ms, &sequence, &out->active_count, &out->highest_fault_code, &out->highest_source_id, &highest_severity) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if ((uint32_t)(now_ms - publish_time_ms) > APP_ALARM_MAX_AGE_MS) { return PX4LITE_STALE; }

  out->header.sample_time_ms  = publish_time_ms;
  out->header.publish_time_ms = publish_time_ms;
  out->header.sequence        = sequence;
  out->header.device_id       = (uint16_t)PX4LITE_MODULE_ALARM;
  out->header.valid           = 1U;
  out->header.flags           = PX4LITE_DATA_VALID;
  out->highest_severity       = (uint8_t)highest_severity;

  for (i = 0U; i < (uint16_t)PX4LITE_MODULE_COUNT; ++i) {
    if (Px4Lite_CopyAlarmRecord(i, &source_record) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
    out->records[i].source_id  = source_record.source_id;
    out->records[i].fault_code = source_record.fault_code;
    out->records[i].severity   = (uint8_t)source_record.severity;
    out->records[i].active     = source_record.active;
    out->records[i].raised_ms  = source_record.raised_ms;
    out->records[i].updated_ms = source_record.updated_ms;
    out->records[i].detail     = source_record.detail;
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
  return Px4Lite_ControlSetMotorThrottlePercent(motor_index, throttle_percent);
}

uint8_t App_CommandMotorThrottlePercent(uint8_t motor_index, uint8_t throttle_percent)
{
  return (App_SetMotorThrottlePercent(motor_index, throttle_percent) == PX4LITE_OK) ? 1U : 0U;
}

Px4Lite_Result_t App_CopyDateTime(App_DateTimeSnapshot_t *out, uint32_t now_ms)
{
  Px4Lite_TimeSnapshot_t source;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }

  memset(out, 0, sizeof(*out));
  if (Px4Lite_CopyTime(&source) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&source.header, now_ms, APP_DATETIME_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }

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

Px4Lite_Result_t App_CopyAlarmSummary(App_AlarmSummary_t *out, uint32_t now_ms)
{
  uint32_t publish_time_ms;
  Px4Lite_AlarmSeverity_t highest_severity;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }

  memset(out, 0, sizeof(*out));
  if (Px4Lite_CopyAlarmSummary(&publish_time_ms, 0, &out->active_count, &out->highest_fault_code, &out->highest_source_id, &highest_severity) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if ((uint32_t)(now_ms - publish_time_ms) > APP_ALARM_MAX_AGE_MS) { return PX4LITE_STALE; }

  out->highest_severity = (uint8_t)highest_severity;
  return PX4LITE_OK;
}

/**
 * @brief 将 Framework 模块状态转换为应用层显示状态。
 */
static App_ViewState_t App_MapViewState(Px4Lite_State_t state)
{
  switch (state) {
    case PX4LITE_STATE_STARTING:
      return APP_VIEW_STATE_STARTING;
    case PX4LITE_STATE_ONLINE:
      return APP_VIEW_STATE_ONLINE;
    case PX4LITE_STATE_DEGRADED:
      return APP_VIEW_STATE_DEGRADED;
    case PX4LITE_STATE_OFFLINE:
      return APP_VIEW_STATE_OFFLINE;
    case PX4LITE_STATE_FAILED:
      return APP_VIEW_STATE_FAILED;
    case PX4LITE_STATE_UNINITIALIZED:
    default:
      break;
  }

  return APP_VIEW_STATE_UNKNOWN;
}

/**
 * @brief 从 Framework 模块状态填充 Display 模块视图。
 */
static void App_FillModuleViewFromStatus(App_ModuleView_t *out, const Px4Lite_ModuleStatus_t *status)
{
  if ((out == 0) || (status == 0)) { return; }

  out->state      = App_MapViewState(status->state);
  out->fault_code = status->fault_code;
  out->severity   = status->severity;
  out->reserved   = 0U;
}

/**
 * @brief 直接读取一个模块状态作为 Display fallback。
 */
static uint8_t App_CopyModuleView(Px4Lite_ModuleId_t module_id, App_ModuleView_t *out)
{
  Px4Lite_ModuleStatus_t status;

  if (out == 0) { return 0U; }
  if (Px4Lite_GetModuleStatus(module_id, &status) != PX4LITE_OK) { return 0U; }

  App_FillModuleViewFromStatus(out, &status);
  return 1U;
}

/**
 * @brief 填充 Display 关心的固定模块视图。
 */
static uint8_t App_FillDisplayModuleViews(App_DisplaySnapshot_t *out, const App_SystemSnapshot_t *system)
{
  uint8_t loaded = 0U;

  if (out == 0) { return 0U; }

  if (system != 0) {
    App_FillModuleViewFromStatus(&out->gnss, &system->modules[PX4LITE_MODULE_GNSS]);
    App_FillModuleViewFromStatus(&out->imu, &system->modules[PX4LITE_MODULE_IMU]);
    App_FillModuleViewFromStatus(&out->baro, &system->modules[PX4LITE_MODULE_BARO]);
    App_FillModuleViewFromStatus(&out->battery, &system->modules[PX4LITE_MODULE_BATTERY]);
    App_FillModuleViewFromStatus(&out->lora, &system->modules[PX4LITE_MODULE_LORA]);
    App_FillModuleViewFromStatus(&out->storage, &system->modules[PX4LITE_MODULE_STORAGE]);
    App_FillModuleViewFromStatus(&out->control, &system->modules[PX4LITE_MODULE_CONTROL]);
    App_FillModuleViewFromStatus(&out->five_g, &system->modules[PX4LITE_MODULE_5G]);
    return 1U;
  }

  loaded |= App_CopyModuleView(PX4LITE_MODULE_GNSS, &out->gnss);
  loaded |= App_CopyModuleView(PX4LITE_MODULE_IMU, &out->imu);
  loaded |= App_CopyModuleView(PX4LITE_MODULE_BARO, &out->baro);
  loaded |= App_CopyModuleView(PX4LITE_MODULE_BATTERY, &out->battery);
  loaded |= App_CopyModuleView(PX4LITE_MODULE_LORA, &out->lora);
  loaded |= App_CopyModuleView(PX4LITE_MODULE_STORAGE, &out->storage);
  loaded |= App_CopyModuleView(PX4LITE_MODULE_CONTROL, &out->control);
  loaded |= App_CopyModuleView(PX4LITE_MODULE_5G, &out->five_g);

  return loaded;
}

/**
 * @brief 从远端模块状态填充一个 Display 模块视图。
 */
static uint8_t App_FillRemoteModuleView(const Px4Lite_RemoteTelemetry_t *remote, Px4Lite_ModuleId_t module_id, App_ModuleView_t *out)
{
  if ((remote == 0) || (out == 0) || (module_id >= PX4LITE_MODULE_COUNT)) { return 0U; }
  if ((remote->module_state_valid_mask & (1UL << (uint32_t)module_id)) == 0U) { return 0U; }

  out->state      = App_MapViewState(remote->module_state[module_id]);
  out->fault_code = 0U;
  out->severity   = 0U;
  out->reserved   = 0U;
  return 1U;
}

/**
 * @brief 填充远端 Display 关心的固定模块视图。
 */
static uint8_t App_FillRemoteModuleViews(App_DisplaySnapshot_t *out, const Px4Lite_RemoteTelemetry_t *remote)
{
  uint8_t loaded = 0U;

  if ((out == 0) || (remote == 0)) { return 0U; }

  loaded |= App_FillRemoteModuleView(remote, PX4LITE_MODULE_GNSS, &out->gnss);
  loaded |= App_FillRemoteModuleView(remote, PX4LITE_MODULE_IMU, &out->imu);
  loaded |= App_FillRemoteModuleView(remote, PX4LITE_MODULE_BARO, &out->baro);
  loaded |= App_FillRemoteModuleView(remote, PX4LITE_MODULE_BATTERY, &out->battery);
  loaded |= App_FillRemoteModuleView(remote, PX4LITE_MODULE_LORA, &out->lora);
  loaded |= App_FillRemoteModuleView(remote, PX4LITE_MODULE_STORAGE, &out->storage);
  loaded |= App_FillRemoteModuleView(remote, PX4LITE_MODULE_CONTROL, &out->control);
  loaded |= App_FillRemoteModuleView(remote, PX4LITE_MODULE_5G, &out->five_g);
  return loaded;
}

/**
 * @brief 判断 Display 状态是否有可显示数据。
 */
static uint8_t App_ViewStateHasUsableData(App_ViewState_t state)
{
  return ((state == APP_VIEW_STATE_ONLINE) || (state == APP_VIEW_STATE_DEGRADED)) ? 1U : 0U;
}

/**
 * @brief 按严重度从高到低复制告警记录到 Display 视图。
 */
static void App_CopyDisplayAlarms(App_DisplaySnapshot_t *out, const Px4Lite_AlarmSnapshot_t *alarm)
{
  int32_t severity;
  uint16_t record_index;
  uint16_t out_index = 0U;

  if ((out == 0) || (alarm == 0)) { return; }

  out->alarm_valid              = 1U;
  out->alarm_active_count       = alarm->active_count;
  out->alarm_highest_fault_code = alarm->highest_fault_code;
  if (alarm->highest_fault_code != 0U) {
    out->highest_fault_code = alarm->highest_fault_code;
    out->highest_source_id  = alarm->highest_source_id;
  }

  for (severity = (int32_t)PX4LITE_ALARM_FATAL; (severity >= (int32_t)PX4LITE_ALARM_INFO) && (out_index < APP_DISPLAY_ALARM_MAX); severity--) {
    for (record_index = 0U; (record_index < (uint16_t)PX4LITE_MODULE_COUNT) && (out_index < APP_DISPLAY_ALARM_MAX); record_index++) {
      const Px4Lite_AlarmRecord_t *record = &alarm->records[record_index];

      if ((record->active != 0U) && (record->fault_code != 0U) && (record->severity == (uint8_t)severity)) {
        out->alarms[out_index].source_id  = record->source_id;
        out->alarms[out_index].fault_code = record->fault_code;
        out->alarms[out_index].severity   = (uint8_t)record->severity;
        out->alarms[out_index].active     = record->active;
        out->alarms[out_index].reserved   = 0U;
        out->alarms[out_index].raised_ms  = record->raised_ms;
        out->alarms[out_index].updated_ms = record->updated_ms;
        out->alarms[out_index].detail     = record->detail;
        out_index++;
      }
    }
  }
}

uint8_t App_CopyDisplaySnapshot(App_DisplaySnapshot_t *out, uint32_t now_ms)
{
  Px4Lite_CommDebugInfo_t comm;
  uint8_t i;
  uint8_t module_loaded;

  if (out == 0) { return 0U; }

  memset(out, 0, sizeof(*out));

  if (App_CopyNavigation(&s_display_navigation_scratch, now_ms) == PX4LITE_OK) {
    out->navigation_valid   = 1U;
    out->attitude_valid     = ((s_display_navigation_scratch.valid_mask & PX4LITE_NAV_VALID_ATTITUDE) != 0U) ? 1U : 0U;
    out->gnss_utc_sec       = s_display_navigation_scratch.gnss_utc_sec;
    out->gnss_utc_date      = s_display_navigation_scratch.gnss_utc_date;
    out->latitude_e7        = s_display_navigation_scratch.latitude_e7;
    out->longitude_e7       = s_display_navigation_scratch.longitude_e7;
    out->altitude_mm        = s_display_navigation_scratch.altitude_mm;
    out->velocity_north_cms = s_display_navigation_scratch.velocity_north_cms;
    out->velocity_east_cms  = s_display_navigation_scratch.velocity_east_cms;
    out->roll_deg100        = s_display_navigation_scratch.roll_deg100;
    out->pitch_deg100       = s_display_navigation_scratch.pitch_deg100;
    out->yaw_deg100         = s_display_navigation_scratch.yaw_deg100;
    out->hdop_x100          = s_display_navigation_scratch.hdop_x100;
    out->satellites_used    = s_display_navigation_scratch.satellites_used;
    out->gnss_fix_type      = s_display_navigation_scratch.gnss_fix_type;
  }

  if (App_CopyDateTime(&s_display_date_time_scratch, now_ms) == PX4LITE_OK) {
    out->date_time_valid   = 1U;
    out->local_date_ymd    = s_display_date_time_scratch.local_date_ymd;
    out->local_time_hhmmss = s_display_date_time_scratch.local_time_hhmmss;
  }

  if (App_CopySystem(&s_display_system_scratch, now_ms) == PX4LITE_OK) {
    out->system_valid        = 1U;
    out->status_version      = s_display_system_scratch.status_version;
    out->warning_fault_mask  = s_display_system_scratch.warning_fault_mask;
    out->blocking_fault_mask = s_display_system_scratch.blocking_fault_mask;
    out->highest_fault_code  = s_display_system_scratch.highest_fault_code;
    out->highest_source_id   = s_display_system_scratch.highest_source_id;
    out->system_ready        = s_display_system_scratch.system_ready;
    module_loaded            = App_FillDisplayModuleViews(out, &s_display_system_scratch);
  } else {
    module_loaded = App_FillDisplayModuleViews(out, 0);
  }

  if ((Px4Lite_CopyAlarmSnapshot(&s_display_alarm_scratch) == PX4LITE_OK) && (Px4Lite_IsFresh(&s_display_alarm_scratch.header, now_ms, APP_ALARM_MAX_AGE_MS) != 0U)) { App_CopyDisplayAlarms(out, &s_display_alarm_scratch); }

  if (App_CopyEnvironment(&s_display_environment_scratch, now_ms) == PX4LITE_OK) {
    out->environment_valid     = 1U;
    out->pressure_pa           = s_display_environment_scratch.pressure_pa;
    out->temperature_c         = s_display_environment_scratch.temperature_c;
    out->relative_humidity_pct = s_display_environment_scratch.relative_humidity_pct;
    out->voltage_mv            = s_display_environment_scratch.voltage_mv;
    out->battery_percent       = s_display_environment_scratch.battery_percent;
  }

  if (App_CopyMotor(&s_display_motor_scratch, now_ms) == PX4LITE_OK) {
    out->motor_valid = 1U;
    for (i = 0U; (i < APP_DISPLAY_MOTOR_COUNT) && (i < PX4LITE_MOTOR_COUNT); i++) {
      out->motor_duty_percent[i] = s_display_motor_scratch.duty_percent[i];
    }
  }

  App_GetCommStats(&comm);
  out->lora_tx_count = comm.tx_frame_count;
  out->lora_rx_count = comm.rx_frame_count;
  out->lora_lost_count = comm.rx_sequence_lost_count;
  out->lora_ack_count = 0U;
  out->lora_loss_rate_x10 = comm.rx_loss_rate_x10;

  /* 姿态显示需要同时满足数据位有效和 IMU 模块状态可用。 */
  if (App_ViewStateHasUsableData(out->imu.state) == 0U) { out->attitude_valid = 0U; }
  if (App_ViewStateHasUsableData(out->baro.state) == 0U) { out->environment_valid = 0U; }
  if (App_ViewStateHasUsableData(out->battery.state) == 0U) { out->battery_percent = 0U; out->voltage_mv = 0U; }

  out->any_valid = ((out->navigation_valid != 0U) || (out->system_valid != 0U) || (out->environment_valid != 0U) || (module_loaded != 0U)) ? 1U : 0U;
  return out->any_valid;
}

uint8_t App_CopyRemoteDisplaySnapshot(App_DisplaySnapshot_t *out, uint32_t now_ms)
{
  Px4Lite_CommDebugInfo_t comm;
  uint8_t module_loaded;

  if (out == 0) { return 0U; }

  memset(out, 0, sizeof(*out));
  if (Px4Lite_CopyRemoteTelemetry(&s_display_remote_scratch) != PX4LITE_OK) { return 0U; }
  if (Px4Lite_IsFresh(&s_display_remote_scratch.header, now_ms, APP_REMOTE_MAX_AGE_MS) == 0U) { return 0U; }

  module_loaded = App_FillRemoteModuleViews(out, &s_display_remote_scratch);
  if (module_loaded != 0U) {
    out->system_valid = 1U;
    out->status_version = s_display_remote_scratch.header.sequence;
    out->system_ready = 1U;
  }

  if ((s_display_remote_scratch.valid_mask & PX4LITE_REMOTE_VALID_NAVIGATION) != 0U) {
    out->navigation_valid   = 1U;
    out->gnss_utc_sec       = s_display_remote_scratch.gnss_utc_sec;
    out->gnss_utc_date      = s_display_remote_scratch.gnss_utc_date;
    out->latitude_e7        = s_display_remote_scratch.latitude_e7;
    out->longitude_e7       = s_display_remote_scratch.longitude_e7;
    out->altitude_mm        = s_display_remote_scratch.altitude_mm;
    out->velocity_north_cms = s_display_remote_scratch.velocity_north_cms;
    out->velocity_east_cms  = s_display_remote_scratch.velocity_east_cms;
    out->yaw_deg100         = s_display_remote_scratch.yaw_deg100;
    out->hdop_x100          = s_display_remote_scratch.hdop_x100;
    out->satellites_used    = s_display_remote_scratch.satellites_used;
    out->gnss_fix_type      = s_display_remote_scratch.gnss_fix_type;
  }

  if ((s_display_remote_scratch.valid_mask & PX4LITE_REMOTE_VALID_ATTITUDE) != 0U) {
    out->attitude_valid    = 1U;
    out->roll_deg100      = s_display_remote_scratch.roll_deg100;
    out->pitch_deg100     = s_display_remote_scratch.pitch_deg100;
    out->yaw_deg100       = s_display_remote_scratch.yaw_deg100;
  }

  if ((s_display_remote_scratch.valid_mask & PX4LITE_REMOTE_VALID_ENVIRONMENT) != 0U) {
    out->environment_valid     = 1U;
    out->pressure_pa           = s_display_remote_scratch.pressure_pa;
    out->temperature_c         = s_display_remote_scratch.temperature_c;
    out->relative_humidity_pct = s_display_remote_scratch.relative_humidity_pct;
  }

  if ((s_display_remote_scratch.valid_mask & PX4LITE_REMOTE_VALID_BATTERY) != 0U) {
    out->environment_valid = 1U;
    out->voltage_mv        = s_display_remote_scratch.voltage_mv;
    out->battery_percent   = s_display_remote_scratch.battery_percent;
  }

  if ((s_display_remote_scratch.valid_mask & PX4LITE_REMOTE_VALID_ALARM) != 0U) {
    out->alarm_valid               = 1U;
    out->highest_fault_code        = s_display_remote_scratch.highest_fault_code;
    out->highest_source_id         = s_display_remote_scratch.highest_source_id;
    out->alarm_active_count        = (s_display_remote_scratch.highest_fault_code != 0U) ? 1U : 0U;
    out->alarm_highest_fault_code  = s_display_remote_scratch.highest_fault_code;
    out->alarms[0].source_id       = s_display_remote_scratch.highest_source_id;
    out->alarms[0].fault_code      = s_display_remote_scratch.highest_fault_code;
    out->alarms[0].severity        = s_display_remote_scratch.highest_severity;
    out->alarms[0].active          = (s_display_remote_scratch.highest_fault_code != 0U) ? 1U : 0U;
    out->alarms[0].updated_ms      = s_display_remote_scratch.header.sample_time_ms;
  }

  App_GetCommStats(&comm);
  out->lora_tx_count = comm.tx_frame_count;
  out->lora_rx_count = s_display_remote_scratch.rx_frame_count;
  out->lora_lost_count = s_display_remote_scratch.rx_sequence_lost_count;
  out->lora_ack_count = 0U;
  out->lora_loss_rate_x10 = s_display_remote_scratch.rx_loss_rate_x10;

  if (((s_display_remote_scratch.valid_mask & PX4LITE_REMOTE_VALID_MODULES) != 0U) && (App_ViewStateHasUsableData(out->imu.state) == 0U)) { out->attitude_valid = 0U; }
  if (((s_display_remote_scratch.valid_mask & PX4LITE_REMOTE_VALID_MODULES) != 0U) && (App_ViewStateHasUsableData(out->baro.state) == 0U)) { out->environment_valid = 0U; }
  if (((s_display_remote_scratch.valid_mask & PX4LITE_REMOTE_VALID_MODULES) != 0U) && (App_ViewStateHasUsableData(out->battery.state) == 0U)) { out->battery_percent = 0U; out->voltage_mv = 0U; }

  out->any_valid = ((out->navigation_valid != 0U) || (out->attitude_valid != 0U) || (out->system_valid != 0U) || (out->environment_valid != 0U) || (out->alarm_valid != 0U)) ? 1U : 0U;
  return out->any_valid;
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

/**
 * @brief 复制最近一帧 LoRa/MAVLink 接收数据。
 */
Px4Lite_Result_t App_CopyCommRxFrame(Px4Lite_CommRxFrame_t *out)
{
  return Px4Lite_CopyCommRxFrame(out);
}

/**
 * @brief 返回模块或故障域的显示名称。
 */
const char *App_GetModuleDisplayName(uint16_t source_id, uint16_t fault_code)
{
  switch ((Px4Lite_ModuleId_t)source_id) {
    case PX4LITE_MODULE_GNSS:
      return "GPS";
    case PX4LITE_MODULE_IMU:
      return "IMU";
    case PX4LITE_MODULE_BARO:
      return "BARO";
    case PX4LITE_MODULE_BATTERY:
      return "POWER";
    case PX4LITE_MODULE_LORA:
      return "LORA";
    case PX4LITE_MODULE_5G:
      return "5G";
    case PX4LITE_MODULE_STORAGE:
      return "SD";
    case PX4LITE_MODULE_REMOTE_ID:
      return "REMOTE ID";
    case PX4LITE_MODULE_DISPLAY:
      return "DISPLAY";
    case PX4LITE_MODULE_CONTROL:
      return "CONTROL";
    case PX4LITE_MODULE_ALARM:
      return "ALARM";
    case PX4LITE_MODULE_SYSTEM:
      return "SYSTEM";
    case PX4LITE_MODULE_ESTIMATOR:
      return "EST";
    case PX4LITE_MODULE_BUSINESS:
      return "BUSINESS";
    default:
      break;
  }

  switch (fault_code) {
    case PX4LITE_FAULT_SYSTEM_SELF_CHECK:
    case PX4LITE_FAULT_SYSTEM_HEAP:
    case PX4LITE_FAULT_SYSTEM_STACK:
    case PX4LITE_FAULT_SYSTEM_TASK_LOST:
      return "SYSTEM";

    case PX4LITE_FAULT_SENSOR_INIT:
    case PX4LITE_FAULT_SENSOR_OFFLINE:
    case PX4LITE_FAULT_SENSOR_INVALID:
    case PX4LITE_FAULT_SENSOR_NO_FIX:
    case PX4LITE_FAULT_SENSOR_TIMEOUT:
      return "SENSOR";

    case PX4LITE_FAULT_COMM_OFFLINE:
    case PX4LITE_FAULT_COMM_TIMEOUT:
    case PX4LITE_FAULT_COMM_FRAME:
      return "COMM";

    case PX4LITE_FAULT_DISPLAY_OFFLINE:
    case PX4LITE_FAULT_DISPLAY_REFRESH:
      return "DISPLAY";

    case PX4LITE_FAULT_STORAGE_NOT_READY:
    case PX4LITE_FAULT_STORAGE_WRITE:
    case PX4LITE_FAULT_STORAGE_FULL:
      return "STORAGE";

    case PX4LITE_FAULT_PROTOCOL_PARSE:
    case PX4LITE_FAULT_PROTOCOL_CRC:
      return "PROTO";

    case PX4LITE_FAULT_APP_INVALID_DATA:
      return "APP";

    case PX4LITE_FAULT_ESTIMATOR_INPUT:
    case PX4LITE_FAULT_ESTIMATOR_DIVERGE:
      return "EST";

    default:
      break;
  }

  return "SYSTEM";
}

/**
 * @brief 返回故障码对应的显示原因文本。
 */
const char *App_GetFaultReasonText(uint16_t fault_code)
{
  switch (fault_code) {
    case PX4LITE_FAULT_SYSTEM_SELF_CHECK:
      return "SELF CHECK";
    case PX4LITE_FAULT_SYSTEM_HEAP:
      return "HEAP FAILED";
    case PX4LITE_FAULT_SYSTEM_STACK:
      return "STACK OVERFLOW";
    case PX4LITE_FAULT_SYSTEM_TASK_LOST:
      return "TASK LOST";
    case PX4LITE_FAULT_SENSOR_INIT:
      return "SENSOR INIT";
    case PX4LITE_FAULT_SENSOR_OFFLINE:
      return "SENSOR OFFLINE";
    case PX4LITE_FAULT_SENSOR_INVALID:
      return "SENSOR INVALID";
    case PX4LITE_FAULT_SENSOR_NO_FIX:
      return "NO FIX";
    case PX4LITE_FAULT_SENSOR_TIMEOUT:
      return "SENSOR TIMEOUT";
    case PX4LITE_FAULT_COMM_OFFLINE:
      return "COMM OFFLINE";
    case PX4LITE_FAULT_COMM_TIMEOUT:
      return "COMM TIMEOUT";
    case PX4LITE_FAULT_COMM_FRAME:
      return "COMM FRAME";
    case PX4LITE_FAULT_DISPLAY_OFFLINE:
      return "DISPLAY OFFLINE";
    case PX4LITE_FAULT_DISPLAY_REFRESH:
      return "DISPLAY REFRESH";
    case PX4LITE_FAULT_STORAGE_NOT_READY:
      return "STORAGE NOT READY";
    case PX4LITE_FAULT_STORAGE_WRITE:
      return "STORAGE WRITE";
    case PX4LITE_FAULT_STORAGE_FULL:
      return "STORAGE FULL";
    case PX4LITE_FAULT_PROTOCOL_PARSE:
      return "PROTO PARSE";
    case PX4LITE_FAULT_PROTOCOL_CRC:
      return "PROTO CRC";
    case PX4LITE_FAULT_APP_INVALID_DATA:
      return "APP INVALID";
    case PX4LITE_FAULT_ESTIMATOR_INPUT:
      return "EST INPUT";
    case PX4LITE_FAULT_ESTIMATOR_DIVERGE:
      return "EST DIVERGE";
    default:
      break;
  }

  return "UNKNOWN ALARM";
}
