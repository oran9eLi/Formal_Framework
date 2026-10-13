/**
 * @file app_data_api.c
 * @brief 实现 Business 层只读数据 API 的新鲜度检查和一致性复制。
 *
 * @details
 * 本文件是 Business 消费 Framework 数据的唯一入口实现。上层业务不得直接读取
 * Framework topic、BSP DMA 缓冲或驱动私有变量。
 */

#include "app_data_api.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>
#include "px4lite_alarm.h"
#include "px4lite_config.h"
#include "px4lite_control.h"
#include "px4lite_faults.h"
#include "px4lite_identity.h"
#include "px4lite_local_msglog.h"
#include "px4lite_mavlink_rx.h"
#include "px4lite_mavlink_tx.h"
#include "px4lite_modules.h"
#include "px4lite_platform.h"
#include "px4lite_remote_telemetry.h"
#include "px4lite_topics.h"

/** @brief 常驻资源采集入口，打印开关不参与门控。 */
void App_RuntimeStatsStep(uint32_t now_ms) { Px4Lite_RuntimeStep(now_ms); }
/** @brief 复制汇总，调用方依据measured_ms显示年龄。 */
Px4Lite_Result_t App_CopyRuntimeStats(App_RuntimeSnapshot_t *out, uint32_t now_ms)
{ (void)now_ms; return Px4Lite_RuntimeCopy(out); }
/** @brief 复制一个任务水位，不触发栈扫描。 */
Px4Lite_Result_t App_CopyTaskRuntime(uint8_t index, App_TaskRuntime_t *out, uint32_t now_ms)
{ (void)now_ms; return Px4Lite_RuntimeCopyTask(index, out); }
/** @brief 仅显示任务提交其LVGL及耗时事实。 */
void App_ReportDisplayRuntime(const App_DisplayRuntime_t *sample) { Px4Lite_RuntimeReportDisplay(sample); }
/** @brief 微秒时基；无符号减法处理约71分钟回绕。 */
uint32_t App_RuntimeGetUs(void) { return Px4Lite_PlatformGetUs(); }
/** @brief 课程字段解码仅访问调用方的稳定原字节副本。 */
Px4Lite_Result_t App_DecodeCapturedFrame(const App_CaptureFrame_t *frame, uint8_t page, App_DecodedPage_t *out)
{ return Px4Lite_DecodeFrame(frame, page, out); }
/** @brief 应用捕获请求边界，不从UI读取生产队列。 */
Px4Lite_Result_t App_RequestCapture(const App_CaptureRequest_t *request) { return Px4Lite_CaptureRequest(request); }
/** @brief 捕获状态复制边界。 */
Px4Lite_Result_t App_CopyCaptureStatus(App_CaptureStatus_t *out) { return Px4Lite_CaptureCopyStatus(out); }
/** @brief 汇总行复制边界。 */
Px4Lite_Result_t App_CopyCaptureSummaryRow(uint32_t session, uint8_t index, App_CaptureSummary_t *out) { return Px4Lite_CaptureCopySummary(session, index, out); }
/** @brief 稳定原始帧复制边界。 */
Px4Lite_Result_t App_CopyCapturedFrame(uint32_t session, uint32_t token, App_CaptureFrame_t *out) { return Px4Lite_CaptureCopyFrame(session, token, out); }
/** @brief 同槽阶段更新，保留Raw选帧字节。 */
Px4Lite_Result_t App_CopyCapturedStage(uint32_t session, uint32_t token, uint8_t *stage) { return Px4Lite_CaptureCopyStage(session, token, stage); }
/** @brief 稳定NMEA句复制边界。 */
Px4Lite_Result_t App_CopyNmeaSentence(uint32_t session, uint32_t token, App_NmeaSentence_t *out) { return Px4Lite_CaptureCopyNmea(session, token, out); }

/** @brief IMU观察不抢占估计器；合法零值与无测量由valid明确区分。 */
Px4Lite_Result_t App_CopyImuLatest(App_ImuSnapshot_t *out, uint32_t now_ms)
{
  if (!out) { return PX4LITE_INVALID_PARAM; }
  memset(out, 0, sizeof(*out));
  if (Px4Lite_CopyImuLatest(out) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (!out->header.valid) { return PX4LITE_NOT_READY; }
  if (now_ms - out->header.sample_time_ms > PX4LITE_IMU_MAX_AGE_MS) { out->header.valid = 0U; return PX4LITE_STALE; }
  return PX4LITE_OK;
}
/** @brief GNSS逐域有效性取真实源时间，不用NMEA链路活动续期位置。 */
Px4Lite_Result_t App_CopyGnssDetails(App_GnssDetails_t *out, uint32_t now_ms)
{
  if (!out) { return PX4LITE_INVALID_PARAM; }
  memset(out, 0, sizeof(*out));
  if (Px4Lite_CopyGnss(out) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (now_ms - out->position_update_ms > PX4LITE_GNSS_MAX_AGE_MS) { out->valid_mask &= ~PX4LITE_GNSS_VALID_POSITION; }
  if (now_ms - out->velocity_update_ms > PX4LITE_GNSS_MAX_AGE_MS) { out->valid_mask &= ~PX4LITE_GNSS_VALID_VELOCITY; }
  if (now_ms - out->time_update_ms > PX4LITE_GNSS_MAX_AGE_MS) { out->valid_mask &= ~PX4LITE_GNSS_VALID_TIME; }
  if (now_ms - out->altitude_update_ms > PX4LITE_GNSS_MAX_AGE_MS) { out->valid_mask &= ~PX4LITE_GNSS_VALID_ALTITUDE; }
  return now_ms - out->last_rx_ms <= PX4LITE_GNSS_MAX_AGE_MS ? PX4LITE_OK : PX4LITE_STALE;
}

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
static Px4Lite_RemoteNodeStatus_t s_remote_node_status_scratch[PX4LITE_REMOTE_NODE_MAX];

static uint16_t App_CountBits32(uint32_t value)
{
  uint16_t count = 0U;

  while (value != 0U) {
    count = (uint16_t)(count + (uint16_t)(value & 1UL));
    value >>= 1U;
  }
  return count;
}

static uint8_t App_IsLeapYear(uint32_t year)
{
  if ((year % 400UL) == 0UL) { return 1U; }
  if ((year % 100UL) == 0UL) { return 0U; }
  return ((year % 4UL) == 0UL) ? 1U : 0U;
}

static uint8_t App_DaysInMonth(uint32_t year, uint32_t month)
{
  static const uint8_t days[12] = {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};

  if ((month < 1U) || (month > 12U)) { return 31U; }
  if ((month == 2U) && (App_IsLeapYear(year) != 0U)) { return 29U; }
  return days[month - 1U];
}

static uint8_t App_FillDisplayTimeFromGnss(App_DisplaySnapshot_t *out, uint32_t utc_date, uint32_t utc_sec)
{
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t local_sec;

  if ((out == 0) || (utc_date == 0U) || (utc_sec >= 86400UL)) { return 0U; }

  year = 2000UL + (utc_date / 10000UL);
  month = (utc_date / 100UL) % 100UL;
  day = utc_date % 100UL;
  if ((year < 2000UL) || (year > 2099UL) || (month < 1UL) || (month > 12UL) || (day < 1UL) || (day > (uint32_t)App_DaysInMonth(year, month))) { return 0U; }

  local_sec = utc_sec + PX4LITE_TIME_LOCAL_OFFSET_S;
  while (local_sec >= 86400UL) {
    local_sec -= 86400UL;
    day++;
    if (day > (uint32_t)App_DaysInMonth(year, month)) {
      day = 1UL;
      month++;
      if (month > 12UL) {
        month = 1UL;
        year++;
      }
    }
  }

  out->date_time_valid = 1U;
  out->local_date_ymd = (year * 10000UL) + (month * 100UL) + day;
  out->local_time_hhmmss = ((local_sec / 3600UL) * 10000UL) + (((local_sec / 60UL) % 60UL) * 100UL) + (local_sec % 60UL);
  return 1U;
}

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
  out->attitude_measurement_ms = source.attitude_update_ms;
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
  Px4Lite_BatteryStatus_t battery2;
  uint8_t copied = 0U;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }

  memset(out, 0, sizeof(*out));
  if ((Px4Lite_CopyBaro(&baro) == PX4LITE_OK) && (Px4Lite_IsFresh(&baro.header, now_ms, APP_ENVIRONMENT_MAX_AGE_MS) != 0U)) {
    out->header                = baro.header;
    out->environment_measurement_ms = baro.header.sample_time_ms;
    out->pressure_pa           = baro.pressure_pa;
    out->temperature_c         = baro.temperature_c;
    out->relative_humidity_pct = baro.relative_humidity_pct;
    out->valid_mask |= APP_ENV_VALID_ENVIRONMENT;
    copied                     = 1U;
  }

  if ((Px4Lite_CopyBattery(&battery) == PX4LITE_OK) && (Px4Lite_IsFresh(&battery.header, now_ms, APP_ENVIRONMENT_MAX_AGE_MS) != 0U)) {
    if (copied == 0U) { out->header = battery.header; }
    out->voltage_mv      = battery.voltage_mv;
    out->power1_measurement_ms = battery.header.sample_time_ms;
    out->current_ma      = (battery.current_valid != 0U) ? battery.current_ma : 0;
    out->battery_percent = battery.percent;
    out->low_voltage     = battery.low_voltage;
    out->valid_mask |= APP_ENV_VALID_VOLTAGE1 | APP_ENV_VALID_PERCENT1;
    if (battery.current_valid != 0U) { out->valid_mask |= APP_ENV_VALID_CURRENT1; }
    copied               = 1U;
  }

  if ((Px4Lite_CopyBattery2(&battery2) == PX4LITE_OK) && (Px4Lite_IsFresh(&battery2.header, now_ms, APP_ENVIRONMENT_MAX_AGE_MS) != 0U)) {
    if (copied == 0U) { out->header = battery2.header; }
    out->voltage2_mv      = battery2.voltage_mv;
    out->power2_measurement_ms = battery2.header.sample_time_ms;
    out->current2_ma      = (battery2.current_valid != 0U) ? battery2.current_ma : 0;
    out->battery2_percent = battery2.percent;
    out->low_voltage2     = battery2.low_voltage;
    out->valid_mask |= APP_ENV_VALID_VOLTAGE2 | APP_ENV_VALID_PERCENT2;
    if (battery2.current_valid != 0U) { out->valid_mask |= APP_ENV_VALID_CURRENT2; }
    copied                = 1U;
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
  out->control_mode = source.control_mode;
  out->operation_state = source.operation_state;
  out->control_mode_valid = 1U;
  out->operation_state_valid = 1U;
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    out->duty_percent[i] = source.duty_percent[i];
    out->pulse_us[i]     = source.pulse_us[i];
    out->base_percent[i] = source.base_percent[i];
  }
  return PX4LITE_OK;
}

Px4Lite_Result_t App_SetMotorThrottlePercent(uint8_t motor_index, uint8_t throttle_percent)
{
  return Px4Lite_ControlSetMotorThrottlePercent(motor_index, throttle_percent);
}

Px4Lite_Result_t App_StartMotorAutoTakeoff(void)
{
  return Px4Lite_ControlStartAutoTakeoff();
}

Px4Lite_Result_t App_StartMotorAutoLanding(void)
{
  return Px4Lite_ControlStartAutoLanding();
}

Px4Lite_Result_t App_SetMotorControlMode(Px4Lite_ControlMode_t mode)
{
  if (App_GetRemoteDisplayMode() == PX4LITE_REMOTE_MODE_REMOTE) { return PX4LITE_NOT_READY; }
  return Px4Lite_ControlSetMode(mode);
}

Px4Lite_Result_t App_EmergencyStopMotors(void)
{
  return Px4Lite_ControlEmergencyStop(Px4Lite_PlatformGetMs());
}

/** @brief 通过 Control 只读查询本机低压事实。 */
uint8_t App_IsMotorPowerLowVoltage(void)
{
  return Px4Lite_ControlIsPowerLowVoltage();
}

uint8_t App_IsMotorPowerInhibited(void)
{
  return Px4Lite_ControlIsPowerInhibited();
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
    App_FillModuleViewFromStatus(&out->remote_id, &system->modules[PX4LITE_MODULE_REMOTE_ID]);
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
  loaded |= App_CopyModuleView(PX4LITE_MODULE_REMOTE_ID, &out->remote_id);

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
  loaded |= App_FillRemoteModuleView(remote, PX4LITE_MODULE_REMOTE_ID, &out->remote_id);
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
  out->view_node_id = (uint8_t)Px4Lite_IdentityGetNodeId();
  out->view_system_id = (uint8_t)Px4Lite_IdentityGetMavlinkSystemId();
  out->view_remote_id_valid = 1U;

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
    out->voltage2_mv           = s_display_environment_scratch.voltage2_mv;
    out->current_ma            = s_display_environment_scratch.current_ma;
    out->current2_ma           = s_display_environment_scratch.current2_ma;
    out->battery_percent       = s_display_environment_scratch.battery_percent;
    out->battery2_percent      = s_display_environment_scratch.battery2_percent;
    out->low_voltage           = s_display_environment_scratch.low_voltage;
    out->low_voltage2          = s_display_environment_scratch.low_voltage2;
  }

  if (App_CopyMotor(&s_display_motor_scratch, now_ms) == PX4LITE_OK) {
    out->motor_valid = 1U;
    for (i = 0U; (i < APP_DISPLAY_MOTOR_COUNT) && (i < PX4LITE_MOTOR_COUNT); i++) {
      out->motor_duty_percent[i] = s_display_motor_scratch.duty_percent[i];
      out->motor_pulse_us[i]     = s_display_motor_scratch.pulse_us[i];
      out->motor_base_percent[i] = s_display_motor_scratch.base_percent[i];
    }
    out->motor_run_state   = s_display_motor_scratch.run_state;
    out->motor_speed_level = s_display_motor_scratch.speed_level;
  }

  App_GetCommStats(&comm);
  out->lora_tx_count = comm.tx_frame_count;
  out->lora_rx_count = comm.rx_frame_count;
  out->lora_lost_count = comm.rx_sequence_lost_count;
  out->lora_ack_count = comm.mav_command_ack_rx_count + comm.mav_command_ack_tx_count;
  out->lora_loss_rate_x10 = comm.rx_loss_rate_x10;

  /* 姿态显示需要同时满足数据位有效和 IMU 模块状态可用。 */
  if (App_ViewStateHasUsableData(out->imu.state) == 0U) { out->attitude_valid = 0U; }
  if (App_ViewStateHasUsableData(out->baro.state) == 0U) { out->environment_valid = 0U; }
  if (App_ViewStateHasUsableData(out->battery.state) == 0U) {
    out->battery_percent = 0U;
    out->battery2_percent = 0U;
    out->voltage_mv = 0U;
    out->voltage2_mv = 0U;
    out->current_ma = 0;
    out->current2_ma = 0;
  }

  out->any_valid = ((out->navigation_valid != 0U) || (out->system_valid != 0U) || (out->environment_valid != 0U) || (module_loaded != 0U)) ? 1U : 0U;
  return out->any_valid;
}

static uint8_t App_CopyRemoteDisplaySnapshotImpl(App_DisplaySnapshot_t *out, uint32_t now_ms)
{
  Px4Lite_CommDebugInfo_t comm;
  uint8_t module_loaded;

  if (out == 0) { return 0U; }

  if (Px4Lite_CopyRemoteTelemetry(&s_display_remote_scratch) != PX4LITE_OK) { return 0U; }
  Px4Lite_RemoteTelemetryRefreshFreshness(&s_display_remote_scratch, now_ms);
  if (Px4Lite_IsFresh(&s_display_remote_scratch.header, now_ms, APP_REMOTE_MAX_AGE_MS) == 0U) { return 0U; }

  memset(out, 0, sizeof(*out));
  out->view_system_id = s_display_remote_scratch.system_id;
  out->view_node_id = s_display_remote_scratch.system_id;
  out->view_remote_id_valid = 1U;

  module_loaded = App_FillRemoteModuleViews(out, &s_display_remote_scratch);
  if (module_loaded != 0U) {
    out->system_valid = 1U;
    out->status_version = s_display_remote_scratch.header.sequence;
    out->system_ready = 1U;
  }

  if ((s_display_remote_scratch.valid_mask & PX4LITE_REMOTE_VALID_HEARTBEAT) != 0U) {
    out->system_valid = 1U;
    out->status_version = s_display_remote_scratch.header.sequence;
    out->system_ready = ((s_display_remote_scratch.stale_mask & PX4LITE_REMOTE_VALID_HEARTBEAT) == 0U) ? 1U : 0U;
    if ((s_display_remote_scratch.module_state_valid_mask & (1UL << (uint32_t)PX4LITE_MODULE_LORA)) == 0U) {
      out->lora.state = (((s_display_remote_scratch.stale_mask & PX4LITE_REMOTE_VALID_HEARTBEAT) == 0U) ? APP_VIEW_STATE_ONLINE : APP_VIEW_STATE_OFFLINE);
      out->lora.fault_code = 0U;
      out->lora.severity = 0U;
    }
  }

  if (((s_display_remote_scratch.navigation_valid_mask & ~s_display_remote_scratch.navigation_stale_mask) & PX4LITE_NAV_VALID_POSITION) != 0U) {
    out->navigation_valid = 1U;
    out->latitude_e7 = s_display_remote_scratch.latitude_e7; out->longitude_e7 = s_display_remote_scratch.longitude_e7;
    out->hdop_x100 = s_display_remote_scratch.hdop_x100; out->satellites_used = s_display_remote_scratch.satellites_used;
    out->gnss_fix_type = s_display_remote_scratch.gnss_fix_type;
  }
  if (((s_display_remote_scratch.navigation_valid_mask & ~s_display_remote_scratch.navigation_stale_mask) & PX4LITE_NAV_VALID_ALTITUDE) != 0U) { out->altitude_mm = s_display_remote_scratch.altitude_mm; }
  if (((s_display_remote_scratch.navigation_valid_mask & ~s_display_remote_scratch.navigation_stale_mask) & PX4LITE_NAV_VALID_VELOCITY) != 0U) {
    out->velocity_north_cms = s_display_remote_scratch.velocity_north_cms; out->velocity_east_cms = s_display_remote_scratch.velocity_east_cms;
  }
  if (((s_display_remote_scratch.navigation_valid_mask & ~s_display_remote_scratch.navigation_stale_mask) & PX4LITE_NAV_VALID_TIME) != 0U) {
    out->gnss_utc_sec = s_display_remote_scratch.gnss_utc_sec; out->gnss_utc_date = s_display_remote_scratch.gnss_utc_date;
    (void)App_FillDisplayTimeFromGnss(out, out->gnss_utc_date, out->gnss_utc_sec);
  }

  if (((s_display_remote_scratch.valid_mask & PX4LITE_REMOTE_VALID_ATTITUDE) != 0U) && ((s_display_remote_scratch.stale_mask & PX4LITE_REMOTE_VALID_ATTITUDE) == 0U)) {
    out->attitude_valid    = 1U;
    out->roll_deg100      = s_display_remote_scratch.roll_deg100;
    out->pitch_deg100     = s_display_remote_scratch.pitch_deg100;
    out->yaw_deg100       = s_display_remote_scratch.yaw_deg100;
  }

  if (((s_display_remote_scratch.environment_valid_mask & ~s_display_remote_scratch.environment_stale_mask) & PX4LITE_REMOTE_ENV_VALID_PRESSURE_TEMPERATURE) != 0U) {
    out->environment_valid = 1U; out->pressure_pa = s_display_remote_scratch.pressure_pa; out->temperature_c = s_display_remote_scratch.temperature_c;
  }
  if (((s_display_remote_scratch.environment_valid_mask & ~s_display_remote_scratch.environment_stale_mask) & PX4LITE_REMOTE_ENV_VALID_HUMIDITY) != 0U) {
    out->environment_valid = 1U; out->relative_humidity_pct = s_display_remote_scratch.relative_humidity_pct;
  }
  if (((s_display_remote_scratch.battery_valid_mask & ~s_display_remote_scratch.battery_stale_mask) & PX4LITE_REMOTE_BATTERY_VALID_VOLTAGE1) != 0U) {
    out->environment_valid = 1U; out->voltage_mv = s_display_remote_scratch.voltage_mv;
    out->low_voltage = s_display_remote_scratch.low_voltage;
  }
  if (((s_display_remote_scratch.battery_valid_mask & ~s_display_remote_scratch.battery_stale_mask) & PX4LITE_REMOTE_BATTERY_VALID_CURRENT1) != 0U) {
    out->environment_valid = 1U; out->current_ma = s_display_remote_scratch.current_ma;
  }
  if (((s_display_remote_scratch.battery_valid_mask & ~s_display_remote_scratch.battery_stale_mask) & PX4LITE_REMOTE_BATTERY_VALID_PERCENT1) != 0U) {
    out->environment_valid = 1U; out->battery_percent = s_display_remote_scratch.battery_percent;
  }
  if (((s_display_remote_scratch.battery_valid_mask & ~s_display_remote_scratch.battery_stale_mask) & PX4LITE_REMOTE_BATTERY_VALID_VOLTAGE2) != 0U) {
    out->environment_valid = 1U; out->voltage2_mv = s_display_remote_scratch.voltage2_mv;
    out->low_voltage2 = s_display_remote_scratch.low_voltage2;
  }
  if (((s_display_remote_scratch.battery_valid_mask & ~s_display_remote_scratch.battery_stale_mask) & PX4LITE_REMOTE_BATTERY_VALID_CURRENT2) != 0U) {
    out->environment_valid = 1U; out->current2_ma = s_display_remote_scratch.current2_ma;
  }
  if (((s_display_remote_scratch.battery_valid_mask & ~s_display_remote_scratch.battery_stale_mask) & PX4LITE_REMOTE_BATTERY_VALID_PERCENT2) != 0U) {
    out->environment_valid = 1U; out->battery2_percent = s_display_remote_scratch.battery2_percent;
  }
  if (((s_display_remote_scratch.valid_mask & PX4LITE_REMOTE_VALID_ALARM) != 0U) && ((s_display_remote_scratch.stale_mask & PX4LITE_REMOTE_VALID_ALARM) == 0U)) {
    uint16_t source;
    uint16_t row = 0U;

    out->alarm_valid               = 1U;
    out->highest_fault_code        = s_display_remote_scratch.highest_fault_code;
    out->highest_source_id         = s_display_remote_scratch.highest_source_id;
    out->warning_fault_mask        = s_display_remote_scratch.alarm_active_mask;
    out->alarm_active_count        = App_CountBits32(s_display_remote_scratch.alarm_active_mask);
    out->alarm_highest_fault_code  = s_display_remote_scratch.highest_fault_code;
    for (source = 0U; (source < (uint16_t)PX4LITE_MODULE_COUNT) && (row < (uint16_t)APP_DISPLAY_ALARM_MAX); ++source) {
      if ((s_display_remote_scratch.alarm_active_mask & (1UL << source)) == 0U) { continue; }
      if (s_display_remote_scratch.alarm_fault_code[source] == 0U) { continue; }
      out->alarms[row].source_id  = source;
      out->alarms[row].fault_code = s_display_remote_scratch.alarm_fault_code[source];
      out->alarms[row].severity   = s_display_remote_scratch.alarm_severity[source];
      out->alarms[row].active     = 1U;
      out->alarms[row].updated_ms = s_display_remote_scratch.alarm_update_ms;
      out->alarms[row].raised_ms  = s_display_remote_scratch.alarm_update_ms;
      row++;
    }
    if ((row == 0U) && (s_display_remote_scratch.highest_fault_code != 0U)) {
      out->alarms[0].source_id  = s_display_remote_scratch.highest_source_id;
      out->alarms[0].fault_code = s_display_remote_scratch.highest_fault_code;
      out->alarms[0].severity   = s_display_remote_scratch.highest_severity;
      out->alarms[0].active     = 1U;
      out->alarms[0].updated_ms = s_display_remote_scratch.alarm_update_ms;
      out->alarms[0].raised_ms  = s_display_remote_scratch.alarm_update_ms;
    }
  }

  if (((s_display_remote_scratch.valid_mask & PX4LITE_REMOTE_VALID_MOTOR) != 0U) && ((s_display_remote_scratch.stale_mask & PX4LITE_REMOTE_VALID_MOTOR) == 0U)) {
    uint8_t i;

    out->motor_valid = 1U;
    for (i = 0U; (i < APP_DISPLAY_MOTOR_COUNT) && (i < PX4LITE_MOTOR_COUNT); i++) {
      out->motor_duty_percent[i] = s_display_remote_scratch.motor_duty_percent[i];
    }
    out->motor_run_state   = s_display_remote_scratch.motor_run_state;
    out->motor_speed_level = s_display_remote_scratch.motor_speed_level;
  }

  App_GetCommStats(&comm);
  out->lora_tx_count = comm.tx_frame_count;
  out->lora_rx_count = s_display_remote_scratch.rx_frame_count;
  out->lora_lost_count = s_display_remote_scratch.rx_sequence_lost_count;
  out->lora_ack_count = comm.mav_command_ack_rx_count + comm.mav_command_ack_tx_count;
  out->lora_loss_rate_x10 = s_display_remote_scratch.rx_loss_rate_x10;

  if (((s_display_remote_scratch.valid_mask & PX4LITE_REMOTE_VALID_MODULES) != 0U) && (App_ViewStateHasUsableData(out->battery.state) == 0U)) {
    out->battery_percent = 0U;
    out->battery2_percent = 0U;
    out->voltage_mv = 0U;
    out->voltage2_mv = 0U;
    out->current_ma = 0;
    out->current2_ma = 0;
  }

  out->any_valid = ((out->navigation_valid != 0U) || (out->attitude_valid != 0U) || (out->system_valid != 0U) || (out->environment_valid != 0U) || (out->alarm_valid != 0U) || (out->motor_valid != 0U) || ((s_display_remote_scratch.valid_mask & PX4LITE_REMOTE_VALID_HEARTBEAT) != 0U)) ? 1U : 0U;
  return out->any_valid;
}

void App_ResetLocalMessageLog(void)
{
  Px4Lite_LocalMsgLogReset();
}

void App_PushLocalMessageLog(uint16_t message_id, uint32_t time_hhmmss, int32_t value0, int32_t value1, int32_t value2, uint8_t severity)
{
  Px4Lite_LocalMsgLogPush(message_id, time_hhmmss, value0, value1, value2, severity);
}

uint16_t App_CopyLocalMessageLog(Px4Lite_LogEntry_t *entries, uint16_t capacity, uint32_t *version, uint16_t *last_seq)
{
  return Px4Lite_LocalMsgLogCopy(entries, capacity, version, last_seq);
}

/** @brief LoRa 日志同步退出，兼容消费者明确得到空记录。 */
uint16_t App_CopyRemoteMessageLog(Px4Lite_LogEntry_t *entries,uint16_t capacity,uint16_t *last_seq,uint32_t now_ms)
{ (void)entries; (void)capacity; (void)now_ms; if(last_seq) { *last_seq=0U; } return 0U; }
/** @brief 没有远端日志缓存或历史版本。 */
uint32_t App_GetRemoteMessageLogVersion(uint32_t now_ms) { (void)now_ms; return 0U; }

/** @brief 旧全局远端模式的兼容入口；启用请求明确拒绝，不改变发送调度。 */
uint8_t App_SetRemoteViewEnabled(uint8_t enabled, uint32_t now_ms)
{
  (void)now_ms;
  return (enabled == 0U) ? 1U : 0U;
}

/** @brief 只选择通信诊断对象；不改变主页面或收发状态。 */
uint8_t App_SelectRemoteNode(uint8_t node_id, uint32_t now_ms)
{
  (void)now_ms;
  if (node_id == Px4Lite_IdentityGetNodeId()) { return 0U; }
  return (Px4Lite_SelectRemoteNode(node_id) == PX4LITE_OK) ? 1U : 0U;
}

/** @brief 旧查看租约已退出；无全局租约可过期，远端数据由显式快照判龄。 */
uint8_t App_RemoteViewExpired(uint32_t now_ms)
{
  (void)now_ms;
  return 0U;
}

uint8_t App_GetSelectedRemoteNode(uint8_t *node_id)
{
  return (Px4Lite_GetSelectedRemoteNode(node_id) == PX4LITE_OK) ? 1U : 0U;
}

uint8_t App_GetLocalNodeId(void)
{
  return (uint8_t)Px4Lite_IdentityGetNodeId();
}

static App_RemoteNodeState_t App_MapRemoteNodeState(Px4Lite_RemoteNodeState_t state)
{
  switch (state) {
    case PX4LITE_REMOTE_NODE_DISCOVERED:
      return APP_REMOTE_NODE_DISCOVERED;
    case PX4LITE_REMOTE_NODE_ACTIVE:
      return APP_REMOTE_NODE_ACTIVE;
    case PX4LITE_REMOTE_NODE_STALE:
      return APP_REMOTE_NODE_STALE;
    case PX4LITE_REMOTE_NODE_EMPTY:
    default:
      break;
  }
  return APP_REMOTE_NODE_EMPTY;
}

static Px4Lite_Result_t App_CopyRemoteNodeStatusesImpl(App_RemoteNodeView_t *out, uint8_t capacity, uint8_t *count, uint32_t now_ms)
{
  uint8_t raw_count = 0U;
  uint8_t i;
  uint8_t written = 0U;
  Px4Lite_Result_t result;

  if ((out == 0) || (count == 0)) { return PX4LITE_INVALID_PARAM; }
  *count = 0U;

  result = Px4Lite_CopyRemoteNodeStatuses(s_remote_node_status_scratch, PX4LITE_REMOTE_NODE_MAX, &raw_count, now_ms);
  if ((result != PX4LITE_OK) && (result != PX4LITE_NOT_READY)) { return result; }

  for (i = 0U; (i < raw_count) && (written < capacity); i++) {

    out[written].node_id = s_remote_node_status_scratch[i].node_id;
    out[written].system_id = s_remote_node_status_scratch[i].system_id;
    out[written].heartbeat_type = s_remote_node_status_scratch[i].heartbeat_type;
    out[written].heartbeat_system_status = s_remote_node_status_scratch[i].heartbeat_system_status;
    out[written].state = App_MapRemoteNodeState(s_remote_node_status_scratch[i].state);
    out[written].last_heartbeat_ms = s_remote_node_status_scratch[i].last_heartbeat_ms;
    out[written].last_data_ms = s_remote_node_status_scratch[i].last_data_ms;
    out[written].rx_frame_count = s_remote_node_status_scratch[i].rx_frame_count;
    out[written].rx_sequence_lost_count = s_remote_node_status_scratch[i].rx_sequence_lost_count;
    out[written].rx_loss_rate_x10 = s_remote_node_status_scratch[i].rx_loss_rate_x10;
    out[written].reserved = 0U;
    out[written].identity_known=s_remote_node_status_scratch[i].identity_known;
    out[written].heartbeat_known=s_remote_node_status_scratch[i].heartbeat_known;
    out[written].id_conflict=s_remote_node_status_scratch[i].id_conflict;
    memcpy(out[written].product_id,s_remote_node_status_scratch[i].product_id,21U);
    written++;
  }

  *count = written;
  return (written != 0U) ? PX4LITE_OK : PX4LITE_NOT_READY;
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
    case PX4LITE_FAULT_MOTOR_DISCONNECT:
    case PX4LITE_FAULT_MOTOR_POWER_LOW:
    case PX4LITE_FAULT_MOTOR_DEAD:
    case PX4LITE_FAULT_MOTOR_CHARGE:
      return "MOTOR";

    case PX4LITE_FAULT_POWER_CHARGE:
      return "POWER";

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
    case PX4LITE_FAULT_DISPLAY_TOUCH:
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
    case PX4LITE_FAULT_DISPLAY_TOUCH:
      return "TOUCH OFFLINE";
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
    case PX4LITE_FAULT_MOTOR_DISCONNECT:
      return "MOTOR POWER ABSENT";
    case PX4LITE_FAULT_MOTOR_POWER_LOW:
      return "MOTOR POWER LOW";
    case PX4LITE_FAULT_MOTOR_DEAD:
      return "MOTOR BATTERY CRITICAL";
    case PX4LITE_FAULT_MOTOR_CHARGE:
      return "MOTOR BATTERY LOW";
    case PX4LITE_FAULT_POWER_CHARGE:
      return "MAIN BATTERY LOW";
    default:
      break;
  }

  return "UNKNOWN ALARM";
}

Px4Lite_Result_t App_RequestAttitudeLevelCalibration(void)
{
  return App_RequestAttitudeAction(PX4LITE_ATTITUDE_ACTION_ZERO);
}

/** @brief 显式操作对象为本机，远端只读模式不下发本机姿态操作。 */
Px4Lite_Result_t App_RequestAttitudeAction(Px4Lite_AttitudeAction_t action)
{
  if (App_GetRemoteDisplayMode() == PX4LITE_REMOTE_MODE_REMOTE) { return PX4LITE_NOT_READY; }
  return Px4Lite_RequestAttitudeAction(action);
}

/** @brief 经应用边界查询操作结果，OK 不代表校准成功，须读取 phase。 */
Px4Lite_Result_t App_CopyAttitudeStatus(App_AttitudeStatus_t *out)
{
  return Px4Lite_CopyAttitudeStatus(out);
}

Px4Lite_RemoteMode_t App_GetRemoteDisplayMode(void)
{
  return PX4LITE_REMOTE_MODE_LOCAL;
}

/** @brief UI 异步请求必须通过应用边界。 */
Px4Lite_Result_t App_RequestLoraLab(const Px4Lite_LabRequest_t *request) { return Px4Lite_LoraLabRequest(request); }
/** @brief 应用只复制已发布事实。 */
void App_CopyLoraLab(Px4Lite_LabSnapshot_t *out) { Px4Lite_LoraLabCopy(out); }
/** @brief 通过应用边界取得本机长期身份；长度由调用方指定。 */
void App_FormatLocalProductId(char *out,uint8_t capacity) { Px4Lite_IdentityFormatVendorProductId(out,capacity); }
/** @brief 单对端数据与发现身份分开读取，身份失败时不展示旧负载。 */
static void App_CopyRemoteLabViewImpl(App_RemoteLabView_t *out,uint32_t now_ms)
{
  Px4Lite_RemoteNodeStatus_t selected;
  uint32_t mask;
  Px4Lite_RemoteTelemetry_t *r=&s_display_remote_scratch;
  if(!out) { return; } memset(out,0,sizeof(*out));
  out->heartbeat_age_ms=UINT32_MAX; out->data_age_ms=UINT32_MAX;
  out->gnss_age_ms=UINT32_MAX; out->attitude_age_ms=UINT32_MAX;
  out->power1_age_ms=UINT32_MAX; out->power2_age_ms=UINT32_MAX;
  out->environment_age_ms=UINT32_MAX; out->humidity_age_ms=UINT32_MAX;
  (void)Px4Lite_GetSelectedRemoteNode(&out->node_id);
  if(Px4Lite_CopySelectedRemoteStatus(&selected,now_ms)==PX4LITE_OK) {
    out->identity_known=selected.identity_known; out->id_conflict=selected.id_conflict;
    memcpy(out->product_id,selected.product_id,21U);
    if(selected.heartbeat_known) { out->heartbeat_age_ms=Px4Lite_ElapsedMs(now_ms,selected.last_heartbeat_ms); }
  }
  if(Px4Lite_CopyRemoteTelemetry(r)!=PX4LITE_OK || r->system_id!=out->node_id || out->id_conflict || !out->identity_known) { return; }
  out->available=1U;
  out->rx_frames=r->rx_frame_count; out->loss_rate_x10=r->rx_loss_rate_x10;
  mask=r->navigation_valid_mask&~r->navigation_stale_mask;
  if(mask&PX4LITE_NAV_VALID_POSITION) { out->valid_bits|=APP_LAB_POSITION; out->latitude_e7=r->latitude_e7; out->longitude_e7=r->longitude_e7; out->gnss_fix_type=r->gnss_fix_type; out->satellites_used=r->satellites_used; }
  if(mask&PX4LITE_NAV_VALID_ALTITUDE) { out->valid_bits|=APP_LAB_ALTITUDE; out->altitude_mm=r->altitude_mm; }
  if(mask&PX4LITE_NAV_VALID_VELOCITY) { out->valid_bits|=APP_LAB_VELOCITY; out->velocity_north_cms=r->velocity_north_cms; out->velocity_east_cms=r->velocity_east_cms; }
  if((r->valid_mask&PX4LITE_REMOTE_VALID_ATTITUDE) && !(r->stale_mask&PX4LITE_REMOTE_VALID_ATTITUDE)) {
    out->valid_bits|=APP_LAB_ATTITUDE; out->roll_deg100=r->roll_deg100; out->pitch_deg100=r->pitch_deg100; out->yaw_deg100=r->yaw_deg100;
    out->roll_rate_dps100=r->roll_rate_dps100; out->pitch_rate_dps100=r->pitch_rate_dps100; out->yaw_rate_dps100=r->yaw_rate_dps100;
  }
  mask=r->battery_valid_mask&~r->battery_stale_mask;
  if(mask&PX4LITE_REMOTE_BATTERY_VALID_VOLTAGE1) { out->valid_bits|=APP_LAB_VOLTAGE1; out->voltage_mv=r->voltage_mv; }
  if(mask&PX4LITE_REMOTE_BATTERY_VALID_VOLTAGE2) { out->valid_bits|=APP_LAB_VOLTAGE2; out->voltage2_mv=r->voltage2_mv; }
  if(mask&PX4LITE_REMOTE_BATTERY_VALID_CURRENT1) { out->valid_bits|=APP_LAB_CURRENT1; out->current_ma=r->current_ma; }
  if(mask&PX4LITE_REMOTE_BATTERY_VALID_CURRENT2) { out->valid_bits|=APP_LAB_CURRENT2; out->current2_ma=r->current2_ma; }
  if(mask&PX4LITE_REMOTE_BATTERY_VALID_PERCENT1) { out->valid_bits|=APP_LAB_PERCENT1; out->battery_percent=r->battery_percent; }
  if(mask&PX4LITE_REMOTE_BATTERY_VALID_PERCENT2) { out->valid_bits|=APP_LAB_PERCENT2; out->battery2_percent=r->battery2_percent; }
  mask=r->environment_valid_mask&~r->environment_stale_mask;
  if(mask&PX4LITE_REMOTE_ENV_VALID_PRESSURE_TEMPERATURE) { out->valid_bits|=APP_LAB_ENVIRONMENT; out->pressure_pa=(uint32_t)r->pressure_pa; out->temperature_deg100=(int32_t)(r->temperature_c*100.0f); }
  if(mask&PX4LITE_REMOTE_ENV_VALID_HUMIDITY) { out->valid_bits|=APP_LAB_HUMIDITY; out->humidity_pct100=(uint32_t)(r->relative_humidity_pct*100.0f); }
  if(r->navigation_valid_mask&PX4LITE_NAV_VALID_POSITION) { out->gnss_age_ms=Px4Lite_ElapsedMs(now_ms,r->position_update_ms); }
  if(r->valid_mask&PX4LITE_REMOTE_VALID_ATTITUDE) { out->attitude_age_ms=Px4Lite_ElapsedMs(now_ms,r->attitude_update_ms); }
  if(r->battery_valid_mask&PX4LITE_REMOTE_BATTERY_VALID_PRIMARY) { out->power1_age_ms=Px4Lite_ElapsedMs(now_ms,r->battery1_update_ms); }
  if(r->battery_valid_mask&PX4LITE_REMOTE_BATTERY_VALID_SECONDARY) { out->power2_age_ms=Px4Lite_ElapsedMs(now_ms,r->battery2_update_ms); }
  if(r->environment_valid_mask&PX4LITE_REMOTE_ENV_VALID_PRESSURE_TEMPERATURE) { out->environment_age_ms=Px4Lite_ElapsedMs(now_ms,r->pressure_temperature_update_ms); }
  if(r->environment_valid_mask&PX4LITE_REMOTE_ENV_VALID_HUMIDITY) { out->humidity_age_ms=Px4Lite_ElapsedMs(now_ms,r->humidity_update_ms); }
  /* 数据年龄来自真实实验域，心跳不能刷新主数据年龄。 */
  if(r->navigation_valid_mask&PX4LITE_NAV_VALID_POSITION) { out->data_age_ms=Px4Lite_ElapsedMs(now_ms,r->position_update_ms); }
  if((r->valid_mask&PX4LITE_REMOTE_VALID_ATTITUDE) && Px4Lite_ElapsedMs(now_ms,r->attitude_update_ms)<out->data_age_ms) { out->data_age_ms=Px4Lite_ElapsedMs(now_ms,r->attitude_update_ms); }
  if((r->battery_valid_mask&PX4LITE_REMOTE_BATTERY_VALID_VOLTAGE1) && Px4Lite_ElapsedMs(now_ms,r->battery1_update_ms)<out->data_age_ms) { out->data_age_ms=Px4Lite_ElapsedMs(now_ms,r->battery1_update_ms); }
  if((r->battery_valid_mask&PX4LITE_REMOTE_BATTERY_VALID_VOLTAGE2) && Px4Lite_ElapsedMs(now_ms,r->battery2_update_ms)<out->data_age_ms) { out->data_age_ms=Px4Lite_ElapsedMs(now_ms,r->battery2_update_ms); }
  if((r->environment_valid_mask&PX4LITE_REMOTE_ENV_VALID_PRESSURE_TEMPERATURE) && Px4Lite_ElapsedMs(now_ms,r->pressure_temperature_update_ms)<out->data_age_ms) { out->data_age_ms=Px4Lite_ElapsedMs(now_ms,r->pressure_temperature_update_ms); }
  if((r->environment_valid_mask&PX4LITE_REMOTE_ENV_VALID_HUMIDITY) && Px4Lite_ElapsedMs(now_ms,r->humidity_update_ms)<out->data_age_ms) { out->data_age_ms=Px4Lite_ElapsedMs(now_ms,r->humidity_update_ms); }
}

/** @brief 任务之间串行使用静态转换 scratch；计算不关闭中断，不含阻塞 I/O。 */
uint8_t App_CopyRemoteDisplaySnapshot(App_DisplaySnapshot_t *out,uint32_t now_ms)
{ uint8_t result; vTaskSuspendAll(); result=App_CopyRemoteDisplaySnapshotImpl(out,now_ms); (void)xTaskResumeAll(); return result; }

/** @brief 任务之间串行使用静态转换 scratch；计算不关闭中断，不含阻塞 I/O。 */
Px4Lite_Result_t App_CopyRemoteNodeStatuses(App_RemoteNodeView_t *out,uint8_t capacity,uint8_t *count,uint32_t now_ms)
{ Px4Lite_Result_t result; vTaskSuspendAll(); result=App_CopyRemoteNodeStatusesImpl(out,capacity,count,now_ms); (void)xTaskResumeAll(); return result; }

/** @brief 任务之间串行使用静态转换 scratch；计算不关闭中断，不含阻塞 I/O。 */
void App_CopyRemoteLabView(App_RemoteLabView_t *out,uint32_t now_ms)
{ vTaskSuspendAll(); App_CopyRemoteLabViewImpl(out,now_ms); (void)xTaskResumeAll(); }
