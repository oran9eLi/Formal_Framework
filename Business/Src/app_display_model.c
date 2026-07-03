/**
 * @file app_display_model.c
 * @brief 显示层统一取数接口实现：按模式选源并把远端快照归一到 App_*Snapshot_t。
 *
 * @details
 * 本模块是 LOCAL/REMOTE 模式解析与远端→本机结构体归一的**唯一一处**。显示层(老显示、
 * LVGL)只调用 `App_GetDisplay*`，不感知数据来源。归一字段映射与原 `display.c` 内的
 * `Display_LoadRemote*` 保持一致，行为不变。
 *
 * 依赖边界：仅依赖 `app_data_api.h`(本机 `App_Copy*` 与远端 `App_CopyRemoteTelemetry`)，
 * 不包含 Display/LVGL/BSP/Sensor 头文件。
 */

#include "app_display_model.h"

#include <string.h>

#include "app_message_log.h"

/**
 * @brief 把远端某显示域的字段有效/过期位映射为统一新鲜度返回值。
 *
 * @param[in] valid_mask 远端快照 valid_mask(曾收到的字段)。
 * @param[in] stale_mask 远端快照 stale_mask(当前已过期的字段)。
 * @param[in] domain_bits 该显示域对应的一个或多个 `PX4LITE_REMOTE_VALID_*` 位。
 *
 * @return 新鲜度：从未收到 `NOT_READY`；至少一个有效位未过期 `OK`；全部过期 `STALE`。
 */
static Px4Lite_Result_t AppDisplay_RemoteFreshness(uint32_t valid_mask, uint32_t stale_mask, uint32_t domain_bits)
{
  uint32_t valid = valid_mask & domain_bits;

  if (valid == 0U) { return PX4LITE_NOT_READY; }
  return ((valid & ~stale_mask) != 0U) ? PX4LITE_OK : PX4LITE_STALE;
}

/**
 * @brief 取一份远端快照并判断目标显示域新鲜度；非 REMOTE 或未收到时给出指示。
 *
 * @return `PX4LITE_OK`/`PX4LITE_STALE` 时 `remote` 已填好且该域曾收到；否则返回
 *         `NOT_READY`，调用方应清零输出。
 */
static Px4Lite_Result_t AppDisplay_RemoteDomain(Px4Lite_RemoteTelemetrySnapshot_t *remote, uint32_t domain_bits, uint32_t now_ms)
{
  if (App_CopyRemoteTelemetry(remote, now_ms) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  return AppDisplay_RemoteFreshness(remote->valid_mask, remote->stale_mask, domain_bits);
}

Px4Lite_Result_t App_GetDisplayNavigation(App_NavigationSnapshot_t *out, uint32_t now_ms)
{
  Px4Lite_RemoteTelemetrySnapshot_t remote;
  Px4Lite_Result_t fresh;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  if (App_GetRemoteDisplayMode() != PX4LITE_REMOTE_MODE_REMOTE) { return App_CopyNavigation(out, now_ms); }

  memset(out, 0, sizeof(*out));
  fresh = AppDisplay_RemoteDomain(&remote, PX4LITE_REMOTE_VALID_NAVIGATION | PX4LITE_REMOTE_VALID_ATTITUDE, now_ms);
  if (fresh == PX4LITE_NOT_READY) { return PX4LITE_NOT_READY; }

  out->header             = remote.header;
  out->latitude_e7        = remote.latitude_e7;
  out->longitude_e7       = remote.longitude_e7;
  out->altitude_mm        = remote.altitude_mm;
  out->velocity_north_cms = remote.velocity_north_cms;
  out->velocity_east_cms  = remote.velocity_east_cms;
  out->velocity_down_cms  = remote.velocity_down_cms;
  out->roll_deg100        = remote.roll_deg100;
  out->pitch_deg100       = remote.pitch_deg100;
  out->yaw_deg100         = remote.yaw_deg100;
  out->roll_rate_dps100   = remote.roll_rate_dps100;
  out->pitch_rate_dps100  = remote.pitch_rate_dps100;
  out->yaw_rate_dps100    = remote.yaw_rate_dps100;
  out->hdop_x100          = remote.hdop_x100;
  out->satellites_used    = remote.satellites_used;
  out->gnss_fix_type      = remote.gnss_fix_type;
  out->navigation_quality = 100U;
  if ((remote.valid_mask & PX4LITE_REMOTE_VALID_NAVIGATION) != 0U) {
    out->valid_mask |= PX4LITE_NAV_VALID_POSITION | PX4LITE_NAV_VALID_VELOCITY | PX4LITE_NAV_VALID_ALTITUDE;
  }
  if ((remote.valid_mask & PX4LITE_REMOTE_VALID_ATTITUDE) != 0U) { out->valid_mask |= PX4LITE_NAV_VALID_ATTITUDE; }
  return fresh;
}

Px4Lite_Result_t App_GetDisplayEnvironment(App_EnvironmentSnapshot_t *out, uint32_t now_ms)
{
  Px4Lite_RemoteTelemetrySnapshot_t remote;
  Px4Lite_Result_t fresh;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  if (App_GetRemoteDisplayMode() != PX4LITE_REMOTE_MODE_REMOTE) { return App_CopyEnvironment(out, now_ms); }

  memset(out, 0, sizeof(*out));
  fresh = AppDisplay_RemoteDomain(&remote, PX4LITE_REMOTE_VALID_ENVIRONMENT | PX4LITE_REMOTE_VALID_POWER, now_ms);
  if (fresh == PX4LITE_NOT_READY) { return PX4LITE_NOT_READY; }

  out->header                = remote.header;
  out->pressure_pa           = remote.pressure_pa;
  out->temperature_c         = remote.temperature_c;
  out->relative_humidity_pct = remote.relative_humidity_pct;
  out->voltage_mv            = remote.voltage_mv;
  out->battery_percent       = remote.battery_percent;
  return fresh;
}

Px4Lite_Result_t App_GetDisplaySystem(App_SystemSnapshot_t *out, uint32_t now_ms)
{
  Px4Lite_RemoteTelemetrySnapshot_t remote;
  Px4Lite_Result_t fresh;
  uint32_t i;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  if (App_GetRemoteDisplayMode() != PX4LITE_REMOTE_MODE_REMOTE) { return App_CopySystem(out, now_ms); }

  memset(out, 0, sizeof(*out));
  fresh = AppDisplay_RemoteDomain(&remote, PX4LITE_REMOTE_VALID_MODULES, now_ms);
  if (fresh == PX4LITE_NOT_READY) { return PX4LITE_NOT_READY; }

  out->header = remote.header;
  for (i = 0U; i < (uint32_t)PX4LITE_MODULE_COUNT; ++i) {
    out->modules[i].module_id = (Px4Lite_ModuleId_t)i;
    out->modules[i].state     = (Px4Lite_State_t)remote.module_state[i];
  }
  {
    Px4Lite_ModuleStatus_t lora_status;
    if (App_GetModuleStatus(PX4LITE_MODULE_LORA, &lora_status) == PX4LITE_OK) {
      out->modules[PX4LITE_MODULE_LORA] = lora_status;
    }
  }
  out->system_ready      = remote.system_ready;
  out->highest_severity  = remote.highest_severity;
  out->highest_fault_code = remote.highest_fault_code;
  out->highest_source_id = remote.highest_source_id;
  out->active_alarm_count = (remote.highest_fault_code != 0U) ? 1U : 0U;
  return fresh;
}

Px4Lite_Result_t App_GetDisplayAlarm(App_AlarmSnapshot_t *out, uint32_t now_ms)
{
  Px4Lite_RemoteTelemetrySnapshot_t remote;
  Px4Lite_Result_t fresh;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  if (App_GetRemoteDisplayMode() != PX4LITE_REMOTE_MODE_REMOTE) { return App_CopyAlarm(out, now_ms); }

  memset(out, 0, sizeof(*out));
  fresh = AppDisplay_RemoteDomain(&remote, PX4LITE_REMOTE_VALID_ALARM, now_ms);
  if (fresh == PX4LITE_NOT_READY) { return PX4LITE_NOT_READY; }

  out->header             = remote.header;
  out->highest_fault_code = remote.highest_fault_code;
  out->highest_source_id  = remote.highest_source_id;
  out->highest_severity   = remote.highest_severity;
  if (remote.alarm_table_count > 0U) {
    /* 完整告警表：用远端逐行填满 records(TUNNEL 同步)。 */
    uint8_t i;
    uint8_t n = remote.alarm_table_count;
    if (n > (uint8_t)PX4LITE_MODULE_COUNT) { n = (uint8_t)PX4LITE_MODULE_COUNT; }
    out->active_count = n;
    for (i = 0U; i < n; ++i) {
      out->records[i].source_id  = remote.alarm_records[i].source_id;
      out->records[i].fault_code = remote.alarm_records[i].fault_code;
      out->records[i].severity   = (uint8_t)remote.alarm_records[i].severity;
      out->records[i].active     = remote.alarm_records[i].active;
      out->records[i].raised_ms  = remote.alarm_records[i].raised_ms;
      out->records[i].updated_ms = now_ms;
      out->records[i].detail     = remote.alarm_records[i].detail;
    }
  } else if (remote.highest_fault_code != 0U) {
    /* 仅收到摘要(整表未到/无活动行)：退回最高项单行。 */
    out->active_count          = 1U;
    out->records[0].source_id  = remote.highest_source_id;
    out->records[0].fault_code = remote.highest_fault_code;
    out->records[0].severity   = remote.highest_severity;
    out->records[0].active     = 1U;
  }
  return fresh;
}

Px4Lite_Result_t App_GetDisplayMotor(App_MotorSnapshot_t *out, uint32_t now_ms)
{
  Px4Lite_RemoteTelemetrySnapshot_t remote;
  Px4Lite_Result_t fresh;
  uint8_t i;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  if (App_GetRemoteDisplayMode() != PX4LITE_REMOTE_MODE_REMOTE) { return App_CopyMotor(out, now_ms); }

  memset(out, 0, sizeof(*out));
  fresh = AppDisplay_RemoteDomain(&remote, PX4LITE_REMOTE_VALID_MOTOR, now_ms);
  if (fresh == PX4LITE_NOT_READY) { return PX4LITE_NOT_READY; }

  out->header      = remote.header;
  out->run_state   = remote.motor_run_state;
  out->speed_level = remote.motor_speed_level;
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; ++i) { out->duty_percent[i] = remote.motor_duty_percent[i]; }
  return fresh;
}

Px4Lite_Result_t App_GetDisplayDateTime(App_DateTimeSnapshot_t *out, uint32_t now_ms)
{
  Px4Lite_RemoteTelemetrySnapshot_t remote;
  Px4Lite_Result_t fresh;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  if (App_GetRemoteDisplayMode() != PX4LITE_REMOTE_MODE_REMOTE) { return App_CopyDateTime(out, now_ms); }

  memset(out, 0, sizeof(*out));
  fresh = AppDisplay_RemoteDomain(&remote, PX4LITE_REMOTE_VALID_TIME, now_ms);
  if (fresh == PX4LITE_NOT_READY) { return PX4LITE_NOT_READY; }

  /* 远端只提供本地显示时间/日期；UTC 字段与校时状态本机未知，保持 0/默认。 */
  out->header             = remote.header;
  out->local_time_hhmmss  = remote.time_hhmmss;
  out->local_date_ymd     = remote.date_ymd;
  return fresh;
}

Px4Lite_Result_t App_GetDisplayMessageLog(App_DisplayLogSnapshot_t *out, uint32_t now_ms)
{
  Px4Lite_RemoteTelemetrySnapshot_t remote;
  Px4Lite_Result_t fresh;
  uint16_t i, n;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }

  /* LOCAL：返回本机业务日志缓冲(app_message_log 生产，与旧显示内部日志同源同算)。 */
  if (App_GetRemoteDisplayMode() != PX4LITE_REMOTE_MODE_REMOTE) {
    return App_MessageLogCopy(out);
  }

  /* REMOTE：返回远端同步日志(version 取 log_last_seq)；断链由新鲜度判 STALE 保留旧值。 */
  memset(out, 0, sizeof(*out));
  fresh = AppDisplay_RemoteDomain(&remote, PX4LITE_REMOTE_VALID_LOG, now_ms);
  if (fresh == PX4LITE_NOT_READY) { return PX4LITE_NOT_READY; }

  n = remote.log_count;
  if (n > (uint16_t)APP_DISPLAY_LOG_CAP) { n = (uint16_t)APP_DISPLAY_LOG_CAP; }
  out->version = remote.log_last_seq;
  out->count   = n;
  for (i = 0U; i < n; ++i) {
    out->entries[i].sequence    = remote.log_entries[i].sequence;
    out->entries[i].message_id  = remote.log_entries[i].message_id;
    out->entries[i].time_hhmmss = remote.log_entries[i].time_hhmmss;
    out->entries[i].fault_code  = remote.log_entries[i].fault_code;
    out->entries[i].severity    = remote.log_entries[i].severity;
    out->entries[i].source_id   = remote.log_entries[i].source_id;
    out->entries[i].active      = remote.log_entries[i].active;
  }
  return fresh;
}

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
  out->loss_permille  = (uint16_t)comm.rx_loss_permille;
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
