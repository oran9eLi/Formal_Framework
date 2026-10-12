/**
 * @file app_message_log.c
 * @brief Business 本机状态事件观察者；由采集任务每 200 ms 推进。
 * @details 仅通过 App API 读取快照；不依赖 LCD、冻结或选中节点。控制/姿态事件由所属
 * Framework 状态机直接记录。本层只确认持续 1500 ms 的状态，并保留首次候选 Uptime。
 */
#include "app_message_log.h"
#include "app_data_api.h"
#include "px4lite_local_msglog.h"
#include "px4lite_faults.h"
#include "px4lite_config.h"
#include <string.h>

#define APP_MSGLOG_STABLE_MS 1500U
#define APP_MSGLOG_MODULES 9U
#define APP_MSGLOG_NONE ((uint16_t)PX4LITE_LOGMSG_COUNT)
#define APP_MSGLOG_BAND_UNKNOWN 255U

/** @brief 状态候选和已确认事实；计时均为毫秒 Uptime。 */
typedef struct {
  uint32_t since_ms; /**< 当前候选首次观察时刻。 */
  uint32_t candidate_key; /**< 消息、故障和严重度组成的事实标识。 */
  uint32_t committed_key; /**< 已确认事实标识，防止重复刷新写日志。 */
  uint16_t committed; /**< 已确认消息 ID，NONE 表示未确认。 */
  uint16_t candidate; /**< 当前候选消息 ID。 */
  uint8_t candidate_source; /**< 当前候选来源模块。 */
  uint8_t committed_source; /**< 已确认来源模块。 */
  uint8_t recovery_pending; /**< 曾确认异常，等待正常恢复。 */
} AppLog_Debounce_t;
/** @brief 每个告警来源的候选与已确认事实；仅由采集任务维护，无动态内存。 */
typedef struct {
  uint32_t since_ms; /**< 候选首次连续观察时刻，单位 ms。 */
  uint32_t candidate_raised_ms; /**< 候选告警源记录的触发时刻，单位 ms。 */
  uint32_t committed_raised_ms; /**< 已确认告警触发时刻，区分同故障再次触发。 */
  uint16_t candidate_fault; /**< 候选故障码；无活动告警时为 0。 */
  uint16_t committed_fault; /**< 已确认故障码；清除事件保留此码。 */
  uint8_t candidate_severity; /**< 候选原始严重度，见 Px4Lite_AlarmSeverity_t。 */
  uint8_t committed_severity; /**< 已确认原始严重度。 */
  uint8_t candidate_active; /**< 候选是否活动，0 或 1。 */
  uint8_t committed_active; /**< 已确认是否活动，0 或 1。 */
  uint8_t candidate_valid; /**< 读取中断后必须重新连续观察候选。 */
} AppLog_AlarmState_t;
/** @brief 本机模块状态目录；每组连续对应公开状态 0~6、恢复和未知。 */
typedef struct {
  Px4Lite_ModuleId_t module; /**< App API 模块 ID。 */
  uint16_t base; /**< UNINITIALIZED 事件 ID。 */
} AppLog_Module_t;
static const AppLog_Module_t s_modules[APP_MSGLOG_MODULES] = {
  {PX4LITE_MODULE_GNSS, PX4LITE_LOGMSG_GNSS_UNINITIALIZED},
  {PX4LITE_MODULE_IMU, PX4LITE_LOGMSG_IMU_UNINITIALIZED},
  {PX4LITE_MODULE_BARO, PX4LITE_LOGMSG_BARO_UNINITIALIZED},
  {PX4LITE_MODULE_BATTERY, PX4LITE_LOGMSG_BATTERY_UNINITIALIZED},
  {PX4LITE_MODULE_LORA, PX4LITE_LOGMSG_LORA_UNINITIALIZED},
  {PX4LITE_MODULE_5G, PX4LITE_LOGMSG_NETWORK_UNINITIALIZED},
  {PX4LITE_MODULE_STORAGE, PX4LITE_LOGMSG_STORAGE_UNINITIALIZED},
  {PX4LITE_MODULE_REMOTE_ID, PX4LITE_LOGMSG_RID_UNINITIALIZED},
  {PX4LITE_MODULE_CONTROL, PX4LITE_LOGMSG_CONTROL_UNINITIALIZED}
};
/** @brief 启动检查仅覆盖这五项；禁用项不计入通过集合。 */
static const uint8_t s_checks[5] = {0U, 1U, 2U, 4U, 6U};
static AppLog_Debounce_t s_module_db[APP_MSGLOG_MODULES];
static AppLog_Debounce_t s_check_db, s_fix_db, s_motor_db, s_main_db, s_alarm_db;
static uint8_t s_initialized;
static uint8_t s_motor_band = APP_MSGLOG_BAND_UNKNOWN;
static uint8_t s_main_band = APP_MSGLOG_BAND_UNKNOWN;
static uint32_t s_check_signature;
static uint8_t s_check_signature_valid;
static uint32_t s_rid_errors, s_rid_error_ms;
static uint8_t s_rid_error_active;
static AppLog_AlarmState_t s_alarm_sources[PX4LITE_MODULE_COUNT];
/** @brief 大型只读快照暂存由唯一采集任务持有，避免扩大任务栈。 */
static App_AlarmSnapshot_t s_alarm;

/** @brief 按明确事实和实际严重度写入；计算时间在共享环临界区外完成。 */
static void AppLog_Push(uint16_t message, uint32_t time_ms, uint16_t fault, uint8_t source, uint8_t severity)
{
  uint32_t sec = time_ms / 1000U;
  uint32_t time = (((sec / 3600U) % 100U) * 10000U) + (((sec / 60U) % 60U) * 100U) + (sec % 60U);
  Px4Lite_LocalMsgLogPush(message, time, fault, severity, source, (severity != 0U) ? 1U : 0U);
}
/** @brief 初始化观察状态，不操作共享日志环。 */
static void AppLog_InitDebounce(AppLog_Debounce_t *db)
{
  memset(db, 0, sizeof(*db));
  db->candidate = APP_MSGLOG_NONE;
  db->committed = APP_MSGLOG_NONE;
}
/** @brief 候选稳定 1500 ms 后记录首次观察时刻；正常恢复使用单独事件 ID。 */
static void AppLog_Observe(AppLog_Debounce_t *db, uint16_t message, uint16_t normal,
                           uint16_t restored, uint16_t fault, uint8_t source, uint8_t severity, uint32_t now_ms)
{
  uint32_t key = ((uint32_t)message << 18U) | ((uint32_t)severity << 16U) | fault;
  uint16_t event = message;
  if ((db->candidate != message) || (db->candidate_key != key) || (db->candidate_source != source)) {
    db->candidate = message;
    db->candidate_key = key;
    db->candidate_source = source;
    db->since_ms = now_ms;
  }
  if (((db->committed == message) && (db->committed_key == key) && (db->committed_source == source)) ||
      ((uint32_t)(now_ms - db->since_ms) < APP_MSGLOG_STABLE_MS)) { return; }
  if ((message == normal) && (restored != APP_MSGLOG_NONE) && (db->recovery_pending != 0U)) { event = restored; }
  if (message == normal) { db->recovery_pending = 0U; }
  else if ((normal >= PX4LITE_LOGMSG_GNSS_ONLINE) && (normal <= PX4LITE_LOGMSG_CONTROL_ONLINE)) {
    /* 首次启动和禁用不构成故障；经过故障后的 STARTING 不抹掉恢复记忆。 */
    if ((message != normal - PX4LITE_STATE_ONLINE) && (message != normal - 1U) &&
        (message != normal + (PX4LITE_STATE_DISABLED - PX4LITE_STATE_ONLINE))) { db->recovery_pending = 1U; }
  } else { db->recovery_pending = 1U; }
  AppLog_Push(event, db->since_ms, fault, source, severity);
  db->committed = message;
  db->committed_key = key;
  db->committed_source = source;
}
/** @brief 仅输入有效电压时推进带 200 mV 滞回的档位；首次按实际电压分类。 */
static uint8_t AppLog_Band(uint8_t current, uint32_t mv, const uint32_t *limits, uint8_t count)
{
  uint8_t plain = 0U;
  while ((plain < count) && (mv >= limits[plain])) { ++plain; }
  if (current == APP_MSGLOG_BAND_UNKNOWN) { return plain; }
  if ((plain > current) && (mv >= limits[current] + 200U)) { return plain; }
  if ((plain < current) && (mv + 200U < limits[current - 1U])) { return plain; }
  return current;
}
/** @brief 分别观察主电与动力电压；失效不沿用旧电压，不推断电机机械状态。 */
static void AppLog_ObservePower(uint32_t now_ms)
{
  static const uint32_t motor_limits[4] = {5000U, 9000U, 9900U, 10500U};
  static const uint32_t main_limits[2] = {5000U, 10500U};
  static const uint16_t motor_messages[5] = {PX4LITE_LOGMSG_MOTOR_DISCONNECT, PX4LITE_LOGMSG_MOTOR_DEAD,
    PX4LITE_LOGMSG_MOTOR_LOWPOWER, PX4LITE_LOGMSG_MOTOR_CHARGE, PX4LITE_LOGMSG_MOTOR_OK};
  static const uint16_t main_messages[3] = {PX4LITE_LOGMSG_MAIN_ABSENT, PX4LITE_LOGMSG_MAIN_CHARGE, PX4LITE_LOGMSG_MAIN_NORMAL};
  App_EnvironmentSnapshot_t env;
  uint16_t motor = PX4LITE_LOGMSG_MOTOR_UNAVAILABLE, main = PX4LITE_LOGMSG_MAIN_UNAVAILABLE;
  if (App_CopyEnvironment(&env, now_ms) == PX4LITE_OK) {
    if ((env.valid_mask & APP_ENV_VALID_VOLTAGE2) != 0U) {
      if (env.voltage2_mv > 13500U) { s_motor_band = APP_MSGLOG_BAND_UNKNOWN; motor = PX4LITE_LOGMSG_MOTOR_DISCONNECT; }
      else { s_motor_band = AppLog_Band(s_motor_band, env.voltage2_mv, motor_limits, 4U); motor = motor_messages[s_motor_band]; }
    } else { s_motor_band = APP_MSGLOG_BAND_UNKNOWN; }
    if ((env.valid_mask & APP_ENV_VALID_VOLTAGE1) != 0U) {
      s_main_band = AppLog_Band(s_main_band, env.voltage_mv, main_limits, 2U); main = main_messages[s_main_band];
    } else { s_main_band = APP_MSGLOG_BAND_UNKNOWN; }
  } else { s_motor_band = APP_MSGLOG_BAND_UNKNOWN; s_main_band = APP_MSGLOG_BAND_UNKNOWN; }
  AppLog_Observe(&s_motor_db, motor, PX4LITE_LOGMSG_MOTOR_OK, PX4LITE_LOGMSG_MOTOR_RESTORED,
                 0U, PX4LITE_MODULE_BATTERY, Px4Lite_LogMessageSeverity(motor), now_ms);
  AppLog_Observe(&s_main_db, main, PX4LITE_LOGMSG_MAIN_NORMAL, PX4LITE_LOGMSG_MAIN_RESTORED,
                 0U, PX4LITE_MODULE_BATTERY, Px4Lite_LogMessageSeverity(main), now_ms);
}
/**
 * @brief 观察 RID 发送提交错误；成功提交不代表 ESP 已接收。
 * @details error_count 为本次上电累计值，热恢复不清零；模差覆盖 uint32_t 回绕。
 *          只依据计数变化记录新错误，不以模块状态变化推断统计周期。
 */
static void AppLog_ObserveRid(const Px4Lite_ModuleStatus_t *status, uint32_t now_ms)
{
  uint32_t delta = status->error_count - s_rid_errors;
  if (delta != 0U) {
    if (s_rid_error_active == 0U) {
      Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_RID_TX_ERROR, now_ms, status->fault_code, PX4LITE_MODULE_REMOTE_ID, 1U);
    }
    s_rid_error_ms = now_ms;
    s_rid_error_active = 1U;
  } else if ((s_rid_error_active != 0U) && (status->last_rx_ms != 0U) &&
             ((int32_t)(status->last_rx_ms - s_rid_error_ms) > 0)) {
    s_rid_error_active = 0U;
    Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_RID_TX_RESTORED, status->last_rx_ms, 0U, PX4LITE_MODULE_REMOTE_ID, 0U);
  }
  s_rid_errors = status->error_count;
}
/** @brief 显式写入告警活动位；INFO 告警也可活动，清除事件始终为 0。 */
static void AppLog_PushAlarm(uint16_t message, uint32_t time_ms, uint16_t fault,
                             uint8_t source, uint8_t severity, uint8_t active)
{
  uint32_t sec = time_ms / 1000U;
  uint32_t time = (((sec / 3600U) % 100U) * 10000U) + (((sec / 60U) % 60U) * 100U) + (sec % 60U);
  Px4Lite_LocalMsgLogPush(message, time, fault, (severity > 2U) ? 2U : severity, source, active);
}
/** @brief 按每个来源观察触发、清除及故障/严重度变化；不可用不会推进清除。 */
static void AppLog_ObserveAlarms(uint32_t now_ms)
{
  uint16_t i;
  uint8_t available = (uint8_t)(App_CopyAlarm(&s_alarm, now_ms) == PX4LITE_OK);
  uint16_t message = available ? PX4LITE_LOGMSG_ALARM_RESTORED : PX4LITE_LOGMSG_ALARM_UNAVAILABLE;
  AppLog_Observe(&s_alarm_db, message, PX4LITE_LOGMSG_ALARM_RESTORED, APP_MSGLOG_NONE,
                 0U, PX4LITE_MODULE_ALARM, Px4Lite_LogMessageSeverity(message), now_ms);
  for (i = 0U; i < PX4LITE_MODULE_COUNT; ++i) {
    AppLog_AlarmState_t *state = &s_alarm_sources[i];
    const App_AlarmRecord_t *record = &s_alarm.records[i];
    uint8_t active;
    uint16_t fault;
    uint8_t severity;
    uint32_t raised_ms;
    if (available == 0U) { state->candidate_valid = 0U; continue; }
    active = (uint8_t)(record->active != 0U);
    fault = active ? record->fault_code : 0U;
    severity = active ? record->severity : 0U;
    raised_ms = active ? record->raised_ms : 0U;
    if ((state->candidate_valid == 0U) || (state->candidate_active != active) ||
        (state->candidate_fault != fault) || (state->candidate_severity != severity) ||
        (state->candidate_raised_ms != raised_ms)) {
      state->candidate_valid = 1U;
      state->candidate_active = active; state->candidate_fault = fault;
      state->candidate_severity = severity; state->candidate_raised_ms = raised_ms;
      state->since_ms = now_ms;
    }
    if ((uint32_t)(now_ms - state->since_ms) < APP_MSGLOG_STABLE_MS) { continue; }
    if ((state->committed_active == active) && (state->committed_fault == fault) &&
        (state->committed_severity == severity) && (state->committed_raised_ms == raised_ms)) { continue; }
    if (state->committed_active && (!active || (state->committed_fault != fault) ||
                                    (state->committed_raised_ms != raised_ms))) {
      AppLog_PushAlarm(PX4LITE_LOGMSG_ALARM_CLEARED, state->since_ms, state->committed_fault, (uint8_t)i, 0U, 0U);
    }
    if (active) {
      uint8_t raised = (uint8_t)(!state->committed_active || (state->committed_fault != fault) ||
                                 (state->committed_raised_ms != raised_ms));
      AppLog_PushAlarm(raised ? PX4LITE_LOGMSG_ALARM_RAISED : PX4LITE_LOGMSG_ALARM_UPDATED,
                       raised ? raised_ms : state->since_ms, fault, (uint8_t)i, severity, 1U);
    }
    state->committed_active = active; state->committed_fault = fault;
    state->committed_severity = severity; state->committed_raised_ms = raised_ms;
  }
}
/** @brief 启动时调用一次；先前由控制任务记录的事实必须保留。 */
void App_MessageLogInit(uint32_t now_ms)
{
  uint8_t i;
  if (s_initialized != 0U) { return; }
  for (i = 0U; i < APP_MSGLOG_MODULES; ++i) { AppLog_InitDebounce(&s_module_db[i]); }
  AppLog_InitDebounce(&s_check_db); AppLog_InitDebounce(&s_fix_db);
  AppLog_InitDebounce(&s_motor_db); AppLog_InitDebounce(&s_main_db); AppLog_InitDebounce(&s_alarm_db);
  s_check_db.committed = PX4LITE_LOGMSG_CHECKING;
  s_check_db.committed_key = (uint32_t)PX4LITE_LOGMSG_CHECKING << 18U;
  s_initialized = 1U;
  Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_SYSTEM_START, now_ms, 0U, 0U, 0U);
  Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_CHECKING, now_ms, 0U, 0U, 0U);
}
/** @brief 独立采集服务推进状态，不受显示初始化、切页、冻结或重试影响。 */
void App_MessageLogUpdate(uint32_t now_ms)
{
  Px4Lite_ModuleStatus_t status;
  Px4Lite_ModuleStatus_t gnss;
  uint8_t states[APP_MSGLOG_MODULES];
  uint8_t i, passed = 0U, enabled = 0U, failed = 0U, starting = 0U, unknown = 0U;
  uint16_t message;
  uint32_t signature = 0U;
  App_NavigationSnapshot_t nav;
  if (s_initialized == 0U) { App_MessageLogInit(now_ms); }
  memset(&gnss, 0, sizeof(gnss));
  for (i = 0U; i < APP_MSGLOG_MODULES; ++i) {
    uint16_t base = s_modules[i].base;
    uint16_t fault = 0U;
    states[i] = 8U;
    if (App_GetModuleStatus(s_modules[i].module, &status) == PX4LITE_OK) {
      if (status.state <= PX4LITE_STATE_DISABLED) { states[i] = (uint8_t)status.state; }
      fault = status.fault_code;
      if (s_modules[i].module == PX4LITE_MODULE_GNSS) { gnss = status; }
      if (s_modules[i].module == PX4LITE_MODULE_REMOTE_ID) {
        AppLog_ObserveRid(&status, now_ms);
      }
    }
    message = (uint16_t)(base + states[i]);
    if ((s_modules[i].module == PX4LITE_MODULE_5G) && (states[i] == PX4LITE_STATE_OFFLINE)) {
      message = (fault == PX4LITE_FAULT_COMM_OFFLINE) ? PX4LITE_LOGMSG_5G_LOST : PX4LITE_LOGMSG_NETWORK_STALE;
    }
    AppLog_Observe(&s_module_db[i], message, (uint16_t)(base + PX4LITE_STATE_ONLINE), (uint16_t)(base + 7U),
                   fault, (uint8_t)s_modules[i].module, Px4Lite_LogMessageSeverity(message), now_ms);
  }
  for (i = 0U; i < 5U; ++i) {
    uint8_t state = states[s_checks[i]];
    signature |= (uint32_t)state << (i * 4U);
    if (state == PX4LITE_STATE_DISABLED) { continue; }
    ++enabled;
    if (state == PX4LITE_STATE_ONLINE) { ++passed; }
    else if ((state == PX4LITE_STATE_FAILED) || (state == PX4LITE_STATE_OFFLINE)) { failed = 1U; }
    else if (state == 8U) { unknown = 1U; }
    else if ((state == PX4LITE_STATE_STARTING) || (state == PX4LITE_STATE_UNINITIALIZED)) { starting = 1U; }
  }
  message = (unknown || !enabled) ? PX4LITE_LOGMSG_CHECKS_UNAVAILABLE :
            failed ? PX4LITE_LOGMSG_SELFCHECK_FAIL : starting ? PX4LITE_LOGMSG_CHECKING :
            (passed == enabled) ? PX4LITE_LOGMSG_SELFCHECK_OK : PX4LITE_LOGMSG_SELFCHECK_PART;
  if ((!s_check_signature_valid) || (signature != s_check_signature)) {
    s_check_signature_valid = 1U; s_check_signature = signature; s_check_db.since_ms = now_ms;
  }
  AppLog_Observe(&s_check_db, message, PX4LITE_LOGMSG_SELFCHECK_OK, APP_MSGLOG_NONE,
                 0U, 0U, Px4Lite_LogMessageSeverity(message), now_ms);
  /* GNSS 定位与接收链路独立：仅有新鲜合法 RX 才能称 no fix。 */
  message = PX4LITE_LOGMSG_GNSS_UNAVAILABLE;
  if ((states[0] == PX4LITE_STATE_ONLINE) || (states[0] == PX4LITE_STATE_DEGRADED) || (states[0] == PX4LITE_STATE_OFFLINE)) {
    if ((gnss.last_rx_ms == 0U) || ((uint32_t)(now_ms - gnss.last_rx_ms) > PX4LITE_GNSS_MAX_AGE_MS)) { message = PX4LITE_LOGMSG_GPS_LOST; }
    else if ((App_CopyNavigation(&nav, now_ms) == PX4LITE_OK) && ((nav.valid_mask & PX4LITE_NAV_VALID_POSITION) != 0U)) { message = PX4LITE_LOGMSG_GPS_OK; }
    else { message = PX4LITE_LOGMSG_GPS_NOSIG; }
    if ((message == PX4LITE_LOGMSG_GPS_NOSIG) && ((s_fix_db.committed == PX4LITE_LOGMSG_GPS_OK) || (s_fix_db.committed == PX4LITE_LOGMSG_GNSS_FIX_LOST))) { message = PX4LITE_LOGMSG_GNSS_FIX_LOST; }
    AppLog_Observe(&s_fix_db, message, PX4LITE_LOGMSG_GPS_OK, APP_MSGLOG_NONE, 0U, PX4LITE_MODULE_GNSS, Px4Lite_LogMessageSeverity(message), now_ms);
  } else { AppLog_InitDebounce(&s_fix_db); }
  AppLog_ObservePower(now_ms);
  AppLog_ObserveAlarms(now_ms);
}
/** @brief 兼容历史业务接口；新生产路径仅由对应 Framework 状态机调用。 */
void App_MessageLogPushTakeoffSensorFail(uint32_t now_ms) { Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_TAKEOFF_SENSOR_FAIL, now_ms, 0U, PX4LITE_MODULE_CONTROL, 1U); }
/** @brief 兼容历史接口，记录动力准入拒绝。 */
void App_MessageLogPushTakeoffPowerFail(uint32_t now_ms) { Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_TAKEOFF_POWER_FAIL, now_ms, 0U, PX4LITE_MODULE_CONTROL, 1U); }
/** @brief 兼容历史接口，记录油门准入拒绝。 */
void App_MessageLogPushThrottlePowerFail(uint32_t now_ms) { Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_THROTTLE_POWER_FAIL, now_ms, 0U, PX4LITE_MODULE_CONTROL, 1U); }
/** @brief 兼容历史接口，记录缓降拒绝。 */
void App_MessageLogPushLandingFail(uint32_t now_ms) { Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_LANDING_FAIL, now_ms, 0U, PX4LITE_MODULE_CONTROL, 1U); }
/** @brief 兼容历史接口，仅真实运行中保护撤收可调用。 */
void App_MessageLogPushMotorPowerCutoff(uint32_t now_ms) { Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_MOTOR_POWER_CUTOFF, now_ms, 0U, PX4LITE_MODULE_CONTROL, 1U); }
/** @brief 兼容历史接口，记录缓降期间非零请求被拒。 */
void App_MessageLogPushLandingThrottleReject(uint32_t now_ms) { Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_LANDING_THROTTLE_REJECT, now_ms, 0U, PX4LITE_MODULE_CONTROL, 1U); }
/** @brief 直接复制固定容量环，版本用于刷新；无需 sequence 大小比较或大型栈临时数组。 */
Px4Lite_Result_t App_MessageLogCopy(App_DisplayLogSnapshot_t *out)
{
  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  out->count = Px4Lite_LocalMsgLogCopy(out->entries, APP_DISPLAY_LOG_CAP, &out->version, 0);
  return (out->count != 0U) ? PX4LITE_OK : PX4LITE_NOT_READY;
}
