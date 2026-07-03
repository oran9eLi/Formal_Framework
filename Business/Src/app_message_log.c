/**
 * @file app_message_log.c
 * @brief Business层结构化消息日志生产器实现。
 *
 * @details
 * 本模块通过 `app_data_api.h` 读取本机聚合视图，按去抖规则生成业务消息，并写入
 * Framework 固定环形日志。实现不访问 BSP、Sensor 或 Display 私有状态，所有大快照使用
 * 静态缓存，避免扩大业务任务栈。
 */

#include "app_message_log.h"

#include <string.h>

#include "app_data_api.h"

#define APP_MSGLOG_DEBOUNCE_MS       1500U
#define APP_MSGLOG_MOTOR_CONNECT_MV  5000U
#define APP_MSGLOG_MOTOR_DEAD_MV     9000U
#define APP_MSGLOG_MOTOR_RUN_MV      9900U
#define APP_MSGLOG_MOTOR_OK_MV       10500U
#define APP_MSGLOG_MAIN_PRESENT_MV   5000U
#define APP_MSGLOG_MAIN_CHARGE_MV    10500U
#define APP_MSGLOG_BAT_HYST_MV       200U

#define APP_MSGLOG_MOTOR_BAND_DISCONNECT 0U
#define APP_MSGLOG_MOTOR_BAND_DEAD       1U
#define APP_MSGLOG_MOTOR_BAND_LOWPOWER   2U
#define APP_MSGLOG_MOTOR_BAND_CHARGE     3U
#define APP_MSGLOG_MAIN_BAND_CHARGE      1U

typedef struct {
  App_LogMessageId_t committed; /**< 已提交到日志的状态。 */
  App_LogMessageId_t candidate; /**< 当前候选状态。 */
  uint32_t since_ms;            /**< 候选状态首次出现时间，单位：ms。 */
} AppMsgLog_Debounce_t;

static App_DisplaySnapshot_t s_view;
static AppMsgLog_Debounce_t s_db_gps;
static AppMsgLog_Debounce_t s_db_att;
static AppMsgLog_Debounce_t s_db_env;
static AppMsgLog_Debounce_t s_db_comm;
static AppMsgLog_Debounce_t s_db_store;
static AppMsgLog_Debounce_t s_db_motor;
static AppMsgLog_Debounce_t s_db_main;
static AppMsgLog_Debounce_t s_db_alarm;
static App_LogMessageId_t s_boot_self;
static App_LogMessageId_t s_boot_gps;
static App_LogMessageId_t s_boot_att;
static App_LogMessageId_t s_boot_env;
static App_LogMessageId_t s_boot_comm;
static App_LogMessageId_t s_boot_store;
static App_LogMessageId_t s_boot_motor;
static App_LogMessageId_t s_boot_alarm;
static uint32_t s_boot_since_ms;
static uint8_t s_boot_done;
static uint8_t s_boot_tracking;
static uint8_t s_motor_band;
static uint8_t s_main_band;
static uint8_t s_imu_level_ok_logged;

static uint32_t AppMsgLog_Hhmmss(uint32_t now_ms)
{
  uint32_t total_s = now_ms / 1000U;
  uint32_t hh = (total_s / 3600U) % 100U;
  uint32_t mm = (total_s / 60U) % 60U;
  uint32_t ss = total_s % 60U;

  return (hh * 10000U) + (mm * 100U) + ss;
}

static void AppMsgLog_Push(App_LogMessageId_t msg, uint32_t now_ms)
{
  if (msg >= APP_LOGMSG_COUNT) { return; }
  App_PushLocalMessageLog((uint16_t)msg, AppMsgLog_Hhmmss(now_ms), 0, 0, 0, (uint8_t)((msg == APP_LOGMSG_ALARM_ACTIVE) ? 1U : 0U));
}

static void AppMsgLog_ResetDebounce(AppMsgLog_Debounce_t *db, uint32_t now_ms)
{
  if (db == 0) { return; }
  db->committed = APP_LOGMSG_COUNT;
  db->candidate = APP_LOGMSG_COUNT;
  db->since_ms = now_ms;
}

static void AppMsgLog_CommitInitial(AppMsgLog_Debounce_t *db, App_LogMessageId_t msg, uint32_t now_ms)
{
  if (db == 0) { return; }
  db->committed = msg;
  db->candidate = msg;
  db->since_ms = now_ms;
}

static uint8_t AppMsgLog_Debounce(AppMsgLog_Debounce_t *db, App_LogMessageId_t now_msg, uint32_t now_ms)
{
  if (db == 0) { return 0U; }
  if (now_msg != db->candidate) {
    db->candidate = now_msg;
    db->since_ms = now_ms;
  }
  if ((db->candidate != db->committed) && (Px4Lite_ElapsedMs(now_ms, db->since_ms) >= APP_MSGLOG_DEBOUNCE_MS)) {
    AppMsgLog_Push(db->candidate, now_ms);
    db->committed = db->candidate;
    return 1U;
  }
  return 0U;
}

static uint8_t AppMsgLog_BandHyst(uint8_t current, uint32_t mv, const uint32_t *thresholds, uint8_t count)
{
  uint8_t plain = 0U;

  while ((plain < count) && (mv >= thresholds[plain])) { plain++; }
  if (plain > current) {
    if (mv >= (uint32_t)(thresholds[current] + APP_MSGLOG_BAT_HYST_MV)) { return plain; }
  } else if ((plain < current) && (current > 0U)) {
    if ((mv + APP_MSGLOG_BAT_HYST_MV) < thresholds[current - 1U]) { return plain; }
  }
  return current;
}

static void AppMsgLog_UpdateBands(const App_DisplaySnapshot_t *view)
{
  static const uint32_t motor_t[4] = {APP_MSGLOG_MOTOR_CONNECT_MV, APP_MSGLOG_MOTOR_DEAD_MV, APP_MSGLOG_MOTOR_RUN_MV, APP_MSGLOG_MOTOR_OK_MV};
  static const uint32_t main_t[2] = {APP_MSGLOG_MAIN_PRESENT_MV, APP_MSGLOG_MAIN_CHARGE_MV};

  if (view == 0) { return; }
  s_motor_band = AppMsgLog_BandHyst(s_motor_band, view->voltage2_mv, motor_t, 4U);
  s_main_band = AppMsgLog_BandHyst(s_main_band, view->voltage_mv, main_t, 2U);
}

static App_LogMessageId_t AppMsgLog_GpsMsg(App_ViewState_t state)
{
  if (state == APP_VIEW_STATE_ONLINE) { return APP_LOGMSG_GPS_OK; }
  if ((state == APP_VIEW_STATE_OFFLINE) || (state == APP_VIEW_STATE_FAILED)) { return APP_LOGMSG_GPS_LOST; }
  return APP_LOGMSG_GPS_NOSIG;
}

static App_LogMessageId_t AppMsgLog_TwoStateMsg(App_ViewState_t state, App_LogMessageId_t ok_msg, App_LogMessageId_t lost_msg)
{
  return (state == APP_VIEW_STATE_ONLINE) ? ok_msg : lost_msg;
}

static App_LogMessageId_t AppMsgLog_SelfCheckMsg(const App_ViewState_t *states, uint8_t count)
{
  uint8_t i;
  uint8_t online = 0U;
  uint8_t failed = 0U;

  if ((states == 0) || (count == 0U)) { return APP_LOGMSG_SELFCHECK_FAIL; }
  for (i = 0U; i < count; i++) {
    if (states[i] == APP_VIEW_STATE_ONLINE) { online++; }
    else if ((states[i] == APP_VIEW_STATE_OFFLINE) || (states[i] == APP_VIEW_STATE_FAILED)) { failed++; }
  }
  if (online == count) { return APP_LOGMSG_SELFCHECK_OK; }
  if (failed > 0U) { return APP_LOGMSG_SELFCHECK_FAIL; }
  return APP_LOGMSG_SELFCHECK_PART;
}

static App_LogMessageId_t AppMsgLog_MotorMsg(void)
{
  switch (s_motor_band) {
    case APP_MSGLOG_MOTOR_BAND_DISCONNECT: return APP_LOGMSG_MOTOR_DISCONNECT;
    case APP_MSGLOG_MOTOR_BAND_DEAD: return APP_LOGMSG_MOTOR_DEAD;
    case APP_MSGLOG_MOTOR_BAND_LOWPOWER: return APP_LOGMSG_MOTOR_LOWPOWER;
    case APP_MSGLOG_MOTOR_BAND_CHARGE: return APP_LOGMSG_MOTOR_CHARGE;
    default: return APP_LOGMSG_MOTOR_OK;
  }
}

static App_LogMessageId_t AppMsgLog_MainMsg(void)
{
  return (s_main_band == APP_MSGLOG_MAIN_BAND_CHARGE) ? APP_LOGMSG_MAIN_CHARGE : APP_LOGMSG_COUNT;
}

void App_MessageLogInit(uint32_t now_ms)
{
  App_ResetLocalMessageLog();
  memset(&s_view, 0, sizeof(s_view));
  s_boot_self = APP_LOGMSG_COUNT;
  s_boot_gps = APP_LOGMSG_COUNT;
  s_boot_att = APP_LOGMSG_COUNT;
  s_boot_env = APP_LOGMSG_COUNT;
  s_boot_comm = APP_LOGMSG_COUNT;
  s_boot_store = APP_LOGMSG_COUNT;
  s_boot_motor = APP_LOGMSG_COUNT;
  s_boot_alarm = APP_LOGMSG_COUNT;
  s_boot_since_ms = now_ms;
  s_boot_done = 0U;
  s_boot_tracking = 0U;
  s_motor_band = APP_MSGLOG_MOTOR_BAND_DISCONNECT;
  s_main_band = 2U;
  s_imu_level_ok_logged = 0U;
  AppMsgLog_ResetDebounce(&s_db_gps, now_ms);
  AppMsgLog_ResetDebounce(&s_db_att, now_ms);
  AppMsgLog_ResetDebounce(&s_db_env, now_ms);
  AppMsgLog_ResetDebounce(&s_db_comm, now_ms);
  AppMsgLog_ResetDebounce(&s_db_store, now_ms);
  AppMsgLog_ResetDebounce(&s_db_motor, now_ms);
  AppMsgLog_ResetDebounce(&s_db_main, now_ms);
  AppMsgLog_ResetDebounce(&s_db_alarm, now_ms);
}

void App_MessageLogUpdate(uint32_t now_ms)
{
  App_ViewState_t states[5];
  App_LogMessageId_t now_self;
  App_LogMessageId_t now_gps;
  App_LogMessageId_t now_att;
  App_LogMessageId_t now_env;
  App_LogMessageId_t now_comm;
  App_LogMessageId_t now_store;
  App_LogMessageId_t now_motor;
  App_LogMessageId_t now_main;
  App_LogMessageId_t now_alarm;

  if (App_CopyDisplaySnapshot(&s_view, now_ms) == 0U) { return; }
  AppMsgLog_UpdateBands(&s_view);

  states[0] = s_view.gnss.state;
  states[1] = s_view.imu.state;
  states[2] = s_view.baro.state;
  states[3] = s_view.lora.state;
  states[4] = s_view.storage.state;

  now_self = AppMsgLog_SelfCheckMsg(states, 5U);
  now_gps = AppMsgLog_GpsMsg(s_view.gnss.state);
  now_att = AppMsgLog_TwoStateMsg(s_view.imu.state, APP_LOGMSG_ATT_OK, APP_LOGMSG_ATT_LOST);
  if (s_view.attitude_valid == 0U) { now_att = APP_LOGMSG_ATT_LOST; }
  now_env = AppMsgLog_TwoStateMsg(s_view.baro.state, APP_LOGMSG_ENV_OK, APP_LOGMSG_ENV_LOST);
  now_comm = AppMsgLog_TwoStateMsg(s_view.lora.state, APP_LOGMSG_COMM_OK, APP_LOGMSG_COMM_LOST);
  now_store = AppMsgLog_TwoStateMsg(s_view.storage.state, APP_LOGMSG_STORAGE_OK, APP_LOGMSG_STORAGE_LOST);
  now_motor = AppMsgLog_MotorMsg();
  now_main = AppMsgLog_MainMsg();
  now_alarm = (s_view.highest_fault_code != 0U) ? APP_LOGMSG_ALARM_ACTIVE : APP_LOGMSG_ALARM_NONE;

  if (s_boot_done == 0U) {
    if ((s_boot_tracking == 0U) || (now_self != s_boot_self) || (now_gps != s_boot_gps) || (now_att != s_boot_att) ||
        (now_env != s_boot_env) || (now_comm != s_boot_comm) || (now_store != s_boot_store) || (now_motor != s_boot_motor) || (now_alarm != s_boot_alarm)) {
      s_boot_self = now_self;
      s_boot_gps = now_gps;
      s_boot_att = now_att;
      s_boot_env = now_env;
      s_boot_comm = now_comm;
      s_boot_store = now_store;
      s_boot_motor = now_motor;
      s_boot_alarm = now_alarm;
      s_boot_since_ms = now_ms;
      if (s_boot_tracking == 0U) {
        AppMsgLog_Push(APP_LOGMSG_SYSTEM_START, now_ms);
        AppMsgLog_Push(APP_LOGMSG_IMU_LEVEL_WAIT, now_ms);
        s_boot_tracking = 1U;
      }
      return;
    }
    if (Px4Lite_ElapsedMs(now_ms, s_boot_since_ms) < APP_MSGLOG_DEBOUNCE_MS) { return; }

    AppMsgLog_Push(s_boot_self, now_ms);
    AppMsgLog_Push(s_boot_gps, now_ms);
    AppMsgLog_Push(s_boot_att, now_ms);
    AppMsgLog_Push(s_boot_env, now_ms);
    AppMsgLog_Push(s_boot_comm, now_ms);
    AppMsgLog_Push(s_boot_store, now_ms);
    AppMsgLog_Push(s_boot_motor, now_ms);
    AppMsgLog_Push(s_boot_alarm, now_ms);
    if (s_boot_att == APP_LOGMSG_ATT_OK) {
      AppMsgLog_Push(APP_LOGMSG_IMU_LEVEL_OK, now_ms);
      s_imu_level_ok_logged = 1U;
    }
    AppMsgLog_CommitInitial(&s_db_gps, s_boot_gps, now_ms);
    AppMsgLog_CommitInitial(&s_db_att, s_boot_att, now_ms);
    AppMsgLog_CommitInitial(&s_db_env, s_boot_env, now_ms);
    AppMsgLog_CommitInitial(&s_db_comm, s_boot_comm, now_ms);
    AppMsgLog_CommitInitial(&s_db_store, s_boot_store, now_ms);
    AppMsgLog_CommitInitial(&s_db_motor, s_boot_motor, now_ms);
    AppMsgLog_CommitInitial(&s_db_main, now_main, now_ms);
    AppMsgLog_CommitInitial(&s_db_alarm, s_boot_alarm, now_ms);
    s_boot_done = 1U;
    return;
  }

  (void)AppMsgLog_Debounce(&s_db_gps, now_gps, now_ms);
  if ((AppMsgLog_Debounce(&s_db_att, now_att, now_ms) != 0U) && (now_att == APP_LOGMSG_ATT_OK) && (s_imu_level_ok_logged == 0U)) {
    AppMsgLog_Push(APP_LOGMSG_IMU_LEVEL_OK, now_ms);
    s_imu_level_ok_logged = 1U;
  }
  (void)AppMsgLog_Debounce(&s_db_env, now_env, now_ms);
  (void)AppMsgLog_Debounce(&s_db_comm, now_comm, now_ms);
  (void)AppMsgLog_Debounce(&s_db_store, now_store, now_ms);
  (void)AppMsgLog_Debounce(&s_db_motor, now_motor, now_ms);
  (void)AppMsgLog_Debounce(&s_db_main, now_main, now_ms);
  (void)AppMsgLog_Debounce(&s_db_alarm, now_alarm, now_ms);
}

uint16_t App_MessageLogCopy(Px4Lite_LogEntry_t *entries, uint16_t capacity, uint32_t *version, uint16_t *last_seq)
{
  if ((entries == 0) || (capacity == 0U)) { return 0U; }
  return App_CopyLocalMessageLog(entries, capacity, version, last_seq);
}

uint32_t App_MessageLogGetVersion(void)
{
  uint32_t version = 0U;

  (void)App_CopyLocalMessageLog(0, 0U, &version, 0);
  return version;
}
