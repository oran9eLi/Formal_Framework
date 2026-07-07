/**
 * @file app_message_log.c
 * @brief 业务层结构化消息日志生产者：本机状态变化经去抖生成带序号日志条目。
 *
 * @details
 * 迁自显示层日志机；数据源改为本机 App_* 接口，电池档位本层自算(滞回)。
 * 环形缓冲容量 APP_DISPLAY_LOG_CAP，每条带单调 sequence(从 1 起)。
 */
#include "app_message_log.h"

#include <string.h>

#include "app_data_api.h"
#include "px4lite_local_msglog.h"

#define APP_MSGLOG_DEBOUNCE_MS 1500U

/* 电池档位门限(mV)与滞回，数值与显示层一致(各算各的，同输入同结果)。 */
#define APP_MOTOR_BAT_CONNECT_MV 5000U
#define APP_MOTOR_BAT_DEAD_MV    9000U
#define APP_MOTOR_BAT_RUN_MV     9900U
#define APP_MOTOR_BAT_OK_MV      10500U
#define APP_MOTOR_BAT_MAX_MV     13500U /* 超此值视为电机电池未插(ADC悬空高值)，判断开；须与显示层一致 */
#define APP_MAIN_BAT_PRESENT_MV  5000U
#define APP_MAIN_BAT_CHARGE_MV   10500U
#define APP_BAT_HYST_MV          200U

#define APP_MOTOR_BAND_DISCONNECT 0U
#define APP_MOTOR_BAND_DEAD       1U
#define APP_MOTOR_BAND_LOWPOWER   2U
#define APP_MOTOR_BAND_CHARGE     3U
#define APP_MAIN_BAND_CHARGE      1U

typedef struct {
  App_LogMessageId_t committed;
  App_LogMessageId_t candidate;
  uint32_t since_ms;
} AppLog_Debounce_t;

/* 日志环形缓冲已下沉到 Framework px4lite_local_msglog(单一真值源)，本层只做生成策略。 */

/* band 自算状态 */
static uint8_t s_motor_band;  /* 初值=断开 */
static uint8_t s_main_band;   /* 初值=正常(2) */

/* 去抖与启动跟踪 */
static uint8_t s_boot_done;
static uint8_t s_boot_tracking;
static AppLog_Debounce_t s_db_gps, s_db_att, s_db_env, s_db_comm, s_db_store, s_db_motor, s_db_main, s_db_alarm, s_db_remoteid;
static App_LogMessageId_t s_boot_self, s_boot_gps, s_boot_att, s_boot_env, s_boot_comm, s_boot_store, s_boot_alarm, s_boot_remoteid;
static uint32_t s_boot_since_ms, s_boot_start_t;

static uint32_t AppLog_Hhmmss(uint32_t now_ms)
{
  uint32_t total_s = now_ms / 1000U;
  uint32_t hh = (total_s / 3600U) % 100U;
  uint32_t mm = (total_s / 60U) % 60U;
  uint32_t ss = total_s % 60U;
  return (hh * 10000U) + (mm * 100U) + ss;
}

static void AppLog_Push(App_LogMessageId_t msg, uint32_t now_ms)
{
  Px4Lite_LocalMsgLogPush((uint16_t)msg, AppLog_Hhmmss(now_ms),
                          0U, 0U, 0U, (uint8_t)((msg == APP_LOGMSG_ALARM_ACTIVE) ? 1U : 0U));
}

static uint8_t AppLog_Debounce(AppLog_Debounce_t *d, App_LogMessageId_t now, uint32_t now_ms)
{
  if (now != d->candidate) { d->candidate = now; d->since_ms = now_ms; }
  if ((d->candidate != d->committed) && ((uint32_t)(now_ms - d->since_ms) >= APP_MSGLOG_DEBOUNCE_MS)) {
    AppLog_Push(d->candidate, now_ms);
    d->committed = d->candidate;
    return 1U;
  }
  return 0U;
}

static void AppLog_CommitInitial(AppLog_Debounce_t *d, App_LogMessageId_t msg, uint32_t now_ms)
{ d->candidate = msg; d->committed = msg; d->since_ms = now_ms; }

static uint8_t AppLog_BandHyst(uint8_t cur, uint32_t mv, const uint32_t *t, uint8_t n)
{
  uint8_t plain = 0U;
  while ((plain < n) && (mv >= t[plain])) { plain++; }
  if (plain > cur) {
    if (mv >= (uint32_t)(t[cur] + APP_BAT_HYST_MV)) { return plain; }
  } else if (plain < cur) {
    if ((mv + APP_BAT_HYST_MV) < t[cur - 1U]) { return plain; }
  }
  return cur;
}

static void AppLog_UpdateBands(uint32_t now_ms)
{
  static const uint32_t motor_t[4] = {APP_MOTOR_BAT_CONNECT_MV, APP_MOTOR_BAT_DEAD_MV, APP_MOTOR_BAT_RUN_MV, APP_MOTOR_BAT_OK_MV};
  static const uint32_t main_t[2]  = {APP_MAIN_BAT_PRESENT_MV, APP_MAIN_BAT_CHARGE_MV};
  App_EnvironmentSnapshot_t env;
  if (App_CopyEnvironment(&env, now_ms) != PX4LITE_OK) { return; }
  s_motor_band = AppLog_BandHyst(s_motor_band, env.voltage2_mv, motor_t, 4U);
  /* 超上限视为未插电池(ADC 悬空高值)，强制断开档，与显示层电机灯判定一致，避免没插却报"电机正常"。 */
  if (env.voltage2_mv > APP_MOTOR_BAT_MAX_MV) { s_motor_band = APP_MOTOR_BAND_DISCONNECT; }
  s_main_band  = AppLog_BandHyst(s_main_band, env.voltage_mv, main_t, 2U);
}

static Px4Lite_State_t AppLog_ModuleState(Px4Lite_ModuleId_t id, Px4Lite_ModuleStatus_t *status)
{
  Px4Lite_ModuleStatus_t local;
  Px4Lite_ModuleStatus_t *slot = (status != 0) ? status : &local;

  /* status 可为 NULL（调用方只需 state 时）：仍用局部缓冲读真实状态，
     不能因 NULL 就直接返回 OFFLINE，否则姿态/环境/通信/存储永远被判断开。 */
  if (App_GetModuleStatus(id, slot) == PX4LITE_OK) { return slot->state; }
  if (status != 0) { memset(status, 0, sizeof(*status)); status->module_id = id; status->state = PX4LITE_STATE_OFFLINE; }
  return PX4LITE_STATE_OFFLINE;
}

static App_LogMessageId_t AppLog_GpsMsg(const Px4Lite_ModuleStatus_t *s)
{
  if (s == 0) { return APP_LOGMSG_GPS_LOST; }
  if (s->state == PX4LITE_STATE_ONLINE) { return APP_LOGMSG_GPS_OK; }
  if ((s->state == PX4LITE_STATE_DEGRADED) && (s->last_rx_ms != 0U)) { return APP_LOGMSG_GPS_NOSIG; }
  return APP_LOGMSG_GPS_LOST;
}

static App_LogMessageId_t AppLog_TwoState(Px4Lite_State_t st, App_LogMessageId_t ok, App_LogMessageId_t lost)
{ return (st == PX4LITE_STATE_ONLINE) ? ok : lost; }

static App_LogMessageId_t AppLog_SelfCheckMsg(const Px4Lite_State_t *st, uint16_t n)
{
  uint16_t online = 0U, failed = 0U, i;
  for (i = 0U; i < n; ++i) {
    if (st[i] == PX4LITE_STATE_ONLINE) { online++; }
    else if ((st[i] == PX4LITE_STATE_OFFLINE) || (st[i] == PX4LITE_STATE_FAILED)) { failed++; }
  }
  if (online == n) { return APP_LOGMSG_SELFCHECK_OK; }
  if (failed > 0U) { return APP_LOGMSG_SELFCHECK_FAIL; }
  return APP_LOGMSG_SELFCHECK_PART;
}

static App_LogMessageId_t AppLog_MotorMsg(void)
{
  switch (s_motor_band) {
    case APP_MOTOR_BAND_DISCONNECT: return APP_LOGMSG_MOTOR_DISCONNECT;
    case APP_MOTOR_BAND_DEAD:       return APP_LOGMSG_MOTOR_DEAD;
    case APP_MOTOR_BAND_LOWPOWER:   return APP_LOGMSG_MOTOR_LOWPOWER;
    case APP_MOTOR_BAND_CHARGE:     return APP_LOGMSG_MOTOR_CHARGE;
    default:                        return APP_LOGMSG_MOTOR_OK;
  }
}

static App_LogMessageId_t AppLog_MainMsg(void)
{ return (s_main_band == APP_MAIN_BAND_CHARGE) ? APP_LOGMSG_MAIN_CHARGE : APP_LOGMSG_COUNT; }

void App_MessageLogInit(uint32_t now_ms)
{
  Px4Lite_LocalMsgLogReset();
  s_motor_band = APP_MOTOR_BAND_DISCONNECT;
  s_main_band  = 2U;
  s_boot_done = 0U; s_boot_tracking = 0U;
  s_boot_since_ms = now_ms; s_boot_start_t = 0U;
  s_boot_self = APP_LOGMSG_COUNT; s_boot_gps = APP_LOGMSG_COUNT; s_boot_att = APP_LOGMSG_COUNT;
  s_boot_env = APP_LOGMSG_COUNT; s_boot_comm = APP_LOGMSG_COUNT; s_boot_store = APP_LOGMSG_COUNT; s_boot_alarm = APP_LOGMSG_COUNT;
  s_boot_remoteid = APP_LOGMSG_COUNT;
  /* 去抖初值置为 COUNT(无效)，保证首次真实状态触发提交。 */
  s_db_gps.candidate = s_db_gps.committed = APP_LOGMSG_COUNT; s_db_gps.since_ms = now_ms;
  s_db_att.candidate = s_db_att.committed = APP_LOGMSG_COUNT; s_db_att.since_ms = now_ms;
  s_db_env.candidate = s_db_env.committed = APP_LOGMSG_COUNT; s_db_env.since_ms = now_ms;
  s_db_comm.candidate = s_db_comm.committed = APP_LOGMSG_COUNT; s_db_comm.since_ms = now_ms;
  s_db_store.candidate = s_db_store.committed = APP_LOGMSG_COUNT; s_db_store.since_ms = now_ms;
  s_db_motor.candidate = s_db_motor.committed = APP_LOGMSG_COUNT; s_db_motor.since_ms = now_ms;
  s_db_main.candidate = s_db_main.committed = APP_LOGMSG_COUNT; s_db_main.since_ms = now_ms;
  s_db_alarm.candidate = s_db_alarm.committed = APP_LOGMSG_COUNT; s_db_alarm.since_ms = now_ms;
  s_db_remoteid.candidate = s_db_remoteid.committed = APP_LOGMSG_COUNT; s_db_remoteid.since_ms = now_ms;
}

void App_MessageLogUpdate(uint32_t now_ms)
{
  Px4Lite_ModuleStatus_t gnss_status;
  Px4Lite_State_t states[6];
  App_AlarmSnapshot_t alarm;
  uint16_t highest = 0U;
  uint32_t t;
  App_LogMessageId_t now_self, now_gps, now_att, now_env, now_comm, now_store, now_motor, now_main, now_alarm, now_remoteid;

  AppLog_UpdateBands(now_ms);
  t = AppLog_Hhmmss(now_ms);
  (void)t;

  states[0] = AppLog_ModuleState(PX4LITE_MODULE_GNSS, &gnss_status);
  states[1] = AppLog_ModuleState(PX4LITE_MODULE_IMU, 0);
  states[2] = AppLog_ModuleState(PX4LITE_MODULE_BARO, 0);
  states[3] = AppLog_ModuleState(PX4LITE_MODULE_LORA, 0);
  states[4] = AppLog_ModuleState(PX4LITE_MODULE_STORAGE, 0);
  states[5] = AppLog_ModuleState(PX4LITE_MODULE_REMOTE_ID, 0);

  if (App_CopyAlarm(&alarm, now_ms) == PX4LITE_OK) { highest = alarm.highest_fault_code; }

  now_self     = AppLog_SelfCheckMsg(states, 5U);
  now_gps      = AppLog_GpsMsg(&gnss_status);
  now_att      = AppLog_TwoState(states[1], APP_LOGMSG_ATT_OK, APP_LOGMSG_ATT_LOST);
  now_env      = AppLog_TwoState(states[2], APP_LOGMSG_ENV_OK, APP_LOGMSG_ENV_LOST);
  now_comm     = AppLog_TwoState(states[3], APP_LOGMSG_COMM_OK, APP_LOGMSG_COMM_LOST);
  now_store    = AppLog_TwoState(states[4], APP_LOGMSG_STORAGE_OK, APP_LOGMSG_STORAGE_LOST);
  now_remoteid = AppLog_TwoState(states[5], APP_LOGMSG_REMOTEID_OK, APP_LOGMSG_REMOTEID_LOST);
  now_motor = AppLog_MotorMsg();
  now_main  = AppLog_MainMsg();
  now_alarm = (highest != 0U) ? APP_LOGMSG_ALARM_ACTIVE : APP_LOGMSG_ALARM_NONE;

  if (s_boot_done == 0U) {
    if ((s_boot_tracking == 0U) || (now_self != s_boot_self) || (now_gps != s_boot_gps) || (now_att != s_boot_att) ||
        (now_env != s_boot_env) || (now_comm != s_boot_comm) || (now_store != s_boot_store) || (now_alarm != s_boot_alarm) ||
        (now_remoteid != s_boot_remoteid)) {
      s_boot_self = now_self; s_boot_gps = now_gps; s_boot_att = now_att; s_boot_env = now_env;
      s_boot_comm = now_comm; s_boot_store = now_store; s_boot_alarm = now_alarm; s_boot_remoteid = now_remoteid; s_boot_since_ms = now_ms;
      if (s_boot_tracking == 0U) {
        s_boot_start_t = t;
        (void)s_boot_start_t;
        AppLog_Push(APP_LOGMSG_SYSTEM_START, now_ms);
        s_boot_tracking = 1U;
      }
      return;
    }
    if ((uint32_t)(now_ms - s_boot_since_ms) < APP_MSGLOG_DEBOUNCE_MS) { return; }

    AppLog_Push(s_boot_self, now_ms);
    AppLog_Push(s_boot_gps, now_ms);
    AppLog_Push(s_boot_att, now_ms);
    AppLog_Push(s_boot_env, now_ms);
    AppLog_Push(s_boot_comm, now_ms);
    AppLog_Push(s_boot_store, now_ms);
    AppLog_Push(now_motor, now_ms);
    AppLog_Push(s_boot_alarm, now_ms);
    AppLog_Push(s_boot_remoteid, now_ms);

    AppLog_CommitInitial(&s_db_gps, s_boot_gps, now_ms);
    AppLog_CommitInitial(&s_db_att, s_boot_att, now_ms);
    AppLog_CommitInitial(&s_db_env, s_boot_env, now_ms);
    AppLog_CommitInitial(&s_db_comm, s_boot_comm, now_ms);
    AppLog_CommitInitial(&s_db_store, s_boot_store, now_ms);
    AppLog_CommitInitial(&s_db_motor, now_motor, now_ms);
    AppLog_CommitInitial(&s_db_main, now_main, now_ms);
    AppLog_CommitInitial(&s_db_alarm, s_boot_alarm, now_ms);
    AppLog_CommitInitial(&s_db_remoteid, s_boot_remoteid, now_ms);
    s_boot_done = 1U;
    return;
  }

  (void)AppLog_Debounce(&s_db_gps, now_gps, now_ms);
  (void)AppLog_Debounce(&s_db_att, now_att, now_ms);
  (void)AppLog_Debounce(&s_db_env, now_env, now_ms);
  (void)AppLog_Debounce(&s_db_comm, now_comm, now_ms);
  (void)AppLog_Debounce(&s_db_store, now_store, now_ms);
  (void)AppLog_Debounce(&s_db_motor, now_motor, now_ms);
  (void)AppLog_Debounce(&s_db_main, now_main, now_ms);
  (void)AppLog_Debounce(&s_db_alarm, now_alarm, now_ms);
  (void)AppLog_Debounce(&s_db_remoteid, now_remoteid, now_ms);
}

Px4Lite_Result_t App_MessageLogCopy(App_DisplayLogSnapshot_t *out)
{
  Px4Lite_LogEntry_t tmp[PX4LITE_LOCAL_LOG_CAP];
  uint16_t n, i, last = 0U;
  uint32_t ver = 0U;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  memset(out, 0, sizeof(*out));
  n = Px4Lite_LocalMsgLogCopy(tmp, PX4LITE_LOCAL_LOG_CAP, &ver, &last);
  out->version = ver;
  out->count   = n;
  if (n == 0U) { return PX4LITE_NOT_READY; }
  if (n > (uint16_t)APP_DISPLAY_LOG_CAP) { n = (uint16_t)APP_DISPLAY_LOG_CAP; out->count = n; }
  for (i = 0U; i < n; ++i) {
    out->entries[i].sequence    = tmp[i].sequence;
    out->entries[i].message_id  = tmp[i].message_id;
    out->entries[i].time_hhmmss = tmp[i].time_hhmmss;
    out->entries[i].fault_code  = tmp[i].fault_code;
    out->entries[i].severity    = tmp[i].severity;
    out->entries[i].source_id   = tmp[i].source_id;
    out->entries[i].active      = tmp[i].active;
  }
  return PX4LITE_OK;
}
