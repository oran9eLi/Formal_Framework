/**
 * @file test_app_message_log.c
 * @brief 业务日志生产者：启动批量、去抖追加、单调序号、环形容量。
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_message_log.h"
#include "app_data_api.h"

static int g_fail;
static void Eq(const char *n, uint32_t a, uint32_t e)
{ if (a != e) { printf("FAIL %s actual=%lu expected=%lu\n", n, (unsigned long)a, (unsigned long)e); g_fail++; } }

/* ---- 可控数据源桩 ---- */
static Px4Lite_State_t s_state[PX4LITE_MODULE_COUNT];
static uint32_t s_gnss_rx_ms;
static uint32_t s_v1_mv = 12000U; /* 主控电池正常 */
static uint32_t s_v2_mv = 12000U; /* 电机电池正常 */
static uint16_t s_highest_fault;

Px4Lite_Result_t App_GetModuleStatus(Px4Lite_ModuleId_t m, Px4Lite_ModuleStatus_t *o)
{
  if (o == 0) { return PX4LITE_INVALID_PARAM; }
  memset(o, 0, sizeof(*o));
  o->module_id = m;
  o->state = s_state[m];
  if (m == PX4LITE_MODULE_GNSS) { o->last_rx_ms = s_gnss_rx_ms; }
  return PX4LITE_OK;
}
Px4Lite_Result_t App_CopyEnvironment(App_EnvironmentSnapshot_t *o, uint32_t n)
{ (void)n; if (o == 0) { return PX4LITE_INVALID_PARAM; } memset(o, 0, sizeof(*o)); o->voltage_mv = s_v1_mv; o->voltage2_mv = s_v2_mv; return PX4LITE_OK; }
Px4Lite_Result_t App_CopyAlarm(App_AlarmSnapshot_t *o, uint32_t n)
{ (void)n; if (o == 0) { return PX4LITE_INVALID_PARAM; } memset(o, 0, sizeof(*o)); o->highest_fault_code = s_highest_fault; return PX4LITE_OK; }

static void AllOnline(void)
{ uint8_t i; for (i = 0U; i < (uint8_t)PX4LITE_MODULE_COUNT; ++i) { s_state[i] = PX4LITE_STATE_ONLINE; } s_gnss_rx_ms = 1U; }

/* 推进 update 若干次，跨过去抖窗口。 */
static void Advance(uint32_t *now, uint32_t span_ms, uint32_t step_ms)
{ uint32_t end = *now + span_ms; while (*now < end) { *now += step_ms; App_MessageLogUpdate(*now); } }

static void TestBootBatchThenDebounce(void)
{
  App_DisplayLogSnapshot_t log;
  uint32_t now = 1000U;
  uint16_t i;
  uint16_t prev_seq = 0U;

  AllOnline();
  App_MessageLogInit(now);

  /* 首次 update：记“系统启动”并开始启动跟踪。 */
  App_MessageLogUpdate(now);
  /* 稳定跨过去抖窗口后，批量提交自检/各模块/告警。 */
  Advance(&now, 2000U, 100U);

  Eq("copy ok", App_MessageLogCopy(&log), PX4LITE_OK);
  if (log.count == 0U) { printf("FAIL no entries after boot\n"); g_fail++; return; }
  Eq("first is system_start", log.entries[0].message_id, APP_LOGMSG_SYSTEM_START);
  /* 序号单调递增、time 已填、message_id 合法。 */
  for (i = 0U; i < log.count; ++i) {
    if (log.entries[i].sequence <= prev_seq) { printf("FAIL seq not monotonic at %u\n", i); g_fail++; }
    prev_seq = log.entries[i].sequence;
    if (log.entries[i].message_id >= APP_LOGMSG_COUNT) { printf("FAIL bad message_id at %u\n", i); g_fail++; }
  }

  /* 运行期制造一次 GPS 掉线，稳定去抖后应追加一条 GPS_LOST。 */
  {
    uint16_t before = log.count;
    uint16_t last_seq = log.entries[log.count - 1U].sequence;
    s_state[PX4LITE_MODULE_GNSS] = PX4LITE_STATE_OFFLINE;
    Advance(&now, 2000U, 100U);
    Eq("copy ok2", App_MessageLogCopy(&log), PX4LITE_OK);
    if (log.count < before && before <= APP_DISPLAY_LOG_CAP) { printf("FAIL count shrank\n"); g_fail++; }
    /* 最新一条应为 GPS_LOST，且序号更大。 */
    Eq("gps lost appended", log.entries[log.count - 1U].message_id, APP_LOGMSG_GPS_LOST);
    if (log.entries[log.count - 1U].sequence <= last_seq) { printf("FAIL new seq not greater\n"); g_fail++; }
  }
}

static void TestDebounceSwallowsTransient(void)
{
  App_DisplayLogSnapshot_t log;
  uint32_t now = 50000U;
  uint16_t baseline;

  AllOnline();
  App_MessageLogInit(now);
  App_MessageLogUpdate(now);
  Advance(&now, 2000U, 100U);
  (void)App_MessageLogCopy(&log);
  baseline = log.count;

  /* 抖动：掉线仅 500ms(< 1500ms 去抖)即恢复，不应追加。 */
  s_state[PX4LITE_MODULE_BARO] = PX4LITE_STATE_OFFLINE;
  Advance(&now, 500U, 100U);
  s_state[PX4LITE_MODULE_BARO] = PX4LITE_STATE_ONLINE;
  Advance(&now, 2000U, 100U);
  (void)App_MessageLogCopy(&log);
  Eq("transient swallowed", log.count, baseline);
}

int main(void)
{
  TestBootBatchThenDebounce();
  TestDebounceSwallowsTransient();
  if (g_fail == 0) { printf("PASS test_app_message_log\n"); return 0; }
  printf("FAIL test_app_message_log fails=%d\n", g_fail); return 1;
}
