/**
 * @file test_app_message_log.c
 * @brief 业务日志生产者：启动批量、去抖追加、单调序号。
 * @details
 * 沿用旧版同名测试的场景(启动批量提交、GPS 掉线追加、瞬时抖动被去抖吞掉)，
 * 改用当前 app_data_api.h 的数据源签名(Px4Lite_CopyHealth +
 * Px4Lite_CopyModuleStatuses + Px4Lite_GetStatusVersion 三者一致才认为
 * 模块状态有效，App_CopyEnvironment 等改为直接对接框架 Copy* 函数)。
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_data_api.h"
#include "app_message_log.h"
#include "px4lite_alarm.h"
#include "px4lite_control.h"
#include "px4lite_local_msglog.h"
#include "px4lite_mavlink_rx.h"
#include "px4lite_mavlink_tx.h"
#include "px4lite_modules.h"
#include "px4lite_remote_telemetry.h"
#include "px4lite_time.h"

static int g_fail;
static void Eq(const char *n, uint32_t a, uint32_t e)
{ if (a != e) { printf("FAIL %s actual=%lu expected=%lu\n", n, (unsigned long)a, (unsigned long)e); g_fail++; } }

/* ---- 可控模块状态桩：三个函数共享同一份状态，版本号保持一致。 ---- */
static Px4Lite_State_t s_state[PX4LITE_MODULE_COUNT];
static uint32_t s_status_version = 1U;
static uint32_t s_gnss_last_rx_ms;
static uint32_t s_now_ms; /* App_MessageLogUpdate 驱动的当前测试时间，供健康快照保持"新鲜"。 */

Px4Lite_Result_t Px4Lite_CopyHealth(Px4Lite_SystemHealth_t *out)
{
  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  memset(out, 0, sizeof(*out));
  out->header.valid = 1U;
  out->header.sample_time_ms = s_now_ms;
  out->status_version = s_status_version;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_CopyModuleStatuses(Px4Lite_ModuleStatus_t *status, uint16_t count, uint32_t *version)
{
  uint16_t i;
  if (status == 0) { return PX4LITE_INVALID_PARAM; }
  for (i = 0U; (i < count) && (i < (uint16_t)PX4LITE_MODULE_COUNT); ++i) {
    memset(&status[i], 0, sizeof(status[i]));
    status[i].module_id = (Px4Lite_ModuleId_t)i;
    status[i].state     = s_state[i];
    if (i == (uint16_t)PX4LITE_MODULE_GNSS) { status[i].last_rx_ms = s_gnss_last_rx_ms; }
  }
  if (version != 0) { *version = s_status_version; }
  return PX4LITE_OK;
}

uint32_t Px4Lite_GetStatusVersion(void) { return s_status_version; }

/* 本测试只关心系统健康/模块状态路径，其余数据源一律 NOT_READY。 */
Px4Lite_Result_t Px4Lite_CopyNavigation(Px4Lite_VehicleNavigation_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyTime(Px4Lite_TimeSnapshot_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBaro(Px4Lite_SensorBaro_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBattery(Px4Lite_BatteryStatus_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBattery2(Px4Lite_BatteryStatus_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyMotor(Px4Lite_MotorOutputs_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyAlarmSnapshot(Px4Lite_AlarmSnapshot_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyAlarmSummary(uint32_t *publish_time_ms, uint32_t *sequence, uint16_t *active_count, uint16_t *highest_fault_code, uint16_t *highest_source_id, Px4Lite_AlarmSeverity_t *highest_severity)
{ (void)publish_time_ms; (void)sequence; (void)active_count; (void)highest_fault_code; (void)highest_source_id; (void)highest_severity; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyAlarmRecord(uint16_t index, Px4Lite_AlarmRecord_t *out) { (void)index; (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t module_id, Px4Lite_ModuleStatus_t *status) { (void)module_id; (void)status; return PX4LITE_NOT_READY; }
void Px4Lite_GetCommDebugInfo(Px4Lite_CommDebugInfo_t *out) { if (out) { memset(out, 0, sizeof(*out)); } }
Px4Lite_Result_t Px4Lite_ControlSetMotorThrottlePercent(uint8_t motor_index, uint8_t throttle_percent) { (void)motor_index; (void)throttle_percent; return PX4LITE_NOT_READY; }
void Px4Lite_MavlinkSetTxEnabled(uint8_t enabled) { (void)enabled; }
Px4Lite_Result_t Px4Lite_CopyCommRxFrame(Px4Lite_CommRxFrame_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyEnvironment(App_EnvironmentSnapshot_t *out, uint32_t now_ms) { (void)out; (void)now_ms; return PX4LITE_NOT_READY; }

static void AllOnline(uint32_t now_ms)
{
  uint8_t i;
  for (i = 0U; i < (uint8_t)PX4LITE_MODULE_COUNT; ++i) { s_state[i] = PX4LITE_STATE_ONLINE; }
  s_gnss_last_rx_ms = now_ms;
}

/* 状态改变后需要推进版本号，否则 App_CopySystem 的一致性检查会一直复制到同一份旧快照。 */
static void BumpVersion(void) { s_status_version++; }

static void Advance(uint32_t *now, uint32_t span_ms, uint32_t step_ms)
{ uint32_t end = *now + span_ms; while (*now < end) { *now += step_ms; s_now_ms = *now; App_MessageLogUpdate(*now); } }

static void TestBootBatchThenGpsLost(void)
{
  Px4Lite_LogEntry_t entries[APP_LOGMSG_COUNT];
  uint16_t count;
  uint16_t i;
  uint16_t prev_seq = 0U;
  uint32_t now = 1000U;

  AllOnline(now);
  s_now_ms = now;
  App_MessageLogInit(now);
  App_MessageLogUpdate(now);
  Advance(&now, 2000U, 100U);

  count = App_MessageLogCopy(entries, APP_LOGMSG_COUNT, 0, 0);
  if (count == 0U) { printf("FAIL no entries after boot\n"); g_fail++; return; }
  /* 启动一次性提交自检/各模块/告警共 8 条 + 2 条起始追踪 = 10 条，超过环
     形容量 PX4LITE_LOCAL_LOG_CAP(9)，最旧的 SYSTEM_START 会被挤出——
     这里验证的是"最旧一条序号最小、按顺序排列"而非固定是哪一条消息。 */
  if (count > PX4LITE_LOCAL_LOG_CAP) { printf("FAIL count exceeds ring capacity: %u\n", count); g_fail++; }
  for (i = 0U; i < count; ++i) {
    if (entries[i].sequence <= prev_seq) { printf("FAIL seq not monotonic at %u\n", i); g_fail++; }
    prev_seq = entries[i].sequence;
    if (entries[i].message_id >= APP_LOGMSG_COUNT) { printf("FAIL bad message_id at %u\n", i); g_fail++; }
  }

  {
    uint16_t before = count;
    uint16_t last_seq_before;
    last_seq_before = entries[count - 1U].sequence;

    s_state[PX4LITE_MODULE_GNSS] = PX4LITE_STATE_OFFLINE;
    BumpVersion();
    Advance(&now, 2000U, 100U);

    count = App_MessageLogCopy(entries, APP_LOGMSG_COUNT, 0, 0);
    if (count < before) { printf("FAIL count shrank\n"); g_fail++; }
    Eq("gps lost appended", entries[count - 1U].message_id, APP_LOGMSG_GPS_LOST);
    if (entries[count - 1U].sequence <= last_seq_before) { printf("FAIL new seq not greater\n"); g_fail++; }
  }
}

static void TestDebounceSwallowsTransient(void)
{
  Px4Lite_LogEntry_t entries[APP_LOGMSG_COUNT];
  uint16_t baseline;
  uint32_t now = 50000U;

  AllOnline(now);
  s_now_ms = now;
  App_MessageLogInit(now);
  App_MessageLogUpdate(now);
  Advance(&now, 2000U, 100U);
  baseline = App_MessageLogCopy(entries, APP_LOGMSG_COUNT, 0, 0);

  /* 抖动：掉线仅 500ms(< 1500ms 去抖窗口)即恢复，不应追加新条目。 */
  s_state[PX4LITE_MODULE_BARO] = PX4LITE_STATE_OFFLINE;
  BumpVersion();
  Advance(&now, 500U, 100U);
  s_state[PX4LITE_MODULE_BARO] = PX4LITE_STATE_ONLINE;
  BumpVersion();
  Advance(&now, 2000U, 100U);
  Eq("transient swallowed", App_MessageLogCopy(entries, APP_LOGMSG_COUNT, 0, 0), baseline);
}

int main(void)
{
  TestBootBatchThenGpsLost();
  TestDebounceSwallowsTransient();
  if (g_fail == 0) { printf("PASS test_app_message_log\n"); return 0; }
  printf("FAIL test_app_message_log fails=%d\n", g_fail);
  return 1;
}
