/**
 * @file test_mavlink_tx_module_status.c
 * @brief 验证远程模块状态扩展不会把 Control 状态误当作电机存在检测。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "px4lite_mavlink_tx.h"
#include "px4lite_platform.h"
#include "px4lite_topics.h"
#include "px4lite_alarm.h"
#include "px4lite_time.h"

#if defined(__GNUC__)
#define MAVLINK_ALIGNED_FIELDS 0
#endif
#include "common/mavlink.h"

static Px4Lite_SystemHealth_t s_health;
static uint8_t s_last_tx[300];
static uint16_t s_last_tx_len;

Px4Lite_Result_t Px4Lite_LoRaSend(const uint8_t *data, uint16_t len)
{
  if ((data == 0) || (len == 0U) || (len > sizeof(s_last_tx))) { return PX4LITE_INVALID_PARAM; }
  memcpy(s_last_tx, data, len);
  s_last_tx_len = len;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_CopyHealth(Px4Lite_SystemHealth_t *data)
{
  if (data == 0) { return PX4LITE_INVALID_PARAM; }
  *data = s_health;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_CopyGnss(Px4Lite_SensorGnss_t *data) { (void)data; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyTime(Px4Lite_TimeSnapshot_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBaro(Px4Lite_SensorBaro_t *data) { (void)data; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyAlarmSnapshot(Px4Lite_AlarmSnapshot_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyNavigation(Px4Lite_VehicleNavigation_t *data) { (void)data; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBattery(Px4Lite_BatteryStatus_t *data) { (void)data; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyMotor(Px4Lite_MotorOutputs_t *data) { (void)data; return PX4LITE_NOT_READY; }

static int DecodeLastMessage(mavlink_message_t *msg)
{
  mavlink_status_t status;
  uint16_t i;

  memset(&status, 0, sizeof(status));
  memset(msg, 0, sizeof(*msg));
  for (i = 0U; i < s_last_tx_len; ++i) {
    if (mavlink_parse_char(MAVLINK_COMM_0, s_last_tx[i], msg, &status) != 0) { return 1; }
  }
  return 0;
}

static int ExpectU32(const char *name, uint32_t actual, uint32_t expected)
{
  if (actual != expected) {
    printf("FAIL %s actual=%lu expected=%lu\n", name, (unsigned long)actual, (unsigned long)expected);
    return 1;
  }
  return 0;
}

static int ExpectName(const char *name, const char actual[10], const char *expected, uint8_t len)
{
  if (memcmp(actual, expected, len) != 0) {
    printf("FAIL %s\n", name);
    return 1;
  }
  return 0;
}

static int ReadNextNamedValue(mavlink_named_value_int_t *named, const char *expected_name, uint8_t expected_len)
{
  mavlink_message_t msg;
  uint8_t attempt;

  for (attempt = 0U; attempt < 20U; ++attempt) {
    s_last_tx_len = 0U;
    (void)Px4Lite_MavlinkTxRun(2000U + attempt);
    if (s_last_tx_len == 0U) { continue; }
    if (DecodeLastMessage(&msg) == 0) { continue; }
    if (msg.msgid != MAVLINK_MSG_ID_NAMED_VALUE_INT) { continue; }
    mavlink_msg_named_value_int_decode(&msg, named);
    if (memcmp(named->name, expected_name, expected_len) == 0) { return 1; }
  }
  return 0;
}

static int TestRemoteModuleStatusKeepsMotorOnline(void)
{
  mavlink_named_value_int_t named;
  uint32_t packed;
  int failures = 0;

  memset(&s_health, 0, sizeof(s_health));
  s_health.header.valid = 1U;
  s_health.header.sample_time_ms = 2000U;
  s_health.module_state[PX4LITE_MODULE_GNSS] = PX4LITE_STATE_ONLINE;
  s_health.module_state[PX4LITE_MODULE_IMU] = PX4LITE_STATE_ONLINE;
  s_health.module_state[PX4LITE_MODULE_BARO] = PX4LITE_STATE_ONLINE;
  s_health.module_state[PX4LITE_MODULE_5G] = PX4LITE_STATE_DISABLED;
  s_health.module_state[PX4LITE_MODULE_STORAGE] = PX4LITE_STATE_ONLINE;
  s_health.module_state[PX4LITE_MODULE_CONTROL] = PX4LITE_STATE_STARTING;
  s_health.not_ready_mask = (1UL << PX4LITE_MODULE_CONTROL);

  failures += ExpectU32("tx init", Px4Lite_MavlinkTxInit(1000U), PX4LITE_OK);
  failures += ExpectU32("read modstat", (uint32_t)ReadNextNamedValue(&named, "MODSTAT", 7U), 1U);
  failures += ExpectName("modstat name", named.name, "MODSTAT", 7U);

  packed = (uint32_t)named.value;
  failures += ExpectU32("modstat motor state", (packed >> 20U) & 0x0FU, PX4LITE_STATE_ONLINE);
  return failures;
}

int main(void)
{
  int failures = 0;

  failures += TestRemoteModuleStatusKeepsMotorOnline();

  if (failures != 0) {
    printf("mavlink tx module status tests failed: %d\n", failures);
    return 1;
  }

  printf("mavlink tx module status tests passed\n");
  return 0;
}
