/**
 * @file test_mavlink_tx_motor_priority.c
 * @brief 验证电机 PWM 变化会优先触发远程电机 MAVLink 扩展发送。
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

static Px4Lite_MotorOutputs_t s_motor;
static uint8_t s_last_tx[300];
static uint16_t s_last_tx_len;

Px4Lite_Result_t Px4Lite_LoRaSend(const uint8_t *data, uint16_t len)
{
  if ((data == 0) || (len == 0U) || (len > sizeof(s_last_tx))) { return PX4LITE_INVALID_PARAM; }
  memcpy(s_last_tx, data, len);
  s_last_tx_len = len;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_CopyMotor(Px4Lite_MotorOutputs_t *data)
{
  if (data == 0) { return PX4LITE_INVALID_PARAM; }
  *data = s_motor;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_CopyGnss(Px4Lite_SensorGnss_t *data) { (void)data; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyTime(Px4Lite_TimeSnapshot_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBaro(Px4Lite_SensorBaro_t *data) { (void)data; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyHealth(Px4Lite_SystemHealth_t *data) { (void)data; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyAlarmSnapshot(Px4Lite_AlarmSnapshot_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyNavigation(Px4Lite_VehicleNavigation_t *data) { (void)data; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBattery(Px4Lite_BatteryStatus_t *data) { (void)data; return PX4LITE_NOT_READY; }

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

static int TestMotorChangePreemptsNormalTelemetry(void)
{
  mavlink_message_t msg;
  mavlink_named_value_int_t named;
  int failures = 0;

  memset(&s_motor, 0, sizeof(s_motor));
  s_motor.header.valid = 1U;
  s_motor.header.sequence = 1U;
  s_motor.header.sample_time_ms = 1000U;
  s_motor.duty_percent[0] = 12U;
  s_motor.duty_percent[1] = 34U;
  s_motor.duty_percent[2] = 56U;
  s_motor.duty_percent[3] = 78U;
  s_motor.run_state = 1U;
  s_motor.speed_level = 78U;
  s_last_tx_len = 0U;

  failures += ExpectU32("tx init", Px4Lite_MavlinkTxInit(1000U), PX4LITE_OK);
  failures += ExpectU32("first tx after motor change", Px4Lite_MavlinkTxRun(1010U), PX4LITE_OK);
  failures += ExpectU32("decode last tx", (uint32_t)DecodeLastMessage(&msg), 1U);
  failures += ExpectU32("motor urgent msg id", msg.msgid, MAVLINK_MSG_ID_NAMED_VALUE_INT);

  mavlink_msg_named_value_int_decode(&msg, &named);
  failures += ExpectName("motor urgent name", named.name, "MOTOR12", 7U);
  failures += ExpectU32("motor urgent value low", (uint32_t)(named.value & 0xFF), 12U);
  failures += ExpectU32("motor urgent value high", (uint32_t)((named.value >> 8) & 0xFF), 34U);
  return failures;
}

static int TestMotor34ChangePreemptsWithMotor34(void)
{
  mavlink_message_t msg;
  mavlink_named_value_int_t named;
  int failures = 0;

  memset(&s_motor, 0, sizeof(s_motor));
  s_motor.header.valid = 1U;
  s_motor.header.sequence = 10U;
  s_motor.header.sample_time_ms = 2000U;
  s_motor.duty_percent[0] = 10U;
  s_motor.duty_percent[1] = 20U;
  s_motor.duty_percent[2] = 30U;
  s_motor.duty_percent[3] = 40U;
  s_motor.run_state = 1U;
  s_motor.speed_level = 80U;
  s_last_tx_len = 0U;

  failures += ExpectU32("tx init motor34", Px4Lite_MavlinkTxInit(2000U), PX4LITE_OK);
  failures += ExpectU32("baseline motor frame", Px4Lite_MavlinkTxRun(2010U), PX4LITE_OK);

  s_motor.header.sequence = 11U;
  s_motor.header.sample_time_ms = 2020U;
  s_motor.duty_percent[2] = 60U;
  s_motor.duty_percent[3] = 70U;
  s_last_tx_len = 0U;

  failures += ExpectU32("motor34 changed tx", Px4Lite_MavlinkTxRun(2020U), PX4LITE_OK);
  failures += ExpectU32("decode motor34 tx", (uint32_t)DecodeLastMessage(&msg), 1U);
  failures += ExpectU32("motor34 urgent msg id", msg.msgid, MAVLINK_MSG_ID_NAMED_VALUE_INT);

  mavlink_msg_named_value_int_decode(&msg, &named);
  failures += ExpectName("motor34 urgent name", named.name, "MOTOR34", 7U);
  failures += ExpectU32("motor34 urgent value low", (uint32_t)(named.value & 0xFF), 60U);
  failures += ExpectU32("motor34 urgent value high", (uint32_t)((named.value >> 8) & 0xFF), 70U);
  return failures;
}

static int TestMotor34ChangeWithSpeedLevelStillPrefersMotor34(void)
{
  mavlink_message_t msg;
  mavlink_named_value_int_t named;
  int failures = 0;

  memset(&s_motor, 0, sizeof(s_motor));
  s_motor.header.valid = 1U;
  s_motor.header.sequence = 20U;
  s_motor.header.sample_time_ms = 3000U;
  s_motor.duty_percent[0] = 10U;
  s_motor.duty_percent[1] = 20U;
  s_motor.duty_percent[2] = 30U;
  s_motor.duty_percent[3] = 40U;
  s_motor.run_state = 1U;
  s_motor.speed_level = 40U; /* 当前最大油门 = duty[3] */
  s_last_tx_len = 0U;

  failures += ExpectU32("tx init speed", Px4Lite_MavlinkTxInit(3000U), PX4LITE_OK);
  failures += ExpectU32("baseline speed frame", Px4Lite_MavlinkTxRun(3010U), PX4LITE_OK);

  /* 拖动 4 号滑块成为新的最大油门：duty[3] 与 speed_level 同时变化。
     回归点：必须仍优先发 MOTOR34，而不是被 speed_level 变化误带回 MOTOR12。 */
  s_motor.header.sequence = 21U;
  s_motor.header.sample_time_ms = 3020U;
  s_motor.duty_percent[3] = 90U;
  s_motor.speed_level = 90U;
  s_last_tx_len = 0U;

  failures += ExpectU32("motor34 speed changed tx", Px4Lite_MavlinkTxRun(3020U), PX4LITE_OK);
  failures += ExpectU32("decode motor34 speed tx", (uint32_t)DecodeLastMessage(&msg), 1U);
  failures += ExpectU32("motor34 speed msg id", msg.msgid, MAVLINK_MSG_ID_NAMED_VALUE_INT);

  mavlink_msg_named_value_int_decode(&msg, &named);
  failures += ExpectName("motor34 speed name", named.name, "MOTOR34", 7U);
  failures += ExpectU32("motor34 speed value low", (uint32_t)(named.value & 0xFF), 30U);
  failures += ExpectU32("motor34 speed value high", (uint32_t)((named.value >> 8) & 0xFF), 90U);
  return failures;
}

int main(void)
{
  int failures = 0;

  failures += TestMotorChangePreemptsNormalTelemetry();
  failures += TestMotor34ChangePreemptsWithMotor34();
  failures += TestMotor34ChangeWithSpeedLevelStillPrefersMotor34();

  if (failures != 0) {
    printf("mavlink tx motor priority tests failed: %d\n", failures);
    return 1;
  }

  printf("mavlink tx motor priority tests passed\n");
  return 0;
}
