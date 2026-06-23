/**
 * @file test_mavlink_rx_remote_telemetry.c
 * @brief 验证 LoRa 远程模式下 MAVLink RX 写入远端显示快照。
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "px4lite_mavlink_rx.h"
#include "px4lite_remote_telemetry.h"
#include "px4lite_platform.h"

#if defined(__GNUC__)
#define MAVLINK_ALIGNED_FIELDS 0
#endif
#include "common/mavlink.h"

static int ExpectU32(const char *name, uint32_t actual, uint32_t expected)
{
  if (actual != expected) {
    printf("FAIL %s actual=%lu expected=%lu\n", name, (unsigned long)actual, (unsigned long)expected);
    return 1;
  }
  return 0;
}

static int ExpectI32(const char *name, int32_t actual, int32_t expected)
{
  if (actual != expected) {
    printf("FAIL %s actual=%ld expected=%ld\n", name, (long)actual, (long)expected);
    return 1;
  }
  return 0;
}

static void FillFrameFromMessage(const mavlink_message_t *msg, Px4Lite_LoRaRxFrame_t *frame)
{
  memset(frame, 0, sizeof(*frame));
  frame->system_id = msg->sysid;
  frame->component_id = msg->compid;
  frame->sequence = msg->seq;
  frame->payload_len = msg->len;
  frame->msg_id = msg->msgid;
  memcpy(frame->data, _MAV_PAYLOAD(msg), msg->len);
}

static int TestRemoteModeFiltersBySysidAndDecodesTelemetry(void)
{
  Px4Lite_LoRaRxFrame_t frame;
  mavlink_message_t msg;
  Px4Lite_RemoteTelemetrySnapshot_t snapshot;
  Px4Lite_RemoteDeviceInfo_t devices[PX4LITE_REMOTE_DEVICE_TABLE_SIZE];
  uint8_t device_count = 0U;
  uint32_t late_ms;
  int failures = 0;

  Px4Lite_RemoteTelemetryInit(1000U);
  Px4Lite_MavlinkRxInit(1000U);
  failures += ExpectU32("default local", Px4Lite_RemoteTelemetryGetMode(), PX4LITE_REMOTE_MODE_LOCAL);
  failures += ExpectU32("default any target", Px4Lite_RemoteTelemetryGetTargetSysId(), PX4LITE_REMOTE_TARGET_SYSID_ANY);
  failures += ExpectU32("reject own target", Px4Lite_RemoteTelemetrySetTargetSysId(PX4LITE_MAVLINK_SYSTEM_ID), PX4LITE_INVALID_PARAM);
  failures += ExpectU32("set remote", Px4Lite_RemoteTelemetrySetMode(PX4LITE_REMOTE_MODE_REMOTE, 1010U), PX4LITE_OK);
  failures += ExpectU32("target sysid", Px4Lite_RemoteTelemetrySetTargetSysId(42U), PX4LITE_OK);

  mavlink_msg_heartbeat_pack(7U, 191U, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, 0U, 0U, MAV_STATE_ACTIVE);
  FillFrameFromMessage(&msg, &frame);
  failures += ExpectU32("ignore other sysid", Px4Lite_MavlinkRxHandleFrame(&frame, 1100U), PX4LITE_OK);
  failures += ExpectU32("other sysid not ready", Px4Lite_RemoteTelemetryCopySnapshot(&snapshot, 1100U), PX4LITE_NOT_READY);
  failures += ExpectU32("device table after heartbeat", Px4Lite_RemoteTelemetryCopyDevices(devices, PX4LITE_REMOTE_DEVICE_TABLE_SIZE, &device_count, 1100U), PX4LITE_OK);
  failures += ExpectU32("device table count", device_count, 1U);
  failures += ExpectU32("discovered sysid", devices[0].sysid, 7U);
  failures += ExpectU32("discovered state", devices[0].state, PX4LITE_REMOTE_DEVICE_DISCOVERED);

  mavlink_msg_heartbeat_pack(42U, 191U, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, 0U, 0U, MAV_STATE_ACTIVE);
  FillFrameFromMessage(&msg, &frame);
  failures += ExpectU32("heartbeat", Px4Lite_MavlinkRxHandleFrame(&frame, 1200U), PX4LITE_OK);

  mavlink_msg_attitude_pack(42U, 191U, &msg, 1200U, 0.10f, -0.20f, 1.50f, 0.01f, -0.02f, 0.03f);
  FillFrameFromMessage(&msg, &frame);
  failures += ExpectU32("attitude", Px4Lite_MavlinkRxHandleFrame(&frame, 1210U), PX4LITE_OK);

  mavlink_msg_global_position_int_pack(42U, 191U, &msg, 1220U, 311234567, 1217654321, 123000, 120000, 110, -220, 30, 9000);
  FillFrameFromMessage(&msg, &frame);
  failures += ExpectU32("position", Px4Lite_MavlinkRxHandleFrame(&frame, 1220U), PX4LITE_OK);

  mavlink_msg_sys_status_pack(42U, 191U, &msg, 0U, 0U, 0U, 500U, 11800U, -1, 83, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  FillFrameFromMessage(&msg, &frame);
  failures += ExpectU32("sys status", Px4Lite_MavlinkRxHandleFrame(&frame, 1230U), PX4LITE_OK);

  mavlink_msg_named_value_int_pack(42U, 191U, &msg, 1240U, "GNSS_SAT", (int32_t)(9U | (11U << 8U) | (7U << 16U) | (8U << 24U)));
  FillFrameFromMessage(&msg, &frame);
  failures += ExpectU32("gnss detail", Px4Lite_MavlinkRxHandleFrame(&frame, 1240U), PX4LITE_OK);

  mavlink_msg_named_value_int_pack(42U, 191U, &msg, 1250U, "TIME_LOC", 153045);
  FillFrameFromMessage(&msg, &frame);
  failures += ExpectU32("remote time", Px4Lite_MavlinkRxHandleFrame(&frame, 1250U), PX4LITE_OK);

  mavlink_msg_named_value_int_pack(42U, 191U, &msg, 1260U, "DATE_LOC", 20260623);
  FillFrameFromMessage(&msg, &frame);
  failures += ExpectU32("remote date", Px4Lite_MavlinkRxHandleFrame(&frame, 1260U), PX4LITE_OK);

  mavlink_msg_named_value_int_pack(42U, 191U, &msg, 1270U, "HUMIDITY", 584);
  FillFrameFromMessage(&msg, &frame);
  failures += ExpectU32("remote humidity", Px4Lite_MavlinkRxHandleFrame(&frame, 1270U), PX4LITE_OK);

  mavlink_msg_named_value_int_pack(42U, 191U, &msg, 1280U, "MOTOR12", (int32_t)(11U | (22U << 8U) | (1U << 16U) | (44U << 24U)));
  FillFrameFromMessage(&msg, &frame);
  failures += ExpectU32("remote motor12", Px4Lite_MavlinkRxHandleFrame(&frame, 1280U), PX4LITE_OK);

  mavlink_msg_named_value_int_pack(42U, 191U, &msg, 1290U, "MOTOR34", (int32_t)(33U | (44U << 8U) | (1U << 16U) | (44U << 24U)));
  FillFrameFromMessage(&msg, &frame);
  failures += ExpectU32("remote motor34", Px4Lite_MavlinkRxHandleFrame(&frame, 1290U), PX4LITE_OK);

  failures += ExpectU32("copy snapshot", Px4Lite_RemoteTelemetryCopySnapshot(&snapshot, 1290U), PX4LITE_OK);
  failures += ExpectU32("source sysid", snapshot.source_sysid, 42U);
  failures += ExpectU32("identity valid", (snapshot.valid_mask & PX4LITE_REMOTE_VALID_IDENTITY) != 0U, 1U);
  failures += ExpectU32("attitude valid", (snapshot.valid_mask & PX4LITE_REMOTE_VALID_ATTITUDE) != 0U, 1U);
  failures += ExpectU32("navigation valid", (snapshot.valid_mask & PX4LITE_REMOTE_VALID_NAVIGATION) != 0U, 1U);
  failures += ExpectU32("power valid", (snapshot.valid_mask & PX4LITE_REMOTE_VALID_POWER) != 0U, 1U);
  failures += ExpectU32("time valid", (snapshot.valid_mask & PX4LITE_REMOTE_VALID_TIME) != 0U, 1U);
  failures += ExpectU32("motor valid", (snapshot.valid_mask & PX4LITE_REMOTE_VALID_MOTOR) != 0U, 1U);
  failures += ExpectI32("latitude", snapshot.latitude_e7, 311234567);
  failures += ExpectI32("longitude", snapshot.longitude_e7, 1217654321);
  failures += ExpectI32("altitude", snapshot.altitude_mm, 120000);
  failures += ExpectI32("roll deg100", snapshot.roll_deg100, 573);
  failures += ExpectI32("pitch deg100", snapshot.pitch_deg100, -1146);
  failures += ExpectI32("yaw deg100", snapshot.yaw_deg100, 8594);
  failures += ExpectU32("voltage", snapshot.voltage_mv, 11800U);
  failures += ExpectU32("battery", snapshot.battery_percent, 83U);
  failures += ExpectU32("satellites used", snapshot.satellites_used, 15U);
  failures += ExpectU32("local time", snapshot.time_hhmmss, 153045U);
  failures += ExpectU32("local date", snapshot.date_ymd, 20260623U);
  failures += ExpectU32("humidity tenths", (uint32_t)((snapshot.relative_humidity_pct * 10.0f) + 0.5f), 584U);
  failures += ExpectU32("motor1", snapshot.motor_duty_percent[0], 11U);
  failures += ExpectU32("motor4", snapshot.motor_duty_percent[3], 44U);
  failures += ExpectU32("motor run", snapshot.motor_run_state, 1U);
  failures += ExpectU32("motor speed", snapshot.motor_speed_level, 44U);

  failures += ExpectU32("stale copy keeps last value", Px4Lite_RemoteTelemetryCopySnapshot(&snapshot, 1290U + PX4LITE_REMOTE_TELEMETRY_TIMEOUT_MS + 1U), PX4LITE_OK);
  failures += ExpectU32("motor remains valid when stale", (snapshot.valid_mask & PX4LITE_REMOTE_VALID_MOTOR) != 0U, 1U);
  failures += ExpectU32("motor stale bit set", (snapshot.stale_mask & PX4LITE_REMOTE_VALID_MOTOR) != 0U, 1U);
  failures += ExpectU32("stale motor value kept", snapshot.motor_duty_percent[3], 44U);

  late_ms = 1290U + PX4LITE_REMOTE_TELEMETRY_TIMEOUT_MS + 100U;
  mavlink_msg_heartbeat_pack(42U, 191U, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, 0U, 0U, MAV_STATE_ACTIVE);
  FillFrameFromMessage(&msg, &frame);
  failures += ExpectU32("fresh heartbeat after field stale", Px4Lite_MavlinkRxHandleFrame(&frame, late_ms), PX4LITE_OK);
  failures += ExpectU32("copy identity only", Px4Lite_RemoteTelemetryCopySnapshot(&snapshot, late_ms), PX4LITE_OK);
  failures += ExpectU32("identity remains after field stale", (snapshot.valid_mask & PX4LITE_REMOTE_VALID_IDENTITY) != 0U, 1U);
  failures += ExpectU32("motor field still displayable", (snapshot.valid_mask & PX4LITE_REMOTE_VALID_MOTOR) != 0U, 1U);
  failures += ExpectU32("motor field remains stale", (snapshot.stale_mask & PX4LITE_REMOTE_VALID_MOTOR) != 0U, 1U);
  return failures;
}

static int TestLocalModeDoesNotConsumeRemoteFrames(void)
{
  Px4Lite_LoRaRxFrame_t frame;
  mavlink_message_t msg;
  Px4Lite_RemoteTelemetrySnapshot_t snapshot;
  int failures = 0;

  Px4Lite_RemoteTelemetryInit(2000U);
  Px4Lite_MavlinkRxInit(2000U);
  mavlink_msg_heartbeat_pack(42U, 191U, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, 0U, 0U, MAV_STATE_ACTIVE);
  FillFrameFromMessage(&msg, &frame);
  failures += ExpectU32("local ignore", Px4Lite_MavlinkRxHandleFrame(&frame, 2010U), PX4LITE_IDLE);
  failures += ExpectU32("local no snapshot", Px4Lite_RemoteTelemetryCopySnapshot(&snapshot, 2010U), PX4LITE_NOT_READY);
  return failures;
}

static int TestModeButtonDebounceTogglesOncePerStablePress(void)
{
  int failures = 0;

  Px4Lite_RemoteTelemetryInit(3000U);
  failures += ExpectU32("button default local", Px4Lite_RemoteTelemetryGetMode(), PX4LITE_REMOTE_MODE_LOCAL);
  failures += ExpectU32("button raw press 1", Px4Lite_RemoteTelemetryUpdateModeButton(1U, 3010U), PX4LITE_REMOTE_MODE_LOCAL);
  failures += ExpectU32("button raw press 2", Px4Lite_RemoteTelemetryUpdateModeButton(1U, 3020U), PX4LITE_REMOTE_MODE_LOCAL);
  failures += ExpectU32("button stable press", Px4Lite_RemoteTelemetryUpdateModeButton(1U, 3030U), PX4LITE_REMOTE_MODE_REMOTE);
  failures += ExpectU32("button hold no repeat", Px4Lite_RemoteTelemetryUpdateModeButton(1U, 3040U), PX4LITE_REMOTE_MODE_REMOTE);
  failures += ExpectU32("button release", Px4Lite_RemoteTelemetryUpdateModeButton(0U, 3050U), PX4LITE_REMOTE_MODE_REMOTE);
  failures += ExpectU32("button second press 1", Px4Lite_RemoteTelemetryUpdateModeButton(1U, 3060U), PX4LITE_REMOTE_MODE_REMOTE);
  failures += ExpectU32("button second press 2", Px4Lite_RemoteTelemetryUpdateModeButton(1U, 3070U), PX4LITE_REMOTE_MODE_REMOTE);
  failures += ExpectU32("button second stable press", Px4Lite_RemoteTelemetryUpdateModeButton(1U, 3080U), PX4LITE_REMOTE_MODE_LOCAL);
  return failures;
}
int main(void)
{
  int failures = 0;

  failures += TestRemoteModeFiltersBySysidAndDecodesTelemetry();
  failures += TestLocalModeDoesNotConsumeRemoteFrames();
  failures += TestModeButtonDebounceTogglesOncePerStablePress();

  if (failures != 0) {
    printf("mavlink rx remote telemetry tests failed: %d\n", failures);
    return 1;
  }

  printf("mavlink rx remote telemetry tests passed\n");
  return 0;
}


