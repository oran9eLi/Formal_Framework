/**
 * @file px4lite_mavlink_rx.c
 * @brief 将 LoRa 接收到的 MAVLink 消息解码为远端显示快照。
 *
 * @details
 * CommTask 是本文件唯一调用者。消息按 MAVLink 字段解码后写入 RemoteTelemetry，
 * 不直接发送 C 结构体内存，不执行任何远程控制命令。
 */

#include "px4lite_mavlink_rx.h"

#include <string.h>

#include "px4lite_config.h"
#include "px4lite_remote_telemetry.h"

#if defined(__CC_ARM)
#define MAVLINK_ALIGNED_FIELDS   0
#define MAVLINK_COMM_NUM_BUFFERS 1
#pragma diag_suppress 66
#endif

#include "common/mavlink.h"

#if defined(__CC_ARM)
#pragma diag_default 66
#endif

#define MAVLINK_RX_RAD_TO_DEG100 5729.57795131f

static Px4Lite_MavlinkRxStats_t s_stats;

static int32_t MavlinkRx_RadToDeg100(float value)
{
  float scaled = value * MAVLINK_RX_RAD_TO_DEG100;
  return (scaled >= 0.0f) ? (int32_t)(scaled + 0.5f) : (int32_t)(scaled - 0.5f);
}

static void MavlinkRx_LoadMessage(const Px4Lite_LoRaRxFrame_t *frame, mavlink_message_t *msg)
{
  memset(msg, 0, sizeof(*msg));
  msg->len    = frame->payload_len;
  msg->sysid  = frame->system_id;
  msg->compid = frame->component_id;
  msg->seq    = frame->sequence;
  msg->msgid  = frame->msg_id;
  memcpy(_MAV_PAYLOAD_NON_CONST(msg), frame->data, frame->payload_len);
}

static Px4Lite_Result_t MavlinkRx_HandleHeartbeat(const mavlink_message_t *msg, uint32_t now_ms)
{
  mavlink_heartbeat_t heartbeat;
  Px4Lite_RemoteTelemetrySnapshot_t *snapshot = Px4Lite_RemoteTelemetryMutable();

  mavlink_msg_heartbeat_decode(msg, &heartbeat);
  snapshot->mav_type      = heartbeat.type;
  snapshot->base_mode     = heartbeat.base_mode;
  snapshot->system_status = heartbeat.system_status;
  Px4Lite_RemoteTelemetryCommit(msg->sysid, msg->compid, PX4LITE_REMOTE_VALID_IDENTITY | PX4LITE_REMOTE_VALID_STATUS, now_ms);
  return PX4LITE_OK;
}

static Px4Lite_Result_t MavlinkRx_HandleAttitude(const mavlink_message_t *msg, uint32_t now_ms)
{
  mavlink_attitude_t attitude;
  Px4Lite_RemoteTelemetrySnapshot_t *snapshot = Px4Lite_RemoteTelemetryMutable();

  mavlink_msg_attitude_decode(msg, &attitude);
  snapshot->roll_deg100       = MavlinkRx_RadToDeg100(attitude.roll);
  snapshot->pitch_deg100      = MavlinkRx_RadToDeg100(attitude.pitch);
  snapshot->yaw_deg100        = MavlinkRx_RadToDeg100(attitude.yaw);
  snapshot->roll_rate_dps100  = MavlinkRx_RadToDeg100(attitude.rollspeed);
  snapshot->pitch_rate_dps100 = MavlinkRx_RadToDeg100(attitude.pitchspeed);
  snapshot->yaw_rate_dps100   = MavlinkRx_RadToDeg100(attitude.yawspeed);
  Px4Lite_RemoteTelemetryCommit(msg->sysid, msg->compid, PX4LITE_REMOTE_VALID_ATTITUDE, now_ms);
  return PX4LITE_OK;
}

static Px4Lite_Result_t MavlinkRx_HandleGpsRaw(const mavlink_message_t *msg, uint32_t now_ms)
{
  mavlink_gps_raw_int_t gps;
  Px4Lite_RemoteTelemetrySnapshot_t *snapshot = Px4Lite_RemoteTelemetryMutable();

  mavlink_msg_gps_raw_int_decode(msg, &gps);
  snapshot->latitude_e7     = gps.lat;
  snapshot->longitude_e7    = gps.lon;
  snapshot->altitude_mm     = gps.alt;
  snapshot->hdop_x100       = gps.eph;
  snapshot->satellites_used = gps.satellites_visible;
  snapshot->gnss_fix_type   = gps.fix_type;
  Px4Lite_RemoteTelemetryCommit(msg->sysid, msg->compid, PX4LITE_REMOTE_VALID_NAVIGATION, now_ms);
  return PX4LITE_OK;
}

static Px4Lite_Result_t MavlinkRx_HandleNamedValueInt(const mavlink_message_t *msg, uint32_t now_ms)
{
  mavlink_named_value_int_t named;
  Px4Lite_RemoteTelemetrySnapshot_t *snapshot = Px4Lite_RemoteTelemetryMutable();
  uint32_t packed;
  uint16_t satellites_used;

  mavlink_msg_named_value_int_decode(msg, &named);
  if (memcmp(named.name, "GNSS_SAT", 8U) != 0) { return PX4LITE_IDLE; }

  packed           = (uint32_t)named.value;
  satellites_used  = (uint16_t)(((packed >> 16U) & 0xFFU) + ((packed >> 24U) & 0xFFU));
  snapshot->satellites_used = (satellites_used > 255U) ? 255U : (uint8_t)satellites_used;
  Px4Lite_RemoteTelemetryCommit(msg->sysid, msg->compid, PX4LITE_REMOTE_VALID_NAVIGATION, now_ms);
  return PX4LITE_OK;
}

static Px4Lite_Result_t MavlinkRx_HandleGlobalPosition(const mavlink_message_t *msg, uint32_t now_ms)
{
  mavlink_global_position_int_t pos;
  Px4Lite_RemoteTelemetrySnapshot_t *snapshot = Px4Lite_RemoteTelemetryMutable();

  mavlink_msg_global_position_int_decode(msg, &pos);
  snapshot->latitude_e7         = pos.lat;
  snapshot->longitude_e7        = pos.lon;
  snapshot->altitude_mm         = pos.relative_alt;
  snapshot->velocity_north_cms  = pos.vx;
  snapshot->velocity_east_cms   = pos.vy;
  snapshot->velocity_down_cms   = pos.vz;
  if ((snapshot->valid_mask & PX4LITE_REMOTE_VALID_ATTITUDE) == 0U) { snapshot->yaw_deg100 = pos.hdg; }
  Px4Lite_RemoteTelemetryCommit(msg->sysid, msg->compid, PX4LITE_REMOTE_VALID_NAVIGATION, now_ms);
  return PX4LITE_OK;
}

static Px4Lite_Result_t MavlinkRx_HandleSysStatus(const mavlink_message_t *msg, uint32_t now_ms)
{
  mavlink_sys_status_t sys;
  Px4Lite_RemoteTelemetrySnapshot_t *snapshot = Px4Lite_RemoteTelemetryMutable();

  mavlink_msg_sys_status_decode(msg, &sys);
  snapshot->voltage_mv      = sys.voltage_battery;
  snapshot->current_ma      = sys.current_battery;
  snapshot->battery_percent = (sys.battery_remaining < 0) ? 0U : (uint8_t)sys.battery_remaining;
  Px4Lite_RemoteTelemetryCommit(msg->sysid, msg->compid, PX4LITE_REMOTE_VALID_POWER | PX4LITE_REMOTE_VALID_STATUS, now_ms);
  return PX4LITE_OK;
}

static Px4Lite_Result_t MavlinkRx_HandleBatteryStatus(const mavlink_message_t *msg, uint32_t now_ms)
{
  mavlink_battery_status_t battery;
  Px4Lite_RemoteTelemetrySnapshot_t *snapshot = Px4Lite_RemoteTelemetryMutable();
  uint32_t total_mv = 0U;
  uint8_t cells     = 0U;
  uint8_t i;

  mavlink_msg_battery_status_decode(msg, &battery);
  for (i = 0U; i < 10U; ++i) {
    if ((battery.voltages[i] == UINT16_MAX) || (battery.voltages[i] == 0U)) { continue; }
    total_mv += battery.voltages[i];
    cells++;
  }
  if (cells != 0U) { snapshot->voltage_mv = total_mv; }
  snapshot->current_ma      = battery.current_battery * 10;
  snapshot->battery_percent = (battery.battery_remaining < 0) ? 0U : (uint8_t)battery.battery_remaining;
  Px4Lite_RemoteTelemetryCommit(msg->sysid, msg->compid, PX4LITE_REMOTE_VALID_POWER, now_ms);
  return PX4LITE_OK;
}

static Px4Lite_Result_t MavlinkRx_HandleScaledPressure(const mavlink_message_t *msg, uint32_t now_ms)
{
  mavlink_scaled_pressure_t pressure;
  Px4Lite_RemoteTelemetrySnapshot_t *snapshot = Px4Lite_RemoteTelemetryMutable();

  mavlink_msg_scaled_pressure_decode(msg, &pressure);
  snapshot->pressure_pa   = pressure.press_abs * 100.0f;
  snapshot->temperature_c = ((float)pressure.temperature) / 100.0f;
  Px4Lite_RemoteTelemetryCommit(msg->sysid, msg->compid, PX4LITE_REMOTE_VALID_ENVIRONMENT, now_ms);
  return PX4LITE_OK;
}

static Px4Lite_Result_t MavlinkRx_HandleStatustext(const mavlink_message_t *msg, uint32_t now_ms)
{
  mavlink_statustext_t text;
  Px4Lite_RemoteTelemetrySnapshot_t *snapshot = Px4Lite_RemoteTelemetryMutable();

  mavlink_msg_statustext_decode(msg, &text);
  memcpy(snapshot->status_text, text.text, sizeof(snapshot->status_text) - 1U);
  snapshot->status_text[sizeof(snapshot->status_text) - 1U] = '\0';
  Px4Lite_RemoteTelemetryCommit(msg->sysid, msg->compid, PX4LITE_REMOTE_VALID_TEXT, now_ms);
  return PX4LITE_OK;
}

void Px4Lite_MavlinkRxInit(uint32_t now_ms)
{
  (void)now_ms;
  memset(&s_stats, 0, sizeof(s_stats));
}

Px4Lite_Result_t Px4Lite_MavlinkRxHandleFrame(const Px4Lite_LoRaRxFrame_t *frame, uint32_t now_ms)
{
  mavlink_message_t msg;
  Px4Lite_Result_t result;

  if (frame == 0) {
    s_stats.invalid_count++;
    return PX4LITE_INVALID_PARAM;
  }

  if (frame->payload_len > PX4LITE_LORA_RX_PAYLOAD_MAX) {
    s_stats.invalid_count++;
    return PX4LITE_INVALID_PARAM;
  }

  if (Px4Lite_RemoteTelemetryAcceptSysId(frame->system_id) == 0U) {
    s_stats.filtered_count++;
    Px4Lite_RemoteTelemetryRecordFiltered();
    return (Px4Lite_RemoteTelemetryGetMode() == PX4LITE_REMOTE_MODE_REMOTE) ? PX4LITE_OK : PX4LITE_IDLE;
  }

  MavlinkRx_LoadMessage(frame, &msg);
  switch (frame->msg_id) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      result = MavlinkRx_HandleHeartbeat(&msg, now_ms);
      break;
    case MAVLINK_MSG_ID_GPS_RAW_INT:
      result = MavlinkRx_HandleGpsRaw(&msg, now_ms);
      break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      result = MavlinkRx_HandleGlobalPosition(&msg, now_ms);
      break;
    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
      result = MavlinkRx_HandleNamedValueInt(&msg, now_ms);
      break;
    case MAVLINK_MSG_ID_ATTITUDE:
      result = MavlinkRx_HandleAttitude(&msg, now_ms);
      break;
    case MAVLINK_MSG_ID_SYS_STATUS:
      result = MavlinkRx_HandleSysStatus(&msg, now_ms);
      break;
    case MAVLINK_MSG_ID_BATTERY_STATUS:
      result = MavlinkRx_HandleBatteryStatus(&msg, now_ms);
      break;
    case MAVLINK_MSG_ID_SCALED_PRESSURE:
      result = MavlinkRx_HandleScaledPressure(&msg, now_ms);
      break;
    case MAVLINK_MSG_ID_STATUSTEXT:
      result = MavlinkRx_HandleStatustext(&msg, now_ms);
      break;
    default:
      s_stats.unsupported_count++;
      Px4Lite_RemoteTelemetryRecordUnsupported();
      return PX4LITE_IDLE;
  }

  if (result == PX4LITE_OK) {
    s_stats.handled_count++;
    s_stats.last_msg_id = frame->msg_id;
    s_stats.last_sysid  = frame->system_id;
    s_stats.last_compid = frame->component_id;
  } else if (result == PX4LITE_IDLE) {
    s_stats.unsupported_count++;
    Px4Lite_RemoteTelemetryRecordUnsupported();
  }
  return result;
}

void Px4Lite_MavlinkRxGetStats(Px4Lite_MavlinkRxStats_t *out)
{
  if (out == 0) { return; }
  *out = s_stats;
}




