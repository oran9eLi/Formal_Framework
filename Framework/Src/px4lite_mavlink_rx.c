/**
 * @file px4lite_mavlink_rx.c
 * @brief 将通信层接收的 MAVLink 帧解码为远端遥测快照。
 *
 * @details
 * Comm task 是本文件唯一写者。LoRa 驱动只提供 MAVLink 帧事实，本文件按标准
 * MAVLink message id 逐字段解码，禁止直接发送或解释 C 结构体内存。
 */

#include "px4lite_mavlink_rx.h"

#include <limits.h>
#include <string.h>

#include "px4lite_config.h"
#include "px4lite_command.h"
#include "px4lite_mavlink_tx.h"
#include "px4lite_modules.h"
#include "px4lite_remote_telemetry.h"
#include "px4lite_remote_tunnel.h"

#if defined(__CC_ARM)
#define MAVLINK_ALIGNED_FIELDS   0
#define MAVLINK_COMM_NUM_BUFFERS 1
#pragma diag_suppress 66
#endif

#include "common/mavlink.h"

#if defined(__CC_ARM)
#pragma diag_default 66
#endif

#define MAV_RX_RAD_TO_DEG100 5729.57795f
#define PX4LITE_RX_STREAM_COMMAND ((uint16_t)MAV_CMD_USER_1)
#define PX4LITE_RX_NODE_POLL_COMMAND ((uint16_t)MAV_CMD_USER_2)

static uint8_t MavRx_NameEquals(const char name[10], const char *literal);

static Px4Lite_CommRxFrame_t s_rx_frame_scratch;
static mavlink_message_t s_rx_message_scratch;
static Px4Lite_RemoteTelemetry_t s_rx_remote_scratch;
static Px4Lite_LogEntry_t s_rx_log_scratch[PX4LITE_LOCAL_LOG_CAP];

static int32_t MavRx_FloatToInt32(float value)
{
  if (value >= 0.0f) { return (int32_t)(value + 0.5f); }
  return (int32_t)(value - 0.5f);
}

static uint16_t MavRx_CalcLossRateX10(uint32_t lost_count, uint32_t expected_count)
{
  uint64_t scaled;

  if (expected_count == 0U) { return 0U; }
  if (expected_count < PX4LITE_MAVLINK_LOSS_MIN_EXPECTED) { return 0U; }

  scaled = (((uint64_t)lost_count * 1000ULL) + ((uint64_t)expected_count / 2ULL)) / (uint64_t)expected_count;
  if (scaled > 1000ULL) { return 1000U; }
  return (uint16_t)scaled;
}

static void MavRx_UpdateSequenceStats(Px4Lite_RemoteTelemetry_t *remote, const Px4Lite_CommRxFrame_t *frame)
{
  uint8_t delta;

  if ((remote == 0) || (frame == 0)) { return; }

  if (remote->sequence_seen == 0U) {
    remote->sequence_seen = 1U;
    remote->last_packet_sequence = frame->sequence;
  } else {
    delta = (uint8_t)(frame->sequence - remote->last_packet_sequence);
    if ((delta != 0U) && (delta < 128U)) { remote->rx_sequence_lost_count += (uint32_t)(delta - 1U); }
    if (delta != 0U) { remote->last_packet_sequence = frame->sequence; }
  }

  remote->rx_sequence_expected_count = remote->rx_frame_count + remote->rx_sequence_lost_count;
  remote->rx_loss_rate_x10 = MavRx_CalcLossRateX10(remote->rx_sequence_lost_count, remote->rx_sequence_expected_count);
}

static uint8_t MavRx_FrameToRemoteNode(const Px4Lite_CommRxFrame_t *frame, uint8_t *node_id_out)
{
  uint8_t node_id;

  if ((frame == 0) || (node_id_out == 0) || (frame->system_id == 0U)) { return 0U; }
  node_id = frame->system_id;
  if (node_id == (uint8_t)PX4LITE_NODE_ID) { return 0U; }
  if (node_id >= PX4LITE_REMOTE_NODE_MAX) { return 0U; }
  *node_id_out = node_id;
  return 1U;
}

static uint8_t MavRx_IsLoRaSummaryMessage(const mavlink_message_t *message)
{
  mavlink_named_value_int_t packet;

  if ((message == 0) || (message->msgid != MAVLINK_MSG_ID_NAMED_VALUE_INT)) { return 0U; }
  mavlink_msg_named_value_int_decode(message, &packet);
  return MavRx_NameEquals(packet.name, "LORASUM");
}

static uint8_t MavRx_IsFullDataMessage(const mavlink_message_t *message)
{
  if (message == 0) { return 0U; }
  switch (message->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
    case MAVLINK_MSG_ID_COMMAND_LONG:
    case MAVLINK_MSG_ID_COMMAND_ACK:
      return 0U;
    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
      return (MavRx_IsLoRaSummaryMessage(message) == 0U) ? 1U : 0U;
    default:
      return 1U;
  }
}

static uint8_t MavRx_NameEquals(const char name[10], const char *literal)
{
  uint8_t i;

  if ((name == 0) || (literal == 0)) { return 0U; }
  for (i = 0U; i < 10U; i++) {
    if (literal[i] == '\0') { return 1U; }
    if (name[i] != literal[i]) { return 0U; }
  }
  return (literal[10] == '\0') ? 1U : 0U;
}

static void MavRx_MessageFromFrame(const Px4Lite_CommRxFrame_t *frame, mavlink_message_t *message)
{
  uint8_t payload_len;

  memset(message, 0, sizeof(*message));
  payload_len = frame->payload_len;
  if (payload_len > MAVLINK_MAX_PAYLOAD_LEN) { payload_len = MAVLINK_MAX_PAYLOAD_LEN; }
  message->msgid  = frame->msg_id;
  message->sysid  = frame->system_id;
  message->compid = frame->component_id;
  message->seq    = frame->sequence;
  message->len    = payload_len;
  memcpy(_MAV_PAYLOAD_NON_CONST(message), frame->payload, payload_len);
}

static void MavRx_UpdateSource(Px4Lite_RemoteTelemetry_t *remote, const Px4Lite_CommRxFrame_t *frame)
{
  remote->last_rx_ms     = frame->rx_time_ms;
  remote->last_msg_id    = frame->msg_id;
  remote->system_id      = frame->system_id;
  remote->component_id   = frame->component_id;
  remote->rx_frame_count = remote->rx_frame_count + 1U;
  MavRx_UpdateSequenceStats(remote, frame);
}

static void MavRx_UpdateStaleMask(Px4Lite_RemoteTelemetry_t *remote, uint32_t now_ms)
{
  uint32_t stale = 0U;

  if (remote == 0) { return; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_HEARTBEAT) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->heartbeat_update_ms) > PX4LITE_REMOTE_HEARTBEAT_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_HEARTBEAT; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_NAVIGATION) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->navigation_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_NAVIGATION; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_ATTITUDE) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->attitude_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_ATTITUDE; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_ENVIRONMENT) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->environment_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_ENVIRONMENT; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_BATTERY) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->battery_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_BATTERY; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_MODULES) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->modules_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_MODULES; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_ALARM) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->alarm_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_ALARM; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_LOG) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->log_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_LOG; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_MOTOR) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->motor_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_MOTOR; }
  remote->stale_mask = stale;
}

static void MavRx_MarkHeartbeatSeen(Px4Lite_RemoteTelemetry_t *remote)
{
  if (remote == 0) { return; }
  remote->valid_mask |= PX4LITE_REMOTE_VALID_HEARTBEAT;
  remote->heartbeat_update_ms = remote->last_rx_ms;
}

static uint8_t MavRx_DecodeHeartbeat(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_heartbeat_t packet;

  if ((remote == 0) || (message == 0)) { return 0U; }

  mavlink_msg_heartbeat_decode(message, &packet);
  remote->heartbeat_type            = packet.type;
  remote->heartbeat_autopilot       = packet.autopilot;
  remote->heartbeat_base_mode       = packet.base_mode;
  remote->heartbeat_system_status   = packet.system_status;
  remote->heartbeat_mavlink_version = packet.mavlink_version;
  MavRx_MarkHeartbeatSeen(remote);
  return 1U;
}

static uint8_t MavRx_DecodeGpsRaw(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_gps_raw_int_t packet;

  mavlink_msg_gps_raw_int_decode(message, &packet);
  remote->latitude_e7     = packet.lat;
  remote->longitude_e7    = packet.lon;
  remote->altitude_mm     = packet.alt;
  remote->gnss_fix_type   = packet.fix_type;
  remote->hdop_x100       = (packet.eph != UINT16_MAX) ? packet.eph : 0U;
  remote->satellites_used = (packet.satellites_visible != UINT8_MAX) ? packet.satellites_visible : 0U;
  if ((packet.cog != UINT16_MAX) && (packet.cog <= 36000U)) { remote->yaw_deg100 = (int32_t)packet.cog; }
  remote->valid_mask |= PX4LITE_REMOTE_VALID_NAVIGATION;
  remote->navigation_update_ms = remote->last_rx_ms;
  return 1U;
}

static uint8_t MavRx_DecodeAttitude(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_attitude_t packet;

  mavlink_msg_attitude_decode(message, &packet);
  remote->roll_deg100       = MavRx_FloatToInt32(packet.roll * MAV_RX_RAD_TO_DEG100);
  remote->pitch_deg100      = MavRx_FloatToInt32(packet.pitch * MAV_RX_RAD_TO_DEG100);
  remote->yaw_deg100        = MavRx_FloatToInt32(packet.yaw * MAV_RX_RAD_TO_DEG100);
  remote->roll_rate_dps100  = MavRx_FloatToInt32(packet.rollspeed * MAV_RX_RAD_TO_DEG100);
  remote->pitch_rate_dps100 = MavRx_FloatToInt32(packet.pitchspeed * MAV_RX_RAD_TO_DEG100);
  remote->yaw_rate_dps100   = MavRx_FloatToInt32(packet.yawspeed * MAV_RX_RAD_TO_DEG100);
  remote->valid_mask |= PX4LITE_REMOTE_VALID_ATTITUDE;
  remote->attitude_update_ms = remote->last_rx_ms;
  return 1U;
}

static uint8_t MavRx_DecodePosition(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_global_position_int_t packet;

  mavlink_msg_global_position_int_decode(message, &packet);
  remote->latitude_e7        = packet.lat;
  remote->longitude_e7       = packet.lon;
  remote->altitude_mm        = packet.alt;
  remote->velocity_north_cms = packet.vx;
  remote->velocity_east_cms  = packet.vy;
  remote->velocity_down_cms  = packet.vz;
  if (packet.hdg != UINT16_MAX) { remote->yaw_deg100 = (int32_t)packet.hdg; }
  remote->valid_mask |= PX4LITE_REMOTE_VALID_NAVIGATION;
  remote->navigation_update_ms = remote->last_rx_ms;
  return 1U;
}

static uint8_t MavRx_DecodeSysStatus(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_sys_status_t packet;

  mavlink_msg_sys_status_decode(message, &packet);
  if (packet.voltage_battery != UINT16_MAX) {
    remote->voltage_mv = packet.voltage_battery;
    remote->valid_mask |= PX4LITE_REMOTE_VALID_BATTERY;
    remote->battery_update_ms = remote->last_rx_ms;
  }
  if (packet.battery_remaining >= 0) {
    remote->battery_percent = (uint8_t)packet.battery_remaining;
    remote->valid_mask |= PX4LITE_REMOTE_VALID_BATTERY;
    remote->battery_update_ms = remote->last_rx_ms;
  }
  remote->low_voltage = (packet.battery_remaining >= 0 && packet.battery_remaining <= 15) ? 1U : 0U;
  return 1U;
}

static uint8_t MavRx_DecodeBattery(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_battery_status_t packet;

  mavlink_msg_battery_status_decode(message, &packet);
  if (packet.voltages[0] != UINT16_MAX) { remote->voltage_mv = packet.voltages[0]; }
  if (packet.battery_remaining >= 0) { remote->battery_percent = (uint8_t)packet.battery_remaining; }
  remote->low_voltage = (packet.charge_state == (uint8_t)MAV_BATTERY_CHARGE_STATE_LOW) ? 1U : 0U;
  remote->valid_mask |= PX4LITE_REMOTE_VALID_BATTERY;
  remote->battery_update_ms = remote->last_rx_ms;
  return 1U;
}

static uint8_t MavRx_DecodePressure(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_scaled_pressure_t packet;

  mavlink_msg_scaled_pressure_decode(message, &packet);
  remote->pressure_pa           = packet.press_abs * 100.0f;
  remote->temperature_c         = ((float)packet.temperature) / 100.0f;
  remote->relative_humidity_pct = 0.0f;
  remote->valid_mask |= PX4LITE_REMOTE_VALID_ENVIRONMENT;
  remote->environment_update_ms = remote->last_rx_ms;
  return 1U;
}

static void MavRx_ApplyModuleStatePart(Px4Lite_RemoteTelemetry_t *remote, uint8_t part, uint32_t packed)
{
  uint8_t i;

  for (i = 0U; i < 8U; i++) {
    uint8_t module_index = (uint8_t)(part * 8U + i);
    uint8_t state_value  = (uint8_t)((packed >> (i * 4U)) & 0x0FU);

    if (module_index >= (uint8_t)PX4LITE_MODULE_COUNT) { break; }
    if (state_value > (uint8_t)PX4LITE_STATE_DISABLED) { state_value = (uint8_t)PX4LITE_STATE_UNINITIALIZED; }
    remote->module_state[module_index] = (Px4Lite_State_t)state_value;
    remote->module_state_valid_mask |= (1UL << module_index);
  }
  remote->valid_mask |= PX4LITE_REMOTE_VALID_MODULES;
  remote->modules_update_ms = remote->last_rx_ms;
}

static uint8_t MavRx_DecodeNamedValueInt(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message, uint32_t now_ms)
{
  mavlink_named_value_int_t packet;
  uint32_t value;
  uint8_t gps_used;
  uint8_t bds_used;
  uint8_t i;

  mavlink_msg_named_value_int_decode(message, &packet);
  value = (uint32_t)packet.value;

  if (MavRx_NameEquals(packet.name, "LORASUM") != 0U) {
    remote->lora_active_viewer_node_id = (uint8_t)(value & 0xFFU);
    remote->lora_view_lease_id = (uint8_t)((value >> 8U) & 0xFFU);
    remote->lora_view_remaining_s = (uint16_t)((value >> 16U) & 0xFFFFU);
    remote->lora_summary_update_ms = remote->last_rx_ms;
    remote->valid_mask |= PX4LITE_REMOTE_VALID_LORA_SUMMARY;
    Px4Lite_MavlinkHandleLoRaSummary((uint8_t)message->sysid, remote->lora_active_viewer_node_id, remote->lora_view_lease_id, remote->lora_view_remaining_s, now_ms);
    return 1U;
  }

  if (MavRx_NameEquals(packet.name, "GNSS_SAT") != 0U) {
    gps_used = (uint8_t)((value >> 16U) & 0xFFU);
    bds_used = (uint8_t)((value >> 24U) & 0xFFU);
    remote->satellites_used = (uint8_t)(gps_used + bds_used);
    if (remote->satellites_used == 0U) { remote->satellites_used = (uint8_t)((value & 0xFFU) + ((value >> 8U) & 0xFFU)); }
    remote->valid_mask |= PX4LITE_REMOTE_VALID_NAVIGATION;
    remote->navigation_update_ms = remote->last_rx_ms;
    return 1U;
  }

  if (MavRx_NameEquals(packet.name, "MODSTAT0") != 0U) {
    MavRx_ApplyModuleStatePart(remote, 0U, value);
    return 1U;
  }

  if (MavRx_NameEquals(packet.name, "MODSTAT1") != 0U) {
    MavRx_ApplyModuleStatePart(remote, 1U, value);
    return 1U;
  }

  if (MavRx_NameEquals(packet.name, "BAT2STAT") != 0U) {
    remote->voltage2_mv = (uint32_t)(value & 0xFFFFU);
    remote->battery2_percent = (uint8_t)((value >> 16U) & 0xFFU);
    remote->low_voltage2 = (uint8_t)((value >> 24U) & 0x01U);
    remote->valid_mask |= PX4LITE_REMOTE_VALID_BATTERY;
    remote->battery_update_ms = remote->last_rx_ms;
    return 1U;
  }

  if (MavRx_NameEquals(packet.name, "ENVHUM") != 0U) {
    if (value > 1000U) { value = 1000U; }
    remote->relative_humidity_pct = ((float)value) / 10.0f;
    remote->valid_mask |= PX4LITE_REMOTE_VALID_ENVIRONMENT;
    remote->environment_update_ms = remote->last_rx_ms;
    return 1U;
  }

  if (MavRx_NameEquals(packet.name, "MOTORPWM") != 0U) {
    remote->motor_speed_level = 0U;
    for (i = 0U; (i < PX4LITE_MOTOR_COUNT) && (i < 4U); i++) {
      remote->motor_duty_percent[i] = (uint8_t)((value >> (i * 8U)) & 0xFFU);
      if (remote->motor_duty_percent[i] > 100U) { remote->motor_duty_percent[i] = 100U; }
      if (remote->motor_duty_percent[i] > remote->motor_speed_level) { remote->motor_speed_level = remote->motor_duty_percent[i]; }
    }
    remote->valid_mask |= PX4LITE_REMOTE_VALID_MOTOR;
    remote->motor_update_ms = remote->last_rx_ms;
    return 1U;
  }

  return 0U;
}

static uint8_t MavRx_DecodeTunnel(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message, uint32_t now_ms)
{
  mavlink_tunnel_t packet;
  uint8_t count = 0U;
  uint8_t version = 0U;
  uint16_t latest_seq = 0U;
  uint8_t i;

  mavlink_msg_tunnel_decode(message, &packet);
  if ((packet.target_system != 0U) && (packet.target_system != (uint8_t)PX4LITE_MAVLINK_SYSTEM_ID)) { return 0U; }
  if ((packet.target_component != 0U) && (packet.target_component != (uint8_t)PX4LITE_MAVLINK_COMPONENT_ID)) { return 0U; }

  if (packet.payload_type == PX4LITE_TUNNEL_PT_ALARM_TABLE) {
    if (Px4Lite_UnpackAlarmTable(packet.payload, packet.payload_length, now_ms, remote->alarm_records, PX4LITE_REMOTE_ALARM_RECORD_MAX, &count, &version) != PX4LITE_OK) { return 0U; }
    remote->alarm_record_count = count;
    remote->alarm_table_version = version;
    remote->alarm_active_mask = 0U;
    remote->highest_fault_code = 0U;
    remote->highest_source_id = (uint16_t)PX4LITE_MODULE_COUNT;
    remote->highest_severity = 0U;
    for (i = 0U; i < count; i++) {
      if ((remote->alarm_records[i].active != 0U) && (remote->alarm_records[i].source_id < 32U)) { remote->alarm_active_mask |= (1UL << remote->alarm_records[i].source_id); }
      if ((remote->alarm_records[i].active != 0U) && ((remote->highest_fault_code == 0U) || ((uint8_t)remote->alarm_records[i].severity > remote->highest_severity))) {
        remote->highest_fault_code = remote->alarm_records[i].fault_code;
        remote->highest_source_id = remote->alarm_records[i].source_id;
        remote->highest_severity = (uint8_t)remote->alarm_records[i].severity;
      }
    }
    remote->valid_mask |= PX4LITE_REMOTE_VALID_ALARM;
    remote->alarm_update_ms = remote->last_rx_ms;
    return 1U;
  }

  if (packet.payload_type == PX4LITE_TUNNEL_PT_MESSAGE_LOG) {
    if (Px4Lite_UnpackMessageLog(packet.payload, packet.payload_length, s_rx_log_scratch, PX4LITE_LOCAL_LOG_CAP, &count, &latest_seq) != PX4LITE_OK) { return 0U; }
    if (count > PX4LITE_REMOTE_LOG_ENTRY_MAX) { count = PX4LITE_REMOTE_LOG_ENTRY_MAX; }
    remote->log_count = count;
    remote->log_latest_seq = latest_seq;
    for (i = 0U; (i < count) && (i < PX4LITE_REMOTE_LOG_ENTRY_MAX); i++) {
      remote->log_entries[i] = s_rx_log_scratch[i];
    }
    remote->valid_mask |= PX4LITE_REMOTE_VALID_LOG;
    remote->log_update_ms = remote->last_rx_ms;
    return 1U;
  }

  return 0U;
}

static uint8_t MavRx_DecodeStatusText(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_statustext_t packet;

  mavlink_msg_statustext_decode(message, &packet);
  remote->highest_fault_code = packet.id;
  remote->highest_source_id  = (uint16_t)PX4LITE_MODULE_COUNT;
  remote->highest_severity   = packet.severity;
  remote->valid_mask |= PX4LITE_REMOTE_VALID_ALARM;
  remote->alarm_update_ms = remote->last_rx_ms;
  return 1U;
}

static uint8_t MavRx_DecodeCommandLong(const mavlink_message_t *message, uint32_t now_ms)
{
  mavlink_command_long_t packet;

  mavlink_msg_command_long_decode(message, &packet);
  return (Px4Lite_CommandHandleMavlinkLong(packet.command, message->sysid, message->compid, packet.target_system, packet.target_component, packet.param1, packet.param2, packet.param3, packet.param4, now_ms) == PX4LITE_OK) ? 1U : 0U;
}

static uint8_t MavRx_DecodeCommandAck(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message, uint32_t now_ms)
{
  mavlink_command_ack_t packet;
  Px4Lite_Result_t result;

  mavlink_msg_command_ack_decode(message, &packet);
  result = Px4Lite_CommandHandleMavlinkAck(packet.command, packet.result, packet.target_system, packet.target_component, now_ms);
  if ((result == PX4LITE_OK) && (packet.result == (uint8_t)MAV_RESULT_ACCEPTED) &&
      ((packet.command == PX4LITE_RX_NODE_POLL_COMMAND) || (packet.command == PX4LITE_RX_STREAM_COMMAND))) {
    MavRx_MarkHeartbeatSeen(remote);
  }
  return (result == PX4LITE_OK) ? 1U : 0U;
}

static uint8_t MavRx_DecodeMessage(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message, uint32_t now_ms)
{
  switch (message->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      return MavRx_DecodeHeartbeat(remote, message);
    case MAVLINK_MSG_ID_GPS_RAW_INT:
      return MavRx_DecodeGpsRaw(remote, message);
    case MAVLINK_MSG_ID_ATTITUDE:
      return MavRx_DecodeAttitude(remote, message);
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      return MavRx_DecodePosition(remote, message);
    case MAVLINK_MSG_ID_SYS_STATUS:
      return MavRx_DecodeSysStatus(remote, message);
    case MAVLINK_MSG_ID_BATTERY_STATUS:
      return MavRx_DecodeBattery(remote, message);
    case MAVLINK_MSG_ID_SCALED_PRESSURE:
      return MavRx_DecodePressure(remote, message);
    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
      return MavRx_DecodeNamedValueInt(remote, message, now_ms);
    case MAVLINK_MSG_ID_STATUSTEXT:
      return MavRx_DecodeStatusText(remote, message);
    case MAVLINK_MSG_ID_TUNNEL:
      return MavRx_DecodeTunnel(remote, message, now_ms);
    case MAVLINK_MSG_ID_COMMAND_LONG:
      return MavRx_DecodeCommandLong(message, now_ms);
    case MAVLINK_MSG_ID_COMMAND_ACK:
      return MavRx_DecodeCommandAck(remote, message, now_ms);
    default:
      return 0U;
  }
}

Px4Lite_Result_t Px4Lite_MavlinkRxRun(uint32_t now_ms)
{
  uint8_t remote_node_id;
  uint8_t decoded;
  Px4Lite_Result_t copy_result;

  if (Px4Lite_CopyCommRxFrame(&s_rx_frame_scratch) != PX4LITE_OK) { return PX4LITE_IDLE; }
  if (s_rx_frame_scratch.payload_len > PX4LITE_COMM_RX_PAYLOAD_MAX) { return PX4LITE_OVERFLOW; }
  if (MavRx_FrameToRemoteNode(&s_rx_frame_scratch, &remote_node_id) == 0U) { return PX4LITE_IDLE; }

  MavRx_MessageFromFrame(&s_rx_frame_scratch, &s_rx_message_scratch);
  if ((MavRx_IsFullDataMessage(&s_rx_message_scratch) != 0U) && (Px4Lite_MavlinkShouldAcceptFullFrom(remote_node_id, now_ms) == 0U)) {
    return PX4LITE_IDLE;
  }

  copy_result = Px4Lite_RemoteTelemetryCopyNode(remote_node_id, &s_rx_remote_scratch);
  if (copy_result == PX4LITE_NOT_READY) {
    memset(&s_rx_remote_scratch, 0, sizeof(s_rx_remote_scratch));
  } else if (copy_result != PX4LITE_OK) {
    return copy_result;
  }

  MavRx_UpdateSource(&s_rx_remote_scratch, &s_rx_frame_scratch);
  decoded = MavRx_DecodeMessage(&s_rx_remote_scratch, &s_rx_message_scratch, now_ms);
  if (decoded != 0U) {
    MavRx_UpdateStaleMask(&s_rx_remote_scratch, now_ms);
    s_rx_remote_scratch.decoded_frame_count = s_rx_remote_scratch.decoded_frame_count + 1U;
    s_rx_remote_scratch.header.sample_time_ms  = s_rx_frame_scratch.rx_time_ms;
    s_rx_remote_scratch.header.publish_time_ms = now_ms;
    s_rx_remote_scratch.header.flags           = PX4LITE_DATA_VALID;
    s_rx_remote_scratch.header.device_id       = (uint16_t)(((uint16_t)s_rx_frame_scratch.system_id << 8U) | s_rx_frame_scratch.component_id);
    s_rx_remote_scratch.header.valid           = 1U;
    s_rx_remote_scratch.header.quality         = 100U;
    (void)Px4Lite_RemoteTelemetryCommitNode(remote_node_id, &s_rx_remote_scratch, now_ms);
  }

  return (decoded != 0U) ? PX4LITE_OK : PX4LITE_IDLE;
}
