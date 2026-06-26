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

#include "FreeRTOS.h"
#include "task.h"
#include "px4lite_config.h"
#include "px4lite_mavlink_tx.h"
#include "px4lite_modules.h"

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

static Px4Lite_RemoteTelemetry_t s_remote[PX4LITE_REMOTE_NODE_MAX];
static uint32_t s_remote_sequence[PX4LITE_REMOTE_NODE_MAX];
static uint8_t s_selected_remote_node_id = (PX4LITE_NODE_ID == 0U) ? 1U : 0U;

static int32_t MavRx_FloatToInt32(float value)
{
  if (value >= 0.0f) { return (int32_t)(value + 0.5f); }
  return (int32_t)(value - 0.5f);
}

static uint8_t MavRx_NodeIdToIndex(uint8_t node_id, uint8_t *index)
{
  if ((index == 0) || (node_id >= PX4LITE_REMOTE_NODE_MAX)) { return 0U; }
  *index = node_id;
  return 1U;
}

static uint16_t MavRx_CalcLossRateX10(uint32_t lost_count, uint32_t expected_count)
{
  uint64_t scaled;

  if (expected_count == 0U) { return 0U; }

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

static uint8_t MavRx_FrameToRemoteIndex(const Px4Lite_CommRxFrame_t *frame, uint8_t *index)
{
  uint8_t node_id;

  if ((frame == 0) || (index == 0) || (frame->system_id == 0U)) { return 0U; }
  node_id = (uint8_t)(frame->system_id - 1U);
  if (node_id == (uint8_t)PX4LITE_NODE_ID) { return 0U; }
  return MavRx_NodeIdToIndex(node_id, index);
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

static uint32_t MavRx_MaxU32(uint32_t a, uint32_t b)
{
  return (a > b) ? a : b;
}

static uint32_t MavRx_LastMainDataMs(const Px4Lite_RemoteTelemetry_t *remote)
{
  uint32_t last_ms = 0U;

  if (remote == 0) { return 0U; }
  last_ms = MavRx_MaxU32(last_ms, remote->navigation_update_ms);
  last_ms = MavRx_MaxU32(last_ms, remote->attitude_update_ms);
  last_ms = MavRx_MaxU32(last_ms, remote->environment_update_ms);
  last_ms = MavRx_MaxU32(last_ms, remote->battery_update_ms);
  last_ms = MavRx_MaxU32(last_ms, remote->modules_update_ms);
  last_ms = MavRx_MaxU32(last_ms, remote->alarm_update_ms);
  return last_ms;
}

static void MavRx_UpdateStaleMask(Px4Lite_RemoteTelemetry_t *remote, uint32_t now_ms)
{
  uint32_t stale = 0U;

  if (remote == 0) { return; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_HEARTBEAT) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->heartbeat_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_HEARTBEAT; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_NAVIGATION) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->navigation_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_NAVIGATION; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_ATTITUDE) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->attitude_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_ATTITUDE; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_ENVIRONMENT) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->environment_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_ENVIRONMENT; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_BATTERY) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->battery_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_BATTERY; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_MODULES) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->modules_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_MODULES; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_ALARM) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->alarm_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_ALARM; }
  remote->stale_mask = stale;
}

static uint8_t MavRx_DecodeHeartbeat(Px4Lite_RemoteTelemetry_t *remote)
{
  remote->valid_mask |= PX4LITE_REMOTE_VALID_HEARTBEAT;
  remote->heartbeat_update_ms = remote->last_rx_ms;
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
  return 1U;
}

static uint8_t MavRx_DecodeBattery(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_battery_status_t packet;

  mavlink_msg_battery_status_decode(message, &packet);
  if (packet.voltages[0] != UINT16_MAX) { remote->voltage_mv = packet.voltages[0]; }
  if (packet.battery_remaining >= 0) { remote->battery_percent = (uint8_t)packet.battery_remaining; }
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

static uint8_t MavRx_DecodeNamedValueInt(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_named_value_int_t packet;
  uint32_t value;
  uint8_t gps_used;
  uint8_t bds_used;

  mavlink_msg_named_value_int_decode(message, &packet);
  value = (uint32_t)packet.value;

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
  return (Px4Lite_MavlinkHandleCommandLong(packet.command, message->sysid, message->compid, packet.target_system, packet.target_component, packet.param1, packet.param2, packet.param3, packet.param4, now_ms) == PX4LITE_OK) ? 1U : 0U;
}

static uint8_t MavRx_DecodeCommandAck(const mavlink_message_t *message, uint32_t now_ms)
{
  mavlink_command_ack_t packet;

  mavlink_msg_command_ack_decode(message, &packet);
  return (Px4Lite_MavlinkHandleCommandAck(packet.command, packet.result, packet.target_system, packet.target_component, now_ms) == PX4LITE_OK) ? 1U : 0U;
}

static uint8_t MavRx_DecodeMessage(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message, uint32_t now_ms)
{
  switch (message->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      return MavRx_DecodeHeartbeat(remote);
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
      return MavRx_DecodeNamedValueInt(remote, message);
    case MAVLINK_MSG_ID_STATUSTEXT:
      return MavRx_DecodeStatusText(remote, message);
    case MAVLINK_MSG_ID_COMMAND_LONG:
      return MavRx_DecodeCommandLong(message, now_ms);
    case MAVLINK_MSG_ID_COMMAND_ACK:
      return MavRx_DecodeCommandAck(message, now_ms);
    default:
      return 0U;
  }
}

Px4Lite_Result_t Px4Lite_MavlinkRxRun(uint32_t now_ms)
{
  Px4Lite_CommRxFrame_t frame;
  mavlink_message_t message;
  Px4Lite_RemoteTelemetry_t remote;
  uint8_t remote_index;
  uint8_t decoded;

  if (Px4Lite_CopyCommRxFrame(&frame) != PX4LITE_OK) { return PX4LITE_IDLE; }
  if (frame.payload_len > PX4LITE_COMM_RX_PAYLOAD_MAX) { return PX4LITE_OVERFLOW; }
  if (MavRx_FrameToRemoteIndex(&frame, &remote_index) == 0U) { return PX4LITE_IDLE; }

  MavRx_MessageFromFrame(&frame, &message);

  taskENTER_CRITICAL();
  remote = s_remote[remote_index];
  taskEXIT_CRITICAL();

  MavRx_UpdateSource(&remote, &frame);
  decoded = MavRx_DecodeMessage(&remote, &message, now_ms);
  if (decoded != 0U) {
    MavRx_UpdateStaleMask(&remote, now_ms);
    remote.decoded_frame_count = remote.decoded_frame_count + 1U;
    remote.header.sample_time_ms  = frame.rx_time_ms;
    remote.header.publish_time_ms = now_ms;
    remote.header.flags           = PX4LITE_DATA_VALID;
    remote.header.device_id       = (uint16_t)(((uint16_t)frame.system_id << 8U) | frame.component_id);
    remote.header.valid           = 1U;
    remote.header.quality         = 100U;
    taskENTER_CRITICAL();
    remote.header.sequence = ++s_remote_sequence[remote_index];
    s_remote[remote_index] = remote;
    taskEXIT_CRITICAL();
  }

  return (decoded != 0U) ? PX4LITE_OK : PX4LITE_IDLE;
}

Px4Lite_Result_t Px4Lite_CopyRemoteNodeStatuses(Px4Lite_RemoteNodeStatus_t *out, uint8_t capacity, uint8_t *count, uint32_t now_ms)
{
  Px4Lite_RemoteTelemetry_t remote;
  uint8_t i;
  uint8_t written = 0U;
  uint32_t last_data_ms;

  if ((out == 0) || (count == 0)) { return PX4LITE_INVALID_PARAM; }
  *count = 0U;

  taskENTER_CRITICAL();
  for (i = 0U; (i < PX4LITE_REMOTE_NODE_MAX) && (written < capacity); ++i) {
    if (s_remote[i].header.valid == 0U) { continue; }
    remote = s_remote[i];
    taskEXIT_CRITICAL();

    last_data_ms = MavRx_LastMainDataMs(&remote);
    memset(&out[written], 0, sizeof(out[written]));
    out[written].node_id = i;
    out[written].system_id = remote.system_id;
    out[written].component_id = remote.component_id;
    out[written].last_heartbeat_ms = remote.heartbeat_update_ms;
    out[written].last_data_ms = last_data_ms;
    out[written].rx_frame_count = remote.rx_frame_count;
    out[written].rx_sequence_lost_count = remote.rx_sequence_lost_count;
    out[written].rx_loss_rate_x10 = remote.rx_loss_rate_x10;
    if (Px4Lite_ElapsedMs(now_ms, remote.heartbeat_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS) {
      out[written].state = PX4LITE_REMOTE_NODE_STALE;
    } else if ((last_data_ms != 0U) && (Px4Lite_ElapsedMs(now_ms, last_data_ms) <= PX4LITE_REMOTE_DATA_STALE_MS)) {
      out[written].state = PX4LITE_REMOTE_NODE_ACTIVE;
    } else {
      out[written].state = PX4LITE_REMOTE_NODE_DISCOVERED;
    }
    written++;

    taskENTER_CRITICAL();
  }
  taskEXIT_CRITICAL();

  *count = written;
  return (written != 0U) ? PX4LITE_OK : PX4LITE_NOT_READY;
}

Px4Lite_Result_t Px4Lite_CopyRemoteTelemetry(Px4Lite_RemoteTelemetry_t *out)
{
  uint8_t remote_index;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  if (MavRx_NodeIdToIndex(s_selected_remote_node_id, &remote_index) == 0U) {
    memset(out, 0, sizeof(*out));
    return PX4LITE_INVALID_PARAM;
  }

  taskENTER_CRITICAL();
  if (s_remote[remote_index].header.valid == 0U) {
    taskEXIT_CRITICAL();
    memset(out, 0, sizeof(*out));
    return PX4LITE_NOT_READY;
  }
  *out = s_remote[remote_index];
  taskEXIT_CRITICAL();
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_SelectRemoteNode(uint8_t node_id)
{
  uint8_t remote_index;

  if (node_id == (uint8_t)PX4LITE_NODE_ID) { return PX4LITE_INVALID_PARAM; }
  if (MavRx_NodeIdToIndex(node_id, &remote_index) == 0U) { return PX4LITE_INVALID_PARAM; }

  (void)remote_index;
  taskENTER_CRITICAL();
  s_selected_remote_node_id = node_id;
  taskEXIT_CRITICAL();
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_GetSelectedRemoteNode(uint8_t *node_id)
{
  if (node_id == 0) { return PX4LITE_INVALID_PARAM; }

  taskENTER_CRITICAL();
  *node_id = s_selected_remote_node_id;
  taskEXIT_CRITICAL();
  return PX4LITE_OK;
}
