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
#include "px4lite_faults.h"
#include "px4lite_identity.h"
#include "px4lite_mavlink_tx.h"
#include "px4lite_modules.h"
#include "px4lite_platform.h"
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

#define MAV_RX_RAD_TO_DEG100 5729.57795f
#define MAV_RX_SEQUENCE_JUMP_MAX 64U
#define MAV_RX_RPI_CELLULAR_ONLINE_MASK 0x01U

static uint8_t MavRx_NameEquals(const char name[10], const char *literal);

static Px4Lite_CommRxFrame_t s_rx_frame_scratch;
static mavlink_message_t s_rx_message_scratch;
static Px4Lite_RemoteTelemetry_t s_rx_remote_scratch;
static uint32_t s_last_peer_rx_ms;

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

static void MavRx_UpdateSequenceStats(Px4Lite_RemoteTelemetry_t *remote, const Px4Lite_CommRxFrame_t *frame, uint32_t previous_rx_ms)
{
  uint8_t delta;

  if ((remote == 0) || (frame == 0)) { return; }
  (void)previous_rx_ms;

  if (remote->sequence_seen == 0U) {
    remote->sequence_seen = 1U;
    remote->last_packet_sequence = frame->sequence;
  } else {
    delta = (uint8_t)(frame->sequence - remote->last_packet_sequence);
    if (delta != 0U) {
      /* 只按序号跳变幅度判断，不再按静默时长判断：静默再久也不能免罚，
         否则"走远失联很久再回来"这种真实丢包会被当成正常重新同步而永远
         算作 0% 丢包。序号跳变过大(通常是对端序号被重置/重启)才不计入，
         真实的连续丢包(哪怕跨越很长静默时间)必须计数。 */
      if (delta <= MAV_RX_SEQUENCE_JUMP_MAX) {
        remote->rx_sequence_lost_count += (uint32_t)(delta - 1U);
      }
      remote->last_packet_sequence = frame->sequence;
    }
  }

  remote->rx_sequence_expected_count = remote->rx_frame_count + remote->rx_sequence_lost_count;
  remote->rx_loss_rate_x10 = MavRx_CalcLossRateX10(remote->rx_sequence_lost_count, remote->rx_sequence_expected_count);
}

static uint8_t MavRx_FrameToRemoteNode(const Px4Lite_CommRxFrame_t *frame, uint8_t *node_id_out)
{
  uint8_t node_id;

  if ((frame == 0) || (node_id_out == 0) || (frame->system_id == 0U)) { return 0U; }
  node_id = frame->system_id;
  if (node_id == (uint8_t)Px4Lite_IdentityGetNodeId()) { return 0U; }
  /* node_id 为对端 sysid(1..250)，不再按表容量截断；槽位由 RemoteTelemetry 动态分配。 */
  *node_id_out = node_id;
  return 1U;
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

static uint8_t MavRx_TextContains(const char text[50], const char *literal)
{
  uint8_t i;
  uint8_t j;

  if (literal == 0) { return 0U; }
  for (i = 0U; i < 50U; i++) {
    for (j = 0U; (literal[j] != '\0') && ((uint8_t)(i + j) < 50U) && (text[(uint8_t)(i + j)] == literal[j]); j++) {
    }
    if (literal[j] == '\0') { return 1U; }
  }
  return 0U;
}

static uint16_t MavRx_ParseAlarmSource(const char text[50])
{
  if (MavRx_TextContains(text, "GNSS") != 0U) { return (uint16_t)PX4LITE_MODULE_GNSS; }
  if (MavRx_TextContains(text, "IMU") != 0U) { return (uint16_t)PX4LITE_MODULE_IMU; }
  if (MavRx_TextContains(text, "BARO") != 0U) { return (uint16_t)PX4LITE_MODULE_BARO; }
  if (MavRx_TextContains(text, "POWER") != 0U) { return (uint16_t)PX4LITE_MODULE_BATTERY; }
  if (MavRx_TextContains(text, "LORA") != 0U) { return (uint16_t)PX4LITE_MODULE_LORA; }
  if (MavRx_TextContains(text, "DISPLAY") != 0U) { return (uint16_t)PX4LITE_MODULE_DISPLAY; }
  if (MavRx_TextContains(text, "CONTROL") != 0U) { return (uint16_t)PX4LITE_MODULE_CONTROL; }
  if (MavRx_TextContains(text, "SYSTEM") != 0U) { return (uint16_t)PX4LITE_MODULE_SYSTEM; }
  if (MavRx_TextContains(text, "EST") != 0U) { return (uint16_t)PX4LITE_MODULE_ESTIMATOR; }
  if (MavRx_TextContains(text, "SD") != 0U) { return (uint16_t)PX4LITE_MODULE_STORAGE; }
  if (MavRx_TextContains(text, "5G") != 0U) { return (uint16_t)PX4LITE_MODULE_5G; }
  if (MavRx_TextContains(text, "REMOTEID") != 0U) { return (uint16_t)PX4LITE_MODULE_REMOTE_ID; }
  if (MavRx_TextContains(text, "ALARM ALARM") != 0U) { return (uint16_t)PX4LITE_MODULE_ALARM; }
  return (uint16_t)PX4LITE_MODULE_COUNT;
}

static void MavRx_AppendRemoteLog(Px4Lite_RemoteTelemetry_t *remote, const Px4Lite_LogEntry_t *entry)
{
  uint8_t i;

  if ((remote == 0) || (entry == 0) || (entry->sequence == 0U)) { return; }
  for (i = 0U; i < remote->remote_log_count; i++) {
    if (remote->remote_log_entries[i].sequence == entry->sequence) {
      remote->remote_log_entries[i] = *entry;
      remote->remote_log_latest_seq = entry->sequence;
      remote->valid_mask |= PX4LITE_REMOTE_VALID_LOG;
      remote->log_update_ms = remote->last_rx_ms;
      return;
    }
  }

  if (remote->remote_log_count < PX4LITE_LOCAL_LOG_CAP) {
    remote->remote_log_entries[remote->remote_log_count] = *entry;
    remote->remote_log_count++;
  } else {
    for (i = 1U; i < PX4LITE_LOCAL_LOG_CAP; i++) {
      remote->remote_log_entries[(uint8_t)(i - 1U)] = remote->remote_log_entries[i];
    }
    remote->remote_log_entries[(uint8_t)(PX4LITE_LOCAL_LOG_CAP - 1U)] = *entry;
  }

  remote->remote_log_latest_seq = entry->sequence;
  remote->valid_mask |= PX4LITE_REMOTE_VALID_LOG;
  remote->log_update_ms = remote->last_rx_ms;
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
  uint32_t previous_rx_ms;

  previous_rx_ms        = remote->last_rx_ms;
  remote->last_rx_ms     = frame->rx_time_ms;
  remote->last_msg_id    = frame->msg_id;
  remote->system_id      = frame->system_id;
  remote->component_id   = frame->component_id;
  remote->rx_frame_count = remote->rx_frame_count + 1U;
  MavRx_UpdateSequenceStats(remote, frame, previous_rx_ms);
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
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_MOTOR) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->motor_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_MOTOR; }
  if (((remote->valid_mask & PX4LITE_REMOTE_VALID_LOG) != 0U) && (Px4Lite_ElapsedMs(now_ms, remote->log_update_ms) > PX4LITE_REMOTE_DATA_STALE_MS)) { stale |= PX4LITE_REMOTE_VALID_LOG; }
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
  /* current_battery 单位 cA(10mA)，-1 表示未测量；换算回 mA。 */
  if (packet.current_battery >= 0) {
    remote->current_ma = (uint32_t)packet.current_battery * 10U;
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
  /* MAVLink 用 id 区分多块电池：id=1 电池2，其余(id=0)电池1。
     current_battery 单位 cA(10mA)，-1 表示未测量；换算回 mA。 */
  if (packet.id == 1U) {
    if (packet.voltages[0] != UINT16_MAX) { remote->voltage2_mv = packet.voltages[0]; }
    if (packet.current_battery >= 0) { remote->current2_ma = (uint32_t)packet.current_battery * 10U; }
    if (packet.battery_remaining >= 0) { remote->battery2_percent = (uint8_t)packet.battery_remaining; }
    remote->low_voltage2 = (packet.charge_state == (uint8_t)MAV_BATTERY_CHARGE_STATE_LOW) ? 1U : 0U;
  } else {
    if (packet.voltages[0] != UINT16_MAX) { remote->voltage_mv = packet.voltages[0]; }
    if (packet.current_battery >= 0) { remote->current_ma = (uint32_t)packet.current_battery * 10U; }
    if (packet.battery_remaining >= 0) { remote->battery_percent = (uint8_t)packet.battery_remaining; }
    remote->low_voltage = (packet.charge_state == (uint8_t)MAV_BATTERY_CHARGE_STATE_LOW) ? 1U : 0U;
  }
  remote->valid_mask |= PX4LITE_REMOTE_VALID_BATTERY;
  remote->battery_update_ms = remote->last_rx_ms;
  return 1U;
}

static uint8_t MavRx_DecodePressure(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_scaled_pressure_t packet;

  mavlink_msg_scaled_pressure_decode(message, &packet);
  remote->pressure_pa   = packet.press_abs * 100.0f;          /* hPa → Pa */
  remote->temperature_c = ((float)packet.temperature) / 100.0f; /* cdegC → degC */
  /* 湿度不在 SCALED_PRESSURE 里，由独立的 HUMIDITY 帧维护，这里不得清零覆盖。 */
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

static Px4Lite_State_t MavRx_NormalizeState(uint32_t value)
{
  if (value > (uint32_t)PX4LITE_STATE_DISABLED) { return PX4LITE_STATE_UNINITIALIZED; }
  return (Px4Lite_State_t)value;
}

/**
 * @brief 把合法 ESC 脉宽换算为远端显示使用的油门百分比。
 */
static uint8_t MavRx_PulseToDutyPercent(uint16_t pulse_us)
{
  uint32_t range;
  uint32_t offset;

  if (pulse_us <= PX4LITE_CONTROL_ESC_MIN_PULSE_US) { return 0U; }
  if (pulse_us >= PX4LITE_CONTROL_ESC_MAX_PULSE_US) { return 100U; }
  range = (uint32_t)PX4LITE_CONTROL_ESC_MAX_PULSE_US - (uint32_t)PX4LITE_CONTROL_ESC_MIN_PULSE_US;
  offset = (uint32_t)pulse_us - (uint32_t)PX4LITE_CONTROL_ESC_MIN_PULSE_US;
  return (uint8_t)(((offset * 100U) + (range / 2U)) / range);
}

static void MavRx_ApplyCompactModuleState(Px4Lite_RemoteTelemetry_t *remote, uint32_t packed)
{
  if (remote == 0) { return; }

  remote->module_state[PX4LITE_MODULE_GNSS]    = MavRx_NormalizeState(packed & 0x0FUL);
  remote->module_state[PX4LITE_MODULE_IMU]     = MavRx_NormalizeState((packed >> 4U) & 0x0FUL);
  remote->module_state[PX4LITE_MODULE_BARO]    = MavRx_NormalizeState((packed >> 8U) & 0x0FUL);
  remote->module_state[PX4LITE_MODULE_5G]      = MavRx_NormalizeState((packed >> 12U) & 0x0FUL);
  remote->module_state[PX4LITE_MODULE_STORAGE] = MavRx_NormalizeState((packed >> 16U) & 0x0FUL);
  remote->module_state[PX4LITE_MODULE_CONTROL] = MavRx_NormalizeState((packed >> 20U) & 0x0FUL);

  remote->module_state_valid_mask |= (1UL << PX4LITE_MODULE_GNSS);
  remote->module_state_valid_mask |= (1UL << PX4LITE_MODULE_IMU);
  remote->module_state_valid_mask |= (1UL << PX4LITE_MODULE_BARO);
  remote->module_state_valid_mask |= (1UL << PX4LITE_MODULE_5G);
  remote->module_state_valid_mask |= (1UL << PX4LITE_MODULE_STORAGE);
  remote->module_state_valid_mask |= (1UL << PX4LITE_MODULE_CONTROL);
  remote->valid_mask |= PX4LITE_REMOTE_VALID_MODULES;
  remote->modules_update_ms = remote->last_rx_ms;
}

static void MavRx_ApplyMotorPair(Px4Lite_RemoteTelemetry_t *remote, uint8_t pair_part, uint32_t packed)
{
  uint8_t first;
  uint8_t duty0;
  uint8_t duty1;
  uint8_t i;

  if (remote == 0) { return; }
  pair_part = (uint8_t)(pair_part % 2U);
  first = (uint8_t)(pair_part * 2U);
  duty0 = (uint8_t)(packed & 0xFFU);
  duty1 = (uint8_t)((packed >> 8U) & 0xFFU);
  if (duty0 > 100U) { duty0 = 100U; }
  if (duty1 > 100U) { duty1 = 100U; }

  if (first < PX4LITE_MOTOR_COUNT) {
    remote->motor_duty_percent[first] = duty0;
  }
  if ((uint8_t)(first + 1U) < PX4LITE_MOTOR_COUNT) {
    remote->motor_duty_percent[(uint8_t)(first + 1U)] = duty1;
  }
  remote->motor_run_state = (uint8_t)((packed >> 16U) & 0x01U);
  remote->motor_speed_level = (uint8_t)((packed >> 24U) & 0xFFU);
  if (remote->motor_speed_level > 100U) { remote->motor_speed_level = 100U; }
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    if (remote->motor_duty_percent[i] > remote->motor_speed_level) { remote->motor_speed_level = remote->motor_duty_percent[i]; }
  }
  remote->valid_mask |= PX4LITE_REMOTE_VALID_MOTOR;
  remote->motor_update_ms = remote->last_rx_ms;
}

/**
 * @brief 解码主输出组的 SERVO_OUTPUT_RAW 四路 PWM 脉宽。
 */
static uint8_t MavRx_DecodeMotorPulse(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_servo_output_raw_t packet;
  uint16_t pulse_us[PX4LITE_MOTOR_COUNT];
  uint8_t any_valid = 0U;
  uint8_t i;

  if ((remote == 0) || (message == 0)) { return 0U; }
  mavlink_msg_servo_output_raw_decode(message, &packet);
  if (packet.port != 0U) { return 0U; }

  pulse_us[0] = packet.servo1_raw;
  pulse_us[1] = packet.servo2_raw;
  pulse_us[2] = packet.servo3_raw;
  pulse_us[3] = packet.servo4_raw;
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    if ((pulse_us[i] < PX4LITE_CONTROL_ESC_MIN_PULSE_US) || (pulse_us[i] > PX4LITE_CONTROL_ESC_MAX_PULSE_US)) { continue; }
    remote->motor_duty_percent[i] = MavRx_PulseToDutyPercent(pulse_us[i]);
    remote->motor_pulse_us[i]     = pulse_us[i];
    any_valid = 1U;
  }
  if (any_valid == 0U) { return 0U; }

  remote->motor_speed_level = 0U;
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    if (remote->motor_duty_percent[i] > remote->motor_speed_level) { remote->motor_speed_level = remote->motor_duty_percent[i]; }
  }
  remote->valid_mask |= PX4LITE_REMOTE_VALID_MOTOR;
  remote->motor_update_ms = remote->last_rx_ms;
  return 1U;
}

static uint8_t MavRx_DecodeNamedValueInt(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message, uint32_t now_ms)
{
  mavlink_named_value_int_t packet;
  Px4Lite_LogEntry_t log_entry;
  uint32_t value;
  uint8_t gps_used;
  uint8_t bds_used;
  uint8_t i;

  mavlink_msg_named_value_int_decode(message, &packet);
  value = (uint32_t)packet.value;
  (void)now_ms;
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

  if (MavRx_NameEquals(packet.name, "MODSTAT") != 0U) {
    MavRx_ApplyCompactModuleState(remote, value);
    return 1U;
  }

  /* LoRa 按 fj-lora 兼容格式继续接收第二电池摘要 BAT2STAT。 */

  if (MavRx_NameEquals(packet.name, "BAT2STAT") != 0U) {
    remote->voltage2_mv = (uint32_t)(value & 0xFFFFU);
    remote->battery2_percent = (uint8_t)((value >> 16U) & 0xFFU);
    remote->low_voltage2 = (uint8_t)((value >> 24U) & 0x01U);
    remote->valid_mask |= PX4LITE_REMOTE_VALID_BATTERY;
    remote->battery_update_ms = remote->last_rx_ms;
    return 1U;
  }

  if (MavRx_NameEquals(packet.name, "GNSSUTC") != 0U) {
    remote->gnss_utc_date = value;
    remote->gnss_utc_sec = packet.time_boot_ms;
    remote->valid_mask |= PX4LITE_REMOTE_VALID_NAVIGATION;
    remote->navigation_update_ms = remote->last_rx_ms;
    return 1U;
  }

  if (MavRx_NameEquals(packet.name, "BAROALT") != 0U) {
    remote->altitude_mm = (int32_t)value;
    remote->valid_mask |= PX4LITE_REMOTE_VALID_NAVIGATION;
    remote->navigation_update_ms = remote->last_rx_ms;
    return 1U;
  }

  if ((MavRx_NameEquals(packet.name, "HUMIDITY") != 0U) || (MavRx_NameEquals(packet.name, "ENVHUM") != 0U)) {
    if (value > 1000U) { value = 1000U; }
    remote->relative_humidity_pct = ((float)value) / 10.0f;
    remote->valid_mask |= PX4LITE_REMOTE_VALID_ENVIRONMENT;
    remote->environment_update_ms = remote->last_rx_ms;
    return 1U;
  }

  /* 温度、气压已改回官方 SCALED_PRESSURE(msgID 29)，由 MavRx_DecodePressure 解码，
     不再走 BAROTEMP/BAROPRES 自定义 NAMED_VALUE。 */

  /* LoRa 按 fj-lora 兼容格式继续接收告警摘要 ALRMHI/ALRMMSK。 */

  if (MavRx_NameEquals(packet.name, "ALRMHI") != 0U) {
    remote->highest_fault_code = (uint16_t)(value & 0xFFFFU);
    remote->highest_source_id = (uint16_t)((value >> 16U) & 0xFFU);
    remote->highest_severity = (Px4Lite_AlarmSeverity_t)((value >> 24U) & 0x0FU);
    remote->valid_mask |= PX4LITE_REMOTE_VALID_ALARM;
    remote->alarm_update_ms = remote->last_rx_ms;
    return 1U;
  }

  if (MavRx_NameEquals(packet.name, "ALRMMSK") != 0U) {
    remote->alarm_active_mask = value;
    remote->valid_mask |= PX4LITE_REMOTE_VALID_ALARM;
    remote->alarm_update_ms = remote->last_rx_ms;
    return 1U;
  }

  if (MavRx_NameEquals(packet.name, "MOTOR12") != 0U) {
    MavRx_ApplyMotorPair(remote, 0U, value);
    return 1U;
  }

  if (MavRx_NameEquals(packet.name, "MOTOR34") != 0U) {
    MavRx_ApplyMotorPair(remote, 1U, value);
    return 1U;
  }

  if (MavRx_NameEquals(packet.name, "MOTORPWM") != 0U) {
    remote->motor_speed_level = 0U;
    for (i = 0U; (i < PX4LITE_MOTOR_COUNT) && (i < 4U); i++) {
      remote->motor_duty_percent[i] = (uint8_t)((value >> (i * 8U)) & 0xFFU);
      if (remote->motor_duty_percent[i] > 100U) { remote->motor_duty_percent[i] = 100U; }
      if (remote->motor_duty_percent[i] > remote->motor_speed_level) { remote->motor_speed_level = remote->motor_duty_percent[i]; }
    }
    remote->motor_run_state = (uint8_t)(packet.time_boot_ms & 0x01U);
    if (((packet.time_boot_ms >> 8U) & 0xFFU) > remote->motor_speed_level) { remote->motor_speed_level = (uint8_t)((packet.time_boot_ms >> 8U) & 0xFFU); }
    if (remote->motor_speed_level > 100U) { remote->motor_speed_level = 100U; }
    remote->valid_mask |= PX4LITE_REMOTE_VALID_MOTOR;
    remote->motor_update_ms = remote->last_rx_ms;
    return 1U;
  }

  if (MavRx_NameEquals(packet.name, "LOGSYNC") != 0U) {
    memset(&log_entry, 0, sizeof(log_entry));
    log_entry.sequence    = (uint16_t)(value & 0xFFFFU);
    log_entry.message_id  = (uint16_t)((value >> 16U) & 0xFFU);
    log_entry.time_hhmmss = packet.time_boot_ms;
    log_entry.severity    = (uint8_t)((value >> 24U) & 0x0FU);
    log_entry.active      = (uint8_t)((value >> 28U) & 0x01U);
    log_entry.source_id   = (uint8_t)((value >> 29U) & 0x07U);
    MavRx_AppendRemoteLog(remote, &log_entry);
    return 1U;
  }

  return 0U;
}
/**
 * @brief 解码 LoRa 全量告警表 TUNNEL(payload_type=0x8001)。
 * @details
 * 布局与发送端 MavTx_PackAlarmTable 逐字段一致：表头 ver(1)+active_count(1)，每行 7 字节
 * source_id(1)+fault_code(2,LE)+severity(1)+active(1)+age_s(2,LE)。据此重建远端活动位图、
 * 逐来源 fault_code/severity 与最高告警。每帧全量重建，故告警清空(count=0)也能感知。
 * LoRa 上只有 0x8001；0x8002 日志表仅 RPi 用，这里忽略。
 */
static uint8_t MavRx_DecodeTunnel(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_tunnel_t packet;
  uint16_t off;
  uint8_t n;
  uint8_t i;
  uint32_t mask = 0U;
  uint16_t highest_fault = 0U;
  uint16_t highest_src = (uint16_t)PX4LITE_MODULE_COUNT;
  uint8_t highest_sev = 0U;
  uint8_t have_highest = 0U;

  mavlink_msg_tunnel_decode(message, &packet);
  if (packet.payload_type != 0x8001U) { return 0U; }
  if (packet.payload_length < 2U) { return 0U; }

  n   = packet.payload[1];
  off = 2U;
  for (i = 0U; i < n; ++i) {
    uint8_t src;
    uint8_t sev;
    uint16_t fault;

    if ((uint16_t)(off + 7U) > (uint16_t)packet.payload_length) { break; }
    src   = packet.payload[off];
    fault = (uint16_t)((uint16_t)packet.payload[off + 1U] | ((uint16_t)packet.payload[off + 2U] << 8U));
    sev   = packet.payload[off + 3U];
    off   = (uint16_t)(off + 7U);

    if (src >= (uint8_t)PX4LITE_MODULE_COUNT) { continue; }
    remote->alarm_fault_code[src] = fault;
    remote->alarm_severity[src]   = sev;
    if (src < 32U) { mask |= (1UL << src); }
    if ((have_highest == 0U) || (sev > highest_sev)) {
      have_highest  = 1U;
      highest_sev   = sev;
      highest_fault = fault;
      highest_src   = src;
    }
  }

  remote->alarm_active_mask  = mask;
  remote->highest_fault_code = highest_fault;
  remote->highest_source_id  = highest_src;
  remote->highest_severity   = highest_sev;
  remote->valid_mask |= PX4LITE_REMOTE_VALID_ALARM;
  remote->alarm_update_ms = remote->last_rx_ms;
  return 1U;
}

static uint8_t MavRx_DecodeStatusText(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_statustext_t packet;

  mavlink_msg_statustext_decode(message, &packet);
  /* STATUSTEXT 只更新"最高告警"横幅字段；活动位图与逐条详情由 TUNNEL 0x8001 全量表维护，
     这里不得改写 alarm_active_mask，否则会把全量表的完整位图覆盖成"仅最高一条"。 */
  remote->highest_fault_code = packet.id;
  remote->highest_source_id  = MavRx_ParseAlarmSource(packet.text);
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
  (void)remote;
  return (result == PX4LITE_OK) ? 1U : 0U;
}

static uint8_t MavRx_DecodeRpiNamedValueInt(const mavlink_message_t *message, uint32_t now_ms)
{
  mavlink_named_value_int_t packet;
  uint32_t value;
  uint8_t online;

  mavlink_msg_named_value_int_decode(message, &packet);
  if (MavRx_NameEquals(packet.name, "RPICELL") == 0U) { return 0U; }

  value = (uint32_t)packet.value;
  online = ((value & MAV_RX_RPI_CELLULAR_ONLINE_MASK) != 0U) ? 1U : 0U;
  if (online != 0U) {
    Px4Lite_SetExternalModuleState(PX4LITE_MODULE_5G, PX4LITE_STATE_ONLINE, PX4LITE_FAULT_NONE, now_ms);
    Px4Lite_MavlinkSetCellularLinkEnabled(1U);
    Px4Lite_MavlinkSetRpiUplinkEnabled(1U);
  } else {
    Px4Lite_SetExternalModuleState(PX4LITE_MODULE_5G, PX4LITE_STATE_OFFLINE, PX4LITE_FAULT_COMM_OFFLINE, now_ms);
    Px4Lite_MavlinkSetCellularLinkEnabled(0U);
    Px4Lite_MavlinkSetRpiUplinkEnabled(0U);
  }
  return 1U;
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
    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
      return MavRx_DecodeMotorPulse(remote, message);
    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
      return MavRx_DecodeNamedValueInt(remote, message, now_ms);
    case MAVLINK_MSG_ID_TUNNEL:
      return MavRx_DecodeTunnel(remote, message);
    case MAVLINK_MSG_ID_STATUSTEXT:
      return MavRx_DecodeStatusText(remote, message);
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
  copy_result = Px4Lite_RemoteTelemetryCopyNode(remote_node_id, &s_rx_remote_scratch);
  if (copy_result == PX4LITE_NOT_READY) {
    memset(&s_rx_remote_scratch, 0, sizeof(s_rx_remote_scratch));
  } else if (copy_result != PX4LITE_OK) {
    return copy_result;
  }

  MavRx_UpdateSource(&s_rx_remote_scratch, &s_rx_frame_scratch);
  decoded = MavRx_DecodeMessage(&s_rx_remote_scratch, &s_rx_message_scratch, now_ms);
  if (decoded != 0U) {
    s_last_peer_rx_ms = s_rx_frame_scratch.rx_time_ms;
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

Px4Lite_Result_t Px4Lite_MavlinkRxRunRpi(uint32_t now_ms)
{
#if PX4LITE_ENABLE_RPI_MAVLINK
  uint8_t decoded = 0U;

  if (Px4Lite_RpiMavlinkCopyRxFrame(&s_rx_frame_scratch) != PX4LITE_OK) { return PX4LITE_IDLE; }
  if (s_rx_frame_scratch.payload_len > PX4LITE_COMM_RX_PAYLOAD_MAX) { return PX4LITE_OVERFLOW; }

  MavRx_MessageFromFrame(&s_rx_frame_scratch, &s_rx_message_scratch);
  switch (s_rx_message_scratch.msgid) {
    case MAVLINK_MSG_ID_COMMAND_LONG:
      decoded = MavRx_DecodeCommandLong(&s_rx_message_scratch, now_ms);
      break;
    case MAVLINK_MSG_ID_COMMAND_ACK:
      decoded = MavRx_DecodeCommandAck(0, &s_rx_message_scratch, now_ms);
      break;
    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
      decoded = MavRx_DecodeRpiNamedValueInt(&s_rx_message_scratch, now_ms);
      break;
    default:
      decoded = 0U;
      break;
  }

  return (decoded != 0U) ? PX4LITE_OK : PX4LITE_IDLE;
#else
  (void)now_ms;
  return PX4LITE_IDLE;
#endif
}

uint32_t Px4Lite_MavlinkRxLastPeerMs(void)
{
  return s_last_peer_rx_ms;
}
