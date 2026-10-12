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
#include <math.h>
#include <string.h>

#include "px4lite_config.h"
#include "px4lite_command.h"
#include "px4lite_faults.h"
#include "px4lite_identity.h"
#include "px4lite_modules.h"
#include "px4lite_platform.h"
#include "px4lite_remote_telemetry.h"
#include "px4lite_rpi_baud_switch.h"
#include "px4lite_rpi_h1_echo.h"

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

/** @brief 有界且有限的角度/角速度换算值取整，调用方先检查范围。 */
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

/** @brief 只为本帧实际承载的导航字段记录接收时间。 */
static void MavRx_MarkNavigation(Px4Lite_RemoteTelemetry_t *remote, uint32_t fields)
{
  remote->navigation_valid_mask |= fields;
  remote->valid_mask |= PX4LITE_REMOTE_VALID_NAVIGATION;
  remote->navigation_update_ms = remote->last_rx_ms;
  if ((fields & PX4LITE_NAV_VALID_POSITION) != 0U) { remote->position_update_ms = remote->last_rx_ms; }
  if ((fields & PX4LITE_NAV_VALID_VELOCITY) != 0U) { remote->velocity_update_ms = remote->last_rx_ms; }
  if ((fields & PX4LITE_NAV_VALID_TIME) != 0U) { remote->time_update_ms = remote->last_rx_ms; }
  if ((fields & PX4LITE_NAV_VALID_ALTITUDE) != 0U) { remote->altitude_update_ms = remote->last_rx_ms; }
}

/** @brief 校验 yymmdd 与 UTC 当日秒；00:00:00 是有效时间。 */


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

/** @brief GPS_RAW_INT 的定位与速度各自检查协议有效值，COG 不冒充姿态 yaw。 */
static uint8_t MavRx_DecodeGpsRaw(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_gps_raw_int_t packet;
  float course;
  mavlink_msg_gps_raw_int_decode(message, &packet);
  remote->navigation_valid_mask &= ~(PX4LITE_NAV_VALID_POSITION | PX4LITE_NAV_VALID_VELOCITY | PX4LITE_NAV_VALID_ALTITUDE);
  remote->gnss_fix_type = 0U;
  remote->satellites_used = 0U;
  if ((packet.fix_type < 2U) || (packet.lat < -900000000L) || (packet.lat > 900000000L) ||
      (packet.lon < -1800000000L) || (packet.lon > 1800000000L)) { return 1U; }
  remote->latitude_e7 = packet.lat;
  remote->longitude_e7 = packet.lon;
  remote->gnss_fix_type = packet.fix_type;
  remote->hdop_x100 = (packet.eph != UINT16_MAX) ? packet.eph : 0U;
  remote->satellites_used = (packet.satellites_visible != UINT8_MAX) ? packet.satellites_visible : 0U;
  MavRx_MarkNavigation(remote, PX4LITE_NAV_VALID_POSITION);
  if (packet.fix_type >= 3U) {
    remote->altitude_mm = packet.alt;
    MavRx_MarkNavigation(remote, PX4LITE_NAV_VALID_ALTITUDE);
  }
  if ((packet.vel != UINT16_MAX) && (packet.cog != UINT16_MAX) && (packet.cog <= 36000U)) {
    course = (float)packet.cog * (3.14159265358979323846f / 18000.0f);
    remote->velocity_north_cms = (int32_t)((float)packet.vel * cosf(course));
    remote->velocity_east_cms = (int32_t)((float)packet.vel * sinf(course));
    remote->velocity_down_cms = 0;
    MavRx_MarkNavigation(remote, PX4LITE_NAV_VALID_VELOCITY);
  }
  return 1U;
}

/** @brief 校验有限角度与角速度后发布姿态；非法浮点不转换为整数或刷新年龄。 */
static uint8_t MavRx_DecodeAttitude(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_attitude_t packet;

  mavlink_msg_attitude_decode(message, &packet);
  remote->valid_mask &= ~PX4LITE_REMOTE_VALID_ATTITUDE;
  /* 双向比较同时拒绝 NaN/Inf；宽裕上限避免异常输入溢出整数转换。 */
  if (!(packet.roll>=-1000.0f && packet.roll<=1000.0f &&
        packet.pitch>=-1000.0f && packet.pitch<=1000.0f &&
        packet.yaw>=-1000.0f && packet.yaw<=1000.0f &&
        packet.rollspeed>=-1000.0f && packet.rollspeed<=1000.0f &&
        packet.pitchspeed>=-1000.0f && packet.pitchspeed<=1000.0f &&
        packet.yawspeed>=-1000.0f && packet.yawspeed<=1000.0f)) { return 1U; }
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

/** @brief 只推进本帧所属电池的有效位和接收时间。 */
static void MavRx_MarkBattery(Px4Lite_RemoteTelemetry_t *remote, uint8_t secondary)
{
  remote->valid_mask |= PX4LITE_REMOTE_VALID_BATTERY;
  remote->battery_update_ms = remote->last_rx_ms;
  if (secondary != 0U) {
    remote->battery2_update_ms = remote->last_rx_ms;
  } else {
    remote->battery1_update_ms = remote->last_rx_ms;
  }
}

/** @brief SYS_STATUS 只携带主电池，不激活第二电池。 */
static uint8_t MavRx_DecodeSysStatus(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_sys_status_t packet;

  mavlink_msg_sys_status_decode(message, &packet);
  remote->battery_valid_mask &= ~PX4LITE_REMOTE_BATTERY_VALID_PRIMARY;
  remote->voltage_mv = 0U; remote->current_ma = 0U; remote->battery_percent = 0U;
  if (packet.voltage_battery != UINT16_MAX) {
    remote->voltage_mv = packet.voltage_battery;
    remote->battery_valid_mask |= PX4LITE_REMOTE_BATTERY_VALID_VOLTAGE1;
    MavRx_MarkBattery(remote, 0U);
  }
  /* current_battery 单位 cA(10mA)，-1 表示未测量；换算回 mA。 */
  if (packet.current_battery >= 0) {
    remote->current_ma = (uint32_t)packet.current_battery * 10U;
    remote->battery_valid_mask |= PX4LITE_REMOTE_BATTERY_VALID_CURRENT1;
    MavRx_MarkBattery(remote, 0U);
  }
  if ((packet.battery_remaining >= 0) && (packet.battery_remaining <= 100)) {
    remote->battery_percent = (uint8_t)packet.battery_remaining;
    remote->battery_valid_mask |= PX4LITE_REMOTE_BATTERY_VALID_PERCENT1;
    MavRx_MarkBattery(remote, 0U);
  }
  remote->low_voltage = (packet.battery_remaining >= 0 && packet.battery_remaining <= 15) ? 1U : 0U;
  return 1U;
}

/** @brief 按电池 ID 更新各已知标量，未知字段撤位清零，不借另一路时间续期。 */
static uint8_t MavRx_DecodeBattery(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_battery_status_t packet;

  mavlink_msg_battery_status_decode(message, &packet);
  if (packet.id > 1U) { return 0U; }
  if (packet.id == 1U) { remote->battery_valid_mask &= ~PX4LITE_REMOTE_BATTERY_VALID_SECONDARY; remote->voltage2_mv = 0U; remote->current2_ma = 0U; remote->battery2_percent = 0U; }
  else { remote->battery_valid_mask &= ~PX4LITE_REMOTE_BATTERY_VALID_PRIMARY; remote->voltage_mv = 0U; remote->current_ma = 0U; remote->battery_percent = 0U; }
  /* MAVLink 用 id 区分多块电池：id=1 电池2，其余(id=0)电池1。
     current_battery 单位 cA(10mA)，-1 表示未测量；换算回 mA。 */
  if (packet.id == 1U) {
    if (packet.voltages[0] != UINT16_MAX) { remote->voltage2_mv = packet.voltages[0]; remote->battery_valid_mask |= PX4LITE_REMOTE_BATTERY_VALID_VOLTAGE2; }
    if (packet.current_battery >= 0) { remote->current2_ma = (uint32_t)packet.current_battery * 10U; remote->battery_valid_mask |= PX4LITE_REMOTE_BATTERY_VALID_CURRENT2; }
    if ((packet.battery_remaining >= 0) && (packet.battery_remaining <= 100)) { remote->battery2_percent = (uint8_t)packet.battery_remaining; remote->battery_valid_mask |= PX4LITE_REMOTE_BATTERY_VALID_PERCENT2; }
    remote->low_voltage2 = (packet.charge_state == (uint8_t)MAV_BATTERY_CHARGE_STATE_LOW) ? 1U : 0U;
  } else {
    if (packet.voltages[0] != UINT16_MAX) { remote->voltage_mv = packet.voltages[0]; remote->battery_valid_mask |= PX4LITE_REMOTE_BATTERY_VALID_VOLTAGE1; }
    if (packet.current_battery >= 0) { remote->current_ma = (uint32_t)packet.current_battery * 10U; remote->battery_valid_mask |= PX4LITE_REMOTE_BATTERY_VALID_CURRENT1; }
    if ((packet.battery_remaining >= 0) && (packet.battery_remaining <= 100)) { remote->battery_percent = (uint8_t)packet.battery_remaining; remote->battery_valid_mask |= PX4LITE_REMOTE_BATTERY_VALID_PERCENT1; }
    remote->low_voltage = (packet.charge_state == (uint8_t)MAV_BATTERY_CHARGE_STATE_LOW) ? 1U : 0U;
  }
  if ((remote->battery_valid_mask & ((packet.id == 1U) ? PX4LITE_REMOTE_BATTERY_VALID_SECONDARY : PX4LITE_REMOTE_BATTERY_VALID_PRIMARY)) != 0U) { MavRx_MarkBattery(remote, packet.id); }
  return 1U;
}

/** @brief 温压帧只更新温压来源的有效位和时间，湿度来源独立。 */
static uint8_t MavRx_DecodePressure(Px4Lite_RemoteTelemetry_t *remote, const mavlink_message_t *message)
{
  mavlink_scaled_pressure_t packet;

  mavlink_msg_scaled_pressure_decode(message, &packet);
  remote->environment_valid_mask &= ~PX4LITE_REMOTE_ENV_VALID_PRESSURE_TEMPERATURE;
  remote->pressure_pa = 0.0f; remote->temperature_c = 0.0f;
  if (!(packet.press_abs > 0.0f && packet.press_abs < 2000.0f)) { return 1U; }
  remote->pressure_pa   = packet.press_abs * 100.0f;          /* hPa → Pa */
  remote->temperature_c = ((float)packet.temperature) / 100.0f; /* cdegC → degC */
  /* 湿度不在 SCALED_PRESSURE 里，由独立的 HUMIDITY 帧维护，这里不得清零覆盖。 */
  remote->valid_mask |= PX4LITE_REMOTE_VALID_ENVIRONMENT;
  remote->environment_update_ms = remote->last_rx_ms;
  remote->pressure_temperature_update_ms = remote->last_rx_ms;
  remote->environment_valid_mask |= PX4LITE_REMOTE_ENV_VALID_PRESSURE_TEMPERATURE;
  return 1U;
}





/**
 * @brief 把合法 ESC 脉宽换算为远端显示使用的油门百分比。
 */






/**
 * @brief 解码主输出组的 SERVO_OUTPUT_RAW 四路 PWM 脉宽。
 */


/** @brief LoRa 扩展接收仅保留环境组湿度，不接收旧日志/告警/模块/电机镜像。 */
static uint8_t MavRx_DecodeNamedValueInt(Px4Lite_RemoteTelemetry_t *remote,const mavlink_message_t *message,uint32_t now_ms)
{
  mavlink_named_value_int_t packet;
  (void)now_ms; mavlink_msg_named_value_int_decode(message,&packet);
  if(!MavRx_NameEquals(packet.name,"HUMIDITY")) { return 0U; }
  remote->environment_valid_mask&=~PX4LITE_REMOTE_ENV_VALID_HUMIDITY;
  if(packet.value<0 || packet.value>1000) { return 1U; }
  remote->relative_humidity_pct=(float)packet.value/10.0f;
  remote->valid_mask|=PX4LITE_REMOTE_VALID_ENVIRONMENT;
  remote->environment_update_ms=remote->last_rx_ms; remote->humidity_update_ms=remote->last_rx_ms;
  remote->environment_valid_mask|=PX4LITE_REMOTE_ENV_VALID_HUMIDITY; return 1U;
}



#if PX4LITE_ENABLE_RPI_MAVLINK
/** @brief 仅在 RPi 物理链路识别 H1 TUNNEL，不进入业务遥测快照。 */
static uint8_t MavRx_DecodeRpiH1EchoTunnel(const mavlink_message_t *message)
{
  mavlink_tunnel_t packet;
  Px4Lite_Result_t result;
  Px4Lite_RpiH1EchoResponseView_t pending_echo;
  uint8_t wire_payload_length;

  if (message == 0) { return 0U; }
  mavlink_msg_tunnel_decode(message, &packet);
  if (packet.payload_type != PX4LITE_RPI_H1_TUNNEL_TYPE) { return 0U; }
  if (packet.payload_length > (uint8_t)sizeof(packet.payload)) { return 1U; }
  /* TUNNEL 的固定字段在线上占 5 字节；不能把 MAVLink 解码补零当作实际文本。 */
  wire_payload_length = (message->len >= 5U) ? (uint8_t)(message->len - 5U) : 0U;

#if PX4LITE_RPI_BAUD_SWITCH_ENABLE
  if ((packet.payload_length >= 2U) &&
      (packet.payload[1] == PX4LITE_RPI_BAUD_REQUEST_TYPE)) {
    /* 两种实验事务共用单个 CommTask 发送口；不让切速率响应越过已排队的 H1 回显。 */
    if (Px4Lite_RpiH1EchoPeekResponse(&pending_echo) == PX4LITE_OK) { return 1U; }
    result = Px4Lite_RpiBaudSwitchHandleTunnel(message->sysid,
                                               message->compid,
                                               packet.target_system,
                                               packet.target_component,
                                               Px4Lite_IdentityGetMavlinkSystemId(),
                                               packet.payload_type,
                                               packet.payload,
                                               packet.payload_length,
                                               wire_payload_length);
    return (result == PX4LITE_IDLE) ? 0U : 1U;
  }
  /* 切速率事务挂起期间，拒绝新 H1 请求，避免旧速率响应滞留到 B1。 */
  if (Px4Lite_RpiBaudSwitchIsPending() != 0U) { return 1U; }
#endif

  result = Px4Lite_RpiH1EchoHandleTunnel(message->sysid,
                                         message->compid,
                                         packet.target_system,
                                         packet.target_component,
                                         Px4Lite_IdentityGetMavlinkSystemId(),
                                         packet.payload_type,
                                         packet.payload,
                                         packet.payload_length,
                                         wire_payload_length);
  return (result == PX4LITE_IDLE) ? 0U : 1U;
}
#endif



static uint8_t MavRx_DecodeCommandLong(const mavlink_message_t *message, Px4Lite_MavlinkLink_t link, uint32_t now_ms)
{
  mavlink_command_long_t packet;

  mavlink_msg_command_long_decode(message, &packet);
  return (Px4Lite_CommandHandleMavlinkLong(packet.command, message->sysid, message->compid, packet.target_system, packet.target_component, packet.param1, packet.param2, packet.param3, packet.param4, link, now_ms) == PX4LITE_OK) ? 1U : 0U;
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

  /* 只取 bit0(ONLINE)，由树莓派统一计算；bit1..4 是故障细分，保留位必须忽略。
     RPICELL 只反映树莓派 Linux 侧网络链路状态，不代表 USART6 物理链路，
     因此只驱动 5G 模块显示状态，不得据此门控本机对 RPi 的遥测出口。 */
  value = (uint32_t)packet.value;
  online = ((value & MAV_RX_RPI_CELLULAR_ONLINE_MASK) != 0U) ? 1U : 0U;
  if (online != 0U) {
    Px4Lite_SetExternalModuleState(PX4LITE_MODULE_5G, PX4LITE_STATE_ONLINE, PX4LITE_FAULT_NONE, now_ms);
  } else {
    Px4Lite_SetExternalModuleState(PX4LITE_MODULE_5G, PX4LITE_STATE_OFFLINE, PX4LITE_FAULT_COMM_OFFLINE, now_ms);
  }
  return 1U;
}

/** @brief LoRa 白名单仅解码本轮基础与实验数据；RPi 命令入口独立。 */
static uint8_t MavRx_DecodeMessage(Px4Lite_RemoteTelemetry_t *remote,const mavlink_message_t *message,uint32_t now_ms)
{
  switch(message->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: return MavRx_DecodeHeartbeat(remote,message);
    case MAVLINK_MSG_ID_GPS_RAW_INT: return MavRx_DecodeGpsRaw(remote,message);
    case MAVLINK_MSG_ID_ATTITUDE: return MavRx_DecodeAttitude(remote,message);
    case MAVLINK_MSG_ID_SYS_STATUS: return MavRx_DecodeSysStatus(remote,message);
    case MAVLINK_MSG_ID_BATTERY_STATUS: return MavRx_DecodeBattery(remote,message);
    case MAVLINK_MSG_ID_SCALED_PRESSURE: return MavRx_DecodePressure(remote,message);
    case MAVLINK_MSG_ID_NAMED_VALUE_INT: return MavRx_DecodeNamedValueInt(remote,message,now_ms);
    default: return 0U;
  }
}
/** @brief 发现只接受基础广播与已确认的四组消息。 */
static uint8_t MavRx_LabMessage(const mavlink_message_t *message)
{
  mavlink_named_value_int_t packet;
  switch(message->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: case MAVLINK_MSG_ID_GPS_RAW_INT: case MAVLINK_MSG_ID_ATTITUDE:
    case MAVLINK_MSG_ID_SYS_STATUS: case MAVLINK_MSG_ID_BATTERY_STATUS: case MAVLINK_MSG_ID_SCALED_PRESSURE: return 1U;
    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
      mavlink_msg_named_value_int_decode(message,&packet); return MavRx_NameEquals(packet.name,"HUMIDITY");
    default: return 0U;
  }
}
/** @brief 在忽略同本机 sysid 前解码长期身份，防止相同地址冲突被隐藏。 */
static void MavRx_ObserveIdentity(const mavlink_message_t *message,uint32_t now_ms)
{
  mavlink_open_drone_id_basic_id_t packet;
  uint8_t i;
  if(message->len<MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID_MIN_LEN) { return; }
  mavlink_msg_open_drone_id_basic_id_decode(message,&packet);
  if(packet.id_type!=MAV_ODID_ID_TYPE_SERIAL_NUMBER || memcmp(packet.uas_id,"DCDWCNS1",8U)!=0) { return; }
  for(i=8U;i<20U;i++) { if(!((packet.uas_id[i]>='0' && packet.uas_id[i]<='9') || strchr("ABCDEFGHJKLMNPQRSTUVWXYZ",packet.uas_id[i])!=0 && packet.uas_id[i]!=0U)) { return; } }
  Px4Lite_RemoteObserve(message->sysid,message->compid,message->seq,message->msgid,(const char *)packet.uas_id,now_ms);
}

Px4Lite_Result_t Px4Lite_MavlinkRxRun(uint32_t now_ms)
{
  uint8_t remote_node_id;
  uint8_t decoded;
  Px4Lite_Result_t copy_result;

  if (Px4Lite_CopyCommRxFrame(&s_rx_frame_scratch) != PX4LITE_OK) { return PX4LITE_IDLE; }
  if (s_rx_frame_scratch.payload_len > PX4LITE_COMM_RX_PAYLOAD_MAX) { return PX4LITE_OVERFLOW; }
  MavRx_MessageFromFrame(&s_rx_frame_scratch,&s_rx_message_scratch);
  if(s_rx_message_scratch.msgid==MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID) {
    MavRx_ObserveIdentity(&s_rx_message_scratch,s_rx_frame_scratch.rx_time_ms); return PX4LITE_IDLE;
  }
  if(!MavRx_LabMessage(&s_rx_message_scratch)) { return PX4LITE_IDLE; }
  if (MavRx_FrameToRemoteNode(&s_rx_frame_scratch,&remote_node_id)==0U) { return PX4LITE_IDLE; }
  Px4Lite_RemoteObserve(remote_node_id,s_rx_frame_scratch.component_id,s_rx_frame_scratch.sequence,s_rx_frame_scratch.msg_id,0,s_rx_frame_scratch.rx_time_ms);
  s_last_peer_rx_ms=s_rx_frame_scratch.rx_time_ms;
  if(!Px4Lite_RemoteCanAccept(remote_node_id,now_ms)) { return PX4LITE_IDLE; }
  copy_result = Px4Lite_RemoteTelemetryCopyNode(remote_node_id, &s_rx_remote_scratch);
  if (copy_result == PX4LITE_NOT_READY) {
    return PX4LITE_IDLE;
  } else if (copy_result != PX4LITE_OK) {
    return copy_result;
  }

  MavRx_UpdateSource(&s_rx_remote_scratch, &s_rx_frame_scratch);
  decoded = MavRx_DecodeMessage(&s_rx_remote_scratch, &s_rx_message_scratch, now_ms);
  if (decoded != 0U) {
    s_last_peer_rx_ms = s_rx_frame_scratch.rx_time_ms;
    Px4Lite_RemoteTelemetryRefreshFreshness(&s_rx_remote_scratch, now_ms);
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
      decoded = MavRx_DecodeCommandLong(&s_rx_message_scratch, PX4LITE_MAVLINK_LINK_RPI, now_ms);
      break;
    case MAVLINK_MSG_ID_COMMAND_ACK:
      decoded = MavRx_DecodeCommandAck(0, &s_rx_message_scratch, now_ms);
      break;
    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
      decoded = MavRx_DecodeRpiNamedValueInt(&s_rx_message_scratch, now_ms);
      break;
    case MAVLINK_MSG_ID_TUNNEL:
      decoded = MavRx_DecodeRpiH1EchoTunnel(&s_rx_message_scratch);
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
