/**
 * @file px4lite_mavlink_tx.c
 * @brief 将一致的 Framework topic 转换为 MAVLink 遥测帧。
 *
 * @details
 * CommTask 是本文件唯一调用者。所有 MAVLink 消息均逐字段编码后交给 LoRa 发送，
 * 不允许直接发送 C 结构体内存。LoRa 发送返回 OK 仅表示帧已进入发送缓冲。
 */

#include "px4lite_mavlink_tx.h"

#include <limits.h>
#include <string.h>

#include "px4lite_config.h"
#include "px4lite_alarm.h"
#include "px4lite_platform.h"
#include "px4lite_topics.h"

#if defined(__CC_ARM)
#define MAVLINK_ALIGNED_FIELDS   0
#define MAVLINK_COMM_NUM_BUFFERS 1
#pragma diag_suppress 66
#endif

#include "common/mavlink.h"

#if defined(__CC_ARM)
#pragma diag_default 66
#endif

typedef Px4Lite_Result_t (*MavTx_EncodeFn_t)(uint32_t now_ms);
typedef void (*MavTx_SuccessHook_t)(void);

#define MAV_TX_SCOPE_ALWAYS    0U
#define MAV_TX_SCOPE_STANDARD  1U
#define MAV_TX_SCOPE_EXTENSION 2U

#define PX4LITE_MAVLINK_STREAM_COMMAND ((uint16_t)MAV_CMD_USER_1)
#define MAV_TX_STREAM_ACTION_STOP      0U
#define MAV_TX_STREAM_ACTION_START     1U

/**
 * @brief LoRa/MAVLink 外发数据目录项。
 * @details
 * 目录只描述调度属性，不保存外设大数据。每个编码函数按需读取一个同源快照，
 * 单周期最多提交一帧，避免半双工 LoRa 和 comm 任务栈被大包拖垮。
 */
typedef struct {
  const char *name;                /**< 数据项名称，用于调试和后续审查。 */
  uint8_t enabled;                 /**< 使能开关，0 表示跳过该项。 */
  uint32_t period_ms;              /**< 发送周期，单位：ms。 */
  uint32_t message_id;             /**< MAVLink message id。 */
  uint32_t *next_ms;               /**< 下一次允许发送时间，单位：ms。 */
  uint32_t *success_count;         /**< 成功提交计数指针。 */
  MavTx_EncodeFn_t encode;         /**< 编码并提交一帧的函数。 */
  MavTx_SuccessHook_t on_success;  /**< 成功提交后的可选状态推进回调。 */
  uint8_t scope;
} MavTx_Item_t;

typedef struct {
  uint8_t valid;
  uint8_t target_system;
  uint8_t target_component;
  uint8_t confirmation;
  uint16_t command;
  float param1;
  float param2;
  float param3;
  float param4;
  uint32_t next_try_ms;
} MavTx_PendingCommand_t;

typedef struct {
  uint8_t valid;
  uint8_t result;
  uint8_t target_system;
  uint8_t target_component;
  uint16_t command;
} MavTx_PendingAck_t;

static mavlink_message_t s_message;
/* 所有者：仅 CommTask。LoRa 驱动会先复制该缓冲区，再允许调用者复用。 */
static uint8_t s_frame[MAVLINK_MAX_PACKET_LEN];
static Px4Lite_MavlinkTxStats_t s_stats;

static uint32_t s_next_heartbeat_ms;
static uint32_t s_next_gps_raw_ms;
static uint32_t s_next_gnss_detail_ms;
static uint32_t s_next_attitude_ms;
static uint32_t s_next_position_ms;
static uint32_t s_next_sys_status_ms;
static uint32_t s_next_module_state_ms;
static uint32_t s_next_battery_ms;
static uint32_t s_next_pressure_ms;
static uint32_t s_next_statustext_ms;
static uint8_t s_catalog_index;
static uint8_t s_module_state_part;
static uint8_t s_remote_view_enabled;
static uint8_t s_remote_view_target_node;
#if PX4LITE_NODE_ROLE == PX4LITE_NODE_ROLE_SLAVE
static uint32_t s_remote_view_start_ms;
#endif
static uint32_t s_next_stream_request_ms;
static uint32_t s_stream_until_ms;
static uint32_t s_stream_mask;
static MavTx_PendingCommand_t s_pending_command;
static MavTx_PendingAck_t s_pending_ack;

#define MAV_TX_DEG100_TO_RAD 0.0001745329252f

/**
 * @brief 使用回绕安全差值判断一个毫秒截止时间是否到期。
 */
static uint8_t MavTx_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0) ? 1U : 0U;
}

static uint8_t MavTx_NodeIdToSystemId(uint8_t node_id)
{
  return (uint8_t)(node_id + 1U);
}

static uint8_t MavTx_IsForThisSystem(uint8_t target_system, uint8_t target_component)
{
  if ((target_system != 0U) && (target_system != (uint8_t)PX4LITE_MAVLINK_SYSTEM_ID)) { return 0U; }
  if ((target_component != 0U) && (target_component != (uint8_t)PX4LITE_MAVLINK_COMPONENT_ID)) { return 0U; }
  return 1U;
}

static uint8_t MavTx_MainStreamActive(uint32_t now_ms)
{
  return ((s_stream_mask != 0U) && (MavTx_TimeReached(now_ms, s_stream_until_ms) == 0U)) ? 1U : 0U;
}

static uint8_t MavTx_ItemAllowed(const MavTx_Item_t *item, uint32_t now_ms)
{
  if (item == 0) { return 0U; }
  if (item->scope == MAV_TX_SCOPE_ALWAYS) { return 1U; }

#if PX4LITE_MAVLINK_LINK_MODE == PX4LITE_MAVLINK_LINK_MODE_GCS
  if (item->scope == MAV_TX_SCOPE_EXTENSION) { return (PX4LITE_MAVLINK_ENABLE_NAMED_VALUE_EXTENSIONS != 0U) ? 1U : 0U; }
  (void)now_ms;
  return 1U;
#else
  if ((item->scope == MAV_TX_SCOPE_EXTENSION) && (PX4LITE_MAVLINK_ENABLE_NAMED_VALUE_EXTENSIONS == 0U)) { return 0U; }
  return (MavTx_MainStreamActive(now_ms) != 0U) ? 1U : 0U;
#endif
}

static void MavTx_QueueStreamCommand(uint8_t target_node_id, uint8_t action, uint32_t stream_mask, uint32_t lease_ms, uint32_t now_ms)
{
  memset(&s_pending_command, 0, sizeof(s_pending_command));
  s_pending_command.valid            = 1U;
  s_pending_command.target_system    = MavTx_NodeIdToSystemId(target_node_id);
  s_pending_command.target_component = PX4LITE_MAVLINK_COMPONENT_ID;
  s_pending_command.command          = PX4LITE_MAVLINK_STREAM_COMMAND;
  s_pending_command.param1           = (float)action;
  s_pending_command.param2           = (float)PX4LITE_NODE_ID;
  s_pending_command.param3           = (float)stream_mask;
  s_pending_command.param4           = (float)lease_ms;
  s_pending_command.next_try_ms      = now_ms;
}

static void MavTx_QueueAck(uint16_t command, uint8_t result, uint8_t target_system, uint8_t target_component)
{
  s_pending_ack.valid            = 1U;
  s_pending_ack.command          = command;
  s_pending_ack.result           = result;
  s_pending_ack.target_system    = target_system;
  s_pending_ack.target_component = target_component;
}

/**
 * @brief 将有符号 Framework 数值饱和收窄到 MAVLink int16 字段。
 */
static int16_t MavTx_SaturateInt16(int32_t value)
{
  if (value > INT16_MAX) { return INT16_MAX; }
  if (value < INT16_MIN) { return INT16_MIN; }
  return (int16_t)value;
}

/**
 * @brief 将无符号 Framework 数值饱和收窄到 MAVLink uint16 字段。
 */
static uint16_t MavTx_SaturateUint16(uint32_t value)
{
  return (value > UINT16_MAX) ? UINT16_MAX : (uint16_t)value;
}

/**
 * @brief 将 mA 电流转换为 MAVLink BATTERY_STATUS 使用的 10mA 单位。
 *
 * @param[in] current_ma 电流，单位 mA；0 表示未知。
 *
 * @return MAVLink current_battery 字段值，单位 10mA；-1 表示未知。
 */
static int16_t MavTx_SaturateCentiAmp(int32_t current_ma)
{
  int32_t centi_amp;

  if (current_ma == 0) { return -1; }

  centi_amp = current_ma / 10;
  if (centi_amp > INT16_MAX) { return INT16_MAX; }
  if (centi_amp < INT16_MIN) { return INT16_MIN; }
  return (int16_t)centi_amp;
}

/**
 * @brief 将摄氏度浮点温度转换为 MAVLink 使用的摄氏度 * 100。
 *
 * @param[in] temperature_c 温度，单位摄氏度。
 *
 * @return 饱和后的摄氏度 * 100 定点值。
 */
static int16_t MavTx_SaturateCdegFromFloat(float temperature_c)
{
  float cdeg = temperature_c * 100.0f;

  if (cdeg > (float)INT16_MAX) { return INT16_MAX; }
  if (cdeg < (float)INT16_MIN) { return INT16_MIN; }
  return (int16_t)((cdeg >= 0.0f) ? (cdeg + 0.5f) : (cdeg - 0.5f));
}

/**
 * @brief 将 degree*100 航向角归一化到 [0, 35999]。
 */
static uint16_t MavTx_NormalizeHeading(int32_t heading_deg100)
{
  int32_t normalized = heading_deg100 % 36000;

  if (normalized < 0) { normalized += 36000; }
  return (uint16_t)normalized;
}

/**
 * @brief 将 degree*100 角度转换为弧度。
 *
 * @param[in] value 角度，单位 degree * 100。
 *
 * @return 角度弧度值。
 */
static float MavTx_Deg100ToRad(int32_t value)
{
  return ((float)value) * MAV_TX_DEG100_TO_RAD;
}

/**
 * @brief 将 NMEA GGA fix quality 映射为 MAVLink GPS_FIX_TYPE。
 */
static uint8_t MavTx_MapGpsFixType(uint8_t fix_quality)
{
  switch (fix_quality) {
    case 0U:
      return (uint8_t)GPS_FIX_TYPE_NO_FIX;
    case 2U:
      return (uint8_t)GPS_FIX_TYPE_DGPS;
    case 4U:
      return (uint8_t)GPS_FIX_TYPE_RTK_FIXED;
    case 5U:
      return (uint8_t)GPS_FIX_TYPE_RTK_FLOAT;
    case 1U:
    case 6U:
    default:
      return (uint8_t)GPS_FIX_TYPE_3D_FIX;
  }
}

/**
 * @brief 返回 GPS 和北斗可见卫星数量之和。
 */
static uint8_t MavTx_VisibleSatellites(const Px4Lite_SensorGnss_t *gnss)
{
  uint16_t total = (uint16_t)gnss->gps_visible + (uint16_t)gnss->bds_visible;

  return (total > UINT8_MAX) ? UINT8_MAX : (uint8_t)total;
}

/**
 * @brief 序列化当前 MAVLink 消息并提交给 LoRa 发送。
 */
static Px4Lite_Result_t MavTx_SendPrepared(void)
{
  uint16_t length;

  length = mavlink_msg_to_send_buffer(s_frame, &s_message);
  if ((length == 0U) || (length > (uint16_t)sizeof(s_frame))) { return PX4LITE_IO_ERROR; }

  return Px4Lite_LoRaSend(s_frame, length);
}

static Px4Lite_Result_t MavTx_SendPendingAck(uint32_t now_ms)
{
  mavlink_command_ack_t packet;
  Px4Lite_Result_t result;

  (void)now_ms;
  if (s_pending_ack.valid == 0U) { return PX4LITE_IDLE; }

  memset(&packet, 0, sizeof(packet));
  packet.command          = s_pending_ack.command;
  packet.result           = s_pending_ack.result;
  packet.progress         = UINT8_MAX;
  packet.result_param2    = 0;
  packet.target_system    = s_pending_ack.target_system;
  packet.target_component = s_pending_ack.target_component;

  (void)mavlink_msg_command_ack_encode_chan(PX4LITE_MAVLINK_SYSTEM_ID, PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);
  result = MavTx_SendPrepared();
  if (result == PX4LITE_OK) {
    s_pending_ack.valid = 0U;
    s_stats.command_ack_tx_count++;
    s_stats.last_message_id = MAVLINK_MSG_ID_COMMAND_ACK;
  }
  return result;
}

static Px4Lite_Result_t MavTx_SendPendingCommand(uint32_t now_ms)
{
  mavlink_command_long_t packet;
  Px4Lite_Result_t result;

  if (s_pending_command.valid == 0U) { return PX4LITE_IDLE; }
  if (MavTx_TimeReached(now_ms, s_pending_command.next_try_ms) == 0U) { return PX4LITE_IDLE; }

  memset(&packet, 0, sizeof(packet));
  packet.target_system    = s_pending_command.target_system;
  packet.target_component = s_pending_command.target_component;
  packet.command          = s_pending_command.command;
  packet.confirmation     = s_pending_command.confirmation++;
  packet.param1           = s_pending_command.param1;
  packet.param2           = s_pending_command.param2;
  packet.param3           = s_pending_command.param3;
  packet.param4           = s_pending_command.param4;

  (void)mavlink_msg_command_long_encode_chan(PX4LITE_MAVLINK_SYSTEM_ID, PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);
  result = MavTx_SendPrepared();
  if (result == PX4LITE_OK) {
    s_pending_command.valid = 0U;
    s_stats.command_count++;
    s_stats.last_message_id = MAVLINK_MSG_ID_COMMAND_LONG;
  } else if (result == PX4LITE_BUSY) {
    s_pending_command.next_try_ms = now_ms + PX4LITE_MAVLINK_COMMAND_RETRY_MS;
  }
  return result;
}

static void MavTx_UpdateStreamRequest(uint32_t now_ms)
{
#if PX4LITE_MAVLINK_LINK_MODE == PX4LITE_MAVLINK_LINK_MODE_PRODUCT
  if (s_remote_view_enabled == 0U) { return; }
  if (MavTx_TimeReached(now_ms, s_next_stream_request_ms) == 0U) { return; }
  if (s_pending_command.valid != 0U) { return; }

  MavTx_QueueStreamCommand(s_remote_view_target_node, MAV_TX_STREAM_ACTION_START, PX4LITE_MAVLINK_STREAM_MASK_ALL, PX4LITE_MAVLINK_STREAM_LEASE_MS, now_ms);
  s_next_stream_request_ms = now_ms + PX4LITE_MAVLINK_STREAM_RENEW_MS;
#else
  (void)now_ms;
#endif
}

/**
 * @brief 编码并发送 MAVLink HEARTBEAT。
 */
static Px4Lite_Result_t MavTx_SendHeartbeat(uint32_t now_ms)
{
  (void)now_ms;
  (void)mavlink_msg_heartbeat_pack_chan(PX4LITE_MAVLINK_SYSTEM_ID, PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0U, 0U, MAV_STATE_ACTIVE);

  return MavTx_SendPrepared();
}

/**
 * @brief 将最新 GNSS 快照编码为 MAVLink GPS_RAW_INT。
 */
static Px4Lite_Result_t MavTx_SendGpsRaw(uint32_t now_ms)
{
  Px4Lite_SensorGnss_t gnss;
  mavlink_gps_raw_int_t packet;
  Px4Lite_Result_t result;
  uint8_t mav_fix;

  result = Px4Lite_CopyGnss(&gnss);
  if (result != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  /* 首次定位前抑制 GPS_RAW，避免持续发送 no-fix 位置帧。 */
  if (gnss.fix_type == 0U) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&gnss.header, now_ms, PX4LITE_GNSS_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }
  if (gnss.header.sequence == s_stats.last_gps_sequence) { return PX4LITE_IDLE; }

  memset(&packet, 0, sizeof(packet));

  /* GNSS topic 不提供完整 UTC epoch，这里按 MAVLink 允许的系统启动后时间填充。 */
  packet.time_usec = (uint64_t)gnss.header.sample_time_ms * 1000ULL;

  mav_fix                   = MavTx_MapGpsFixType(gnss.fix_type);
  packet.fix_type           = mav_fix;
  packet.eph                = (gnss.hdop_x100 != 0U) ? gnss.hdop_x100 : UINT16_MAX;
  packet.epv                = UINT16_MAX;
  packet.satellites_visible = MavTx_VisibleSatellites(&gnss);

  if (mav_fix >= (uint8_t)GPS_FIX_TYPE_2D_FIX) {
    packet.lat = gnss.latitude_e7;
    packet.lon = gnss.longitude_e7;
    packet.alt = gnss.altitude_mm;
    packet.vel = MavTx_SaturateUint16(gnss.ground_speed_cms);
    packet.cog = MavTx_NormalizeHeading(gnss.heading_deg100);
  } else {
    packet.vel = UINT16_MAX;
    packet.cog = UINT16_MAX;
  }

  /* ATGM336H 当前未提供这些精度和双天线字段。 */
  packet.alt_ellipsoid = 0;
  packet.h_acc         = UINT32_MAX;
  packet.v_acc         = UINT32_MAX;
  packet.vel_acc       = UINT32_MAX;
  packet.hdg_acc       = UINT32_MAX;
  packet.yaw           = 0U;

  (void)mavlink_msg_gps_raw_int_encode_chan(PX4LITE_MAVLINK_SYSTEM_ID, PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

  result = MavTx_SendPrepared();
  if (result == PX4LITE_OK) { s_stats.last_gps_sequence = gnss.header.sequence; }
  return result;
}

/**
 * @brief 用标准 NAMED_VALUE_INT 发送 GPS/北斗可见与使用卫星数。
 *
 * @details
 * name 固定为 "GNSS_SAT"。uint32 布局：0..7 位为 GPS visible，8..15 位为北斗
 * visible，16..23 位为 GPS used，24..31 位为北斗 used。
 */
static Px4Lite_Result_t MavTx_SendGnssDetail(uint32_t now_ms)
{
  Px4Lite_SensorGnss_t gnss;
  mavlink_named_value_int_t packet;
  Px4Lite_Result_t result;
  uint32_t packed;

  result = Px4Lite_CopyGnss(&gnss);
  if (result != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  /* 卫星细节跟随 GPS_RAW 策略：定位成功后才发送。 */
  if (gnss.fix_type == 0U) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&gnss.header, now_ms, PX4LITE_GNSS_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }
  if (gnss.header.sequence == s_stats.last_detail_sequence) { return PX4LITE_IDLE; }

  packed = (uint32_t)gnss.gps_visible | ((uint32_t)gnss.bds_visible << 8U) | ((uint32_t)gnss.gps_used << 16U) | ((uint32_t)gnss.bds_used << 24U);

  memset(&packet, 0, sizeof(packet));
  packet.time_boot_ms = now_ms;
  packet.value        = (int32_t)packed;
  memcpy(packet.name, "GNSS_SAT", 8U);

  (void)mavlink_msg_named_value_int_encode_chan(PX4LITE_MAVLINK_SYSTEM_ID, PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

  result = MavTx_SendPrepared();
  if (result == PX4LITE_OK) { s_stats.last_detail_sequence = gnss.header.sequence; }
  return result;
}

/**
 * @brief 将最新估计器姿态编码为 MAVLink ATTITUDE。
 */
static Px4Lite_Result_t MavTx_SendAttitude(uint32_t now_ms)
{
  Px4Lite_VehicleNavigation_t navigation;
  mavlink_attitude_t packet;
  Px4Lite_Result_t result;

  result = Px4Lite_CopyNavigation(&navigation);
  if (result != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&navigation.header, now_ms, PX4LITE_IMU_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }
  if ((navigation.valid_mask & PX4LITE_NAV_VALID_ATTITUDE) == 0U) { return PX4LITE_NOT_READY; }
  if (navigation.header.sequence == s_stats.last_attitude_sequence) { return PX4LITE_IDLE; }

  memset(&packet, 0, sizeof(packet));
  packet.time_boot_ms = now_ms;
  packet.roll         = MavTx_Deg100ToRad(navigation.roll_deg100);
  packet.pitch        = MavTx_Deg100ToRad(navigation.pitch_deg100);
  packet.yaw          = MavTx_Deg100ToRad(navigation.yaw_deg100);
  packet.rollspeed    = MavTx_Deg100ToRad(navigation.roll_rate_dps100);
  packet.pitchspeed   = MavTx_Deg100ToRad(navigation.pitch_rate_dps100);
  packet.yawspeed     = MavTx_Deg100ToRad(navigation.yaw_rate_dps100);

  (void)mavlink_msg_attitude_encode_chan(PX4LITE_MAVLINK_SYSTEM_ID, PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

  result = MavTx_SendPrepared();
  if (result == PX4LITE_OK) { s_stats.last_attitude_sequence = navigation.header.sequence; }
  return result;
}

/**
 * @brief 将导航快照编码为 MAVLink GLOBAL_POSITION_INT。
 */
static Px4Lite_Result_t MavTx_SendPosition(uint32_t now_ms)
{
  Px4Lite_VehicleNavigation_t navigation;
  mavlink_global_position_int_t packet;

  if (Px4Lite_CopyNavigation(&navigation) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&navigation.header, now_ms, PX4LITE_GNSS_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }
  if ((navigation.valid_mask & PX4LITE_NAV_VALID_POSITION) == 0U) { return PX4LITE_NOT_READY; }

  memset(&packet, 0, sizeof(packet));
  packet.time_boot_ms = now_ms;
  packet.lat          = navigation.latitude_e7;
  packet.lon          = navigation.longitude_e7;
  packet.alt          = navigation.fused_altitude_mm;

  /* 后续 Home 模块应提供相对高度。 */
  packet.relative_alt = 0;

  if ((navigation.valid_mask & PX4LITE_NAV_VALID_VELOCITY) != 0U) {
    packet.vx = MavTx_SaturateInt16(navigation.velocity_north_cms);
    packet.vy = MavTx_SaturateInt16(navigation.velocity_east_cms);
    packet.vz = MavTx_SaturateInt16(navigation.velocity_down_cms);
  }

  packet.hdg = ((navigation.valid_mask & PX4LITE_NAV_VALID_ATTITUDE) != 0U) ? MavTx_NormalizeHeading(navigation.yaw_deg100) : UINT16_MAX;

  (void)mavlink_msg_global_position_int_encode_chan(PX4LITE_MAVLINK_SYSTEM_ID, PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

  return MavTx_SendPrepared();
}

/**
 * @brief 将最新电池 topic 编码为 MAVLink BATTERY_STATUS。
 */
static Px4Lite_Result_t MavTx_SendBatteryStatus(uint32_t now_ms)
{
  Px4Lite_BatteryStatus_t battery;
  mavlink_battery_status_t packet;
  Px4Lite_Result_t result;
  uint8_t i;

  result = Px4Lite_CopyBattery(&battery);
  if (result != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&battery.header, now_ms, PX4LITE_BATTERY_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }
  if (battery.header.sequence == s_stats.last_battery_sequence) { return PX4LITE_IDLE; }

  memset(&packet, 0, sizeof(packet));
  packet.id               = 0U;
  packet.battery_function = (uint8_t)MAV_BATTERY_FUNCTION_ALL;
  packet.type             = (uint8_t)MAV_BATTERY_TYPE_UNKNOWN;
  packet.temperature      = INT16_MAX;
  for (i = 0U; i < 10U; ++i) { packet.voltages[i] = UINT16_MAX; }
  packet.voltages[0]       = MavTx_SaturateUint16(battery.voltage_mv);
  packet.current_battery   = MavTx_SaturateCentiAmp(battery.current_ma);
  packet.current_consumed  = -1;
  packet.energy_consumed   = -1;
  packet.battery_remaining = (int8_t)battery.percent;
  packet.time_remaining    = 0;
  packet.charge_state      = (battery.low_voltage != 0U) ? (uint8_t)MAV_BATTERY_CHARGE_STATE_LOW : (uint8_t)MAV_BATTERY_CHARGE_STATE_OK;

  (void)mavlink_msg_battery_status_encode_chan(PX4LITE_MAVLINK_SYSTEM_ID, PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

  result = MavTx_SendPrepared();
  if (result == PX4LITE_OK) { s_stats.last_battery_sequence = battery.header.sequence; }
  return result;
}

/**
 * @brief 将最新气压计 topic 编码为 MAVLink SCALED_PRESSURE。
 */
static Px4Lite_Result_t MavTx_SendScaledPressure(uint32_t now_ms)
{
  Px4Lite_SensorBaro_t baro;
  mavlink_scaled_pressure_t packet;
  Px4Lite_Result_t result;

  result = Px4Lite_CopyBaro(&baro);
  if (result != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&baro.header, now_ms, PX4LITE_BARO_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }
  if (baro.header.sequence == s_stats.last_pressure_sequence) { return PX4LITE_IDLE; }

  memset(&packet, 0, sizeof(packet));
  packet.time_boot_ms           = baro.header.sample_time_ms;
  packet.press_abs              = baro.pressure_pa / 100.0f;
  packet.press_diff             = 0.0f;
  packet.temperature            = MavTx_SaturateCdegFromFloat(baro.temperature_c);
  packet.temperature_press_diff = 0;

  (void)mavlink_msg_scaled_pressure_encode_chan(PX4LITE_MAVLINK_SYSTEM_ID, PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

  result = MavTx_SendPrepared();
  if (result == PX4LITE_OK) { s_stats.last_pressure_sequence = baro.header.sequence; }
  return result;
}

/**
 * @brief 判断模块状态是否可映射为 MAVLink healthy。
 */
static uint8_t MavTx_ModuleHealthy(Px4Lite_State_t state)
{
  return ((state == PX4LITE_STATE_ONLINE) || (state == PX4LITE_STATE_DEGRADED)) ? 1U : 0U;
}

/**
 * @brief 将 Framework 告警严重度映射为 MAVLink 严重度。
 *
 * @param[in] severity Framework 告警严重度。
 *
 * @return MAVLink MAV_SEVERITY 枚举值。
 */
static uint8_t MavTx_MapSeverity(Px4Lite_AlarmSeverity_t severity)
{
  switch (severity) {
    case PX4LITE_ALARM_FATAL:
      return (uint8_t)MAV_SEVERITY_ALERT;
    case PX4LITE_ALARM_CRITICAL:
      return (uint8_t)MAV_SEVERITY_CRITICAL;
    case PX4LITE_ALARM_ERROR:
      return (uint8_t)MAV_SEVERITY_ERROR;
    case PX4LITE_ALARM_WARNING:
      return (uint8_t)MAV_SEVERITY_WARNING;
    case PX4LITE_ALARM_INFO:
    default:
      return (uint8_t)MAV_SEVERITY_INFO;
  }
}

/**
 * @brief 将模块编号映射为 STATUSTEXT 中的短名称。
 *
 * @param[in] module_id Framework 模块编号。
 *
 * @return 常量字符串短名称。
 */
static const char *MavTx_ModuleName(uint16_t module_id)
{
  switch ((Px4Lite_ModuleId_t)module_id) {
    case PX4LITE_MODULE_GNSS:
      return "GNSS";
    case PX4LITE_MODULE_IMU:
      return "IMU";
    case PX4LITE_MODULE_BARO:
      return "BARO";
    case PX4LITE_MODULE_BATTERY:
      return "POWER";
    case PX4LITE_MODULE_LORA:
      return "LORA";
    case PX4LITE_MODULE_DISPLAY:
      return "DISPLAY";
    case PX4LITE_MODULE_ALARM:
      return "ALARM";
    case PX4LITE_MODULE_SYSTEM:
      return "SYSTEM";
    case PX4LITE_MODULE_ESTIMATOR:
      return "EST";
    default:
      return "MODULE";
  }
}

/**
 * @brief 将低 4 bit 数值转换为大写十六进制字符。
 *
 * @param[in] value 待转换数值。
 *
 * @return 十六进制字符。
 */
static char MavTx_HexNibble(uint8_t value)
{
  value &= 0x0FU;
  return (value < 10U) ? (char)('0' + value) : (char)('A' + (value - 10U));
}

/**
 * @brief 构造 MAVLink STATUSTEXT 文本字段。
 *
 * @param[out] text MAVLink 文本缓冲区，固定 50 字节。
 * @param[in] prefix 文本前缀。
 * @param[in] module 模块短名称。
 * @param[in] fault_code Framework 故障码。
 */
static void MavTx_CopyText(char text[50], const char *prefix, const char *module, uint16_t fault_code)
{
  uint8_t pos = 0U;
  uint8_t i;

  memset(text, 0, 50U);
  for (i = 0U; (prefix[i] != '\0') && (pos < 49U); ++i) { text[pos++] = prefix[i]; }
  if (pos < 49U) { text[pos++] = ' '; }
  for (i = 0U; (module[i] != '\0') && (pos < 49U); ++i) { text[pos++] = module[i]; }
  if ((pos + 7U) < 50U) {
    text[pos++] = ' ';
    text[pos++] = '0';
    text[pos++] = 'x';
    text[pos++] = MavTx_HexNibble((uint8_t)(fault_code >> 12));
    text[pos++] = MavTx_HexNibble((uint8_t)(fault_code >> 8));
    text[pos++] = MavTx_HexNibble((uint8_t)(fault_code >> 4));
    text[pos++] = MavTx_HexNibble((uint8_t)fault_code);
  }
}

/**
 * @brief 将当前最高活动告警编码为 MAVLink STATUSTEXT。
 */
static Px4Lite_Result_t MavTx_SendStatusText(uint32_t now_ms)
{
  mavlink_statustext_t packet;
  Px4Lite_Result_t result;
  uint32_t alarm_publish_ms;
  uint32_t alarm_sequence;
  uint16_t active_count;
  uint16_t highest_fault_code;
  uint16_t highest_source_id;
  Px4Lite_AlarmSeverity_t highest_severity;

  if (Px4Lite_CopyAlarmSummary(&alarm_publish_ms, &alarm_sequence, &active_count, &highest_fault_code, &highest_source_id, &highest_severity) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if ((uint32_t)(now_ms - alarm_publish_ms) > (PX4LITE_HEALTH_PERIOD_MS * 10U)) { return PX4LITE_STALE; }
  if (active_count == 0U) { return PX4LITE_NOT_READY; }
  if (alarm_sequence == s_stats.last_alarm_sequence) { return PX4LITE_IDLE; }

  memset(&packet, 0, sizeof(packet));
  packet.severity  = MavTx_MapSeverity(highest_severity);
  packet.id        = highest_fault_code;
  packet.chunk_seq = 0U;
  MavTx_CopyText(packet.text, "PX4LITE ALARM", MavTx_ModuleName(highest_source_id), highest_fault_code);

  (void)mavlink_msg_statustext_encode_chan(PX4LITE_MAVLINK_SYSTEM_ID, PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

  result = MavTx_SendPrepared();
  if (result == PX4LITE_OK) { s_stats.last_alarm_sequence = alarm_sequence; }
  return result;
}

/**
 * @brief 将启用的 Framework 传感器健康状态编码为 SYS_STATUS。
 */
static Px4Lite_Result_t MavTx_SendSystemStatus(uint32_t now_ms)
{
  Px4Lite_SystemHealth_t health;
  mavlink_sys_status_t packet;
  Px4Lite_BatteryStatus_t battery;
  uint32_t present = 0U;
  uint32_t enabled = 0U;
  uint32_t healthy = 0U;

  if (Px4Lite_CopyHealth(&health) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&health.header, now_ms, PX4LITE_HEALTH_PERIOD_MS * 5U) == 0U) { return PX4LITE_STALE; }

#if PX4LITE_ENABLE_GNSS
  present |= MAV_SYS_STATUS_SENSOR_GPS;
  enabled |= MAV_SYS_STATUS_SENSOR_GPS;
  if (MavTx_ModuleHealthy(health.module_state[PX4LITE_MODULE_GNSS]) != 0U) { healthy |= MAV_SYS_STATUS_SENSOR_GPS; }
#endif

#if PX4LITE_ENABLE_IMU
  present |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
  present |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
  enabled |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
  enabled |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
  if (MavTx_ModuleHealthy(health.module_state[PX4LITE_MODULE_IMU]) != 0U) {
    healthy |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
    healthy |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
  }
#endif

#if PX4LITE_ENABLE_BARO
  present |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
  enabled |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
  if (MavTx_ModuleHealthy(health.module_state[PX4LITE_MODULE_BARO]) != 0U) { healthy |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE; }
#endif

  memset(&packet, 0, sizeof(packet));
  packet.onboard_control_sensors_present = present;
  packet.onboard_control_sensors_enabled = enabled;
  packet.onboard_control_sensors_health  = healthy;
  packet.voltage_battery                 = UINT16_MAX;
  packet.current_battery                 = -1;
  packet.battery_remaining               = -1;
  if ((Px4Lite_CopyBattery(&battery) == PX4LITE_OK) && (Px4Lite_IsFresh(&battery.header, now_ms, PX4LITE_BATTERY_MAX_AGE_MS) != 0U)) {
    packet.voltage_battery   = MavTx_SaturateUint16(battery.voltage_mv);
    packet.current_battery   = MavTx_SaturateCentiAmp(battery.current_ma);
    packet.battery_remaining = (int8_t)battery.percent;
  }

  (void)mavlink_msg_sys_status_encode_chan(PX4LITE_MAVLINK_SYSTEM_ID, PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

  return MavTx_SendPrepared();
}

/**
 * @brief 使用标准 NAMED_VALUE_INT 发送 Framework 模块状态分片。
 * @details
 * MODSTAT0/MODSTAT1 每帧打包 8 个模块状态，每个状态占 4 bit，模块顺序与
 * Px4Lite_ModuleId_t 一致。该帧用于让对端和地面站看到与本机界面同源的模块状态。
 */
static Px4Lite_Result_t MavTx_SendModuleState(uint32_t now_ms)
{
  Px4Lite_SystemHealth_t health;
  mavlink_named_value_int_t packet;
  uint32_t packed = 0U;
  uint8_t start;
  uint8_t i;
  uint8_t index;

  if (Px4Lite_CopyHealth(&health) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&health.header, now_ms, PX4LITE_HEALTH_PERIOD_MS * 5U) == 0U) { return PX4LITE_STALE; }

  start = (s_module_state_part == 0U) ? 0U : 8U;
  for (i = 0U; i < 8U; ++i) {
    index = (uint8_t)(start + i);
    if (index < (uint8_t)PX4LITE_MODULE_COUNT) { packed |= (((uint32_t)health.module_state[index]) & 0x0FUL) << (i * 4U); }
  }

  memset(&packet, 0, sizeof(packet));
  packet.time_boot_ms = now_ms;
  packet.value        = (int32_t)packed;
  if (s_module_state_part == 0U) {
    memcpy(packet.name, "MODSTAT0", 8U);
  } else {
    memcpy(packet.name, "MODSTAT1", 8U);
  }

  (void)mavlink_msg_named_value_int_encode_chan(PX4LITE_MAVLINK_SYSTEM_ID, PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

  return MavTx_SendPrepared();
}

/**
 * @brief 模块状态分片成功发送后的分片推进。
 */
static void MavTx_ModuleStateSuccess(void)
{
  s_module_state_part ^= 1U;
}

static void MavTx_RecordResult(Px4Lite_Result_t result, uint32_t now_ms, uint32_t period_ms, uint32_t *next_ms, uint32_t message_id, uint32_t *success_count)
{
  if (result == PX4LITE_OK) {
    *next_ms                = now_ms + period_ms;
    *success_count          = *success_count + 1U;
    s_stats.last_message_id = message_id;
  } else {
    *next_ms = now_ms + PX4LITE_MAVLINK_RETRY_PERIOD_MS;

    if ((result == PX4LITE_NOT_READY) || (result == PX4LITE_IDLE)) {
      s_stats.no_data_count++;
    } else if (result == PX4LITE_STALE) {
      s_stats.stale_count++;
    } else if (result == PX4LITE_BUSY) {
      s_stats.busy_count++;
    } else {
      s_stats.error_count++;
    }
  }
}

static const MavTx_Item_t s_mav_tx_catalog[] = {
    {"HEARTBEAT", PX4LITE_MAVLINK_ENABLE_HEARTBEAT, PX4LITE_MAVLINK_HEARTBEAT_PERIOD_MS, MAVLINK_MSG_ID_HEARTBEAT, &s_next_heartbeat_ms, &s_stats.heartbeat_count, MavTx_SendHeartbeat, 0, MAV_TX_SCOPE_ALWAYS},
    {"GPS_RAW", PX4LITE_MAVLINK_ENABLE_GPS_RAW, PX4LITE_MAVLINK_GPS_RAW_PERIOD_MS, MAVLINK_MSG_ID_GPS_RAW_INT, &s_next_gps_raw_ms, &s_stats.gps_raw_count, MavTx_SendGpsRaw, 0, MAV_TX_SCOPE_STANDARD},
    {"GNSS_DETAIL", PX4LITE_MAVLINK_ENABLE_GNSS_DETAIL, PX4LITE_MAVLINK_GNSS_DETAIL_PERIOD_MS, MAVLINK_MSG_ID_NAMED_VALUE_INT, &s_next_gnss_detail_ms, &s_stats.gnss_detail_count, MavTx_SendGnssDetail, 0, MAV_TX_SCOPE_EXTENSION},
    {"ATTITUDE", PX4LITE_MAVLINK_ENABLE_ATTITUDE, PX4LITE_MAVLINK_ATTITUDE_PERIOD_MS, MAVLINK_MSG_ID_ATTITUDE, &s_next_attitude_ms, &s_stats.attitude_count, MavTx_SendAttitude, 0, MAV_TX_SCOPE_STANDARD},
    {"POSITION", PX4LITE_MAVLINK_ENABLE_GLOBAL_POSITION, PX4LITE_MAVLINK_POSITION_PERIOD_MS, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, &s_next_position_ms, &s_stats.position_count, MavTx_SendPosition, 0, MAV_TX_SCOPE_STANDARD},
    {"SYS_STATUS", PX4LITE_MAVLINK_ENABLE_SYS_STATUS, PX4LITE_MAVLINK_SYS_STATUS_PERIOD_MS, MAVLINK_MSG_ID_SYS_STATUS, &s_next_sys_status_ms, &s_stats.sys_status_count, MavTx_SendSystemStatus, 0, MAV_TX_SCOPE_STANDARD},
    {"MODULE_STATE", PX4LITE_MAVLINK_ENABLE_MODULE_STATE, PX4LITE_MAVLINK_MODULE_STATE_PERIOD_MS, MAVLINK_MSG_ID_NAMED_VALUE_INT, &s_next_module_state_ms, &s_stats.module_state_count, MavTx_SendModuleState, MavTx_ModuleStateSuccess, MAV_TX_SCOPE_EXTENSION},
    {"BATTERY", PX4LITE_MAVLINK_ENABLE_BATTERY_STATUS, PX4LITE_MAVLINK_BATTERY_PERIOD_MS, MAVLINK_MSG_ID_BATTERY_STATUS, &s_next_battery_ms, &s_stats.battery_status_count, MavTx_SendBatteryStatus, 0, MAV_TX_SCOPE_STANDARD},
    {"PRESSURE", PX4LITE_MAVLINK_ENABLE_SCALED_PRESSURE, PX4LITE_MAVLINK_PRESSURE_PERIOD_MS, MAVLINK_MSG_ID_SCALED_PRESSURE, &s_next_pressure_ms, &s_stats.scaled_pressure_count, MavTx_SendScaledPressure, 0, MAV_TX_SCOPE_STANDARD},
    {"STATUSTEXT", PX4LITE_MAVLINK_ENABLE_STATUSTEXT, PX4LITE_MAVLINK_STATUSTEXT_PERIOD_MS, MAVLINK_MSG_ID_STATUSTEXT, &s_next_statustext_ms, &s_stats.statustext_count, MavTx_SendStatusText, 0, MAV_TX_SCOPE_STANDARD},
};

#define MAV_TX_CATALOG_COUNT ((uint8_t)(sizeof(s_mav_tx_catalog) / sizeof(s_mav_tx_catalog[0])))

Px4Lite_Result_t Px4Lite_MavlinkTxInit(uint32_t now_ms)
{
  memset(&s_message, 0, sizeof(s_message));
  memset(s_frame, 0, sizeof(s_frame));
  memset(&s_stats, 0, sizeof(s_stats));

  s_next_heartbeat_ms   = now_ms;
  s_next_gps_raw_ms     = now_ms + 100U;
  s_next_gnss_detail_ms = now_ms + 150U;
  s_next_attitude_ms    = now_ms + 50U;
  s_next_position_ms    = now_ms + 200U;
  s_next_sys_status_ms  = now_ms + 300U;
  s_next_module_state_ms = now_ms + 320U;
  s_next_battery_ms     = now_ms + 350U;
  s_next_pressure_ms    = now_ms + 400U;
  s_next_statustext_ms  = now_ms + 450U;
  s_catalog_index       = 0U;
  s_module_state_part   = 0U;
  s_remote_view_enabled = 0U;
  s_remote_view_target_node = (PX4LITE_NODE_ID == 0U) ? 1U : 0U;
#if PX4LITE_NODE_ROLE == PX4LITE_NODE_ROLE_SLAVE
  s_remote_view_start_ms = 0U;
#endif
  s_next_stream_request_ms = now_ms;
  s_stream_until_ms = 0U;
  s_stream_mask = 0U;
  memset(&s_pending_command, 0, sizeof(s_pending_command));
  memset(&s_pending_ack, 0, sizeof(s_pending_ack));
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_MavlinkTxRun(uint32_t now_ms)
{
  const MavTx_Item_t *item;
  Px4Lite_Result_t result;
  uint8_t checked;

  MavTx_UpdateStreamRequest(now_ms);

  result = MavTx_SendPendingAck(now_ms);
  if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }

  result = MavTx_SendPendingCommand(now_ms);
  if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }

  for (checked = 0U; checked < MAV_TX_CATALOG_COUNT; ++checked) {
    item = &s_mav_tx_catalog[s_catalog_index];
    s_catalog_index = (uint8_t)((s_catalog_index + 1U) % MAV_TX_CATALOG_COUNT);

    if ((item->enabled == 0U) || (MavTx_ItemAllowed(item, now_ms) == 0U) || (MavTx_TimeReached(now_ms, *item->next_ms) == 0U)) { continue; }

    result = item->encode(now_ms);
    MavTx_RecordResult(result, now_ms, item->period_ms, item->next_ms, item->message_id, item->success_count);
    if ((result == PX4LITE_OK) && (item->on_success != 0)) { item->on_success(); }
    if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }
  }

  return PX4LITE_IDLE;
}

void Px4Lite_MavlinkTxGetStats(Px4Lite_MavlinkTxStats_t *out)
{
  if (out != 0) { *out = s_stats; }
}

Px4Lite_Result_t Px4Lite_MavlinkSetRemoteView(uint8_t enabled, uint8_t target_node_id, uint32_t now_ms)
{
  if (target_node_id >= PX4LITE_REMOTE_NODE_MAX) { return PX4LITE_INVALID_PARAM; }
  if (target_node_id == (uint8_t)PX4LITE_NODE_ID) { return PX4LITE_INVALID_PARAM; }

  s_remote_view_target_node = target_node_id;
  s_remote_view_enabled = (enabled != 0U) ? 1U : 0U;
#if PX4LITE_NODE_ROLE == PX4LITE_NODE_ROLE_SLAVE
  s_remote_view_start_ms = (enabled != 0U) ? now_ms : 0U;
#endif
  s_next_stream_request_ms = now_ms;

  if (enabled != 0U) {
    MavTx_QueueStreamCommand(target_node_id, MAV_TX_STREAM_ACTION_START, PX4LITE_MAVLINK_STREAM_MASK_ALL, PX4LITE_MAVLINK_STREAM_LEASE_MS, now_ms);
  } else {
    MavTx_QueueStreamCommand(target_node_id, MAV_TX_STREAM_ACTION_STOP, 0U, 0U, now_ms);
  }

  return PX4LITE_OK;
}

uint8_t Px4Lite_MavlinkRemoteViewExpired(uint32_t now_ms)
{
#if PX4LITE_NODE_ROLE == PX4LITE_NODE_ROLE_SLAVE
  return ((s_remote_view_enabled != 0U) && (Px4Lite_ElapsedMs(now_ms, s_remote_view_start_ms) >= PX4LITE_REMOTE_VIEW_TIMEOUT_MS)) ? 1U : 0U;
#else
  (void)now_ms;
  return 0U;
#endif
}

Px4Lite_Result_t Px4Lite_MavlinkHandleCommandLong(uint16_t command, uint8_t source_system, uint8_t source_component, uint8_t target_system, uint8_t target_component, float param1, float param2, float param3, float param4, uint32_t now_ms)
{
  uint32_t lease_ms;

  if (MavTx_IsForThisSystem(target_system, target_component) == 0U) { return PX4LITE_IDLE; }
  if (command != PX4LITE_MAVLINK_STREAM_COMMAND) {
    MavTx_QueueAck(command, (uint8_t)MAV_RESULT_UNSUPPORTED, source_system, source_component);
    return PX4LITE_IDLE;
  }

  if ((uint8_t)param1 == MAV_TX_STREAM_ACTION_START) {
    lease_ms = (uint32_t)param4;
    if (lease_ms == 0U) { lease_ms = PX4LITE_MAVLINK_STREAM_LEASE_MS; }
    s_stream_mask = (uint32_t)param3;
    if (s_stream_mask == 0U) { s_stream_mask = PX4LITE_MAVLINK_STREAM_MASK_ALL; }
    s_stream_until_ms = now_ms + lease_ms;
    MavTx_QueueAck(command, (uint8_t)MAV_RESULT_ACCEPTED, source_system, source_component);
    return PX4LITE_OK;
  }

  s_stream_mask = 0U;
  s_stream_until_ms = 0U;
  MavTx_QueueAck(command, (uint8_t)MAV_RESULT_ACCEPTED, source_system, source_component);
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_MavlinkHandleCommandAck(uint16_t command, uint8_t result, uint8_t target_system, uint8_t target_component, uint32_t now_ms)
{
  (void)now_ms;
  if (MavTx_IsForThisSystem(target_system, target_component) == 0U) { return PX4LITE_IDLE; }
  if (command != PX4LITE_MAVLINK_STREAM_COMMAND) { return PX4LITE_IDLE; }
  s_stats.command_ack_rx_count++;
  (void)result;
  return PX4LITE_OK;
}
