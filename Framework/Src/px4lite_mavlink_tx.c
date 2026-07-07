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
#include "px4lite_identity.h"
#include "px4lite_local_msglog.h"
#include "px4lite_platform.h"
#include "px4lite_remoteid_tx.h"
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
#define MAV_TX_DEG100_TO_RAD   (3.14159265358979323846f / 18000.0f)

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
  uint8_t awaiting_ack;
  uint8_t retry_count;
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
static uint32_t s_env_humidity_count;
static uint32_t s_alarm_status_count;
static uint32_t s_battery2_status_count;
static uint32_t s_datetime_count;
static uint32_t s_baro_altitude_count;

static uint32_t s_next_heartbeat_ms;
static uint32_t s_next_gps_raw_ms;
static uint32_t s_next_gnss_detail_ms;
static uint32_t s_next_attitude_ms;
static uint32_t s_next_position_ms;
static uint32_t s_next_baro_altitude_ms;
static uint32_t s_next_sys_status_ms;
static uint32_t s_next_module_state_ms;
static uint32_t s_next_battery_ms;
static uint32_t s_next_battery2_ms;
static uint32_t s_next_datetime_ms;
static uint32_t s_next_pressure_ms;
static uint32_t s_next_env_humidity_ms;
static uint32_t s_next_statustext_ms;
static uint32_t s_next_motor_ms;
static uint32_t s_next_alarm_status_ms;
static uint32_t s_next_log_ms;
static uint8_t s_catalog_index;
static MavTx_PendingCommand_t s_pending_command;
static MavTx_PendingAck_t s_pending_ack;
static uint32_t s_motor_status_count;
static uint32_t s_message_log_count;
static uint8_t s_alarm_status_part;
static uint16_t s_log_sync_cursor;
static uint16_t s_log_replay_cursor;
static uint8_t s_log_replay_active;
static uint32_t s_next_log_replay_ms;
static Px4Lite_LogEntry_t s_log_sync_entry;
static Px4Lite_MotorOutputs_t s_last_motor_snapshot;
static uint32_t s_last_battery2_sequence;
static uint8_t s_last_motor_valid;
static uint8_t s_motor_pair_part;
static uint8_t s_motor_urgent_part;
static uint8_t s_motor_urgent_remaining;
static uint8_t s_motor_urgent_hold;
static uint8_t s_module_state_part;
static uint8_t s_tx_enabled;

#if PX4LITE_ENABLE_RPI_MAVLINK
/* 树莓派链路独立发送序号。RPi 被视为一条独立 MAVLink 链路，其序号必须自成
   连续序列(镜像帧 + RPi 专属扩展帧共用本计数器)，才能让树莓派侧正确统计丢包。 */
static uint8_t s_rpi_tx_seq;
static uint32_t s_next_rpi_lorastat_ms;
static uint32_t s_next_rpi_ridstat_ms;
static uint32_t s_next_rpi_alarm_ms;
static uint32_t s_next_rpi_log_ms;
static uint32_t s_rpi_lorastat_count;
static uint32_t s_rpi_ridstat_count;
static uint32_t s_rpi_alarm_count;
static uint32_t s_rpi_log_count;
#endif

#define MAV_TX_MOTOR_PAIR_COUNT     2U
#define MAV_TX_MOTOR_URGENT_FRAMES  4U

/**
 * @brief 使用回绕安全差值判断一个毫秒截止时间是否到期。
 */
static uint8_t MavTx_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0) ? 1U : 0U;
}

static uint32_t MavTx_ItemPeriodMs(const MavTx_Item_t *item, uint32_t now_ms)
{
  (void)now_ms;
  if (item == 0) { return PX4LITE_MAVLINK_RETRY_PERIOD_MS; }
  return item->period_ms;
}

static uint8_t MavTx_ItemAllowed(const MavTx_Item_t *item, uint32_t now_ms)
{
  (void)now_ms;
  if (item == 0) { return 0U; }
  if ((item->scope == MAV_TX_SCOPE_EXTENSION) && (PX4LITE_MAVLINK_ENABLE_NAMED_VALUE_EXTENSIONS == 0U)) { return 0U; }
  return 1U;
}

#if PX4LITE_ENABLE_RPI_MAVLINK
/**
 * @brief 依据当前 magic/len/seq/sysid/compid/msgid 与载荷重算 MAVLink CRC。
 * @details
 * 改写 compid 或 seq 后 s_message.checksum 失配，接收端会判为 CRC 错误。
 * 本函数按 MAVLink1/2 头部布局重算校验和并写回 s_message.checksum，
 * 供 RPi 镜像帧与 RPi 专属扩展帧复用。
 */
static void MavTx_RecomputeChecksum(mavlink_message_t *msg)
{
  uint8_t header[MAVLINK_CORE_HEADER_LEN + 1U];
  uint8_t header_len;
  uint16_t checksum;

  if (msg->magic == MAVLINK_STX_MAVLINK1) {
    header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN;
    header[0] = msg->magic;
    header[1] = msg->len;
    header[2] = msg->seq;
    header[3] = msg->sysid;
    header[4] = msg->compid;
    header[5] = (uint8_t)(msg->msgid & 0xFFU);
  } else {
    header_len = MAVLINK_CORE_HEADER_LEN;
    header[0] = msg->magic;
    header[1] = msg->len;
    header[2] = msg->incompat_flags;
    header[3] = msg->compat_flags;
    header[4] = msg->seq;
    header[5] = msg->sysid;
    header[6] = msg->compid;
    header[7] = (uint8_t)(msg->msgid & 0xFFU);
    header[8] = (uint8_t)((msg->msgid >> 8) & 0xFFU);
    header[9] = (uint8_t)((msg->msgid >> 16) & 0xFFU);
  }

  checksum = crc_calculate(&header[1], (uint16_t)(header_len - 1U));
  crc_accumulate_buffer(&checksum, _MAV_PAYLOAD(msg), msg->len);
  crc_accumulate(mavlink_get_crc_extra(msg), &checksum);
  msg->checksum = checksum;
}
#endif

#if PX4LITE_ENABLE_RPI_MAVLINK
/**
 * @brief 把任意已编码 MAVLink 帧镜像一份到树莓派 USART1(公开接口，见头文件)。
 * @details
 * 改写 compid=193、用 RPi 独立连续序号(s_rpi_tx_seq)重编号并重算 CRC，只写 USART1，
 * 返回前恢复原帧 compid/seq/checksum，不影响调用方。RPi 链路自成连续序列(镜像帧、
 * RPi 专属帧、RemoteID 身份镜像共用本计数器)，让树莓派侧丢包统计正确；LoRa/RemoteID
 * 各自的 COMM_0 通道序号不受影响。
 */
Px4Lite_Result_t Px4Lite_MavlinkTxMirrorToRpi(mavlink_message_t *msg)
{
  uint8_t original_component_id;
  uint8_t original_seq;
  uint16_t original_checksum;
  uint16_t length;

  if (msg == 0) { return PX4LITE_INVALID_PARAM; }

  original_component_id = msg->compid;
  original_seq = msg->seq;
  original_checksum = msg->checksum;

  msg->compid = PX4LITE_RPI_MAVLINK_COMPONENT_ID;
  msg->seq    = s_rpi_tx_seq++;
  MavTx_RecomputeChecksum(msg);

  length = mavlink_msg_to_send_buffer(s_frame, msg);

  msg->compid   = original_component_id;
  msg->seq      = original_seq;
  msg->checksum = original_checksum;

  if ((length == 0U) || (length > (uint16_t)sizeof(s_frame))) { return PX4LITE_IO_ERROR; }
  return Px4Lite_RpiMavlinkSend(s_frame, length);
}
#endif

static void MavTx_SendRpiCopy(void)
{
#if PX4LITE_ENABLE_RPI_MAVLINK
  (void)Px4Lite_MavlinkTxMirrorToRpi(&s_message);
#endif
}

#if PX4LITE_ENABLE_RPI_MAVLINK
/**
 * @brief 将已编码(compid=193, COMM_0)的 s_message 仅发送到树莓派 USART1。
 * @details
 * 调用方用 encode_chan 在 COMM_0 上编码 RPi 专属扩展消息，这会占用一个 LoRa
 * 发送序号，但该帧不经 LoRa。这里先把 COMM_0 序号回退，避免远端 LoRa 接收方
 * 把这个"空洞"误判为丢包(见 Px4Lite_MavlinkTxRun 内相关注释)，再用 RPi 独立
 * 序号重新编号并重算 CRC，最后只写 USART1。
 */
static Px4Lite_Result_t MavTx_SendRpiExclusive(void)
{
  mavlink_status_t *chan;
  uint16_t length;

  chan = mavlink_get_channel_status(MAVLINK_COMM_0);
  if (chan != 0) { chan->current_tx_seq = s_message.seq; }

  s_message.seq = s_rpi_tx_seq++;
  MavTx_RecomputeChecksum(&s_message);

  length = mavlink_msg_to_send_buffer(s_frame, &s_message);
  if ((length == 0U) || (length > (uint16_t)sizeof(s_frame))) { return PX4LITE_IO_ERROR; }
  return Px4Lite_RpiMavlinkSend(s_frame, length);
}
#endif

static void MavTx_QueueAck(uint16_t command, uint8_t result, uint8_t target_system, uint8_t target_component)
{
  s_pending_ack.valid            = 1U;
  s_pending_ack.command          = command;
  s_pending_ack.result           = result;
  s_pending_ack.target_system    = target_system;
  s_pending_ack.target_component = target_component;
}




/**
 * @brief 判断当前是否处于双方互相查看的半双工高负载状态。
 */
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

static uint8_t MavTx_SaturatePercent(uint8_t value)
{
  return (value > 100U) ? 100U : value;
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
  Px4Lite_Result_t result;
  uint16_t length;

  length = mavlink_msg_to_send_buffer(s_frame, &s_message);
  if ((length == 0U) || (length > (uint16_t)sizeof(s_frame))) { return PX4LITE_IO_ERROR; }

  result = PX4LITE_OK;
  if (s_tx_enabled != 0U) {
    result = Px4Lite_LoRaSend(s_frame, length);
  }
  MavTx_SendRpiCopy();
  return result;
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

  (void)mavlink_msg_command_ack_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);
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

  (void)mavlink_msg_command_long_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);
  result = MavTx_SendPrepared();
  if (result == PX4LITE_OK) {
    s_pending_command.awaiting_ack = 1U;
    if (s_pending_command.retry_count < 255U) { s_pending_command.retry_count++; }
    if (s_pending_command.retry_count >= PX4LITE_MAVLINK_COMMAND_RETRY_MAX) {
      s_pending_command.valid = 0U;
    } else {
      s_pending_command.next_try_ms = now_ms + PX4LITE_MAVLINK_COMMAND_RETRY_MS;
    }
    s_stats.command_count++;
    s_stats.last_message_id = MAVLINK_MSG_ID_COMMAND_LONG;
  } else if (result == PX4LITE_BUSY) {
    s_pending_command.next_try_ms = now_ms + PX4LITE_MAVLINK_COMMAND_RETRY_MS;
  }
  return result;
}

/**
 * @brief 编码并发送 MAVLink HEARTBEAT。
 */
static Px4Lite_Result_t MavTx_SendHeartbeat(uint32_t now_ms)
{
  (void)now_ms;
  (void)mavlink_msg_heartbeat_pack_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0U, 0U, MAV_STATE_ACTIVE);

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

  (void)mavlink_msg_gps_raw_int_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

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

  (void)mavlink_msg_named_value_int_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

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

  (void)mavlink_msg_attitude_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

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

  (void)mavlink_msg_global_position_int_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

  return MavTx_SendPrepared();
}

/**
 * @brief 使用标准 NAMED_VALUE_INT 独立发送融合高度，不依赖 GPS 定位有效。
 * @details
 * 融合高度(气压计+GPS对齐校正)有独立的 PX4LITE_NAV_VALID_ALTITUDE 有效位，
 * GPS 未定位时 GLOBAL_POSITION_INT 整帧被抑制，这里单独广播高度，让地面站
 * 在没有 GPS 定位的情况下也能看到气压高度。
 */
static Px4Lite_Result_t MavTx_SendBaroAltitude(uint32_t now_ms)
{
  Px4Lite_VehicleNavigation_t navigation;
  mavlink_named_value_int_t packet;

  if (Px4Lite_CopyNavigation(&navigation) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&navigation.header, now_ms, PX4LITE_BARO_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }
  if ((navigation.valid_mask & PX4LITE_NAV_VALID_ALTITUDE) == 0U) { return PX4LITE_NOT_READY; }

  memset(&packet, 0, sizeof(packet));
  packet.time_boot_ms = now_ms;
  packet.value         = navigation.fused_altitude_mm;
  memcpy(packet.name, "BAROALT", 7U);

  (void)mavlink_msg_named_value_int_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

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

  (void)mavlink_msg_battery_status_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

  result = MavTx_SendPrepared();
  if (result == PX4LITE_OK) { s_stats.last_battery_sequence = battery.header.sequence; }
  return result;
}

/**
 * @brief 将第二电池/外设独立供电 topic 编码为 BAT2STAT 扩展帧。
 */
static Px4Lite_Result_t MavTx_SendBattery2Status(uint32_t now_ms)
{
  Px4Lite_BatteryStatus_t battery;
  mavlink_named_value_int_t packet;
  Px4Lite_Result_t result;
  uint32_t packed;

  result = Px4Lite_CopyBattery2(&battery);
  if (result != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&battery.header, now_ms, PX4LITE_BATTERY_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }
  if (battery.header.sequence == s_last_battery2_sequence) { return PX4LITE_IDLE; }

  memset(&packet, 0, sizeof(packet));
  packed = (battery.voltage_mv & 0xFFFFUL) |
           (((uint32_t)battery.percent & 0xFFUL) << 16U) |
           (((uint32_t)battery.low_voltage & 0x01UL) << 24U);
  packet.time_boot_ms =  battery.header.sample_time_ms;
  packet.value        = (int32_t)packed;
  memcpy(packet.name, "BAT2STAT", 8U);

  (void)mavlink_msg_named_value_int_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);
  result = MavTx_SendPrepared();
  if (result == PX4LITE_OK) { s_last_battery2_sequence = battery.header.sequence; }
  return result;
}

/**
 * @brief 将导航域 GNSS UTC 日期时间编码为 GNSSUTC 扩展帧。
 */
static Px4Lite_Result_t MavTx_SendDateTime(uint32_t now_ms)
{
  Px4Lite_VehicleNavigation_t navigation;
  mavlink_named_value_int_t packet;
  Px4Lite_Result_t result;

  result = Px4Lite_CopyNavigation(&navigation);
  if (result != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&navigation.header, now_ms, PX4LITE_GNSS_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }
  if ((navigation.gnss_utc_date == 0U) || (navigation.gnss_utc_sec >= 86400UL)) { return PX4LITE_NOT_READY; }

  memset(&packet, 0, sizeof(packet));
  packet.time_boot_ms = navigation.gnss_utc_sec;
  packet.value        = (int32_t)navigation.gnss_utc_date;
  memcpy(packet.name, "GNSSUTC", 7U);

  (void)mavlink_msg_named_value_int_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);
  return MavTx_SendPrepared();
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

  (void)mavlink_msg_scaled_pressure_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

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

static Px4Lite_Result_t MavTx_SendEnvHumidity(uint32_t now_ms)
{
  Px4Lite_SensorBaro_t baro;
  mavlink_named_value_int_t packet;
  Px4Lite_Result_t result;
  int32_t humidity_x10;

  result = Px4Lite_CopyBaro(&baro);
  if (result != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&baro.header, now_ms, PX4LITE_BARO_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }

  humidity_x10 = (int32_t)((baro.relative_humidity_pct * 10.0f) + 0.5f);
  if (humidity_x10 < 0) { humidity_x10 = 0; }
  if (humidity_x10 > 1000) { humidity_x10 = 1000; }

  memset(&packet, 0, sizeof(packet));
  packet.time_boot_ms = baro.header.sample_time_ms;
  packet.value = humidity_x10;
  (void)memcpy(packet.name, "HUMIDITY", 8U);

  (void)mavlink_msg_named_value_int_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);
  return MavTx_SendPrepared();
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
    case PX4LITE_MODULE_CONTROL:
      return "CONTROL";
    case PX4LITE_MODULE_ALARM:
      return "ALARM";
    case PX4LITE_MODULE_SYSTEM:
      return "SYSTEM";
    case PX4LITE_MODULE_ESTIMATOR:
      return "EST";
    case PX4LITE_MODULE_STORAGE:
      return "SD";
    case PX4LITE_MODULE_5G:
      return "5G";
    case PX4LITE_MODULE_REMOTE_ID:
      return "REMOTEID";
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
  if (alarm_sequence == s_stats.last_alarm_sequence) { return PX4LITE_IDLE; }

  memset(&packet, 0, sizeof(packet));
  packet.chunk_seq = 0U;
  if (active_count == 0U) {
    packet.severity = (uint8_t)MAV_SEVERITY_INFO;
    packet.id       = 0U;
    MavTx_CopyText(packet.text, "PX4LITE ALARM", "NONE", 0U);
  } else {
    packet.severity = MavTx_MapSeverity(highest_severity);
    packet.id       = highest_fault_code;
    MavTx_CopyText(packet.text, "PX4LITE ALARM", MavTx_ModuleName(highest_source_id), highest_fault_code);
  }

  (void)mavlink_msg_statustext_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

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

  (void)mavlink_msg_sys_status_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

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
  uint8_t part;
  uint8_t i;
  uint8_t module_index;
  Px4Lite_Result_t result;

  if (Px4Lite_CopyHealth(&health) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&health.header, now_ms, PX4LITE_HEALTH_PERIOD_MS * 5U) == 0U) { return PX4LITE_STALE; }

  part = (uint8_t)(s_module_state_part % 2U);
  for (i = 0U; i < 8U; i++) {
    module_index = (uint8_t)((part * 8U) + i);
    if (module_index < (uint8_t)PX4LITE_MODULE_COUNT) {
      packed |= (((uint32_t)health.module_state[module_index] & 0x0FUL) << (i * 4U));
    }
  }

  memset(&packet, 0, sizeof(packet));
  packet.time_boot_ms = now_ms;
  packet.value        = (int32_t)packed;
  if (part == 0U) {
    memcpy(packet.name, "MODSTAT0", 8U);
  } else {
    memcpy(packet.name, "MODSTAT1", 8U);
  }

  (void)mavlink_msg_named_value_int_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

  result = MavTx_SendPrepared();
  if (result == PX4LITE_OK) { s_module_state_part = (uint8_t)((s_module_state_part + 1U) % 2U); }
  return result;
}

/**
 * @brief 模块状态分片成功发送后的分片推进。
 */

/**
 * @brief 使用 NAMED_VALUE_INT 发送四路电机目标油门。
 */
static Px4Lite_Result_t MavTx_SendAlarmStatus(uint32_t now_ms)
{
  mavlink_named_value_int_t packet;
  Px4Lite_AlarmRecord_t record;
  Px4Lite_Result_t result;
  uint32_t alarm_publish_ms;
  uint32_t packed;
  uint32_t active_mask = 0U;
  uint16_t active_count;
  uint16_t highest_fault_code;
  uint16_t highest_source_id;
  Px4Lite_AlarmSeverity_t highest_severity;
  uint16_t i;

  result = Px4Lite_CopyAlarmSummary(&alarm_publish_ms, 0, &active_count, &highest_fault_code, &highest_source_id, &highest_severity);
  if (result != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if ((uint32_t)(now_ms - alarm_publish_ms) > (PX4LITE_HEALTH_PERIOD_MS * 10U)) { return PX4LITE_STALE; }

  memset(&packet, 0, sizeof(packet));
  if (s_alarm_status_part == 0U) {
    packed = ((uint32_t)highest_fault_code & 0xFFFFUL) |
             (((uint32_t)highest_source_id & 0xFFUL) << 16U) |
             (((uint32_t)highest_severity & 0x0FUL) << 24U);
    memcpy(packet.name, "ALRMHI", 6U);
  } else {
    for (i = 0U; i < (uint16_t)PX4LITE_MODULE_COUNT; i++) {
      if ((Px4Lite_CopyAlarmRecord(i, &record) == PX4LITE_OK) && (record.active != 0U) && (i < 32U)) { active_mask |= (1UL << i); }
    }
    packed = active_mask;
    memcpy(packet.name, "ALRMMSK", 7U);
  }

  packet.time_boot_ms = alarm_publish_ms;
  packet.value        = (int32_t)packed;
  (void)mavlink_msg_named_value_int_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);
  result = MavTx_SendPrepared();
  if (result == PX4LITE_OK) { s_alarm_status_part ^= 1U; }
  return result;
}

static uint8_t MavTx_MotorChanged(const Px4Lite_MotorOutputs_t *motor)
{
  uint8_t i;

  if (motor == 0) { return 0U; }
  if (s_last_motor_valid == 0U) { return 1U; }
  if (motor->run_state != s_last_motor_snapshot.run_state) { return 1U; }
  if (motor->speed_level != s_last_motor_snapshot.speed_level) { return 1U; }
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    if (motor->duty_percent[i] != s_last_motor_snapshot.duty_percent[i]) { return 1U; }
  }
  return 0U;
}

static void MavTx_UpdateMotorUrgent(const Px4Lite_MotorOutputs_t *motor)
{
  if (motor == 0) { return; }
  if (MavTx_MotorChanged(motor) != 0U) {
    /* 只延长突发窗口，不回退 part：数值持续抖动时也要让两对电机轮流发出。 */
    s_motor_urgent_remaining = MAV_TX_MOTOR_URGENT_FRAMES;
  }
  s_last_motor_snapshot = *motor;
  s_last_motor_valid = 1U;
}

static Px4Lite_Result_t MavTx_SendMotorPair(uint32_t now_ms, uint8_t pair_part, const Px4Lite_MotorOutputs_t *motor_in)
{
  Px4Lite_MotorOutputs_t motor;
  mavlink_named_value_int_t packet;
  uint32_t packed;
  Px4Lite_Result_t result;
  uint8_t first;
  uint8_t duty0;
  uint8_t duty1;

  if (motor_in == 0) {
    result = Px4Lite_CopyMotor(&motor);
    if (result != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  } else {
    motor = *motor_in;
  }
  if (Px4Lite_IsFresh(&motor.header, now_ms, PX4LITE_CONTROL_FAILSAFE_TIMEOUT_MS * 5U) == 0U) { return PX4LITE_STALE; }
  MavTx_UpdateMotorUrgent(&motor);

  pair_part = (uint8_t)(pair_part % MAV_TX_MOTOR_PAIR_COUNT);
  first = (uint8_t)(pair_part * 2U);
  duty0 = (first < PX4LITE_MOTOR_COUNT) ? MavTx_SaturatePercent(motor.duty_percent[first]) : 0U;
  duty1 = ((uint8_t)(first + 1U) < PX4LITE_MOTOR_COUNT) ? MavTx_SaturatePercent(motor.duty_percent[(uint8_t)(first + 1U)]) : 0U;
  packed = ((uint32_t)duty0) |
           (((uint32_t)duty1) << 8U) |
           (((uint32_t)motor.run_state & 0x01UL) << 16U) |
           (((uint32_t)MavTx_SaturatePercent(motor.speed_level)) << 24U);

  memset(&packet, 0, sizeof(packet));
  packet.time_boot_ms = motor.header.sample_time_ms;
  packet.value        = (int32_t)packed;
  if (pair_part == 0U) {
    memcpy(packet.name, "MOTOR12", 7U);
  } else {
    memcpy(packet.name, "MOTOR34", 7U);
  }

  (void)mavlink_msg_named_value_int_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);
  return MavTx_SendPrepared();
}

static Px4Lite_Result_t MavTx_SendMotorStatus(uint32_t now_ms)
{
  Px4Lite_Result_t result;

  result = MavTx_SendMotorPair(now_ms, s_motor_pair_part, 0);
  if (result == PX4LITE_OK) { s_motor_pair_part = (uint8_t)((s_motor_pair_part + 1U) % MAV_TX_MOTOR_PAIR_COUNT); }
  return result;
}

static Px4Lite_Result_t MavTx_RunMotorUrgent(uint32_t now_ms)
{
  Px4Lite_MotorOutputs_t motor;
  Px4Lite_Result_t result;

  /* 每完成一轮突发就强制让位一次，保证心跳/姿态/GPS/电池等常规调度表
     一定能拿到执行机会，不会被持续变化的电机数据长期饿死。 */
  if (s_motor_urgent_hold != 0U) {
    s_motor_urgent_hold = 0U;
    return PX4LITE_IDLE;
  }

  result = Px4Lite_CopyMotor(&motor);
  if (result != PX4LITE_OK) { return PX4LITE_IDLE; }
  if (Px4Lite_IsFresh(&motor.header, now_ms, PX4LITE_CONTROL_FAILSAFE_TIMEOUT_MS * 5U) == 0U) { return PX4LITE_IDLE; }
  MavTx_UpdateMotorUrgent(&motor);
  if (s_motor_urgent_remaining == 0U) { return PX4LITE_IDLE; }

  result = MavTx_SendMotorPair(now_ms, s_motor_urgent_part, &motor);
  if (result == PX4LITE_OK) {
    s_motor_urgent_part = (uint8_t)((s_motor_urgent_part + 1U) % MAV_TX_MOTOR_PAIR_COUNT);
    s_motor_urgent_remaining--;
    if (s_motor_urgent_remaining == 0U) { s_motor_urgent_hold = 1U; }
    s_motor_status_count++;
    s_stats.last_message_id = MAVLINK_MSG_ID_NAMED_VALUE_INT;
  } else if (result == PX4LITE_BUSY) {
    s_stats.busy_count++;
  } else if (result != PX4LITE_IDLE) {
    s_stats.error_count++;
  }
  return result;
}

/**
 * @brief 增量发送本机结构化消息日志，并周期性发起一轮低频全量重播。
 * @details
 * 重播不依赖对端主动请求重同步：LoRa 链路上控制类报文本就容易被顶掉/
 * 丢失，被动等待对端发现"日志断档"再回传请求同样不可靠。这里改为
 * 发送端每隔 PX4LITE_MAVLINK_LOG_REPLAY_PERIOD_MS 主动把重播游标拉回
 * 缓冲区最早一条，独立于增量游标重新发一遍。接收端按 sequence 去重
 * (MavRx_AppendRemoteLog)，重复收到同一条不会产生重复行，可以安全重发。
 */
static Px4Lite_Result_t MavTx_SendMessageLog(uint32_t now_ms)
{
  mavlink_named_value_int_t packet;
  uint16_t latest_seq = 0U;
  uint16_t count;
  uint32_t packed;
  Px4Lite_Result_t result;
  uint16_t cursor;

  if ((s_log_replay_active == 0U) && (MavTx_TimeReached(now_ms, s_next_log_replay_ms) != 0U)) {
    s_log_replay_active  = 1U;
    s_log_replay_cursor  = 0U;
    s_next_log_replay_ms = now_ms + PX4LITE_MAVLINK_LOG_REPLAY_PERIOD_MS;
  }

  cursor = (s_log_replay_active != 0U) ? s_log_replay_cursor : s_log_sync_cursor;
  count  = Px4Lite_LocalMsgLogDrainSince(cursor, &s_log_sync_entry, 1U, &latest_seq);
  if (count == 0U) {
    if (s_log_replay_active != 0U) {
      s_log_replay_active = 0U;
    } else {
      s_log_sync_cursor = latest_seq;
    }
    return PX4LITE_IDLE;
  }

  packed = ((uint32_t)s_log_sync_entry.sequence & 0xFFFFUL) |
           (((uint32_t)s_log_sync_entry.message_id & 0xFFUL) << 16U) |
           (((uint32_t)s_log_sync_entry.severity & 0x0FUL) << 24U) |
           (((uint32_t)s_log_sync_entry.active & 0x01UL) << 28U) |
           (((uint32_t)s_log_sync_entry.source_id & 0x07UL) << 29U);

  memset(&packet, 0, sizeof(packet));
  packet.time_boot_ms = s_log_sync_entry.time_hhmmss;
  packet.value        = (int32_t)packed;
  memcpy(packet.name, "LOGSYNC", 7U);

  (void)mavlink_msg_named_value_int_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);
  result = MavTx_SendPrepared();
  if (result == PX4LITE_OK) {
    if (s_log_replay_active != 0U) {
      s_log_replay_cursor = s_log_sync_entry.sequence;
    } else {
      s_log_sync_cursor = s_log_sync_entry.sequence;
    }
  }
  (void)now_ms;
  return result;
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

#if PX4LITE_ENABLE_RPI_MAVLINK
/* 小端写 16 位，TUNNEL payload 多字节字段统一小端(与 RPi 侧 ReadU16LE 对齐)。 */
static void MavTx_TunPutU16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

/**
 * @brief 打包完整告警表为 TUNNEL payload(payload_type=0x8001)。
 * @details
 * 表头 2 字节：ver + active_count；每行 7 字节：source_id(1)+fault_code(2,LE)+
 * severity(1)+active(1)+age_s(2,LE)。只收 active 行，最多 PX4LITE_MODULE_COUNT 行。
 * 布局与 RPi 侧 DecodeAlarmTable 逐字段一致。返回 payload 长度(至少 2 字节表头)。
 */
static uint16_t MavTx_PackAlarmTable(uint8_t ver, uint32_t now_ms, uint8_t *out, uint16_t out_cap)
{
  uint16_t off = 2U;
  uint8_t n = 0U;
  uint16_t i;
  Px4Lite_AlarmRecord_t rec;

  if ((out == 0) || (out_cap < 2U)) { return 0U; }

  for (i = 0U; i < (uint16_t)PX4LITE_MODULE_COUNT; ++i) {
    uint8_t *row;
    uint32_t age_s;

    if (Px4Lite_CopyAlarmRecord(i, &rec) != PX4LITE_OK) { continue; }
    if (rec.active == 0U) { continue; }
    if ((uint16_t)(off + 7U) > out_cap) { break; }

    row    = &out[off];
    row[0] = (uint8_t)(rec.source_id & 0xFFU);
    MavTx_TunPutU16(&row[1], rec.fault_code);
    row[3] = (uint8_t)rec.severity;
    row[4] = rec.active;
    age_s  = (now_ms - rec.raised_ms) / 1000U;
    if (age_s > 0xFFFFU) { age_s = 0xFFFFU; }
    MavTx_TunPutU16(&row[5], (uint16_t)age_s);

    off = (uint16_t)(off + 7U);
    n++;
  }

  out[0] = ver;
  out[1] = n;
  return off;
}

/**
 * @brief 打包当前完整消息日志为 TUNNEL payload(payload_type=0x8002)。
 * @details
 * 表头 3 字节：latest_seq(2,LE)+count；每条 8 字节：sequence(2,LE)+message_id(2,LE)+
 * time(3 字节，按 RPi 契约填时/分/秒各 1 字节)+severity(1)。取当前日志全量快照
 * (最多 PX4LITE_LOCAL_LOG_CAP 条)，RPi 侧按 sequence 去重。返回 payload 长度。
 * 注意 time 用时/分/秒三字节，非旧 tunnel 的 U24 十进制，遵循 RPi 数据格式文档。
 */
static uint16_t MavTx_PackMessageLogTable(uint8_t *out, uint16_t out_cap)
{
  Px4Lite_LogEntry_t entries[PX4LITE_LOCAL_LOG_CAP];
  uint16_t latest_seq = 0U;
  uint16_t count;
  uint16_t off = 3U;
  uint8_t n = 0U;
  uint16_t i;

  if ((out == 0) || (out_cap < 3U)) { return 0U; }

  count = Px4Lite_LocalMsgLogCopy(entries, (uint16_t)PX4LITE_LOCAL_LOG_CAP, 0, &latest_seq);
  for (i = 0U; i < count; ++i) {
    uint8_t *row;
    uint32_t t;

    if ((uint16_t)(off + 8U) > out_cap) { break; }
    row = &out[off];
    MavTx_TunPutU16(&row[0], entries[i].sequence);
    MavTx_TunPutU16(&row[2], entries[i].message_id);
    t = entries[i].time_hhmmss;
    row[4] = (uint8_t)((t / 10000U) % 100U); /* 时 */
    row[5] = (uint8_t)((t / 100U) % 100U);   /* 分 */
    row[6] = (uint8_t)(t % 100U);            /* 秒 */
    row[7] = (uint8_t)entries[i].severity;

    off = (uint16_t)(off + 8U);
    n++;
  }

  MavTx_TunPutU16(&out[0], latest_seq);
  out[2] = n;
  return off;
}

/**
 * @brief RPi 专属：用 NAMED_VALUE_INT("LORASTAT") 报告 LoRa 链路状态。
 * @details
 * 树莓派不接 LoRa 串口，需由本机把 LoRa 链路状态转达。value 位布局：
 *   [0:15]  接收侧估算丢包率，单位 0.1%(0..1000)
 *   [16:23] LoRa 节点 ID
 *   [24]    LoRa 模块在位标志
 *   [25:27] LoRa 链路状态枚举(Px4Lite_State_t)
 * 仅发 USART1，不进入 LoRa 目录。
 */
static Px4Lite_Result_t MavTx_SendRpiLoraStatus(uint32_t now_ms)
{
  Px4Lite_CommDebugInfo_t info;
  mavlink_named_value_int_t packet;
  uint32_t packed;
  uint16_t loss;
  uint8_t state;

  Px4Lite_LoRaGetDebugInfo(&info);
  loss  = (info.rx_loss_rate_x10 > 1000U) ? 1000U : info.rx_loss_rate_x10;
  state = (uint8_t)Px4Lite_LoRaGetState(now_ms);

  packed  = (uint32_t)loss & 0xFFFFUL;
  packed |= ((uint32_t)Px4Lite_IdentityGetNodeId() & 0xFFUL) << 16U;
  packed |= ((uint32_t)((Px4Lite_LoRaIsPresent() != 0U) ? 1U : 0U)) << 24U;
  packed |= ((uint32_t)(state & 0x07U)) << 25U;

  memset(&packet, 0, sizeof(packet));
  packet.time_boot_ms = now_ms;
  packet.value        = (int32_t)packed;
  memcpy(packet.name, "LORASTAT", 8U);

  (void)mavlink_msg_named_value_int_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_RPI_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);
  return MavTx_SendRpiExclusive();
}

/**
 * @brief RPi 专属：用 NAMED_VALUE_INT("RIDSTAT") 报告 RemoteID 广播状态。
 * @details
 * value 位布局：[0:15] 位置广播成功计数低 16 位；[16:31] 编码/提交错误计数低 16 位。
 * time_boot_ms 复用 RemoteID 最近一次成功提交时间，树莓派据此判断广播是否仍在推进。
 * 仅发 USART1。
 */
static Px4Lite_Result_t MavTx_SendRpiRemoteIdStatus(uint32_t now_ms)
{
  Px4Lite_RemoteIdTxStats_t rid;
  mavlink_named_value_int_t packet;
  uint32_t packed;

  Px4Lite_RemoteIdTxGetStats(&rid);
  packed  = (uint32_t)(rid.location_count & 0xFFFFUL);
  packed |= ((uint32_t)(rid.error_count & 0xFFFFUL)) << 16U;

  memset(&packet, 0, sizeof(packet));
  packet.time_boot_ms = rid.last_success_ms;
  packet.value        = (int32_t)packed;
  memcpy(packet.name, "RIDSTAT", 7U);

  (void)now_ms;
  (void)mavlink_msg_named_value_int_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_RPI_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);
  return MavTx_SendRpiExclusive();
}

/**
 * @brief RPi 专属：用 TUNNEL(payload_type=0x8001) 发送完整活动告警表。
 * @details
 * 替代 LoRa 上的 ALRMHI/ALRMMSK 摘要，给树莓派逐行完整告警表(D1=全量)。无活动告警
 * 时 count=0，让 RPi 侧感知告警已清空。仅发 USART1。
 */
static Px4Lite_Result_t MavTx_SendRpiAlarmTable(uint32_t now_ms)
{
  mavlink_tunnel_t packet;
  uint16_t len;

  memset(&packet, 0, sizeof(packet));
  len = MavTx_PackAlarmTable(1U, now_ms, packet.payload, (uint16_t)sizeof(packet.payload));
  if (len < 2U) { return PX4LITE_NOT_READY; }

  packet.target_system    = 0U;
  packet.target_component = 0U;
  packet.payload_type     = 0x8001U;
  packet.payload_length   = (uint8_t)len;

  (void)mavlink_msg_tunnel_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_RPI_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);
  return MavTx_SendRpiExclusive();
}

/**
 * @brief RPi 专属：用 TUNNEL(payload_type=0x8002) 发送完整消息日志快照。
 * @details
 * 替代 LoRa 上的单条 LOGSYNC，一帧带当前全部日志(最多 PX4LITE_LOCAL_LOG_CAP 条)，
 * RPi 侧按 sequence 去重。周期性重发即为全量重播。仅发 USART1。
 */
static Px4Lite_Result_t MavTx_SendRpiMessageLogTable(uint32_t now_ms)
{
  mavlink_tunnel_t packet;
  uint16_t len;

  (void)now_ms;
  memset(&packet, 0, sizeof(packet));
  len = MavTx_PackMessageLogTable(packet.payload, (uint16_t)sizeof(packet.payload));
  if (len < 3U) { return PX4LITE_NOT_READY; }

  packet.target_system    = 0U;
  packet.target_component = 0U;
  packet.payload_type     = 0x8002U;
  packet.payload_length   = (uint8_t)len;

  (void)mavlink_msg_tunnel_encode_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_RPI_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);
  return MavTx_SendRpiExclusive();
}
#endif

static const MavTx_Item_t s_mav_tx_catalog[] = {
    {"HEARTBEAT", PX4LITE_MAVLINK_ENABLE_HEARTBEAT, PX4LITE_MAVLINK_HEARTBEAT_PERIOD_MS, MAVLINK_MSG_ID_HEARTBEAT, &s_next_heartbeat_ms, &s_stats.heartbeat_count, MavTx_SendHeartbeat, 0, MAV_TX_SCOPE_ALWAYS},
    {"GPS_RAW", PX4LITE_MAVLINK_ENABLE_GPS_RAW, PX4LITE_MAVLINK_GPS_RAW_PERIOD_MS, MAVLINK_MSG_ID_GPS_RAW_INT, &s_next_gps_raw_ms, &s_stats.gps_raw_count, MavTx_SendGpsRaw, 0, MAV_TX_SCOPE_STANDARD},
    {"GNSS_DETAIL", PX4LITE_MAVLINK_ENABLE_GNSS_DETAIL, PX4LITE_MAVLINK_GNSS_DETAIL_PERIOD_MS, MAVLINK_MSG_ID_NAMED_VALUE_INT, &s_next_gnss_detail_ms, &s_stats.gnss_detail_count, MavTx_SendGnssDetail, 0, MAV_TX_SCOPE_EXTENSION},
    {"ATTITUDE", PX4LITE_MAVLINK_ENABLE_ATTITUDE, PX4LITE_MAVLINK_ATTITUDE_PERIOD_MS, MAVLINK_MSG_ID_ATTITUDE, &s_next_attitude_ms, &s_stats.attitude_count, MavTx_SendAttitude, 0, MAV_TX_SCOPE_STANDARD},
    {"POSITION", PX4LITE_MAVLINK_ENABLE_GLOBAL_POSITION, PX4LITE_MAVLINK_POSITION_PERIOD_MS, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, &s_next_position_ms, &s_stats.position_count, MavTx_SendPosition, 0, MAV_TX_SCOPE_STANDARD},
    {"BAROALT", PX4LITE_MAVLINK_ENABLE_GLOBAL_POSITION, PX4LITE_MAVLINK_POSITION_PERIOD_MS, MAVLINK_MSG_ID_NAMED_VALUE_INT, &s_next_baro_altitude_ms, &s_baro_altitude_count, MavTx_SendBaroAltitude, 0, MAV_TX_SCOPE_EXTENSION},
    {"SYS_STATUS", PX4LITE_MAVLINK_ENABLE_SYS_STATUS, PX4LITE_MAVLINK_SYS_STATUS_PERIOD_MS, MAVLINK_MSG_ID_SYS_STATUS, &s_next_sys_status_ms, &s_stats.sys_status_count, MavTx_SendSystemStatus, 0, MAV_TX_SCOPE_STANDARD},
    {"MODULE_STATE", PX4LITE_MAVLINK_ENABLE_MODULE_STATE, PX4LITE_MAVLINK_MODULE_STATE_PERIOD_MS, MAVLINK_MSG_ID_NAMED_VALUE_INT, &s_next_module_state_ms, &s_stats.module_state_count, MavTx_SendModuleState, 0, MAV_TX_SCOPE_EXTENSION},
    {"BATTERY", PX4LITE_MAVLINK_ENABLE_BATTERY_STATUS, PX4LITE_MAVLINK_BATTERY_PERIOD_MS, MAVLINK_MSG_ID_BATTERY_STATUS, &s_next_battery_ms, &s_stats.battery_status_count, MavTx_SendBatteryStatus, 0, MAV_TX_SCOPE_STANDARD},
    {"BAT2STAT", PX4LITE_MAVLINK_ENABLE_BATTERY_STATUS, PX4LITE_MAVLINK_BATTERY_PERIOD_MS, MAVLINK_MSG_ID_NAMED_VALUE_INT, &s_next_battery2_ms, &s_battery2_status_count, MavTx_SendBattery2Status, 0, MAV_TX_SCOPE_EXTENSION},
    {"GNSSUTC", PX4LITE_MAVLINK_ENABLE_DATETIME, PX4LITE_MAVLINK_DATETIME_PERIOD_MS, MAVLINK_MSG_ID_NAMED_VALUE_INT, &s_next_datetime_ms, &s_datetime_count, MavTx_SendDateTime, 0, MAV_TX_SCOPE_EXTENSION},
    {"PRESSURE", PX4LITE_MAVLINK_ENABLE_SCALED_PRESSURE, PX4LITE_MAVLINK_PRESSURE_PERIOD_MS, MAVLINK_MSG_ID_SCALED_PRESSURE, &s_next_pressure_ms, &s_stats.scaled_pressure_count, MavTx_SendScaledPressure, 0, MAV_TX_SCOPE_STANDARD},
    {"ENV_HUM", PX4LITE_MAVLINK_ENABLE_ENV_HUMIDITY, PX4LITE_MAVLINK_ENV_HUMIDITY_PERIOD_MS, MAVLINK_MSG_ID_NAMED_VALUE_INT, &s_next_env_humidity_ms, &s_env_humidity_count, MavTx_SendEnvHumidity, 0, MAV_TX_SCOPE_EXTENSION},
    {"ALARM", PX4LITE_MAVLINK_ENABLE_ALARM_STATUS, PX4LITE_MAVLINK_ALARM_STATUS_PERIOD_MS, MAVLINK_MSG_ID_NAMED_VALUE_INT, &s_next_alarm_status_ms, &s_alarm_status_count, MavTx_SendAlarmStatus, 0, MAV_TX_SCOPE_EXTENSION},
    {"STATUSTEXT", PX4LITE_MAVLINK_ENABLE_STATUSTEXT, PX4LITE_MAVLINK_STATUSTEXT_PERIOD_MS, MAVLINK_MSG_ID_STATUSTEXT, &s_next_statustext_ms, &s_stats.statustext_count, MavTx_SendStatusText, 0, MAV_TX_SCOPE_STANDARD},
    {"MOTOR", PX4LITE_MAVLINK_ENABLE_MOTOR_STATUS, PX4LITE_MAVLINK_MOTOR_PERIOD_MS, MAVLINK_MSG_ID_NAMED_VALUE_INT, &s_next_motor_ms, &s_motor_status_count, MavTx_SendMotorStatus, 0, MAV_TX_SCOPE_EXTENSION},
    {"LOG", PX4LITE_MAVLINK_ENABLE_MESSAGE_LOG, PX4LITE_MAVLINK_MESSAGE_LOG_PERIOD_MS, MAVLINK_MSG_ID_NAMED_VALUE_INT, &s_next_log_ms, &s_message_log_count, MavTx_SendMessageLog, 0, MAV_TX_SCOPE_EXTENSION},
#if PX4LITE_ENABLE_RPI_MAVLINK
    /* 树莓派专属扩展：只发 USART1(compid 193)，不进入 LoRa 空口。scope 用 ALWAYS，
       不受 LoRa NAMED_VALUE 扩展策略约束，因为它们是独立于 LoRa 的 RPi 出口数据。 */
    {"RPILORA", 1U, PX4LITE_MAVLINK_RPI_LORASTAT_PERIOD_MS, MAVLINK_MSG_ID_NAMED_VALUE_INT, &s_next_rpi_lorastat_ms, &s_rpi_lorastat_count, MavTx_SendRpiLoraStatus, 0, MAV_TX_SCOPE_ALWAYS},
    {"RPIRID", 1U, PX4LITE_MAVLINK_RPI_RIDSTAT_PERIOD_MS, MAVLINK_MSG_ID_NAMED_VALUE_INT, &s_next_rpi_ridstat_ms, &s_rpi_ridstat_count, MavTx_SendRpiRemoteIdStatus, 0, MAV_TX_SCOPE_ALWAYS},
    {"RPIALRM", 1U, PX4LITE_MAVLINK_RPI_ALARM_PERIOD_MS, MAVLINK_MSG_ID_TUNNEL, &s_next_rpi_alarm_ms, &s_rpi_alarm_count, MavTx_SendRpiAlarmTable, 0, MAV_TX_SCOPE_ALWAYS},
    {"RPILOG", 1U, PX4LITE_MAVLINK_RPI_LOG_PERIOD_MS, MAVLINK_MSG_ID_TUNNEL, &s_next_rpi_log_ms, &s_rpi_log_count, MavTx_SendRpiMessageLogTable, 0, MAV_TX_SCOPE_ALWAYS},
#endif
};

#define MAV_TX_CATALOG_COUNT ((uint8_t)(sizeof(s_mav_tx_catalog) / sizeof(s_mav_tx_catalog[0])))

Px4Lite_Result_t Px4Lite_MavlinkTxInit(uint32_t now_ms)
{
  memset(&s_message, 0, sizeof(s_message));
  memset(s_frame, 0, sizeof(s_frame));
  memset(&s_stats, 0, sizeof(s_stats));
  s_env_humidity_count = 0U;
  s_alarm_status_count = 0U;
  s_battery2_status_count = 0U;
  s_datetime_count = 0U;
  s_baro_altitude_count = 0U;
  s_motor_status_count = 0U;
  s_message_log_count  = 0U;
  s_alarm_status_part  = 0U;
  s_log_sync_cursor    = 0U;
  s_log_replay_cursor  = 0U;
  s_log_replay_active  = 0U;
  s_next_log_replay_ms = now_ms + PX4LITE_MAVLINK_LOG_REPLAY_PERIOD_MS;
  memset(&s_log_sync_entry, 0, sizeof(s_log_sync_entry));
  memset(&s_last_motor_snapshot, 0, sizeof(s_last_motor_snapshot));
  s_last_battery2_sequence = 0U;
  s_last_motor_valid = 0U;
  s_motor_pair_part = 0U;
  s_motor_urgent_part = 0U;
  s_motor_urgent_remaining = 0U;
  s_motor_urgent_hold = 0U;
  s_module_state_part = 0U;
  s_tx_enabled = 1U;

#if PX4LITE_ENABLE_RPI_MAVLINK
  if (Px4Lite_RpiMavlinkInit() != PX4LITE_OK) { return PX4LITE_IO_ERROR; }
  s_rpi_tx_seq           = 0U;
  s_rpi_lorastat_count   = 0U;
  s_rpi_ridstat_count    = 0U;
  s_rpi_alarm_count      = 0U;
  s_rpi_log_count        = 0U;
  s_next_rpi_lorastat_ms = now_ms + 600U;
  s_next_rpi_ridstat_ms  = now_ms + 650U;
  s_next_rpi_alarm_ms    = now_ms + 700U;
  s_next_rpi_log_ms      = now_ms + 750U;
#endif

  s_next_heartbeat_ms   = now_ms + 100U;
  s_next_gps_raw_ms     = now_ms + 100U;
  s_next_gnss_detail_ms = now_ms + 150U;
  s_next_attitude_ms    = now_ms + 50U;
  s_next_position_ms    = now_ms + 200U;
  s_next_sys_status_ms  = now_ms + 300U;
  s_next_module_state_ms = now_ms + 320U;
  s_next_battery_ms     = now_ms + 350U;
  s_next_battery2_ms    = now_ms + 370U;
  s_next_datetime_ms    = now_ms + 390U;
  s_next_pressure_ms    = now_ms + 400U;
  s_next_env_humidity_ms = now_ms + 430U;
  s_next_statustext_ms  = now_ms + 450U;
  s_next_motor_ms       = now_ms + 480U;
  s_next_alarm_status_ms = now_ms + 500U;
  s_next_log_ms         = now_ms + 2500U;
  s_catalog_index       = 0U;
  memset(&s_pending_command, 0, sizeof(s_pending_command));
  memset(&s_pending_ack, 0, sizeof(s_pending_ack));
  Px4Lite_LocalMsgLogReset();
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_MavlinkTxRun(uint32_t now_ms)
{
  const MavTx_Item_t *item;
  Px4Lite_Result_t result;
  uint8_t checked;

#if !PX4LITE_ENABLE_RPI_MAVLINK
  if (s_tx_enabled == 0U) { return PX4LITE_IDLE; }
#endif

  /* 上一帧仍在发送时，任何 encode_chan 调用都会白白消耗 MAVLink 通道序号
     (库内部自增)，但帧实际并未发出，接收端会把这个空洞误判为丢包。
     发送前先确认通道空闲，忙时直接跳过整轮编码，不做任何 encode 尝试。 */
  if ((s_tx_enabled != 0U) && (Px4Lite_LoRaIsTxIdle() == 0U)) { return PX4LITE_BUSY; }

  result = MavTx_SendPendingAck(now_ms);
  if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }

  result = MavTx_SendPendingCommand(now_ms);
  if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }

  result = MavTx_RunMotorUrgent(now_ms);
  if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }

  for (checked = 0U; checked < MAV_TX_CATALOG_COUNT; ++checked) {
    item = &s_mav_tx_catalog[s_catalog_index];
    s_catalog_index = (uint8_t)((s_catalog_index + 1U) % MAV_TX_CATALOG_COUNT);

    if ((item->enabled == 0U) || (MavTx_ItemAllowed(item, now_ms) == 0U) || (MavTx_TimeReached(now_ms, *item->next_ms) == 0U)) { continue; }

    result = item->encode(now_ms);
    MavTx_RecordResult(result, now_ms, MavTx_ItemPeriodMs(item, now_ms), item->next_ms, item->message_id, item->success_count);
    if ((result == PX4LITE_OK) && (item->on_success != 0)) { item->on_success(); }
    if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }
  }

  return PX4LITE_IDLE;
}

void Px4Lite_MavlinkTxGetStats(Px4Lite_MavlinkTxStats_t *out)
{
  if (out != 0) { *out = s_stats; }
}

Px4Lite_Result_t Px4Lite_MavlinkStartRemoteView(uint8_t target_node_id, uint32_t now_ms)
{
  (void)target_node_id;
  (void)now_ms;
  Px4Lite_MavlinkSetTxEnabled(0U);
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_MavlinkStopRemoteView(uint8_t target_node_id, uint32_t now_ms)
{
  (void)target_node_id;
  (void)now_ms;
  Px4Lite_MavlinkSetTxEnabled(1U);
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_MavlinkAcceptRemoteViewRequest(uint8_t requester_node_id, uint32_t lease_ms, uint32_t now_ms)
{
  (void)requester_node_id;
  (void)lease_ms;
  (void)now_ms;
  return PX4LITE_OK;
}

void Px4Lite_MavlinkAcceptRemoteViewStop(uint8_t requester_node_id, uint32_t now_ms)
{
  (void)requester_node_id;
  (void)now_ms;
}

void Px4Lite_MavlinkSetTxEnabled(uint8_t enabled)
{
  s_tx_enabled = (enabled != 0U) ? 1U : 0U;
}

uint8_t Px4Lite_MavlinkGetTxEnabled(void)
{
  return s_tx_enabled;
}

void Px4Lite_MavlinkQueueCommandAck(uint16_t command, uint8_t result, uint8_t target_system, uint8_t target_component)
{
  MavTx_QueueAck(command, result, target_system, target_component);
}

void Px4Lite_MavlinkRecordCommandAck(uint16_t command, uint8_t result, uint32_t now_ms)
{
  s_stats.command_ack_rx_count++;
  if ((s_pending_command.valid != 0U) && (s_pending_command.awaiting_ack != 0U) && (s_pending_command.command == command)) {
    if (result == MAV_RESULT_ACCEPTED) {
      s_pending_command.valid = 0U;
    } else {
      s_pending_command.next_try_ms = 0U;
    }
  }
  (void)now_ms;
}
