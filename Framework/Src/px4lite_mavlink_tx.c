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
#include "px4lite_time.h"
#include "px4lite_topics.h"
#include "px4lite_remote_tunnel.h"
#include "px4lite_local_msglog.h"

#if defined(__CC_ARM)
#define MAVLINK_ALIGNED_FIELDS   0
#define MAVLINK_COMM_NUM_BUFFERS 1
#pragma diag_suppress 66
#endif

#include "common/mavlink.h"

#if defined(__CC_ARM)
#pragma diag_default 66
#endif

typedef enum {
  MAV_TX_SLOT_HEARTBEAT = 0,
  MAV_TX_SLOT_GPS_RAW,
  MAV_TX_SLOT_GNSS_DETAIL,
  MAV_TX_SLOT_ATTITUDE,
  MAV_TX_SLOT_POSITION,
  MAV_TX_SLOT_SYS_STATUS,
  MAV_TX_SLOT_BATTERY_STATUS,
  MAV_TX_SLOT_SCALED_PRESSURE,
  MAV_TX_SLOT_REMOTE_DETAIL,
  MAV_TX_SLOT_REMOTE_MOTOR,
  MAV_TX_SLOT_REMOTE_STATUS,
  MAV_TX_SLOT_STATUSTEXT,
  MAV_TX_SLOT_REMOTE_ALARM,
  MAV_TX_SLOT_REMOTE_LOG,
  MAV_TX_SLOT_COUNT
} MavTx_Slot_t;

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
static uint32_t s_next_battery_ms;
static uint32_t s_next_pressure_ms;
static uint32_t s_next_remote_detail_ms;
static uint32_t s_next_remote_motor_ms;
static uint32_t s_next_remote_status_ms;
static uint32_t s_next_statustext_ms;
static uint32_t s_next_remote_alarm_ms;
static uint32_t s_last_alarm_sig;
static uint8_t  s_alarm_ver;
static uint32_t s_next_remote_log_ms;
static uint16_t s_last_sent_log_seq;
static uint8_t s_remote_detail_index;
static uint8_t s_remote_motor_index;
static uint8_t s_remote_status_index;
static Px4Lite_MotorOutputs_t s_last_remote_motor;
static uint8_t s_last_remote_motor_valid;
static uint8_t s_remote_motor_urgent_remaining;
static uint8_t s_remote_motor_urgent_pair;
static uint8_t s_slot;

#define MAV_TX_DEG100_TO_RAD 0.0001745329252f
#define MAV_TX_REMOTE_DETAIL_COUNT 4U
#define MAV_TX_REMOTE_MOTOR_COUNT 2U
#define MAV_TX_REMOTE_STATUS_COUNT 3U
#define MAV_TX_REMOTE_MOTOR_URGENT_FRAMES 4U

/**
 * @brief 使用回绕安全差值判断一个毫秒截止时间是否到期。
 */
static uint8_t MavTx_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0) ? 1U : 0U;
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
 * @brief 将百分比限制到 0 到 100。
 */
static uint8_t MavTx_SaturatePercent(uint8_t value)
{
  return (value > 100U) ? 100U : value;
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

/**
 * @brief 编码并发送 MAVLink HEARTBEAT。
 */
static Px4Lite_Result_t MavTx_SendHeartbeat(void)
{
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
 * @brief 编码并发送 NAMED_VALUE_INT 扩展字段。
 */
static Px4Lite_Result_t MavTx_SendNamedValueInt(uint32_t now_ms, const char *name, uint8_t name_len, int32_t value)
{
  mavlink_named_value_int_t packet;

  if ((name == 0) || (name_len > 10U)) { return PX4LITE_INVALID_PARAM; }

  memset(&packet, 0, sizeof(packet));
  packet.time_boot_ms = now_ms;
  packet.value        = value;
  memcpy(packet.name, name, name_len);

  (void)mavlink_msg_named_value_int_encode_chan(PX4LITE_MAVLINK_SYSTEM_ID, PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

  return MavTx_SendPrepared();
}

/**
 * @brief 发送远程显示使用的本地日期或时间。
 */
static Px4Lite_Result_t MavTx_SendRemoteTime(uint32_t now_ms, uint8_t send_date)
{
  Px4Lite_TimeSnapshot_t time_snapshot;

  if (Px4Lite_CopyTime(&time_snapshot) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&time_snapshot.header, now_ms, PX4LITE_TIME_STALE_MS) == 0U) { return PX4LITE_STALE; }

  if (send_date != 0U) {
    return MavTx_SendNamedValueInt(now_ms, "DATE_LOC", 8U, (int32_t)time_snapshot.local_date_ymd);
  }
  return MavTx_SendNamedValueInt(now_ms, "TIME_LOC", 8U, (int32_t)time_snapshot.local_time_hhmmss);
}

/**
 * @brief 发送远程显示使用的相对湿度。
 */
static Px4Lite_Result_t MavTx_SendRemoteHumidity(uint32_t now_ms)
{
  Px4Lite_SensorBaro_t baro;
  int32_t humidity_tenths;

  if (Px4Lite_CopyBaro(&baro) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&baro.header, now_ms, PX4LITE_BARO_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }

  humidity_tenths = (int32_t)((baro.relative_humidity_pct * 10.0f) + 0.5f);
  if (humidity_tenths < 0) { humidity_tenths = 0; }
  if (humidity_tenths > 1000) { humidity_tenths = 1000; }
  return MavTx_SendNamedValueInt(now_ms, "HUMIDITY", 8U, humidity_tenths);
}

/**
 * @brief 发送远程显示使用的电机电池电压、电量和低压标志。
 */
static Px4Lite_Result_t MavTx_SendRemoteMotorBattery(uint32_t now_ms)
{
  Px4Lite_BatteryStatus_t battery;
  uint32_t packed;

  if (Px4Lite_CopyBattery2(&battery) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&battery.header, now_ms, PX4LITE_BATTERY_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }

  packed = (uint32_t)MavTx_SaturateUint16(battery.voltage_mv);
  packed |= ((uint32_t)MavTx_SaturatePercent(battery.percent) << 16U);
  packed |= ((uint32_t)(battery.low_voltage != 0U ? 1U : 0U) << 24U);

  return MavTx_SendNamedValueInt(now_ms, "MOTBAT", 6U, (int32_t)packed);
}

/**
 * @brief 发送远程显示使用的电机占空比对。
 */
static Px4Lite_Result_t MavTx_SendRemoteMotorPair(uint32_t now_ms, uint8_t pair_index)
{
  Px4Lite_MotorOutputs_t motor;
  uint32_t packed;
  uint8_t first;

  if (Px4Lite_CopyMotor(&motor) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&motor.header, now_ms, PX4LITE_CONTROL_FAILSAFE_TIMEOUT_MS * 5U) == 0U) { return PX4LITE_STALE; }

  first  = (pair_index == 0U) ? 0U : 2U;
  packed = (uint32_t)MavTx_SaturatePercent(motor.duty_percent[first]);
  packed |= ((uint32_t)MavTx_SaturatePercent(motor.duty_percent[first + 1U]) << 8U);
  packed |= ((uint32_t)motor.run_state << 16U);
  packed |= ((uint32_t)MavTx_SaturatePercent(motor.speed_level) << 24U);

  return MavTx_SendNamedValueInt(now_ms, (pair_index == 0U) ? "MOTOR12" : "MOTOR34", 7U, (int32_t)packed);
}

/**
 * @brief 判断电机远程显示字段是否发生变化。
 */
static uint8_t MavTx_RemoteMotorChanged(const Px4Lite_MotorOutputs_t *motor, uint8_t *first_pair)
{
  uint8_t pair0_changed;
  uint8_t pair1_changed;

  if (first_pair == 0) { return 0U; }
  *first_pair = 0U;
  if (s_last_remote_motor_valid == 0U) { return 1U; }

  /* 先判断哪一对占空比真的变了，并据此选择优先发送的对。
     run_state/speed_level 是两对共有的元数据(speed_level = 四路油门最大值)，必须在
     占空比判定之后再作为兜底触发；否则拖动 3/4 改变最大油门时会因 speed_level 变化
     提前返回并把 first_pair 误设为 0，导致 MOTOR34 被 MOTOR12 抢占而明显不跟手。 */
  pair0_changed = ((s_last_remote_motor.duty_percent[0] != motor->duty_percent[0]) || (s_last_remote_motor.duty_percent[1] != motor->duty_percent[1])) ? 1U : 0U;
  pair1_changed = ((s_last_remote_motor.duty_percent[2] != motor->duty_percent[2]) || (s_last_remote_motor.duty_percent[3] != motor->duty_percent[3])) ? 1U : 0U;
  if ((pair1_changed != 0U) && (pair0_changed == 0U)) { *first_pair = 1U; }
  if ((pair0_changed != 0U) || (pair1_changed != 0U)) { return 1U; }

  if (s_last_remote_motor.run_state != motor->run_state) { return 1U; }
  if (s_last_remote_motor.speed_level != motor->speed_level) { return 1U; }
  return 0U;
}

/**
 * @brief 检测电机变化并安排短时高优先级重发。
 */
static void MavTx_UpdateRemoteMotorUrgent(uint32_t now_ms)
{
  Px4Lite_MotorOutputs_t motor;
  uint8_t first_pair;

  if (Px4Lite_CopyMotor(&motor) != PX4LITE_OK) { return; }
  if (Px4Lite_IsFresh(&motor.header, now_ms, PX4LITE_CONTROL_FAILSAFE_TIMEOUT_MS * 5U) == 0U) { return; }
  if (MavTx_RemoteMotorChanged(&motor, &first_pair) == 0U) { return; }

  s_last_remote_motor = motor;
  s_last_remote_motor_valid = 1U;
  s_remote_motor_urgent_remaining = MAV_TX_REMOTE_MOTOR_URGENT_FRAMES;
  s_remote_motor_urgent_pair = first_pair;
}

/**
 * @brief 在普通遥测轮转前优先发送电机变化帧。
 */
static Px4Lite_Result_t MavTx_RunRemoteMotorUrgent(uint32_t now_ms)
{
  Px4Lite_Result_t result;

  MavTx_UpdateRemoteMotorUrgent(now_ms);
  if ((PX4LITE_MAVLINK_ENABLE_REMOTE_MOTOR == 0U) || (s_remote_motor_urgent_remaining == 0U)) { return PX4LITE_IDLE; }

  result = MavTx_SendRemoteMotorPair(now_ms, s_remote_motor_urgent_pair);
  if (result == PX4LITE_OK) {
    s_remote_motor_urgent_pair = (uint8_t)((s_remote_motor_urgent_pair + 1U) % MAV_TX_REMOTE_MOTOR_COUNT);
    s_remote_motor_urgent_remaining--;
    s_stats.remote_motor_count++;
    s_stats.last_message_id = MAVLINK_MSG_ID_NAMED_VALUE_INT;
  } else if (result == PX4LITE_BUSY) {
    s_stats.busy_count++;
  } else if (result == PX4LITE_STALE) {
    s_stats.stale_count++;
  } else if ((result != PX4LITE_IDLE) && (result != PX4LITE_NOT_READY)) {
    s_stats.error_count++;
  }
  return result;
}

/**
 * @brief 轮转发送远程显示扩展字段（时间、日期、湿度、电机电池）。
 *
 * @details
 * 电机占空比已拆到独立的 MAV_TX_SLOT_REMOTE_MOTOR 高频槽，不再走本轮转。
 * BUSY 或发送错误时保持轮转位置，下次重试同一条，避免该字段被丢一整圈。
 */
static Px4Lite_Result_t MavTx_SendRemoteDetail(uint32_t now_ms)
{
  Px4Lite_Result_t result = PX4LITE_NOT_READY;
  uint8_t attempt;

  for (attempt = 0U; attempt < MAV_TX_REMOTE_DETAIL_COUNT; ++attempt) {
    uint8_t current = s_remote_detail_index;

    switch (current) {
      case 0U:
        result = MavTx_SendRemoteTime(now_ms, 0U);
        break;
      case 1U:
        result = MavTx_SendRemoteTime(now_ms, 1U);
        break;
      case 2U:
        result = MavTx_SendRemoteHumidity(now_ms);
        break;
      default:
        result = MavTx_SendRemoteMotorBattery(now_ms);
        break;
    }

    /* BUSY/发送错误：本条已就绪但未发出，保持位置下次重试。 */
    if ((result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR) || (result == PX4LITE_INVALID_PARAM)) { return result; }

    /* OK 或本条无数据：推进到下一条。 */
    s_remote_detail_index = (uint8_t)((s_remote_detail_index + 1U) % MAV_TX_REMOTE_DETAIL_COUNT);
    if (result == PX4LITE_OK) { return result; }
  }

  return result;
}

/**
 * @brief 轮转发送远程电机占空比对（MOTOR12 / MOTOR34）。
 *
 * @details
 * 单独占用 MAV_TX_SLOT_REMOTE_MOTOR，按 PX4LITE_MAVLINK_REMOTE_MOTOR_PERIOD_MS
 * 两路交替发送，使每路刷新周期明显小于接收端字段超时，避免远程电机滑块周期性归零。
 * BUSY 或发送错误时保持轮转位置，下次重试同一对。
 */
static Px4Lite_Result_t MavTx_SendRemoteMotor(uint32_t now_ms)
{
  Px4Lite_Result_t result = PX4LITE_NOT_READY;
  uint8_t attempt;

  for (attempt = 0U; attempt < MAV_TX_REMOTE_MOTOR_COUNT; ++attempt) {
    uint8_t current = s_remote_motor_index;

    result = MavTx_SendRemoteMotorPair(now_ms, current);

    /* BUSY/发送错误：本对已就绪但未发出，保持位置下次重试。 */
    if ((result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR) || (result == PX4LITE_INVALID_PARAM)) { return result; }

    /* OK 或本对无数据：推进到下一对。 */
    s_remote_motor_index = (uint8_t)((s_remote_motor_index + 1U) % MAV_TX_REMOTE_MOTOR_COUNT);
    if (result == PX4LITE_OK) { return result; }
  }

  return result;
}

/**
 * @brief 发送远程显示模块状态灯与系统就绪。
 *
 * @details
 * name 固定为 "MODSTAT"。uint32 布局每 4 位一个模块状态(Px4Lite_State_t)：
 * 0..3 GNSS，4..7 IMU，8..11 Baro，12..15 5G，16..19 Storage，20..23 电机显示，
 * bit24 system_ready。LoRa 状态由显示端本机提供，不在此发送。当前硬件没有
 * ESC/电机真实存在检测，电机显示状态固定为 ONLINE，只表示该状态灯不参与故障判定。
 */
static Px4Lite_Result_t MavTx_SendRemoteModuleStatus(uint32_t now_ms)
{
  Px4Lite_SystemHealth_t health;
  uint32_t packed;
  uint8_t system_ready;

  if (Px4Lite_CopyHealth(&health) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&health.header, now_ms, PX4LITE_HEALTH_PERIOD_MS * 5U) == 0U) { return PX4LITE_STALE; }

  system_ready = ((health.blocking_fault_mask == 0U) && (health.not_ready_mask == 0U)) ? 1U : 0U;
  packed  = ((uint32_t)health.module_state[PX4LITE_MODULE_GNSS] & 0x0FU);
  packed |= ((uint32_t)health.module_state[PX4LITE_MODULE_IMU] & 0x0FU) << 4U;
  packed |= ((uint32_t)health.module_state[PX4LITE_MODULE_BARO] & 0x0FU) << 8U;
  packed |= ((uint32_t)health.module_state[PX4LITE_MODULE_5G] & 0x0FU) << 12U;
  packed |= ((uint32_t)health.module_state[PX4LITE_MODULE_STORAGE] & 0x0FU) << 16U;
  packed |= ((uint32_t)PX4LITE_STATE_ONLINE & 0x0FU) << 20U;
  packed |= ((uint32_t)system_ready & 0x01U) << 24U;

  return MavTx_SendNamedValueInt(now_ms, "MODSTAT", 7U, (int32_t)packed);
}

/**
 * @brief 发送远程显示告警摘要：最高告警或活动来源位图。
 *
 * @param[in] send_mask 1 表示发送 "ALRMMSK" 活动来源位图，0 表示发送 "ALRMHI" 最高告警。
 *
 * @details
 * "ALRMHI" 布局：0..15 fault_code，16..23 source_id，24..27 severity。
 * "ALRMMSK" 为活动来源位图，bit 对应 source_id。无活动告警时两者均发 0，
 * 使显示端能据此清除远端告警显示。
 */
static Px4Lite_Result_t MavTx_SendRemoteAlarmSummary(uint32_t now_ms, uint8_t send_mask)
{
  Px4Lite_AlarmSnapshot_t alarm;
  uint32_t packed;
  uint16_t i;

  if (Px4Lite_CopyAlarmSnapshot(&alarm) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&alarm.header, now_ms, PX4LITE_HEALTH_PERIOD_MS * 10U) == 0U) { return PX4LITE_STALE; }

  if (send_mask != 0U) {
    packed = 0U;
    for (i = 0U; i < (uint16_t)PX4LITE_MODULE_COUNT; ++i) {
      const Px4Lite_AlarmRecord_t *record = &alarm.records[i];
      if ((record->active != 0U) && (record->fault_code != 0U) && (record->source_id < 32U)) {
        packed |= (1UL << record->source_id);
      }
    }
    return MavTx_SendNamedValueInt(now_ms, "ALRMMSK", 7U, (int32_t)packed);
  }

  packed  = ((uint32_t)alarm.highest_fault_code & 0xFFFFU);
  packed |= ((uint32_t)(alarm.highest_source_id & 0xFFU)) << 16U;
  packed |= ((uint32_t)((uint8_t)alarm.highest_severity & 0x0FU)) << 24U;
  return MavTx_SendNamedValueInt(now_ms, "ALRMHI", 6U, (int32_t)packed);
}

/**
 * @brief 轮转发送远程显示状态字段（模块状态灯、最高告警、活动告警位图）。
 *
 * @details
 * BUSY 或发送错误时保持轮转位置，下次重试同一条，避免该字段被丢一整圈。
 */
static Px4Lite_Result_t MavTx_SendRemoteStatus(uint32_t now_ms)
{
  Px4Lite_Result_t result = PX4LITE_NOT_READY;
  uint8_t attempt;

  for (attempt = 0U; attempt < MAV_TX_REMOTE_STATUS_COUNT; ++attempt) {
    uint8_t current = s_remote_status_index;

    switch (current) {
      case 0U:
        result = MavTx_SendRemoteModuleStatus(now_ms);
        break;
      case 1U:
        result = MavTx_SendRemoteAlarmSummary(now_ms, 0U);
        break;
      default:
        result = MavTx_SendRemoteAlarmSummary(now_ms, 1U);
        break;
    }

    /* BUSY/发送错误：本条已就绪但未发出，保持位置下次重试。 */
    if ((result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR) || (result == PX4LITE_INVALID_PARAM)) { return result; }

    /* OK 或本条无数据：推进到下一条。 */
    s_remote_status_index = (uint8_t)((s_remote_status_index + 1U) % MAV_TX_REMOTE_STATUS_COUNT);
    if (result == PX4LITE_OK) { return result; }
  }

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

  /* 远程显示只用 roll/pitch/yaw 三个角，不渲染角速度。rollspeed/pitchspeed/yawspeed
     是 ATTITUDE 的尾部字段，保持为 0(memset)后 MAVLink v2 尾部零截断会自动省掉这 12
     字节，每帧由 40B 降到 28B；仍是标准 ATTITUDE 报文，地面站可正常解析。 */
  memset(&packet, 0, sizeof(packet));
  packet.time_boot_ms = now_ms;
  packet.roll         = MavTx_Deg100ToRad(navigation.roll_deg100);
  packet.pitch        = MavTx_Deg100ToRad(navigation.pitch_deg100);
  packet.yaw          = MavTx_Deg100ToRad(navigation.yaw_deg100);

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
  packet.temperature_press_diff = 1;

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
  Px4Lite_AlarmSnapshot_t alarm;
  mavlink_statustext_t packet;
  Px4Lite_Result_t result;

  if (Px4Lite_CopyAlarmSnapshot(&alarm) != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&alarm.header, now_ms, PX4LITE_HEALTH_PERIOD_MS * 10U) == 0U) { return PX4LITE_STALE; }
  if (alarm.active_count == 0U) { return PX4LITE_NOT_READY; }
  if (alarm.header.sequence == s_stats.last_alarm_sequence) { return PX4LITE_IDLE; }

  memset(&packet, 0, sizeof(packet));
  packet.severity  = MavTx_MapSeverity(alarm.highest_severity);
  packet.id        = alarm.highest_fault_code;
  packet.chunk_seq = 0U;
  MavTx_CopyText(packet.text, "PX4LITE ALARM", MavTx_ModuleName(alarm.highest_source_id), alarm.highest_fault_code);

  (void)mavlink_msg_statustext_encode_chan(PX4LITE_MAVLINK_SYSTEM_ID, PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_message, &packet);

  result = MavTx_SendPrepared();
  if (result == PX4LITE_OK) { s_stats.last_alarm_sequence = alarm.header.sequence; }
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
 * @brief 记录一个发送槽位结果并计算下一次发送截止时间。
 */
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
  s_next_battery_ms     = now_ms + 350U;
  s_next_pressure_ms    = now_ms + 400U;
  s_next_remote_detail_ms = now_ms + 450U;
  s_next_remote_motor_ms  = now_ms + 250U;
  s_next_remote_status_ms = now_ms + 550U;
  s_next_statustext_ms  = now_ms + 500U;
  s_next_remote_alarm_ms = now_ms + 600U;
  s_last_alarm_sig      = 0U;
  s_alarm_ver           = 0U;
  s_next_remote_log_ms  = now_ms + 650U;
  s_last_sent_log_seq   = 0U;
  s_remote_detail_index = 0U;
  s_remote_motor_index  = 0U;
  s_remote_status_index = 0U;
  memset(&s_last_remote_motor, 0, sizeof(s_last_remote_motor));
  s_last_remote_motor_valid = 0U;
  s_remote_motor_urgent_remaining = 0U;
  s_remote_motor_urgent_pair = 0U;
  s_slot                = (uint8_t)MAV_TX_SLOT_HEARTBEAT;
  return PX4LITE_OK;
}

/**
 * @brief 告警表：内容变化即发，否则按保活周期发；TUNNEL(0x8001) 打包 active 行。
 */
static Px4Lite_Result_t MavTx_SendRemoteAlarmTable(uint32_t now_ms)
{
  Px4Lite_AlarmSnapshot_t alarm;
  uint8_t payload[PX4LITE_TUNNEL_ALARM_MAX_BYTES];
  uint16_t plen;
  uint32_t sig;
  uint8_t changed;
  uint8_t keepalive_due;
  Px4Lite_Result_t result;

  if (Px4Lite_CopyAlarmSnapshot(&alarm) != PX4LITE_OK) { return PX4LITE_NOT_READY; }

  sig           = Px4Lite_AlarmTableSignature(alarm.records, (uint8_t)PX4LITE_MODULE_COUNT);
  changed       = (sig != s_last_alarm_sig) ? 1U : 0U;
  keepalive_due = MavTx_TimeReached(now_ms, s_next_remote_alarm_ms);
  if ((changed == 0U) && (keepalive_due == 0U)) { return PX4LITE_IDLE; }

  if (changed != 0U) { s_alarm_ver++; }

  plen = Px4Lite_PackAlarmTable(alarm.records, (uint8_t)PX4LITE_MODULE_COUNT, s_alarm_ver, now_ms, payload, sizeof(payload));
  if (plen == 0U) { return PX4LITE_IO_ERROR; }

  (void)mavlink_msg_tunnel_pack_chan(PX4LITE_MAVLINK_SYSTEM_ID, PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0,
                                     &s_message, 0U, 0U,
                                     PX4LITE_TUNNEL_PT_ALARM_TABLE, (uint8_t)plen, payload);
  result = MavTx_SendPrepared();
  if (result == PX4LITE_OK) {
    s_last_alarm_sig       = sig;
    s_next_remote_alarm_ms = now_ms + PX4LITE_MAVLINK_REMOTE_ALARM_PERIOD_MS;
    s_stats.remote_alarm_count++;
  }
  return result;
}

/**
 * @brief 消息日志：有新条目即增量发，否则按周期发 count=0 的 LOGSEQ 心跳。
 */
static Px4Lite_Result_t MavTx_SendRemoteLog(uint32_t now_ms)
{
  Px4Lite_LogEntry_t entries[PX4LITE_TUNNEL_LOG_MAX_ENTRIES];
  uint8_t payload[PX4LITE_TUNNEL_LOG_MAX_BYTES];
  uint16_t latest = 0U;
  uint16_t n;
  uint16_t plen;
  uint8_t keepalive_due;
  Px4Lite_Result_t result;

  n = Px4Lite_LocalMsgLogDrainSince(s_last_sent_log_seq, entries, PX4LITE_TUNNEL_LOG_MAX_ENTRIES, &latest);
  keepalive_due = MavTx_TimeReached(now_ms, s_next_remote_log_ms);
  if ((n == 0U) && (keepalive_due == 0U)) { return PX4LITE_IDLE; }

  plen = Px4Lite_PackMessageLog(entries, (uint8_t)n, latest, payload, sizeof(payload));
  if (plen == 0U) { return PX4LITE_IO_ERROR; }

  (void)mavlink_msg_tunnel_pack_chan(PX4LITE_MAVLINK_SYSTEM_ID, PX4LITE_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0,
                                     &s_message, 0U, 0U,
                                     PX4LITE_TUNNEL_PT_MESSAGE_LOG, (uint8_t)plen, payload);
  result = MavTx_SendPrepared();
  if (result == PX4LITE_OK) {
    if (n > 0U) { s_last_sent_log_seq = entries[n - 1U].sequence; }
    s_next_remote_log_ms = now_ms + PX4LITE_MAVLINK_REMOTE_LOG_PERIOD_MS;
    s_stats.remote_log_count++;
  }
  return result;
}

Px4Lite_Result_t Px4Lite_MavlinkTxRun(uint32_t now_ms)
{
  Px4Lite_Result_t result;
  uint8_t checked;

  result = MavTx_RunRemoteMotorUrgent(now_ms);
  if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }

  for (checked = 0U; checked < (uint8_t)MAV_TX_SLOT_COUNT; ++checked) {
    MavTx_Slot_t current = (MavTx_Slot_t)s_slot;

    s_slot = (uint8_t)((s_slot + 1U) % (uint8_t)MAV_TX_SLOT_COUNT);

    switch (current) {
      case MAV_TX_SLOT_HEARTBEAT:
        if ((PX4LITE_MAVLINK_ENABLE_HEARTBEAT == 0U) || (MavTx_TimeReached(now_ms, s_next_heartbeat_ms) == 0U)) { continue; }
        result = MavTx_SendHeartbeat();
        MavTx_RecordResult(result, now_ms, PX4LITE_MAVLINK_HEARTBEAT_PERIOD_MS, &s_next_heartbeat_ms, MAVLINK_MSG_ID_HEARTBEAT, &s_stats.heartbeat_count);
        return result;

      case MAV_TX_SLOT_GPS_RAW:
        if ((PX4LITE_MAVLINK_ENABLE_GPS_RAW == 0U) || (MavTx_TimeReached(now_ms, s_next_gps_raw_ms) == 0U)) { continue; }
        result = MavTx_SendGpsRaw(now_ms);
        MavTx_RecordResult(result, now_ms, PX4LITE_MAVLINK_GPS_RAW_PERIOD_MS, &s_next_gps_raw_ms, MAVLINK_MSG_ID_GPS_RAW_INT, &s_stats.gps_raw_count);
        if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }
        continue;

      case MAV_TX_SLOT_GNSS_DETAIL:
        if ((PX4LITE_MAVLINK_ENABLE_GNSS_DETAIL == 0U) || (MavTx_TimeReached(now_ms, s_next_gnss_detail_ms) == 0U)) { continue; }
        result = MavTx_SendGnssDetail(now_ms);
        MavTx_RecordResult(result, now_ms, PX4LITE_MAVLINK_GNSS_DETAIL_PERIOD_MS, &s_next_gnss_detail_ms, MAVLINK_MSG_ID_NAMED_VALUE_INT, &s_stats.gnss_detail_count);
        if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }
        continue;

      case MAV_TX_SLOT_ATTITUDE:
        if ((PX4LITE_MAVLINK_ENABLE_ATTITUDE == 0U) || (MavTx_TimeReached(now_ms, s_next_attitude_ms) == 0U)) { continue; }
        result = MavTx_SendAttitude(now_ms);
        MavTx_RecordResult(result, now_ms, PX4LITE_MAVLINK_ATTITUDE_PERIOD_MS, &s_next_attitude_ms, MAVLINK_MSG_ID_ATTITUDE, &s_stats.attitude_count);
        if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }
        continue;

      case MAV_TX_SLOT_POSITION:
        if ((PX4LITE_MAVLINK_ENABLE_GLOBAL_POSITION == 0U) || (MavTx_TimeReached(now_ms, s_next_position_ms) == 0U)) { continue; }
        result = MavTx_SendPosition(now_ms);
        MavTx_RecordResult(result, now_ms, PX4LITE_MAVLINK_POSITION_PERIOD_MS, &s_next_position_ms, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, &s_stats.position_count);
        if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }
        continue;

      case MAV_TX_SLOT_SYS_STATUS:
        if ((PX4LITE_MAVLINK_ENABLE_SYS_STATUS == 0U) || (MavTx_TimeReached(now_ms, s_next_sys_status_ms) == 0U)) { continue; }
        result = MavTx_SendSystemStatus(now_ms);
        MavTx_RecordResult(result, now_ms, PX4LITE_MAVLINK_SYS_STATUS_PERIOD_MS, &s_next_sys_status_ms, MAVLINK_MSG_ID_SYS_STATUS, &s_stats.sys_status_count);
        if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }
        continue;

      case MAV_TX_SLOT_BATTERY_STATUS:
        if ((PX4LITE_MAVLINK_ENABLE_BATTERY_STATUS == 0U) || (MavTx_TimeReached(now_ms, s_next_battery_ms) == 0U)) { continue; }
        result = MavTx_SendBatteryStatus(now_ms);
        MavTx_RecordResult(result, now_ms, PX4LITE_MAVLINK_BATTERY_PERIOD_MS, &s_next_battery_ms, MAVLINK_MSG_ID_BATTERY_STATUS, &s_stats.battery_status_count);
        if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }
        continue;

      case MAV_TX_SLOT_SCALED_PRESSURE:
        if ((PX4LITE_MAVLINK_ENABLE_SCALED_PRESSURE == 0U) || (MavTx_TimeReached(now_ms, s_next_pressure_ms) == 0U)) { continue; }
        result = MavTx_SendScaledPressure(now_ms);
        MavTx_RecordResult(result, now_ms, PX4LITE_MAVLINK_PRESSURE_PERIOD_MS, &s_next_pressure_ms, MAVLINK_MSG_ID_SCALED_PRESSURE, &s_stats.scaled_pressure_count);
        if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }
        continue;

      case MAV_TX_SLOT_REMOTE_DETAIL:
        if ((PX4LITE_MAVLINK_ENABLE_REMOTE_DETAIL == 0U) || (MavTx_TimeReached(now_ms, s_next_remote_detail_ms) == 0U)) { continue; }
        result = MavTx_SendRemoteDetail(now_ms);
        MavTx_RecordResult(result, now_ms, PX4LITE_MAVLINK_REMOTE_DETAIL_PERIOD_MS, &s_next_remote_detail_ms, MAVLINK_MSG_ID_NAMED_VALUE_INT, &s_stats.remote_detail_count);
        if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }
        continue;

      case MAV_TX_SLOT_REMOTE_MOTOR:
        if ((PX4LITE_MAVLINK_ENABLE_REMOTE_MOTOR == 0U) || (MavTx_TimeReached(now_ms, s_next_remote_motor_ms) == 0U)) { continue; }
        result = MavTx_SendRemoteMotor(now_ms);
        MavTx_RecordResult(result, now_ms, PX4LITE_MAVLINK_REMOTE_MOTOR_PERIOD_MS, &s_next_remote_motor_ms, MAVLINK_MSG_ID_NAMED_VALUE_INT, &s_stats.remote_motor_count);
        if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }
        continue;

      case MAV_TX_SLOT_REMOTE_STATUS:
        if ((PX4LITE_MAVLINK_ENABLE_REMOTE_STATUS == 0U) || (MavTx_TimeReached(now_ms, s_next_remote_status_ms) == 0U)) { continue; }
        result = MavTx_SendRemoteStatus(now_ms);
        MavTx_RecordResult(result, now_ms, PX4LITE_MAVLINK_REMOTE_STATUS_PERIOD_MS, &s_next_remote_status_ms, MAVLINK_MSG_ID_NAMED_VALUE_INT, &s_stats.remote_status_count);
        if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }
        continue;

      case MAV_TX_SLOT_STATUSTEXT:
        if ((PX4LITE_MAVLINK_ENABLE_STATUSTEXT == 0U) || (MavTx_TimeReached(now_ms, s_next_statustext_ms) == 0U)) { continue; }
        result = MavTx_SendStatusText(now_ms);
        MavTx_RecordResult(result, now_ms, PX4LITE_MAVLINK_STATUSTEXT_PERIOD_MS, &s_next_statustext_ms, MAVLINK_MSG_ID_STATUSTEXT, &s_stats.statustext_count);
        if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }
        continue;

      case MAV_TX_SLOT_REMOTE_ALARM:
        if (PX4LITE_MAVLINK_ENABLE_REMOTE_ALARM == 0U) { continue; }
        result = MavTx_SendRemoteAlarmTable(now_ms);
        if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }
        continue;

      case MAV_TX_SLOT_REMOTE_LOG:
        if (PX4LITE_MAVLINK_ENABLE_REMOTE_LOG == 0U) { continue; }
        result = MavTx_SendRemoteLog(now_ms);
        if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }
        continue;

      default:
        s_slot = (uint8_t)MAV_TX_SLOT_HEARTBEAT;
        break;
    }
  }

  return PX4LITE_IDLE;
}

void Px4Lite_MavlinkTxGetStats(Px4Lite_MavlinkTxStats_t *out)
{
  if (out != 0) { *out = s_stats; }
}
