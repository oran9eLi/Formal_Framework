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
#define MAVLINK_ALIGNED_FIELDS 0
#define MAVLINK_COMM_NUM_BUFFERS 1
#pragma diag_suppress 66
#endif

#include "common/mavlink.h"

#if defined(__CC_ARM)
#pragma diag_default 66
#endif

typedef enum
{
    MAV_TX_SLOT_HEARTBEAT = 0,
    MAV_TX_SLOT_GPS_RAW,
    MAV_TX_SLOT_GNSS_DETAIL,
    MAV_TX_SLOT_ATTITUDE,
    MAV_TX_SLOT_POSITION,
    MAV_TX_SLOT_SYS_STATUS,
    MAV_TX_SLOT_BATTERY_STATUS,
    MAV_TX_SLOT_SCALED_PRESSURE,
    MAV_TX_SLOT_STATUSTEXT,
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
static uint32_t s_next_statustext_ms;
static uint8_t s_slot;

#define MAV_TX_DEG100_TO_RAD 0.0001745329252f

/**
 * @brief 使用回绕安全差值判断一个毫秒截止时间是否到期。
 */
static uint8_t MavTx_TimeReached(uint32_t now_ms,
                                 uint32_t deadline_ms)
{
    return ((int32_t)(now_ms - deadline_ms) >= 0)
               ? 1U
               : 0U;
}

/**
 * @brief 将有符号 Framework 数值饱和收窄到 MAVLink int16 字段。
 */
static int16_t MavTx_SaturateInt16(int32_t value)
{
    if (value > INT16_MAX)
    {
        return INT16_MAX;
    }
    if (value < INT16_MIN)
    {
        return INT16_MIN;
    }
    return (int16_t)value;
}

/**
 * @brief 将无符号 Framework 数值饱和收窄到 MAVLink uint16 字段。
 */
static uint16_t MavTx_SaturateUint16(uint32_t value)
{
    return (value > UINT16_MAX)
               ? UINT16_MAX
               : (uint16_t)value;
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

    if (current_ma == 0)
    {
        return -1;
    }

    centi_amp = current_ma / 10;
    if (centi_amp > INT16_MAX)
    {
        return INT16_MAX;
    }
    if (centi_amp < INT16_MIN)
    {
        return INT16_MIN;
    }
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

    if (cdeg > (float)INT16_MAX)
    {
        return INT16_MAX;
    }
    if (cdeg < (float)INT16_MIN)
    {
        return INT16_MIN;
    }
    return (int16_t)((cdeg >= 0.0f) ? (cdeg + 0.5f) : (cdeg - 0.5f));
}

/**
 * @brief 将 degree*100 航向角归一化到 [0, 35999]。
 */
static uint16_t MavTx_NormalizeHeading(int32_t heading_deg100)
{
    int32_t normalized = heading_deg100 % 36000;

    if (normalized < 0)
    {
        normalized += 36000;
    }
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
    switch (fix_quality)
    {
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
static uint8_t MavTx_VisibleSatellites(
    const Px4Lite_SensorGnss_t *gnss)
{
    uint16_t total =
        (uint16_t)gnss->gps_visible +
        (uint16_t)gnss->bds_visible;

    return (total > UINT8_MAX)
               ? UINT8_MAX
               : (uint8_t)total;
}

/**
 * @brief 序列化当前 MAVLink 消息并提交给 LoRa 发送。
 */
static Px4Lite_Result_t MavTx_SendPrepared(void)
{
    uint16_t length;

    length = mavlink_msg_to_send_buffer(s_frame, &s_message);
    if ((length == 0U) ||
        (length > (uint16_t)sizeof(s_frame)))
    {
        return PX4LITE_IO_ERROR;
    }

    return Px4Lite_LoRaSend(s_frame, length);
}

/**
 * @brief 编码并发送 MAVLink HEARTBEAT。
 */
static Px4Lite_Result_t MavTx_SendHeartbeat(void)
{
    (void)mavlink_msg_heartbeat_pack_chan(
        PX4LITE_MAVLINK_SYSTEM_ID,
        PX4LITE_MAVLINK_COMPONENT_ID,
        MAVLINK_COMM_0,
        &s_message,
        MAV_TYPE_ONBOARD_CONTROLLER,
        MAV_AUTOPILOT_INVALID,
        0U,
        0U,
        MAV_STATE_ACTIVE);

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
    if (result != PX4LITE_OK)
    {
        return PX4LITE_NOT_READY;
    }
    /* 首次定位前抑制 GPS_RAW，避免持续发送 no-fix 位置帧。 */
    if (gnss.fix_type == 0U)
    {
        return PX4LITE_NOT_READY;
    }
    if (Px4Lite_IsFresh(&gnss.header,
                        now_ms,
                        PX4LITE_GNSS_MAX_AGE_MS) == 0U)
    {
        return PX4LITE_STALE;
    }
    if (gnss.header.sequence == s_stats.last_gps_sequence)
    {
        return PX4LITE_IDLE;
    }

    memset(&packet, 0, sizeof(packet));

    /* GNSS topic 不提供完整 UTC epoch，这里按 MAVLink 允许的系统启动后时间填充。 */
    packet.time_usec =
        (uint64_t)gnss.header.sample_time_ms * 1000ULL;

    mav_fix = MavTx_MapGpsFixType(gnss.fix_type);
    packet.fix_type = mav_fix;
    packet.eph =
        (gnss.hdop_x100 != 0U)
            ? gnss.hdop_x100
            : UINT16_MAX;
    packet.epv = UINT16_MAX;
    packet.satellites_visible =
        MavTx_VisibleSatellites(&gnss);

    if (mav_fix >= (uint8_t)GPS_FIX_TYPE_2D_FIX)
    {
        packet.lat = gnss.latitude_e7;
        packet.lon = gnss.longitude_e7;
        packet.alt = gnss.altitude_mm;
        packet.vel =
            MavTx_SaturateUint16(gnss.ground_speed_cms);
        packet.cog =
            MavTx_NormalizeHeading(gnss.heading_deg100);
    }
    else
    {
        packet.vel = UINT16_MAX;
        packet.cog = UINT16_MAX;
    }

    /* ATGM336H 当前未提供这些精度和双天线字段。 */
    packet.alt_ellipsoid = 0;
    packet.h_acc = UINT32_MAX;
    packet.v_acc = UINT32_MAX;
    packet.vel_acc = UINT32_MAX;
    packet.hdg_acc = UINT32_MAX;
    packet.yaw = 0U;

    (void)mavlink_msg_gps_raw_int_encode_chan(
        PX4LITE_MAVLINK_SYSTEM_ID,
        PX4LITE_MAVLINK_COMPONENT_ID,
        MAVLINK_COMM_0,
        &s_message,
        &packet);

    result = MavTx_SendPrepared();
    if (result == PX4LITE_OK)
    {
        s_stats.last_gps_sequence = gnss.header.sequence;
    }
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
    if (result != PX4LITE_OK)
    {
        return PX4LITE_NOT_READY;
    }
    /* 卫星细节跟随 GPS_RAW 策略：定位成功后才发送。 */
    if (gnss.fix_type == 0U)
    {
        return PX4LITE_NOT_READY;
    }
    if (Px4Lite_IsFresh(&gnss.header,
                        now_ms,
                        PX4LITE_GNSS_MAX_AGE_MS) == 0U)
    {
        return PX4LITE_STALE;
    }
    if (gnss.header.sequence ==
        s_stats.last_detail_sequence)
    {
        return PX4LITE_IDLE;
    }

    packed =
        (uint32_t)gnss.gps_visible |
        ((uint32_t)gnss.bds_visible << 8U) |
        ((uint32_t)gnss.gps_used << 16U) |
        ((uint32_t)gnss.bds_used << 24U);

    memset(&packet, 0, sizeof(packet));
    packet.time_boot_ms = now_ms;
    packet.value = (int32_t)packed;
    memcpy(packet.name, "GNSS_SAT", 8U);

    (void)mavlink_msg_named_value_int_encode_chan(
        PX4LITE_MAVLINK_SYSTEM_ID,
        PX4LITE_MAVLINK_COMPONENT_ID,
        MAVLINK_COMM_0,
        &s_message,
        &packet);

    result = MavTx_SendPrepared();
    if (result == PX4LITE_OK)
    {
        s_stats.last_detail_sequence =
            gnss.header.sequence;
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
    if (result != PX4LITE_OK)
    {
        return PX4LITE_NOT_READY;
    }
    if (Px4Lite_IsFresh(&navigation.header,
                        now_ms,
                        PX4LITE_IMU_MAX_AGE_MS) == 0U)
    {
        return PX4LITE_STALE;
    }
    if ((navigation.valid_mask &
         PX4LITE_NAV_VALID_ATTITUDE) == 0U)
    {
        return PX4LITE_NOT_READY;
    }
    if (navigation.header.sequence ==
        s_stats.last_attitude_sequence)
    {
        return PX4LITE_IDLE;
    }

    memset(&packet, 0, sizeof(packet));
    packet.time_boot_ms = now_ms;
    packet.roll = MavTx_Deg100ToRad(navigation.roll_deg100);
    packet.pitch = MavTx_Deg100ToRad(navigation.pitch_deg100);
    packet.yaw = MavTx_Deg100ToRad(navigation.yaw_deg100);
    packet.rollspeed =
        MavTx_Deg100ToRad(navigation.roll_rate_dps100);
    packet.pitchspeed =
        MavTx_Deg100ToRad(navigation.pitch_rate_dps100);
    packet.yawspeed =
        MavTx_Deg100ToRad(navigation.yaw_rate_dps100);

    (void)mavlink_msg_attitude_encode_chan(
        PX4LITE_MAVLINK_SYSTEM_ID,
        PX4LITE_MAVLINK_COMPONENT_ID,
        MAVLINK_COMM_0,
        &s_message,
        &packet);

    result = MavTx_SendPrepared();
    if (result == PX4LITE_OK)
    {
        s_stats.last_attitude_sequence =
            navigation.header.sequence;
    }
    return result;
}

/**
 * @brief 将导航快照编码为 MAVLink GLOBAL_POSITION_INT。
 */
static Px4Lite_Result_t MavTx_SendPosition(uint32_t now_ms)
{
    Px4Lite_VehicleNavigation_t navigation;
    mavlink_global_position_int_t packet;

    if (Px4Lite_CopyNavigation(&navigation) != PX4LITE_OK)
    {
        return PX4LITE_NOT_READY;
    }
    if (Px4Lite_IsFresh(&navigation.header,
                        now_ms,
                        PX4LITE_GNSS_MAX_AGE_MS) == 0U)
    {
        return PX4LITE_STALE;
    }
    if ((navigation.valid_mask &
         PX4LITE_NAV_VALID_POSITION) == 0U)
    {
        return PX4LITE_NOT_READY;
    }

    memset(&packet, 0, sizeof(packet));
    packet.time_boot_ms = now_ms;
    packet.lat = navigation.latitude_e7;
    packet.lon = navigation.longitude_e7;
    packet.alt = navigation.fused_altitude_mm;

    /* 后续 Home 模块应提供相对高度。 */
    packet.relative_alt = 0;

    if ((navigation.valid_mask &
         PX4LITE_NAV_VALID_VELOCITY) != 0U)
    {
        packet.vx =
            MavTx_SaturateInt16(
                navigation.velocity_north_cms);
        packet.vy =
            MavTx_SaturateInt16(
                navigation.velocity_east_cms);
        packet.vz =
            MavTx_SaturateInt16(
                navigation.velocity_down_cms);
    }

    packet.hdg =
        ((navigation.valid_mask &
          PX4LITE_NAV_VALID_ATTITUDE) != 0U)
            ? MavTx_NormalizeHeading(
                  navigation.yaw_deg100)
            : UINT16_MAX;

    (void)mavlink_msg_global_position_int_encode_chan(
        PX4LITE_MAVLINK_SYSTEM_ID,
        PX4LITE_MAVLINK_COMPONENT_ID,
        MAVLINK_COMM_0,
        &s_message,
        &packet);

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
    if (result != PX4LITE_OK)
    {
        return PX4LITE_NOT_READY;
    }
    if (Px4Lite_IsFresh(&battery.header,
                        now_ms,
                        PX4LITE_BATTERY_MAX_AGE_MS) == 0U)
    {
        return PX4LITE_STALE;
    }
    if (battery.header.sequence == s_stats.last_battery_sequence)
    {
        return PX4LITE_IDLE;
    }

    memset(&packet, 0, sizeof(packet));
    packet.id = 0U;
    packet.battery_function = (uint8_t)MAV_BATTERY_FUNCTION_ALL;
    packet.type = (uint8_t)MAV_BATTERY_TYPE_UNKNOWN;
    packet.temperature = INT16_MAX;
    for (i = 0U; i < 10U; ++i)
    {
        packet.voltages[i] = UINT16_MAX;
    }
    packet.voltages[0] = MavTx_SaturateUint16(battery.voltage_mv);
    packet.current_battery =
        MavTx_SaturateCentiAmp(battery.current_ma);
    packet.current_consumed = -1;
    packet.energy_consumed = -1;
    packet.battery_remaining = (int8_t)battery.percent;
    packet.time_remaining = 0;
    packet.charge_state =
        (battery.low_voltage != 0U)
            ? (uint8_t)MAV_BATTERY_CHARGE_STATE_LOW
            : (uint8_t)MAV_BATTERY_CHARGE_STATE_OK;

    (void)mavlink_msg_battery_status_encode_chan(
        PX4LITE_MAVLINK_SYSTEM_ID,
        PX4LITE_MAVLINK_COMPONENT_ID,
        MAVLINK_COMM_0,
        &s_message,
        &packet);

    result = MavTx_SendPrepared();
    if (result == PX4LITE_OK)
    {
        s_stats.last_battery_sequence = battery.header.sequence;
    }
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
    if (result != PX4LITE_OK)
    {
        return PX4LITE_NOT_READY;
    }
    if (Px4Lite_IsFresh(&baro.header,
                        now_ms,
                        PX4LITE_BARO_MAX_AGE_MS) == 0U)
    {
        return PX4LITE_STALE;
    }
    if (baro.header.sequence == s_stats.last_pressure_sequence)
    {
        return PX4LITE_IDLE;
    }

    memset(&packet, 0, sizeof(packet));
    packet.time_boot_ms = baro.header.sample_time_ms;
    packet.press_abs = baro.pressure_pa / 100.0f;
    packet.press_diff = 0.0f;
    packet.temperature =
        MavTx_SaturateCdegFromFloat(baro.temperature_c);
    packet.temperature_press_diff = 1;

    (void)mavlink_msg_scaled_pressure_encode_chan(
        PX4LITE_MAVLINK_SYSTEM_ID,
        PX4LITE_MAVLINK_COMPONENT_ID,
        MAVLINK_COMM_0,
        &s_message,
        &packet);

    result = MavTx_SendPrepared();
    if (result == PX4LITE_OK)
    {
        s_stats.last_pressure_sequence = baro.header.sequence;
    }
    return result;
}

/**
 * @brief 判断模块状态是否可映射为 MAVLink healthy。
 */
static uint8_t MavTx_ModuleHealthy(Px4Lite_State_t state)
{
    return ((state == PX4LITE_STATE_ONLINE) ||
            (state == PX4LITE_STATE_DEGRADED))
               ? 1U
               : 0U;
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
    switch (severity)
    {
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
    switch ((Px4Lite_ModuleId_t)module_id)
    {
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
    return (value < 10U)
               ? (char)('0' + value)
               : (char)('A' + (value - 10U));
}

/**
 * @brief 构造 MAVLink STATUSTEXT 文本字段。
 *
 * @param[out] text MAVLink 文本缓冲区，固定 50 字节。
 * @param[in] prefix 文本前缀。
 * @param[in] module 模块短名称。
 * @param[in] fault_code Framework 故障码。
 */
static void MavTx_CopyText(char text[50],
                           const char *prefix,
                           const char *module,
                           uint16_t fault_code)
{
    uint8_t pos = 0U;
    uint8_t i;

    memset(text, 0, 50U);
    for (i = 0U; (prefix[i] != '\0') && (pos < 49U); ++i)
    {
        text[pos++] = prefix[i];
    }
    if (pos < 49U)
    {
        text[pos++] = ' ';
    }
    for (i = 0U; (module[i] != '\0') && (pos < 49U); ++i)
    {
        text[pos++] = module[i];
    }
    if ((pos + 7U) < 50U)
    {
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

    if (Px4Lite_CopyAlarmSnapshot(&alarm) != PX4LITE_OK)
    {
        return PX4LITE_NOT_READY;
    }
    if (Px4Lite_IsFresh(&alarm.header,
                        now_ms,
                        PX4LITE_HEALTH_PERIOD_MS * 10U) == 0U)
    {
        return PX4LITE_STALE;
    }
    if (alarm.active_count == 0U)
    {
        return PX4LITE_NOT_READY;
    }
    if (alarm.header.sequence == s_stats.last_alarm_sequence)
    {
        return PX4LITE_IDLE;
    }

    memset(&packet, 0, sizeof(packet));
    packet.severity = MavTx_MapSeverity(alarm.highest_severity);
    packet.id = alarm.highest_fault_code;
    packet.chunk_seq = 0U;
    MavTx_CopyText(packet.text,
                   "PX4LITE ALARM",
                   MavTx_ModuleName(alarm.highest_source_id),
                   alarm.highest_fault_code);

    (void)mavlink_msg_statustext_encode_chan(
        PX4LITE_MAVLINK_SYSTEM_ID,
        PX4LITE_MAVLINK_COMPONENT_ID,
        MAVLINK_COMM_0,
        &s_message,
        &packet);

    result = MavTx_SendPrepared();
    if (result == PX4LITE_OK)
    {
        s_stats.last_alarm_sequence = alarm.header.sequence;
    }
    return result;
}

/**
 * @brief 将启用的 Framework 传感器健康状态编码为 SYS_STATUS。
 */
static Px4Lite_Result_t MavTx_SendSystemStatus(
    uint32_t now_ms)
{
    Px4Lite_SystemHealth_t health;
    mavlink_sys_status_t packet;
    Px4Lite_BatteryStatus_t battery;
    uint32_t present = 0U;
    uint32_t enabled = 0U;
    uint32_t healthy = 0U;

    if (Px4Lite_CopyHealth(&health) != PX4LITE_OK)
    {
        return PX4LITE_NOT_READY;
    }
    if (Px4Lite_IsFresh(&health.header,
                        now_ms,
                        PX4LITE_HEALTH_PERIOD_MS * 5U) == 0U)
    {
        return PX4LITE_STALE;
    }

#if PX4LITE_ENABLE_GNSS
    present |= MAV_SYS_STATUS_SENSOR_GPS;
    enabled |= MAV_SYS_STATUS_SENSOR_GPS;
    if (MavTx_ModuleHealthy(
            health.module_state[PX4LITE_MODULE_GNSS]) != 0U)
    {
        healthy |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#endif

#if PX4LITE_ENABLE_IMU
    present |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
    present |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    enabled |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
    enabled |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    if (MavTx_ModuleHealthy(
            health.module_state[PX4LITE_MODULE_IMU]) != 0U)
    {
        healthy |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
        healthy |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    }
#endif

#if PX4LITE_ENABLE_BARO
    present |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    enabled |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    if (MavTx_ModuleHealthy(
            health.module_state[PX4LITE_MODULE_BARO]) != 0U)
    {
        healthy |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }
#endif

    memset(&packet, 0, sizeof(packet));
    packet.onboard_control_sensors_present = present;
    packet.onboard_control_sensors_enabled = enabled;
    packet.onboard_control_sensors_health = healthy;
    packet.voltage_battery = UINT16_MAX;
    packet.current_battery = -1;
    packet.battery_remaining = -1;
    if ((Px4Lite_CopyBattery(&battery) == PX4LITE_OK) &&
        (Px4Lite_IsFresh(&battery.header,
                         now_ms,
                         PX4LITE_BATTERY_MAX_AGE_MS) != 0U))
    {
        packet.voltage_battery =
            MavTx_SaturateUint16(battery.voltage_mv);
        packet.current_battery =
            MavTx_SaturateCentiAmp(battery.current_ma);
        packet.battery_remaining = (int8_t)battery.percent;
    }

    (void)mavlink_msg_sys_status_encode_chan(
        PX4LITE_MAVLINK_SYSTEM_ID,
        PX4LITE_MAVLINK_COMPONENT_ID,
        MAVLINK_COMM_0,
        &s_message,
        &packet);

    return MavTx_SendPrepared();
}

/**
 * @brief 记录一个发送槽位结果并计算下一次发送截止时间。
 */
static void MavTx_RecordResult(Px4Lite_Result_t result,
                               uint32_t now_ms,
                               uint32_t period_ms,
                               uint32_t *next_ms,
                               uint32_t message_id,
                               uint32_t *success_count)
{
    if (result == PX4LITE_OK)
    {
        *next_ms = now_ms + period_ms;
        *success_count = *success_count + 1U;
        s_stats.last_message_id = message_id;
    }
    else
    {
        *next_ms =
            now_ms + PX4LITE_MAVLINK_RETRY_PERIOD_MS;

        if ((result == PX4LITE_NOT_READY) ||
            (result == PX4LITE_IDLE))
        {
            s_stats.no_data_count++;
        }
        else if (result == PX4LITE_STALE)
        {
            s_stats.stale_count++;
        }
        else if (result == PX4LITE_BUSY)
        {
            s_stats.busy_count++;
        }
        else
        {
            s_stats.error_count++;
        }
    }
}

Px4Lite_Result_t Px4Lite_MavlinkTxInit(uint32_t now_ms)
{
    memset(&s_message, 0, sizeof(s_message));
    memset(s_frame, 0, sizeof(s_frame));
    memset(&s_stats, 0, sizeof(s_stats));

    s_next_heartbeat_ms = now_ms;
    s_next_gps_raw_ms = now_ms + 100U;
    s_next_gnss_detail_ms = now_ms + 150U;
    s_next_attitude_ms = now_ms + 50U;
    s_next_position_ms = now_ms + 200U;
    s_next_sys_status_ms = now_ms + 300U;
    s_next_battery_ms = now_ms + 350U;
    s_next_pressure_ms = now_ms + 400U;
    s_next_statustext_ms = now_ms + 450U;
    s_slot = (uint8_t)MAV_TX_SLOT_HEARTBEAT;
    return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_MavlinkTxRun(uint32_t now_ms)
{
    Px4Lite_Result_t result;
    uint8_t checked;

    for (checked = 0U;
         checked < (uint8_t)MAV_TX_SLOT_COUNT;
         ++checked)
    {
        MavTx_Slot_t current = (MavTx_Slot_t)s_slot;

        s_slot =
            (uint8_t)((s_slot + 1U) %
                      (uint8_t)MAV_TX_SLOT_COUNT);

        switch (current)
        {
            case MAV_TX_SLOT_HEARTBEAT:
                if ((PX4LITE_MAVLINK_ENABLE_HEARTBEAT == 0U) ||
                    (MavTx_TimeReached(
                        now_ms,
                        s_next_heartbeat_ms) == 0U))
                {
                    continue;
                }
                result = MavTx_SendHeartbeat();
                MavTx_RecordResult(
                    result,
                    now_ms,
                    PX4LITE_MAVLINK_HEARTBEAT_PERIOD_MS,
                    &s_next_heartbeat_ms,
                    MAVLINK_MSG_ID_HEARTBEAT,
                    &s_stats.heartbeat_count);
                return result;

            case MAV_TX_SLOT_GPS_RAW:
                if ((PX4LITE_MAVLINK_ENABLE_GPS_RAW == 0U) ||
                    (MavTx_TimeReached(
                        now_ms,
                        s_next_gps_raw_ms) == 0U))
                {
                    continue;
                }
                result = MavTx_SendGpsRaw(now_ms);
                MavTx_RecordResult(
                    result,
                    now_ms,
                    PX4LITE_MAVLINK_GPS_RAW_PERIOD_MS,
                    &s_next_gps_raw_ms,
                    MAVLINK_MSG_ID_GPS_RAW_INT,
                    &s_stats.gps_raw_count);
                if ((result == PX4LITE_OK) ||
                    (result == PX4LITE_BUSY) ||
                    (result == PX4LITE_IO_ERROR))
                {
                    return result;
                }
                continue;

            case MAV_TX_SLOT_GNSS_DETAIL:
                if ((PX4LITE_MAVLINK_ENABLE_GNSS_DETAIL == 0U) ||
                    (MavTx_TimeReached(
                        now_ms,
                        s_next_gnss_detail_ms) == 0U))
                {
                    continue;
                }
                result = MavTx_SendGnssDetail(now_ms);
                MavTx_RecordResult(
                    result,
                    now_ms,
                    PX4LITE_MAVLINK_GNSS_DETAIL_PERIOD_MS,
                    &s_next_gnss_detail_ms,
                    MAVLINK_MSG_ID_NAMED_VALUE_INT,
                    &s_stats.gnss_detail_count);
                if ((result == PX4LITE_OK) ||
                    (result == PX4LITE_BUSY) ||
                    (result == PX4LITE_IO_ERROR))
                {
                    return result;
                }
                continue;

            case MAV_TX_SLOT_ATTITUDE:
                if ((PX4LITE_MAVLINK_ENABLE_ATTITUDE == 0U) ||
                    (MavTx_TimeReached(
                        now_ms,
                        s_next_attitude_ms) == 0U))
                {
                    continue;
                }
                result = MavTx_SendAttitude(now_ms);
                MavTx_RecordResult(
                    result,
                    now_ms,
                    PX4LITE_MAVLINK_ATTITUDE_PERIOD_MS,
                    &s_next_attitude_ms,
                    MAVLINK_MSG_ID_ATTITUDE,
                    &s_stats.attitude_count);
                if ((result == PX4LITE_OK) ||
                    (result == PX4LITE_BUSY) ||
                    (result == PX4LITE_IO_ERROR))
                {
                    return result;
                }
                continue;

            case MAV_TX_SLOT_POSITION:
                if ((PX4LITE_MAVLINK_ENABLE_GLOBAL_POSITION == 0U) ||
                    (MavTx_TimeReached(
                        now_ms,
                        s_next_position_ms) == 0U))
                {
                    continue;
                }
                result = MavTx_SendPosition(now_ms);
                MavTx_RecordResult(
                    result,
                    now_ms,
                    PX4LITE_MAVLINK_POSITION_PERIOD_MS,
                    &s_next_position_ms,
                    MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                    &s_stats.position_count);
                if ((result == PX4LITE_OK) ||
                    (result == PX4LITE_BUSY) ||
                    (result == PX4LITE_IO_ERROR))
                {
                    return result;
                }
                continue;

            case MAV_TX_SLOT_SYS_STATUS:
                if ((PX4LITE_MAVLINK_ENABLE_SYS_STATUS == 0U) ||
                    (MavTx_TimeReached(
                        now_ms,
                        s_next_sys_status_ms) == 0U))
                {
                    continue;
                }
                result = MavTx_SendSystemStatus(now_ms);
                MavTx_RecordResult(
                    result,
                    now_ms,
                    PX4LITE_MAVLINK_SYS_STATUS_PERIOD_MS,
                    &s_next_sys_status_ms,
                    MAVLINK_MSG_ID_SYS_STATUS,
                    &s_stats.sys_status_count);
                if ((result == PX4LITE_OK) ||
                    (result == PX4LITE_BUSY) ||
                    (result == PX4LITE_IO_ERROR))
                {
                    return result;
                }
                continue;

            case MAV_TX_SLOT_BATTERY_STATUS:
                if ((PX4LITE_MAVLINK_ENABLE_BATTERY_STATUS == 0U) ||
                    (MavTx_TimeReached(
                        now_ms,
                        s_next_battery_ms) == 0U))
                {
                    continue;
                }
                result = MavTx_SendBatteryStatus(now_ms);
                MavTx_RecordResult(
                    result,
                    now_ms,
                    PX4LITE_MAVLINK_BATTERY_PERIOD_MS,
                    &s_next_battery_ms,
                    MAVLINK_MSG_ID_BATTERY_STATUS,
                    &s_stats.battery_status_count);
                if ((result == PX4LITE_OK) ||
                    (result == PX4LITE_BUSY) ||
                    (result == PX4LITE_IO_ERROR))
                {
                    return result;
                }
                continue;

            case MAV_TX_SLOT_SCALED_PRESSURE:
                if ((PX4LITE_MAVLINK_ENABLE_SCALED_PRESSURE == 0U) ||
                    (MavTx_TimeReached(
                        now_ms,
                        s_next_pressure_ms) == 0U))
                {
                    continue;
                }
                result = MavTx_SendScaledPressure(now_ms);
                MavTx_RecordResult(
                    result,
                    now_ms,
                    PX4LITE_MAVLINK_PRESSURE_PERIOD_MS,
                    &s_next_pressure_ms,
                    MAVLINK_MSG_ID_SCALED_PRESSURE,
                    &s_stats.scaled_pressure_count);
                if ((result == PX4LITE_OK) ||
                    (result == PX4LITE_BUSY) ||
                    (result == PX4LITE_IO_ERROR))
                {
                    return result;
                }
                continue;

            case MAV_TX_SLOT_STATUSTEXT:
                if ((PX4LITE_MAVLINK_ENABLE_STATUSTEXT == 0U) ||
                    (MavTx_TimeReached(
                        now_ms,
                        s_next_statustext_ms) == 0U))
                {
                    continue;
                }
                result = MavTx_SendStatusText(now_ms);
                MavTx_RecordResult(
                    result,
                    now_ms,
                    PX4LITE_MAVLINK_STATUSTEXT_PERIOD_MS,
                    &s_next_statustext_ms,
                    MAVLINK_MSG_ID_STATUSTEXT,
                    &s_stats.statustext_count);
                if ((result == PX4LITE_OK) ||
                    (result == PX4LITE_BUSY) ||
                    (result == PX4LITE_IO_ERROR))
                {
                    return result;
                }
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
    if (out != 0)
    {
        *out = s_stats;
    }
}
