/**
 * @file px4lite_types.h
 * @brief Define framework measurement, domain, health, and command types.
 */

#ifndef PX4LITE_TYPES_H
#define PX4LITE_TYPES_H

#include <stdint.h>

typedef enum
{
    PX4LITE_OK = 0,
    PX4LITE_IDLE,
    PX4LITE_BUSY,
    PX4LITE_INVALID_PARAM,
    PX4LITE_NOT_READY,
    PX4LITE_STALE,
    PX4LITE_IO_ERROR,
    PX4LITE_OVERFLOW
} Px4Lite_Result_t;

typedef enum
{
    PX4LITE_STATE_UNINITIALIZED = 0,
    PX4LITE_STATE_STARTING,
    PX4LITE_STATE_ONLINE,
    PX4LITE_STATE_DEGRADED,
    PX4LITE_STATE_OFFLINE,
    PX4LITE_STATE_FAILED,
    PX4LITE_STATE_DISABLED
} Px4Lite_State_t;

typedef enum
{
    PX4LITE_MODULE_GNSS = 0,
    PX4LITE_MODULE_IMU,
    PX4LITE_MODULE_BARO,
    PX4LITE_MODULE_BATTERY,
    PX4LITE_MODULE_LORA,
    PX4LITE_MODULE_5G,
    PX4LITE_MODULE_STORAGE,
    PX4LITE_MODULE_REMOTE_ID,
    PX4LITE_MODULE_DISPLAY,
    PX4LITE_MODULE_CONTROL,
    PX4LITE_MODULE_ALARM,
    PX4LITE_MODULE_SYSTEM,
    PX4LITE_MODULE_ESTIMATOR,
    PX4LITE_MODULE_BUSINESS,
    PX4LITE_MODULE_COUNT
} Px4Lite_ModuleId_t;

typedef struct
{
    uint32_t sequence;
    uint32_t sample_time_ms;
    uint32_t publish_time_ms;
    uint32_t flags;
    uint16_t device_id;
    uint8_t valid;
    uint8_t quality;
} Px4Lite_TopicHeader_t;

#define PX4LITE_DATA_VALID       (1UL << 0)
#define PX4LITE_DATA_CALIBRATED  (1UL << 1)
#define PX4LITE_DATA_FILTERED    (1UL << 2)
#define PX4LITE_DATA_FUSED       (1UL << 3)
#define PX4LITE_DATA_DEGRADED    (1UL << 4)

typedef struct
{
    Px4Lite_ModuleId_t module_id;
    Px4Lite_State_t state;
    uint16_t fault_code;
    uint8_t severity;
    uint8_t recovery_count;
    uint32_t state_since_ms;
    uint32_t last_rx_ms;
    uint32_t last_valid_ms;
    uint32_t error_count;
    uint32_t drop_count;
    uint16_t consecutive_errors;
    uint16_t consecutive_valid;
} Px4Lite_ModuleStatus_t;

typedef struct
{
    Px4Lite_TopicHeader_t header;
    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t altitude_mm;
    uint32_t ground_speed_cms;
    uint32_t utc_sec;
    uint32_t utc_date;   /* Packed yymmdd from RMC; 0 when no valid date. */
    uint16_t heading_deg100;
    uint16_t hdop_x100;
    uint8_t fix_type;
    uint8_t fix_dimension;
    uint8_t satellites_used;
    uint8_t gps_visible;
    uint8_t bds_visible;
    uint8_t gps_used;
    uint8_t bds_used;
    uint8_t antenna_state;
    uint8_t reserved;
} Px4Lite_SensorGnss_t;

typedef struct
{
    Px4Lite_TopicHeader_t header;
    int32_t accel_mg[3];
    int32_t gyro_mdps[3];
    int16_t temperature_cdeg;
    uint16_t sample_period_us;
} Px4Lite_SensorImu_t;

typedef struct
{
    Px4Lite_TopicHeader_t header;
    float pressure_pa;
    float temperature_c;
    float relative_humidity_pct;
    int32_t pressure_altitude_mm;
    int32_t vertical_speed_cms;
} Px4Lite_SensorBaro_t;

typedef struct
{
    Px4Lite_TopicHeader_t header;
    uint32_t voltage_mv;
    int32_t current_ma;
    uint8_t percent;
    uint8_t low_voltage;
    uint16_t reserved;
} Px4Lite_BatteryStatus_t;

#define PX4LITE_NAV_VALID_ATTITUDE  (1UL << 0)
#define PX4LITE_NAV_VALID_POSITION  (1UL << 1)
#define PX4LITE_NAV_VALID_VELOCITY  (1UL << 2)
#define PX4LITE_NAV_VALID_ALTITUDE  (1UL << 3)
#define PX4LITE_NAV_VALID_YAW_REL   (1UL << 4)

typedef struct
{
    Px4Lite_TopicHeader_t header;
    uint32_t valid_mask;
    uint32_t gnss_utc_sec;
    uint32_t gnss_utc_date;   /* Packed yymmdd from RMC; 0 when no valid date. */
    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t fused_altitude_mm;
    int32_t vertical_speed_cms;
    int32_t roll_deg100;
    int32_t pitch_deg100;
    int32_t yaw_deg100;
    int32_t roll_rate_dps100;
    int32_t pitch_rate_dps100;
    int32_t yaw_rate_dps100;
    int32_t velocity_north_cms;
    int32_t velocity_east_cms;
    int32_t velocity_down_cms;
    uint16_t hdop_x100;
    uint8_t satellites_used;
    uint8_t gnss_fix_type;
    uint8_t navigation_quality;
    uint8_t reserved;
} Px4Lite_VehicleNavigation_t;

typedef struct
{
    Px4Lite_TopicHeader_t header;
    Px4Lite_State_t module_state[PX4LITE_MODULE_COUNT];
    uint32_t blocking_fault_mask;
    uint32_t warning_fault_mask;
    uint32_t not_ready_mask;
    uint16_t imu_fifo_peak;
    uint16_t alarm_queue_peak;
    uint16_t log_pool_free_min;
    uint16_t reserved;
    uint32_t status_version;
    uint32_t imu_fifo_overflow;
    uint32_t log_drop_count;
} Px4Lite_SystemHealth_t;

typedef enum
{
    PX4LITE_ALARM_INFO = 0,
    PX4LITE_ALARM_WARNING,
    PX4LITE_ALARM_ERROR,
    PX4LITE_ALARM_CRITICAL,
    PX4LITE_ALARM_FATAL
} Px4Lite_AlarmSeverity_t;

typedef struct
{
    uint32_t timestamp_ms;
    uint32_t sequence;
    uint16_t source_id;
    uint16_t fault_code;
    Px4Lite_AlarmSeverity_t severity;
    uint8_t active;
    uint8_t reserved[3];
    uint32_t detail;
} Px4Lite_AlarmEvent_t;

typedef struct
{
    uint16_t source_id;
    uint16_t fault_code;
    Px4Lite_AlarmSeverity_t severity;
    uint8_t active;
    uint8_t reserved[3];
    uint32_t raised_ms;
    uint32_t updated_ms;
    uint32_t detail;
} Px4Lite_AlarmRecord_t;

typedef struct
{
    Px4Lite_TopicHeader_t header;
    uint16_t active_count;
    uint16_t highest_fault_code;
    uint16_t highest_source_id;
    Px4Lite_AlarmSeverity_t highest_severity;
    uint8_t reserved[3];
    Px4Lite_AlarmRecord_t records[PX4LITE_MODULE_COUNT];
} Px4Lite_AlarmSnapshot_t;

typedef struct
{
    uint32_t timestamp_ms;
    uint32_t sequence;
    uint16_t command_id;
    uint16_t source_id;
    int32_t parameter[4];
} Px4Lite_Command_t;

typedef struct
{
    uint32_t timestamp_ms;
    uint32_t command_sequence;
    uint16_t command_id;
    uint16_t result;
} Px4Lite_CommandAck_t;

typedef struct
{
    uint32_t rx_frame_count;
    uint32_t tx_frame_count;
    uint32_t tx_busy_count;
    uint32_t mav_heartbeat_count;
    uint32_t mav_gps_raw_count;
    uint32_t mav_gnss_detail_count;
    uint32_t mav_attitude_count;
    uint32_t mav_position_count;
    uint32_t mav_sys_status_count;
    uint32_t mav_battery_status_count;
    uint32_t mav_scaled_pressure_count;
    uint32_t mav_statustext_count;
    uint32_t mav_no_data_count;
    uint32_t mav_stale_count;
    uint32_t mav_error_count;
    uint32_t mav_last_tx_msg_id;
    uint32_t crc_error_count;
    uint32_t send_error_count;
    uint32_t parse_error_count;
    uint32_t rx_byte_count;
    uint32_t rx_overflow_count;
    uint32_t rx_drop_count;
    uint32_t last_rx_ms;
    uint32_t last_tx_ms;
    uint32_t last_msg_id;
} Px4Lite_CommDebugInfo_t;

/**
 * @brief Return elapsed milliseconds, saturating small cross-task future samples to zero.
 */
static __inline uint32_t Px4Lite_ElapsedMs(
    uint32_t now_ms,
    uint32_t then_ms)
{
    int32_t delta = (int32_t)(now_ms - then_ms);

    return (delta >= 0) ? (uint32_t)delta : 0U;
}

/**
 * @brief Check whether a valid topic sample is within its allowed age.
 */
static __inline uint8_t Px4Lite_IsFresh(
    const Px4Lite_TopicHeader_t *header,
    uint32_t now_ms,
    uint32_t max_age_ms)
{
    if ((header == 0) || (header->valid == 0U))
    {
        return 0U;
    }

    return (Px4Lite_ElapsedMs(now_ms, header->sample_time_ms) <= max_age_ms)
               ? 1U
               : 0U;
}

#endif
