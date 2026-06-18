/**
 * @file app_data_api.h
 * @brief Declare the only supported read-only API for business consumers.
 */

#ifndef APP_DATA_API_H
#define APP_DATA_API_H

#include <stdint.h>
#include "px4lite_types.h"

#define APP_NAVIGATION_MAX_AGE_MS  1500U
#define APP_SYSTEM_MAX_AGE_MS       500U
#define APP_ENVIRONMENT_MAX_AGE_MS 2500U
#define APP_ALARM_MAX_AGE_MS        500U
#define APP_STATUS_COPY_RETRY_MAX     3U

typedef struct
{
    Px4Lite_TopicHeader_t header;
    uint32_t valid_mask;
    uint32_t gnss_utc_sec;
    uint32_t gnss_utc_date;   /* Packed yymmdd from RMC; 0 when no valid date. */
    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t altitude_mm;
    int32_t velocity_north_cms;
    int32_t velocity_east_cms;
    int32_t velocity_down_cms;
    int32_t roll_deg100;
    int32_t pitch_deg100;
    int32_t yaw_deg100;
    int32_t roll_rate_dps100;
    int32_t pitch_rate_dps100;
    int32_t yaw_rate_dps100;
    uint16_t hdop_x100;
    uint8_t satellites_used;
    uint8_t gnss_fix_type;
    uint8_t navigation_quality;
    uint8_t reserved;
} App_NavigationSnapshot_t;

typedef struct
{
    Px4Lite_TopicHeader_t header;
    Px4Lite_ModuleStatus_t modules[PX4LITE_MODULE_COUNT];
    uint32_t blocking_fault_mask;
    uint32_t warning_fault_mask;
    uint32_t not_ready_mask;
    uint32_t status_version;
    uint16_t active_alarm_count;
    uint16_t highest_fault_code;
    uint16_t highest_source_id;
    uint8_t highest_severity;
    uint8_t system_ready;
    uint8_t reserved[2];
} App_SystemSnapshot_t;

typedef struct
{
    uint16_t source_id;
    uint16_t fault_code;
    uint8_t severity;
    uint8_t active;
    uint16_t reserved;
    uint32_t raised_ms;
    uint32_t updated_ms;
    uint32_t detail;
} App_AlarmRecord_t;

typedef struct
{
    Px4Lite_TopicHeader_t header;
    uint16_t active_count;
    uint16_t highest_fault_code;
    uint16_t highest_source_id;
    uint8_t highest_severity;
    uint8_t reserved[3];
    App_AlarmRecord_t records[PX4LITE_MODULE_COUNT];
} App_AlarmSnapshot_t;

typedef struct
{
    Px4Lite_TopicHeader_t header;
    float pressure_pa;
    float temperature_c;
    float relative_humidity_pct;
    uint32_t voltage_mv;
    uint8_t battery_percent;
    uint8_t low_voltage;
    uint16_t reserved;
} App_EnvironmentSnapshot_t;

/**
 * @brief Copy a fresh and coherent navigation snapshot for application consumers.
 */
Px4Lite_Result_t App_CopyNavigation(
    App_NavigationSnapshot_t *out,
    uint32_t now_ms);
/**
 * @brief Copy a version-consistent system health and module status snapshot.
 */
Px4Lite_Result_t App_CopySystem(
    App_SystemSnapshot_t *out,
    uint32_t now_ms);
/**
 * @brief Copy the active alarm table for display, logging, and debug consumers.
 */
Px4Lite_Result_t App_CopyAlarm(
    App_AlarmSnapshot_t *out,
    uint32_t now_ms);
/**
 * @brief Copy a fresh environmental snapshot for application consumers.
 */
Px4Lite_Result_t App_CopyEnvironment(
    App_EnvironmentSnapshot_t *out,
    uint32_t now_ms);
/**
 * @brief Copy the latest status of one framework module.
 */
Px4Lite_Result_t App_GetModuleStatus(
    Px4Lite_ModuleId_t module_id,
    Px4Lite_ModuleStatus_t *out);
/**
 * @brief Copy the latest LoRa/MAVLink communication statistics.
 */
void App_GetCommStats(Px4Lite_CommDebugInfo_t *out);

#endif
