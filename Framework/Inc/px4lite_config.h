/**
 * @file px4lite_config.h
 * @brief Configure enabled modules, periods, timeouts, queues, and stacks.
 */

#ifndef PX4LITE_CONFIG_H
#define PX4LITE_CONFIG_H

#define PX4LITE_ENABLE_GNSS                 1U
#define PX4LITE_ENABLE_IMU                  1U
#define PX4LITE_ENABLE_BARO                 1U
#define PX4LITE_ENABLE_BATTERY              1U
#define PX4LITE_ENABLE_LORA                 1U
#define PX4LITE_ENABLE_5G                   0U
#define PX4LITE_ENABLE_STORAGE              1U
#define PX4LITE_ENABLE_REMOTE_ID            0U
#define PX4LITE_ENABLE_DISPLAY              1U
#define PX4LITE_ENABLE_CONTROL              0U
#define PX4LITE_ENABLE_ALARM                1U
#define PX4LITE_ENABLE_COMMAND              0U
#define PX4LITE_ENABLE_LOG_POOL             0U

#define PX4LITE_SENSOR_WORK_PERIOD_MS      10U
#define PX4LITE_ESTIMATOR_PERIOD_MS        20U
#define PX4LITE_HEALTH_PERIOD_MS          100U
#define PX4LITE_COMM_PERIOD_MS             10U
#define PX4LITE_TASK_HEARTBEAT_TIMEOUT_MS 500U
#define PX4LITE_IMU_WORK_PERIOD_MS         10U
#define PX4LITE_BARO_WORK_PERIOD_MS      1000U
#define PX4LITE_BATTERY_WORK_PERIOD_MS   1000U
#define PX4LITE_STORAGE_PERIOD_MS          50U

#define PX4LITE_PRIORITY_SENSOR       (tskIDLE_PRIORITY + 4U)
#define PX4LITE_PRIORITY_ESTIMATOR    (tskIDLE_PRIORITY + 3U)
#define PX4LITE_PRIORITY_HEALTH       (tskIDLE_PRIORITY + 2U)
#define PX4LITE_PRIORITY_COMM         (tskIDLE_PRIORITY + 2U)
/* Storage does blocking SD I/O; keep it strictly below biz_display (idle+1). */
#define PX4LITE_PRIORITY_STORAGE      (tskIDLE_PRIORITY)

/*
 * Keep disabled until BSP_WatchdogRefresh() is implemented.
 * Enabling this macro without that function intentionally causes a link error.
 */
#define PX4LITE_ENABLE_HARDWARE_WATCHDOG   0U

#define PX4LITE_GNSS_STARTUP_GRACE_MS     5000U
#define PX4LITE_GNSS_OFFLINE_MS           2000U
#define PX4LITE_GNSS_MAX_AGE_MS           1500U
#define PX4LITE_IMU_STARTUP_GRACE_MS      2000U
#define PX4LITE_IMU_OFFLINE_MS             500U
#define PX4LITE_IMU_MAX_AGE_MS             200U
#define PX4LITE_IMU_STABLE_VALID_MIN         3U
#define PX4LITE_BARO_STARTUP_GRACE_MS     5000U
#define PX4LITE_BARO_OFFLINE_MS           3000U
#define PX4LITE_BARO_MAX_AGE_MS           2500U
#define PX4LITE_BATTERY_STARTUP_GRACE_MS  5000U
#define PX4LITE_BATTERY_OFFLINE_MS        3000U
#define PX4LITE_BATTERY_MAX_AGE_MS        2500U
#define PX4LITE_DISPLAY_STARTUP_GRACE_MS  5000U
#define PX4LITE_DISPLAY_OFFLINE_MS        3000U

/*
 * Recovery is wired into HealthRun but remains disabled until the BSP
 * recovery path has completed hardware fault-injection testing.
 */
#define PX4LITE_GNSS_RECOVERY_ENABLE          0U
#define PX4LITE_GNSS_ERROR_THRESHOLD          3U
#define PX4LITE_GNSS_RECOVERY_DELAY_MS     1000U
#define PX4LITE_GNSS_RECOVERY_MAX_RETRY       5U

/*
 * Generic registry-driven module recovery (Health supervises; the owning
 * module's Service performs the re-init). Enable only after every recoverable
 * module's `recover` callback is the cheap "request re-init" form (no blocking
 * bus I/O in the Health task). Retries are rate-limited but unbounded so a
 * device unplugged for any duration recovers when it returns (hot-plug).
 */
#define PX4LITE_RECOVERY_ENABLE               1U
#define PX4LITE_RECOVERY_ERROR_THRESHOLD      5U
#define PX4LITE_RECOVERY_DELAY_MS          2000U

#define PX4LITE_IMU_FIFO_CAPACITY           32U
#define PX4LITE_ALARM_QUEUE_LENGTH          16U
#define PX4LITE_COMMAND_QUEUE_LENGTH         8U

#define PX4LITE_STACK_SENSOR               512U
#define PX4LITE_STACK_ESTIMATOR            512U
#define PX4LITE_STACK_HEALTH               768U
#define PX4LITE_STACK_COMM                 384U
#define PX4LITE_STACK_STORAGE              768U

#define PX4LITE_LORA_OFFLINE_MS          3000U
#define PX4LITE_LORA_STARTUP_GRACE_MS    3000U
#define PX4LITE_LORA_RECOVERY_ENABLE        0U
#define PX4LITE_MAVLINK_SYSTEM_ID            1U
#define PX4LITE_MAVLINK_COMPONENT_ID       191U

/*
 * MAVLink telemetry selection. CommTask is the only transmission owner.
 * Current product phase sends heartbeat and parsed GNSS only.
 */
#define PX4LITE_MAVLINK_ENABLE_HEARTBEAT       1U
#define PX4LITE_MAVLINK_ENABLE_GPS_RAW         1U
#define PX4LITE_MAVLINK_ENABLE_GNSS_DETAIL     1U
#define PX4LITE_MAVLINK_ENABLE_ATTITUDE        1U
#define PX4LITE_MAVLINK_ENABLE_GLOBAL_POSITION 1U
#define PX4LITE_MAVLINK_ENABLE_SYS_STATUS      1U
#define PX4LITE_MAVLINK_ENABLE_BATTERY_STATUS  1U
#define PX4LITE_MAVLINK_ENABLE_SCALED_PRESSURE 1U
#define PX4LITE_MAVLINK_ENABLE_STATUSTEXT      1U

#define PX4LITE_MAVLINK_HEARTBEAT_PERIOD_MS 1000U
#define PX4LITE_MAVLINK_GPS_RAW_PERIOD_MS    1000U
#define PX4LITE_MAVLINK_GNSS_DETAIL_PERIOD_MS 1000U
#define PX4LITE_MAVLINK_ATTITUDE_PERIOD_MS     50U
#define PX4LITE_MAVLINK_POSITION_PERIOD_MS    500U
#define PX4LITE_MAVLINK_SYS_STATUS_PERIOD_MS 1000U
#define PX4LITE_MAVLINK_BATTERY_PERIOD_MS    1000U
#define PX4LITE_MAVLINK_PRESSURE_PERIOD_MS   1000U
#define PX4LITE_MAVLINK_STATUSTEXT_PERIOD_MS 2000U
#define PX4LITE_MAVLINK_RETRY_PERIOD_MS        50U

#endif
