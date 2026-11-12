/**
 * @file debug_config.h
 * @brief Configure independent debug monitors and their reporting periods.
 */

#ifndef DEBUG_CONFIG_H
#define DEBUG_CONFIG_H

/*
 * Every switch is independent.
 *
 * Examples:
 * - Stack only: enable DEBUG_STACK_MONITOR_ENABLE.
 * - GNSS plus stack usage: enable both STACK and GNSS.
 * - GNSS only: enable DEBUG_GNSS_MONITOR_ENABLE.
 *
 * Future sensor and communication monitors shall follow the same pattern.
 */
#define DEBUG_BOOT_LOG_ENABLE         1U
#define DEBUG_BUSINESS_LOG_ENABLE     0U

/** @brief 本轮GNSS/IMU/Baro共红诊断临时开启；仅读既有资源副本，不控制采集任务。 */
#define DEBUG_STACK_MONITOR_ENABLE    1U
#define DEBUG_GNSS_MONITOR_ENABLE     1U
#define DEBUG_GNSS_BSP_MONITOR_ENABLE 0U

/* 独立诊断输出开关；不改变设备使能、采集和恢复策略。 */
#ifndef DEBUG_IMU_MONITOR_ENABLE
/** @brief 本轮三模块诊断输出IMU型号/接口/双ID与驱动阶段，仅读缓存。 */
#define DEBUG_IMU_MONITOR_ENABLE       1U
#endif
/** @brief 本轮三模块诊断输出Baro真实状态及sample有效性，仅读缓存，不访问I2C。 */
#define DEBUG_BARO_MONITOR_ENABLE      1U
#define DEBUG_POWER_MONITOR_ENABLE     1U /**< 主回路调试：每秒输出topic和同轮ADC/参考缓存，不触发采样。 */
#define DEBUG_LORA_MONITOR_ENABLE      0U /* 临时诊断LoRa收不到：打印rx_byte/rx_frame；定位后改回0 */
#define DEBUG_ALARM_MONITOR_ENABLE     0U
#define DEBUG_STORAGE_MONITOR_ENABLE   0U
#define DEBUG_HEALTH_MONITOR_ENABLE    0U
#define DEBUG_DISPLAY_MONITOR_ENABLE   0U

/*
 * Debug self-test switches.
 *
 * Keep the master and every item disabled in production. During board bring-up,
 * enable DEBUG_SELFTEST_ENABLE plus one item at a time, then observe the UART
 * report without changing the production data path.
 */
#ifndef DEBUG_SELFTEST_ENABLE
#define DEBUG_SELFTEST_ENABLE                0U
#endif
#ifndef DEBUG_SELFTEST_IMU_SNAPSHOT_ENABLE
#define DEBUG_SELFTEST_IMU_SNAPSHOT_ENABLE   0U
#endif
#define DEBUG_SELFTEST_BSP_INIT_ENABLE       0U
#define DEBUG_SELFTEST_DISPLAY_BUDGET_ENABLE 0U
#define DEBUG_SELFTEST_REMOTE_VIEW_ENABLE    0U
#define DEBUG_SELFTEST_GT911_ENABLE          0U
#define DEBUG_SELFTEST_GNSS_ENABLE           0U
#define DEBUG_SELFTEST_LORA_ENABLE           0U

#define DEBUG_STACK_REPORT_PERIOD_MS   5000U
#define DEBUG_GNSS_REPORT_PERIOD_MS    1000U
#define DEBUG_IMU_REPORT_PERIOD_MS     1000U
#define DEBUG_BARO_REPORT_PERIOD_MS    1000U
#define DEBUG_POWER_REPORT_PERIOD_MS   1000U
#define DEBUG_LORA_REPORT_PERIOD_MS    1000U
#define DEBUG_ALARM_REPORT_PERIOD_MS   1000U
#define DEBUG_SELFTEST_REPORT_PERIOD_MS 1000U
#define DEBUG_SERVICE_PERIOD_MS        100U
#define DEBUG_SERVICE_TASK_PRIORITY    (tskIDLE_PRIORITY + 1U)
#define DEBUG_SERVICE_TASK_STACK_WORDS 384U

/*
 * Internal derived switches. Developers do not edit these definitions.
 * They keep the console and DebugTask available only when a selected
 * diagnostic feature requires them.
 */
#define DEBUG_SELFTEST_ACTIVE_ENABLE   (DEBUG_SELFTEST_ENABLE && (DEBUG_SELFTEST_IMU_SNAPSHOT_ENABLE || DEBUG_SELFTEST_BSP_INIT_ENABLE || DEBUG_SELFTEST_DISPLAY_BUDGET_ENABLE || DEBUG_SELFTEST_REMOTE_VIEW_ENABLE || DEBUG_SELFTEST_GT911_ENABLE || DEBUG_SELFTEST_GNSS_ENABLE || DEBUG_SELFTEST_LORA_ENABLE))

#define DEBUG_PERIODIC_SERVICE_ENABLE (DEBUG_STACK_MONITOR_ENABLE || DEBUG_GNSS_MONITOR_ENABLE || DEBUG_IMU_MONITOR_ENABLE || DEBUG_BARO_MONITOR_ENABLE || DEBUG_POWER_MONITOR_ENABLE || DEBUG_LORA_MONITOR_ENABLE || DEBUG_ALARM_MONITOR_ENABLE || DEBUG_STORAGE_MONITOR_ENABLE || DEBUG_HEALTH_MONITOR_ENABLE || DEBUG_DISPLAY_MONITOR_ENABLE || DEBUG_SELFTEST_ACTIVE_ENABLE)

#define DEBUG_CONSOLE_ENABLE          (DEBUG_BOOT_LOG_ENABLE || DEBUG_BUSINESS_LOG_ENABLE || DEBUG_PERIODIC_SERVICE_ENABLE || DEBUG_GNSS_BSP_MONITOR_ENABLE)

#endif
