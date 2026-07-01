/**
 * @file display.c
 * @brief Coordinate display snapshots, touch navigation, and budgeted refresh.
 */

#include "display.h"
#include "app_data_api.h"
#include "app_message_log.h"
#include "display_lvgl.h"
#include "px4lite_platform.h"

/*
 * Display facade metadata.
 * var_addr is a firmware-side routing id for App, Display and diagnostics; it
 * is not an LCD controller register address.
 */

#define DISPLAY_ARRAY_SIZE(array) ((uint16_t)(sizeof(array) / sizeof((array)[0])))

#define DISPLAY_MOTOR_SLIDER_MAX_VALUE 100U
#define DISPLAY_PRESSURE_MAX_PA        110000.0f
typedef struct {
  uint32_t value; /* 当前缓存的原始显示值 */
  uint8_t valid;  /* 0 表示缓存无效，1 表示可用于 LVGL 页面 */
} Display_ValueCache_t;

/*
 * 页面 ID 用于 App 和调试链路路由，枚举值保持稳定。
 */
static const Display_HmiPageConfig_t s_hmi_pages[] = {{DISPLAY_HMI_PAGE_LOGO, 0x0003U, "logo", 0U}, {DISPLAY_HMI_PAGE_SELF_CHECK, 0x0000U, "self_check", 200U}, {DISPLAY_HMI_PAGE_FLIGHT, 0x0004U, "flight", 500U}, {DISPLAY_HMI_PAGE_AIRCRAFT, 0x0005U, "aircraft", 500U}, {DISPLAY_HMI_PAGE_DATA, 0x0001U, "data", 500U}, {DISPLAY_HMI_PAGE_MOTOR, 0x0006U, "motor", 500U}, {DISPLAY_HMI_PAGE_ALARM, 0x0002U, "alarm", 200U}, {DISPLAY_HMI_PAGE_HIDDEN, 0x0007U, "remote_link", 0U}};

/*
 * LVGL 800x480 显示变量配置。
 * var_addr 是固件内部变量地址，不是 LCD 控制器寄存器地址。
 */
static const Display_HmiVariableConfig_t s_hmi_variables[] = {
    /* 上电自检页 -- 9 个主要设备状态 */
    {DISPLAY_HMI_VAR_SELF_CHECK_GNSS, DISPLAY_HMI_PAGE_SELF_CHECK, 0x1009U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 70U, 140U, 16U, 16U, "self_check_gnss", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, DISPLAY_HMI_PAGE_SELF_CHECK, 0x1001U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 226U, 140U, 16U, 16U, "self_check_mpu", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_BME280, DISPLAY_HMI_PAGE_SELF_CHECK, 0x1002U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 382U, 140U, 16U, 16U, "self_check_bme", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_LORA, DISPLAY_HMI_PAGE_SELF_CHECK, 0x100AU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 70U, 252U, 16U, 16U, "self_check_lora", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_REMOTEID, DISPLAY_HMI_PAGE_SELF_CHECK, 0x100FU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 226U, 364U, 16U, 16U, "self_check_remoteid", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_SD, DISPLAY_HMI_PAGE_SELF_CHECK, 0x1003U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 226U, 252U, 16U, 16U, "self_check_sd", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR, DISPLAY_HMI_PAGE_SELF_CHECK, 0x1004U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 382U, 252U, 16U, 16U, "self_check_motor1", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2, DISPLAY_HMI_PAGE_SELF_CHECK, 0x100CU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 70U, 364U, 16U, 16U, "self_check_motor2", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3, DISPLAY_HMI_PAGE_SELF_CHECK, 0x100DU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 226U, 364U, 16U, 16U, "self_check_motor3", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4, DISPLAY_HMI_PAGE_SELF_CHECK, 0x100EU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 382U, 364U, 16U, 16U, "self_check_motor4", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_5GA, DISPLAY_HMI_PAGE_SELF_CHECK, 0x1000U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 0U, 0U, 0U, 0U, "self_check_5ga", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_POWER, DISPLAY_HMI_PAGE_SELF_CHECK, 0x1005U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 0U, 0U, 0U, 0U, "self_check_power", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_BUZZER, DISPLAY_HMI_PAGE_SELF_CHECK, 0x1006U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 0U, 0U, 0U, 0U, "self_check_buzzer", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_KEY, DISPLAY_HMI_PAGE_SELF_CHECK, 0x1007U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 0U, 0U, 0U, 0U, "self_check_key", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_DEBUG, DISPLAY_HMI_PAGE_SELF_CHECK, 0x1008U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 0U, 0U, 0U, 0U, "self_check_debug", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_ERROR_CODE, DISPLAY_HMI_PAGE_SELF_CHECK, 0x100BU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 480U, 82U, 312U, 310U, "self_check_errcode", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_LORA_STATUS, DISPLAY_HMI_PAGE_SELF_CHECK, 0x1400U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 500U, 616U, 16U, 24U, 16U, "self_header_lora_status", "-", "CNS_State.lora"},
    {DISPLAY_HMI_VAR_SELF_CHECK_GNSS, DISPLAY_HMI_PAGE_DATA, 0x1009U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 122U, 24U, 16U, "data_status_gnss", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, DISPLAY_HMI_PAGE_DATA, 0x1001U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 152U, 24U, 16U, "data_status_mpu", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_BME280, DISPLAY_HMI_PAGE_DATA, 0x1002U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 182U, 24U, 16U, "data_status_bme", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_LORA, DISPLAY_HMI_PAGE_DATA, 0x100AU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 212U, 24U, 16U, "data_status_lora", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_SD, DISPLAY_HMI_PAGE_DATA, 0x1003U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 242U, 24U, 16U, "data_status_sd", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR, DISPLAY_HMI_PAGE_DATA, 0x1004U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 272U, 24U, 16U, "data_status_motor1", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2, DISPLAY_HMI_PAGE_DATA, 0x100CU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 302U, 24U, 16U, "data_status_motor2", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3, DISPLAY_HMI_PAGE_DATA, 0x100DU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 332U, 24U, 16U, "data_status_motor3", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4, DISPLAY_HMI_PAGE_DATA, 0x100EU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 362U, 24U, 16U, "data_status_motor4", "-", "App_Registry"},

    /* 数据页 -- scale2 标签、数值和单位 */
    {DISPLAY_HMI_VAR_UPTIME_MS, DISPLAY_HMI_PAGE_DATA, 0x1101U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "uptime_ms", "ms", "CNS_State.system"},
    {DISPLAY_HMI_VAR_DATA_SELFCHECK_RESULT, DISPLAY_HMI_PAGE_DATA, 0x1100U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "selfcheck_result", "-", "CNS_State.system"},
    {DISPLAY_HMI_VAR_BATTERY_VOLTAGE, DISPLAY_HMI_PAGE_DATA, 0x1103U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "battery_voltage", "V", "CNS_State.power"},
    {DISPLAY_HMI_VAR_BATTERY_PERCENT, DISPLAY_HMI_PAGE_DATA, 0x1104U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "battery_percent", "%", "CNS_State.power"},
    {DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE, DISPLAY_HMI_PAGE_DATA, 0x110BU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "motor_bat_voltage", "V", "App_Display"},
    {DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT, DISPLAY_HMI_PAGE_DATA, 0x110CU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "motor_bat_percent", "%", "App_Display"},
    {DISPLAY_HMI_VAR_GNSS_FIX, DISPLAY_HMI_PAGE_DATA, 0x1200U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 390U, 120U, 96U, 20U, "gnss_fix", "-", "CNS_State.gnss"},
    {DISPLAY_HMI_VAR_GNSS_SAT_COUNT, DISPLAY_HMI_PAGE_DATA, 0x1209U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 390U, 162U, 96U, 22U, "gnss_sat_count", "-", "CNS_State.gnss"},
    {DISPLAY_HMI_VAR_GNSS_HDOP, DISPLAY_HMI_PAGE_DATA, 0x120AU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 390U, 330U, 96U, 22U, "gnss_hdop", "-", "CNS_State.gnss"},
    {DISPLAY_HMI_VAR_LATITUDE, DISPLAY_HMI_PAGE_DATA, 0x1201U, DISPLAY_HMI_TYPE_I32, DISPLAY_HMI_ACCESS_RO, 1000U, 390U, 204U, 118U, 22U, "latitude", "\xB0", "CNS_State.gnss"},
    {DISPLAY_HMI_VAR_LONGITUDE, DISPLAY_HMI_PAGE_DATA, 0x1203U, DISPLAY_HMI_TYPE_I32, DISPLAY_HMI_ACCESS_RO, 1000U, 390U, 246U, 118U, 22U, "longitude", "\xB0", "CNS_State.gnss"},
    {DISPLAY_HMI_VAR_ALTITUDE, DISPLAY_HMI_PAGE_DATA, 0x1205U, DISPLAY_HMI_TYPE_I32, DISPLAY_HMI_ACCESS_RO, 1000U, 390U, 288U, 118U, 22U, "altitude", "m", "CNS_State.gnss"},
    {DISPLAY_HMI_VAR_GNSS_SPEED, DISPLAY_HMI_PAGE_DATA, 0x1207U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "gnss_speed", "m/s", "CNS_State.gnss"},
    {DISPLAY_HMI_VAR_GNSS_TIME, DISPLAY_HMI_PAGE_DATA, 0x1208U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "gnss_time", "UTC+8", "HMI/Clock"},
    {DISPLAY_HMI_VAR_ROLL, DISPLAY_HMI_PAGE_DATA, 0x1300U, DISPLAY_HMI_TYPE_I16, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "roll", "\xB0", "CNS_State.sensor"},
    {DISPLAY_HMI_VAR_PITCH, DISPLAY_HMI_PAGE_DATA, 0x1301U, DISPLAY_HMI_TYPE_I16, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "pitch", "\xB0", "CNS_State.sensor"},
    {DISPLAY_HMI_VAR_YAW, DISPLAY_HMI_PAGE_DATA, 0x1305U, DISPLAY_HMI_TYPE_I16, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "yaw", "\xB0", "CNS_State.navigation"},
    {DISPLAY_HMI_VAR_TEMPERATURE, DISPLAY_HMI_PAGE_DATA, 0x1302U, DISPLAY_HMI_TYPE_I16, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "temperature",
     "\xB0"
     "C",
     "CNS_State.sensor"},
    {DISPLAY_HMI_VAR_HUMIDITY, DISPLAY_HMI_PAGE_DATA, 0x1303U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "humidity", "%", "CNS_State.sensor"},
    {DISPLAY_HMI_VAR_PRESSURE, DISPLAY_HMI_PAGE_DATA, 0x1304U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "pressure", "Pa", "CNS_State.sensor"},
    {DISPLAY_HMI_VAR_LORA_STATUS, DISPLAY_HMI_PAGE_DATA, 0x1400U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 500U, 616U, 16U, 24U, 16U, "data_header_lora_status", "-", "CNS_State.lora"},
    {DISPLAY_HMI_VAR_LORA_TX_COUNT, DISPLAY_HMI_PAGE_DATA, 0x1401U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "lora_tx_count", "-", "CNS_State.lora"},
    {DISPLAY_HMI_VAR_LORA_RX_COUNT, DISPLAY_HMI_PAGE_DATA, 0x1403U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "lora_rx_count", "-", "CNS_State.lora"},
    {DISPLAY_HMI_VAR_LORA_HEARTBEAT, DISPLAY_HMI_PAGE_DATA, 0x1407U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 500U, 0U, 0U, 0U, 0U, "lora_heartbeat", "-", "CNS_State.lora"},
    {DISPLAY_HMI_VAR_LORA_ACK_COUNT, DISPLAY_HMI_PAGE_DATA, 0x1408U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "lora_ack_count", "-", "CNS_State.lora"},
    {DISPLAY_HMI_VAR_LORA_LOSS_RATE, DISPLAY_HMI_PAGE_DATA, 0x1405U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "lora_loss_rate", "%", "CNS_State.lora"},
    {DISPLAY_HMI_VAR_ALARM_CODE, DISPLAY_HMI_PAGE_DATA, 0x1406U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 0U, 0U, 0U, 0U, "alarm_code", "-", "CNS_State.alarm"},
    {DISPLAY_HMI_VAR_SYSTEM_STATUS, DISPLAY_HMI_PAGE_DATA, 0x1102U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 500U, 0U, 0U, 0U, 0U, "system_status", "-", "CNS_State.system"},
    {DISPLAY_HMI_VAR_MOTOR_PWM_1, DISPLAY_HMI_PAGE_DATA, 0x1600U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RW, 0U, 0U, 0U, 0U, 0U, "motor_pwm_1", "%", "HMI/Motor"},
    {DISPLAY_HMI_VAR_MOTOR_PWM_2, DISPLAY_HMI_PAGE_DATA, 0x1601U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RW, 0U, 0U, 0U, 0U, 0U, "motor_pwm_2", "%", "HMI/Motor"},
    {DISPLAY_HMI_VAR_MOTOR_PWM_3, DISPLAY_HMI_PAGE_DATA, 0x1602U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RW, 0U, 0U, 0U, 0U, 0U, "motor_pwm_3", "%", "HMI/Motor"},
    {DISPLAY_HMI_VAR_MOTOR_PWM_4, DISPLAY_HMI_PAGE_DATA, 0x1603U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RW, 0U, 0U, 0U, 0U, 0U, "motor_pwm_4", "%", "HMI/Motor"},

    /* 飞行数据页 -- 左侧系统栏 9 个模块状态灯 */
    {DISPLAY_HMI_VAR_SELF_CHECK_GNSS, DISPLAY_HMI_PAGE_FLIGHT, 0x1009U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 122U, 24U, 16U, "flight_status_gnss", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, DISPLAY_HMI_PAGE_FLIGHT, 0x1001U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 152U, 24U, 16U, "flight_status_mpu", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_BME280, DISPLAY_HMI_PAGE_FLIGHT, 0x1002U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 182U, 24U, 16U, "flight_status_bme", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_LORA, DISPLAY_HMI_PAGE_FLIGHT, 0x100AU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 212U, 24U, 16U, "flight_status_lora", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_SD, DISPLAY_HMI_PAGE_FLIGHT, 0x1003U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 242U, 24U, 16U, "flight_status_sd", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR, DISPLAY_HMI_PAGE_FLIGHT, 0x1004U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 272U, 24U, 16U, "flight_status_motor1", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2, DISPLAY_HMI_PAGE_FLIGHT, 0x100CU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 302U, 24U, 16U, "flight_status_motor2", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3, DISPLAY_HMI_PAGE_FLIGHT, 0x100DU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 332U, 24U, 16U, "flight_status_motor3", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4, DISPLAY_HMI_PAGE_FLIGHT, 0x100EU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 362U, 24U, 16U, "flight_status_motor4", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_LORA_STATUS, DISPLAY_HMI_PAGE_FLIGHT, 0x1400U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 500U, 616U, 16U, 24U, 16U, "flight_header_lora_status", "-", "CNS_State.lora"},

    /* 飞行数据页 -- 中间栏：时间/飞行时间/电压/电量/温度/湿度/气压 */
    {DISPLAY_HMI_VAR_DATE, DISPLAY_HMI_PAGE_FLIGHT, 0x1107U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 116U, 124U, 22U, "clock_date", "-", "HMI/Clock"},
    {DISPLAY_HMI_VAR_CLOCK_TIME, DISPLAY_HMI_PAGE_FLIGHT, 0x1105U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 152U, 124U, 22U, "clock_time", "-", "HMI/Clock"},
    {DISPLAY_HMI_VAR_MESSAGE_LOG, DISPLAY_HMI_PAGE_DATA, 0x110AU, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 540U, 76U, 252U, 330U, "msg_log_data", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_MESSAGE_LOG, DISPLAY_HMI_PAGE_FLIGHT, 0x110AU, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 540U, 76U, 252U, 330U, "msg_log_flight", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_MESSAGE_LOG, DISPLAY_HMI_PAGE_AIRCRAFT, 0x110AU, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 540U, 76U, 252U, 330U, "msg_log_aircraft", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_VIEW_NODE_ID, DISPLAY_HMI_PAGE_FLIGHT, 0x110DU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 500U, 0U, 0U, 0U, 0U, "view_node_id", "-", "App_Display"},
    {DISPLAY_HMI_VAR_FLIGHT_TIME_S, DISPLAY_HMI_PAGE_FLIGHT, 0x1106U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 188U, 124U, 22U, "flight_time_s", "-", "CNS_State.system"},
    {DISPLAY_HMI_VAR_BATTERY_VOLTAGE, DISPLAY_HMI_PAGE_FLIGHT, 0x1103U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 224U, 124U, 22U, "battery_voltage", "V", "CNS_State.power"},
    {DISPLAY_HMI_VAR_BATTERY_PERCENT, DISPLAY_HMI_PAGE_FLIGHT, 0x1104U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 260U, 124U, 22U, "battery_percent", "%", "CNS_State.power"},
    {DISPLAY_HMI_VAR_TEMPERATURE, DISPLAY_HMI_PAGE_FLIGHT, 0x1302U, DISPLAY_HMI_TYPE_I16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 296U, 124U, 22U, "temperature",
     "\xB0"
     "C",
     "CNS_State.sensor"},
    {DISPLAY_HMI_VAR_HUMIDITY, DISPLAY_HMI_PAGE_FLIGHT, 0x1303U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 332U, 124U, 22U, "humidity", "%", "CNS_State.sensor"},
    {DISPLAY_HMI_VAR_PRESSURE, DISPLAY_HMI_PAGE_FLIGHT, 0x1304U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 368U, 124U, 22U, "pressure", "Pa", "CNS_State.sensor"},

    /* 飞机情况页 -- 左侧系统栏 9 个模块状态灯 */
    {DISPLAY_HMI_VAR_SELF_CHECK_GNSS, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1009U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 122U, 24U, 16U, "ac_status_gnss", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1001U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 152U, 24U, 16U, "ac_status_mpu", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_BME280, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1002U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 182U, 24U, 16U, "ac_status_bme", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_LORA, DISPLAY_HMI_PAGE_AIRCRAFT, 0x100AU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 212U, 24U, 16U, "ac_status_lora", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_SD, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1003U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 242U, 24U, 16U, "ac_status_sd", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1004U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 272U, 24U, 16U, "ac_status_motor1", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2, DISPLAY_HMI_PAGE_AIRCRAFT, 0x100CU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 302U, 24U, 16U, "ac_status_motor2", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3, DISPLAY_HMI_PAGE_AIRCRAFT, 0x100DU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 332U, 24U, 16U, "ac_status_motor3", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4, DISPLAY_HMI_PAGE_AIRCRAFT, 0x100EU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 362U, 24U, 16U, "ac_status_motor4", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_LORA_STATUS, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1400U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 500U, 616U, 16U, 24U, 16U, "ac_header_lora_status", "-", "CNS_State.lora"},

    /* 飞机情况页 -- 中间栏：横滚、俯仰、偏航、LoRa 发送/接收 */
    {DISPLAY_HMI_VAR_ROLL, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1300U, DISPLAY_HMI_TYPE_I16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 140U, 124U, 22U, "ac_roll", "\xB0", "CNS_State.sensor"},
    {DISPLAY_HMI_VAR_PITCH, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1301U, DISPLAY_HMI_TYPE_I16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 190U, 124U, 22U, "ac_pitch", "\xB0", "CNS_State.sensor"},
    {DISPLAY_HMI_VAR_YAW, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1305U, DISPLAY_HMI_TYPE_I16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 240U, 124U, 22U, "ac_yaw", "\xB0", "CNS_State.navigation"},
    {DISPLAY_HMI_VAR_LORA_TX_COUNT, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1401U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 290U, 124U, 22U, "ac_lora_tx", "-", "CNS_State.lora"},
    {DISPLAY_HMI_VAR_LORA_RX_COUNT, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1403U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 340U, 124U, 22U, "ac_lora_rx", "-", "CNS_State.lora"},

    /* 电机控制页 -- 左侧状态栏、四路竖向油门滑条和右侧消息日志 */
    {DISPLAY_HMI_VAR_SELF_CHECK_GNSS, DISPLAY_HMI_PAGE_MOTOR, 0x1009U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 122U, 24U, 16U, "mc_status_gnss", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, DISPLAY_HMI_PAGE_MOTOR, 0x1001U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 152U, 24U, 16U, "mc_status_mpu", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_BME280, DISPLAY_HMI_PAGE_MOTOR, 0x1002U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 182U, 24U, 16U, "mc_status_bme", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_LORA, DISPLAY_HMI_PAGE_MOTOR, 0x100AU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 212U, 24U, 16U, "mc_status_lora", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_SD, DISPLAY_HMI_PAGE_MOTOR, 0x1003U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 242U, 24U, 16U, "mc_status_sd", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR, DISPLAY_HMI_PAGE_MOTOR, 0x1004U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 272U, 24U, 16U, "mc_status_motor1", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2, DISPLAY_HMI_PAGE_MOTOR, 0x100CU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 302U, 24U, 16U, "mc_status_motor2", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3, DISPLAY_HMI_PAGE_MOTOR, 0x100DU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 332U, 24U, 16U, "mc_status_motor3", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4, DISPLAY_HMI_PAGE_MOTOR, 0x100EU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 362U, 24U, 16U, "mc_status_motor4", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_LORA_STATUS, DISPLAY_HMI_PAGE_MOTOR, 0x1400U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 500U, 616U, 16U, 24U, 16U, "motor_header_lora_status", "-", "CNS_State.lora"},
    {DISPLAY_HMI_VAR_MOTOR_PWM_1, DISPLAY_HMI_PAGE_MOTOR, 0x1600U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RW, 0U, DISPLAY_MOTOR_TRACK1_X, DISPLAY_MOTOR_TRACK_TOP_Y, DISPLAY_MOTOR_TRACK_W, DISPLAY_MOTOR_TRACK_H, "motor_slider_1", "%", "HMI/Motor"},
    {DISPLAY_HMI_VAR_MOTOR_PWM_2, DISPLAY_HMI_PAGE_MOTOR, 0x1601U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RW, 0U, DISPLAY_MOTOR_TRACK2_X, DISPLAY_MOTOR_TRACK_TOP_Y, DISPLAY_MOTOR_TRACK_W, DISPLAY_MOTOR_TRACK_H, "motor_slider_2", "%", "HMI/Motor"},
    {DISPLAY_HMI_VAR_MOTOR_PWM_3, DISPLAY_HMI_PAGE_MOTOR, 0x1602U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RW, 0U, DISPLAY_MOTOR_TRACK3_X, DISPLAY_MOTOR_TRACK_TOP_Y, DISPLAY_MOTOR_TRACK_W, DISPLAY_MOTOR_TRACK_H, "motor_slider_3", "%", "HMI/Motor"},
    {DISPLAY_HMI_VAR_MOTOR_PWM_4, DISPLAY_HMI_PAGE_MOTOR, 0x1603U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RW, 0U, DISPLAY_MOTOR_TRACK4_X, DISPLAY_MOTOR_TRACK_TOP_Y, DISPLAY_MOTOR_TRACK_W, DISPLAY_MOTOR_TRACK_H, "motor_slider_4", "%", "HMI/Motor"},
    {DISPLAY_HMI_VAR_MESSAGE_LOG, DISPLAY_HMI_PAGE_MOTOR, 0x110AU, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 616U, 76U, 176U, 330U, "msg_log_motor", "-", "App_Registry"},

    /* 告警页 -- 运行期告警项 */
    {DISPLAY_HMI_VAR_LORA_STATUS, DISPLAY_HMI_PAGE_ALARM, 0x1400U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 500U, 616U, 16U, 24U, 16U, "alarm_header_lora_status", "-", "CNS_State.lora"},
    {DISPLAY_HMI_VAR_ALARM_ACTIVE_MASK, DISPLAY_HMI_PAGE_ALARM, 0x1700U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 52U, 72U, 696U, 38U, "alarm_active_mask", "-", "App_Alarm"},
    {DISPLAY_HMI_VAR_ALARM_ROW1_CODE, DISPLAY_HMI_PAGE_ALARM, 0x1701U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 42U, 150U, 716U, 48U, "alarm_row1_code", "-", "App_Alarm"},
    {DISPLAY_HMI_VAR_ALARM_ROW2_CODE, DISPLAY_HMI_PAGE_ALARM, 0x1702U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 42U, 200U, 716U, 48U, "alarm_row2_code", "-", "App_Alarm"},
    {DISPLAY_HMI_VAR_ALARM_ROW3_CODE, DISPLAY_HMI_PAGE_ALARM, 0x1703U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 42U, 250U, 716U, 48U, "alarm_row3_code", "-", "App_Alarm"},
    {DISPLAY_HMI_VAR_ALARM_ROW4_CODE, DISPLAY_HMI_PAGE_ALARM, 0x1704U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 42U, 300U, 716U, 48U, "alarm_row4_code", "-", "App_Alarm"},
    {DISPLAY_HMI_VAR_ALARM_ROW5_CODE, DISPLAY_HMI_PAGE_ALARM, 0x1705U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 42U, 350U, 716U, 48U, "alarm_row5_code", "-", "App_Alarm"}};

static Display_ValueCache_t s_hmi_values[DISPLAY_HMI_VAR_COUNT];
static App_DisplaySnapshot_t s_display_snapshot;
static Display_HmiPage_t s_current_page       = DISPLAY_HMI_PAGE_LOGO;
static Display_DataSource_t s_data_source     = DISPLAY_DATA_SOURCE_LOCAL;
static uint8_t s_display_ready                = 0U;
static uint8_t s_display_initialized          = 0U;
static volatile uint8_t s_recover_requested   = 0U;

/*
 * 根据变量 ID 查找 HMI 变量配置项。
 */
static const Display_HmiVariableConfig_t *Display_GetVariableConfig(Display_HmiVariableId_t id)
{
  uint16_t i;

  for (i = 0U; i < DISPLAY_ARRAY_SIZE(s_hmi_variables); i++) {
    if (s_hmi_variables[i].id == id) { return &s_hmi_variables[i]; }
  }

  return 0;
}

static uint8_t Display_MotorIndexFromId(Display_HmiVariableId_t id, uint8_t *index)
{
  if (index == 0) { return 0U; }

  switch (id) {
    case DISPLAY_HMI_VAR_MOTOR_PWM_1:
      *index = 0U;
      return 1U;
    case DISPLAY_HMI_VAR_MOTOR_PWM_2:
      *index = 1U;
      return 1U;
    case DISPLAY_HMI_VAR_MOTOR_PWM_3:
      *index = 2U;
      return 1U;
    case DISPLAY_HMI_VAR_MOTOR_PWM_4:
      *index = 3U;
      return 1U;
    default:
      break;
  }

  return 0U;
}

static Display_HmiVariableId_t Display_MotorIdFromIndex(uint8_t motor_index)
{
  switch (motor_index) {
    case 0U:
      return DISPLAY_HMI_VAR_MOTOR_PWM_1;
    case 1U:
      return DISPLAY_HMI_VAR_MOTOR_PWM_2;
    case 2U:
      return DISPLAY_HMI_VAR_MOTOR_PWM_3;
    case 3U:
      return DISPLAY_HMI_VAR_MOTOR_PWM_4;
    default:
      break;
  }

  return DISPLAY_HMI_VAR_COUNT;
}

static Display_Result_t Display_SetMotorThrottleCommand(Display_HmiVariableId_t id, uint16_t throttle_percent)
{
  uint8_t motor_index;

  if (Display_MotorIndexFromId(id, &motor_index) == 0U) { return DISPLAY_ERROR; }
  if (throttle_percent > DISPLAY_MOTOR_SLIDER_MAX_VALUE) { throttle_percent = DISPLAY_MOTOR_SLIDER_MAX_VALUE; }

  if (App_CommandMotorThrottlePercent(motor_index, (uint8_t)throttle_percent) == 0U) { return DISPLAY_ERROR; }

  return Display_SetHmiValueU16(id, throttle_percent);
}

static Display_Result_t Display_MotorEmergencyStop(void)
{
  uint8_t i;

  for (i = 0U; i < 4U; i++) {
    Display_HmiVariableId_t id = Display_MotorIdFromIndex(i);

    (void)App_SetMotorThrottlePercent(i, 0U);
    if (id != DISPLAY_HMI_VAR_COUNT) {
      (void)Display_SetHmiValueU16(id, 0U);
    }
  }

  return DISPLAY_OK;
}

Display_Result_t Display_RequestMotorThrottle(Display_HmiVariableId_t id, uint16_t throttle_percent)
{
  uint8_t motor_index;

  if (s_data_source == DISPLAY_DATA_SOURCE_REMOTE) { return DISPLAY_NOT_READY; }
  if (Display_MotorIndexFromId(id, &motor_index) == 0U) { return DISPLAY_ERROR; }

  return Display_SetMotorThrottleCommand(id, throttle_percent);
}

Display_Result_t Display_RequestMotorEmergencyStop(void)
{
  return Display_MotorEmergencyStop();
}

/**
 * @brief 复制 Display/LVGL 刷新预算诊断统计。
 *
 * @param[out] out 输出缓冲区，允许为 NULL。
 */
void Display_GetDebugStats(Display_DebugStats_t *out)
{
  Display_LvglGetDebugStats(out);
}

static void Display_InitSelfCheckValues(void)
{
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_GNSS, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_BME280, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_LORA, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_REMOTEID, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_5GA, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_SD, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4, 0U);
}

static Display_Result_t Display_EnsureInit(void)
{
  uint32_t now_ms;

  if (s_recover_requested != 0U) {
    s_recover_requested   = 0U;
    s_display_initialized = 0U;
    s_display_ready       = 0U;
  }

  if (s_display_initialized == 0U) {
    return Display_Init();
  }

  if (s_display_ready == 0U) {
    now_ms = Px4Lite_PlatformGetMs();
    if (Display_LvglProbeRecover(now_ms) == DISPLAY_OK) {
      s_display_ready = 1U;
      (void)Display_LvglSetPage(s_current_page);
      return DISPLAY_OK;
    }
  }

  return (s_display_ready != 0U) ? DISPLAY_OK : DISPLAY_NOT_READY;
}

/*
 * 向 HMI 变量缓存写入新值，并同步给 LVGL 显示层。
 */
static Display_Result_t Display_SetValue(Display_HmiVariableId_t id, Display_HmiDataType_t type, uint32_t value)
{
  const Display_HmiVariableConfig_t *variable = Display_GetVariableConfig(id);

  if ((variable == 0) || (id >= DISPLAY_HMI_VAR_COUNT)) { return DISPLAY_ERROR; }

  if (variable->data_type != type) { return DISPLAY_ERROR; }

  if ((s_hmi_values[id].valid != 0U) && (s_hmi_values[id].value == value)) { return DISPLAY_OK; }

  s_hmi_values[id].value = value;
  s_hmi_values[id].valid = 1U;
  (void)Display_LvglSetValue(id, value);
  return DISPLAY_OK;
}

/*
 * 加载本地模拟显示数据，用于调试屏幕绘制。
 */
#ifdef DEBUG_ENABLE
static void Display_LoadMockValues(void)
{
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_5GA, 2U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, 2U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_BME280, 2U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_SD, 3U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR, 2U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2, 2U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3, 2U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4, 2U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_POWER, 2U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_BUZZER, 2U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_KEY, 2U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_DEBUG, 2U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_GNSS, 2U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_LORA, 1U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_REMOTEID, 1U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_ERROR_CODE, 0U);

  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_DATA_SELFCHECK_RESULT, 0x00000020U);

  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SYSTEM_STATUS, 2U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_UPTIME_MS, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_CLOCK_TIME, 93000U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_DATE, 20260617U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_FLIGHT_TIME_S, 768U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_BATTERY_VOLTAGE, 1180U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_BATTERY_PERCENT, 86U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_FIX, 2U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_SAT_COUNT, 12U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_HDOP, 95U);
  (void)Display_SetHmiValueI32(DISPLAY_HMI_VAR_LATITUDE, 399084321);
  (void)Display_SetHmiValueI32(DISPLAY_HMI_VAR_LONGITUDE, 1163971280);
  (void)Display_SetHmiValueI32(DISPLAY_HMI_VAR_ALTITUDE, 43800);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_SPEED, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_GNSS_TIME, 93000U);
  (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_ROLL, 12);
  (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_PITCH, 8);
  (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_YAW, 156);
  (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_TEMPERATURE, 256);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_HUMIDITY, 584U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_PRESSURE, 101325U);

  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_LORA_STATUS, 1U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_LORA_TX_COUNT, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_LORA_RX_COUNT, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_LORA_HEARTBEAT, 1U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_LORA_ACK_COUNT, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_LORA_LOSS_RATE, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_ALARM_CODE, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_1, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_2, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_3, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_4, 0U);

  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ACTIVE_MASK, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ROW1_CODE, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ROW2_CODE, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ROW3_CODE, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ROW4_CODE, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ROW5_CODE, 0U);
}
#endif

/*
 * 写入 uint16 类型 HMI 变量缓存。
 */
Display_Result_t Display_SetHmiValueU16(Display_HmiVariableId_t id, uint16_t value)
{
  return Display_SetValue(id, DISPLAY_HMI_TYPE_U16, value);
}

/*
 * 写入 int16 类型 HMI 变量缓存。
 */
Display_Result_t Display_SetHmiValueI16(Display_HmiVariableId_t id, int16_t value)
{
  return Display_SetValue(id, DISPLAY_HMI_TYPE_I16, (uint16_t)value);
}

/*
 * 写入 uint32 类型 HMI 变量缓存。
 */
Display_Result_t Display_SetHmiValueU32(Display_HmiVariableId_t id, uint32_t value)
{
  return Display_SetValue(id, DISPLAY_HMI_TYPE_U32, value);
}

/*
 * 写入 int32 类型 HMI 变量缓存。
 */
Display_Result_t Display_SetHmiValueI32(Display_HmiVariableId_t id, int32_t value)
{
  return Display_SetValue(id, DISPLAY_HMI_TYPE_I32, (uint32_t)value);
}

/*
 * 读取 HMI 变量缓存中的原始值。
 */
Display_Result_t Display_GetHmiRawValue(Display_HmiVariableId_t id, uint32_t *value)
{
  if ((id >= DISPLAY_HMI_VAR_COUNT) || (value == 0) || (s_hmi_values[id].valid == 0U)) { return DISPLAY_ERROR; }

  *value = s_hmi_values[id].value;
  return DISPLAY_OK;
}

/*
 * 校验指定 HMI 变量是否支持读取请求。
 */
Display_Result_t Display_RequestHmiRead(Display_HmiVariableId_t id)
{
  if ((Display_GetVariableConfig(id) == 0) || (id >= DISPLAY_HMI_VAR_COUNT)) { return DISPLAY_ERROR; }

  return DISPLAY_OK;
}

/*
 * 初始化 Display 模块、端口和初始页面。
 */
Display_Result_t Display_Init(void)
{
  uint16_t i;

  for (i = 0U; i < DISPLAY_HMI_VAR_COUNT; i++) {
    s_hmi_values[i].value = 0U;
    s_hmi_values[i].valid = 0U;
  }

  if (Display_LvglInit(Px4Lite_PlatformGetMs()) != DISPLAY_OK) {
    s_display_ready       = 0U;
    s_display_initialized = 1U;
    return DISPLAY_ERROR;
  }

  s_display_ready         = 1U;
  s_display_initialized   = 1U;
  s_current_page          = DISPLAY_HMI_PAGE_SELF_CHECK;
  Display_InitSelfCheckValues();
#ifdef DEBUG_ENABLE
  Display_LoadMockValues();
#endif

  return DISPLAY_OK;
}

/*
 * 执行显示模块自检并返回错误码。
 */
Display_Result_t Display_SelfCheck(uint16_t *error_code)
{
  if (error_code != 0) { *error_code = 0U; }

  return Display_LvglSelfCheck(error_code);
}

void Display_RequestRecover(void)
{
  s_recover_requested = 1U;
  Display_LvglRequestRecover();
}

/*
 * 根据当前时间刷新当前页的动态字段和告警图标。
 */
Display_Result_t Display_Refresh(uint32_t now_ms)
{
  return Display_LvglRefreshStep(now_ms, 0U);
}

/*
 * 将 Framework 模块状态映射为页面状态灯数值。
 */
static uint16_t Display_MapStateValue(App_ViewState_t state)
{
  if (state == APP_VIEW_STATE_ONLINE) { return 2U; }

  if ((state == APP_VIEW_STATE_STARTING) || (state == APP_VIEW_STATE_DEGRADED)) { return 1U; }

  if ((state == APP_VIEW_STATE_OFFLINE) || (state == APP_VIEW_STATE_FAILED)) { return 3U; }

  return 0U;
}

/*
 * 存储自检为二元状态：内存卡就绪(ONLINE)显示绿灯(2)，否则一律红灯(3)。
 */
static uint16_t Display_MapStorageStateValue(App_ViewState_t state)
{
  return (state == APP_VIEW_STATE_ONLINE) ? 2U : 3U;
}

static uint16_t Display_MapMotorStateValue(const App_DisplaySnapshot_t *view)
{
  if (view == 0) { return 0U; }
  if ((view->voltage2_mv < 5000U) || ((view->battery2_percent == 0U) && (view->voltage2_mv < 9000U))) { return 3U; }
  if ((view->low_voltage2 != 0U) || (view->voltage2_mv < 10500U)) { return 1U; }
  return Display_MapStateValue(view->control.state);
}

static uint32_t Display_AbsI32ToU32(int32_t value)
{
  if (value < 0) { return (uint32_t)(-(value + 1)) + 1U; }
  return (uint32_t)value;
}

static uint32_t Display_U64Sqrt(uint64_t value)
{
  uint64_t bit = 1ULL << 62;
  uint64_t root = 0ULL;

  while (bit > value) { bit >>= 2U; }

  while (bit != 0ULL) {
    if (value >= (root + bit)) {
      value -= root + bit;
      root = (root >> 1U) + bit;
    } else {
      root >>= 1U;
    }
    bit >>= 2U;
  }

  return (root > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)root;
}

static uint16_t Display_VectorSpeedCmsToU16(int32_t north_cms, int32_t east_cms)
{
  uint32_t north = Display_AbsI32ToU32(north_cms);
  uint32_t east  = Display_AbsI32ToU32(east_cms);
  uint64_t sum   = ((uint64_t)north * (uint64_t)north) + ((uint64_t)east * (uint64_t)east);
  uint32_t speed = Display_U64Sqrt(sum);

  return (speed > 65535U) ? 65535U : (uint16_t)speed;
}

static int16_t Display_FloatToI16Tenths(float value)
{
  float scaled = value * 10.0f;

  if (scaled > 32767.0f) { return 32767; }
  if (scaled < -32768.0f) { return -32768; }

  return (int16_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static uint16_t Display_FloatToU16Tenths(float value)
{
  float scaled = value * 10.0f;

  if (scaled <= 0.0f) { return 0U; }
  if (scaled > 65535.0f) { return 65535U; }

  return (uint16_t)(scaled + 0.5f);
}

static uint32_t Display_FloatPaToU32(float pressure_pa)
{
  if (!(pressure_pa > 0.0f)) { return 0U; }
  if (pressure_pa > DISPLAY_PRESSURE_MAX_PA) { return (uint32_t)DISPLAY_PRESSURE_MAX_PA; }

  return (uint32_t)(pressure_pa + 0.5f);
}

static void Display_ClearNavigationSnapshot(void)
{
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_FIX, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_SAT_COUNT, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_HDOP, 0U);
  (void)Display_SetHmiValueI32(DISPLAY_HMI_VAR_LATITUDE, 0);
  (void)Display_SetHmiValueI32(DISPLAY_HMI_VAR_LONGITUDE, 0);
  (void)Display_SetHmiValueI32(DISPLAY_HMI_VAR_ALTITUDE, 0);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_SPEED, 0U);
  (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_ROLL, 0);
  (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_PITCH, 0);
  (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_YAW, 0);
}

static void Display_ClearAttitudeFields(void)
{
  (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_ROLL, 0);
  (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_PITCH, 0);
  (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_YAW, 0);
}

static void Display_ClearEnvironmentFields(void)
{
  (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_TEMPERATURE, 0);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_HUMIDITY, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_PRESSURE, 0U);
}

static void Display_ClearBatteryFields(void)
{
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_BATTERY_VOLTAGE, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_BATTERY_PERCENT, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT, 0U);
}

static void Display_ClearMotorFields(void)
{
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_1, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_2, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_3, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_4, 0U);
}

static void Display_LoadMotorSnapshot(const App_DisplaySnapshot_t *view)
{
  if (view == 0) { return; }

  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_1, view->motor_duty_percent[0]);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_2, view->motor_duty_percent[1]);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_3, view->motor_duty_percent[2]);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_4, view->motor_duty_percent[3]);
}

/*
 * 将导航快照字段写入 Display 缓存。
 */
static void Display_LoadNavigationSnapshot(const App_DisplaySnapshot_t *view)
{
  uint16_t ground_speed_cms;

  if (view == 0) { return; }

  ground_speed_cms = Display_VectorSpeedCmsToU16(view->velocity_north_cms, view->velocity_east_cms);

  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_FIX, (uint16_t)view->gnss_fix_type);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_SAT_COUNT, (uint16_t)view->satellites_used);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_HDOP, view->hdop_x100);
  (void)Display_SetHmiValueI32(DISPLAY_HMI_VAR_LATITUDE, view->latitude_e7);
  (void)Display_SetHmiValueI32(DISPLAY_HMI_VAR_LONGITUDE, view->longitude_e7);
  (void)Display_SetHmiValueI32(DISPLAY_HMI_VAR_ALTITUDE, view->altitude_mm);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_SPEED, ground_speed_cms);
  if (view->attitude_valid != 0U) {
    (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_ROLL, (int16_t)(view->roll_deg100 / 10));
    (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_PITCH, (int16_t)(view->pitch_deg100 / 10));
    (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_YAW, (int16_t)(view->yaw_deg100 / 10));
  } else {
    Display_ClearAttitudeFields();
  }
}

/*
 * 将统一日期时间快照写入 Display 缓存。
 */
static void Display_LoadDateTimeSnapshot(const App_DisplaySnapshot_t *view)
{
  if (view == 0) { return; }

  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_GNSS_TIME, view->local_time_hhmmss);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_CLOCK_TIME, view->local_time_hhmmss);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_DATE, view->local_date_ymd);
}

/*
 * 清除统一日期时间显示字段。
 */
static void Display_ClearDateTimeSnapshot(void)
{
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_GNSS_TIME, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_CLOCK_TIME, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_DATE, 0U);
}

/*
 * 将系统状态快照字段写入 Display 缓存。
 */
static void Display_LoadSystemSnapshot(const App_DisplaySnapshot_t *view)
{
  uint32_t alarm_row;
  uint16_t motor_state;

  if (view == 0) { return; }

  alarm_row = (view->highest_fault_code != 0U) ? (((uint32_t)view->highest_source_id << 16) | view->highest_fault_code) : 0U;
  motor_state = Display_MapMotorStateValue(view);

  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SYSTEM_STATUS, view->system_ready ? 2U : 1U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_VIEW_NODE_ID, view->view_node_id);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_DATA_SELFCHECK_RESULT, view->status_version);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_ALARM_CODE, view->highest_fault_code);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ACTIVE_MASK, view->warning_fault_mask | view->blocking_fault_mask);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ROW1_CODE, alarm_row);

  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_GNSS, Display_MapStateValue(view->gnss.state));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, Display_MapStateValue(view->imu.state));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_BME280, Display_MapStateValue(view->baro.state));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_LORA, Display_MapStateValue(view->lora.state));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_REMOTEID, Display_MapStateValue(view->remote_id.state));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_LORA_STATUS, Display_MapStateValue(view->lora.state));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_SD, Display_MapStorageStateValue(view->storage.state));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR, motor_state);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2, motor_state);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3, motor_state);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4, motor_state);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_5GA, Display_MapStateValue(view->five_g.state));
  /* 自检页错误码表由 Display_LoadAlarmSnapshot 按激活故障列表整体刷新，
     此处不再用单个 highest_fault_code 驱动。 */
}

static void Display_LoadAlarmSnapshot(const App_DisplaySnapshot_t *view)
{
  static const Display_HmiVariableId_t row_ids[] = {DISPLAY_HMI_VAR_ALARM_ROW1_CODE, DISPLAY_HMI_VAR_ALARM_ROW2_CODE, DISPLAY_HMI_VAR_ALARM_ROW3_CODE, DISPLAY_HMI_VAR_ALARM_ROW4_CODE, DISPLAY_HMI_VAR_ALARM_ROW5_CODE};
  uint16_t record_index;
  uint16_t row_index   = 0U;
  uint32_t active_mask = 0U;
  uint16_t selfcheck_count = 0U;
  uint32_t selfcheck_fp    = 0U;

  if (view == 0) { return; }

  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_ALARM_CODE, view->alarm_highest_fault_code);

  for (record_index = 0U; record_index < APP_DISPLAY_ALARM_MAX; record_index++) {
    const App_AlarmRecord_t *record = &view->alarms[record_index];

    if ((record->active != 0U) && (record->source_id < 32U)) { active_mask |= (1UL << record->source_id); }

    /* 收集自检页错误码表的激活故障：有一条记一条，故障恢复(active=0)
       后不再计入，对应行会随之消失。 */
    if ((record->active != 0U) && (record->fault_code != 0U)) {
      uint32_t packed = ((uint32_t)record->source_id << 16) | (uint32_t)record->fault_code;
      selfcheck_count++;
      selfcheck_fp = (selfcheck_fp * 31U) + packed;
    }
  }

  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ACTIVE_MASK, active_mask);

  /* 把当前激活故障列表交给绘制层，并用集合指纹驱动错误码表重绘：
     故障集合不变则指纹不变(不重绘)，新增/消除任一故障即触发刷新。 */
  selfcheck_fp = (selfcheck_fp ^ (selfcheck_fp >> 16)) + selfcheck_count;
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_ERROR_CODE, (uint16_t)selfcheck_fp);

  for (record_index = 0U; (record_index < APP_DISPLAY_ALARM_MAX) && (row_index < DISPLAY_ARRAY_SIZE(row_ids)); record_index++) {
    const App_AlarmRecord_t *record = &view->alarms[record_index];

    if ((record->active != 0U) && (record->fault_code != 0U)) {
      (void)Display_SetHmiValueU32(row_ids[row_index], ((uint32_t)record->source_id << 16) | record->fault_code);
      row_index++;
    }
  }

  while (row_index < DISPLAY_ARRAY_SIZE(row_ids)) {
    (void)Display_SetHmiValueU32(row_ids[row_index], 0U);
    row_index++;
  }
}

static void Display_LoadEnvironmentSnapshot(const App_DisplaySnapshot_t *view)
{
  if (view == 0) { return; }

  (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_TEMPERATURE, Display_FloatToI16Tenths(view->temperature_c));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_HUMIDITY, Display_FloatToU16Tenths(view->relative_humidity_pct));
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_PRESSURE, Display_FloatPaToU32(view->pressure_pa));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_BATTERY_VOLTAGE, (uint16_t)((view->voltage_mv + 5U) / 10U));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_BATTERY_PERCENT, view->battery_percent);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE, (uint16_t)((view->voltage2_mv + 5U) / 10U));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT, view->battery2_percent);
}

/*
 * 将 LoRa 通信统计(收发帧计数和接收侧估算丢包率)写入 Display 缓存。
 * 计数为累计值，始终可读，无需新鲜度判定。
 */
static void Display_LoadLoraStats(const App_DisplaySnapshot_t *view)
{
  if (view == 0) { return; }

  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_LORA_TX_COUNT, view->lora_tx_count);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_LORA_RX_COUNT, view->lora_rx_count);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_LORA_HEARTBEAT, Display_MapStateValue(view->lora.state));
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_LORA_ACK_COUNT, view->lora_ack_count);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_LORA_LOSS_RATE, view->lora_loss_rate_x10);
}

/*
 * 远端快照断链或超时后，立即回到本机视图并重建当前 LVGL 页面。
 * 这样显示任务不会长期停在远端数据源的 NOT_READY 状态，也不会继续显示过期从机页面。
 */
static void Display_ReturnToLocalView(uint32_t now_ms)
{
  (void)App_SetRemoteViewEnabled(0U, now_ms);
  (void)Display_SetDataSource(DISPLAY_DATA_SOURCE_LOCAL);
  if (s_display_ready != 0U) {
    (void)Display_LvglSetPage(s_current_page);
  }
}

/*
 * 从应用只读快照准备一版完整显示缓存。
 */
Display_Result_t Display_PrepareSnapshot(uint32_t now_ms)
{
  uint8_t snapshot_ready;

  if (Display_EnsureInit() != DISPLAY_OK) { return DISPLAY_NOT_READY; }

  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_UPTIME_MS, now_ms);
  /* 飞行时间(上电后运行)，秒粒度，避免毫秒每帧抖动导致重绘 */
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_FLIGHT_TIME_S, now_ms / 1000U);

  if ((s_data_source == DISPLAY_DATA_SOURCE_REMOTE) && (App_RemoteViewExpired(now_ms) != 0U)) {
    Display_ReturnToLocalView(now_ms);
  }

  if (s_data_source == DISPLAY_DATA_SOURCE_REMOTE) {
    snapshot_ready = App_CopyRemoteDisplaySnapshot(&s_display_snapshot, now_ms);
    if (snapshot_ready == 0U) {
      return DISPLAY_NOT_READY;
    }
  } else {
    snapshot_ready = App_CopyDisplaySnapshot(&s_display_snapshot, now_ms);
  }

  if (snapshot_ready == 0U) { return DISPLAY_NOT_READY; }

  if (s_display_snapshot.navigation_valid != 0U) {
    Display_LoadNavigationSnapshot(&s_display_snapshot);
  } else {
    Display_ClearNavigationSnapshot();
  }

  if (s_display_snapshot.date_time_valid != 0U) {
    Display_LoadDateTimeSnapshot(&s_display_snapshot);
  } else {
    Display_ClearDateTimeSnapshot();
  }

  Display_LoadSystemSnapshot(&s_display_snapshot);

  Display_LoadAlarmSnapshot(&s_display_snapshot);

  if (s_display_snapshot.environment_valid != 0U) {
    Display_LoadEnvironmentSnapshot(&s_display_snapshot);
  } else {
    Display_ClearEnvironmentFields();
    Display_ClearBatteryFields();
  }

  if (s_display_snapshot.motor_valid != 0U) {
    Display_LoadMotorSnapshot(&s_display_snapshot);
  } else {
    Display_ClearMotorFields();
  }

  /* LoRa 收发帧计数和丢包率为累计统计，独立于上面三个快照，每帧都刷新 */
  Display_LoadLoraStats(&s_display_snapshot);

  /* 消息日志缓冲版本变化即触发日志区重绘。 */
  if (s_data_source == DISPLAY_DATA_SOURCE_REMOTE) {
    (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_MESSAGE_LOG, 0x80000000UL | App_GetRemoteMessageLogVersion(now_ms));
  } else {
    (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_MESSAGE_LOG, App_MessageLogGetVersion());
  }

  return DISPLAY_OK;
}

/*
 * 执行一次有预算约束的显示刷新步骤。
 */
Display_Result_t Display_RefreshStep(uint32_t now_ms, uint32_t budget_us)
{
  Display_Result_t result = Display_LvglRefreshStep(now_ms, budget_us);

  if ((result == DISPLAY_NOT_READY) && (Display_LvglNeedsRefresh() == 0U)) {
    s_display_ready = 0U;
  } else if (result == DISPLAY_OK) {
    s_display_ready = 1U;
  }

  return result;
}

/*
 * 切换当前 HMI 页面。
 */
Display_Result_t Display_SetHmiPage(Display_HmiPage_t page)
{
  if (page >= DISPLAY_HMI_PAGE_COUNT) { return DISPLAY_ERROR; }

  if (s_display_ready == 0U) { return DISPLAY_NOT_READY; }

  s_current_page = page;

  return Display_LvglSetPage(page);
}

/*
 * 获取当前 HMI 页面 ID。
 */
Display_HmiPage_t Display_GetCurrentHmiPage(void)
{
  return s_current_page;
}

Display_Result_t Display_SetDataSource(Display_DataSource_t source)
{
  if ((source != DISPLAY_DATA_SOURCE_LOCAL) && (source != DISPLAY_DATA_SOURCE_REMOTE)) { return DISPLAY_ERROR; }
  if (s_data_source == source) { return DISPLAY_OK; }

  s_data_source = source;
  return DISPLAY_OK;
}

Display_DataSource_t Display_GetDataSource(void)
{
  return s_data_source;
}

uint8_t Display_HasPendingRedraw(void)
{
  return Display_LvglNeedsRefresh();
}

/*
 * 获取 HMI 页面配置表数量。
 */
uint16_t Display_GetHmiPageCount(void)
{
  return DISPLAY_ARRAY_SIZE(s_hmi_pages);
}

/*
 * 获取 HMI 页面配置表。
 */
const Display_HmiPageConfig_t *Display_GetHmiPageTable(void)
{
  return s_hmi_pages;
}

/*
 * 获取 HMI 变量配置表数量。
 */
uint16_t Display_GetHmiVariableCount(void)
{
  return DISPLAY_ARRAY_SIZE(s_hmi_variables);
}

/*
 * 获取 HMI 变量配置表。
 */
const Display_HmiVariableConfig_t *Display_GetHmiVariableTable(void)
{
  return s_hmi_variables;
}

/*
 * 获取 HMI 触摸区域配置表数量。
 */
uint16_t Display_GetHmiTouchRegionCount(void)
{
  return 0U;
}

/*
 * 获取 HMI 触摸区域配置表。
 */
const Display_HmiTouchRegionConfig_t *Display_GetHmiTouchRegionTable(void)
{
  return 0;
}
