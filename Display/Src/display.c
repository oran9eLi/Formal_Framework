/**
 * @file display.c
 * @brief Display 层本机快照、输入门面和预算化刷新。
 * @details 仅消费 App 快照；事件由所属功能记录，LCD 生命周期不决定控制或日志。
 */

#include "display.h"
#include "app_data_api.h"
#include "app_display_model.h"
#include "px4lite_faults.h"
#include "px4lite_config.h"
#include "px4lite_platform.h"
#include "display_lvgl.h"
#include "display_pages.h"
#include "debug_console.h"
#include <math.h>
#include <string.h>

#define DISPLAY_USE_LVGL_BACKEND 1U

#define DISPLAY_ARRAY_SIZE(array) ((uint16_t)(sizeof(array) / sizeof((array)[0])))

#define DISPLAY_MOTOR_SLIDER_MAX_VALUE 100U
#define DISPLAY_INIT_RETRY_MS          500U

typedef struct {
  uint32_t value;           /* 当前缓存的原始显示值 */
  uint32_t drawn_value;     /* 上一次已经绘制到屏幕的值 */
  uint32_t last_refresh_ms; /* 上一次字段重绘时间 */
  uint8_t valid;            /* 0 表示缓存无效，1 表示可用于绘制 */
  uint8_t drawn_valid;      /* 0 表示当前页还未绘制过该字段 */
  uint8_t dirty;            /* 1 表示下次 LVGL 刷新需要重绘 */
} Display_ValueCache_t;

/*
 * 页面 ID 用于 App 和调试链路路由，枚举值保持稳定。
 */
static const Display_HmiPageConfig_t s_hmi_pages[] = {{DISPLAY_HMI_PAGE_SELF_CHECK, 0x0000U, "self_check", 200U}, {DISPLAY_HMI_PAGE_FLIGHT, 0x0004U, "flight", 500U}, {DISPLAY_HMI_PAGE_AIRCRAFT, 0x0005U, "aircraft", 500U}, {DISPLAY_HMI_PAGE_DATA, 0x0001U, "data", 500U}, {DISPLAY_HMI_PAGE_MOTOR, 0x0006U, "motor", 500U}, {DISPLAY_HMI_PAGE_ALARM, 0x0002U, "alarm", 200U}, {DISPLAY_HMI_PAGE_HIDDEN, 0x0007U, "comm", 200U}, {DISPLAY_HMI_PAGE_DIAGNOSTICS, 0x0008U, "diagnostics", 200U}, {DISPLAY_HMI_PAGE_EVENT_HISTORY, 0x0009U, "event_history", 200U}};

/*
 * ATK-MD0700 800x480 显示变量配置。
 * var_addr 是固件内部变量地址，不是 LCD 控制器寄存器地址。
 */
static const Display_HmiVariableConfig_t s_hmi_variables[] = {
    /* 上电自检页 -- 11 个主要设备状态 */
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
    {DISPLAY_HMI_VAR_SELF_CHECK_ERROR_CODE, DISPLAY_HMI_PAGE_SELF_CHECK, 0x100BU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 480U, 82U, 312U, 310U, "self_check_errcode", "-", "App_Registry"},    {DISPLAY_HMI_VAR_SELF_CHECK_GNSS, DISPLAY_HMI_PAGE_DATA, 0x1009U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 122U, 24U, 16U, "data_status_gnss", "-", "App_Registry"},
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
    {DISPLAY_HMI_VAR_PRESSURE, DISPLAY_HMI_PAGE_DATA, 0x1304U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "pressure", "Pa", "CNS_State.sensor"},    {DISPLAY_HMI_VAR_LORA_TX_COUNT, DISPLAY_HMI_PAGE_DATA, 0x1401U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "lora_tx_count", "-", "CNS_State.lora"},
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

    /* 飞行数据页 -- 左侧系统栏模块状态灯 */
    {DISPLAY_HMI_VAR_SELF_CHECK_GNSS, DISPLAY_HMI_PAGE_FLIGHT, 0x1009U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 122U, 24U, 16U, "flight_status_gnss", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, DISPLAY_HMI_PAGE_FLIGHT, 0x1001U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 152U, 24U, 16U, "flight_status_mpu", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_BME280, DISPLAY_HMI_PAGE_FLIGHT, 0x1002U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 182U, 24U, 16U, "flight_status_bme", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_LORA, DISPLAY_HMI_PAGE_FLIGHT, 0x100AU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 212U, 24U, 16U, "flight_status_lora", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_SD, DISPLAY_HMI_PAGE_FLIGHT, 0x1003U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 242U, 24U, 16U, "flight_status_sd", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR, DISPLAY_HMI_PAGE_FLIGHT, 0x1004U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 272U, 24U, 16U, "flight_status_motor1", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2, DISPLAY_HMI_PAGE_FLIGHT, 0x100CU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 302U, 24U, 16U, "flight_status_motor2", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3, DISPLAY_HMI_PAGE_FLIGHT, 0x100DU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 332U, 24U, 16U, "flight_status_motor3", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4, DISPLAY_HMI_PAGE_FLIGHT, 0x100EU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 362U, 24U, 16U, "flight_status_motor4", "-", "App_Registry"},
    /* 飞行数据页 -- 中间栏：主控/电机电池、飞行时间、温度/湿度/气压 */
    {DISPLAY_HMI_VAR_BATTERY_VOLTAGE, DISPLAY_HMI_PAGE_FLIGHT, 0x1103U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 116U, 124U, 22U, "flight_main_voltage", "V", "CNS_State.power"},
    {DISPLAY_HMI_VAR_BATTERY_CURRENT, DISPLAY_HMI_PAGE_FLIGHT, 0x110BU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 134U, 124U, 22U, "flight_main_current", "A", "CNS_State.power"},
    {DISPLAY_HMI_VAR_BATTERY_PERCENT, DISPLAY_HMI_PAGE_FLIGHT, 0x1104U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 152U, 124U, 22U, "flight_main_battery", "%", "CNS_State.power"},
    {DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE, DISPLAY_HMI_PAGE_FLIGHT, 0x1105U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 188U, 124U, 22U, "flight_motor_voltage", "V", "CNS_State.power"},
    {DISPLAY_HMI_VAR_MOTOR_BAT_CURRENT, DISPLAY_HMI_PAGE_FLIGHT, 0x110CU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 206U, 124U, 22U, "flight_motor_current", "A", "CNS_State.power"},
    {DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT, DISPLAY_HMI_PAGE_FLIGHT, 0x1107U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 224U, 124U, 22U, "flight_motor_battery", "%", "CNS_State.power"},
    {DISPLAY_HMI_VAR_MESSAGE_LOG, DISPLAY_HMI_PAGE_DATA, 0x110AU, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 540U, 76U, 252U, 330U, "msg_log_data", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_MESSAGE_LOG, DISPLAY_HMI_PAGE_FLIGHT, 0x110AU, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 540U, 76U, 252U, 330U, "msg_log_flight", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_MESSAGE_LOG, DISPLAY_HMI_PAGE_AIRCRAFT, 0x110AU, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 540U, 76U, 252U, 330U, "msg_log_aircraft", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_VIEW_NODE_ID, DISPLAY_HMI_PAGE_FLIGHT, 0x110DU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 500U, 0U, 0U, 0U, 0U, "view_node_id", "-", "App_Display"},
    {DISPLAY_HMI_VAR_DATE, DISPLAY_HMI_PAGE_FLIGHT, 0x110EU, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "clock_date", "-", "App_Display"},
    {DISPLAY_HMI_VAR_CLOCK_TIME, DISPLAY_HMI_PAGE_FLIGHT, 0x110FU, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 0U, 0U, 0U, 0U, "clock_time", "-", "App_Display"},
    {DISPLAY_HMI_VAR_FLIGHT_TIME_S, DISPLAY_HMI_PAGE_FLIGHT, 0x1106U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 260U, 124U, 22U, "flight_time_s", "-", "CNS_State.system"},
    {DISPLAY_HMI_VAR_TEMPERATURE, DISPLAY_HMI_PAGE_FLIGHT, 0x1302U, DISPLAY_HMI_TYPE_I16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 296U, 124U, 22U, "temperature",
     "\xB0"
     "C",
     "CNS_State.sensor"},
    {DISPLAY_HMI_VAR_HUMIDITY, DISPLAY_HMI_PAGE_FLIGHT, 0x1303U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 332U, 124U, 22U, "humidity", "%", "CNS_State.sensor"},
    {DISPLAY_HMI_VAR_PRESSURE, DISPLAY_HMI_PAGE_FLIGHT, 0x1304U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 368U, 124U, 22U, "pressure", "Pa", "CNS_State.sensor"},

    /* 飞机情况页 -- 左侧系统栏模块状态灯 */
    {DISPLAY_HMI_VAR_SELF_CHECK_GNSS, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1009U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 122U, 24U, 16U, "ac_status_gnss", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1001U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 152U, 24U, 16U, "ac_status_mpu", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_BME280, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1002U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 182U, 24U, 16U, "ac_status_bme", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_LORA, DISPLAY_HMI_PAGE_AIRCRAFT, 0x100AU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 212U, 24U, 16U, "ac_status_lora", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_SD, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1003U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 242U, 24U, 16U, "ac_status_sd", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1004U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 272U, 24U, 16U, "ac_status_motor1", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2, DISPLAY_HMI_PAGE_AIRCRAFT, 0x100CU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 302U, 24U, 16U, "ac_status_motor2", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3, DISPLAY_HMI_PAGE_AIRCRAFT, 0x100DU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 332U, 24U, 16U, "ac_status_motor3", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4, DISPLAY_HMI_PAGE_AIRCRAFT, 0x100EU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 362U, 24U, 16U, "ac_status_motor4", "-", "App_Registry"},
    /* 飞机情况页 -- 中间栏：横滚、俯仰、偏航、LoRa 发送/接收 */
    {DISPLAY_HMI_VAR_ROLL, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1300U, DISPLAY_HMI_TYPE_I16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 118U, 124U, 22U, "ac_roll", "\xB0", "CNS_State.sensor"},
    {DISPLAY_HMI_VAR_PITCH, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1301U, DISPLAY_HMI_TYPE_I16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 150U, 124U, 22U, "ac_pitch", "\xB0", "CNS_State.sensor"},
    {DISPLAY_HMI_VAR_YAW, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1305U, DISPLAY_HMI_TYPE_I16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 182U, 124U, 22U, "ac_yaw", "\xB0", "CNS_State.navigation"},
    {DISPLAY_HMI_VAR_ATTITUDE_ACTION, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1310U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 0U, 0U, 0U, 0U, "attitude_action", "-", "App_Attitude"},
    {DISPLAY_HMI_VAR_ATTITUDE_PHASE, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1311U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 0U, 0U, 0U, 0U, "attitude_phase", "-", "App_Attitude"},
    {DISPLAY_HMI_VAR_ATTITUDE_REASON, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1312U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 0U, 0U, 0U, 0U, "attitude_reason", "-", "App_Attitude"},
    {DISPLAY_HMI_VAR_ATTITUDE_PROGRESS, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1313U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 0U, 0U, 0U, 0U, "attitude_progress", "%", "App_Attitude"},
    {DISPLAY_HMI_VAR_ATTITUDE_REFERENCE, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1314U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 0U, 0U, 0U, 0U, "attitude_reference", "-", "App_Attitude"},
    {DISPLAY_HMI_VAR_LORA_TX_COUNT, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1401U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 214U, 124U, 22U, "ac_lora_tx", "-", "CNS_State.lora"},
    {DISPLAY_HMI_VAR_LORA_RX_COUNT, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1403U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 246U, 124U, 22U, "ac_lora_rx", "-", "CNS_State.lora"},

    /* 电机控制页 -- 左侧状态栏、四路竖向油门滑条和右侧消息日志 */
    {DISPLAY_HMI_VAR_SELF_CHECK_GNSS, DISPLAY_HMI_PAGE_MOTOR, 0x1009U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 122U, 24U, 16U, "mc_status_gnss", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, DISPLAY_HMI_PAGE_MOTOR, 0x1001U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 152U, 24U, 16U, "mc_status_mpu", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_BME280, DISPLAY_HMI_PAGE_MOTOR, 0x1002U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 182U, 24U, 16U, "mc_status_bme", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_LORA, DISPLAY_HMI_PAGE_MOTOR, 0x100AU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 212U, 24U, 16U, "mc_status_lora", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_SD, DISPLAY_HMI_PAGE_MOTOR, 0x1003U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 242U, 24U, 16U, "mc_status_sd", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR, DISPLAY_HMI_PAGE_MOTOR, 0x1004U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 272U, 24U, 16U, "mc_status_motor1", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2, DISPLAY_HMI_PAGE_MOTOR, 0x100CU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 302U, 24U, 16U, "mc_status_motor2", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3, DISPLAY_HMI_PAGE_MOTOR, 0x100DU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 332U, 24U, 16U, "mc_status_motor3", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4, DISPLAY_HMI_PAGE_MOTOR, 0x100EU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 18U, 362U, 24U, 16U, "mc_status_motor4", "-", "App_Registry"},    {DISPLAY_HMI_VAR_MOTOR_PWM_1, DISPLAY_HMI_PAGE_MOTOR, 0x1600U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RW, 0U, DISPLAY_MOTOR_TRACK1_X, DISPLAY_MOTOR_TRACK_TOP_Y, DISPLAY_MOTOR_TRACK_W, DISPLAY_MOTOR_TRACK_H, "motor_slider_1", "%", "HMI/Motor"},
    {DISPLAY_HMI_VAR_MOTOR_PWM_2, DISPLAY_HMI_PAGE_MOTOR, 0x1601U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RW, 0U, DISPLAY_MOTOR_TRACK2_X, DISPLAY_MOTOR_TRACK_TOP_Y, DISPLAY_MOTOR_TRACK_W, DISPLAY_MOTOR_TRACK_H, "motor_slider_2", "%", "HMI/Motor"},
    {DISPLAY_HMI_VAR_MOTOR_PWM_3, DISPLAY_HMI_PAGE_MOTOR, 0x1602U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RW, 0U, DISPLAY_MOTOR_TRACK3_X, DISPLAY_MOTOR_TRACK_TOP_Y, DISPLAY_MOTOR_TRACK_W, DISPLAY_MOTOR_TRACK_H, "motor_slider_3", "%", "HMI/Motor"},
    {DISPLAY_HMI_VAR_MOTOR_PWM_4, DISPLAY_HMI_PAGE_MOTOR, 0x1603U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RW, 0U, DISPLAY_MOTOR_TRACK4_X, DISPLAY_MOTOR_TRACK_TOP_Y, DISPLAY_MOTOR_TRACK_W, DISPLAY_MOTOR_TRACK_H, "motor_slider_4", "%", "HMI/Motor"},
    {DISPLAY_HMI_VAR_MOTOR_TARGET_1, DISPLAY_HMI_PAGE_MOTOR, 0x1610U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 0U, 0U, 0U, 0U, "motor_target_1", "%", "App_Control"},
    {DISPLAY_HMI_VAR_MOTOR_TARGET_2, DISPLAY_HMI_PAGE_MOTOR, 0x1611U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 0U, 0U, 0U, 0U, "motor_target_2", "%", "App_Control"},
    {DISPLAY_HMI_VAR_MOTOR_TARGET_3, DISPLAY_HMI_PAGE_MOTOR, 0x1612U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 0U, 0U, 0U, 0U, "motor_target_3", "%", "App_Control"},
    {DISPLAY_HMI_VAR_MOTOR_TARGET_4, DISPLAY_HMI_PAGE_MOTOR, 0x1613U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 0U, 0U, 0U, 0U, "motor_target_4", "%", "App_Control"},
    {DISPLAY_HMI_VAR_MOTOR_CONTROL_MODE, DISPLAY_HMI_PAGE_MOTOR, 0x1614U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 0U, 0U, 0U, 0U, "motor_control_mode", "-", "App_Control"},
    {DISPLAY_HMI_VAR_MOTOR_OPERATION_STATE, DISPLAY_HMI_PAGE_MOTOR, 0x1615U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 0U, 0U, 0U, 0U, "motor_operation_state", "-", "App_Control"},
    {DISPLAY_HMI_VAR_MESSAGE_LOG, DISPLAY_HMI_PAGE_MOTOR, 0x110AU, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 616U, 76U, 176U, 330U, "msg_log_motor", "-", "App_Registry"},

    /* 告警页 -- 运行期告警项 */    {DISPLAY_HMI_VAR_ALARM_ACTIVE_MASK, DISPLAY_HMI_PAGE_ALARM, 0x1700U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 52U, 72U, 696U, 38U, "alarm_active_mask", "-", "App_Alarm"},
    {DISPLAY_HMI_VAR_ALARM_ROW1_CODE, DISPLAY_HMI_PAGE_ALARM, 0x1701U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 42U, 150U, 716U, 48U, "alarm_row1_code", "-", "App_Alarm"},
    {DISPLAY_HMI_VAR_ALARM_ROW2_CODE, DISPLAY_HMI_PAGE_ALARM, 0x1702U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 42U, 200U, 716U, 48U, "alarm_row2_code", "-", "App_Alarm"},
    {DISPLAY_HMI_VAR_ALARM_ROW3_CODE, DISPLAY_HMI_PAGE_ALARM, 0x1703U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 42U, 250U, 716U, 48U, "alarm_row3_code", "-", "App_Alarm"},
    {DISPLAY_HMI_VAR_ALARM_ROW4_CODE, DISPLAY_HMI_PAGE_ALARM, 0x1704U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 42U, 300U, 716U, 48U, "alarm_row4_code", "-", "App_Alarm"},
    {DISPLAY_HMI_VAR_ALARM_ROW5_CODE, DISPLAY_HMI_PAGE_ALARM, 0x1705U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 42U, 350U, 716U, 48U, "alarm_row5_code", "-", "App_Alarm"}};

static Display_ValueCache_t s_hmi_values[DISPLAY_HMI_VAR_COUNT];
static Display_HmiPage_t s_current_page       = DISPLAY_HMI_PAGE_SELF_CHECK;
static uint8_t s_display_ready                = 0U;
static uint8_t s_display_initialized          = 0U;
static volatile uint8_t s_recover_requested   = 0U;
static uint32_t s_next_init_attempt_ms        = 0U;
static uint8_t s_motor_band                   = 4U; /* 电机电池档位(带滞回)，初值=正常 */
static uint8_t s_main_band                    = 2U; /* 主控电池档位(带滞回)，初值=正常 */
static uint16_t s_disp_main_dv                = 0U; /* 屏幕主控电压(去抖后)，单位 0.1V */
static uint16_t s_disp_motor_dv               = 0U; /* 屏幕电机电压(去抖后)，单位 0.1V */
static uint8_t s_disp_main_dv_valid           = 0U; /* 主控显示电压去抖状态是否已播种 */
static uint8_t s_disp_motor_dv_valid          = 0U; /* 电机显示电压去抖状态是否已播种 */

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

static void Display_ClearMotorOutputFields(void);
static void Display_ClearMotorFields(void);

static const Display_HmiVariableConfig_t *Display_FindMotorSliderByIndex(uint8_t motor_index)
{
  uint16_t i;

  for (i = 0U; i < DISPLAY_ARRAY_SIZE(s_hmi_variables); i++) {
    const Display_HmiVariableConfig_t *variable = &s_hmi_variables[i];
    uint8_t index;

    if ((variable->page != DISPLAY_HMI_PAGE_MOTOR) || (Display_MotorIndexFromId(variable->id, &index) == 0U)) { continue; }
    if (index == motor_index) { return variable; }
  }

  return 0;
}

/** @brief 把本机目标交给 Control；显示层不重复维护准入策略或事件。 */
static Display_Result_t Display_SetMotorThrottleCommand(const Display_HmiVariableConfig_t *variable, uint16_t throttle_percent)
{
  uint8_t motor_index;
  Px4Lite_Result_t result;
  if ((variable == 0) || (Display_MotorIndexFromId(variable->id, &motor_index) == 0U)) { return DISPLAY_ERROR; }
  if (throttle_percent > DISPLAY_MOTOR_SLIDER_MAX_VALUE) { throttle_percent = DISPLAY_MOTOR_SLIDER_MAX_VALUE; }
  /* 准入、拒绝原因和事件归 Control，避免显示提前拒绝而漏掉真实操作记录。 */
  result = App_SetMotorThrottlePercent(motor_index, (uint8_t)throttle_percent);
  if (result == PX4LITE_NOT_READY) { return DISPLAY_NOT_READY; }
  if (result != PX4LITE_OK) { return DISPLAY_ERROR; }
  (void)Display_SetHmiValueU16(
      (Display_HmiVariableId_t)((uint16_t)DISPLAY_HMI_VAR_MOTOR_TARGET_1 + motor_index), throttle_percent);
  return DISPLAY_OK;
}

/** @brief 本机急停仅在所属服务确认成功后回显受控最小输出。 */
static Display_Result_t Display_MotorEmergencyStop(void)
{
  Px4Lite_Result_t result;

  result = App_EmergencyStopMotors();
  if (result == PX4LITE_OK) {
    Display_ClearMotorOutputFields();
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_OPERATION_STATE, PX4LITE_MOTOR_OP_STOPPED);
  }

  return (result == PX4LITE_OK) ? DISPLAY_OK : DISPLAY_ERROR;
}

Display_Result_t Display_RequestMotorThrottle(Display_HmiVariableId_t id, uint16_t throttle_percent)
{
  const Display_HmiVariableConfig_t *variable;
  uint8_t motor_index;

  if (Display_MotorIndexFromId(id, &motor_index) == 0U) { return DISPLAY_ERROR; }

  variable = Display_FindMotorSliderByIndex(motor_index);
  return Display_SetMotorThrottleCommand(variable, throttle_percent);
}

/** @brief 发起本机同步升速；由 Control 判断准入并记录实际过程。 */
Display_Result_t Display_RequestMotorAutoTakeoff(void)
{
  Px4Lite_Result_t result = App_StartMotorAutoTakeoff();
  if (result == PX4LITE_NOT_READY) { return DISPLAY_NOT_READY; }
  return (result == PX4LITE_OK) ? DISPLAY_OK : DISPLAY_ERROR;
}

/** @brief 无论查看哪个节点，明确急停本机；远端数据保持由遥测更新。 */
Display_Result_t Display_RequestMotorEmergencyStop(void)
{
  return Display_MotorEmergencyStop();
}

/** @brief 发起本机缓降；由 Control 记录接受和达到最小输出。 */
Display_Result_t Display_RequestMotorAutoLanding(void)
{
  Px4Lite_Result_t result = App_StartMotorAutoLanding();
  return (result == PX4LITE_OK) ? DISPLAY_OK : DISPLAY_ERROR;
}

/** @brief 本机显式选择直控/姿态辅助；远端视图保持只读。 */
Display_Result_t Display_RequestMotorControlMode(Px4Lite_ControlMode_t mode)
{
  Px4Lite_Result_t result;

  if (App_GetRemoteDisplayMode() == PX4LITE_REMOTE_MODE_REMOTE) { return DISPLAY_NOT_READY; }
  result = App_SetMotorControlMode(mode);
  if (result == PX4LITE_NOT_READY) { return DISPLAY_NOT_READY; }
  return (result == PX4LITE_OK) ? DISPLAY_OK : DISPLAY_ERROR;
}

/** @brief 本地姿态操作透传有效结果，远端视图拒绝操作本机。 */
Display_Result_t Display_RequestAttitudeAction(Px4Lite_AttitudeAction_t action)
{
  Px4Lite_Result_t result;

  if (App_GetRemoteDisplayMode() == PX4LITE_REMOTE_MODE_REMOTE) { return DISPLAY_NOT_READY; }
  result = App_RequestAttitudeAction(action);
  if ((result == PX4LITE_NOT_READY) || (result == PX4LITE_BUSY)) { return DISPLAY_NOT_READY; }
  return (result == PX4LITE_OK) ? DISPLAY_OK : DISPLAY_ERROR;
}

/** @brief 兼容旧入口：语义是设置本机显示零位，不是传感器校准。 */
Display_Result_t Display_RequestAttitudeLevelCalibration(void)
{
  return Display_RequestAttitudeAction(PX4LITE_ATTITUDE_ACTION_ZERO);
}

static void Display_InitSelfCheckValues(void)
{
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_GNSS, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_BME280, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_LORA, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_REMOTEID, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_SD, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4, 0U);
}

/*
 * 确保 Display 硬件和绘图层已经初始化。
 */
static Display_Result_t Display_EnsureInit(void)
{
  Display_Result_t result;
  uint32_t now_ms;

  if (s_recover_requested != 0U) {
    s_recover_requested   = 0U;
    s_display_initialized = 0U;
    s_display_ready       = 0U;
    s_next_init_attempt_ms = 0U;
  }

  if (s_display_initialized == 0U) {
    now_ms = Px4Lite_PlatformGetMs();
    if ((s_next_init_attempt_ms != 0U) && ((int32_t)(now_ms - s_next_init_attempt_ms) < 0)) {
      return DISPLAY_NOT_READY;
    }

    result = Display_Init();
    if (result != DISPLAY_OK) { s_next_init_attempt_ms = now_ms + DISPLAY_INIT_RETRY_MS; }
    return result;
  }

  return s_display_ready ? DISPLAY_OK : DISPLAY_NOT_READY;
}

/*
 * 向 HMI 变量缓存写入新值，并标记为待重绘。
 */
static Display_Result_t Display_SetValue(Display_HmiVariableId_t id, Display_HmiDataType_t type, uint32_t value)
{
  const Display_HmiVariableConfig_t *variable = Display_GetVariableConfig(id);

  if ((variable == 0) || (id >= DISPLAY_HMI_VAR_COUNT)) { return DISPLAY_ERROR; }

  if (variable->data_type != type) { return DISPLAY_ERROR; }

  if ((s_hmi_values[id].valid != 0U) && (s_hmi_values[id].value == value)) {
    return DISPLAY_OK;
  }

  s_hmi_values[id].value = value;
  s_hmi_values[id].valid = 1U;
  s_hmi_values[id].dirty = 1U;
#if DISPLAY_USE_LVGL_BACKEND
  (void)Display_LvglSetValue(id, value);
#endif
  return DISPLAY_OK;
}

/** @brief 撤销一个 HMI 字段；保留原值但清除有效/已绘制标志，保证同值恢复也重绘。 */
static void Display_InvalidateValue(Display_HmiVariableId_t id)
{
  if ((id >= DISPLAY_HMI_VAR_COUNT) || (s_hmi_values[id].valid == 0U)) { return; }
  s_hmi_values[id].valid = 0U;
  s_hmi_values[id].drawn_valid = 0U;
  s_hmi_values[id].dirty = 1U;
#if DISPLAY_USE_LVGL_BACKEND
  (void)Display_LvglInvalidateValue(id);
#endif
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
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_LORA, 2U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_ERROR_CODE, 0U);

  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_DATA_SELFCHECK_RESULT, 0x00000020U);

  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SYSTEM_STATUS, 2U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_UPTIME_MS, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_CLOCK_TIME, 93000U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_DATE, 20260617U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_FLIGHT_TIME_S, 768U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_BATTERY_VOLTAGE, 1180U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_BATTERY_CURRENT, 24U);   /* 0.1A 单位，示例 2.4A */
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_BATTERY_PERCENT, 86U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE, 1200U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_BAT_CURRENT, 153U); /* 0.1A 单位，示例 15.3A */
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT, 90U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_FIX, 3U); /* 示例数据：3D定位 */
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

  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_LORA_STATUS, 2U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_LORA_TX_COUNT, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_LORA_RX_COUNT, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_LORA_HEARTBEAT, 2U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_LORA_ACK_COUNT, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_LORA_LOSS_RATE, 5U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_ALARM_CODE, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_1, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_2, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_3, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_4, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_TARGET_1, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_TARGET_2, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_TARGET_3, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_TARGET_4, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_CONTROL_MODE, 2U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_OPERATION_STATE, 255U);

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
    s_hmi_values[i].value           = 0U;
    s_hmi_values[i].drawn_value     = 0U;
    s_hmi_values[i].last_refresh_ms = 0U;
    s_hmi_values[i].valid           = 0U;
    s_hmi_values[i].drawn_valid     = 0U;
    s_hmi_values[i].dirty           = 0U;
  }

  /* LCD 恢复仅重建显示缓存，不初始化或清空生产日志。 */

#if DISPLAY_USE_LVGL_BACKEND
  if (Display_LvglInit(Px4Lite_PlatformGetMs()) != DISPLAY_OK) {
    s_display_ready       = 0U;
    s_display_initialized = 0U;
    return DISPLAY_ERROR;
  }

  s_display_ready         = 1U;
  s_display_initialized   = 1U;
  s_next_init_attempt_ms  = 0U;
  s_current_page          = DISPLAY_HMI_PAGE_SELF_CHECK;
  Display_InitSelfCheckValues();
#ifdef DEBUG_ENABLE
  Display_LoadMockValues();
#endif
  return DISPLAY_OK;
#endif
}

/*
 * 执行显示模块自检并返回错误码。
 */
Display_Result_t Display_SelfCheck(uint16_t *error_code)
{
  return Display_LvglSelfCheck(error_code);
}

void Display_RequestRecover(void)
{
  Display_LvglRequestRecover();
  s_recover_requested = 1U;
}

/*
 * 将 Framework 模块状态映射为页面状态灯数值。
 */
static uint16_t Display_MapStateValue(Px4Lite_State_t state)
{
  switch (state) {
    case PX4LITE_STATE_STARTING: return 1U;
    case PX4LITE_STATE_ONLINE: return 2U;
    case PX4LITE_STATE_OFFLINE: return 3U;
    case PX4LITE_STATE_DEGRADED: return 4U;
    case PX4LITE_STATE_FAILED: return 5U;
    case PX4LITE_STATE_DISABLED: return 6U;
    default: return 0U;
  }
}

/*
 * 存储状态保留启动、降级、离线、失败和禁用。
 */
static uint16_t Display_MapStorageStateValue(Px4Lite_State_t state)
{
  return Display_MapStateValue(state);
}

/*
 * 通信状态只表达本机模块事实，ONLINE 不代表对端连接。
 */
static uint16_t Display_MapCommStateValue(Px4Lite_State_t state)
{
  return Display_MapStateValue(state);
}

/*
 * 由北向/东向速度分量算地速大小(cm/s)：√(vN²+vE²)。
 * 此前用 max(|vN|,|vE|) 近似，斜向运动会偏小(45° 时约少 29%)，这里改为真正的矢量模。
 */
static uint16_t Display_GroundSpeedCms(int32_t north_cms, int32_t east_cms)
{
  float n   = (float)north_cms;
  float e   = (float)east_cms;
  float mag = sqrtf((n * n) + (e * e));

  if (mag > 65535.0f) { return 65535U; }
  return (uint16_t)(mag + 0.5f);
}

static uint32_t Display_TimeSecondsToHhmmss(uint32_t seconds)
{
  seconds %= 86400UL;
  return ((seconds / 3600UL) * 10000UL) + (((seconds / 60UL) % 60UL) * 100UL) + (seconds % 60UL);
}

static uint32_t Display_GnssUtcSecondsToLocalHhmmss(uint32_t utc_seconds)
{
  return Display_TimeSecondsToHhmmss(utc_seconds + PX4LITE_TIME_LOCAL_OFFSET_S);
}

static uint8_t Display_IsLeapYear(uint32_t year)
{
  if ((year % 400UL) == 0UL) { return 1U; }
  if ((year % 100UL) == 0UL) { return 0U; }
  return ((year % 4UL) == 0UL) ? 1U : 0U;
}

static uint8_t Display_DaysInMonth(uint32_t year, uint32_t month)
{
  static const uint8_t days[12] = {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};

  if ((month < 1UL) || (month > 12UL)) { return 31U; }
  if ((month == 2UL) && (Display_IsLeapYear(year) != 0U)) { return 29U; }
  return days[month - 1UL];
}

static uint32_t Display_GnssDateToLocalYmd(uint32_t utc_date, uint32_t utc_seconds)
{
  uint32_t year;
  uint32_t month;
  uint32_t day;

  if ((utc_date == 0U) || (utc_seconds >= 86400UL)) { return 0U; }

  year  = 2000UL + (utc_date / 10000UL);
  month = (utc_date / 100UL) % 100UL;
  day   = utc_date % 100UL;
  if ((year < 2000UL) || (year > 2099UL) || (month < 1UL) || (month > 12UL) || (day < 1UL) || (day > Display_DaysInMonth(year, month))) { return 0U; }

  if ((utc_seconds + PX4LITE_TIME_LOCAL_OFFSET_S) >= 86400UL) {
    day++;
    if (day > Display_DaysInMonth(year, month)) {
      day = 1UL;
      month++;
      if (month > 12UL) {
        month = 1UL;
        year++;
      }
    }
  }

  return (year * 10000UL) + (month * 100UL) + day;
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
  if (pressure_pa <= 0.0f) { return 0U; }
  if (pressure_pa > 4294967040.0f) { return 0xFFFFFFFFUL; }

  return (uint32_t)(pressure_pa + 0.5f);
}

/** @brief 撤销过期或缺失的显示字段，禁止以有效数值零代替未知。 */
static void Display_ClearNavigationSnapshot(void)
{
  Display_InvalidateValue(DISPLAY_HMI_VAR_GNSS_FIX);
  Display_InvalidateValue(DISPLAY_HMI_VAR_GNSS_SAT_COUNT);
  Display_InvalidateValue(DISPLAY_HMI_VAR_GNSS_HDOP);
  Display_InvalidateValue(DISPLAY_HMI_VAR_LATITUDE);
  Display_InvalidateValue(DISPLAY_HMI_VAR_LONGITUDE);
  Display_InvalidateValue(DISPLAY_HMI_VAR_ALTITUDE);
  Display_InvalidateValue(DISPLAY_HMI_VAR_GNSS_SPEED);
  Display_InvalidateValue(DISPLAY_HMI_VAR_ROLL);
  Display_InvalidateValue(DISPLAY_HMI_VAR_PITCH);
  Display_InvalidateValue(DISPLAY_HMI_VAR_YAW);
}

/** @brief 撤销过期或缺失的显示字段，禁止以有效数值零代替未知。 */
static void Display_ClearAttitudeFields(void)
{
  Display_InvalidateValue(DISPLAY_HMI_VAR_ROLL);
  Display_InvalidateValue(DISPLAY_HMI_VAR_PITCH);
  Display_InvalidateValue(DISPLAY_HMI_VAR_YAW);
}

/** @brief 撤销过期或缺失的显示字段，禁止以有效数值零代替未知。 */
static void Display_ClearEnvironmentFields(void)
{
  Display_InvalidateValue(DISPLAY_HMI_VAR_TEMPERATURE);
  Display_InvalidateValue(DISPLAY_HMI_VAR_HUMIDITY);
  Display_InvalidateValue(DISPLAY_HMI_VAR_PRESSURE);
}

/* 电机电池(ADC2)门限(单位 mV)：
 *   <9.0V  没电：电量/电压显示 0、电机停机、锁 PWM；
 *   9.0~9.9V 供电不足：电机仍不能启动(停机/锁 PWM)，消息+告警提示供电不足；
 *   9.9~10.5V 低电：电机可运行，提示充电；
 *   >=10.5V 正常。 */
#define DISPLAY_MOTOR_BAT_CONNECT_MV 5000U  /* 在位门限：<5V 视为未插电池(电机断开)，电量/电压显示 0 */
#define DISPLAY_MOTOR_BAT_DEAD_MV    9000U  /* 没电门限：已插电池但 <9.0V */
#define DISPLAY_MOTOR_BAT_RUN_MV     9900U  /* 电机可启动门限：<9.9V 停机并锁 PWM */
#define DISPLAY_MOTOR_BAT_OK_MV      10500U /* 充电提示门限：<10.5V 提示充电 */
/* 在位上限：3S 锂电满电也就 ~12.6V，>13.5V 不可能是真实电池，判为引脚悬空/未插，电压电量显示 0。
   解决上电后 ADC2 悬空被采样电容灌电荷顶出十几伏虚高的问题。 */
#define DISPLAY_MOTOR_BAT_MAX_MV     13500U
/* 主控电池(ADC1)门限：在位门限(显示 0)5.0V；低于 10.5V 提示充电。 */
#define DISPLAY_MAIN_BAT_PRESENT_MV  5000U
#define DISPLAY_MAIN_BAT_CHARGE_MV   10500U

/* 档位滞回余量：进入某档后，电压需反向越过阈值此值才换档，吸收 ~0.1V 噪声防横跳。 */
#define DISPLAY_BAT_HYST_MV          200U
/* 电机电池档位 */
#define DISPLAY_MOTOR_BAND_DISCONNECT 0U /* <5.0V 未插电池 */
#define DISPLAY_MOTOR_BAND_DEAD       1U /* 5.0~9.0V 没电 */
#define DISPLAY_MOTOR_BAND_LOWPOWER   2U /* 9.0~9.9V 供电不足 */
#define DISPLAY_MOTOR_BAND_CHARGE     3U /* 9.9~10.5V 需充电 */
#define DISPLAY_MOTOR_BAND_OK         4U /* >=10.5V 正常 */
/* 主控电池档位 */
#define DISPLAY_MAIN_BAND_ABSENT      0U /* <5.0V 未接入 */
#define DISPLAY_MAIN_BAND_CHARGE      1U /* 5.0~10.5V 需充电 */
#define DISPLAY_MAIN_BAND_OK          2U /* >=10.5V 正常 */

/*
 * 依据电机电池档位(带滞回)决定电机自检灯颜色：
 * <9.9V(断开/没电/供电不足) 红灯(3)，9.9~10.5V(需充电) 黄灯(1)，>=10.5V 绿灯(2)。
 */
static uint16_t Display_MotorSelfCheckValue(void)
{
  if (s_motor_band <= DISPLAY_MOTOR_BAND_LOWPOWER) { return 3U; }
  if (s_motor_band == DISPLAY_MOTOR_BAND_CHARGE) { return 1U; }
  return 2U;
}



/* 4 个电机自检灯共用同一块电机电池(ADC2)状态，统一刷新。 */
static void Display_SetMotorSelfCheckLights(uint16_t value)
{
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR, value);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2, value);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3, value);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4, value);
}

/** @brief 撤销过期或缺失的显示字段，禁止以有效数值零代替未知。 */
static void Display_ClearBatteryFields(void)
{
  Display_InvalidateValue(DISPLAY_HMI_VAR_BATTERY_VOLTAGE);
  Display_InvalidateValue(DISPLAY_HMI_VAR_BATTERY_CURRENT);
  Display_InvalidateValue(DISPLAY_HMI_VAR_BATTERY_PERCENT);
  Display_InvalidateValue(DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE);
  Display_InvalidateValue(DISPLAY_HMI_VAR_MOTOR_BAT_CURRENT);
  Display_InvalidateValue(DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT);
  /* 复位电压去抖状态，重新接入/上线时按首帧重新播种。 */
  s_disp_main_dv_valid  = 0U;
  s_disp_motor_dv_valid = 0U;
  /* 没有电压事实时状态未知，不伪造断开故障。 */
  Display_InvalidateValue(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR);
  Display_InvalidateValue(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2);
  Display_InvalidateValue(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3);
  Display_InvalidateValue(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4);
}

/** @brief 已确认急停后的受控清零；数据不可用时禁止调用。 */
static void Display_ClearMotorOutputFields(void)
{
  uint8_t i;

  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_1, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_2, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_3, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_4, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_TARGET_1, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_TARGET_2, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_TARGET_3, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_TARGET_4, 0U);
#if DISPLAY_USE_LVGL_BACKEND
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    (void)Display_LvglSetMotorPulseUs(i, PX4LITE_CONTROL_ESC_MIN_PULSE_US);
  }
#else
  (void)i;
#endif
}

/** @brief 数据源无有效电机快照时撤销所有控制反馈；不能把未知输出伪造为零。 */
static void Display_ClearMotorFields(void)
{
  uint8_t i;
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; ++i) {
    Display_InvalidateValue((Display_HmiVariableId_t)(DISPLAY_HMI_VAR_MOTOR_PWM_1 + i));
    Display_InvalidateValue((Display_HmiVariableId_t)(DISPLAY_HMI_VAR_MOTOR_TARGET_1 + i));
#if DISPLAY_USE_LVGL_BACKEND
    (void)Display_LvglInvalidateMotorPulseUs(i);
#endif
  }
  Display_InvalidateValue(DISPLAY_HMI_VAR_MOTOR_CONTROL_MODE);
  Display_InvalidateValue(DISPLAY_HMI_VAR_MOTOR_OPERATION_STATE);
}

/**
 * @brief 分别发布学生设定目标、含姿态修正的输出、PWM、模式和过程状态。
 *
 * @details 滑轨跟随 base_percent；输出百分比与真实 PWM 另列显示，使姿态辅助差值可见，
 * 同时避免输出修正被再次提交为目标。
 */
static void Display_LoadMotorSnapshot(const App_MotorSnapshot_t *motor)
{
  if (motor == 0) { return; }

  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_1, motor->duty_percent[0]);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_2, motor->duty_percent[1]);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_3, motor->duty_percent[2]);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_4, motor->duty_percent[3]);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_TARGET_1, motor->base_percent[0]);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_TARGET_2, motor->base_percent[1]);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_TARGET_3, motor->base_percent[2]);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_TARGET_4, motor->base_percent[3]);
  if (motor->control_mode_valid != 0U) { (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_CONTROL_MODE, motor->control_mode); }
  else { Display_InvalidateValue(DISPLAY_HMI_VAR_MOTOR_CONTROL_MODE); }
  if (motor->operation_state_valid != 0U) { (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_OPERATION_STATE, motor->operation_state); }
  else { Display_InvalidateValue(DISPLAY_HMI_VAR_MOTOR_OPERATION_STATE); }
}

/** @brief 发布姿态操作阶段到显示缓存；远端页面用 255 表示本机操作不可用。 */
static void Display_LoadAttitudeOperationStatus(Px4Lite_RemoteMode_t display_mode)
{
  App_AttitudeStatus_t status;

  if ((display_mode == PX4LITE_REMOTE_MODE_REMOTE) ||
      (App_CopyAttitudeStatus(&status) != PX4LITE_OK)) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_ATTITUDE_ACTION, 0U);
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_ATTITUDE_PHASE, 255U);
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_ATTITUDE_REASON, 0U);
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_ATTITUDE_PROGRESS, 0U);
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_ATTITUDE_REFERENCE, 0U);
    return;
  }
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_ATTITUDE_ACTION, status.action);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_ATTITUDE_PHASE, status.phase);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_ATTITUDE_REASON, status.reason);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_ATTITUDE_PROGRESS, status.progress);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_ATTITUDE_REFERENCE, status.reference_active);
}

/**
 * @brief 把 Control 最终写入的四路真实 PWM 脉宽刷新到电机页。
 *
 * @details 该路径与整数百分比滑条分离，避免小幅姿态修正被百分比取整后隐藏。
 */
static void Display_LoadMotorPulseSnapshot(const App_MotorSnapshot_t *motor)
{
#if DISPLAY_USE_LVGL_BACKEND
  uint8_t i;

  if (motor == 0) { return; }
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    (void)Display_LvglSetMotorPulseUs(i, motor->pulse_us[i]);
  }
#else
  (void)motor;
#endif
}

/** @brief 按 POSITION/ALTITUDE/VELOCITY/ATTITUDE 独立有效位更新导航显示。 */
static void Display_LoadNavigationSnapshot(const App_NavigationSnapshot_t *navigation)
{
  if (navigation == 0) { Display_ClearNavigationSnapshot(); return; }
  if ((navigation->valid_mask & PX4LITE_NAV_VALID_POSITION) != 0U) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_FIX, navigation->gnss_fix_type);
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_SAT_COUNT, navigation->satellites_used);
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_HDOP, navigation->hdop_x100);
    (void)Display_SetHmiValueI32(DISPLAY_HMI_VAR_LATITUDE, navigation->latitude_e7);
    (void)Display_SetHmiValueI32(DISPLAY_HMI_VAR_LONGITUDE, navigation->longitude_e7);
  } else {
    Display_InvalidateValue(DISPLAY_HMI_VAR_GNSS_FIX);
    Display_InvalidateValue(DISPLAY_HMI_VAR_GNSS_SAT_COUNT);
    Display_InvalidateValue(DISPLAY_HMI_VAR_GNSS_HDOP);
    Display_InvalidateValue(DISPLAY_HMI_VAR_LATITUDE);
    Display_InvalidateValue(DISPLAY_HMI_VAR_LONGITUDE);
  }
  if ((navigation->valid_mask & PX4LITE_NAV_VALID_ALTITUDE) != 0U) {
    (void)Display_SetHmiValueI32(DISPLAY_HMI_VAR_ALTITUDE, navigation->altitude_mm);
  } else { Display_InvalidateValue(DISPLAY_HMI_VAR_ALTITUDE); }
  if ((navigation->valid_mask & PX4LITE_NAV_VALID_VELOCITY) != 0U) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_SPEED,
        Display_GroundSpeedCms(navigation->velocity_north_cms, navigation->velocity_east_cms));
  } else { Display_InvalidateValue(DISPLAY_HMI_VAR_GNSS_SPEED); }
  if ((navigation->valid_mask & PX4LITE_NAV_VALID_ATTITUDE) != 0U) {
    (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_ROLL, (int16_t)(navigation->roll_deg100 / 10));
    (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_PITCH, (int16_t)(navigation->pitch_deg100 / 10));
    (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_YAW, (int16_t)(navigation->yaw_deg100 / 10));
  } else { Display_ClearAttitudeFields(); }
}

/*
 * 将统一日期时间快照写入 Display 缓存。
 */
static void Display_LoadDateTimeSnapshot(const App_DateTimeSnapshot_t *date_time)
{
  if (date_time == 0) { return; }

  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_GNSS_TIME, date_time->local_time_hhmmss);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_CLOCK_TIME, date_time->local_time_hhmmss);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_DATE, date_time->local_date_ymd);
}

/**
 * @brief 本机统一时间暂不可用时，仅使用独立有效的本机导航 UTC；均无效时撤销时钟。
 *
 * @details
 * 本函数不得读取当前显示模式对应的导航快照，避免切换远端节点时把远端 GNSS 时间
 * 写入顶栏，或因远端模式改用不同兜底源而造成日期时间跳变。
 */
static void Display_LoadFallbackDateTime(uint32_t now_ms)
{
  App_NavigationSnapshot_t local_navigation;
  const App_NavigationSnapshot_t *navigation = 0;
  uint32_t fallback_time;
  uint32_t fallback_date;

  if ((App_CopyNavigation(&local_navigation, now_ms) == PX4LITE_OK) &&
      ((local_navigation.valid_mask & PX4LITE_NAV_VALID_TIME) != 0U) &&
      (local_navigation.gnss_utc_sec < 86400UL)) { navigation = &local_navigation; }
  if (navigation == 0) {
    Display_InvalidateValue(DISPLAY_HMI_VAR_GNSS_TIME);
    Display_InvalidateValue(DISPLAY_HMI_VAR_CLOCK_TIME);
    Display_InvalidateValue(DISPLAY_HMI_VAR_DATE);
    return;
  }
  fallback_time = Display_GnssUtcSecondsToLocalHhmmss(navigation->gnss_utc_sec);
  fallback_date = Display_GnssDateToLocalYmd(navigation->gnss_utc_date, navigation->gnss_utc_sec);

  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_GNSS_TIME, fallback_time);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_CLOCK_TIME, fallback_time);
  if (fallback_date != 0U) { (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_DATE, fallback_date); }
  else { Display_InvalidateValue(DISPLAY_HMI_VAR_DATE); }
}

/*
 * 将系统状态快照字段写入 Display 缓存。
 */
static void Display_LoadSystemSnapshot(const App_SystemSnapshot_t *system)
{
  uint32_t alarm_row;

  if (system == 0) { return; }

  alarm_row = (system->highest_fault_code != 0U) ? (((uint32_t)system->highest_source_id << 16) | system->highest_fault_code) : 0U;

  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SYSTEM_STATUS, system->system_ready ? 2U : 1U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_DATA_SELFCHECK_RESULT, system->status_version);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_ALARM_CODE, system->highest_fault_code);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ACTIVE_MASK, system->warning_fault_mask | system->blocking_fault_mask);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ROW1_CODE, alarm_row);

  /* 自检-定位模块仍反映 GNSS 模块在线态；"定位状态"(GNSS_FIX)改由导航快照的解算 fix 等级
     驱动(见 Display_LoadNavigationSnapshot)，这里不再用模块态覆盖。 */
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_GNSS, Display_MapStateValue(system->modules[PX4LITE_MODULE_GNSS].state));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, Display_MapStateValue(system->modules[PX4LITE_MODULE_IMU].state));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_BME280, Display_MapStateValue(system->modules[PX4LITE_MODULE_BARO].state));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_LORA, Display_MapCommStateValue(system->modules[PX4LITE_MODULE_LORA].state));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_LORA_STATUS, Display_MapCommStateValue(system->modules[PX4LITE_MODULE_LORA].state));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_REMOTEID, Display_MapCommStateValue(system->modules[PX4LITE_MODULE_REMOTE_ID].state));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_SD, Display_MapStorageStateValue(system->modules[PX4LITE_MODULE_STORAGE].state));
  /* 电机 4 个状态灯改由 Display_LoadEnvironmentSnapshot 依据电机电池(ADC2)
     电压驱动：<9.0V 红灯，9.0V~9.9V 黄灯，>=9.9V 绿灯，此处不再覆盖。 */
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_5GA, Display_MapStateValue(system->modules[PX4LITE_MODULE_5G].state));
  /* 自检页错误码表由 Display_LoadAlarmSnapshot 按激活故障列表整体刷新，
     此处不再用单个 highest_fault_code 驱动。 */
}

static void Display_ClearSystemSnapshot(void)
{
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SYSTEM_STATUS, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_DATA_SELFCHECK_RESULT, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_ALARM_CODE, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ACTIVE_MASK, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ROW1_CODE, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_GNSS, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_BME280, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_LORA, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_LORA_STATUS, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_REMOTEID, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_SD, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_5GA, 0U);
  Display_SetMotorSelfCheckLights(0U);
}

/* 由电机电池(ADC2)推导电机告警码：<9.0V 没电(电机断开告警)，9.0~9.9V 供电不足告警。
   9.9~10.5V 仅消息提示充电、不报告警。阈值与电机自检灯/消息日志一致。 */
/* 带滞回的升降档：阈值升序 t[0..n-1] 分出 n+1 档(0..n)。
   升档需越过本档上界 + 滞回余量；降档需跌破本档下界 - 滞回余量；否则保持当前档。 */
static uint8_t Display_BatBandHyst(uint8_t cur, uint32_t mv, const uint32_t *t, uint8_t n)
{
  uint8_t plain = 0U;

  while ((plain < n) && (mv >= t[plain])) { plain++; }

  if (plain > cur) {
    if (mv >= (uint32_t)(t[cur] + DISPLAY_BAT_HYST_MV)) { return plain; }
  } else if (plain < cur) {
    if ((mv + DISPLAY_BAT_HYST_MV) < t[cur - 1U]) { return plain; }
  }
  return cur;
}

/** @brief 仅对应电压有效时更新电池档位，缺测不解释为断开。 */
static void Display_UpdateBatteryBands(const App_EnvironmentSnapshot_t *environment)
{
  static const uint32_t motor_t[4] = {DISPLAY_MOTOR_BAT_CONNECT_MV, DISPLAY_MOTOR_BAT_DEAD_MV, DISPLAY_MOTOR_BAT_RUN_MV, DISPLAY_MOTOR_BAT_OK_MV};
  static const uint32_t main_t[2]  = {DISPLAY_MAIN_BAT_PRESENT_MV, DISPLAY_MAIN_BAT_CHARGE_MV};

  if (environment == 0) { return; }
  if ((environment->valid_mask & APP_ENV_VALID_VOLTAGE2) != 0U) {
    s_motor_band = Display_BatBandHyst(s_motor_band, environment->voltage2_mv, motor_t, 4U);
  /* 超过在位上限视为悬空/未插，强制断开档：电压电量归 0、电机自检灯红、锁 PWM。 */
  if (environment->voltage2_mv > DISPLAY_MOTOR_BAT_MAX_MV) { s_motor_band = DISPLAY_MOTOR_BAND_DISCONNECT; }
  }
  if ((environment->valid_mask & APP_ENV_VALID_VOLTAGE1) != 0U) {
    s_main_band = Display_BatBandHyst(s_main_band, environment->voltage_mv, main_t, 2U);
  }
}

/** @brief 仅凭有效电机电压产生电源告警，缺测交由模块状态报告。 */
static uint16_t Display_EvalMotorFault(const App_EnvironmentSnapshot_t *environment)
{
  if ((environment == 0) || ((environment->valid_mask & APP_ENV_VALID_VOLTAGE2) == 0U)) { return 0U; }
  switch (s_motor_band) {
    case DISPLAY_MOTOR_BAND_DISCONNECT:
      return (uint16_t)PX4LITE_FAULT_MOTOR_DISCONNECT;
    case DISPLAY_MOTOR_BAND_DEAD:
      return (uint16_t)PX4LITE_FAULT_MOTOR_DEAD;
    case DISPLAY_MOTOR_BAND_LOWPOWER:
      return (uint16_t)PX4LITE_FAULT_MOTOR_POWER_LOW;
    case DISPLAY_MOTOR_BAND_CHARGE:
      return (uint16_t)PX4LITE_FAULT_MOTOR_CHARGE;
    default:
      return 0U;
  }
}

/** @brief 主控电压有效且档位偏低时提示充电，其他数据不能授权此告警。 */
static uint16_t Display_EvalMainFault(const App_EnvironmentSnapshot_t *environment)
{
  if ((environment == 0) || ((environment->valid_mask & APP_ENV_VALID_VOLTAGE1) == 0U)) { return 0U; }
  return (s_main_band == DISPLAY_MAIN_BAND_CHARGE) ? (uint16_t)PX4LITE_FAULT_POWER_CHARGE : 0U;
}

/** @brief 同一份本机活动告警驱动头部、自检摘要和完整表；不虚构网络占位告警。 */
static void Display_LoadAlarmSnapshot(const App_AlarmSnapshot_t *alarm, uint16_t motor_fault, uint16_t main_fault)
{
  static const Display_HmiVariableId_t row_ids[] = {
    DISPLAY_HMI_VAR_ALARM_ROW1_CODE, DISPLAY_HMI_VAR_ALARM_ROW2_CODE,
    DISPLAY_HMI_VAR_ALARM_ROW3_CODE, DISPLAY_HMI_VAR_ALARM_ROW4_CODE, DISPLAY_HMI_VAR_ALARM_ROW5_CODE
  };
  uint32_t packed[APP_DISPLAY_ALARM_MAX];
  uint16_t count = 0U;
  uint16_t i;
  int32_t severity;
  uint32_t active_mask = 0U;
  uint32_t fingerprint = 0U;
  if (alarm == 0) { return; }
  /* 动力/主电电压提示属于对应电源来源，只补充源告警中尚未报告的同一故障。 */
  for (i = 0U; i < PX4LITE_MODULE_COUNT; i++) {
    const App_AlarmRecord_t *record = &alarm->records[i];
    if (record->active == 0U) { continue; }
    if ((record->source_id == PX4LITE_MODULE_BATTERY) && (record->fault_code == motor_fault)) { motor_fault = 0U; }
    if ((record->source_id == PX4LITE_MODULE_BATTERY) && (record->fault_code == main_fault)) { main_fault = 0U; }
  }
  for (severity = PX4LITE_ALARM_FATAL; severity >= PX4LITE_ALARM_INFO; severity--) {
    for (i = 0U; (i < PX4LITE_MODULE_COUNT) && (count < APP_DISPLAY_ALARM_MAX); i++) {
      const App_AlarmRecord_t *record = &alarm->records[i];
      if ((record->active != 0U) && (record->fault_code != 0U) && ((int32_t)record->severity == severity)) {
        packed[count++] = ((uint32_t)record->source_id << 16) | record->fault_code;
      }
    }
    if (severity == PX4LITE_ALARM_WARNING) {
      if ((motor_fault != 0U) && (count < APP_DISPLAY_ALARM_MAX)) {
        packed[count++] = ((uint32_t)PX4LITE_MODULE_BATTERY << 16) | motor_fault;
      }
      if ((main_fault != 0U) && (count < APP_DISPLAY_ALARM_MAX)) {
        packed[count++] = ((uint32_t)PX4LITE_MODULE_BATTERY << 16) | main_fault;
      }
    }
  }
  for (i = 0U; i < count; i++) {
    uint16_t source = (uint16_t)(packed[i] >> 16);
    if (source < 32U) { active_mask |= (1UL << source); }
    fingerprint = fingerprint * 31U + packed[i];
  }
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_ALARM_CODE, (count != 0U) ? (uint16_t)packed[0] : 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ACTIVE_MASK, active_mask);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_ERROR_CODE,
                             (uint16_t)((fingerprint ^ (fingerprint >> 16)) + count));
  for (i = 0U; i < DISPLAY_ARRAY_SIZE(row_ids); i++) {
    (void)Display_SetHmiValueU32(row_ids[i], (i < count) ? packed[i] : 0U);
  }
  Display_LvglSetAlarmSnapshot(alarm, packed, count);
}

/** @brief 撤下不可确认的旧告警行；摘要和详情同时标为不可用，不能短暂显示零告警。 */
static void Display_ClearAlarmSnapshot(void)
{
  uint8_t i;
  Display_InvalidateValue(DISPLAY_HMI_VAR_ALARM_CODE);
  Display_InvalidateValue(DISPLAY_HMI_VAR_ALARM_ACTIVE_MASK);
  Display_InvalidateValue(DISPLAY_HMI_VAR_SELF_CHECK_ERROR_CODE);
  for (i = 0U; i < 5U; ++i) {
    Display_InvalidateValue((Display_HmiVariableId_t)(DISPLAY_HMI_VAR_ALARM_ROW1_CODE + i));
  }
  Display_LvglSetAlarmSnapshot(0, 0, 0U);
}

static uint8_t Display_LoadModuleStatusFallback(void)
{
  Px4Lite_ModuleStatus_t status;
  uint8_t loaded = 0U;

  if (App_GetModuleStatus(PX4LITE_MODULE_GNSS, &status) == PX4LITE_OK) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_GNSS, Display_MapStateValue(status.state));
    /* 定位状态(GNSS_FIX)由导航快照的解算 fix 等级驱动，兜底路径不再用模块态覆盖。 */
    loaded = 1U;
  }
  if (App_GetModuleStatus(PX4LITE_MODULE_IMU, &status) == PX4LITE_OK) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, Display_MapStateValue(status.state));
    loaded = 1U;
  }
  if (App_GetModuleStatus(PX4LITE_MODULE_BARO, &status) == PX4LITE_OK) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_BME280, Display_MapStateValue(status.state));
    loaded = 1U;
  }
  if (App_GetModuleStatus(PX4LITE_MODULE_LORA, &status) == PX4LITE_OK) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_LORA, Display_MapCommStateValue(status.state));
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_LORA_STATUS, Display_MapCommStateValue(status.state));
    loaded = 1U;
  }
  if (App_GetModuleStatus(PX4LITE_MODULE_5G, &status) == PX4LITE_OK) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_5GA, Display_MapStateValue(status.state));
    loaded = 1U;
  }
  if (App_GetModuleStatus(PX4LITE_MODULE_REMOTE_ID, &status) == PX4LITE_OK) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_REMOTEID, Display_MapCommStateValue(status.state));
    loaded = 1U;
  }
  if (App_GetModuleStatus(PX4LITE_MODULE_STORAGE, &status) == PX4LITE_OK) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_SD, Display_MapStorageStateValue(status.state));
    loaded = 1U;
  }

  return loaded;
}

/* 屏幕电压去抖：把 mV 量化到 0.1V 并加滞回，保证显示末位不在相邻 0.1V 档间抖动。
   仅作用于屏幕显示，不改动 sensor 层 4.1 算法与上报快照。 */
#define DISPLAY_VOLT_STEP_MV 100U /* 显示分辨率 0.1V */
#define DISPLAY_VOLT_HYST_MV 40U  /* 滞回余量：需越过半档(0.05V)再 +0.04V 才换档，吸收约 0.09V 噪声 */
#define DISPLAY_VOLT_JUMP_MV 300U /* 突变 >0.3V(插拔/真实变化) 立即跟随，不被滞回拖慢 */

/* 返回去抖后电压，单位 0.01V(供显示层渲染为一位小数)。present=0 归零并复位去抖状态。 */
static uint16_t Display_StabilizeVoltCv(uint32_t mv, uint8_t present, uint16_t *disp_dv, uint8_t *valid)
{
  uint32_t center_mv;

  if (present == 0U) { *valid = 0U; *disp_dv = 0U; return 0U; }

  if (*valid == 0U) {
    *disp_dv = (uint16_t)((mv + (DISPLAY_VOLT_STEP_MV / 2U)) / DISPLAY_VOLT_STEP_MV);
    *valid   = 1U;
    return (uint16_t)(*disp_dv * 10U);
  }

  center_mv = (uint32_t)(*disp_dv) * DISPLAY_VOLT_STEP_MV;
  if ((mv > (center_mv + DISPLAY_VOLT_JUMP_MV)) || ((mv + DISPLAY_VOLT_JUMP_MV) < center_mv)) {
    *disp_dv = (uint16_t)((mv + (DISPLAY_VOLT_STEP_MV / 2U)) / DISPLAY_VOLT_STEP_MV);
  } else if (mv >= (center_mv + (DISPLAY_VOLT_STEP_MV / 2U) + DISPLAY_VOLT_HYST_MV)) {
    (*disp_dv)++;
  } else if ((mv + (DISPLAY_VOLT_STEP_MV / 2U) + DISPLAY_VOLT_HYST_MV) <= center_mv) {
    (*disp_dv)--;
  }

  return (uint16_t)(*disp_dv * 10U);
}

/* 电流 mA 取绝对值换算为 0.1A 单位（显示层保留一位小数）；未接入时上层传 0。 */
static uint16_t Display_CurrentMaToDeciA(int32_t current_ma)
{
  uint32_t abs_ma = (current_ma < 0) ? (uint32_t)(-current_ma) : (uint32_t)current_ma;
  uint32_t deci_a = (abs_ma + 50U) / 100U;
  return (deci_a > 0xFFFFU) ? (uint16_t)0xFFFFU : (uint16_t)deci_a;
}

/** @brief 每个环境/电源字段仅由自己的有效位授权，部分快照不能填补其他域。 */
static void Display_LoadEnvironmentSnapshot(const App_EnvironmentSnapshot_t *environment)
{
  uint32_t valid;
  if (environment == 0) { Display_ClearEnvironmentFields(); Display_ClearBatteryFields(); return; }
  valid = environment->valid_mask;
  if ((valid & APP_ENV_VALID_PRESSURE_TEMPERATURE) != 0U) {
    (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_TEMPERATURE, Display_FloatToI16Tenths(environment->temperature_c));
    (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_PRESSURE, Display_FloatPaToU32(environment->pressure_pa));
  } else {
    Display_InvalidateValue(DISPLAY_HMI_VAR_TEMPERATURE);
    Display_InvalidateValue(DISPLAY_HMI_VAR_PRESSURE);
  }
  if ((valid & APP_ENV_VALID_HUMIDITY) != 0U) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_HUMIDITY, Display_FloatToU16Tenths(environment->relative_humidity_pct));
  } else { Display_InvalidateValue(DISPLAY_HMI_VAR_HUMIDITY); }
  if ((valid & APP_ENV_VALID_VOLTAGE1) != 0U) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_BATTERY_VOLTAGE, Display_StabilizeVoltCv(environment->voltage_mv,
        (s_main_band != DISPLAY_MAIN_BAND_ABSENT) ? 1U : 0U, &s_disp_main_dv, &s_disp_main_dv_valid));
  } else { Display_InvalidateValue(DISPLAY_HMI_VAR_BATTERY_VOLTAGE); s_disp_main_dv_valid = 0U; }
  if ((valid & APP_ENV_VALID_CURRENT1) != 0U) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_BATTERY_CURRENT, Display_CurrentMaToDeciA(environment->current_ma));
  } else { Display_InvalidateValue(DISPLAY_HMI_VAR_BATTERY_CURRENT); }
  if ((valid & APP_ENV_VALID_PERCENT1) != 0U) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_BATTERY_PERCENT, environment->battery_percent);
  } else { Display_InvalidateValue(DISPLAY_HMI_VAR_BATTERY_PERCENT); }
  if ((valid & APP_ENV_VALID_VOLTAGE2) != 0U) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE, Display_StabilizeVoltCv(environment->voltage2_mv,
        (s_motor_band != DISPLAY_MOTOR_BAND_DISCONNECT) ? 1U : 0U, &s_disp_motor_dv, &s_disp_motor_dv_valid));
    Display_SetMotorSelfCheckLights(Display_MotorSelfCheckValue());
  } else {
    Display_InvalidateValue(DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE); s_disp_motor_dv_valid = 0U;
    Display_InvalidateValue(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR);
    Display_InvalidateValue(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2);
    Display_InvalidateValue(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3);
    Display_InvalidateValue(DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4);
  }
  if ((valid & APP_ENV_VALID_CURRENT2) != 0U) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_BAT_CURRENT, Display_CurrentMaToDeciA(environment->current2_ma));
  } else { Display_InvalidateValue(DISPLAY_HMI_VAR_MOTOR_BAT_CURRENT); }
  if ((valid & APP_ENV_VALID_PERCENT2) != 0U) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT, environment->battery2_percent);
  } else { Display_InvalidateValue(DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT); }
}

/*
 * 将 LoRa 通信统计(收发帧计数)写入 Display 缓存。
 * 计数为累计值，始终可读，无需新鲜度判定。
 */
static void Display_LoadLoraStats(void)
{
  App_DisplayLinkStatus_t link;

  App_GetDisplayLinkStatus(&link, 0U);

  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_LORA_TX_COUNT, link.tx_frame_count);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_LORA_RX_COUNT, link.rx_frame_count);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_LORA_LOSS_RATE, link.loss_permille);
}

/*
 * 从应用只读快照准备一版完整显示缓存。
 */
Display_Result_t Display_PrepareSnapshot(uint32_t now_ms)
{
  App_NavigationSnapshot_t navigation;
  App_SystemSnapshot_t system;
  App_EnvironmentSnapshot_t environment;
  App_AlarmSnapshot_t alarm;
  App_DateTimeSnapshot_t date_time;
  App_MotorSnapshot_t motor;
  /* 显示任务独占，避免 32 条事件历史额外占用调用栈。 */
  static App_DisplayLogSnapshot_t log;
  Px4Lite_Result_t navigation_result;
  Px4Lite_Result_t system_result;
  Px4Lite_Result_t environment_result;
  Px4Lite_Result_t alarm_result;
  Px4Lite_Result_t date_time_result;
  Px4Lite_Result_t motor_result;
  Px4Lite_Result_t log_result;
  uint16_t motor_fault = 0U;
  uint16_t main_fault = 0U;
  uint8_t status_fallback_loaded = 0U;
  uint8_t frozen;

  if (Display_EnsureInit() != DISPLAY_OK) { return DISPLAY_NOT_READY; }
  frozen = Display_LvglObservationFrozen();
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_VIEW_NODE_ID, App_GetLocalNodeId());
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_UPTIME_MS, now_ms);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_FLIGHT_TIME_S, now_ms / 1000U);

  navigation_result = App_GetDisplayNavigation(&navigation, now_ms);
  if (frozen == 0U) {
    if (navigation_result == PX4LITE_OK) { Display_LoadNavigationSnapshot(&navigation); }
    else { Display_ClearNavigationSnapshot(); }
  }
  Display_LoadAttitudeOperationStatus(PX4LITE_REMOTE_MODE_LOCAL);

  date_time_result = App_GetDisplayDateTime(&date_time, now_ms);
  if ((date_time_result == PX4LITE_OK) && (date_time.local_date_ymd != 0U)) { Display_LoadDateTimeSnapshot(&date_time); }
  else { Display_LoadFallbackDateTime(now_ms); }

  system_result = App_GetDisplaySystem(&system, now_ms);
  if (system_result == PX4LITE_OK) { Display_LoadSystemSnapshot(&system); }
  else {
    Display_ClearSystemSnapshot();
    status_fallback_loaded = Display_LoadModuleStatusFallback();
  }

  /* 冻结不暂停电源事实、告警与控制反馈，只有测量观察字段保持。 */
  environment_result = App_GetDisplayEnvironment(&environment, now_ms);
  if (environment_result == PX4LITE_OK) {
    Display_UpdateBatteryBands(&environment);
    motor_fault = Display_EvalMotorFault(&environment);
    main_fault = Display_EvalMainFault(&environment);
  }
  alarm_result = App_GetDisplayAlarm(&alarm, now_ms);
  if (alarm_result == PX4LITE_OK) {
    Display_LoadAlarmSnapshot(&alarm, motor_fault, main_fault);
  } else {
    Display_ClearAlarmSnapshot();
  }

  log_result = App_GetDisplayMessageLog(&log, now_ms);
  if ((log_result == PX4LITE_OK) || (log_result == PX4LITE_NOT_READY)) {
    Display_PagesSetLogSnapshot(&log);
    Display_LvglSetLogSnapshotVersion(log.version);
  }
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_MESSAGE_LOG, Display_PagesGetLogVersion());

  if (frozen == 0U) {
    if (environment_result == PX4LITE_OK) { Display_LoadEnvironmentSnapshot(&environment); }
    else { Display_ClearEnvironmentFields(); Display_ClearBatteryFields(); }
  }
  motor_result = App_GetDisplayMotor(&motor, now_ms);
  if (motor_result == PX4LITE_OK) {
    Display_LoadMotorSnapshot(&motor);
    Display_LoadMotorPulseSnapshot(&motor);
  } else { Display_ClearMotorFields(); }
  Display_LoadLoraStats();
  if ((navigation_result != PX4LITE_OK) && (system_result != PX4LITE_OK) &&
      (environment_result != PX4LITE_OK) && (status_fallback_loaded == 0U)) { return DISPLAY_NOT_READY; }
  return DISPLAY_OK;
}

/*
 * 执行一次有预算约束的显示刷新步骤。
 */
Display_Result_t Display_RefreshStep(uint32_t now_ms, uint32_t budget_us)
{
  if (Display_EnsureInit() != DISPLAY_OK) { return DISPLAY_NOT_READY; }
  return Display_LvglRefreshStep(now_ms, budget_us);
}

/*
 * 切换当前 HMI 页面并重绘页面静态内容。
 */
Display_Result_t Display_SetHmiPage(Display_HmiPage_t page)
{
  Display_Result_t result;

  result = Display_LvglSetPage(page);
  if (result != DISPLAY_OK) { return result; }
  s_current_page = page;
  return DISPLAY_OK;
}

/*
 * 获取当前 HMI 页面 ID。
 */
Display_HmiPage_t Display_GetCurrentHmiPage(void)
{
  return s_current_page;
}

/*
 * 是否有待处理的整页静态重绘(切页触发)，供显示任务在 200ms 常规节拍之外
 * 立即触发一次刷新，缩短切页响应延迟。
 */
uint8_t Display_HasPendingRedraw(void)
{
  return Display_LvglNeedsRefresh();
}

void Display_ShowBootCode(uint8_t code)
{
  (void)code;
}

/*
 * 轮询触摸状态并处理触摸事件。
 */
/*
 * 电机油门保活重发。控制层有 200ms 指令看门狗(PX4LITE_CONTROL_FAILSAFE_TIMEOUT_MS)：
 * Control 层已经锁存最近一次本地滑条或树莓派下行目标。
 * 显示任务不再周期重发本地滑条缓存，避免旧的本地值覆盖新的下行控制值。
 */
static void Display_ServiceMotorKeepAlive(void)
{
}

Display_Result_t Display_PollTouch(void)
{
  if (s_display_ready == 0U) { return DISPLAY_NOT_READY; }
  Display_ServiceMotorKeepAlive();
  return DISPLAY_OK;
}

/*
 * 轮询 KEY0 并做简易去抖，检测到按下边沿时切换隐藏页。
 * 非隐藏页按下：记忆当前页并弹出隐藏页；隐藏页按下：返回原页面。
 */
Display_Result_t Display_PollKey(void)
{
  return (s_display_ready != 0U) ? DISPLAY_OK : DISPLAY_NOT_READY;
}

/*
 * 根据触摸坐标处理变量写入或页面导航。
 */
Display_Result_t Display_HandleTouch(uint16_t x, uint16_t y, Display_HmiVariableId_t *id, uint32_t *value)
{
  (void)x;
  (void)y;
  if (id != 0) { *id = DISPLAY_HMI_VAR_COUNT; }
  if (value != 0) { *value = 0U; }
  return (s_display_ready != 0U) ? DISPLAY_OK : DISPLAY_NOT_READY;
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

/** @brief 显示门面状态入口；不暴露 LVGL 类型，不发起触摸 I/O。 */
void Display_GetRuntimeStatus(Display_RuntimeStatus_t *out)
{
  if (out == 0) { return; }
#if DISPLAY_USE_LVGL_BACKEND
  Display_LvglGetStatus(out);
#else
  memset(out, 0, sizeof(*out));
  out->display_ready = s_display_ready;
  /* 此后端没有可确认的触摸采集路径，保持 touch_ready 为 0。 */
#endif
}
