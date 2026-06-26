/**
 * @file display.c
 * @brief Coordinate display snapshots, touch navigation, and budgeted refresh.
 */

#include "display.h"
#include "app_data_api.h"
#include "app_display_model.h"
#include "px4lite_faults.h"
#include "px4lite_platform.h"
#include "display_gfx.h"
#include "display_gt911.h"
#include "display_lvgl.h"
#include "display_pages.h"
#include "display_logo.h"
#include "display_ssd1963.h"
#include "debug_console.h"
#include <string.h>

#define DISPLAY_USE_LVGL_BACKEND 1U

#if DISPLAY_USE_LVGL_BACKEND
/* Old pixel renderer symbols remain in this file as a staged migration
 * fallback. They are intentionally unused while the LVGL backend is active. */
#pragma diag_suppress 177
#pragma diag_suppress 550
#endif

/*
 * ATK-MD0700 显示骨架。
 *
 * ATK-MD0700 使用 FSMC/FMC 16-bit 8080 并口显示。
 * 下方 var_addr 是固件内部变量地址，用于 App、Display 和调试链路路由。
 * 实际 LCD 刷新方式基于固定坐标区域，直接绘制 RGB565 像素。
 */

#define DISPLAY_ARRAY_SIZE(array) ((uint16_t)(sizeof(array) / sizeof((array)[0])))

#define DISPLAY_ATK_MD0700_WIDTH  800U
#define DISPLAY_ATK_MD0700_HEIGHT 480U
#define DISPLAY_ATK_MD0700_PID    0x61U

#define DISPLAY_HEADER_HEIGHT     64U
#define DISPLAY_FOOTER_Y          420U
#define DISPLAY_FOOTER_HEIGHT     60U
#define DISPLAY_NAV_SIDE_WIDTH    170U
/* 触摸判定区与底部可见按钮矩形一致：左=上一页，右=下一页。
 * 只命中按钮本身，避免点到中间页码圆点区域误翻页。 */
#define DISPLAY_NAV_BUTTON_X1_PREV 8U
#define DISPLAY_NAV_BUTTON_X2_PREV 258U
#define DISPLAY_NAV_BUTTON_X1_NEXT 542U
#define DISPLAY_NAV_BUTTON_X2_NEXT 792U
#define DISPLAY_NAV_BUTTON_Y1      426U
#define DISPLAY_NAV_BUTTON_Y2      474U
#define DISPLAY_MOTOR_SLIDER_MAX_VALUE 100U
#define DISPLAY_MOTOR_HANDLE_GRAB_PAD  18U
typedef struct {
  uint32_t value;           /* 当前缓存的原始显示值 */
  uint32_t drawn_value;     /* 上一次已经绘制到屏幕的值 */
  uint32_t last_refresh_ms; /* 上一次字段重绘时间 */
  uint8_t valid;            /* 0 表示缓存无效，1 表示可用于绘制 */
  uint8_t drawn_valid;      /* 0 表示当前页还未绘制过该字段 */
  uint8_t dirty;            /* 1 表示下次 Display_Refresh() 需要重绘 */
} Display_ValueCache_t;

typedef enum {
  DISPLAY_NAV_TOUCH_NONE = 0,
  DISPLAY_NAV_TOUCH_PREV,
  DISPLAY_NAV_TOUCH_NEXT,
  DISPLAY_NAV_TOUCH_TAB
} Display_NavTouch_t;

/*
 * 页面 ID 用于 App 和调试链路路由，枚举值保持稳定。
 */
static const Display_HmiPageConfig_t s_hmi_pages[] = {{DISPLAY_HMI_PAGE_SELF_CHECK, 0x0000U, "self_check", 200U}, {DISPLAY_HMI_PAGE_FLIGHT, 0x0004U, "flight", 500U}, {DISPLAY_HMI_PAGE_AIRCRAFT, 0x0005U, "aircraft", 500U}, {DISPLAY_HMI_PAGE_DATA, 0x0001U, "data", 500U}, {DISPLAY_HMI_PAGE_MOTOR, 0x0006U, "motor", 500U}, {DISPLAY_HMI_PAGE_ALARM, 0x0002U, "alarm", 200U}, {DISPLAY_HMI_PAGE_HIDDEN, 0x0007U, "hidden", 0U}};

/*
 * ATK-MD0700 800x480 显示变量配置。
 * var_addr 是固件内部变量地址，不是 LCD 控制器寄存器地址。
 */
static const Display_HmiVariableConfig_t s_hmi_variables[] = {
    /* 上电自检页 -- 9 个主要设备状态 */
    {DISPLAY_HMI_VAR_SELF_CHECK_GNSS, DISPLAY_HMI_PAGE_SELF_CHECK, 0x1009U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 70U, 140U, 16U, 16U, "self_check_gnss", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, DISPLAY_HMI_PAGE_SELF_CHECK, 0x1001U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 226U, 140U, 16U, 16U, "self_check_mpu", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_BME280, DISPLAY_HMI_PAGE_SELF_CHECK, 0x1002U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 382U, 140U, 16U, 16U, "self_check_bme", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_SELF_CHECK_LORA, DISPLAY_HMI_PAGE_SELF_CHECK, 0x100AU, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 200U, 70U, 252U, 16U, 16U, "self_check_lora", "-", "App_Registry"},
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
    /* 飞行数据页 -- 中间栏：主控/电机电池、飞行时间、温度/湿度/气压 */
    {DISPLAY_HMI_VAR_BATTERY_VOLTAGE, DISPLAY_HMI_PAGE_FLIGHT, 0x1103U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 116U, 124U, 22U, "flight_main_voltage", "V", "CNS_State.power"},
    {DISPLAY_HMI_VAR_BATTERY_PERCENT, DISPLAY_HMI_PAGE_FLIGHT, 0x1104U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 152U, 124U, 22U, "flight_main_battery", "%", "CNS_State.power"},
    {DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE, DISPLAY_HMI_PAGE_FLIGHT, 0x1105U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 188U, 124U, 22U, "flight_motor_voltage", "V", "CNS_State.power"},
    {DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT, DISPLAY_HMI_PAGE_FLIGHT, 0x1107U, DISPLAY_HMI_TYPE_U16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 224U, 124U, 22U, "flight_motor_battery", "%", "CNS_State.power"},
    {DISPLAY_HMI_VAR_MESSAGE_LOG, DISPLAY_HMI_PAGE_DATA, 0x110AU, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 540U, 76U, 252U, 330U, "msg_log_data", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_MESSAGE_LOG, DISPLAY_HMI_PAGE_FLIGHT, 0x110AU, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 540U, 76U, 252U, 330U, "msg_log_flight", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_MESSAGE_LOG, DISPLAY_HMI_PAGE_AIRCRAFT, 0x110AU, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 200U, 540U, 76U, 252U, 330U, "msg_log_aircraft", "-", "App_Registry"},
    {DISPLAY_HMI_VAR_FLIGHT_TIME_S, DISPLAY_HMI_PAGE_FLIGHT, 0x1106U, DISPLAY_HMI_TYPE_U32, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 260U, 124U, 22U, "flight_time_s", "-", "CNS_State.system"},
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
    /* 飞机情况页 -- 中间栏：横滚、俯仰、偏航、LoRa 发送/接收 */
    {DISPLAY_HMI_VAR_ROLL, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1300U, DISPLAY_HMI_TYPE_I16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 118U, 124U, 22U, "ac_roll", "\xB0", "CNS_State.sensor"},
    {DISPLAY_HMI_VAR_PITCH, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1301U, DISPLAY_HMI_TYPE_I16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 150U, 124U, 22U, "ac_pitch", "\xB0", "CNS_State.sensor"},
    {DISPLAY_HMI_VAR_YAW, DISPLAY_HMI_PAGE_AIRCRAFT, 0x1305U, DISPLAY_HMI_TYPE_I16, DISPLAY_HMI_ACCESS_RO, 1000U, 392U, 182U, 124U, 22U, "ac_yaw", "\xB0", "CNS_State.navigation"},
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
static uint8_t s_touch_ready                  = 0U;
static uint8_t s_touch_down                   = 0U;
static uint8_t s_touch_retry_count            = 0U;
static uint8_t s_touch_poll_count             = 0U;
static Display_NavTouch_t s_touch_nav_latched = DISPLAY_NAV_TOUCH_NONE;
static Display_HmiPage_t s_touch_tab_target   = DISPLAY_HMI_PAGE_SELF_CHECK;
static const Display_HmiVariableConfig_t *s_touch_slider_latched;
static uint8_t s_selected_motor_index         = 0U;
static uint8_t s_display_initialized          = 0U;
static volatile uint8_t s_recover_requested   = 0U;
static uint16_t s_refresh_cursor              = 0U;
static uint8_t s_static_redraw_pending        = 0U;
static uint8_t s_motor_bat_cutoff             = 0U; /* 电机电池电压不足：停机并锁定 PWM 滑块 */
static uint8_t s_motor_band                   = 4U; /* 电机电池档位(带滞回)，初值=正常 */
static uint8_t s_main_band                    = 2U; /* 主控电池档位(带滞回)，初值=正常 */
static Display_HmiPage_t s_page_before_hidden = DISPLAY_HMI_PAGE_SELF_CHECK; /* 进入隐藏页前的页面，供 KEY0 返回 */
static uint8_t s_key0_last_raw                = 0U;
static uint8_t s_key0_stable                  = 0U;
static uint8_t s_key0_count                   = 0U;

#define DISPLAY_KEY_DEBOUNCE_CYCLES 2U

/*
 * Check whether a point is inside an inclusive rectangle.
 */
static uint8_t Display_IsPointInBox(uint16_t x, uint16_t y, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  return (uint8_t)((x >= x1) && (x <= x2) && (y >= y1) && (y <= y2));
}

/*
 * 将图元层画点调用转接到底层 LCD 端口。 */
static void AtkMd0700_GfxDrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  Display_Ssd1963_DrawPixel(x, y, color);
}

/*
 * 将图元层矩形填充调用转接到底层 LCD 端口。
 */
static void AtkMd0700_GfxFillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
  Display_Ssd1963_FillRect(x, y, width, height, color);
}

static const Display_GfxPort_t s_atk_md0700_gfx_port = {AtkMd0700_GfxDrawPixel, AtkMd0700_GfxFillRect};

/*
 * 读取指定 HMI 变量的缓存值，缓存无效时返回默认值。
 */
static uint32_t Display_GetCachedValue(Display_HmiVariableId_t id, uint32_t default_value)
{
  if ((id < DISPLAY_HMI_VAR_COUNT) && (s_hmi_values[id].valid != 0U)) { return s_hmi_values[id].value; }

  return default_value;
}

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

/*
 * 判断触摸坐标是否落在底部翻页按钮内。
 * 触摸已是横屏显示坐标，直接与按钮矩形对应：左=上一页，右=下一页。
 */
static Display_NavTouch_t Display_CheckNavRect(uint16_t x, uint16_t y)
{
  uint16_t tab_count = (uint16_t)(DISPLAY_HMI_PAGE_HIDDEN - DISPLAY_HMI_PAGE_SELF_CHECK);
  uint16_t tab_w     = (uint16_t)(DISPLAY_ATK_MD0700_WIDTH / tab_count);
  uint16_t idx;

  /* 底部标签栏：落在底栏内即按 X 映射到对应页面，整条都是触摸热区，灵敏直达。 */
  if (y < DISPLAY_FOOTER_Y) { return DISPLAY_NAV_TOUCH_NONE; }

  idx = (uint16_t)(x / tab_w);
  if (idx >= tab_count) { idx = (uint16_t)(tab_count - 1U); }
  s_touch_tab_target = (Display_HmiPage_t)((uint16_t)DISPLAY_HMI_PAGE_SELF_CHECK + idx);
  return DISPLAY_NAV_TOUCH_TAB;
}

static uint8_t Display_IsMotorSliderId(Display_HmiVariableId_t id)
{
  return (uint8_t)((id == DISPLAY_HMI_VAR_MOTOR_PWM_1) || (id == DISPLAY_HMI_VAR_MOTOR_PWM_2) || (id == DISPLAY_HMI_VAR_MOTOR_PWM_3) || (id == DISPLAY_HMI_VAR_MOTOR_PWM_4));
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

static uint16_t Display_MotorHandleCenterY(const Display_HmiVariableConfig_t *variable, uint16_t value)
{
  uint32_t filled;
  uint16_t cy;
  uint16_t lo;
  uint16_t hi;

  if ((variable == 0) || (variable->height == 0U)) { return 0U; }
  if (value > DISPLAY_MOTOR_SLIDER_MAX_VALUE) { value = DISPLAY_MOTOR_SLIDER_MAX_VALUE; }

  filled = ((uint32_t)value * variable->height) / DISPLAY_MOTOR_SLIDER_MAX_VALUE;
  cy     = (uint16_t)(variable->y + variable->height - filled);
  lo     = (uint16_t)(variable->y + DISPLAY_MOTOR_HANDLE_HALF_H);
  hi     = (uint16_t)(variable->y + variable->height - DISPLAY_MOTOR_HANDLE_HALF_H);
  if (cy < lo) { cy = lo; }
  if (cy > hi) { cy = hi; }

  return cy;
}

static uint16_t Display_MotorSliderValueFromY(const Display_HmiVariableConfig_t *variable, uint16_t y)
{
  uint16_t bottom;
  uint32_t value;

  if ((variable == 0) || (variable->height == 0U)) { return 0U; }

  bottom = (uint16_t)(variable->y + variable->height);
  if (y >= bottom) { return 0U; }
  if (y <= variable->y) { return DISPLAY_MOTOR_SLIDER_MAX_VALUE; }

  value = (((uint32_t)(bottom - y) * DISPLAY_MOTOR_SLIDER_MAX_VALUE) + (variable->height / 2U)) / variable->height;
  if (value > DISPLAY_MOTOR_SLIDER_MAX_VALUE) { value = DISPLAY_MOTOR_SLIDER_MAX_VALUE; }
  return (uint16_t)value;
}

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

static void Display_DrawSelectedMotorTriangle(void)
{
  uint8_t i;

  if (s_current_page != DISPLAY_HMI_PAGE_MOTOR) { return; }

  for (i = 0U; i < 4U; i++) {
    const Display_HmiVariableConfig_t *variable = Display_FindMotorSliderByIndex(i);
    uint16_t cx;
    uint16_t top;
    uint16_t dy;

    if (variable == 0) { continue; }

    cx  = (uint16_t)(variable->x + (variable->width / 2U));
    top = (uint16_t)(variable->y - 10U);
    (void)Display_GfxFillRect((uint16_t)(cx - 6U), top, 13U, 8U, DISPLAY_THEME_PANEL);
    if (i != s_selected_motor_index) { continue; }

    for (dy = 0U; dy < 6U; dy++) {
      uint16_t half = (uint16_t)(5U - dy);
      (void)Display_GfxDrawHLine((uint16_t)(cx - half), (uint16_t)(top + dy), (uint16_t)((half * 2U) + 1U), DISPLAY_GFX_COLOR_BLUE);
    }
  }
}

static uint8_t Display_IsOnMotorHandle(const Display_HmiVariableConfig_t *variable, uint16_t x, uint16_t y)
{
  uint16_t cx;
  uint16_t cy;
  uint16_t hw;
  uint16_t hh;
  uint16_t x1;
  uint16_t x2;
  uint16_t y1;
  uint16_t y2;

  if (variable == 0) { return 0U; }

  cx = (uint16_t)(variable->x + (variable->width / 2U));
  cy = Display_MotorHandleCenterY(variable, (uint16_t)Display_GetCachedValue(variable->id, 0U));
  hw = (uint16_t)(DISPLAY_MOTOR_HANDLE_HALF_W + DISPLAY_MOTOR_HANDLE_GRAB_PAD);
  hh = (uint16_t)(DISPLAY_MOTOR_HANDLE_HALF_H + DISPLAY_MOTOR_HANDLE_GRAB_PAD);
  x1 = (cx > hw) ? (uint16_t)(cx - hw) : 0U;
  x2 = (uint16_t)(cx + hw);
  y1 = (cy > hh) ? (uint16_t)(cy - hh) : 0U;
  y2 = (uint16_t)(cy + hh);

  return Display_IsPointInBox(x, y, x1, y1, x2, y2);
}

static const Display_HmiVariableConfig_t *Display_FindMotorSliderAt(uint16_t x, uint16_t y)
{
  uint16_t i;

  if (s_current_page != DISPLAY_HMI_PAGE_MOTOR) { return 0; }

  for (i = 0U; i < DISPLAY_ARRAY_SIZE(s_hmi_variables); i++) {
    const Display_HmiVariableConfig_t *variable = &s_hmi_variables[i];
    if ((variable->page != s_current_page) || (Display_IsMotorSliderId(variable->id) == 0U)) { continue; }
    if (Display_IsOnMotorHandle(variable, x, y) != 0U) { return variable; }
  }

  return 0;
}

static Display_Result_t Display_DrawMotorSliderNow(const Display_HmiVariableConfig_t *variable, uint16_t throttle_percent, uint8_t draw_percent)
{
  Display_ValueCache_t *cache;

  if (variable == 0) { return DISPLAY_ERROR; }

  cache = &s_hmi_values[variable->id];
  if ((cache->drawn_valid != 0U) && (cache->drawn_value == throttle_percent) && (draw_percent == 0U)) {
    cache->last_refresh_ms = 0U;
    cache->dirty           = 0U;
    return DISPLAY_OK;
  }

  if (Display_PagesDrawMotorSliderField(variable, cache->drawn_value, throttle_percent, (cache->drawn_valid == 0U) ? 1U : 0U, draw_percent) != DISPLAY_OK) { return DISPLAY_ERROR; }
  cache->drawn_value     = throttle_percent;
  cache->drawn_valid     = 1U;
  cache->last_refresh_ms = 0U;
  cache->dirty           = 0U;

  return DISPLAY_OK;
}

static Display_Result_t Display_SetMotorThrottleCommand(const Display_HmiVariableConfig_t *variable, uint16_t throttle_percent)
{
  uint8_t motor_index;
#if !DISPLAY_USE_LVGL_BACKEND
  uint8_t selection_changed;
#endif

  if ((variable == 0) || (Display_MotorIndexFromId(variable->id, &motor_index) == 0U)) { return DISPLAY_ERROR; }

  /* 电机电池<9.0V：锁定 PWM，滑动滑块无反应（电机已由快照强制停机）。 */
  if (s_motor_bat_cutoff != 0U) { return DISPLAY_OK; }

  if (App_GetRemoteDisplayMode() == PX4LITE_REMOTE_MODE_REMOTE) { return DISPLAY_OK; }

  if (throttle_percent > DISPLAY_MOTOR_SLIDER_MAX_VALUE) { throttle_percent = DISPLAY_MOTOR_SLIDER_MAX_VALUE; }

  if (App_SetMotorThrottlePercent(motor_index, (uint8_t)throttle_percent) != PX4LITE_OK) { return DISPLAY_ERROR; }

#if !DISPLAY_USE_LVGL_BACKEND
  selection_changed      = (s_selected_motor_index != motor_index) ? 1U : 0U;
#endif
  s_selected_motor_index = motor_index;
  (void)Display_SetHmiValueU16(variable->id, throttle_percent);

#if !DISPLAY_USE_LVGL_BACKEND
  if (s_current_page == DISPLAY_HMI_PAGE_MOTOR) {
    if (selection_changed != 0U) { Display_DrawSelectedMotorTriangle(); }
    return Display_DrawMotorSliderNow(variable, throttle_percent, 1U);
  }
#endif

  return DISPLAY_OK;
}

static Display_Result_t Display_HandleMotorSliderTouch(const Display_HmiVariableConfig_t *variable, uint16_t y, Display_HmiVariableId_t *id, uint32_t *value)
{
  uint16_t throttle_percent;

  if (variable == 0) { return DISPLAY_ERROR; }

  throttle_percent = Display_MotorSliderValueFromY(variable, y);
  if (id != 0) { *id = variable->id; }
  if (value != 0) { *value = throttle_percent; }

  return Display_SetMotorThrottleCommand(variable, throttle_percent);
}

static Display_Result_t Display_MotorEmergencyStop(void)
{
  uint8_t i;

  if (App_GetRemoteDisplayMode() == PX4LITE_REMOTE_MODE_REMOTE) { return DISPLAY_OK; }

  for (i = 0U; i < 4U; i++) {
    const Display_HmiVariableConfig_t *variable = Display_FindMotorSliderByIndex(i);

    (void)App_SetMotorThrottlePercent(i, 0U);
    if (variable != 0) {
      (void)Display_SetHmiValueU16(variable->id, 0U);
#if !DISPLAY_USE_LVGL_BACKEND
      if (s_current_page == DISPLAY_HMI_PAGE_MOTOR) { (void)Display_DrawMotorSliderNow(variable, 0U, 1U); }
#endif
    }
  }

  return DISPLAY_OK;
}

Display_Result_t Display_RequestMotorThrottle(Display_HmiVariableId_t id, uint16_t throttle_percent)
{
  const Display_HmiVariableConfig_t *variable;
  uint8_t motor_index;

  if (Display_MotorIndexFromId(id, &motor_index) == 0U) { return DISPLAY_ERROR; }

  variable = Display_FindMotorSliderByIndex(motor_index);
  return Display_SetMotorThrottleCommand(variable, throttle_percent);
}

Display_Result_t Display_RequestMotorEmergencyStop(void)
{
  return Display_MotorEmergencyStop();
}

/* 电机电池<9.0V 时把四路油门强制清零(电机停机)；滑块重绘交由常规刷新处理。 */
static void Display_ForceMotorsOff(void)
{
  uint8_t i;

  for (i = 0U; i < 4U; i++) {
    const Display_HmiVariableConfig_t *variable = Display_FindMotorSliderByIndex(i);

    (void)App_SetMotorThrottlePercent(i, 0U);
    if (variable != 0) { (void)Display_SetHmiValueU16(variable->id, 0U); }
  }
}

static uint8_t Display_IsMotorEstopTouch(uint16_t x, uint16_t y)
{
  if (s_current_page != DISPLAY_HMI_PAGE_MOTOR) { return 0U; }

  return Display_IsPointInBox(x, y, DISPLAY_MOTOR_ESTOP_X, DISPLAY_MOTOR_ESTOP_Y, (uint16_t)(DISPLAY_MOTOR_ESTOP_X + DISPLAY_MOTOR_ESTOP_W), (uint16_t)(DISPLAY_MOTOR_ESTOP_Y + DISPLAY_MOTOR_ESTOP_H));
}

/*
 * 根据触摸芯片输出坐标判断导航触摸。
 */
static Display_NavTouch_t Display_GetNavTouch(uint16_t x, uint16_t y)
{
  return Display_CheckNavRect(x, y);
}

static Display_Result_t Display_HandleNavTouch(Display_NavTouch_t nav_touch)
{
  if (nav_touch == DISPLAY_NAV_TOUCH_TAB) {
    if (s_touch_tab_target == s_current_page) { return DISPLAY_OK; }
    return Display_SetHmiPage(s_touch_tab_target);
  }

  if (nav_touch == DISPLAY_NAV_TOUCH_PREV) { return Display_SetHmiPage(Display_PagesGetPrevPage(s_current_page)); }

  if (nav_touch == DISPLAY_NAV_TOUCH_NEXT) { return Display_SetHmiPage(Display_PagesGetNextPage(s_current_page)); }

  return DISPLAY_OK;
}

/*
 * 初始化 LCD、图元层和触摸端口。
 */
static Display_Result_t AtkMd0700_PortInit(void)
{
  if (Display_Ssd1963_Init() != DISPLAY_SSD1963_OK) { return DISPLAY_NOT_READY; }

  if (Display_GfxInit(&s_atk_md0700_gfx_port) != DISPLAY_GFX_OK) { return DISPLAY_ERROR; }

  s_touch_ready       = (Display_Gt911_Init() == DISPLAY_GT911_OK) ? 1U : 0U;
  s_touch_down        = 0U;
  s_touch_retry_count = 0U;
  DBG_PRINT("display touch init=%u", (unsigned int)s_touch_ready);

  return DISPLAY_OK;
}

static void Display_InitSelfCheckValues(void)
{
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_GNSS, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_BME280, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_LORA, 0U);
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
  if (s_recover_requested != 0U) {
    s_recover_requested   = 0U;
    s_display_initialized = 0U;
    s_display_ready       = 0U;
  }

  if (s_display_initialized == 0U) { return Display_Init(); }

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
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_BATTERY_PERCENT, 86U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE, 1200U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT, 90U);
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

  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ACTIVE_MASK, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ROW1_CODE, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ROW2_CODE, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ROW3_CODE, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ROW4_CODE, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ROW5_CODE, 0U);

  /* LoRa 连接页演示数据：9 个在线节点，触发滚动翻页(占位，待真实节点发现接入) */
  {
    static const Display_LoraNode_t mock_nodes[] = {
        {0x12U, 1U, 143158U}, {0x13U, 2U, 143202U}, {0x15U, 3U, 143010U}, {0x1AU, 5U, 142955U}, {0x21U, 7U, 143140U}, {0x24U, 8U, 143300U}, {0x2FU, 11U, 142847U}, {0x31U, 12U, 143312U}, {0x3AU, 15U, 143350U}};
    Display_PagesSetLoraNodes(mock_nodes, (uint16_t)(sizeof(mock_nodes) / sizeof(mock_nodes[0])));
  }
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

#if DISPLAY_USE_LVGL_BACKEND
  if (Display_LvglInit(Px4Lite_PlatformGetMs()) != DISPLAY_OK) {
    s_display_ready       = 0U;
    s_display_initialized = 1U;
    return DISPLAY_ERROR;
  }

  s_display_ready         = 1U;
  s_display_initialized   = 1U;
  s_current_page          = DISPLAY_HMI_PAGE_SELF_CHECK;
  s_touch_down            = 0U;
  s_touch_nav_latched     = DISPLAY_NAV_TOUCH_NONE;
  s_touch_slider_latched  = 0;
  s_refresh_cursor        = 0U;
  s_static_redraw_pending = 1U;
  Display_InitSelfCheckValues();
#ifdef DEBUG_ENABLE
  Display_LoadMockValues();
#endif
  return DISPLAY_OK;
#else
  if (AtkMd0700_PortInit() != DISPLAY_OK) {
    s_display_ready       = 0U;
    s_display_initialized = 1U;
    return DISPLAY_ERROR;
  }

  s_display_ready         = 1U;
  s_display_initialized   = 1U;
  s_current_page          = DISPLAY_HMI_PAGE_SELF_CHECK;
  s_touch_down            = 0U;
  s_touch_nav_latched     = DISPLAY_NAV_TOUCH_NONE;
  s_touch_slider_latched  = 0;
  s_refresh_cursor        = 0U;
  s_static_redraw_pending = 0U;
  Display_InitSelfCheckValues();
#ifdef DEBUG_ENABLE
  Display_LoadMockValues();
#endif

  if (Display_PagesDrawStatic(s_current_page, Display_GetCachedValue) != DISPLAY_OK) { return DISPLAY_ERROR; }

  return DISPLAY_OK;
#endif
}

/*
 * 执行显示模块自检并返回错误码。
 */
Display_Result_t Display_SelfCheck(uint16_t *error_code)
{
#if DISPLAY_USE_LVGL_BACKEND
  return Display_LvglSelfCheck(error_code);
#else
  if (error_code != 0) { *error_code = 0U; }

  if (s_display_ready == 0U) {
    if (error_code != 0) { *error_code = 1U; }
    return DISPLAY_NOT_READY;
  }

  return DISPLAY_OK;
#endif
}

void Display_RequestRecover(void)
{
#if DISPLAY_USE_LVGL_BACKEND
  Display_LvglRequestRecover();
#endif
  s_recover_requested = 1U;
}

/*
 * 根据当前时间刷新当前页的动态字段和告警图标。
 */
Display_Result_t Display_Refresh(uint32_t now_ms)
{
  uint16_t i;

  if (s_display_ready == 0U) { return DISPLAY_NOT_READY; }

  for (i = 0U; i < DISPLAY_ARRAY_SIZE(s_hmi_variables); i++) {
    const Display_HmiVariableConfig_t *variable = &s_hmi_variables[i];
    Display_ValueCache_t *cache                 = &s_hmi_values[variable->id];

    if ((cache->valid == 0U) || (variable->page != s_current_page)) { continue; }

    if ((cache->dirty == 0U) && (cache->drawn_valid != 0U) && (cache->value == cache->drawn_value)) { continue; }

    if ((cache->dirty == 0U) && (variable->refresh_ms != 0U) && (cache->last_refresh_ms != 0U) && ((now_ms - cache->last_refresh_ms) < variable->refresh_ms)) { continue; }

    if ((cache->dirty == 0U) && (variable->refresh_ms == 0U) && (cache->drawn_valid != 0U)) { continue; }

    if (Display_PagesDrawField(variable, cache->value) != DISPLAY_OK) { return DISPLAY_ERROR; }

    cache->drawn_value     = cache->value;
    cache->drawn_valid     = 1U;
    cache->last_refresh_ms = now_ms;
    cache->dirty           = 0U;
  }

  (void)Display_PagesDrawHeaderDynamic(Display_GetCachedValue, 0U);

  return DISPLAY_OK;
}

/*
 * 将 Framework 模块状态映射为页面状态灯数值。
 */
static uint16_t Display_MapStateValue(Px4Lite_State_t state)
{
  if (state == PX4LITE_STATE_ONLINE) { return 2U; }

  if ((state == PX4LITE_STATE_STARTING) || (state == PX4LITE_STATE_DEGRADED)) { return 1U; }

  if ((state == PX4LITE_STATE_OFFLINE) || (state == PX4LITE_STATE_FAILED)) { return 3U; }

  return 0U;
}

/*
 * 存储自检为二元状态：内存卡就绪(ONLINE)显示绿灯(2)，否则一律红灯(3)。
 */
static uint16_t Display_MapStorageStateValue(Px4Lite_State_t state)
{
  return (state == PX4LITE_STATE_ONLINE) ? 2U : 3U;
}

/*
 * 将有符号厘米每秒转为用于显示的无符号速度量。
 */
static uint16_t Display_AbsCmsToU16(int32_t value)
{
  uint32_t magnitude;

  if (value < 0) {
    magnitude = (uint32_t)(-value);
  } else {
    magnitude = (uint32_t)value;
  }

  if (magnitude > 65535U) { return 65535U; }

  return (uint16_t)magnitude;
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

static uint8_t Display_StateHasUsableData(Px4Lite_State_t state)
{
  return ((state == PX4LITE_STATE_ONLINE) || (state == PX4LITE_STATE_DEGRADED)) ? 1U : 0U;
}

/**
 * @brief 判断本轮显示是否应加载快照。
 *
 * @details LOCAL 沿用原策略，只显示新鲜快照；REMOTE 模式下 `PX4LITE_STALE`
 * 表示该域曾收到远端值但超过刷新窗口，显示层应继续保留最后值，避免低空速或
 * 分域超时造成字段被本机端清零。
 */
static uint8_t Display_ShouldLoadSnapshot(Px4Lite_Result_t result, Px4Lite_RemoteMode_t display_mode)
{
  if (result == PX4LITE_OK) { return 1U; }
  return ((display_mode == PX4LITE_REMOTE_MODE_REMOTE) && (result == PX4LITE_STALE)) ? 1U : 0U;
}

static uint16_t Display_GnssSignalValue(Px4Lite_State_t state)
{
  return (state == PX4LITE_STATE_ONLINE) ? 1U : 0U;
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

/* 电机电池(ADC2)门限(单位 mV)：
 *   <9.0V  没电：电量/电压显示 0、电机停机、锁 PWM；
 *   9.0~9.9V 供电不足：电机仍不能启动(停机/锁 PWM)，消息+告警提示供电不足；
 *   9.9~10.5V 低电：电机可运行，提示充电；
 *   >=10.5V 正常。 */
#define DISPLAY_MOTOR_BAT_CONNECT_MV 5000U  /* 在位门限：<5V 视为未插电池(电机断开)，电量/电压显示 0 */
#define DISPLAY_MOTOR_BAT_DEAD_MV    9000U  /* 没电门限：已插电池但 <9.0V */
#define DISPLAY_MOTOR_BAT_RUN_MV     9900U  /* 电机可启动门限：<9.9V 停机并锁 PWM */
#define DISPLAY_MOTOR_BAT_OK_MV      10500U /* 充电提示门限：<10.5V 提示充电 */
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

static void Display_ClearBatteryFields(void)
{
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_BATTERY_VOLTAGE, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_BATTERY_PERCENT, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT, 0U);
  /* 电机电池数据无效(未接入/掉线)，电机自检灯一律红灯。 */
  Display_SetMotorSelfCheckLights(3U);
}

static void Display_ClearMotorFields(void)
{
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_1, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_2, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_3, 0U);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_4, 0U);
}

static void Display_LoadMotorSnapshot(const App_MotorSnapshot_t *motor)
{
  if (motor == 0) { return; }

  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_1, motor->duty_percent[0]);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_2, motor->duty_percent[1]);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_3, motor->duty_percent[2]);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_PWM_4, motor->duty_percent[3]);
}

/*
 * 将导航快照字段写入 Display 缓存。
 */
static void Display_LoadNavigationSnapshot(const App_NavigationSnapshot_t *navigation)
{
  uint16_t ground_speed_cms;

  if (navigation == 0) { return; }

  ground_speed_cms = Display_AbsCmsToU16(navigation->velocity_north_cms);
  if (Display_AbsCmsToU16(navigation->velocity_east_cms) > ground_speed_cms) { ground_speed_cms = Display_AbsCmsToU16(navigation->velocity_east_cms); }

  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_SAT_COUNT, (uint16_t)navigation->satellites_used);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_HDOP, navigation->hdop_x100);
  (void)Display_SetHmiValueI32(DISPLAY_HMI_VAR_LATITUDE, navigation->latitude_e7);
  (void)Display_SetHmiValueI32(DISPLAY_HMI_VAR_LONGITUDE, navigation->longitude_e7);
  (void)Display_SetHmiValueI32(DISPLAY_HMI_VAR_ALTITUDE, navigation->altitude_mm);
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_SPEED, ground_speed_cms);
  (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_YAW, (int16_t)(navigation->yaw_deg100 / 10));
  if ((navigation->valid_mask & PX4LITE_NAV_VALID_ATTITUDE) != 0U) {
    (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_ROLL, (int16_t)(navigation->roll_deg100 / 10));
    (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_PITCH, (int16_t)(navigation->pitch_deg100 / 10));
  } else {
    Display_ClearAttitudeFields();
  }
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

/*
 * 清除统一日期时间显示字段。
 */
static void Display_ClearDateTimeSnapshot(void)
{
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_GNSS_TIME, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_CLOCK_TIME, 0U);
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_DATE, 0U);
}

/* ---- 消息日志：状态变化检测 ---- */

/* 状态去抖(ms)：某状态需连续稳定这么久才记一条。上电收敛期的瞬时跳变
   (断开/未通过/有告警)持续不到这个时长即被吞掉；状态一稳定很快就记，
   不必死等固定时长。想更快可调小，但太小会把上电抖动也记进去。 */
#define DISPLAY_MSGLOG_DEBOUNCE_MS 1500U

/* 把 now_ms 转成 HHMMSS 编码(开机运行时间占位)。 */
static uint32_t Display_NowHhmmss(uint32_t now_ms)
{
  uint32_t total_s = now_ms / 1000U;
  uint32_t hh      = (total_s / 3600U) % 100U;
  uint32_t mm      = (total_s / 60U) % 60U;
  uint32_t ss      = total_s % 60U;

  return (hh * 10000U) + (mm * 100U) + ss;
}

/* 每类状态的去抖追踪。 */
typedef struct {
  Display_LogMsg_t committed; /* 已记录的状态 */
  Display_LogMsg_t candidate; /* 当前候选状态 */
  uint32_t since_ms;          /* 候选状态起始时刻 */
} Display_LogDebounce_t;

/* 候选状态连续稳定 DEBOUNCE_MS 才提交一条日志，返回是否本次新提交了一条。 */
static uint8_t Display_LogDebounce(Display_LogDebounce_t *d, Display_LogMsg_t now, uint32_t now_ms, uint32_t t)
{
  if (now != d->candidate) {
    d->candidate = now;
    d->since_ms  = now_ms;
  }
  if ((d->candidate != d->committed) && ((uint32_t)(now_ms - d->since_ms) >= DISPLAY_MSGLOG_DEBOUNCE_MS)) {
    Display_PagesPushLogMessage(d->candidate, t);
    d->committed = d->candidate;
    return 1U;
  }
  return 0U;
}

static void Display_LogCommitInitial(Display_LogDebounce_t *d, Display_LogMsg_t msg, uint32_t now_ms)
{
  d->candidate = msg;
  d->committed = msg;
  d->since_ms  = now_ms;
}

/* GPS 三态：在线=正常；已收到模块数据但无定位=无信号；未收到数据/掉线=断开。 */
static Display_LogMsg_t Display_GpsLogMsg(const Px4Lite_ModuleStatus_t *status)
{
  if (status == 0) { return DISPLAY_LOGMSG_GPS_LOST; }
  if (status->state == PX4LITE_STATE_ONLINE) { return DISPLAY_LOGMSG_GPS_OK; }
  if ((status->state == PX4LITE_STATE_DEGRADED) && (status->last_rx_ms != 0U)) { return DISPLAY_LOGMSG_GPS_NOSIG; }
  return DISPLAY_LOGMSG_GPS_LOST;
}

/* 二态：在线=正常，否则断开。 */
static Display_LogMsg_t Display_TwoStateLogMsg(Px4Lite_State_t state, Display_LogMsg_t ok_msg, Display_LogMsg_t lost_msg)
{
  return (state == PX4LITE_STATE_ONLINE) ? ok_msg : lost_msg;
}

/* 单模块状态读取，取不到一律按离线处理(与自检灯 fallback 同源)。 */
static Px4Lite_State_t Display_LogModuleState(Px4Lite_ModuleId_t id)
{
  Px4Lite_ModuleStatus_t status;

  if (App_GetModuleStatus(id, &status) == PX4LITE_OK) { return status.state; }
  return PX4LITE_STATE_OFFLINE;
}

static Px4Lite_State_t Display_LogModuleStatus(Px4Lite_ModuleId_t id, Px4Lite_ModuleStatus_t *status)
{
  if ((status != 0) && (App_GetModuleStatus(id, status) == PX4LITE_OK)) { return status->state; }
  if (status != 0) {
    memset(status, 0, sizeof(*status));
    status->module_id = id;
    status->state     = PX4LITE_STATE_OFFLINE;
  }
  return PX4LITE_STATE_OFFLINE;
}

/* 自检三态：5 个模块全在线=通过，有掉线/失败=未通过，否则部分通过。 */
static Display_LogMsg_t Display_SelfCheckLogMsg(const Px4Lite_State_t *states, uint16_t count)
{
  uint16_t online = 0U;
  uint16_t failed = 0U;
  uint16_t i;

  for (i = 0U; i < count; i++) {
    if (states[i] == PX4LITE_STATE_ONLINE) {
      online++;
    } else if ((states[i] == PX4LITE_STATE_OFFLINE) || (states[i] == PX4LITE_STATE_FAILED)) {
      failed++;
    } else {
      /* STARTING/DEGRADED 计入部分通过 */
    }
  }

  if (online == count) { return DISPLAY_LOGMSG_SELFCHECK_OK; }
  if (failed > 0U) { return DISPLAY_LOGMSG_SELFCHECK_FAIL; }
  return DISPLAY_LOGMSG_SELFCHECK_PART;
}

/* 电机电池分档消息(读带滞回的档位)：断开/没电/供电不足/需充电/正常。 */
static Display_LogMsg_t Display_MotorLogMsg(uint32_t now_ms)
{
  (void)now_ms;
  switch (s_motor_band) {
    case DISPLAY_MOTOR_BAND_DISCONNECT:
      return DISPLAY_LOGMSG_MOTOR_DISCONNECT;
    case DISPLAY_MOTOR_BAND_DEAD:
      return DISPLAY_LOGMSG_MOTOR_DEAD;
    case DISPLAY_MOTOR_BAND_LOWPOWER:
      return DISPLAY_LOGMSG_MOTOR_LOWPOWER;
    case DISPLAY_MOTOR_BAND_CHARGE:
      return DISPLAY_LOGMSG_MOTOR_CHARGE;
    default:
      return DISPLAY_LOGMSG_MOTOR_OK;
  }
}

/* 主控电池(读带滞回的档位)：5~10.5V 提示需充电；未接入或正常返回 COUNT(不追加日志)。 */
static Display_LogMsg_t Display_MainBatteryLogMsg(uint32_t now_ms)
{
  (void)now_ms;
  return (s_main_band == DISPLAY_MAIN_BAND_CHARGE) ? DISPLAY_LOGMSG_MAIN_CHARGE : DISPLAY_LOGMSG_COUNT;
}

/*
 * 检测各类状态变化并写入消息日志。数据源与自检灯/告警表同源:
 * 模块状态用 App_GetModuleStatus 逐个取，告警码由调用方传入。
 * 不依赖完整 system 快照，因此 system 快照不可用时日志仍能更新。
 * 电机状态来源电机电池(ADC2)，正常/断开/供电不足三态实时追加。
 */
static void Display_UpdateMessageLog(uint32_t now_ms, uint16_t highest_fault_code)
{
  static uint8_t boot_done              = 0U;
  static uint8_t boot_tracking          = 0U;
  static Display_LogDebounce_t db_gps   = {DISPLAY_LOGMSG_COUNT, DISPLAY_LOGMSG_COUNT, 0U};
  static Display_LogDebounce_t db_att   = {DISPLAY_LOGMSG_COUNT, DISPLAY_LOGMSG_COUNT, 0U};
  static Display_LogDebounce_t db_env   = {DISPLAY_LOGMSG_COUNT, DISPLAY_LOGMSG_COUNT, 0U};
  static Display_LogDebounce_t db_comm  = {DISPLAY_LOGMSG_COUNT, DISPLAY_LOGMSG_COUNT, 0U};
  static Display_LogDebounce_t db_store = {DISPLAY_LOGMSG_COUNT, DISPLAY_LOGMSG_COUNT, 0U};
  static Display_LogDebounce_t db_motor = {DISPLAY_LOGMSG_COUNT, DISPLAY_LOGMSG_COUNT, 0U};
  static Display_LogDebounce_t db_main  = {DISPLAY_LOGMSG_COUNT, DISPLAY_LOGMSG_COUNT, 0U};
  static Display_LogDebounce_t db_alarm = {DISPLAY_LOGMSG_COUNT, DISPLAY_LOGMSG_COUNT, 0U};
  static Display_LogMsg_t boot_self     = DISPLAY_LOGMSG_COUNT;
  static Display_LogMsg_t boot_gps      = DISPLAY_LOGMSG_COUNT;
  static Display_LogMsg_t boot_att      = DISPLAY_LOGMSG_COUNT;
  static Display_LogMsg_t boot_env      = DISPLAY_LOGMSG_COUNT;
  static Display_LogMsg_t boot_comm     = DISPLAY_LOGMSG_COUNT;
  static Display_LogMsg_t boot_store    = DISPLAY_LOGMSG_COUNT;
  static Display_LogMsg_t boot_alarm    = DISPLAY_LOGMSG_COUNT;
  static uint32_t boot_since_ms         = 0U;
  static uint32_t boot_start_t          = 0U;
  Px4Lite_ModuleStatus_t gnss_status;
  Px4Lite_State_t states[5];
  uint32_t t = Display_NowHhmmss(now_ms);
  Display_LogMsg_t now_self;
  Display_LogMsg_t now_gps;
  Display_LogMsg_t now_att;
  Display_LogMsg_t now_env;
  Display_LogMsg_t now_comm;
  Display_LogMsg_t now_store;
  Display_LogMsg_t now_motor;
  Display_LogMsg_t now_main;
  Display_LogMsg_t now_alarm;

  states[0] = Display_LogModuleStatus(PX4LITE_MODULE_GNSS, &gnss_status);
  states[1] = Display_LogModuleState(PX4LITE_MODULE_IMU);
  states[2] = Display_LogModuleState(PX4LITE_MODULE_BARO);
  states[3] = Display_LogModuleState(PX4LITE_MODULE_LORA);
  states[4] = Display_LogModuleState(PX4LITE_MODULE_STORAGE);

  now_self  = Display_SelfCheckLogMsg(states, 5U);
  now_gps   = Display_GpsLogMsg(&gnss_status);
  now_att   = Display_TwoStateLogMsg(states[1], DISPLAY_LOGMSG_ATT_OK, DISPLAY_LOGMSG_ATT_LOST);
  now_env   = Display_TwoStateLogMsg(states[2], DISPLAY_LOGMSG_ENV_OK, DISPLAY_LOGMSG_ENV_LOST);
  now_comm  = Display_TwoStateLogMsg(states[3], DISPLAY_LOGMSG_COMM_OK, DISPLAY_LOGMSG_COMM_LOST);
  now_store = Display_TwoStateLogMsg(states[4], DISPLAY_LOGMSG_STORAGE_OK, DISPLAY_LOGMSG_STORAGE_LOST);
  now_motor = Display_MotorLogMsg(now_ms);
  now_main  = Display_MainBatteryLogMsg(now_ms);
  now_alarm = (highest_fault_code != 0U) ? DISPLAY_LOGMSG_ALARM_ACTIVE : DISPLAY_LOGMSG_ALARM_NONE;

  if (boot_done == 0U) {
    /* 电机电量收敛慢(滤波+多次确认)，不纳入启动门控，否则会拖慢整批自检日志；
       提交时直接取电机当前状态，运行阶段再按真实变化追加。 */
    if ((boot_tracking == 0U) || (now_self != boot_self) || (now_gps != boot_gps) || (now_att != boot_att) || (now_env != boot_env) || (now_comm != boot_comm) || (now_store != boot_store) || (now_alarm != boot_alarm)) {
      boot_self     = now_self;
      boot_gps      = now_gps;
      boot_att      = now_att;
      boot_env      = now_env;
      boot_comm     = now_comm;
      boot_store    = now_store;
      boot_alarm    = now_alarm;
      boot_since_ms = now_ms;
      if (boot_tracking == 0U) {
        boot_start_t = t;
        Display_PagesPushLogMessage(DISPLAY_LOGMSG_SYSTEM_START, boot_start_t);
        boot_tracking = 1U;
      }
      return;
    }

    if ((uint32_t)(now_ms - boot_since_ms) < DISPLAY_MSGLOG_DEBOUNCE_MS) { return; }

    Display_PagesPushLogMessage(boot_self, t);
    Display_PagesPushLogMessage(boot_gps, t);
    Display_PagesPushLogMessage(boot_att, t);
    Display_PagesPushLogMessage(boot_env, t);
    Display_PagesPushLogMessage(boot_comm, t);
    Display_PagesPushLogMessage(boot_store, t);
    Display_PagesPushLogMessage(now_motor, t);
    Display_PagesPushLogMessage(boot_alarm, t);

    Display_LogCommitInitial(&db_gps, boot_gps, now_ms);
    Display_LogCommitInitial(&db_att, boot_att, now_ms);
    Display_LogCommitInitial(&db_env, boot_env, now_ms);
    Display_LogCommitInitial(&db_comm, boot_comm, now_ms);
    Display_LogCommitInitial(&db_store, boot_store, now_ms);
    Display_LogCommitInitial(&db_motor, now_motor, now_ms);
    Display_LogCommitInitial(&db_main, now_main, now_ms);
    Display_LogCommitInitial(&db_alarm, boot_alarm, now_ms);
    boot_done = 1U;
    return;
  }

  /* 启动批量日志之后，各类状态按真实变化继续去抖追加。 */
  (void)Display_LogDebounce(&db_gps, now_gps, now_ms, t);
  (void)Display_LogDebounce(&db_att, now_att, now_ms, t);
  (void)Display_LogDebounce(&db_env, now_env, now_ms, t);
  (void)Display_LogDebounce(&db_comm, now_comm, now_ms, t);
  (void)Display_LogDebounce(&db_store, now_store, now_ms, t);
  (void)Display_LogDebounce(&db_motor, now_motor, now_ms, t);
  (void)Display_LogDebounce(&db_main, now_main, now_ms, t);
  (void)Display_LogDebounce(&db_alarm, now_alarm, now_ms, t);
}

/*
 * LoRa 状态灯/链路灯：本机白名单字段。
 *
 * 按 doc18/19，LoRa 状态灯永远反映显示端自身链路健康，不随 LOCAL/REMOTE 模式与
 * 远端系统快照新鲜度变化、不走模式解析。每帧无条件用本机模块状态驱动，确保
 * REMOTE 模式下即便收不到远端模块遥测，这颗灯也照常显示本机状态而非灰。
 */
static void Display_LoadLocalLoraStatus(void)
{
  Px4Lite_ModuleStatus_t status;

  if (App_GetModuleStatus(PX4LITE_MODULE_LORA, &status) == PX4LITE_OK) {
    uint16_t value = Display_MapStateValue(status.state);
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_LORA, value);
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_LORA_STATUS, value);
  }
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

  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_GNSS, Display_MapStateValue(system->modules[PX4LITE_MODULE_GNSS].state));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_FIX, Display_GnssSignalValue(system->modules[PX4LITE_MODULE_GNSS].state));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_MPU6050, Display_MapStateValue(system->modules[PX4LITE_MODULE_IMU].state));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_BME280, Display_MapStateValue(system->modules[PX4LITE_MODULE_BARO].state));
  /* LoRa 状态灯属于本机白名单(doc18/19)：不走远端归一的系统快照，统一由
     Display_LoadLocalLoraStatus() 每帧用本机模块状态驱动，此处不再写。 */
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
  /* LoRa 状态灯不在此清零：它是本机白名单字段，由 Display_LoadLocalLoraStatus()
     永远按本机状态驱动，远端系统快照缺失/清空都不应把它灭成灰(doc18/19)。 */
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

/* 每次快照按电压更新电机/主控电池档位(带滞回)，所有判定统一读档位避免阈值附近横跳。 */
static void Display_UpdateBatteryBands(const App_EnvironmentSnapshot_t *environment)
{
  static const uint32_t motor_t[4] = {DISPLAY_MOTOR_BAT_CONNECT_MV, DISPLAY_MOTOR_BAT_DEAD_MV, DISPLAY_MOTOR_BAT_RUN_MV, DISPLAY_MOTOR_BAT_OK_MV};
  static const uint32_t main_t[2]  = {DISPLAY_MAIN_BAT_PRESENT_MV, DISPLAY_MAIN_BAT_CHARGE_MV};

  if (environment == 0) { return; }
  s_motor_band = Display_BatBandHyst(s_motor_band, environment->voltage2_mv, motor_t, 4U);
  s_main_band  = Display_BatBandHyst(s_main_band, environment->voltage_mv, main_t, 2U);
}

static uint16_t Display_EvalMotorFault(const App_EnvironmentSnapshot_t *environment)
{
  (void)environment;
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

/* 主控电池告警：5~10.5V 报“需充电”；未接入或正常不报。 */
static uint16_t Display_EvalMainFault(const App_EnvironmentSnapshot_t *environment)
{
  (void)environment;
  return (s_main_band == DISPLAY_MAIN_BAND_CHARGE) ? (uint16_t)PX4LITE_FAULT_POWER_CHARGE : 0U;
}

static void Display_LoadAlarmSnapshot(const App_AlarmSnapshot_t *alarm, uint16_t motor_fault, uint16_t main_fault)
{
  static const Display_HmiVariableId_t row_ids[] = {DISPLAY_HMI_VAR_ALARM_ROW1_CODE, DISPLAY_HMI_VAR_ALARM_ROW2_CODE, DISPLAY_HMI_VAR_ALARM_ROW3_CODE, DISPLAY_HMI_VAR_ALARM_ROW4_CODE, DISPLAY_HMI_VAR_ALARM_ROW5_CODE};
  int32_t severity;
  uint16_t record_index;
  uint16_t row_index   = 0U;
  uint32_t active_mask = 0U;
  uint32_t selfcheck_faults[PX4LITE_MODULE_COUNT + 2U];
  uint16_t selfcheck_count = 0U;
  uint32_t selfcheck_fp    = 0U;
  uint32_t motor_packed    = ((uint32_t)0xFFFFU << 16) | (uint32_t)motor_fault;
  uint32_t main_packed     = ((uint32_t)PX4LITE_MODULE_BATTERY << 16) | (uint32_t)main_fault;

  if (alarm == 0) { return; }

  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_ALARM_CODE, alarm->highest_fault_code);

  for (record_index = 0U; record_index < (uint16_t)PX4LITE_MODULE_COUNT; record_index++) {
    const App_AlarmRecord_t *record = &alarm->records[record_index];

    if ((record->active != 0U) && (record->source_id < 32U)) { active_mask |= (1UL << record->source_id); }

    /* 收集自检页错误码表的激活故障：有一条记一条，故障恢复(active=0)
       后不再计入，对应行会随之消失。 */
    if ((record->active != 0U) && (record->fault_code != 0U)) {
      uint32_t packed = ((uint32_t)record->source_id << 16) | (uint32_t)record->fault_code;
      if (selfcheck_count < (uint16_t)PX4LITE_MODULE_COUNT) {
        selfcheck_faults[selfcheck_count] = packed;
        selfcheck_count++;
      }
      selfcheck_fp = (selfcheck_fp * 31U) + packed;
    }
  }

  /* 电机/主控电池告警由 ADC 电压推导，不在框架告警记录里，单独追加到两张表。 */
  if ((motor_fault != 0U) && (selfcheck_count < (uint16_t)(PX4LITE_MODULE_COUNT + 2U))) {
    selfcheck_faults[selfcheck_count] = motor_packed;
    selfcheck_count++;
    selfcheck_fp = (selfcheck_fp * 31U) + motor_packed;
  }
  if ((main_fault != 0U) && (selfcheck_count < (uint16_t)(PX4LITE_MODULE_COUNT + 2U))) {
    selfcheck_faults[selfcheck_count] = main_packed;
    selfcheck_count++;
    selfcheck_fp = (selfcheck_fp * 31U) + main_packed;
  }

  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_ALARM_ACTIVE_MASK, active_mask);

  /* 把当前激活故障列表交给绘制层，并用集合指纹驱动错误码表重绘：
     故障集合不变则指纹不变(不重绘)，新增/消除任一故障即触发刷新。 */
  Display_PagesSetSelfCheckFaults(selfcheck_faults, selfcheck_count);
  selfcheck_fp = (selfcheck_fp ^ (selfcheck_fp >> 16)) + selfcheck_count;
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_ERROR_CODE, (uint16_t)selfcheck_fp);

  for (severity = (int32_t)PX4LITE_ALARM_FATAL; (severity >= (int32_t)PX4LITE_ALARM_INFO) && (row_index < DISPLAY_ARRAY_SIZE(row_ids)); severity--) {
    for (record_index = 0U; (record_index < (uint16_t)PX4LITE_MODULE_COUNT) && (row_index < DISPLAY_ARRAY_SIZE(row_ids)); record_index++) {
      const App_AlarmRecord_t *record = &alarm->records[record_index];

      if ((record->active != 0U) && (record->fault_code != 0U) && (record->severity == (uint8_t)severity)) {
        (void)Display_SetHmiValueU32(row_ids[row_index], ((uint32_t)record->source_id << 16) | record->fault_code);
        row_index++;
      }
    }
  }

  if ((motor_fault != 0U) && (row_index < DISPLAY_ARRAY_SIZE(row_ids))) {
    (void)Display_SetHmiValueU32(row_ids[row_index], motor_packed);
    row_index++;
  }
  if ((main_fault != 0U) && (row_index < DISPLAY_ARRAY_SIZE(row_ids))) {
    (void)Display_SetHmiValueU32(row_ids[row_index], main_packed);
    row_index++;
  }

  while (row_index < DISPLAY_ARRAY_SIZE(row_ids)) {
    (void)Display_SetHmiValueU32(row_ids[row_index], 0U);
    row_index++;
  }
}

static void Display_ClearAlarmSnapshot(void)
{
  App_AlarmSnapshot_t alarm;

  memset(&alarm, 0, sizeof(alarm));
  Display_LoadAlarmSnapshot(&alarm, 0U, 0U);
}

static uint8_t Display_LoadModuleStatusFallback(void)
{
  Px4Lite_ModuleStatus_t status;
  uint8_t loaded = 0U;

  if (App_GetModuleStatus(PX4LITE_MODULE_GNSS, &status) == PX4LITE_OK) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_GNSS, Display_MapStateValue(status.state));
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_GNSS_FIX, Display_GnssSignalValue(status.state));
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
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_LORA, Display_MapStateValue(status.state));
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_LORA_STATUS, Display_MapStateValue(status.state));
    loaded = 1U;
  }
  if (App_GetModuleStatus(PX4LITE_MODULE_STORAGE, &status) == PX4LITE_OK) {
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_SELF_CHECK_SD, Display_MapStorageStateValue(status.state));
    loaded = 1U;
  }

  return loaded;
}

static void Display_ApplyOfflineDataPolicy(void)
{
  Px4Lite_ModuleStatus_t status;

  if ((App_GetModuleStatus(PX4LITE_MODULE_IMU, &status) != PX4LITE_OK) || (Display_StateHasUsableData(status.state) == 0U)) { Display_ClearAttitudeFields(); }
  if ((App_GetModuleStatus(PX4LITE_MODULE_BARO, &status) != PX4LITE_OK) || (Display_StateHasUsableData(status.state) == 0U)) { Display_ClearEnvironmentFields(); }
  if ((App_GetModuleStatus(PX4LITE_MODULE_BATTERY, &status) != PX4LITE_OK) || (Display_StateHasUsableData(status.state) == 0U)) { Display_ClearBatteryFields(); }
}

static void Display_LoadEnvironmentSnapshot(const App_EnvironmentSnapshot_t *environment)
{
  if (environment == 0) { return; }

  (void)Display_SetHmiValueI16(DISPLAY_HMI_VAR_TEMPERATURE, Display_FloatToI16Tenths(environment->temperature_c));
  (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_HUMIDITY, Display_FloatToU16Tenths(environment->relative_humidity_pct));
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_PRESSURE, Display_FloatPaToU32(environment->pressure_pa));

  /* 主控/电机电池：未接入(档位为未接入/断开)时电压与电量都显示 0，接入后才显示真实值；
     电压保留一位小数由显示层格式化。档位带滞回，避免噪声横跳。 */
  {
    uint16_t main_cv  = (s_main_band != DISPLAY_MAIN_BAND_ABSENT) ? (uint16_t)((environment->voltage_mv + 5U) / 10U) : 0U;
    uint8_t main_pct  = (s_main_band != DISPLAY_MAIN_BAND_ABSENT) ? environment->battery_percent : 0U;
    uint16_t motor_cv = (s_motor_band != DISPLAY_MOTOR_BAND_DISCONNECT) ? (uint16_t)((environment->voltage2_mv + 5U) / 10U) : 0U;
    uint8_t motor_pct = (s_motor_band != DISPLAY_MOTOR_BAND_DISCONNECT) ? environment->battery2_percent : 0U;

    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_BATTERY_VOLTAGE, main_cv);
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_BATTERY_PERCENT, main_pct);
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE, motor_cv);
    (void)Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT, motor_pct);
  }

  /* 电机电池<9.9V(断开/没电/供电不足档)：电机不能启动，强制停机并锁定 PWM；回到正常/需充电档自动解锁。 */
  if (s_motor_band <= DISPLAY_MOTOR_BAND_LOWPOWER) {
    s_motor_bat_cutoff = 1U;
    Display_ForceMotorsOff();
  } else {
    s_motor_bat_cutoff = 0U;
  }
  /* 电机自检灯跟随电机电池档位：<9.9V 红灯，9.9~10.5V 黄灯，>=10.5V 绿灯。 */
  Display_SetMotorSelfCheckLights(Display_MotorSelfCheckValue());
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
  Px4Lite_Result_t navigation_result;
  Px4Lite_Result_t system_result;
  Px4Lite_Result_t environment_result;
  Px4Lite_Result_t alarm_result;
  Px4Lite_Result_t date_time_result;
  Px4Lite_Result_t motor_result;
  Px4Lite_RemoteMode_t display_mode;
  uint16_t motor_fault = 0U;
  uint16_t main_fault  = 0U;
  uint8_t status_fallback_loaded = 0U;
  uint16_t msglog_highest        = 0U;

  if (Display_EnsureInit() != DISPLAY_OK) { return DISPLAY_NOT_READY; }

  display_mode = App_GetRemoteDisplayMode();

  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_UPTIME_MS, now_ms);
  /* 飞行时间(上电后运行)，秒粒度，避免毫秒每帧抖动导致重绘 */
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_FLIGHT_TIME_S, now_ms / 1000U);

  navigation_result = App_GetDisplayNavigation(&navigation, now_ms);
  if (Display_ShouldLoadSnapshot(navigation_result, display_mode) != 0U) {
    Display_LoadNavigationSnapshot(&navigation);
  } else {
    Display_ClearNavigationSnapshot();
  }

  date_time_result = App_GetDisplayDateTime(&date_time, now_ms);
  if (Display_ShouldLoadSnapshot(date_time_result, display_mode) != 0U) {
    Display_LoadDateTimeSnapshot(&date_time);
  } else {
    Display_ClearDateTimeSnapshot();
  }

  system_result = App_GetDisplaySystem(&system, now_ms);
  if (Display_ShouldLoadSnapshot(system_result, display_mode) != 0U) {
    Display_LoadSystemSnapshot(&system);
    msglog_highest = system.highest_fault_code;
  } else if (display_mode == PX4LITE_REMOTE_MODE_REMOTE) {
    Display_ClearSystemSnapshot();
  } else {
    status_fallback_loaded = Display_LoadModuleStatusFallback();
  }

  /* LoRa 状态灯为本机白名单：无论上面哪条分支、LOCAL 还是 REMOTE，都用本机状态
     覆盖一次，确保远端模块遥测缺失时这颗灯不会被清成灰(doc18/19)。 */
  Display_LoadLocalLoraStatus();

  /* 环境快照提前取，先用带滞回的档位刷新，再算电机/主控电池告警注入告警表与自检错误码表。 */
  environment_result = App_GetDisplayEnvironment(&environment, now_ms);
  if (Display_ShouldLoadSnapshot(environment_result, display_mode) != 0U) { Display_UpdateBatteryBands(&environment); }
  motor_fault = (Display_ShouldLoadSnapshot(environment_result, display_mode) != 0U) ? Display_EvalMotorFault(&environment) : 0U;
  main_fault  = (Display_ShouldLoadSnapshot(environment_result, display_mode) != 0U) ? Display_EvalMainFault(&environment) : 0U;

  alarm_result = App_GetDisplayAlarm(&alarm, now_ms);
  if (Display_ShouldLoadSnapshot(alarm_result, display_mode) != 0U) {
    Display_LoadAlarmSnapshot(&alarm, motor_fault, main_fault);
    msglog_highest = alarm.highest_fault_code;
  } else if (display_mode == PX4LITE_REMOTE_MODE_REMOTE) {
    Display_ClearAlarmSnapshot();
  }

  /* 消息日志与自检灯/告警表同源(App_GetModuleStatus + 告警码)，
     无论完整 system 快照是否可用都更新，避免日志一直空白。 */
  if (display_mode == PX4LITE_REMOTE_MODE_REMOTE) {
    Display_PagesClearLogMessages();
  } else {
    Display_UpdateMessageLog(now_ms, msglog_highest);
  }

  if (Display_ShouldLoadSnapshot(environment_result, display_mode) != 0U) {
    Display_LoadEnvironmentSnapshot(&environment);
  } else {
    Display_ClearEnvironmentFields();
    Display_ClearBatteryFields();
  }

  motor_result = App_GetDisplayMotor(&motor, now_ms);
  if (Display_ShouldLoadSnapshot(motor_result, display_mode) != 0U) {
    Display_LoadMotorSnapshot(&motor);
  } else {
    Display_ClearMotorFields();
  }

  /* LoRa 收发帧计数为累计值，独立于上面三个快照，每帧都刷新 */
  Display_LoadLoraStats();

  if (display_mode == PX4LITE_REMOTE_MODE_LOCAL) { Display_ApplyOfflineDataPolicy(); }

  /* 消息日志缓冲版本变化即触发日志区重绘 */
  (void)Display_SetHmiValueU32(DISPLAY_HMI_VAR_MESSAGE_LOG, Display_PagesGetLogVersion());

  if ((display_mode == PX4LITE_REMOTE_MODE_LOCAL) && (navigation_result != PX4LITE_OK) && (system_result != PX4LITE_OK) && (environment_result != PX4LITE_OK) && (status_fallback_loaded == 0U)) { return DISPLAY_NOT_READY; }

  return DISPLAY_OK;
}

/**
 * @brief 优先刷新当前页内的强节奏字段。
 *
 * @param[in] id 待优先刷新的 HMI 变量 ID。
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return DISPLAY_NOT_READY 表示本次已经绘制一个字段，DISPLAY_OK 表示无需绘制。
 */
static Display_Result_t Display_RefreshPriorityField(Display_HmiVariableId_t id, uint32_t now_ms)
{
  uint16_t i;

  for (i = 0U; i < DISPLAY_ARRAY_SIZE(s_hmi_variables); i++) {
    const Display_HmiVariableConfig_t *variable = &s_hmi_variables[i];
    Display_ValueCache_t *cache;

    if ((variable->id != id) || (variable->page != s_current_page)) { continue; }

    cache = &s_hmi_values[variable->id];
    if (cache->valid == 0U) { return DISPLAY_OK; }
    if ((cache->drawn_valid != 0U) && (cache->value == cache->drawn_value)) { return DISPLAY_OK; }

    if (Display_PagesDrawField(variable, cache->value) != DISPLAY_OK) { return DISPLAY_ERROR; }

    cache->drawn_value     = cache->value;
    cache->drawn_valid     = 1U;
    cache->last_refresh_ms = now_ms;
    cache->dirty           = 0U;
    return DISPLAY_NOT_READY;
  }

  return DISPLAY_OK;
}

/*
 * 执行一次有预算约束的显示刷新步骤。
 */
Display_Result_t Display_RefreshStep(uint32_t now_ms, uint32_t budget_us)
{
#if DISPLAY_USE_LVGL_BACKEND
  if (Display_EnsureInit() != DISPLAY_OK) { return DISPLAY_NOT_READY; }
  return Display_LvglRefreshStep(now_ms, budget_us);
#else
  Display_Result_t result;
  uint16_t scanned;
  uint32_t start_us;

  if (Display_EnsureInit() != DISPLAY_OK) { return DISPLAY_NOT_READY; }

  if (s_static_redraw_pending != 0U) {
    if (Display_PagesDrawStatic(s_current_page, Display_GetCachedValue) != DISPLAY_OK) { return DISPLAY_ERROR; }
    Display_DrawSelectedMotorTriangle();
    s_static_redraw_pending = 0U;
    s_refresh_cursor        = 0U;
    return DISPLAY_NOT_READY;
  }

  result = Display_RefreshPriorityField(DISPLAY_HMI_VAR_CLOCK_TIME, now_ms);
  if (result != DISPLAY_OK) { return result; }

  result = Display_RefreshPriorityField(DISPLAY_HMI_VAR_FLIGHT_TIME_S, now_ms);
  if (result != DISPLAY_OK) { return result; }

  /* 一次调用内扫描整张变量表，绘制当前页所有到期字段；每画完一个字段检查
     时间预算，超出即让出，剩余字段在下个节拍续绘。扫满整表(scanned 上界)是
     硬性终止条件，即使微秒时基异常也不会卡死或漏刷。 */
  start_us = Px4Lite_PlatformGetUs();

  for (scanned = 0U; scanned < DISPLAY_ARRAY_SIZE(s_hmi_variables); scanned++) {
    const Display_HmiVariableConfig_t *variable;
    Display_ValueCache_t *cache;

    variable = &s_hmi_variables[s_refresh_cursor];
    cache    = &s_hmi_values[variable->id];
    s_refresh_cursor++;
    if (s_refresh_cursor >= DISPLAY_ARRAY_SIZE(s_hmi_variables)) { s_refresh_cursor = 0U; }

    if ((cache->valid == 0U) || (variable->page != s_current_page)) { continue; }

    if ((cache->dirty == 0U) && (cache->drawn_valid != 0U) && (cache->value == cache->drawn_value)) { continue; }

    if ((cache->dirty == 0U) && (variable->refresh_ms != 0U) && (cache->last_refresh_ms != 0U) && ((now_ms - cache->last_refresh_ms) < variable->refresh_ms)) { continue; }

    if (Display_IsMotorSliderId(variable->id) != 0U) {
      if (Display_PagesDrawMotorSliderField(variable, cache->drawn_value, cache->value, (cache->drawn_valid == 0U) ? 1U : 0U, 1U) != DISPLAY_OK) { return DISPLAY_ERROR; }
    } else if (Display_PagesDrawField(variable, cache->value) != DISPLAY_OK) {
      return DISPLAY_ERROR;
    }

    cache->drawn_value     = cache->value;
    cache->drawn_valid     = 1U;
    cache->last_refresh_ms = now_ms;
    cache->dirty           = 0U;

    if ((budget_us != 0U) && ((uint32_t)(Px4Lite_PlatformGetUs() - start_us) >= budget_us)) {
      return DISPLAY_NOT_READY;
    }
  }

  (void)Display_PagesDrawHeaderDynamic(Display_GetCachedValue, 0U);

  /* LoRa 连接页内容由版本号驱动重绘（业务回写节点/连接状态时触发） */
  if ((s_current_page == DISPLAY_HMI_PAGE_HIDDEN) && (Display_PagesLoraContentDirty() != 0U)) {
    Display_PagesDrawLoraContent();
  }

  return DISPLAY_OK;
#endif
}

/*
 * 切换当前 HMI 页面并重绘页面静态内容。
 */
Display_Result_t Display_SetHmiPage(Display_HmiPage_t page)
{
#if DISPLAY_USE_LVGL_BACKEND
  Display_Result_t result;

  result = Display_LvglSetPage(page);
  if (result != DISPLAY_OK) { return result; }
  s_current_page = page;
  return DISPLAY_OK;
#else
  uint16_t i;

  if (page >= DISPLAY_HMI_PAGE_COUNT) { return DISPLAY_ERROR; }

  if (s_display_ready == 0U) { return DISPLAY_NOT_READY; }

  s_current_page = page;

  for (i = 0U; i < DISPLAY_HMI_VAR_COUNT; i++) {
    s_hmi_values[i].dirty       = 1U;
    s_hmi_values[i].drawn_valid = 0U;
  }

  s_static_redraw_pending = 1U;
  s_refresh_cursor        = 0U;
  s_touch_slider_latched  = 0;

  return DISPLAY_OK;
#endif
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
#if DISPLAY_USE_LVGL_BACKEND
  return Display_LvglNeedsRefresh();
#else
  return s_static_redraw_pending;
#endif
}

void Display_ShowBootCode(uint8_t code)
{
  (void)code;
}

/*
 * 轮询触摸状态并处理触摸事件。
 */
Display_Result_t Display_PollTouch(void)
{
#if DISPLAY_USE_LVGL_BACKEND
  return (s_display_ready != 0U) ? DISPLAY_OK : DISPLAY_NOT_READY;
#else
  uint16_t x;
  uint16_t y;
  Display_NavTouch_t nav_touch;
  const Display_HmiVariableConfig_t *slider;
  Display_ValueCache_t *cache = 0;
  Display_Gt911Result_t touch_result;

  if (s_display_ready == 0U) { return DISPLAY_NOT_READY; }

  s_touch_poll_count++;

  if (s_touch_ready == 0U) {
    s_touch_retry_count++;
    if (s_touch_retry_count >= 5U) {
      s_touch_ready       = (Display_Gt911_Init() == DISPLAY_GT911_OK) ? 1U : 0U;
      s_touch_down        = 0U;
      s_touch_nav_latched = DISPLAY_NAV_TOUCH_NONE;
      s_touch_slider_latched = 0;
      s_touch_retry_count = 0U;
    }
    return DISPLAY_OK;
  }

  touch_result = Display_Gt911_Scan(&x, &y);
  if (touch_result == DISPLAY_GT911_OK) {
    s_touch_retry_count = 0U;
    if (s_touch_down == 0U) {
      s_touch_down = 1U;
      if (s_current_page == DISPLAY_HMI_PAGE_HIDDEN) {
        s_touch_nav_latched = DISPLAY_NAV_TOUCH_NONE;
        if (Display_PagesLoraHandleTouch(x, y) != DISPLAY_LORA_TOUCH_NONE) { Display_PagesDrawLoraContent(); }
        return DISPLAY_OK;
      }
      if (Display_IsMotorEstopTouch(x, y) != 0U) {
        s_touch_nav_latched = DISPLAY_NAV_TOUCH_NONE;
        return Display_MotorEmergencyStop();
      }
      slider = Display_FindMotorSliderAt(x, y);
      if (slider != 0) {
        s_touch_slider_latched = slider;
        s_touch_nav_latched    = DISPLAY_NAV_TOUCH_NONE;
        return Display_HandleMotorSliderTouch(slider, y, 0, 0);
      }
      s_touch_nav_latched = Display_GetNavTouch(x, y);
    } else if (s_touch_slider_latched != 0) {
      return Display_HandleMotorSliderTouch(s_touch_slider_latched, y, 0, 0);
    }

    return DISPLAY_OK;
  }

  if (touch_result == DISPLAY_GT911_NO_DATA) {
    s_touch_retry_count = 0U;
    return DISPLAY_OK;
  }

  if (touch_result == DISPLAY_GT911_NO_POINT) {
    nav_touch           = s_touch_nav_latched;
    if (s_touch_slider_latched != 0) {
      cache = &s_hmi_values[s_touch_slider_latched->id];
      if (cache->drawn_valid != 0U) { (void)Display_DrawMotorSliderNow(s_touch_slider_latched, (uint16_t)cache->drawn_value, 1U); }
    }
    s_touch_down        = 0U;
    s_touch_nav_latched = DISPLAY_NAV_TOUCH_NONE;
    s_touch_slider_latched = 0;
    s_touch_retry_count = 0U;
    return Display_HandleNavTouch(nav_touch);
  }

  s_touch_down        = 0U;
  s_touch_nav_latched = DISPLAY_NAV_TOUCH_NONE;
  s_touch_slider_latched = 0;
  s_touch_retry_count++;
  if (s_touch_retry_count >= 3U) {
    s_touch_ready       = (Display_Gt911_Init() == DISPLAY_GT911_OK) ? 1U : 0U;
    s_touch_retry_count = 0U;
  }
  return DISPLAY_OK;
#endif
}

/*
 * 轮询 KEY0 并做简易去抖，检测到按下边沿时切换隐藏页。
 * 非隐藏页按下：记忆当前页并弹出隐藏页；隐藏页按下：返回原页面。
 */
Display_Result_t Display_PollKey(void)
{
#if DISPLAY_USE_LVGL_BACKEND
  return (s_display_ready != 0U) ? DISPLAY_OK : DISPLAY_NOT_READY;
#else
  uint8_t raw;

  if (s_display_ready == 0U) { return DISPLAY_NOT_READY; }

  raw = Px4Lite_ButtonPressed(PX4LITE_BUTTON_KEY0);
  if (raw == s_key0_last_raw) {
    if (s_key0_count < DISPLAY_KEY_DEBOUNCE_CYCLES) { s_key0_count++; }
  } else {
    s_key0_last_raw = raw;
    s_key0_count    = 1U;
  }

  if ((s_key0_count >= DISPLAY_KEY_DEBOUNCE_CYCLES) && (raw != s_key0_stable)) {
    s_key0_stable = raw;
    if (raw != 0U) {
      if (s_current_page == DISPLAY_HMI_PAGE_HIDDEN) { return Display_SetHmiPage(s_page_before_hidden); }

      s_page_before_hidden = s_current_page;
      return Display_SetHmiPage(DISPLAY_HMI_PAGE_HIDDEN);
    }
  }

  return DISPLAY_OK;
#endif
}

/*
 * 根据触摸坐标处理变量写入或页面导航。
 */
Display_Result_t Display_HandleTouch(uint16_t x, uint16_t y, Display_HmiVariableId_t *id, uint32_t *value)
{
#if DISPLAY_USE_LVGL_BACKEND
  (void)x;
  (void)y;
  if (id != 0) { *id = DISPLAY_HMI_VAR_COUNT; }
  if (value != 0) { *value = 0U; }
  return (s_display_ready != 0U) ? DISPLAY_OK : DISPLAY_NOT_READY;
#else
  Display_NavTouch_t nav_touch;
  const Display_HmiVariableConfig_t *slider;

  if (s_display_ready == 0U) { return DISPLAY_NOT_READY; }

  nav_touch = Display_GetNavTouch(x, y);

  if (Display_IsMotorEstopTouch(x, y) != 0U) {
    if (id != 0) { *id = DISPLAY_HMI_VAR_COUNT; }
    if (value != 0) { *value = 0U; }
    return Display_MotorEmergencyStop();
  }

  slider = Display_FindMotorSliderAt(x, y);
  if (slider != 0) { return Display_HandleMotorSliderTouch(slider, y, id, value); }

  if (id != 0) { *id = DISPLAY_HMI_VAR_COUNT; }
  if (value != 0) { *value = 0U; }

  return Display_HandleNavTouch(nav_touch);
#endif
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
