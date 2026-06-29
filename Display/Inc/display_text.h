#ifndef DISPLAY_TEXT_H
#define DISPLAY_TEXT_H

#include "display_gfx.h"
#include "app_message_log.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  DISPLAY_TEXT_SELF_CHECK = 0,
  DISPLAY_TEXT_DATA_PAGE,
  DISPLAY_TEXT_ALARM_PAGE,
  DISPLAY_TEXT_NETWORK,
  DISPLAY_TEXT_LINK,
  DISPLAY_TEXT_PREV,
  DISPLAY_TEXT_NEXT,
  DISPLAY_TEXT_CHECK_PROGRESS,
  DISPLAY_TEXT_RESULT,
  DISPLAY_TEXT_SENSOR,
  DISPLAY_TEXT_POWER,
  DISPLAY_TEXT_ERROR_CODE,
  DISPLAY_TEXT_DATA_SUMMARY,
  DISPLAY_TEXT_MOTOR_PWM,
  DISPLAY_TEXT_ALARM_SUMMARY,
  DISPLAY_TXT_UPTIME,
  DISPLAY_TXT_SELFCHK,
  DISPLAY_TXT_BATTERY,
  DISPLAY_TXT_BATPCT,
  DISPLAY_TXT_FIX,
  DISPLAY_TXT_LAT,
  DISPLAY_TXT_LON,
  DISPLAY_TXT_ALT,
  DISPLAY_TXT_SPEED,
  DISPLAY_TXT_UTC,
  DISPLAY_TXT_ROLL,
  DISPLAY_TXT_PITCH,
  DISPLAY_TXT_YAW,
  DISPLAY_TXT_TEMP,
  DISPLAY_TXT_HUM,
  DISPLAY_TXT_PRESS,
  DISPLAY_TXT_ONLINE,
  DISPLAY_TXT_TX,
  DISPLAY_TXT_RX,
  DISPLAY_TXT_HB,
  DISPLAY_TXT_ACK,
  DISPLAY_TXT_LOSS,
  DISPLAY_TXT_MOTOR,
  DISPLAY_TXT_GAUGE,
  DISPLAY_TEXT_SELF_GNSS,
  DISPLAY_TEXT_SELF_STORAGE,
  DISPLAY_TEXT_SELF_LORA,
  DISPLAY_TEXT_SELF_POWER,
  DISPLAY_TEXT_SELF_ATTITUDE,
  DISPLAY_TEXT_SELF_ENV,
  DISPLAY_TEXT_SELF_5G,
  DISPLAY_TEXT_SELF_BUZZER,
  DISPLAY_TEXT_SELF_KEY,
  DISPLAY_TEXT_SELF_DEBUG,
  DISPLAY_TEXT_CODE,
  DISPLAY_TEXT_MODULE,
  DISPLAY_TEXT_REASON,
  DISPLAY_TEXT_FAILED_MODULES,
  DISPLAY_TITLE_SYSTEM,
  DISPLAY_TITLE_GNSS,
  DISPLAY_TITLE_GPS_INFO,
  DISPLAY_TEXT_MESSAGE_LOG,
  DISPLAY_TXT_STATUS,
  DISPLAY_TXT_SAT_COUNT,
  DISPLAY_LOG_SYSTEM_START,
  DISPLAY_LOG_SELF_CHECK_OK,
  DISPLAY_LOG_GPS_READY,
  DISPLAY_LOG_LINK_OK,
  DISPLAY_LOG_NO_ALARM,
  DISPLAY_TITLE_FLIGHT,
  DISPLAY_TITLE_AIRCRAFT,
  DISPLAY_TXT_DATE,
  DISPLAY_TXT_HDOP,
  DISPLAY_TEXT_COUNT
} Display_TextLabel_t;

/**
 * @brief 消息日志条目类型。已上提为业务级协议枚举 App_LogMessageId_t，
 *        此处别名 + 兼容宏，保持显示层既有 DISPLAY_LOGMSG_* 名称零改动。
 *        每个值对应一条完整中文短语，由 Display_TextDrawLogMessage 用单字字模拼接绘制。
 */
typedef App_LogMessageId_t Display_LogMsg_t;

#define DISPLAY_LOGMSG_SYSTEM_START   APP_LOGMSG_SYSTEM_START
#define DISPLAY_LOGMSG_SELFCHECK_OK   APP_LOGMSG_SELFCHECK_OK
#define DISPLAY_LOGMSG_SELFCHECK_PART APP_LOGMSG_SELFCHECK_PART
#define DISPLAY_LOGMSG_SELFCHECK_FAIL APP_LOGMSG_SELFCHECK_FAIL
#define DISPLAY_LOGMSG_GPS_OK         APP_LOGMSG_GPS_OK
#define DISPLAY_LOGMSG_GPS_NOSIG      APP_LOGMSG_GPS_NOSIG
#define DISPLAY_LOGMSG_GPS_LOST       APP_LOGMSG_GPS_LOST
#define DISPLAY_LOGMSG_ATT_OK         APP_LOGMSG_ATT_OK
#define DISPLAY_LOGMSG_ATT_LOST       APP_LOGMSG_ATT_LOST
#define DISPLAY_LOGMSG_ENV_OK         APP_LOGMSG_ENV_OK
#define DISPLAY_LOGMSG_ENV_LOST       APP_LOGMSG_ENV_LOST
#define DISPLAY_LOGMSG_COMM_OK        APP_LOGMSG_COMM_OK
#define DISPLAY_LOGMSG_COMM_LOST      APP_LOGMSG_COMM_LOST
#define DISPLAY_LOGMSG_STORAGE_OK     APP_LOGMSG_STORAGE_OK
#define DISPLAY_LOGMSG_STORAGE_LOST   APP_LOGMSG_STORAGE_LOST
#define DISPLAY_LOGMSG_MOTOR_OK       APP_LOGMSG_MOTOR_OK
#define DISPLAY_LOGMSG_MOTOR_DISCONNECT APP_LOGMSG_MOTOR_DISCONNECT
#define DISPLAY_LOGMSG_MOTOR_LOWPOWER   APP_LOGMSG_MOTOR_LOWPOWER
#define DISPLAY_LOGMSG_MOTOR1_FAIL    APP_LOGMSG_MOTOR1_FAIL
#define DISPLAY_LOGMSG_MOTOR2_FAIL    APP_LOGMSG_MOTOR2_FAIL
#define DISPLAY_LOGMSG_MOTOR3_FAIL    APP_LOGMSG_MOTOR3_FAIL
#define DISPLAY_LOGMSG_MOTOR4_FAIL    APP_LOGMSG_MOTOR4_FAIL
#define DISPLAY_LOGMSG_MOTOR_ALL_FAIL APP_LOGMSG_MOTOR_ALL_FAIL
#define DISPLAY_LOGMSG_MOTOR_DEAD     APP_LOGMSG_MOTOR_DEAD
#define DISPLAY_LOGMSG_MOTOR_CHARGE   APP_LOGMSG_MOTOR_CHARGE
#define DISPLAY_LOGMSG_MAIN_CHARGE    APP_LOGMSG_MAIN_CHARGE
#define DISPLAY_LOGMSG_ALARM_ACTIVE   APP_LOGMSG_ALARM_ACTIVE
#define DISPLAY_LOGMSG_ALARM_NONE     APP_LOGMSG_ALARM_NONE
#define DISPLAY_LOGMSG_COUNT          APP_LOGMSG_COUNT

/**
 * @brief       绘制内置中文标签
 * @param       x: 标签左上角 X 坐标
 * @param       y: 标签左上角 Y 坐标
 * @param       label: 标签 ID
 * @param       color: RGB565 颜色值
 * @retval      Display_GfxResult_t: 绘制结果
 */
Display_GfxResult_t Display_TextDrawLabel(uint16_t x, uint16_t y, Display_TextLabel_t label, uint16_t color);

Display_GfxResult_t Display_TextDrawLabelBold(uint16_t x, uint16_t y, Display_TextLabel_t label, uint16_t color);

Display_GfxResult_t Display_TextDrawCompanyTitle(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief       绘制电机控制页标题“电机控制”。
 * @param       x: 左上角 X 坐标
 * @param       y: 左上角 Y 坐标
 * @param       color: RGB565 颜色值
 * @retval      Display_GfxResult_t: 绘制结果
 */
Display_GfxResult_t Display_TextDrawMotorTitle(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief       绘制屏幕急停按钮文字“急停”。
 * @param       x: 左上角 X 坐标
 * @param       y: 左上角 Y 坐标
 * @param       color: RGB565 颜色值
 * @retval      Display_GfxResult_t: 绘制结果
 */
Display_GfxResult_t Display_TextDrawEstop(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief       绘制双电池来源标签：主控/电机 + 电压/电池(共4字，16px高)。
 * @param       x: 左上角 X 坐标
 * @param       y: 左上角 Y 坐标
 * @param       is_motor: 0=主控(ADC1)，非0=电机(ADC2)
 * @param       is_battery: 0=电压，非0=电池(电量)
 * @param       color: RGB565 颜色值
 * @retval      Display_GfxResult_t: 绘制结果
 */
Display_GfxResult_t Display_TextDrawBatteryLabel(uint16_t x, uint16_t y, uint8_t is_motor, uint8_t is_battery, uint16_t color);

Display_GfxResult_t Display_TextDrawVoltageLabel(uint16_t x, uint16_t y, uint16_t color);

/* LoRa 连接页中文词标签（16px 高，单字字模拼接）。 */
Display_GfxResult_t Display_TextDrawNodeLabel(uint16_t x, uint16_t y, uint16_t color);       /* 节点 */
Display_GfxResult_t Display_TextDrawConnectLabel(uint16_t x, uint16_t y, uint16_t color);    /* 连接 */
Display_GfxResult_t Display_TextDrawDisconnectLabel(uint16_t x, uint16_t y, uint16_t color); /* 断开 */
Display_GfxResult_t Display_TextDrawSelectTitle(uint16_t x, uint16_t y, uint16_t color);     /* 当前选择 */
Display_GfxResult_t Display_TextDrawWaitConnectLabel(uint16_t x, uint16_t y, uint16_t color);/* 待连接 */
Display_GfxResult_t Display_TextDrawConnectedLabel(uint16_t x, uint16_t y, uint16_t color);  /* 已连接 */
Display_GfxResult_t Display_TextDrawLastCommLabel(uint16_t x, uint16_t y, uint16_t color);   /* 上次通信 */

Display_GfxResult_t Display_TextDrawRawBitmap(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t bytes_per_row, const uint8_t *data, uint16_t color);

/**
 * @brief       绘制一条消息日志短语（单字字模拼接，字高 16px）
 * @param       x: 左上角 X 坐标
 * @param       y: 左上角 Y 坐标
 * @param       msg: 消息条目类型
 * @param       color: RGB565 颜色值
 * @retval      Display_GfxResult_t: 绘制结果
 */
Display_GfxResult_t Display_TextDrawLogMessage(uint16_t x, uint16_t y, Display_LogMsg_t msg, uint16_t color);

/**
 * @brief       获取内置中文标签宽度
 * @param       label: 标签 ID
 * @retval      uint16_t: 标签宽度，单位像素
 */
uint16_t Display_TextGetLabelWidth(Display_TextLabel_t label);

uint16_t Display_TextGetCompanyTitleWidth(void);

#ifdef __cplusplus
}
#endif

#endif
