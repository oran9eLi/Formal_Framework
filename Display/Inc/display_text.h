#ifndef DISPLAY_TEXT_H
#define DISPLAY_TEXT_H

#include "display_gfx.h"

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
  DISPLAY_TEXT_COUNT
} Display_TextLabel_t;

/**
 * @brief 消息日志条目类型。每个值对应一条完整中文短语，
 *        由 Display_TextDrawLogMessage 用单字字模拼接绘制。
 */
typedef enum {
  DISPLAY_LOGMSG_SYSTEM_START = 0, /* 系统启动 */
  DISPLAY_LOGMSG_SELFCHECK_OK,     /* 自检通过 */
  DISPLAY_LOGMSG_SELFCHECK_PART,   /* 自检部分通过 */
  DISPLAY_LOGMSG_SELFCHECK_FAIL,   /* 自检未通过 */
  DISPLAY_LOGMSG_GPS_OK,           /* GPS正常 */
  DISPLAY_LOGMSG_GPS_NOSIG,        /* GPS无信号 */
  DISPLAY_LOGMSG_GPS_LOST,         /* GPS断开 */
  DISPLAY_LOGMSG_ATT_OK,           /* 姿态正常 */
  DISPLAY_LOGMSG_ATT_LOST,         /* 姿态断开 */
  DISPLAY_LOGMSG_ENV_OK,           /* 环境正常 */
  DISPLAY_LOGMSG_ENV_LOST,         /* 环境断开 */
  DISPLAY_LOGMSG_COMM_OK,          /* 通信正常 */
  DISPLAY_LOGMSG_COMM_LOST,        /* 通信断开 */
  DISPLAY_LOGMSG_STORAGE_OK,       /* 存储正常 */
  DISPLAY_LOGMSG_STORAGE_LOST,     /* 存储断开 */
  DISPLAY_LOGMSG_MOTOR_OK,         /* 电机正常 */
  DISPLAY_LOGMSG_MOTOR1_FAIL,      /* 1号电机故障 */
  DISPLAY_LOGMSG_MOTOR2_FAIL,      /* 2号电机故障 */
  DISPLAY_LOGMSG_MOTOR3_FAIL,      /* 3号电机故障 */
  DISPLAY_LOGMSG_MOTOR4_FAIL,      /* 4号电机故障 */
  DISPLAY_LOGMSG_MOTOR_ALL_FAIL,   /* 全部电机故障 */
  DISPLAY_LOGMSG_ALARM_ACTIVE,     /* 有告警 */
  DISPLAY_LOGMSG_ALARM_NONE,       /* 无告警 */
  DISPLAY_LOGMSG_COUNT
} Display_LogMsg_t;

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
