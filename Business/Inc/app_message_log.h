/**
 * @file app_message_log.h
 * @brief 业务层结构化消息日志：协议级消息枚举 + 生产者 API。
 *
 * @details
 * message_id 为与显示无关的协议枚举(append-only 契约)，供本机屏幕、远端同构显示与
 * PC 监控共用。取值镜像历史 Display_LogMsg_t，二者逐一相等。
 */
#ifndef APP_MESSAGE_LOG_H
#define APP_MESSAGE_LOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 消息日志条目类型(协议枚举，append-only：只许尾部追加，不许重排/改号)。
 */
typedef enum {
  APP_LOGMSG_SYSTEM_START = 0, /* 系统启动 */
  APP_LOGMSG_SELFCHECK_OK,     /* 自检通过 */
  APP_LOGMSG_SELFCHECK_PART,   /* 自检部分通过 */
  APP_LOGMSG_SELFCHECK_FAIL,   /* 自检未通过 */
  APP_LOGMSG_GPS_OK,           /* GPS正常 */
  APP_LOGMSG_GPS_NOSIG,        /* GPS无信号 */
  APP_LOGMSG_GPS_LOST,         /* GPS断开 */
  APP_LOGMSG_ATT_OK,           /* 姿态正常 */
  APP_LOGMSG_ATT_LOST,         /* 姿态断开 */
  APP_LOGMSG_ENV_OK,           /* 环境正常 */
  APP_LOGMSG_ENV_LOST,         /* 环境断开 */
  APP_LOGMSG_COMM_OK,          /* 通信正常 */
  APP_LOGMSG_COMM_LOST,        /* 通信断开 */
  APP_LOGMSG_STORAGE_OK,       /* 存储正常 */
  APP_LOGMSG_STORAGE_LOST,     /* 存储断开 */
  APP_LOGMSG_MOTOR_OK,         /* 电机正常 */
  APP_LOGMSG_MOTOR_DISCONNECT, /* 电机断开 */
  APP_LOGMSG_MOTOR_LOWPOWER,   /* 电机供电不足 */
  APP_LOGMSG_MOTOR1_FAIL,      /* 1号电机故障 */
  APP_LOGMSG_MOTOR2_FAIL,      /* 2号电机故障 */
  APP_LOGMSG_MOTOR3_FAIL,      /* 3号电机故障 */
  APP_LOGMSG_MOTOR4_FAIL,      /* 4号电机故障 */
  APP_LOGMSG_MOTOR_ALL_FAIL,   /* 全部电机故障 */
  APP_LOGMSG_MOTOR_DEAD,       /* 电机电池没电 */
  APP_LOGMSG_MOTOR_CHARGE,     /* 电机电池需充电 */
  APP_LOGMSG_MAIN_CHARGE,      /* 主控电池需充电 */
  APP_LOGMSG_ALARM_ACTIVE,     /* 有告警 */
  APP_LOGMSG_ALARM_NONE,       /* 无告警 */
  APP_LOGMSG_COUNT
} App_LogMessageId_t;

#ifdef __cplusplus
}
#endif

#endif /* APP_MESSAGE_LOG_H */
