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
  APP_LOGMSG_REMOTEID_OK,      /* Remote ID正常 */
  APP_LOGMSG_REMOTEID_LOST,    /* Remote ID断开 */
  APP_LOGMSG_COUNT
} App_LogMessageId_t;

#include "app_display_model.h"
#include "px4lite_types.h"

/** @brief 复位日志缓冲与去抖/档位状态。 */
void App_MessageLogInit(uint32_t now_ms);

/** @brief 每周期推进：读本机模块/环境/告警，按去抖规则追加日志。 */
void App_MessageLogUpdate(uint32_t now_ms);

/**
 * @brief 拷出当前本机日志快照(按时间旧→新)。
 * @return PX4LITE_OK 有数据；PX4LITE_NOT_READY 无条目；PX4LITE_INVALID_PARAM 空指针。
 */
Px4Lite_Result_t App_MessageLogCopy(App_DisplayLogSnapshot_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_MESSAGE_LOG_H */
