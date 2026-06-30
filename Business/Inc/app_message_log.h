/**
 * @file app_message_log.h
 * @brief Business层结构化消息日志生产器接口。
 *
 * @details
 * 本模块按业务状态变化生成稳定的消息编号，并写入 Framework 的固定环形日志
 * `px4lite_local_msglog`。Display 层可以保留自己的屏幕显示缓存，但远程日志、Tunnel
 * 中继和后续 5G/RemoteID 业务日志统一以本模块生成的消息为准。
 */

#ifndef APP_MESSAGE_LOG_H
#define APP_MESSAGE_LOG_H

#include <stdint.h>
#include "px4lite_local_msglog.h"
#include "px4lite_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 业务消息编号，append-only，不允许重排已有值。
 */
typedef enum {
  APP_LOGMSG_SYSTEM_START = 0, /**< 系统启动。 */
  APP_LOGMSG_IMU_LEVEL_WAIT,   /**< 开机静止水平基准等待中。 */
  APP_LOGMSG_IMU_LEVEL_OK,     /**< 开机静止水平基准完成。 */
  APP_LOGMSG_SELFCHECK_OK,     /**< 自检通过。 */
  APP_LOGMSG_SELFCHECK_PART,   /**< 自检部分通过。 */
  APP_LOGMSG_SELFCHECK_FAIL,   /**< 自检失败。 */
  APP_LOGMSG_GPS_OK,           /**< GNSS 正常。 */
  APP_LOGMSG_GPS_NOSIG,        /**< GNSS 无定位或信号弱。 */
  APP_LOGMSG_GPS_LOST,         /**< GNSS 断开。 */
  APP_LOGMSG_ATT_OK,           /**< 姿态正常。 */
  APP_LOGMSG_ATT_LOST,         /**< 姿态断开。 */
  APP_LOGMSG_ENV_OK,           /**< 环境/气压正常。 */
  APP_LOGMSG_ENV_LOST,         /**< 环境/气压断开。 */
  APP_LOGMSG_COMM_OK,          /**< 通信正常。 */
  APP_LOGMSG_COMM_LOST,        /**< 通信断开。 */
  APP_LOGMSG_STORAGE_OK,       /**< 存储正常。 */
  APP_LOGMSG_STORAGE_LOST,     /**< 存储断开。 */
  APP_LOGMSG_MOTOR_OK,         /**< 电机供电正常。 */
  APP_LOGMSG_MOTOR_DISCONNECT, /**< 电机电池未连接。 */
  APP_LOGMSG_MOTOR_LOWPOWER,   /**< 电机供电不足。 */
  APP_LOGMSG_MOTOR1_FAIL,      /**< 1号电机故障。 */
  APP_LOGMSG_MOTOR2_FAIL,      /**< 2号电机故障。 */
  APP_LOGMSG_MOTOR3_FAIL,      /**< 3号电机故障。 */
  APP_LOGMSG_MOTOR4_FAIL,      /**< 4号电机故障。 */
  APP_LOGMSG_MOTOR_ALL_FAIL,   /**< 全部电机故障。 */
  APP_LOGMSG_MOTOR_DEAD,       /**< 电机电池严重亏电。 */
  APP_LOGMSG_MOTOR_CHARGE,     /**< 电机电池需充电。 */
  APP_LOGMSG_MAIN_CHARGE,      /**< 主控电池需充电。 */
  APP_LOGMSG_ALARM_ACTIVE,     /**< 有活动告警。 */
  APP_LOGMSG_ALARM_NONE,       /**< 无活动告警。 */
  APP_LOGMSG_COUNT             /**< 消息编号数量。 */
} App_LogMessageId_t;

/**
 * @brief 初始化业务消息日志生产器和 Framework 本地日志 ring。
 *
 * @param[in] now_ms 当前系统时间，单位：ms。
 */
void App_MessageLogInit(uint32_t now_ms);

/**
 * @brief 推进一次业务消息日志状态机。
 *
 * @param[in] now_ms 当前系统时间，单位：ms。
 */
void App_MessageLogUpdate(uint32_t now_ms);

/**
 * @brief 复制当前本地业务消息日志。
 *
 * @param[out] entries 输出日志数组，不能为 NULL。
 * @param[in] capacity 输出数组容量。
 * @param[out] version 日志版本号，可为 NULL。
 * @param[out] last_seq 最新日志序号，可为 NULL。
 *
 * @return 复制到输出数组的条目数。
 */
uint16_t App_MessageLogCopy(Px4Lite_LogEntry_t *entries, uint16_t capacity, uint32_t *version, uint16_t *last_seq);

uint32_t App_MessageLogGetVersion(void);

#ifdef __cplusplus
}
#endif

#endif
