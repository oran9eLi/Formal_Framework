/**
 * @file app_message_log.h
 * @brief 业务层结构化消息日志：共享事件编号的兼容别名和本机状态观察 API。
 *
 * @details
 * message_id 为与显示无关的协议枚举(append-only 契约)，供本机屏幕、远端同构显示与
 * PC 监控共用。0~37 保留历史 Display_LogMsg_t 编号；新增编号由 Framework 共享目录维护。
 */
#ifndef APP_MESSAGE_LOG_H
#define APP_MESSAGE_LOG_H

#include <stdint.h>
#include "px4lite_log_message_ids.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 消息日志条目类型(协议枚举，append-only：只许尾部追加，不许重排/改号)。
 */
typedef enum {
  APP_LOGMSG_SYSTEM_START = PX4LITE_LOGMSG_SYSTEM_START,
  APP_LOGMSG_SELFCHECK_OK = PX4LITE_LOGMSG_SELFCHECK_OK,
  APP_LOGMSG_SELFCHECK_PART = PX4LITE_LOGMSG_SELFCHECK_PART,
  APP_LOGMSG_SELFCHECK_FAIL = PX4LITE_LOGMSG_SELFCHECK_FAIL,
  APP_LOGMSG_GPS_OK = PX4LITE_LOGMSG_GPS_OK,
  APP_LOGMSG_GPS_NOSIG = PX4LITE_LOGMSG_GPS_NOSIG,
  APP_LOGMSG_GPS_LOST = PX4LITE_LOGMSG_GPS_LOST,
  APP_LOGMSG_ATT_OK = PX4LITE_LOGMSG_ATT_OK,
  APP_LOGMSG_ATT_LOST = PX4LITE_LOGMSG_ATT_LOST,
  APP_LOGMSG_ENV_OK = PX4LITE_LOGMSG_ENV_OK,
  APP_LOGMSG_ENV_LOST = PX4LITE_LOGMSG_ENV_LOST,
  APP_LOGMSG_COMM_OK = PX4LITE_LOGMSG_COMM_OK,
  APP_LOGMSG_COMM_LOST = PX4LITE_LOGMSG_COMM_LOST,
  APP_LOGMSG_STORAGE_OK = PX4LITE_LOGMSG_STORAGE_OK,
  APP_LOGMSG_STORAGE_LOST = PX4LITE_LOGMSG_STORAGE_LOST,
  APP_LOGMSG_MOTOR_OK = PX4LITE_LOGMSG_MOTOR_OK,
  APP_LOGMSG_MOTOR_DISCONNECT = PX4LITE_LOGMSG_MOTOR_DISCONNECT,
  APP_LOGMSG_MOTOR_LOWPOWER = PX4LITE_LOGMSG_MOTOR_LOWPOWER,
  APP_LOGMSG_MOTOR1_FAIL = PX4LITE_LOGMSG_MOTOR1_FAIL,
  APP_LOGMSG_MOTOR2_FAIL = PX4LITE_LOGMSG_MOTOR2_FAIL,
  APP_LOGMSG_MOTOR3_FAIL = PX4LITE_LOGMSG_MOTOR3_FAIL,
  APP_LOGMSG_MOTOR4_FAIL = PX4LITE_LOGMSG_MOTOR4_FAIL,
  APP_LOGMSG_MOTOR_ALL_FAIL = PX4LITE_LOGMSG_MOTOR_ALL_FAIL,
  APP_LOGMSG_MOTOR_DEAD = PX4LITE_LOGMSG_MOTOR_DEAD,
  APP_LOGMSG_MOTOR_CHARGE = PX4LITE_LOGMSG_MOTOR_CHARGE,
  APP_LOGMSG_MAIN_CHARGE = PX4LITE_LOGMSG_MAIN_CHARGE,
  APP_LOGMSG_ALARM_ACTIVE = PX4LITE_LOGMSG_ALARM_ACTIVE,
  APP_LOGMSG_ALARM_NONE = PX4LITE_LOGMSG_ALARM_NONE,
  APP_LOGMSG_REMOTEID_OK = PX4LITE_LOGMSG_REMOTEID_OK,
  APP_LOGMSG_REMOTEID_LOST = PX4LITE_LOGMSG_REMOTEID_LOST,
  APP_LOGMSG_5G_OK = PX4LITE_LOGMSG_5G_OK,
  APP_LOGMSG_5G_LOST = PX4LITE_LOGMSG_5G_LOST,
  APP_LOGMSG_TAKEOFF_SENSOR_FAIL = PX4LITE_LOGMSG_TAKEOFF_SENSOR_FAIL,
  APP_LOGMSG_TAKEOFF_POWER_FAIL = PX4LITE_LOGMSG_TAKEOFF_POWER_FAIL,
  APP_LOGMSG_THROTTLE_POWER_FAIL = PX4LITE_LOGMSG_THROTTLE_POWER_FAIL,
  APP_LOGMSG_LANDING_FAIL = PX4LITE_LOGMSG_LANDING_FAIL,
  APP_LOGMSG_MOTOR_POWER_CUTOFF = PX4LITE_LOGMSG_MOTOR_POWER_CUTOFF,
  APP_LOGMSG_LANDING_THROTTLE_REJECT = PX4LITE_LOGMSG_LANDING_THROTTLE_REJECT,
  APP_LOGMSG_COUNT = PX4LITE_LOGMSG_COUNT
} App_LogMessageId_t;

#include "app_display_model.h"
#include "px4lite_types.h"

/** @brief 由采集任务启动时调用一次；重复调用无副作用且不清空共享环。 */
void App_MessageLogInit(uint32_t now_ms);

/** @brief 采集任务每200ms读取本机模块/环境/告警；持续1500ms确认并保存首次观察Uptime。 */
void App_MessageLogUpdate(uint32_t now_ms);
/** @brief 仅观察服务实际Start/Stop/Reset状态变化追加事件，operation为0/1/2。 */
void App_MessageLogObservation(uint8_t operation,uint32_t now_ms);

/** @brief 追加同步升速因姿态/环境传感器未在线而失败的事件日志。 */
void App_MessageLogPushTakeoffSensorFail(uint32_t now_ms);

/** @brief 追加同步升速因电机电池档位不允许输出而失败的事件日志。 */
void App_MessageLogPushTakeoffPowerFail(uint32_t now_ms);

/** @brief 追加滑块油门因电机电池档位不允许输出而被拒的事件日志。 */
void App_MessageLogPushThrottlePowerFail(uint32_t now_ms);

/** @brief 追加同步缓降未被 Control 接受的事件日志。 */
void App_MessageLogPushLandingFail(uint32_t now_ms);

/** @brief 兼容历史调用；仅实际输出完成撤收时记录，不允许以闭锁状态推断。 */
void App_MessageLogPushMotorPowerCutoff(uint32_t now_ms);

/** @brief 追加同步缓降进行中非零手动油门被拒的事件日志。 */
void App_MessageLogPushLandingThrottleReject(uint32_t now_ms);

/**
 * @brief 拷出当前本机日志快照(按时间旧→新)。
 * @return PX4LITE_OK 有数据；PX4LITE_NOT_READY 无条目；PX4LITE_INVALID_PARAM 空指针。
 */
Px4Lite_Result_t App_MessageLogCopy(App_DisplayLogSnapshot_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_MESSAGE_LOG_H */
