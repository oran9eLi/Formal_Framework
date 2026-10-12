/**
 * @file display_pages.h
 * @brief Display 层本机事件视图缓存契约。
 * @details 只消费 Business 完整快照；不生成事件、不清除生产日志或活动告警。
 */
#ifndef DISPLAY_PAGES_H
#define DISPLAY_PAGES_H

#include "display.h"
#include "app_display_model.h"
#include "px4lite_log_message_ids.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  DISPLAY_LOGMSG_SYSTEM_START = 0,
  DISPLAY_LOGMSG_SELFCHECK_OK,
  DISPLAY_LOGMSG_SELFCHECK_PART,
  DISPLAY_LOGMSG_SELFCHECK_FAIL,
  DISPLAY_LOGMSG_GPS_OK,
  DISPLAY_LOGMSG_GPS_NOSIG,
  DISPLAY_LOGMSG_GPS_LOST,
  DISPLAY_LOGMSG_ATT_OK,
  DISPLAY_LOGMSG_ATT_LOST,
  DISPLAY_LOGMSG_ENV_OK,
  DISPLAY_LOGMSG_ENV_LOST,
  DISPLAY_LOGMSG_COMM_OK,
  DISPLAY_LOGMSG_COMM_LOST,
  DISPLAY_LOGMSG_STORAGE_OK,
  DISPLAY_LOGMSG_STORAGE_LOST,
  DISPLAY_LOGMSG_MOTOR_OK,
  DISPLAY_LOGMSG_MOTOR_DISCONNECT,
  DISPLAY_LOGMSG_MOTOR_LOWPOWER,
  DISPLAY_LOGMSG_MOTOR1_FAIL,
  DISPLAY_LOGMSG_MOTOR2_FAIL,
  DISPLAY_LOGMSG_MOTOR3_FAIL,
  DISPLAY_LOGMSG_MOTOR4_FAIL,
  DISPLAY_LOGMSG_MOTOR_ALL_FAIL,
  DISPLAY_LOGMSG_MOTOR_DEAD,
  DISPLAY_LOGMSG_MOTOR_CHARGE,
  DISPLAY_LOGMSG_MAIN_CHARGE,
  DISPLAY_LOGMSG_ALARM_ACTIVE,
  DISPLAY_LOGMSG_ALARM_NONE,
  DISPLAY_LOGMSG_REMOTEID_OK,
  DISPLAY_LOGMSG_REMOTEID_LOST,
  DISPLAY_LOGMSG_5G_OK,
  DISPLAY_LOGMSG_5G_LOST,
  DISPLAY_LOGMSG_TAKEOFF_SENSOR_FAIL = 32,
  DISPLAY_LOGMSG_TAKEOFF_POWER_FAIL = 33,
  DISPLAY_LOGMSG_THROTTLE_POWER_FAIL = 34,
  DISPLAY_LOGMSG_LANDING_FAIL = 35,
  DISPLAY_LOGMSG_MOTOR_POWER_CUTOFF = 36,
  DISPLAY_LOGMSG_LANDING_THROTTLE_REJECT = 37,
  DISPLAY_LOGMSG_COUNT = PX4LITE_LOGMSG_COUNT
} Display_LogMsg_t;

/** @brief 单条事件的显示副本；时间为开机相对 HHMMSS，严重度保留真实值。 */
typedef struct {
  Display_LogMsg_t msg;      /**< 共享消息编号，旧编号保持兼容。 */
  uint32_t time_hhmmss;      /**< 事件首次观察/执行时刻，Uptime HHMMSS。 */
  uint16_t sequence;         /**< 16 位回绕序号，0 保留。 */
  uint16_t fault_code;       /**< 关联故障码，0 表示没有。 */
  uint8_t severity;          /**< 0 INFO，1 WARN，2 及以上 ERROR。 */
  uint8_t source_id;         /**< 事件所属模块。 */
  uint8_t active;            /**< 故障发生为 1，解除为 0。 */
} Display_MessageLogEntry_t;

/** @brief 按完整快照替换视图；不对回绕序号作普通大小比较。仅显示任务调用。 */
void Display_PagesSetLogSnapshot(const App_DisplayLogSnapshot_t *snapshot);
/** @brief 兼容旧显示注入调用；生产事件应写入 Framework 日志环。 */
void Display_PagesPushLogMessage(Display_LogMsg_t msg, uint32_t time_hhmmss);
/** @brief 清空显示副本；不修改源日志和活动告警。 */
void Display_PagesClearLogMessages(void);
/** @brief 复制最近 max_count 条，旧到新；历史告警同样作为事件，alarm_valid固定为0。 */
uint16_t Display_PagesCopyLogMessages(Display_MessageLogEntry_t *entries, uint16_t max_count, Display_MessageLogEntry_t *alarm_entry, uint8_t *alarm_valid);
/** @brief 返回显示副本修订号，用于变化刷新。 */
uint32_t Display_PagesGetLogVersion(void);

#ifdef __cplusplus
}
#endif

#endif
