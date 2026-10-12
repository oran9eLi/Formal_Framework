/**
 * @file px4lite_local_msglog.h
 * @brief 本机消息日志环形缓冲(Framework 单一真值源)。
 *
 * @details
 * 业务层 app_message_log 观察稳定状态，Control/Attitude 直接记录操作事实；显示经 App_MessageLogCopy 读取；
 * 远端同步 TX 经 DrainSince 取增量。Framework 层不依赖 Business。
 */
#ifndef PX4LITE_LOCAL_MSGLOG_H
#define PX4LITE_LOCAL_MSGLOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PX4LITE_LOCAL_LOG_CAP 32U /**< 本机日志容量；须与 Business APP_DISPLAY_LOG_CAP 一致。 */

/** @brief 一条结构化日志条目(与 Business App_DisplayLogEntry_t 字段同构)。 */
typedef struct {
  uint16_t sequence;     /**< 会话内循环序号(从 1 起，65535 后回到 1)。 */
  uint16_t message_id;   /**< 消息编号(Px4Lite_LogMessageId_t 取值)。 */
  uint32_t time_hhmmss;  /**< 事件首次观察 Uptime HHMMSS；100 小时循环。 */
  uint16_t fault_code;   /**< 关联故障码，0 无。 */
  uint8_t severity;      /**< 严重度：0 INFO、1 WARN、2 ERROR。 */
  uint8_t source_id;     /**< 来源 ID。 */
  uint8_t active;        /**< 告警类触发/恢复标志。 */
  uint8_t reserved[3];   /**< 对齐保留。 */
} Px4Lite_LogEntry_t;

/** @brief 仅系统启动前显式复位；任务初始化/恢复不得清空历史。 */
void Px4Lite_LocalMsgLogReset(void);

/** @brief 追加一条日志(序号自增)。 */
void Px4Lite_LocalMsgLogPush(uint16_t message_id, uint32_t time_hhmmss,
                             uint16_t fault_code, uint8_t severity, uint8_t source_id, uint8_t active);

/** @brief 按默认严重度追加事实；now_ms 是首次事实 Uptime，必须在任务上下文调用。 */
void Px4Lite_LocalMsgLogEvent(uint16_t message_id, uint32_t now_ms, uint16_t fault_code, uint8_t source_id, uint8_t active);

/**
 * @brief 拷出全部条目(旧→新)。
 * @return 条目数(<= cap)。out_version/out_last_seq 可空。
 */
uint16_t Px4Lite_LocalMsgLogCopy(Px4Lite_LogEntry_t *out, uint16_t cap,
                                 uint32_t *out_version, uint16_t *out_last_seq);

/**
 * @brief 拷出游标之后条目(旧→新)；游标已覆盖或为0则从保留的最旧条目开始，不做普通大小比较。
 * @param[out] out_latest_seq 当前最新序号(无条目为 0)，可空。
 * @return 拷出条数(<= max)。
 */
uint16_t Px4Lite_LocalMsgLogDrainSince(uint16_t after_seq, Px4Lite_LogEntry_t *out, uint16_t max,
                                       uint16_t *out_latest_seq);

#ifdef __cplusplus
}
#endif

#endif /* PX4LITE_LOCAL_MSGLOG_H */
