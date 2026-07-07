/**
 * @file px4lite_local_msglog.h
 * @brief 本机消息日志环形缓冲(Framework 单一真值源)。
 *
 * @details
 * 业务层 app_message_log 负责去抖/生成并 Push；显示层经 App_MessageLogCopy 读取；
 * 远端同步 TX 经 DrainSince 取增量。Framework 层不依赖 Business。
 */
#ifndef PX4LITE_LOCAL_MSGLOG_H
#define PX4LITE_LOCAL_MSGLOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PX4LITE_LOCAL_LOG_CAP 9U /**< 本机日志容量；须与 Business APP_DISPLAY_LOG_CAP 一致。 */

/** @brief 一条结构化日志条目(与 Business App_DisplayLogEntry_t 字段同构)。 */
typedef struct {
  uint16_t sequence;     /**< 单调序号(从 1 起，跳过 0)。 */
  uint16_t message_id;   /**< 消息编号(App_LogMessageId_t 取值)。 */
  uint32_t time_hhmmss;  /**< 事件时间 HHMMSS。 */
  uint16_t fault_code;   /**< 关联故障码，0 无。 */
  uint8_t severity;      /**< 严重度。 */
  uint8_t source_id;     /**< 来源 ID。 */
  uint8_t active;        /**< 告警类触发/恢复标志。 */
  uint8_t reserved[3];   /**< 对齐保留。 */
} Px4Lite_LogEntry_t;

/** @brief 复位缓冲与序号。 */
void Px4Lite_LocalMsgLogReset(void);

/** @brief 追加一条日志(序号自增)。 */
void Px4Lite_LocalMsgLogPush(uint16_t message_id, uint32_t time_hhmmss,
                             uint16_t fault_code, uint8_t severity, uint8_t source_id, uint8_t active);

/**
 * @brief 拷出全部条目(旧→新)。
 * @return 条目数(<= cap)。out_version/out_last_seq 可空。
 */
uint16_t Px4Lite_LocalMsgLogCopy(Px4Lite_LogEntry_t *out, uint16_t cap,
                                 uint32_t *out_version, uint16_t *out_last_seq);

/**
 * @brief 拷出 sequence 大于 after_seq 的条目(旧→新)，供增量发送。
 * @param[out] out_latest_seq 当前最新序号(无条目为 0)，可空。
 * @return 拷出条数(<= max)。
 */
uint16_t Px4Lite_LocalMsgLogDrainSince(uint16_t after_seq, Px4Lite_LogEntry_t *out, uint16_t max,
                                       uint16_t *out_latest_seq);

#ifdef __cplusplus
}
#endif

#endif /* PX4LITE_LOCAL_MSGLOG_H */
