/**
 * @file px4lite_local_msglog.c
 * @brief 本机结构化消息日志环形缓冲实现。
 *
 * @details
 * Business 任务写入，Display 与 Comm 任务读取，因此所有环形缓冲元数据和条目复制
 * 都必须在临界区内完成，避免多字节字段或条目内容被读到半更新状态。
 */
#include "px4lite_local_msglog.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

static Px4Lite_LogEntry_t s_ring[PX4LITE_LOCAL_LOG_CAP];
static uint16_t s_count;
static uint16_t s_head;     /**< 下一个写入位置。 */
static uint16_t s_next_seq; /**< 下一条序号，从 1 开始，跳过 0。 */
static uint16_t s_last_seq; /**< 最近一条已写序号，0 表示无日志。 */
static uint32_t s_version;

static uint8_t LocalMsgLog_SeqAfter(uint16_t sequence, uint16_t after_seq)
{
  uint16_t delta;

  if (sequence == 0U) { return 0U; }
  if (after_seq == 0U) { return 1U; }

  delta = (uint16_t)(sequence - after_seq);
  return ((delta != 0U) && (delta < 32768U)) ? 1U : 0U;
}

void Px4Lite_LocalMsgLogReset(void)
{
  taskENTER_CRITICAL();
  memset(s_ring, 0, sizeof(s_ring));
  s_count    = 0U;
  s_head     = 0U;
  s_next_seq = 1U;
  s_last_seq = 0U;
  s_version  = 0U;
  taskEXIT_CRITICAL();
}

void Px4Lite_LocalMsgLogPush(uint16_t message_id, uint32_t time_hhmmss,
                             uint16_t fault_code, uint8_t severity, uint8_t source_id, uint8_t active)
{
  Px4Lite_LogEntry_t *entry;

  taskENTER_CRITICAL();
  entry = &s_ring[s_head];
  memset(entry, 0, sizeof(*entry));
  entry->sequence    = s_next_seq;
  entry->message_id  = message_id;
  entry->time_hhmmss = time_hhmmss;
  entry->fault_code  = fault_code;
  entry->severity    = severity;
  entry->source_id   = source_id;
  entry->active      = active;

  s_last_seq = s_next_seq;
  s_next_seq++;
  if (s_next_seq == 0U) { s_next_seq = 1U; }

  s_head = (uint16_t)((s_head + 1U) % PX4LITE_LOCAL_LOG_CAP);
  if (s_count < PX4LITE_LOCAL_LOG_CAP) { s_count++; }
  s_version++;
  taskEXIT_CRITICAL();
}

uint16_t Px4Lite_LocalMsgLogCopy(Px4Lite_LogEntry_t *out, uint16_t cap,
                                 uint32_t *out_version, uint16_t *out_last_seq)
{
  uint16_t i;
  uint16_t n;
  uint16_t start;

  taskENTER_CRITICAL();
  n = (s_count < cap) ? s_count : cap;
  if (out_version != 0) { *out_version = s_version; }
  if (out_last_seq != 0) { *out_last_seq = s_last_seq; }
  if ((out == 0) || (n == 0U)) {
    taskEXIT_CRITICAL();
    return 0U;
  }

  start = (uint16_t)((s_head + PX4LITE_LOCAL_LOG_CAP - s_count) % PX4LITE_LOCAL_LOG_CAP);
  if (s_count > cap) { start = (uint16_t)((start + (s_count - cap)) % PX4LITE_LOCAL_LOG_CAP); }
  for (i = 0U; i < n; ++i) {
    out[i] = s_ring[(uint16_t)((start + i) % PX4LITE_LOCAL_LOG_CAP)];
  }
  taskEXIT_CRITICAL();
  return n;
}

uint16_t Px4Lite_LocalMsgLogDrainSince(uint16_t after_seq, Px4Lite_LogEntry_t *out, uint16_t max,
                                       uint16_t *out_latest_seq)
{
  uint16_t i;
  uint16_t start;
  uint16_t written = 0U;

  taskENTER_CRITICAL();
  if (out_latest_seq != 0) { *out_latest_seq = s_last_seq; }
  if ((out == 0) || (max == 0U) || (s_count == 0U)) {
    taskEXIT_CRITICAL();
    return 0U;
  }

  start = (uint16_t)((s_head + PX4LITE_LOCAL_LOG_CAP - s_count) % PX4LITE_LOCAL_LOG_CAP);
  for (i = 0U; (i < s_count) && (written < max); ++i) {
    const Px4Lite_LogEntry_t *entry = &s_ring[(uint16_t)((start + i) % PX4LITE_LOCAL_LOG_CAP)];
    if (LocalMsgLog_SeqAfter(entry->sequence, after_seq) != 0U) {
      out[written] = *entry;
      written++;
    }
  }
  taskEXIT_CRITICAL();
  return written;
}
