/**
 * @file px4lite_local_msglog.c
 * @brief 本机消息日志环形缓冲实现。无动态分配、无浮点。
 */
#include "px4lite_local_msglog.h"

#include <string.h>

static Px4Lite_LogEntry_t s_ring[PX4LITE_LOCAL_LOG_CAP];
static uint16_t s_count;
static uint16_t s_head;       /* 下一个写入位置 */
static uint16_t s_next_seq;   /* 下一条序号(从 1 起) */
static uint16_t s_last_seq;   /* 最近一条已写序号(0=无) */
static uint32_t s_version;

void Px4Lite_LocalMsgLogReset(void)
{
  memset(s_ring, 0, sizeof(s_ring));
  s_count = 0U; s_head = 0U; s_next_seq = 1U; s_last_seq = 0U; s_version = 0U;
}

void Px4Lite_LocalMsgLogPush(uint16_t message_id, uint32_t time_hhmmss,
                             uint16_t fault_code, uint8_t severity, uint8_t source_id, uint8_t active)
{
  Px4Lite_LogEntry_t *e = &s_ring[s_head];
  memset(e, 0, sizeof(*e));
  e->sequence    = s_next_seq;
  e->message_id  = message_id;
  e->time_hhmmss = time_hhmmss;
  e->fault_code  = fault_code;
  e->severity    = severity;
  e->source_id   = source_id;
  e->active      = active;

  s_last_seq = s_next_seq;
  s_next_seq++;
  if (s_next_seq == 0U) { s_next_seq = 1U; }
  s_head = (uint16_t)((s_head + 1U) % PX4LITE_LOCAL_LOG_CAP);
  if (s_count < PX4LITE_LOCAL_LOG_CAP) { s_count++; }
  s_version++;
}

uint16_t Px4Lite_LocalMsgLogCopy(Px4Lite_LogEntry_t *out, uint16_t cap,
                                 uint32_t *out_version, uint16_t *out_last_seq)
{
  uint16_t i;
  uint16_t n = (s_count < cap) ? s_count : cap;
  uint16_t start;

  if (out_version != 0) { *out_version = s_version; }
  if (out_last_seq != 0) { *out_last_seq = s_last_seq; }
  if ((out == 0) || (n == 0U)) { return 0U; }

  start = (uint16_t)((s_head + PX4LITE_LOCAL_LOG_CAP - s_count) % PX4LITE_LOCAL_LOG_CAP);
  /* 若 cap < count，丢最旧、保最新 n 条。 */
  if (s_count > cap) { start = (uint16_t)((start + (s_count - cap)) % PX4LITE_LOCAL_LOG_CAP); }
  for (i = 0U; i < n; ++i) { out[i] = s_ring[(uint16_t)((start + i) % PX4LITE_LOCAL_LOG_CAP)]; }
  return n;
}

uint16_t Px4Lite_LocalMsgLogDrainSince(uint16_t after_seq, Px4Lite_LogEntry_t *out, uint16_t max,
                                       uint16_t *out_latest_seq)
{
  uint16_t i;
  uint16_t start;
  uint16_t written = 0U;

  if (out_latest_seq != 0) { *out_latest_seq = s_last_seq; }
  if ((out == 0) || (max == 0U) || (s_count == 0U)) { return 0U; }

  start = (uint16_t)((s_head + PX4LITE_LOCAL_LOG_CAP - s_count) % PX4LITE_LOCAL_LOG_CAP);
  for (i = 0U; (i < s_count) && (written < max); ++i) {
    const Px4Lite_LogEntry_t *e = &s_ring[(uint16_t)((start + i) % PX4LITE_LOCAL_LOG_CAP)];
    if (e->sequence > after_seq) { out[written] = *e; written++; }
  }
  return written;
}
