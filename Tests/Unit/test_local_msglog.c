/**
 * @file test_local_msglog.c
 * @brief 本机日志环形缓冲：序号单调、旧→新拷出、增量 DrainSince、容量回卷。
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "px4lite_local_msglog.h"

static int g_fail;
static void Eq(const char *n, uint32_t a, uint32_t e)
{ if (a != e) { printf("FAIL %s actual=%lu expected=%lu\n", n, (unsigned long)a, (unsigned long)e); g_fail++; } }

int main(void)
{
  Px4Lite_LogEntry_t buf[PX4LITE_LOCAL_LOG_CAP];
  uint16_t n, latest = 0U;
  uint16_t i;

  Px4Lite_LocalMsgLogReset();
  for (i = 0U; i < 3U; ++i) { Px4Lite_LocalMsgLogPush((uint16_t)(10U + i), 1000U + i, 0U, 0U, 0U, 0U); }

  n = Px4Lite_LocalMsgLogCopy(buf, PX4LITE_LOCAL_LOG_CAP, 0, &latest);
  Eq("copy n", n, 3U);
  Eq("latest", latest, 3U);
  Eq("seq0", buf[0].sequence, 1U);
  Eq("seq2", buf[2].sequence, 3U);
  Eq("mid0", buf[0].message_id, 10U);

  /* DrainSince(1) 只回 seq>1 的两条。 */
  n = Px4Lite_LocalMsgLogDrainSince(1U, buf, PX4LITE_LOCAL_LOG_CAP, &latest);
  Eq("drain n", n, 2U);
  Eq("drain first seq", buf[0].sequence, 2U);
  Eq("drain latest", latest, 3U);

  /* 容量回卷：再压 8 条(共 11)，只留最新 9，最旧应为 seq3。 */
  for (i = 0U; i < 8U; ++i) { Px4Lite_LocalMsgLogPush(20U, 2000U, 0U, 0U, 0U, 0U); }
  n = Px4Lite_LocalMsgLogCopy(buf, PX4LITE_LOCAL_LOG_CAP, 0, &latest);
  Eq("wrap n", n, PX4LITE_LOCAL_LOG_CAP);
  Eq("wrap oldest seq", buf[0].sequence, 3U);
  Eq("wrap latest", latest, 11U);

  if (g_fail == 0) { printf("PASS test_local_msglog\n"); return 0; }
  printf("FAIL test_local_msglog fails=%d\n", g_fail); return 1;
}
