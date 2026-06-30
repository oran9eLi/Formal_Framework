/**
 * @file test_remote_tunnel_log.c
 * @brief 日志增量 TUNNEL(0x8002) 打包/解包 round-trip 与心跳。
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "px4lite_remote_tunnel.h"

static int g_fail;
static void Eq(const char *n, uint32_t a, uint32_t e)
{ if (a != e) { printf("FAIL %s actual=%lu expected=%lu\n", n, (unsigned long)a, (unsigned long)e); g_fail++; } }

int main(void)
{
  Px4Lite_LogEntry_t in[2];
  Px4Lite_LogEntry_t out[PX4LITE_TUNNEL_LOG_MAX_ENTRIES];
  uint8_t buf[PX4LITE_TUNNEL_LOG_MAX_BYTES];
  uint8_t cnt = 0xFFU;
  uint16_t latest = 0U, len;

  memset(in, 0, sizeof(in));
  in[0].sequence = 7U; in[0].message_id = 4U; in[0].time_hhmmss = 123456U; in[0].severity = 2U;
  in[1].sequence = 8U; in[1].message_id = 12U; in[1].time_hhmmss = 235959U; in[1].severity = 1U;

  len = Px4Lite_PackMessageLog(in, 2U, 8U, buf, sizeof(buf));
  Eq("len", len, PX4LITE_TUNNEL_LOG_HEADER_BYTES + 2U * PX4LITE_TUNNEL_LOG_ENTRY_BYTES);
  Eq("hdr count", buf[2], 2U);

  Eq("unpack ok", Px4Lite_UnpackMessageLog(buf, len, out, PX4LITE_TUNNEL_LOG_MAX_ENTRIES, &cnt, &latest), PX4LITE_OK);
  Eq("count", cnt, 2U);
  Eq("latest", latest, 8U);
  Eq("e0 seq", out[0].sequence, 7U);
  Eq("e0 mid", out[0].message_id, 4U);
  Eq("e0 time", out[0].time_hhmmss, 123456U);
  Eq("e1 time", out[1].time_hhmmss, 235959U);
  Eq("e1 sev", out[1].severity, 1U);

  /* 心跳：count=0，只带 latest_seq。 */
  len = Px4Lite_PackMessageLog(0, 0U, 42U, buf, sizeof(buf));
  Eq("hb len", len, PX4LITE_TUNNEL_LOG_HEADER_BYTES);
  cnt = 0xFFU; latest = 0U;
  Eq("hb unpack", Px4Lite_UnpackMessageLog(buf, len, out, PX4LITE_TUNNEL_LOG_MAX_ENTRIES, &cnt, &latest), PX4LITE_OK);
  Eq("hb count", cnt, 0U);
  Eq("hb latest", latest, 42U);

  /* 截断拒绝：声称 2 条但长度不足。 */
  { uint8_t bad[5] = {1U,0U,2U,0U,0U};
    Eq("short rejected", Px4Lite_UnpackMessageLog(bad, 5U, out, PX4LITE_TUNNEL_LOG_MAX_ENTRIES, &cnt, &latest), PX4LITE_INVALID_PARAM); }

  if (g_fail == 0) { printf("PASS test_remote_tunnel_log\n"); return 0; }
  printf("FAIL test_remote_tunnel_log fails=%d\n", g_fail); return 1;
}
