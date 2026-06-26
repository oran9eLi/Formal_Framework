/**
 * @file test_remote_tunnel_alarm.c
 * @brief 验证告警表 TUNNEL 打包/解包 round-trip 与签名。
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "px4lite_remote_tunnel.h"

static int g_fail;

static void ExpectU32(const char *name, uint32_t a, uint32_t e)
{
  if (a != e) { printf("FAIL %s actual=%lu expected=%lu\n", name, (unsigned long)a, (unsigned long)e); g_fail++; }
}

static Px4Lite_AlarmRecord_t MakeRow(uint16_t src, uint16_t code, uint8_t sev, uint8_t active, uint32_t raised_ms)
{
  Px4Lite_AlarmRecord_t r;
  memset(&r, 0, sizeof(r));
  r.source_id = src; r.fault_code = code; r.severity = (Px4Lite_AlarmSeverity_t)sev;
  r.active = active; r.raised_ms = raised_ms;
  return r;
}

static void TestRoundTripTwoActiveRows(void)
{
  Px4Lite_AlarmRecord_t in[3];
  uint8_t buf[PX4LITE_TUNNEL_ALARM_MAX_BYTES];
  Px4Lite_AlarmRecord_t out[PX4LITE_TUNNEL_ALARM_MAX_ROWS];
  uint8_t out_count = 0xFFU, out_ver = 0U;
  uint16_t len;
  uint32_t now = 10000U;

  in[0] = MakeRow(3U, 0x0102U, 2U, 1U, 7000U); /* age=3s */
  in[1] = MakeRow(5U, 0x0000U, 0U, 0U, 0U);    /* inactive: 不发 */
  in[2] = MakeRow(9U, 0x00ABU, 1U, 1U, 9500U); /* age=0s(向下取整) */

  len = Px4Lite_PackAlarmTable(in, 3U, 7U, now, buf, sizeof(buf));
  ExpectU32("len two rows", len, PX4LITE_TUNNEL_ALARM_HEADER_BYTES + 2U * PX4LITE_TUNNEL_ALARM_ROW_BYTES);
  ExpectU32("header ver", buf[0], 7U);
  ExpectU32("header count", buf[1], 2U);

  ExpectU32("unpack ok", Px4Lite_UnpackAlarmTable(buf, len, now, out, PX4LITE_TUNNEL_ALARM_MAX_ROWS, &out_count, &out_ver), PX4LITE_OK);
  ExpectU32("out count", out_count, 2U);
  ExpectU32("out ver", out_ver, 7U);
  ExpectU32("row0 src", out[0].source_id, 3U);
  ExpectU32("row0 code", out[0].fault_code, 0x0102U);
  ExpectU32("row0 sev", (uint32_t)out[0].severity, 2U);
  ExpectU32("row0 active", out[0].active, 1U);
  ExpectU32("row0 raised", out[0].raised_ms, now - 3000U);
  ExpectU32("row1 src", out[1].source_id, 9U);
  ExpectU32("row1 code", out[1].fault_code, 0x00ABU);
}

static void TestEmptyTable(void)
{
  Px4Lite_AlarmRecord_t in[1];
  uint8_t buf[PX4LITE_TUNNEL_ALARM_MAX_BYTES];
  Px4Lite_AlarmRecord_t out[PX4LITE_TUNNEL_ALARM_MAX_ROWS];
  uint8_t out_count = 0xFFU, out_ver = 0xFFU;
  uint16_t len;

  in[0] = MakeRow(1U, 1U, 1U, 0U, 0U); /* inactive */
  len = Px4Lite_PackAlarmTable(in, 1U, 0U, 1000U, buf, sizeof(buf));
  ExpectU32("empty len", len, PX4LITE_TUNNEL_ALARM_HEADER_BYTES);
  ExpectU32("empty header count", buf[1], 0U);
  ExpectU32("empty unpack ok", Px4Lite_UnpackAlarmTable(buf, len, 1000U, out, PX4LITE_TUNNEL_ALARM_MAX_ROWS, &out_count, &out_ver), PX4LITE_OK);
  ExpectU32("empty out count", out_count, 0U);
}

static void TestSignatureIgnoresAge(void)
{
  Px4Lite_AlarmRecord_t a = MakeRow(3U, 0x0102U, 2U, 1U, 1000U);
  Px4Lite_AlarmRecord_t b = MakeRow(3U, 0x0102U, 2U, 1U, 8000U); /* 不同 raised_ms */
  Px4Lite_AlarmRecord_t c = MakeRow(3U, 0x0103U, 2U, 1U, 1000U); /* 不同 code */
  ExpectU32("sig age-invariant", Px4Lite_AlarmTableSignature(&a, 1U), Px4Lite_AlarmTableSignature(&b, 1U));
  if (Px4Lite_AlarmTableSignature(&a, 1U) == Px4Lite_AlarmTableSignature(&c, 1U)) { printf("FAIL sig should differ on code\n"); g_fail++; }
}

static void TestUnpackRejectsShort(void)
{
  uint8_t buf[3] = {1U, 5U, 0U}; /* 声称 5 行但 len 不够 */
  Px4Lite_AlarmRecord_t out[PX4LITE_TUNNEL_ALARM_MAX_ROWS];
  uint8_t cnt = 0U, ver = 0U;
  ExpectU32("short rejected", Px4Lite_UnpackAlarmTable(buf, 3U, 0U, out, PX4LITE_TUNNEL_ALARM_MAX_ROWS, &cnt, &ver), PX4LITE_INVALID_PARAM);
}

int main(void)
{
  TestRoundTripTwoActiveRows();
  TestEmptyTable();
  TestSignatureIgnoresAge();
  TestUnpackRejectsShort();
  if (g_fail == 0) { printf("PASS test_remote_tunnel_alarm\n"); return 0; }
  printf("FAIL test_remote_tunnel_alarm fails=%d\n", g_fail);
  return 1;
}
