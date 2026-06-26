/**
 * @file px4lite_remote_tunnel.c
 * @brief 远端显示 TUNNEL 载荷编解码(告警表)。纯函数，无全局状态、无动态分配。
 */
#include "px4lite_remote_tunnel.h"
#include <string.h>

static void Tun_PutU16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFFU); p[1] = (uint8_t)((v >> 8) & 0xFFU); }
static uint16_t Tun_GetU16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

uint16_t Px4Lite_PackAlarmTable(const Px4Lite_AlarmRecord_t *records, uint8_t record_count,
                                uint8_t ver, uint32_t now_ms, uint8_t *out, uint16_t out_cap)
{
  uint16_t off = PX4LITE_TUNNEL_ALARM_HEADER_BYTES;
  uint8_t n = 0U;
  uint8_t i;

  if ((records == 0) || (out == 0) || (out_cap < PX4LITE_TUNNEL_ALARM_HEADER_BYTES)) { return 0U; }

  for (i = 0U; i < record_count; ++i) {
    uint8_t *row;
    uint32_t age_s;

    if (records[i].active == 0U) { continue; }
    if ((uint16_t)(off + PX4LITE_TUNNEL_ALARM_ROW_BYTES) > out_cap) { break; }

    row    = &out[off];
    row[0] = (uint8_t)(records[i].source_id & 0xFFU);
    Tun_PutU16(&row[1], records[i].fault_code);
    row[3] = (uint8_t)records[i].severity;
    row[4] = records[i].active;
    age_s  = (now_ms - records[i].raised_ms) / 1000U;
    if (age_s > 0xFFFFU) { age_s = 0xFFFFU; }
    Tun_PutU16(&row[5], (uint16_t)age_s);

    off = (uint16_t)(off + PX4LITE_TUNNEL_ALARM_ROW_BYTES);
    n++;
  }

  out[0] = ver;
  out[1] = n;
  return off;
}

Px4Lite_Result_t Px4Lite_UnpackAlarmTable(const uint8_t *payload, uint16_t len, uint32_t now_ms,
                                          Px4Lite_AlarmRecord_t *out_records, uint8_t out_cap,
                                          uint8_t *out_count, uint8_t *out_ver)
{
  uint16_t off = PX4LITE_TUNNEL_ALARM_HEADER_BYTES;
  uint8_t stored = 0U;
  uint8_t n;
  uint8_t i;

  if ((payload == 0) || (out_records == 0) || (out_count == 0) || (out_ver == 0)) { return PX4LITE_INVALID_PARAM; }
  if (len < PX4LITE_TUNNEL_ALARM_HEADER_BYTES) { return PX4LITE_INVALID_PARAM; }

  *out_ver = payload[0];
  n        = payload[1];

  for (i = 0U; i < n; ++i) {
    const uint8_t *row;
    if ((uint16_t)(off + PX4LITE_TUNNEL_ALARM_ROW_BYTES) > len) { return PX4LITE_INVALID_PARAM; }
    if (stored < out_cap) {
      row = &payload[off];
      memset(&out_records[stored], 0, sizeof(out_records[stored]));
      out_records[stored].source_id  = (uint16_t)row[0];
      out_records[stored].fault_code = Tun_GetU16(&row[1]);
      out_records[stored].severity   = (Px4Lite_AlarmSeverity_t)row[3];
      out_records[stored].active     = row[4];
      out_records[stored].raised_ms  = now_ms - (uint32_t)Tun_GetU16(&row[5]) * 1000U;
      out_records[stored].updated_ms = now_ms;
      stored++;
    }
    off = (uint16_t)(off + PX4LITE_TUNNEL_ALARM_ROW_BYTES);
  }

  *out_count = stored;
  return PX4LITE_OK;
}

uint32_t Px4Lite_AlarmTableSignature(const Px4Lite_AlarmRecord_t *records, uint8_t record_count)
{
  uint32_t h = 2166136261U; /* FNV-1a 32 */
  uint8_t i;

  if (records == 0) { return 0U; }
  for (i = 0U; i < record_count; ++i) {
    if (records[i].active == 0U) { continue; }
    h = (h ^ (uint32_t)(records[i].source_id & 0xFFU)) * 16777619U;
    h = (h ^ (uint32_t)(records[i].fault_code & 0xFFU)) * 16777619U;
    h = (h ^ (uint32_t)((records[i].fault_code >> 8) & 0xFFU)) * 16777619U;
    h = (h ^ (uint32_t)records[i].severity) * 16777619U;
    h = (h ^ (uint32_t)records[i].active) * 16777619U;
  }
  return h;
}
