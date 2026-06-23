/**
 * @file px4lite_remote_telemetry.c
 * @brief 保存 LoRa 远程显示模式和 MAVLink 远端只读快照。
 *
 * @details
 * 本模块由 comm 上下文写入，由 Business/Display 只读复制。第一阶段不提供 ACK、绑定
 * 握手或远程控制，只维护目标 `sysid`、字段有效位和超时状态。
 */

#include "px4lite_remote_telemetry.h"

#include <string.h>

#define PX4LITE_REMOTE_MODE_BUTTON_DEBOUNCE_CYCLES 3U

static Px4Lite_RemoteMode_t s_mode;
static Px4Lite_RemoteTelemetrySnapshot_t s_snapshot;
static uint32_t s_sequence;
static uint32_t s_mode_changed_ms;
static uint8_t s_button_last_raw;
static uint8_t s_button_press_count;
static uint8_t s_button_press_latched;

void Px4Lite_RemoteTelemetryInit(uint32_t now_ms)
{
  memset(&s_snapshot, 0, sizeof(s_snapshot));
  s_mode                 = PX4LITE_REMOTE_MODE_LOCAL;
  s_sequence             = 0U;
  s_mode_changed_ms      = now_ms;
  s_button_last_raw      = 0U;
  s_button_press_count   = 0U;
  s_button_press_latched = 0U;
  s_snapshot.target_sysid = PX4LITE_REMOTE_TARGET_SYSID_ANY;
}

Px4Lite_Result_t Px4Lite_RemoteTelemetrySetMode(Px4Lite_RemoteMode_t mode, uint32_t now_ms)
{
  if ((mode != PX4LITE_REMOTE_MODE_LOCAL) && (mode != PX4LITE_REMOTE_MODE_REMOTE)) { return PX4LITE_INVALID_PARAM; }

  if (s_mode != mode) {
    s_mode            = mode;
    s_mode_changed_ms = now_ms;
    if (mode == PX4LITE_REMOTE_MODE_LOCAL) {
      memset(&s_snapshot, 0, sizeof(s_snapshot));
      s_snapshot.target_sysid = PX4LITE_REMOTE_TARGET_SYSID_ANY;
    }
  }
  return PX4LITE_OK;
}

Px4Lite_RemoteMode_t Px4Lite_RemoteTelemetryToggleMode(uint32_t now_ms)
{
  if (s_mode == PX4LITE_REMOTE_MODE_LOCAL) {
    (void)Px4Lite_RemoteTelemetrySetMode(PX4LITE_REMOTE_MODE_REMOTE, now_ms);
  } else {
    (void)Px4Lite_RemoteTelemetrySetMode(PX4LITE_REMOTE_MODE_LOCAL, now_ms);
  }
  return s_mode;
}

Px4Lite_RemoteMode_t Px4Lite_RemoteTelemetryUpdateModeButton(uint8_t pressed, uint32_t now_ms)
{
  if (pressed == 0U) {
    s_button_last_raw      = 0U;
    s_button_press_count   = 0U;
    s_button_press_latched = 0U;
    return s_mode;
  }

  if (s_button_last_raw == 0U) {
    s_button_last_raw    = 1U;
    s_button_press_count = 1U;
  } else if (s_button_press_count < PX4LITE_REMOTE_MODE_BUTTON_DEBOUNCE_CYCLES) {
    s_button_press_count++;
  }

  if ((s_button_press_count >= PX4LITE_REMOTE_MODE_BUTTON_DEBOUNCE_CYCLES) && (s_button_press_latched == 0U)) {
    s_button_press_latched = 1U;
    (void)Px4Lite_RemoteTelemetryToggleMode(now_ms);
  }

  return s_mode;
}

Px4Lite_RemoteMode_t Px4Lite_RemoteTelemetryGetMode(void)
{
  return s_mode;
}

Px4Lite_Result_t Px4Lite_RemoteTelemetrySetTargetSysId(uint8_t sysid)
{
  s_snapshot.target_sysid = sysid;
  return PX4LITE_OK;
}

uint8_t Px4Lite_RemoteTelemetryGetTargetSysId(void)
{
  return s_snapshot.target_sysid;
}

uint8_t Px4Lite_RemoteTelemetryAcceptSysId(uint8_t sysid)
{
  uint8_t target = s_snapshot.target_sysid;

  if (s_mode != PX4LITE_REMOTE_MODE_REMOTE) { return 0U; }
  if (sysid == PX4LITE_MAVLINK_SYSTEM_ID) { return 0U; }
  if (target == PX4LITE_REMOTE_TARGET_SYSID_ANY) { return 1U; }
  return (sysid == target) ? 1U : 0U;
}

void Px4Lite_RemoteTelemetryRecordFiltered(void)
{
  s_snapshot.filtered_count++;
}

void Px4Lite_RemoteTelemetryRecordUnsupported(void)
{
  s_snapshot.unsupported_count++;
}

Px4Lite_RemoteTelemetrySnapshot_t *Px4Lite_RemoteTelemetryMutable(void)
{
  return &s_snapshot;
}

void Px4Lite_RemoteTelemetryCommit(uint8_t sysid, uint8_t compid, uint32_t valid_bits, uint32_t now_ms)
{
  s_sequence++;
  s_snapshot.header.sequence        = s_sequence;
  s_snapshot.header.sample_time_ms  = now_ms;
  s_snapshot.header.publish_time_ms = now_ms;
  s_snapshot.header.flags           = PX4LITE_DATA_VALID;
  s_snapshot.header.device_id       = sysid;
  s_snapshot.header.valid           = 1U;
  s_snapshot.header.quality         = 100U;
  s_snapshot.source_sysid           = sysid;
  s_snapshot.source_compid          = compid;
  s_snapshot.valid_mask            |= valid_bits;
  s_snapshot.last_msg_ms            = now_ms;
  if ((valid_bits & PX4LITE_REMOTE_VALID_IDENTITY) != 0U) { s_snapshot.last_heartbeat_ms = now_ms; }
  s_snapshot.rx_count++;
}

Px4Lite_Result_t Px4Lite_RemoteTelemetryCopySnapshot(Px4Lite_RemoteTelemetrySnapshot_t *out, uint32_t now_ms)
{
  if (out == 0) { return PX4LITE_INVALID_PARAM; }

  memset(out, 0, sizeof(*out));
  if ((s_snapshot.header.valid == 0U) || (s_snapshot.valid_mask == 0U)) { return PX4LITE_NOT_READY; }

  *out = s_snapshot;
  if ((uint32_t)(now_ms - s_snapshot.last_msg_ms) > PX4LITE_REMOTE_TELEMETRY_TIMEOUT_MS) { return PX4LITE_STALE; }
  (void)s_mode_changed_ms;
  return PX4LITE_OK;
}
