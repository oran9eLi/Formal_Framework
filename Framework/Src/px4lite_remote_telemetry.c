/**
 * @file px4lite_remote_telemetry.c
 * @brief 保存 LoRa 远程显示模式和 MAVLink 远端只读快照。
 *
 * @details
 * 本模块由 comm 上下文写入，由 Business/Display 只读复制。第二阶段仍按手动
 * `sysid` 分配区分设备，不提供 ACK、绑定握手或远程控制。
 */

#include "px4lite_remote_telemetry.h"
#include "px4lite_platform.h"

#include <string.h>

#define PX4LITE_REMOTE_MODE_BUTTON_DEBOUNCE_CYCLES 3U

static Px4Lite_RemoteMode_t s_mode;
static Px4Lite_RemoteTelemetrySnapshot_t s_snapshot;
static Px4Lite_RemoteDeviceInfo_t s_devices[PX4LITE_REMOTE_DEVICE_TABLE_SIZE];
static uint32_t s_sequence;
static uint32_t s_mode_changed_ms;
static uint8_t s_button_last_raw;
static uint8_t s_button_press_count;
static uint8_t s_button_press_latched;

/**
 * @brief 查找或分配远端设备表项。
 */
static Px4Lite_RemoteDeviceInfo_t *RemoteTelemetry_FindDevice(uint8_t sysid, uint8_t allocate, uint32_t now_ms)
{
  Px4Lite_RemoteDeviceInfo_t *fallback = 0;
  uint8_t i;

  if (sysid == 0U) { return 0; }

  for (i = 0U; i < PX4LITE_REMOTE_DEVICE_TABLE_SIZE; ++i) {
    if (s_devices[i].sysid == sysid) { return &s_devices[i]; }
    if ((fallback == 0) && (s_devices[i].sysid == 0U)) { fallback = &s_devices[i]; }
  }

  if (allocate == 0U) { return 0; }

  if (fallback == 0) {
    fallback = &s_devices[0];
    for (i = 1U; i < PX4LITE_REMOTE_DEVICE_TABLE_SIZE; ++i) {
      if (s_devices[i].last_msg_ms < fallback->last_msg_ms) { fallback = &s_devices[i]; }
    }
  }

  memset(fallback, 0, sizeof(*fallback));
  fallback->sysid             = sysid;
  fallback->state             = PX4LITE_REMOTE_DEVICE_DISCOVERED;
  fallback->last_heartbeat_ms = now_ms;
  return fallback;
}

/**
 * @brief 根据当前目标和新鲜度计算设备公开状态。
 */
static Px4Lite_RemoteDeviceState_t RemoteTelemetry_GetDeviceState(const Px4Lite_RemoteDeviceInfo_t *device, uint32_t now_ms)
{
  uint8_t target = s_snapshot.target_sysid;

  if ((device == 0) || (device->sysid == 0U)) { return PX4LITE_REMOTE_DEVICE_STALE; }
  if (((device->last_msg_ms != 0U) && ((uint32_t)(now_ms - device->last_msg_ms) <= PX4LITE_REMOTE_TELEMETRY_TIMEOUT_MS))) { return PX4LITE_REMOTE_DEVICE_ACTIVE; }
  if (((device->last_heartbeat_ms != 0U) && ((uint32_t)(now_ms - device->last_heartbeat_ms) > PX4LITE_REMOTE_TELEMETRY_TIMEOUT_MS)) || ((device->last_msg_ms != 0U) && ((uint32_t)(now_ms - device->last_msg_ms) > PX4LITE_REMOTE_TELEMETRY_TIMEOUT_MS))) { return PX4LITE_REMOTE_DEVICE_STALE; }
  if ((target != PX4LITE_REMOTE_TARGET_SYSID_ANY) && (device->sysid == target)) { return PX4LITE_REMOTE_DEVICE_TARGETED; }
  return PX4LITE_REMOTE_DEVICE_DISCOVERED;
}

/**
 * @brief 判断快照字段是否已经超过远端超时。
 */
static uint8_t RemoteTelemetry_FieldExpired(uint32_t field_ms, uint32_t now_ms)
{
  if (field_ms == 0U) { return 1U; }
  return ((uint32_t)(now_ms - field_ms) > PX4LITE_REMOTE_TELEMETRY_TIMEOUT_MS) ? 1U : 0U;
}

/**
 * @brief 更新本次提交字段的新鲜度时间戳。
 */
static void RemoteTelemetry_MarkFields(uint32_t valid_bits, uint32_t now_ms)
{
  if ((valid_bits & PX4LITE_REMOTE_VALID_IDENTITY) != 0U) { s_snapshot.identity_update_ms = now_ms; }
  if ((valid_bits & PX4LITE_REMOTE_VALID_NAVIGATION) != 0U) { s_snapshot.navigation_update_ms = now_ms; }
  if ((valid_bits & PX4LITE_REMOTE_VALID_ATTITUDE) != 0U) { s_snapshot.attitude_update_ms = now_ms; }
  if ((valid_bits & PX4LITE_REMOTE_VALID_POWER) != 0U) { s_snapshot.power_update_ms = now_ms; }
  if ((valid_bits & PX4LITE_REMOTE_VALID_ENVIRONMENT) != 0U) { s_snapshot.environment_update_ms = now_ms; }
  if ((valid_bits & PX4LITE_REMOTE_VALID_STATUS) != 0U) { s_snapshot.status_update_ms = now_ms; }
  if ((valid_bits & PX4LITE_REMOTE_VALID_TEXT) != 0U) { s_snapshot.text_update_ms = now_ms; }
  if ((valid_bits & PX4LITE_REMOTE_VALID_TIME) != 0U) { s_snapshot.time_update_ms = now_ms; }
  if ((valid_bits & PX4LITE_REMOTE_VALID_MOTOR) != 0U) { s_snapshot.motor_update_ms = now_ms; }
  if ((valid_bits & PX4LITE_REMOTE_VALID_MODULES) != 0U) { s_snapshot.modules_update_ms = now_ms; }
  if ((valid_bits & PX4LITE_REMOTE_VALID_ALARM) != 0U) { s_snapshot.alarm_update_ms = now_ms; }
}

/**
 * @brief 清理复制快照中已过期字段的有效位。
 */
static void RemoteTelemetry_ApplyFieldTimeout(Px4Lite_RemoteTelemetrySnapshot_t *snapshot, uint32_t now_ms)
{
  snapshot->stale_mask = 0U;
  if (((snapshot->valid_mask & PX4LITE_REMOTE_VALID_IDENTITY) != 0U) && (RemoteTelemetry_FieldExpired(snapshot->identity_update_ms, now_ms) != 0U)) { snapshot->stale_mask |= PX4LITE_REMOTE_VALID_IDENTITY; }
  if (((snapshot->valid_mask & PX4LITE_REMOTE_VALID_NAVIGATION) != 0U) && (RemoteTelemetry_FieldExpired(snapshot->navigation_update_ms, now_ms) != 0U)) { snapshot->stale_mask |= PX4LITE_REMOTE_VALID_NAVIGATION; }
  if (((snapshot->valid_mask & PX4LITE_REMOTE_VALID_ATTITUDE) != 0U) && (RemoteTelemetry_FieldExpired(snapshot->attitude_update_ms, now_ms) != 0U)) { snapshot->stale_mask |= PX4LITE_REMOTE_VALID_ATTITUDE; }
  if (((snapshot->valid_mask & PX4LITE_REMOTE_VALID_POWER) != 0U) && (RemoteTelemetry_FieldExpired(snapshot->power_update_ms, now_ms) != 0U)) { snapshot->stale_mask |= PX4LITE_REMOTE_VALID_POWER; }
  if (((snapshot->valid_mask & PX4LITE_REMOTE_VALID_ENVIRONMENT) != 0U) && (RemoteTelemetry_FieldExpired(snapshot->environment_update_ms, now_ms) != 0U)) { snapshot->stale_mask |= PX4LITE_REMOTE_VALID_ENVIRONMENT; }
  if (((snapshot->valid_mask & PX4LITE_REMOTE_VALID_STATUS) != 0U) && (RemoteTelemetry_FieldExpired(snapshot->status_update_ms, now_ms) != 0U)) { snapshot->stale_mask |= PX4LITE_REMOTE_VALID_STATUS; }
  if (((snapshot->valid_mask & PX4LITE_REMOTE_VALID_TEXT) != 0U) && (RemoteTelemetry_FieldExpired(snapshot->text_update_ms, now_ms) != 0U)) { snapshot->stale_mask |= PX4LITE_REMOTE_VALID_TEXT; }
  if (((snapshot->valid_mask & PX4LITE_REMOTE_VALID_TIME) != 0U) && (RemoteTelemetry_FieldExpired(snapshot->time_update_ms, now_ms) != 0U)) { snapshot->stale_mask |= PX4LITE_REMOTE_VALID_TIME; }
  if (((snapshot->valid_mask & PX4LITE_REMOTE_VALID_MOTOR) != 0U) && (RemoteTelemetry_FieldExpired(snapshot->motor_update_ms, now_ms) != 0U)) { snapshot->stale_mask |= PX4LITE_REMOTE_VALID_MOTOR; }
  if (((snapshot->valid_mask & PX4LITE_REMOTE_VALID_MODULES) != 0U) && (RemoteTelemetry_FieldExpired(snapshot->modules_update_ms, now_ms) != 0U)) { snapshot->stale_mask |= PX4LITE_REMOTE_VALID_MODULES; }
  if (((snapshot->valid_mask & PX4LITE_REMOTE_VALID_ALARM) != 0U) && (RemoteTelemetry_FieldExpired(snapshot->alarm_update_ms, now_ms) != 0U)) { snapshot->stale_mask |= PX4LITE_REMOTE_VALID_ALARM; }
}

void Px4Lite_RemoteTelemetryInit(uint32_t now_ms)
{
  memset(&s_snapshot, 0, sizeof(s_snapshot));
  memset(s_devices, 0, sizeof(s_devices));
  s_mode                 = PX4LITE_REMOTE_MODE_LOCAL;
  s_sequence             = 0U;
  s_mode_changed_ms      = now_ms;
  s_button_last_raw      = 0U;
  s_button_press_count   = 0U;
  s_button_press_latched = 0U;
  s_snapshot.target_sysid = PX4LITE_REMOTE_TARGET_SYSID_DEFAULT;
}

Px4Lite_Result_t Px4Lite_RemoteTelemetrySetMode(Px4Lite_RemoteMode_t mode, uint32_t now_ms)
{
  if ((mode != PX4LITE_REMOTE_MODE_LOCAL) && (mode != PX4LITE_REMOTE_MODE_REMOTE)) { return PX4LITE_INVALID_PARAM; }

  if (s_mode != mode) {
    s_mode            = mode;
    s_mode_changed_ms = now_ms;
    if (mode == PX4LITE_REMOTE_MODE_LOCAL) {
      uint8_t target_sysid = s_snapshot.target_sysid;
      memset(&s_snapshot, 0, sizeof(s_snapshot));
      s_snapshot.target_sysid = target_sysid;
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
  if (sysid == Px4Lite_PlatformMavlinkSystemId()) { return PX4LITE_INVALID_PARAM; }
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
  if (sysid == Px4Lite_PlatformMavlinkSystemId()) { return 0U; }
  if (target == PX4LITE_REMOTE_TARGET_SYSID_ANY) { return 1U; }
  return (sysid == target) ? 1U : 0U;
}

void Px4Lite_RemoteTelemetryRecordFiltered(uint8_t sysid)
{
  Px4Lite_RemoteDeviceInfo_t *device;

  s_snapshot.filtered_count++;
  device = RemoteTelemetry_FindDevice(sysid, 0U, 0U);
  if (device != 0) { device->filtered_count++; }
}

void Px4Lite_RemoteTelemetryRecordUnsupported(uint8_t sysid)
{
  Px4Lite_RemoteDeviceInfo_t *device;

  s_snapshot.unsupported_count++;
  device = RemoteTelemetry_FindDevice(sysid, 0U, 0U);
  if (device != 0) { device->unsupported_count++; }
}

void Px4Lite_RemoteTelemetryObserveHeartbeat(uint8_t sysid, uint8_t compid, uint8_t mav_type, uint8_t base_mode, uint8_t system_status, uint32_t now_ms)
{
  Px4Lite_RemoteDeviceInfo_t *device;

  if ((s_mode != PX4LITE_REMOTE_MODE_REMOTE) || (sysid == Px4Lite_PlatformMavlinkSystemId())) { return; }

  device = RemoteTelemetry_FindDevice(sysid, 1U, now_ms);
  if (device == 0) { return; }

  device->compid            = compid;
  device->mav_type          = mav_type;
  device->base_mode         = base_mode;
  device->system_status     = system_status;
  device->last_heartbeat_ms = now_ms;
  device->state             = RemoteTelemetry_GetDeviceState(device, now_ms);
}

Px4Lite_RemoteTelemetrySnapshot_t *Px4Lite_RemoteTelemetryMutable(void)
{
  return &s_snapshot;
}

void Px4Lite_RemoteTelemetryCommit(uint8_t sysid, uint8_t compid, uint32_t valid_bits, uint32_t now_ms)
{
  Px4Lite_RemoteDeviceInfo_t *device;

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
  s_snapshot.stale_mask            &= ~valid_bits;
  s_snapshot.last_msg_ms            = now_ms;
  RemoteTelemetry_MarkFields(valid_bits, now_ms);
  if ((valid_bits & PX4LITE_REMOTE_VALID_IDENTITY) != 0U) { s_snapshot.last_heartbeat_ms = now_ms; }
  s_snapshot.rx_count++;

  device = RemoteTelemetry_FindDevice(sysid, 1U, now_ms);
  if (device != 0) {
    device->compid      = compid;
    device->last_msg_ms = now_ms;
    if ((valid_bits & PX4LITE_REMOTE_VALID_IDENTITY) != 0U) { device->last_heartbeat_ms = now_ms; }
    device->rx_count++;
    device->state = RemoteTelemetry_GetDeviceState(device, now_ms);
  }
}

Px4Lite_Result_t Px4Lite_RemoteTelemetryCopySnapshot(Px4Lite_RemoteTelemetrySnapshot_t *out, uint32_t now_ms)
{
  if (out == 0) { return PX4LITE_INVALID_PARAM; }

  memset(out, 0, sizeof(*out));
  if ((s_snapshot.header.valid == 0U) || (s_snapshot.valid_mask == 0U)) { return PX4LITE_NOT_READY; }

  *out = s_snapshot;
  RemoteTelemetry_ApplyFieldTimeout(out, now_ms);
  if (out->valid_mask == 0U) { return PX4LITE_STALE; }
  (void)s_mode_changed_ms;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_RemoteTelemetryCopyDevices(Px4Lite_RemoteDeviceInfo_t *out, uint8_t max_count, uint8_t *out_count, uint32_t now_ms)
{
  uint8_t copied = 0U;
  uint8_t i;

  if ((out == 0) || (max_count == 0U)) { return PX4LITE_INVALID_PARAM; }

  memset(out, 0, sizeof(out[0]) * max_count);
  for (i = 0U; (i < PX4LITE_REMOTE_DEVICE_TABLE_SIZE) && (copied < max_count); ++i) {
    if (s_devices[i].sysid == 0U) { continue; }
    out[copied]       = s_devices[i];
    out[copied].state = RemoteTelemetry_GetDeviceState(&s_devices[i], now_ms);
    copied++;
  }

  if (out_count != 0) { *out_count = copied; }
  return (copied == 0U) ? PX4LITE_NOT_READY : PX4LITE_OK;
}
