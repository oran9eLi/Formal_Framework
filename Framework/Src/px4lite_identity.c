/**
 * @file px4lite_identity.c
 * @brief Unified identity derivation for LoRa, MAVLink and RemoteID.
 *
 * @details
 * MAVLink/LoRa 本机 sysid、node id 由**芯片 96-bit 唯一 UID** 运行时派生，范围 [1,250]。
 * 同一套固件烧到多块板，各芯片 UID 不同 → sysid 自动不同，无需人工改 PX4LITE_DEVICE_NAME
 * 即可互相识别收发（避免两端 sysid 相同被接收端当作"自己发的"丢弃）。
 * PX4LITE_DEVICE_NAME 仍作可读设备名用于本机显示与 RemoteID 文本，不再决定 sysid。
 */

#include "px4lite_identity.h"

#include "px4lite_config.h"
#include "px4lite_platform.h"

const char *Px4Lite_IdentityGetDeviceName(void)
{
  return PX4LITE_DEVICE_NAME;
}

uint8_t Px4Lite_IdentityGetNumericId(void)
{
  static uint8_t s_cached_id = 0U;
  uint32_t uid[3];
  uint32_t h;

  if (s_cached_id != 0U) { return s_cached_id; }

  /* 芯片 UID 三字异或后取模到 [1,250]，避开 0 与广播 255；同一芯片固定，首次后缓存。 */
  if (Px4Lite_PlatformGetHardwareUid(uid, 3U) != PX4LITE_OK) {
    s_cached_id = 1U;
    return s_cached_id;
  }
  h = uid[0] ^ uid[1] ^ uid[2];
  s_cached_id = (uint8_t)(1U + (h % 250U));
  return s_cached_id;
}

uint8_t Px4Lite_IdentityGetNodeId(void)
{
  return Px4Lite_IdentityGetNumericId();
}

uint8_t Px4Lite_IdentityGetMavlinkSystemId(void)
{
  return Px4Lite_IdentityGetNumericId();
}

void Px4Lite_IdentityFormatFullId(char *out, uint8_t capacity)
{
  uint8_t i;
  const char *name = PX4LITE_DEVICE_NAME;

  if ((out == 0) || (capacity == 0U)) { return; }

  for (i = 0U; i < capacity; i++) { out[i] = '\0'; }
  if (capacity < PX4LITE_IDENTITY_FULL_ID_MAX_LEN) { return; }

  for (i = 0U; (i < (uint8_t)(capacity - 1U)) && (name[i] != '\0'); i++) {
    out[i] = name[i];
  }
}
