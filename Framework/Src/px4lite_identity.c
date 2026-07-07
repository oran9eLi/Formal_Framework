/**
 * @file px4lite_identity.c
 * @brief Unified identity derivation for LoRa, MAVLink and RemoteID.
 */

#include "px4lite_identity.h"

#include "px4lite_config.h"
#include "px4lite_platform.h"

const char *Px4Lite_IdentityGetDeviceName(void)
{
  return PX4LITE_DEVICE_NAME;
}

static uint32_t Identity_Fnv1aUpdate(uint32_t hash, uint8_t value)
{
  hash ^= value;
  return hash * 16777619UL;
}

static uint32_t Identity_BuildUidHash(void)
{
  uint32_t uid[3];
  uint32_t hash = 2166136261UL;
  uint8_t i;
  uint8_t shift;

  if (Px4Lite_PlatformGetHardwareUid(uid, 3U) != PX4LITE_OK) {
    return 0U;
  }

  for (i = 0U; i < 3U; i++) {
    for (shift = 0U; shift < 32U; shift = (uint8_t)(shift + 8U)) {
      hash = Identity_Fnv1aUpdate(hash, (uint8_t)((uid[i] >> shift) & 0xFFU));
    }
  }
  return hash;
}

uint32_t Px4Lite_IdentityGetUidHash(void)
{
  static uint8_t s_hash_valid = 0U;
  static uint32_t s_cached_hash = 0U;

  if (s_hash_valid == 0U) {
    s_cached_hash = Identity_BuildUidHash();
    s_hash_valid = 1U;
  }
  return s_cached_hash;
}

uint8_t Px4Lite_IdentityGetNumericId(void)
{
  static uint8_t s_cached_id = 0U;
  uint32_t h;

  if (s_cached_id != 0U) { return s_cached_id; }

  h = Px4Lite_IdentityGetUidHash();
  if (h == 0U) {
    s_cached_id = 1U;
    return s_cached_id;
  }
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
  uint8_t pos;
  uint8_t id;
  const char *prefix = PX4LITE_DEVICE_NAME;

  if ((out == 0) || (capacity == 0U)) { return; }

  for (pos = 0U; pos < capacity; pos++) { out[pos] = '\0'; }
  if (capacity < PX4LITE_IDENTITY_FULL_ID_MAX_LEN) { return; }

  pos = 0U;
  while ((prefix != 0) && (*prefix != '\0') && ((uint8_t)(pos + 5U) < capacity)) {
    out[pos++] = *prefix++;
  }
  id = Px4Lite_IdentityGetNumericId();
  out[pos++] = '-';
  out[pos++] = (char)('0' + (id / 100U));
  out[pos++] = (char)('0' + ((id / 10U) % 10U));
  out[pos++] = (char)('0' + (id % 10U));
  out[pos] = '\0';
}
