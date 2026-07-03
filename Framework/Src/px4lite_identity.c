/**
 * @file px4lite_identity.c
 * @brief Unified identity derivation for LoRa, MAVLink and RemoteID.
 *
 * @details
 * PX4LITE_DEVICE_NAME is the only manual identity configured by product builds.
 * LoRa node id and MAVLink system id are derived from its numeric suffix, so
 * DCDW-001 maps to node/system id 1, DCDW-002 maps to 2, and so on.
 */

#include "px4lite_identity.h"

#include "px4lite_config.h"

static uint8_t Identity_ParseSuffix3(const char *name)
{
  uint8_t i;
  uint8_t len = 0U;
  uint16_t value;

  if (name == 0) { return 1U; }
  while ((name[len] != '\0') && (len < 32U)) { len++; }
  if (len < 3U) { return 1U; }

  value = 0U;
  for (i = (uint8_t)(len - 3U); i < len; i++) {
    if ((name[i] < '0') || (name[i] > '9')) { return 1U; }
    value = (uint16_t)((value * 10U) + (uint16_t)(name[i] - '0'));
  }
  if ((value == 0U) || (value >= PX4LITE_REMOTE_NODE_MAX) || (value > 255U)) { return 1U; }
  return (uint8_t)value;
}

const char *Px4Lite_IdentityGetDeviceName(void)
{
  return PX4LITE_DEVICE_NAME;
}

uint8_t Px4Lite_IdentityGetNumericId(void)
{
  return Identity_ParseSuffix3(PX4LITE_DEVICE_NAME);
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
