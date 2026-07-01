/**
 * @file px4lite_identity.c
 * @brief Unified identity derivation for LoRa, MAVLink and RemoteID.
 *
 * @details
 * The unit id is the only numeric identity configured by product builds. LoRa
 * node id and MAVLink system id are derived from it directly, so DCDW-001 maps
 * to node/system id 1, DCDW-002 maps to 2, and so on.
 */

#include "px4lite_identity.h"

#include "px4lite_config.h"

uint8_t Px4Lite_IdentityGetUnitId(void)
{
  return (uint8_t)PX4LITE_UNIT_ID;
}

uint8_t Px4Lite_IdentityGetNodeId(void)
{
  return (uint8_t)PX4LITE_NODE_ID;
}

uint8_t Px4Lite_IdentityGetMavlinkSystemId(void)
{
  return (uint8_t)PX4LITE_MAVLINK_SYSTEM_ID;
}

uint8_t Px4Lite_IdentityGetMasterNodeId(void)
{
  return (uint8_t)PX4LITE_MASTER_NODE_ID;
}

void Px4Lite_IdentityFormatFullId(char *out, uint8_t capacity)
{
  uint8_t unit_id;
  uint8_t i;
  const char *prefix = PX4LITE_ORG_PREFIX;

  if ((out == 0) || (capacity == 0U)) { return; }

  for (i = 0U; i < capacity; i++) { out[i] = '\0'; }
  if (capacity < PX4LITE_IDENTITY_FULL_ID_MAX_LEN) { return; }

  for (i = 0U; (i < 4U) && (prefix[i] != '\0'); i++) {
    out[i] = prefix[i];
  }
  out[4] = '-';

  unit_id = (uint8_t)PX4LITE_UNIT_ID;
  out[5] = (char)('0' + ((unit_id / 100U) % 10U));
  out[6] = (char)('0' + ((unit_id / 10U) % 10U));
  out[7] = (char)('0' + (unit_id % 10U));
  out[8] = '\0';
}
