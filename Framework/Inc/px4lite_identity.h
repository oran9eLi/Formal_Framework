/**
 * @file px4lite_identity.h
 * @brief Framework identity helper for RemoteID, LoRa node id and MAVLink system id.
 *
 * @details
 * This module keeps all compile-time identity derivation in one place. It does not
 * access hardware and does not allocate memory at runtime. Callers provide fixed
 * output buffers when a formatted id string is needed.
 */

#ifndef PX4LITE_IDENTITY_H
#define PX4LITE_IDENTITY_H

#include <stdint.h>

#define PX4LITE_IDENTITY_FULL_ID_MAX_LEN 9U

uint8_t Px4Lite_IdentityGetUnitId(void);
uint8_t Px4Lite_IdentityGetNodeId(void);
uint8_t Px4Lite_IdentityGetMavlinkSystemId(void);
uint8_t Px4Lite_IdentityGetMasterNodeId(void);
void Px4Lite_IdentityFormatFullId(char *out, uint8_t capacity);

#endif
