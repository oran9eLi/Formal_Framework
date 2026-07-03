/**
 * @file px4lite_identity.h
 * @brief Framework identity helper for RemoteID, LoRa node id and MAVLink system id.
 *
 * @details
 * This module keeps the human-readable device name as the single manual identity
 * input. It does not access hardware and does not allocate memory at runtime.
 * Hardware UID is exposed by the platform adapter.
 */

#ifndef PX4LITE_IDENTITY_H
#define PX4LITE_IDENTITY_H

#include <stdint.h>

#define PX4LITE_IDENTITY_FULL_ID_MAX_LEN 9U

const char *Px4Lite_IdentityGetDeviceName(void);
uint8_t Px4Lite_IdentityGetNumericId(void);
uint8_t Px4Lite_IdentityGetNodeId(void);
uint8_t Px4Lite_IdentityGetMavlinkSystemId(void);
void Px4Lite_IdentityFormatFullId(char *out, uint8_t capacity);

#endif
