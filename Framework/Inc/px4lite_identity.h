/**
 * @file px4lite_identity.h
 * @brief Framework identity helper for RemoteID, LoRa node id and MAVLink system id.
 *
 * @details
 * This module keeps the UID-derived school-visible DCDW-XXX label separate
 * from the vendor product id used by RemoteID and later cloud/device ledgers.
 */

#ifndef PX4LITE_IDENTITY_H
#define PX4LITE_IDENTITY_H

#include <stdint.h>

#define PX4LITE_IDENTITY_FULL_ID_MAX_LEN 9U
#define PX4LITE_IDENTITY_SN_LEN 12U
#define PX4LITE_IDENTITY_SN_MAX_LEN 13U
#define PX4LITE_IDENTITY_VENDOR_ID_LEN 20U
#define PX4LITE_IDENTITY_VENDOR_ID_MAX_LEN 21U

const char *Px4Lite_IdentityGetDeviceName(void);
uint8_t Px4Lite_IdentityGetNumericId(void);
uint8_t Px4Lite_IdentityGetNodeId(void);
uint8_t Px4Lite_IdentityGetMavlinkSystemId(void);
void Px4Lite_IdentityFormatFullId(char *out, uint8_t capacity);
void Px4Lite_IdentityFormatSerialNumber(char *out, uint8_t capacity);
void Px4Lite_IdentityFormatVendorProductId(char *out, uint8_t capacity);

/**
 * @brief 返回由 STM32 硬件 UID 派生的 32 位 FNV-1a 短身份摘要。
 * @details
 * DCDW-XXX、LoRa node_id 和 MAVLink system_id 由该摘要压缩到 1..250；
 * 权威厂商身份使用
 * Px4Lite_IdentityFormatVendorProductId() 生成的 20 字符识别码。
 */
uint32_t Px4Lite_IdentityGetUidHash(void);

#endif
