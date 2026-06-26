/**
 * @file px4lite_remote_tunnel.h
 * @brief 远端显示 TUNNEL 打包载荷的编解码契约(告警表)。布局见 LY/20 §4。
 */
#ifndef PX4LITE_REMOTE_TUNNEL_H
#define PX4LITE_REMOTE_TUNNEL_H

#include <stdint.h>
#include "px4lite_types.h"

#define PX4LITE_TUNNEL_PT_ALARM_TABLE 0x8001U /**< TUNNEL payload_type：完整告警表。 */

#define PX4LITE_TUNNEL_ALARM_HEADER_BYTES 2U  /**< ver(1)+active_count(1)。 */
#define PX4LITE_TUNNEL_ALARM_ROW_BYTES    7U  /**< source_id(1)+fault_code(2)+severity(1)+active(1)+age_s(2)。 */
#define PX4LITE_TUNNEL_ALARM_MAX_ROWS     ((uint8_t)PX4LITE_MODULE_COUNT)
#define PX4LITE_TUNNEL_ALARM_MAX_BYTES \
  (PX4LITE_TUNNEL_ALARM_HEADER_BYTES + (uint16_t)PX4LITE_TUNNEL_ALARM_ROW_BYTES * PX4LITE_TUNNEL_ALARM_MAX_ROWS)

/**
 * @brief 把 active 告警行打包为告警表载荷(小端)。
 *
 * @param[in] records 本机告警记录数组。
 * @param[in] record_count 数组长度。
 * @param[in] ver 表内容版本，写入表头。
 * @param[in] now_ms 用于计算 age_s。
 * @param[out] out 输出缓冲。
 * @param[in] out_cap 输出容量。
 *
 * @return payload 字节数；参数非法或容量不足表头返回 0。容量不足时截断到能容纳的整行。
 */
uint16_t Px4Lite_PackAlarmTable(const Px4Lite_AlarmRecord_t *records, uint8_t record_count,
                                uint8_t ver, uint32_t now_ms, uint8_t *out, uint16_t out_cap);

/**
 * @brief 解包告警表载荷到记录数组。age_s 还原为 raised_ms = now - age_s*1000(近似)。
 *
 * @return PX4LITE_OK 成功；PX4LITE_INVALID_PARAM 空指针/长度不足/行越界。
 */
Px4Lite_Result_t Px4Lite_UnpackAlarmTable(const uint8_t *payload, uint16_t len, uint32_t now_ms,
                                          Px4Lite_AlarmRecord_t *out_records, uint8_t out_cap,
                                          uint8_t *out_count, uint8_t *out_ver);

/**
 * @brief active 行内容签名(source_id+fault_code+severity+active，不含 age_s)，用于 on-change 检测。
 */
uint32_t Px4Lite_AlarmTableSignature(const Px4Lite_AlarmRecord_t *records, uint8_t record_count);

#endif /* PX4LITE_REMOTE_TUNNEL_H */
