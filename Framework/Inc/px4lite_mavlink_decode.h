/**
 * @file px4lite_mavlink_decode.h
 * @brief Framework 课程MAVLink字段只读解码；不改变协议/控制/认证状态。
 */
#ifndef PX4LITE_MAVLINK_DECODE_H
#define PX4LITE_MAVLINK_DECODE_H
#include "px4lite_diagnostic_capture.h"
#define PX4LITE_DECODE_ROWS 8U /**< 单页最多八行。 */
#define PX4LITE_DECODE_TEXT 64U /**< 每行字节数含终止符。 */
/** @brief 有界页输出，由调用方提供工作区，不保留指针。 */
typedef struct {
  char name[32]; /**< 消息名或Unsupported。 */
  char rows[PX4LITE_DECODE_ROWS][PX4LITE_DECODE_TEXT]; /**< 字段值与线上单位。 */
  uint8_t row_count; /**< 本页行数。 */
  uint8_t page_count; /**< 总页数。 */
  uint8_t page; /**< 实际页号，0起。 */
  uint8_t reserved; /**< 对齐。 */
} Px4Lite_DecodedPage_t;
/** @brief 解码已复制的原始帧；缺省扩展字段按MAVLink零尾规则读取。 */
Px4Lite_Result_t Px4Lite_DecodeFrame(const Px4Lite_CaptureFrame_t *frame, uint8_t page, Px4Lite_DecodedPage_t *out);
#endif
