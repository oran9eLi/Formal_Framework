/**
 * @file sensor_ubx.c
 * @brief Sensor 层 UBX 字节流组帧、Fletcher 校验和帧构造。
 * @details 根据 u-blox UBX 帧规范独立实现，仅依赖 sensor_ubx.h 与标准内存函数。
 *          禁止访问硬件、调度任务、转换 Framework 数据或在载荷内搜索伪帧头。
 */
#include "sensor_ubx.h"

#include <stddef.h>
#include <string.h>

/** @brief 内部解析状态；校验阶段始终消费 CK_A 和 CK_B 两个字节。 */
enum {
  UBX_STATE_SYNC1 = 0,
  UBX_STATE_SYNC2,
  UBX_STATE_CLASS,
  UBX_STATE_ID,
  UBX_STATE_LENGTH_LOW,
  UBX_STATE_LENGTH_HIGH,
  UBX_STATE_PAYLOAD,
  UBX_STATE_CK_A,
  UBX_STATE_CK_B
};

/** @brief 将一个 class 到 payload 范围内的字节加入 8 位 Fletcher 校验。 */
static void Ubx_Accumulate(Ubx_Parser_t *parser, uint8_t byte)
{
  parser->internal_ck_a = (uint8_t)(parser->internal_ck_a + byte);
  parser->internal_ck_b = (uint8_t)(parser->internal_ck_b + parser->internal_ck_a);
}

/** @brief 清空调用者持有的解析器；无硬件或外部状态副作用。 */
void Ubx_Init(Ubx_Parser_t *parser)
{
  if (parser != NULL) { memset(parser, 0, sizeof(*parser)); }
}

/** @brief 按帧声明长度消费字节，完整收到两字节校验后再报告结果。 */
Ubx_Result_t Ubx_Feed(Ubx_Parser_t *parser, uint8_t byte)
{
  uint8_t checksum_error;

  if (parser == NULL) { return UBX_RESULT_NONE; }

  switch (parser->internal_state) {
    case UBX_STATE_SYNC1:
      if (byte == 0xB5U) { parser->internal_state = UBX_STATE_SYNC2; }
      break;

    case UBX_STATE_SYNC2:
      if (byte == 0x62U) {
        parser->internal_ck_a = 0U;
        parser->internal_ck_b = 0U;
        parser->internal_position = 0U;
        parser->length = 0U;
        parser->internal_state = UBX_STATE_CLASS;
      } else if (byte != 0xB5U) {
        parser->internal_state = UBX_STATE_SYNC1;
      }
      break;

    case UBX_STATE_CLASS:
      parser->class_id = byte;
      Ubx_Accumulate(parser, byte);
      parser->internal_state = UBX_STATE_ID;
      break;

    case UBX_STATE_ID:
      parser->message_id = byte;
      Ubx_Accumulate(parser, byte);
      parser->internal_state = UBX_STATE_LENGTH_LOW;
      break;

    case UBX_STATE_LENGTH_LOW:
      parser->length = byte;
      Ubx_Accumulate(parser, byte);
      parser->internal_state = UBX_STATE_LENGTH_HIGH;
      break;

    case UBX_STATE_LENGTH_HIGH:
      parser->length = (uint16_t)(parser->length | ((uint16_t)byte << 8U));
      Ubx_Accumulate(parser, byte);
      parser->internal_state = (parser->length == 0U) ? UBX_STATE_CK_A : UBX_STATE_PAYLOAD;
      break;

    case UBX_STATE_PAYLOAD:
      if (parser->internal_position < UBX_PAYLOAD_CAPACITY) {
        parser->payload[parser->internal_position] = byte;
      }
      Ubx_Accumulate(parser, byte);
      parser->internal_position++;
      if (parser->internal_position == parser->length) { parser->internal_state = UBX_STATE_CK_A; }
      break;

    case UBX_STATE_CK_A:
      parser->internal_received_ck_a = byte;
      parser->internal_state = UBX_STATE_CK_B;
      break;

    case UBX_STATE_CK_B:
      parser->received_ck_a = parser->internal_received_ck_a;
      parser->received_ck_b = byte;
      checksum_error = ((parser->internal_received_ck_a != parser->internal_ck_a) ||
                        (byte != parser->internal_ck_b)) ? 1U : 0U;
      /* 错误尾字节若为 B5，可作为下一轮同步前缀；完整成功帧不复用校验字节。 */
      parser->internal_state = ((checksum_error != 0U) && (byte == 0xB5U)) ? UBX_STATE_SYNC2 : UBX_STATE_SYNC1;
      if (checksum_error != 0U) { return UBX_RESULT_CHECKSUM_ERROR; }
      if (parser->length > UBX_PAYLOAD_CAPACITY) { return UBX_RESULT_OVERSIZE; }
      return UBX_RESULT_FRAME;

    default:
      /* 非法内部状态只丢弃当前半帧，不依赖调用者缓冲区外的数据。 */
      parser->internal_state = (byte == 0xB5U) ? UBX_STATE_SYNC2 : UBX_STATE_SYNC1;
      break;
  }

  return UBX_RESULT_NONE;
}

/** @brief 查询有效接收状态，不修改解析器、计数或载荷。 */
uint8_t Ubx_IsReceiving(const Ubx_Parser_t *parser)
{
  if (parser == NULL) { return 0U; }
  return ((parser->internal_state >= UBX_STATE_SYNC2) &&
          (parser->internal_state <= UBX_STATE_CK_B)) ? 1U : 0U;
}

/** @brief 在完成所有参数与总长检查后构造帧；支持原地或重叠载荷复制。 */
uint16_t Ubx_BuildFrame(uint8_t class_id, uint8_t message_id,
                        const uint8_t *payload, uint16_t length,
                        uint8_t *out, uint16_t capacity)
{
  uint32_t total = (uint32_t)length + UBX_FRAME_OVERHEAD;
  uint16_t i;
  uint8_t ck_a = 0U;
  uint8_t ck_b = 0U;

  if ((out == NULL) || ((payload == NULL) && (length != 0U)) ||
      (total > UINT16_MAX) || (total > capacity)) { return 0U; }

  if (length != 0U) { memmove(&out[6], payload, length); }
  out[0] = 0xB5U;
  out[1] = 0x62U;
  out[2] = class_id;
  out[3] = message_id;
  out[4] = (uint8_t)(length & 0xFFU);
  out[5] = (uint8_t)(length >> 8U);

  for (i = 2U; i < (uint16_t)(total - 2U); i++) {
    ck_a = (uint8_t)(ck_a + out[i]);
    ck_b = (uint8_t)(ck_b + ck_a);
  }
  out[(uint16_t)(total - 2U)] = ck_a;
  out[(uint16_t)(total - 1U)] = ck_b;
  return (uint16_t)total;
}
