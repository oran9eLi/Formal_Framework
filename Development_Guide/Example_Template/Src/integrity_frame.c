#include "integrity_frame.h"
#include <string.h>

static void Integrity_PutU16(uint8_t *buffer, uint16_t value)
{
  buffer[0] = (uint8_t)value;
  buffer[1] = (uint8_t)(value >> 8);
}

static void Integrity_PutU32(uint8_t *buffer, uint32_t value)
{
  buffer[0] = (uint8_t)value;
  buffer[1] = (uint8_t)(value >> 8);
  buffer[2] = (uint8_t)(value >> 16);
  buffer[3] = (uint8_t)(value >> 24);
}

static uint16_t Integrity_GetU16(const uint8_t *buffer)
{
  return (uint16_t)((uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8));
}

static uint32_t Integrity_GetU32(const uint8_t *buffer)
{
  return (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24);
}

uint16_t Integrity_Crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i;
  uint8_t bit;

  for (i = 0U; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (bit = 0U; bit < 8U; ++bit) { crc = ((crc & 0x8000U) != 0U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1); }
  }
  return crc;
}

uint16_t Integrity_FrameEncode(uint8_t message_type, uint32_t sequence, uint32_t timestamp_ms, const uint8_t *payload, uint16_t payload_length, uint8_t *output, uint16_t capacity)
{
  uint16_t frame_length;
  uint16_t crc;

  frame_length = (uint16_t)(INTEGRITY_FRAME_HEADER_SIZE + payload_length + INTEGRITY_FRAME_CRC_SIZE);

  if ((output == 0) || ((payload == 0) && (payload_length != 0U)) || (payload_length > INTEGRITY_FRAME_PAYLOAD_MAX) || (capacity < frame_length)) { return 0U; }

  output[0] = INTEGRITY_FRAME_MAGIC_0;
  output[1] = INTEGRITY_FRAME_MAGIC_1;
  output[2] = INTEGRITY_FRAME_VERSION;
  output[3] = message_type;
  Integrity_PutU16(&output[4], payload_length);
  Integrity_PutU32(&output[6], sequence);
  Integrity_PutU32(&output[10], timestamp_ms);
  output[14] = 0U;
  output[15] = 0U;

  if (payload_length != 0U) { memcpy(&output[INTEGRITY_FRAME_HEADER_SIZE], payload, payload_length); }

  crc = Integrity_Crc16(output, (uint16_t)(frame_length - INTEGRITY_FRAME_CRC_SIZE));
  Integrity_PutU16(&output[frame_length - 2U], crc);
  return frame_length;
}

void Integrity_FrameParserInit(Integrity_FrameParser_t *parser)
{
  if (parser != 0) { memset(parser, 0, sizeof(*parser)); }
}

Integrity_Result_t Integrity_FrameParserFeed(Integrity_FrameParser_t *parser, uint8_t byte, Integrity_DecodedFrame_t *frame)
{
  uint16_t payload_length;
  uint16_t received_crc;
  uint16_t calculated_crc;

  if ((parser == 0) || (frame == 0)) { return INTEGRITY_INVALID_PARAM; }

  if ((parser->position == 0U) && (byte != INTEGRITY_FRAME_MAGIC_0)) { return INTEGRITY_IDLE; }

  if ((parser->position == 1U) && (byte != INTEGRITY_FRAME_MAGIC_1)) {
    parser->position = (byte == INTEGRITY_FRAME_MAGIC_0) ? 1U : 0U;
    parser->format_error_count++;
    return INTEGRITY_IDLE;
  }

  parser->buffer[parser->position++] = byte;

  if (parser->position == 6U) {
    payload_length = Integrity_GetU16(&parser->buffer[4]);
    if ((parser->buffer[2] != INTEGRITY_FRAME_VERSION) || (payload_length > INTEGRITY_FRAME_PAYLOAD_MAX)) {
      parser->position = 0U;
      parser->format_error_count++;
      return INTEGRITY_IO_ERROR;
    }

    parser->expected_length = (uint16_t)(INTEGRITY_FRAME_HEADER_SIZE + payload_length + INTEGRITY_FRAME_CRC_SIZE);
  }

  if ((parser->expected_length == 0U) || (parser->position < parser->expected_length)) { return INTEGRITY_IDLE; }

  received_crc   = Integrity_GetU16(&parser->buffer[parser->expected_length - 2U]);
  calculated_crc = Integrity_Crc16(parser->buffer, (uint16_t)(parser->expected_length - 2U));

  if (received_crc != calculated_crc) {
    parser->position        = 0U;
    parser->expected_length = 0U;
    parser->crc_error_count++;
    return INTEGRITY_CRC_ERROR;
  }

  payload_length        = Integrity_GetU16(&parser->buffer[4]);
  frame->message_type   = parser->buffer[3];
  frame->payload_length = payload_length;
  frame->sequence       = Integrity_GetU32(&parser->buffer[6]);
  frame->timestamp_ms   = Integrity_GetU32(&parser->buffer[10]);
  if (payload_length != 0U) { memcpy(frame->payload, &parser->buffer[INTEGRITY_FRAME_HEADER_SIZE], payload_length); }

  parser->position        = 0U;
  parser->expected_length = 0U;
  parser->complete_count++;
  return INTEGRITY_OK;
}
