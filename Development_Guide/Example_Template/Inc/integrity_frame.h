#ifndef INTEGRITY_FRAME_H
#define INTEGRITY_FRAME_H

#include "integrity_types.h"

#define INTEGRITY_FRAME_MAGIC_0     0xAAU
#define INTEGRITY_FRAME_MAGIC_1     0x55U
#define INTEGRITY_FRAME_VERSION     1U
#define INTEGRITY_FRAME_HEADER_SIZE 16U
#define INTEGRITY_FRAME_CRC_SIZE    2U
#define INTEGRITY_FRAME_PAYLOAD_MAX 256U
#define INTEGRITY_FRAME_MAX_SIZE    (INTEGRITY_FRAME_HEADER_SIZE + INTEGRITY_FRAME_PAYLOAD_MAX + INTEGRITY_FRAME_CRC_SIZE)

typedef struct {
  uint8_t message_type;
  uint32_t sequence;
  uint32_t timestamp_ms;
  uint16_t payload_length;
  uint8_t payload[INTEGRITY_FRAME_PAYLOAD_MAX];
} Integrity_DecodedFrame_t;

typedef struct {
  uint8_t buffer[INTEGRITY_FRAME_MAX_SIZE];
  uint16_t position;
  uint16_t expected_length;
  uint32_t complete_count;
  uint32_t crc_error_count;
  uint32_t format_error_count;
} Integrity_FrameParser_t;

uint16_t Integrity_Crc16(const uint8_t *data, uint16_t length);
uint16_t Integrity_FrameEncode(uint8_t message_type, uint32_t sequence, uint32_t timestamp_ms, const uint8_t *payload, uint16_t payload_length, uint8_t *output, uint16_t capacity);
void Integrity_FrameParserInit(Integrity_FrameParser_t *parser);
Integrity_Result_t Integrity_FrameParserFeed(Integrity_FrameParser_t *parser, uint8_t byte, Integrity_DecodedFrame_t *frame);

#endif
