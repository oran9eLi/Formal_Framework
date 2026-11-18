/**
 * @file test_ubx_codec.c
 * @brief 主机侧验证真实 UBX 编解码器的边界、校验与流式恢复。
 * @details 仅使用标准库及 Sensor codec；不访问 HAL、RTOS 或硬件。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensor_ubx.h"

/** @brief 原厂格式 MON-VER 空载荷轮询向量，校验为 0E 34。 */
static const uint8_t s_poll[] = {0xB5U, 0x62U, 0x0AU, 0x04U, 0x00U, 0x00U, 0x0EU, 0x34U};

/** @brief 固定 ACK 向量，确认 CFG-VALSET 类别和编号。 */
static const uint8_t s_ack[] = {0xB5U, 0x62U, 0x05U, 0x01U, 0x02U, 0x00U, 0x06U, 0x8AU, 0x98U, 0xC1U};

/** @brief 大载荷测试工作区，包含完整 16 位长度帧和两字节校验。 */
static uint8_t s_wire[65535U + UBX_FRAME_OVERHEAD];
/** @brief 生成载荷的静态工作区。 */
static uint8_t s_payload[65535U];

/**
 * @brief 用加权和独立计算参考帧，不调用被测构帧器。
 * @details CK_B 等于从 class 起各字节按剩余长度加权求和，避免接收与构帧共享错误。
 */
static uint32_t ReferenceFrame(uint8_t class_id, uint8_t id, const uint8_t *payload,
                               uint16_t length, uint8_t *out)
{
  uint32_t i;
  uint32_t count = (uint32_t)length + 4U;
  uint64_t sum_a = 0U;
  uint64_t sum_b = 0U;

  out[0] = 0xB5U;
  out[1] = 0x62U;
  out[2] = class_id;
  out[3] = id;
  out[4] = (uint8_t)length;
  out[5] = (uint8_t)(length >> 8U);
  if (length != 0U) { memcpy(out + 6U, payload, length); }
  for (i = 0U; i < count; i++) {
    sum_a += out[i + 2U];
    sum_b += (uint64_t)(count - i) * out[i + 2U];
  }
  out[count + 2U] = (uint8_t)sum_a;
  out[count + 3U] = (uint8_t)sum_b;
  return count + 4U;
}

/** @brief 确认帧在最后 CK_B 到达前不报告成功或错误。 */
static void ExpectComplete(Ubx_Parser_t *parser, const uint8_t *wire, uint32_t size,
                           Ubx_Result_t expected)
{
  uint32_t i;
  assert(size > 0U);
  for (i = 0U; i + 1U < size; i++) { assert(Ubx_Feed(parser, wire[i]) == UBX_RESULT_NONE); }
  assert(Ubx_Feed(parser, wire[size - 1U]) == expected);
}

/** @brief 验证已知线上向量和构帧器参数检查，不接受部分输出。 */
static void TestKnownVectorsAndBuild(void)
{
  Ubx_Parser_t parser;
  uint8_t out[64];
  uint8_t before[64];
  const uint8_t ack_payload[] = {0x06U, 0x8AU};
  uint16_t n;

  Ubx_Init(NULL);
  assert(Ubx_Feed(NULL, 0xB5U) == UBX_RESULT_NONE);
  Ubx_Init(&parser);
  ExpectComplete(&parser, s_poll, sizeof(s_poll), UBX_RESULT_FRAME);
  assert(parser.class_id == 0x0AU && parser.message_id == 0x04U && parser.length == 0U);
  ExpectComplete(&parser, s_ack, sizeof(s_ack), UBX_RESULT_FRAME);
  assert(parser.class_id == 0x05U && parser.message_id == 0x01U && parser.length == 2U);
  assert(memcmp(parser.payload, ack_payload, sizeof(ack_payload)) == 0);
  n = Ubx_BuildFrame(0x0AU, 0x04U, NULL, 0U, out, sizeof(out));
  assert(n == sizeof(s_poll) && memcmp(out, s_poll, n) == 0);
  n = Ubx_BuildFrame(0x05U, 0x01U, ack_payload, sizeof(ack_payload), out, sizeof(out));
  assert(n == sizeof(s_ack) && memcmp(out, s_ack, n) == 0);

  memset(out, 0xA5, sizeof(out));
  memcpy(before, out, sizeof(out));
  assert(Ubx_BuildFrame(1U, 2U, NULL, 1U, out, sizeof(out)) == 0U);
  assert(Ubx_BuildFrame(1U, 2U, ack_payload, sizeof(ack_payload), out, 9U) == 0U);
  assert(Ubx_BuildFrame(1U, 2U, NULL, 0U, out, 7U) == 0U);
  assert(Ubx_BuildFrame(1U, 2U, s_payload, 65528U, out, UINT16_MAX) == 0U);
  assert(Ubx_BuildFrame(1U, 2U, s_payload, UINT16_MAX, out, UINT16_MAX) == 0U);
  assert(Ubx_BuildFrame(1U, 2U, NULL, 0U, NULL, 8U) == 0U);
  assert(memcmp(out, before, sizeof(out)) == 0);

  /* 载荷源位于输出起点或其内部时仍能构造相同帧。 */
  memcpy(out, ack_payload, sizeof(ack_payload));
  assert(Ubx_BuildFrame(0x05U, 0x01U, out, 2U, out, sizeof(out)) == sizeof(s_ack));
  assert(memcmp(out, s_ack, sizeof(s_ack)) == 0);
  memcpy(out + 6U, ack_payload, sizeof(ack_payload));
  assert(Ubx_BuildFrame(0x05U, 0x01U, out + 6U, 2U, out, sizeof(out)) == sizeof(s_ack));
  assert(memcmp(out, s_ack, sizeof(s_ack)) == 0);
}

/** @brief 验证任意分片边界、同步噪声、载荷内帧头和完整结果保留。 */
static void TestStreamingAndFragmentation(void)
{
  Ubx_Parser_t parser;
  uint8_t payload[92];
  uint8_t wire[100];
  uint8_t saved[92];
  uint32_t i;
  uint32_t split;
  uint32_t size;
  static const uint8_t noise[] = {0U, 0x62U, 0xB5U, 0x10U, 0xB5U, 0xB5U};

  for (i = 0U; i < sizeof(payload); i++) { payload[i] = (uint8_t)(i * 13U); }
  memcpy(payload + 20U, s_poll, sizeof(s_poll));
  size = ReferenceFrame(0x01U, 0x07U, payload, sizeof(payload), wire);
  for (split = 0U; split <= size; split++) {
    Ubx_Init(&parser);
    for (i = 0U; i < split; i++) {
      assert(Ubx_Feed(&parser, wire[i]) == ((i + 1U == size) ? UBX_RESULT_FRAME : UBX_RESULT_NONE));
    }
    /* 调用者在两个分片之间不提供任何字节，解析状态必须保留。 */
    for (; i < size; i++) {
      assert(Ubx_Feed(&parser, wire[i]) == ((i + 1U == size) ? UBX_RESULT_FRAME : UBX_RESULT_NONE));
    }
    assert(parser.length == sizeof(payload) && parser.class_id == 1U && parser.message_id == 7U);
    memcpy(saved, parser.payload, sizeof(saved));
    assert(memcmp(saved, payload, sizeof(payload)) == 0);
    assert(memcmp(parser.payload, saved, sizeof(saved)) == 0);
  }
  Ubx_Init(&parser);
  for (i = 0U; i < sizeof(noise); i++) { assert(Ubx_Feed(&parser, noise[i]) == UBX_RESULT_NONE); }
  /* 最后的重复 B5 仍可接收完整帧的 B5 62。 */
  ExpectComplete(&parser, wire, size, UBX_RESULT_FRAME);
  ExpectComplete(&parser, s_poll, sizeof(s_poll), UBX_RESULT_FRAME);
  ExpectComplete(&parser, s_ack, sizeof(s_ack), UBX_RESULT_FRAME);
}

/** @brief 校验每个受保护字节与两个校验字节，并验证坏帧后可接收紧邻尾帧。 */
static void TestChecksumAndTruncation(void)
{
  Ubx_Parser_t parser;
  uint8_t wire[sizeof(s_ack)];
  uint32_t at;

  /* 长度字段另由超长/最大长度测试覆盖，避免改变长度后改变帧边界。 */
  for (at = 2U; at < sizeof(wire); at++) {
    if ((at == 4U) || (at == 5U)) { continue; }
    memcpy(wire, s_ack, sizeof(wire));
    wire[at] ^= 0x01U;
    Ubx_Init(&parser);
    ExpectComplete(&parser, wire, sizeof(wire), UBX_RESULT_CHECKSUM_ERROR);
    ExpectComplete(&parser, s_poll, sizeof(s_poll), UBX_RESULT_FRAME);
  }
  /* CK_A 错误也必须等待 CK_B；不能将紧邻的 CK_B 当噪声提前返回。 */
  memcpy(wire, s_ack, sizeof(wire));
  wire[sizeof(wire) - 2U] ^= 0x80U;
  Ubx_Init(&parser);
  ExpectComplete(&parser, wire, sizeof(wire), UBX_RESULT_CHECKSUM_ERROR);
  ExpectComplete(&parser, s_ack, sizeof(s_ack), UBX_RESULT_FRAME);

  /* 错误 CK_B 本身为下一帧 B5 时，随后 62 仍可恢复同步。 */
  memcpy(wire, s_ack, sizeof(wire));
  wire[sizeof(wire) - 1U] = 0xB5U;
  Ubx_Init(&parser);
  ExpectComplete(&parser, wire, sizeof(wire), UBX_RESULT_CHECKSUM_ERROR);
  ExpectComplete(&parser, s_poll + 1U, sizeof(s_poll) - 1U, UBX_RESULT_FRAME);

  /* 截断的载荷/校验从不产生 FRAME；所有者 Init 可明确撤销半帧。 */
  for (at = 0U; at < sizeof(s_ack); at++) {
    uint32_t i;
    Ubx_Init(&parser);
    for (i = 0U; i < at; i++) { assert(Ubx_Feed(&parser, s_ack[i]) == UBX_RESULT_NONE); }
    Ubx_Init(&parser);
    ExpectComplete(&parser, s_poll, sizeof(s_poll), UBX_RESULT_FRAME);
  }
}

/** @brief 只读接收标志覆盖同步、头、载荷、校验和错误尾前导保留。 */
static void TestReceivingState(void)
{
  Ubx_Parser_t parser;
  Ubx_Parser_t before;
  uint32_t i;

  assert(Ubx_IsReceiving(NULL) == 0U);
  Ubx_Init(&parser);
  assert(Ubx_IsReceiving(&parser) == 0U);
  assert(Ubx_Feed(&parser, 0xB5U) == UBX_RESULT_NONE);
  assert(Ubx_IsReceiving(&parser) == 1U);
  assert(Ubx_Feed(&parser, 0U) == UBX_RESULT_NONE);
  assert(Ubx_IsReceiving(&parser) == 0U);
  for (i = 0U; i + 1U < sizeof(s_ack); i++) {
    assert(Ubx_Feed(&parser, s_ack[i]) == UBX_RESULT_NONE);
    memcpy(&before, &parser, sizeof(before));
    assert(Ubx_IsReceiving(&parser) == 1U);
    assert(memcmp(&parser, &before, sizeof(parser)) == 0);
  }
  assert(Ubx_Feed(&parser, s_ack[sizeof(s_ack) - 1U]) == UBX_RESULT_FRAME);
  assert(Ubx_IsReceiving(&parser) == 0U);
  for (i = 0U; i + 1U < sizeof(s_ack); i++) {
    assert(Ubx_Feed(&parser, s_ack[i]) == UBX_RESULT_NONE);
  }
  assert(Ubx_Feed(&parser, 0xB5U) == UBX_RESULT_CHECKSUM_ERROR);
  assert(Ubx_IsReceiving(&parser) == 1U);
  ExpectComplete(&parser, s_poll + 1U, sizeof(s_poll) - 1U, UBX_RESULT_FRAME);
  assert(Ubx_IsReceiving(&parser) == 0U);
  Ubx_Init(&parser);
  assert(Ubx_IsReceiving(&parser) == 0U);
}

/** @brief 验证精确容量、MON-VER 扩展和超长帧的全长消费，不越过解析器边界。 */
static void TestLengthBoundsAndMessageSizes(void)
{
  struct {
    uint8_t prefix[16];
    Ubx_Parser_t parser;
    uint8_t suffix[16];
  } guarded;
  static const uint16_t sizes[] = {0U, 2U, 18U, 40U, 70U, 92U, 250U, 370U,
                                  UBX_PAYLOAD_CAPACITY, UBX_PAYLOAD_CAPACITY + 1U,
                                  65527U, 65535U};
  uint32_t i;
  uint32_t j;
  uint32_t size;

  memset(&guarded, 0xA5, sizeof(guarded));
  for (i = 0U; i < sizeof(s_payload); i++) { s_payload[i] = (uint8_t)(i * 17U + 3U); }
  /* 超长帧内放置可独立通过校验的帧，不能使解析器提前发布它。 */
  memcpy(s_payload + 40U, s_poll, sizeof(s_poll));
  memcpy(s_payload + UBX_PAYLOAD_CAPACITY + 10U, s_ack, sizeof(s_ack));
  for (i = 0U; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
    Ubx_Result_t expected = (sizes[i] > UBX_PAYLOAD_CAPACITY) ? UBX_RESULT_OVERSIZE : UBX_RESULT_FRAME;
    Ubx_Init(&guarded.parser);
    size = ReferenceFrame(0x0AU, 0x04U, s_payload, sizes[i], s_wire);
    ExpectComplete(&guarded.parser, s_wire, size, expected);
    assert(guarded.parser.length == sizes[i]);
    if (expected == UBX_RESULT_FRAME) { assert(memcmp(guarded.parser.payload, s_payload, sizes[i]) == 0); }
    for (j = 0U; j < sizeof(guarded.prefix); j++) {
      assert(guarded.prefix[j] == 0xA5U && guarded.suffix[j] == 0xA5U);
    }
    ExpectComplete(&guarded.parser, s_ack, sizeof(s_ack), UBX_RESULT_FRAME);
  }

  Ubx_Init(&guarded.parser);
  size = ReferenceFrame(0x01U, 0x07U, s_payload, UBX_PAYLOAD_CAPACITY + 1U, s_wire);
  s_wire[size - 1U] ^= 0x80U;
  ExpectComplete(&guarded.parser, s_wire, size, UBX_RESULT_CHECKSUM_ERROR);
  ExpectComplete(&guarded.parser, s_poll, sizeof(s_poll), UBX_RESULT_FRAME);

  /* 构帧返回值的极限 65535 可表示；再增加一个载荷字节必须失败。 */
  size = ReferenceFrame(1U, 7U, s_payload, 65527U, s_wire);
  assert(size == UINT16_MAX);
  assert(Ubx_BuildFrame(1U, 7U, s_payload, 65527U, s_payload, UINT16_MAX) == UINT16_MAX);
  assert(memcmp(s_payload, s_wire, UINT16_MAX) == 0);
}

/** @brief 执行所有协议级回归。 */
int main(void)
{
  TestKnownVectorsAndBuild();
  TestStreamingAndFragmentation();
  TestChecksumAndTruncation();
  TestReceivingState();
  TestLengthBoundsAndMessageSizes();
  puts("UBX codec tests passed");
  return 0;
}
