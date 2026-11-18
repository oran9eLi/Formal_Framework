/**
 * @file test_rgb_platform.c
 * @brief 主机侧验证真实F407适配器的GRB编码、复位尾槽和返回码映射。
 * @details 复用现有IMU平台夹具提供完整HAL类型替身，重命名其入口；
 *          本测试仅调用RGB接口，不运行IMU场景，不模拟真实DMA或电气波形。
 */
#define main TestRgbFixtureImuMain
#include "test_bmi088_platform.c"
#undef main

static BSP_Status_t s_rgb_init_result = BSP_STATUS_OK;
static BSP_Status_t s_rgb_send_result = BSP_STATUS_OK;
static BSP_Status_t s_rgb_service_result = BSP_STATUS_OK;
static BSP_Status_t s_rgb_timing_result = BSP_STATUS_OK;
static uint16_t s_rgb_frame[124]; /**< 捕获的比较值帧，单位定时器tick。 */
static uint16_t s_rgb_period = 210U, s_rgb_zero = 50U, s_rgb_one = 160U;
static uint16_t s_rgb_count; /**< 捕获帧长度，单位半字。 */
static unsigned s_rgb_sends, s_rgb_deinits;
static uint32_t s_rgb_service_ms; /**< 透传给BSP服务的真实毫秒值。 */

/** @brief 初始化仅返回指定的软件结果。 */
BSP_Status_t BSP_RGB_Init(void) { return s_rgb_init_result; }
/** @brief 记录仅RGB资源撤收。 */
BSP_Status_t BSP_RGB_DeInit(void) { s_rgb_deinits++; return BSP_STATUS_OK; }
/** @brief 捕获栈上帧的副本，符合BSP异步复制接口。 */
BSP_Status_t BSP_RGB_Send(const uint16_t *duty, uint16_t count)
{
  assert(s_critical_depth == 0U);
  assert(duty != 0 && count == 124U);
  s_rgb_sends++;
  s_rgb_count = count;
  memcpy(s_rgb_frame, duty, sizeof(s_rgb_frame));
  return s_rgb_send_result;
}
/** @brief 保存透传时刻，验证适配器未进行tick或微秒换算。 */
BSP_Status_t BSP_RGB_Service(uint32_t now_ms)
{
  s_rgb_service_ms = now_ms;
  return s_rgb_service_result;
}
/** @brief 提供168MHz/800kHz定时参数或未就绪状态。 */
BSP_Status_t BSP_RGB_GetTiming(uint16_t *period, uint16_t *zero, uint16_t *one)
{
  *period = s_rgb_period; *zero = s_rgb_zero; *one = s_rgb_one;
  return s_rgb_timing_result;
}

/** @brief 从捕获脉宽解码一个字节，拒绝非0/1合法宽度。 */
static uint8_t DecodeByte(unsigned first)
{
  unsigned index;
  uint8_t value = 0U;
  for (index = first; index < first + 8U; index++) {
    assert(s_rgb_frame[index] == 50U || s_rgb_frame[index] == 160U);
    value = (uint8_t)((value << 1U) | (s_rgb_frame[index] == 160U ? 1U : 0U));
  }
  return value;
}

/** @brief 独立解码检查通道顺序、高位优先及完整100槽低电平复位。 */
static void CheckColor(uint8_t red, uint8_t green, uint8_t blue)
{
  unsigned index;
  assert(Px4Lite_PlatformRgbSetColor(red, green, blue) == PX4LITE_OK);
  assert(s_rgb_count == 124U);
  assert(DecodeByte(0U) == green);
  assert(DecodeByte(8U) == red);
  assert(DecodeByte(16U) == blue);
  for (index = 24U; index < s_rgb_count; index++) { assert(s_rgb_frame[index] == 0U); }
}

/** @brief 检查GRB边界、错误拒绝及状态映射；不重复生产编码算法。 */
int main(void)
{
  unsigned sends;
  CheckColor(0U, 0U, 0U);
  CheckColor(255U, 0U, 0U);
  CheckColor(0U, 255U, 0U);
  CheckColor(0U, 0U, 255U);
  CheckColor(255U, 255U, 255U);
  CheckColor(0x81U, 0x42U, 0x24U);
  assert(s_rgb_frame[0] == 50U && s_rgb_frame[1] == 160U); /* G=0x42，高位先发。 */
  assert(s_rgb_frame[8] == 160U && s_rgb_frame[9] == 50U); /* R=0x81，与G可区分。 */
  s_rgb_send_result = BSP_STATUS_BUSY;
  assert(Px4Lite_PlatformRgbSetColor(1U, 2U, 3U) == PX4LITE_BUSY);
  s_rgb_send_result = BSP_STATUS_ERROR;
  assert(Px4Lite_PlatformRgbSetColor(1U, 2U, 3U) == PX4LITE_IO_ERROR);
  s_rgb_send_result = BSP_STATUS_OK;
  sends = s_rgb_sends;
  s_rgb_timing_result = BSP_STATUS_ERROR;
  assert(Px4Lite_PlatformRgbSetColor(1U, 2U, 3U) == PX4LITE_NOT_READY);
  assert(s_rgb_sends == sends);
  s_rgb_timing_result = BSP_STATUS_OK;
  s_rgb_zero = 0U;
  assert(Px4Lite_PlatformRgbSetColor(1U, 2U, 3U) == PX4LITE_IO_ERROR);
  s_rgb_zero = 50U; s_rgb_one = s_rgb_period;
  assert(Px4Lite_PlatformRgbSetColor(1U, 2U, 3U) == PX4LITE_IO_ERROR);
  assert(s_rgb_sends == sends);
  s_rgb_one = 160U;
  assert(Px4Lite_PlatformRgbInit() == PX4LITE_OK);
  s_rgb_init_result = BSP_STATUS_ERROR;
  assert(Px4Lite_PlatformRgbInit() == PX4LITE_IO_ERROR);
  s_rgb_init_result = BSP_STATUS_BUSY;
  assert(Px4Lite_PlatformRgbInit() == PX4LITE_BUSY);
  assert(Px4Lite_PlatformRgbService(0xFFFFFFF0U) == PX4LITE_OK);
  assert(s_rgb_service_ms == 0xFFFFFFF0U);
  s_rgb_service_result = BSP_STATUS_BUSY;
  assert(Px4Lite_PlatformRgbService(3U) == PX4LITE_BUSY);
  assert(s_rgb_service_ms == 3U);
  s_rgb_service_result = BSP_STATUS_TIMEOUT;
  assert(Px4Lite_PlatformRgbService(4U) == PX4LITE_IO_ERROR);
  Px4Lite_PlatformRgbDeInit();
  assert(s_rgb_deinits == 1U);
  puts("RGB platform: GRB MSB encoding, channel bounds, 124 halfwords, reset tail, result mapping PASS");
  return 0;
}
