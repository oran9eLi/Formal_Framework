/**
 * @file test_startup_rgb.c
 * @brief 主机侧验证启动旋律、RGB节拍和结束后慢速常亮渐变、失败隔离及恢复。
 * @details 仅依赖真实Framework播放器及平台替身；不访问HAL、不启动任务，
 *          不把软件请求成功解释为实物发声、发光或DMA时序验收。
 */
#include "px4lite_startup_tone.h"
#include "px4lite_config.h"
#include "px4lite_platform.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/** @brief 平台输出替身；颜色和频率分别保存最近实际接受的请求。 */
typedef struct {
  uint32_t now_ms; /**< 独立平台毫秒时钟，可跨32位回绕。 */
  uint32_t service_ms; /**< 最近传给RGB服务的真实毫秒。 */
  unsigned buzzer_init_calls, tone_calls, rgb_init_calls, deinit_calls; /**< 初始化、音调、RGB初始化及撤收调用次数。 */
  unsigned color_calls, service_calls; /**< 颜色提交与服务调用次数，包含拒绝的请求。 */
  uint16_t frequency_hz; /**< 最近接受的频率，0表示静音。 */
  uint8_t red, green, blue; /**< 最近接受的颜色，各0～255。 */
  Px4Lite_Result_t buzzer_init_result, tone_result; /**< 蜂鸣器初始化与非零音调的注入结果。 */
  Px4Lite_Result_t rgb_init_result, color_result, service_result; /**< RGB初始化、发送和服务的注入结果。 */
} MockPlatform_t;

static MockPlatform_t s_mock;

/** @brief 提供独立于微秒参数的真实毫秒时钟。 */
uint32_t Px4Lite_PlatformGetMs(void) { return s_mock.now_ms; }
/** @brief 记录蜂鸣器初始化及注入结果。 */
Px4Lite_Result_t Px4Lite_PlatformBuzzerInit(void)
{
  s_mock.buzzer_init_calls++;
  return s_mock.buzzer_init_result;
}
/** @brief 停音始终可接受，以验证音调失败后的撤收请求。 */
Px4Lite_Result_t Px4Lite_PlatformBuzzerSetFrequency(uint16_t frequency_hz)
{
  s_mock.tone_calls++;
  if ((frequency_hz != 0U) && (s_mock.tone_result != PX4LITE_OK)) { return s_mock.tone_result; }
  s_mock.frequency_hz = frequency_hz;
  return PX4LITE_OK;
}
/** @brief 记录RGB初始化次数及注入结果。 */
Px4Lite_Result_t Px4Lite_PlatformRgbInit(void)
{
  s_mock.rgb_init_calls++;
  return s_mock.rgb_init_result;
}
/** @brief 只记录任务上下文撤收，不等待硬件。 */
void Px4Lite_PlatformRgbDeInit(void) { s_mock.deinit_calls++; }
/** @brief BUSY/失败不接受颜色，模拟DMA缓冲不能覆盖。 */
Px4Lite_Result_t Px4Lite_PlatformRgbSetColor(uint8_t red, uint8_t green, uint8_t blue)
{
  s_mock.color_calls++;
  if (s_mock.color_result == PX4LITE_OK) {
    s_mock.red = red; s_mock.green = green; s_mock.blue = blue;
  }
  return s_mock.color_result;
}
/** @brief 服务结果由测试注入，保存调用时钟核对毫秒语义。 */
Px4Lite_Result_t Px4Lite_PlatformRgbService(uint32_t now_ms)
{
  s_mock.service_calls++;
  s_mock.service_ms = now_ms;
  return s_mock.service_result;
}

/** @brief 清空替身，每个场景从独立的一次性启动事务开始。 */
static void Reset(void) { memset(&s_mock, 0, sizeof(s_mock)); }
/** @brief 按外部指定的两种时钟推进真实播放器。 */
static void Step(uint32_t now_us, uint32_t now_ms)
{
  s_mock.now_ms = now_ms;
  Px4Lite_StartupToneRun(now_us);
}
/** @brief 复制公开诊断接口，不读取播放器私有状态。 */
static Px4Lite_StartupToneStatus_t Status(void)
{
  Px4Lite_StartupToneStatus_t status;
  assert(Px4Lite_StartupToneCopyStatus(&status) == PX4LITE_OK);
  return status;
}

#if PX4LITE_ENABLE_STARTUP_TONE
/** @brief 延时、17个音符、静音间隙、结束和结束后持续服务均符合外部规格。 */
static void TestMelody(void)
{
  unsigned note, prior_calls;
#if PX4LITE_ENABLE_STARTUP_RGB
  uint8_t colors[7][3];
#endif
  Reset();
  Px4Lite_StartupToneInit(0U);
  assert(Status().state == PX4LITE_TONE_WAITING);
  Step(999999U, 999U);
  assert(s_mock.tone_calls == 0U);
  for (note = 0U; note < 17U; note++) {
    uint32_t offset = (note < 9U) ? note * 125000U : 1125000U + (note - 9U) * 62500U;
    uint32_t slot = (note < 9U) ? 125000U : 62500U;
    uint32_t start = 1000000U + offset;
    Step(start, start / 1000U);
    assert(Status().state == PX4LITE_TONE_PLAYING);
    assert(Status().note_index == note);
    assert(s_mock.frequency_hz > 0U);
#if PX4LITE_ENABLE_STARTUP_RGB
    assert((s_mock.red | s_mock.green | s_mock.blue) != 0U);
    if (note < 7U) {
      unsigned other;
      colors[note][0] = s_mock.red; colors[note][1] = s_mock.green; colors[note][2] = s_mock.blue;
      for (other = 0U; other < note; other++) { assert(memcmp(colors[other], colors[note], 3U) != 0); }
    } else {
      uint8_t actual[3] = {s_mock.red, s_mock.green, s_mock.blue};
      assert(memcmp(actual, colors[note % 7U], 3U) == 0);
    }
#endif
    Step(start + slot - slot / 8U, (start + slot - slot / 8U) / 1000U);
    assert(s_mock.frequency_hz == 0U);
#if PX4LITE_ENABLE_STARTUP_RGB
    assert((s_mock.red | s_mock.green | s_mock.blue) == 0U);
#endif
  }
  Step(2625000U, 2625U);
  assert(Status().state == PX4LITE_TONE_DONE);
  assert(s_mock.frequency_hz == 0U);
  prior_calls = s_mock.service_calls;
  Step(2800000U, 2800U);
  assert(Status().state == PX4LITE_TONE_DONE);
#if PX4LITE_ENABLE_STARTUP_RGB
  assert(s_mock.service_calls > prior_calls);
  assert(s_mock.service_ms == 2800U);
  assert((s_mock.red | s_mock.green | s_mock.blue) != 0U);
#else
  assert(s_mock.service_calls == prior_calls);
  assert(s_mock.rgb_init_calls == 0U);
  assert(s_mock.color_calls == 0U);
#endif
}

#if PX4LITE_ENABLE_STARTUP_RGB
/** @brief PWM失败独立终止旋律，并把已亮RGB目标切为黑色。 */
static void TestToneError(void)
{
  Reset();
  Px4Lite_StartupToneInit(0U);
  Step(1000000U, 1000U);
  assert(s_mock.red != 0U);
  s_mock.tone_result = PX4LITE_IO_ERROR;
  Step(1125000U, 1125U);
  assert(Status().state == PX4LITE_TONE_ERROR);
  assert(Status().error_count == 1U);
  assert(s_mock.frequency_hz == 0U);
  assert((s_mock.red | s_mock.green | s_mock.blue) == 0U);
  s_mock.tone_result = PX4LITE_OK;
  Step(1300000U, 1300U);
  assert(Status().state == PX4LITE_TONE_ERROR);
  assert(s_mock.frequency_hz == 0U);
  Step(4000000U, 4000U);
  assert(Status().state == PX4LITE_TONE_ERROR);
  assert((s_mock.red | s_mock.green | s_mock.blue) == 0U);
}

/** @brief 初始失败后按真实毫秒恢复；两种时钟各自回绕仍跳过过期颜色。 */
static void TestRecoveryAndWrap(void)
{
  uint32_t start_us = UINT32_MAX - 500000U;
  uint32_t start_ms = UINT32_MAX - 1000U;
  Reset();
  s_mock.now_ms = start_ms;
  s_mock.rgb_init_result = PX4LITE_IO_ERROR;
  Px4Lite_StartupToneInit(start_us);
  assert(Status().rgb_error_count == 1U);
  Step(start_us + 1000000U, start_ms + 1000U);
  assert(s_mock.frequency_hz != 0U);
  assert(s_mock.rgb_init_calls == 1U);
  s_mock.rgb_init_result = PX4LITE_OK;
  Step(start_us + 1999000U, start_ms + 1999U);
  assert(s_mock.rgb_init_calls == 1U);
  Step(start_us + 2000000U, start_ms + 2000U);
  assert(s_mock.rgb_init_calls == 2U);
  assert(Status().rgb_recovery_count == 1U);
  assert(Status().note_index == 8U);
  assert(s_mock.red != 0U && s_mock.green != 0U); /* 当前橙色，非过期首音红色。 */
  assert(s_mock.service_ms == start_ms + 2000U);
  Step(start_us + 2625000U, start_ms + 2625U);
  assert(Status().state == PX4LITE_TONE_DONE);
  s_mock.service_result = PX4LITE_IO_ERROR;
  Step(start_us + 2700000U, start_ms + 2700U);
  assert(Status().rgb_error_count == 2U);
  s_mock.service_result = PX4LITE_OK;
  Step(start_us + 4699000U, start_ms + 4699U);
  assert(s_mock.rgb_init_calls == 2U);
  Step(start_us + 4700000U, start_ms + 4700U);
  assert(s_mock.rgb_init_calls == 3U);
  assert((s_mock.red | s_mock.green | s_mock.blue) != 0U);
  assert(Status().state == PX4LITE_TONE_DONE);
}

/** @brief 微秒参数跳变不能提前触发毫秒恢复；发送错误也不影响音调。 */
static void TestTxFailureAndClockSeparation(void)
{
  Reset();
  Px4Lite_StartupToneInit(0U);
  s_mock.color_result = PX4LITE_IO_ERROR;
  Step(1000000U, 50U);
  assert(Status().rgb_error_count == 1U);
  assert(Status().state == PX4LITE_TONE_PLAYING);
  assert(s_mock.frequency_hz != 0U);
  s_mock.color_result = PX4LITE_OK;
  Step(2500000U, 2049U);
  assert(s_mock.rgb_init_calls == 1U);
  Step(2500000U, 2050U);
  assert(s_mock.rgb_init_calls == 2U);
  assert(Status().rgb_ready == 1U);
  assert(Status().state == PX4LITE_TONE_PLAYING);
  assert(Status().error_count == 0U);
}

/** @brief 硬件忙时不写颜色，空闲后只提交当前音符，拒绝积压历史帧。 */
static void TestBusyLatestTarget(void)
{
  unsigned calls;
  Reset();
  Px4Lite_StartupToneInit(0U);
  Step(1000000U, 1000U);
  assert(s_mock.red != 0U && s_mock.blue == 0U);
  calls = s_mock.color_calls;
  s_mock.service_result = PX4LITE_BUSY;
  Step(1125000U, 1125U);
  Step(1250000U, 1250U);
  assert(s_mock.color_calls == calls);
  assert(Status().rgb_busy == 1U);
  s_mock.service_result = PX4LITE_OK;
  Step(1625000U, 1625U);
  assert(s_mock.color_calls == calls + 1U);
  assert(s_mock.red == 0U && s_mock.green == 0U && s_mock.blue != 0U); /* 当前第6音蓝色。 */
  s_mock.color_result = PX4LITE_BUSY;
  Step(1750000U, 1750U);
  assert(Status().rgb_error_count == 0U);
  s_mock.color_result = PX4LITE_OK;
  Step(1875000U, 1875U);
  assert(s_mock.red != 0U && s_mock.green == 0U && s_mock.blue == 0U);
  assert(Status().rgb_error_count == 0U);
}

/** @brief RGB服务失败只撤收RGB，旋律继续且结束后恢复当前常亮渐变色。 */
static void TestServiceErrorIsolation(void)
{
  Reset();
  Px4Lite_StartupToneInit(0U);
  Step(1000000U, 1000U);
  s_mock.service_result = PX4LITE_IO_ERROR;
  Step(1125000U, 1125U);
  assert(Status().state == PX4LITE_TONE_PLAYING);
  assert(Status().error_count == 0U);
  assert(Status().rgb_error_count == 1U);
  assert(s_mock.frequency_hz != 0U);
  assert(s_mock.deinit_calls == 1U);
  s_mock.service_result = PX4LITE_OK;
  Step(2625000U, 2625U);
  assert(Status().state == PX4LITE_TONE_DONE);
  Step(3125000U, 3125U);
  assert(Status().rgb_ready == 1U);
  assert((s_mock.red | s_mock.green | s_mock.blue) != 0U);
}

/** @brief 复制已接受颜色，便于比较独立启动实例的同一相对时刻。 */
static void CopyColor(uint8_t color[3])
{
  color[0] = s_mock.red; color[1] = s_mock.green; color[2] = s_mock.blue;
}

/** @brief 用指定时钟起点完成启动；第一次DONE调用建立常亮渐变时间轴。 */
static void BeginAmbient(uint32_t start_us, uint32_t start_ms)
{
  Reset();
  s_mock.now_ms = start_ms;
  Px4Lite_StartupToneInit(start_us);
  Step(start_us + 2562500U, start_ms + 2562U);
  assert(s_mock.frequency_hz != 0U);
  Step(start_us + 2618000U, start_ms + 2618U);
  assert(s_mock.frequency_hz == 0U);
  Step(start_us + 2625000U, start_ms + 2625U);
  assert(Status().state == PX4LITE_TONE_DONE);
  assert(s_mock.frequency_hz == 0U);
  assert(s_mock.red == 32U && s_mock.green == 32U && s_mock.blue == 0U);
}

/** @brief 七色在14s内连续循环，相邻色2s，每50ms只作小幅改变且没有黑隙。 */
static void TestAmbientSmoothAndNoReplay(void)
{
  uint8_t previous[3], current[3], boundary[7][3];
  unsigned elapsed, prior_tones, prior_colors;
  BeginAmbient(0U, 0U);
  CopyColor(previous);
  memcpy(boundary[0], previous, 3U);
  prior_tones = s_mock.tone_calls;
  prior_colors = s_mock.color_calls;
  Step(2674000U, 2674U);
  CopyColor(current);
  assert(memcmp(previous, current, 3U) == 0);
  assert(s_mock.color_calls == prior_colors);
  for (elapsed = 50U; elapsed <= 14000U; elapsed += 50U) {
    unsigned channel;
    Step(2625000U + elapsed * 1000U, 2625U + elapsed);
    CopyColor(current);
    assert((current[0] | current[1] | current[2]) != 0U);
    for (channel = 0U; channel < 3U; channel++) {
      int delta = (int)current[channel] - (int)previous[channel];
      assert(delta >= -2 && delta <= 2);
      assert(current[channel] <= 48U);
    }
    if ((elapsed % 2000U) == 0U) {
      unsigned step = elapsed / 2000U;
      if (step < 7U) {
        unsigned other;
        memcpy(boundary[step], current, 3U);
        for (other = 0U; other < step; other++) { assert(memcmp(boundary[other], current, 3U) != 0); }
      } else { assert(memcmp(boundary[0], current, 3U) == 0); }
    }
    memcpy(previous, current, 3U);
    assert(Status().state == PX4LITE_TONE_DONE);
    assert(s_mock.frequency_hz == 0U && s_mock.tone_calls == prior_tones);
  }
}

/** @brief 硬件忙或失败2s后，恢复只提交当前渐变色，不重放被跳过的颜色。 */
static void TestAmbientBusyAndRecovery(void)
{
  uint8_t expected_busy[3], expected_recovered[3], actual[3];
  unsigned calls;
  BeginAmbient(0U, 0U);
  Step(3625000U, 3625U);
  CopyColor(expected_busy);
  Step(5825000U, 5825U);
  CopyColor(expected_recovered);
  BeginAmbient(0U, 0U);
  calls = s_mock.color_calls;
  s_mock.service_result = PX4LITE_BUSY;
  Step(2825000U, 2825U);
  Step(3125000U, 3125U);
  assert(s_mock.color_calls == calls);
  s_mock.service_result = PX4LITE_OK;
  Step(3625000U, 3625U);
  CopyColor(actual);
  assert(s_mock.color_calls == calls + 1U);
  assert(memcmp(expected_busy, actual, 3U) == 0);
  s_mock.service_result = PX4LITE_IO_ERROR;
  Step(3825000U, 3825U);
  assert(Status().rgb_error_count == 1U);
  assert(Status().state == PX4LITE_TONE_DONE);
  calls = s_mock.rgb_init_calls;
  s_mock.service_result = PX4LITE_OK;
  Step(5824000U, 5824U);
  assert(s_mock.rgb_init_calls == calls);
  Step(5825000U, 5825U);
  assert(s_mock.rgb_init_calls == calls + 1U);
  CopyColor(actual);
  assert(memcmp(expected_recovered, actual, 3U) == 0);
  assert(Status().rgb_recovery_count == 1U);
  assert(s_mock.frequency_hz == 0U);
}

/** @brief DONE后两种时钟独立回绕和长时间跳变仍保持渐变，永不重播启动音。 */
static void TestAmbientClockWrap(void)
{
  uint8_t reference[81][3], actual[3], long_reference[3];
  uint32_t start_us = UINT32_MAX - 4000000U;
  uint32_t start_ms = UINT32_MAX - 3000U;
  unsigned sample, tones;
  BeginAmbient(0U, 0U);
  for (sample = 0U; sample < 81U; sample++) {
    uint32_t elapsed_ms = sample * 250U;
    Step(2625000U + elapsed_ms * 1000U, 2625U + elapsed_ms);
    CopyColor(reference[sample]);
  }
  BeginAmbient(0U, 0U);
  {
    uint32_t phase_ms = (uint32_t)(((1ULL << 32U) + 123456U - 2625U) % 14000U);
    Step(2625000U + phase_ms * 1000U, 2625U + phase_ms);
    CopyColor(long_reference);
  }
  BeginAmbient(start_us, start_ms);
  tones = s_mock.tone_calls;
  for (sample = 0U; sample < 81U; sample++) {
    uint32_t elapsed_ms = sample * 250U;
    Step(start_us + 2625000U + elapsed_ms * 1000U, start_ms + 2625U + elapsed_ms);
    CopyColor(actual);
    assert(memcmp(reference[sample], actual, 3U) == 0);
    assert(Status().state == PX4LITE_TONE_DONE && s_mock.tone_calls == tones);
  }
  /* 已结束的旋律即使微秒数重新落入最初的发声窗，也不能重启。 */
  Step(start_us + 1000000U, start_ms + 0x80000000U);
  assert(Status().state == PX4LITE_TONE_DONE && s_mock.tone_calls == tones);
  assert((s_mock.red | s_mock.green | s_mock.blue) != 0U);
  Step(start_us + 1000000U, start_ms + 123456U);
  assert(Status().state == PX4LITE_TONE_DONE && s_mock.tone_calls == tones);
  assert((s_mock.red | s_mock.green | s_mock.blue) != 0U);
  CopyColor(actual);
  assert(memcmp(long_reference, actual, 3U) == 0);
}
#endif
#endif

/** @brief 执行当前编译开关对应的外部行为检查。 */
int main(void)
{
  assert(Px4Lite_StartupToneCopyStatus(0) == PX4LITE_INVALID_PARAM);
#if PX4LITE_ENABLE_STARTUP_TONE
  TestMelody();
#if PX4LITE_ENABLE_STARTUP_RGB
  TestToneError();
  TestRecoveryAndWrap();
  TestTxFailureAndClockSeparation();
  TestBusyLatestTarget();
  TestServiceErrorIsolation();
  TestAmbientSmoothAndNoReplay();
  TestAmbientBusyAndRecovery();
  TestAmbientClockWrap();
  puts("startup_rgb: 9 scenarios PASS (melody, seven colors, gaps, failures, busy, recovery, ambient smooth/wrap)");
#else
  puts("startup_rgb_off: melody PASS; no RGB calls");
#endif
#else
  Reset();
  Px4Lite_StartupToneInit(0U);
  Step(1000000U, 1000U);
  assert(Status().state == PX4LITE_TONE_DISABLED);
  assert(s_mock.buzzer_init_calls == 0U && s_mock.tone_calls == 0U);
  assert(s_mock.rgb_init_calls == 0U && s_mock.color_calls == 0U && s_mock.service_calls == 0U);
  puts("startup_tone_off: no output calls PASS");
#endif
  return 0;
}
