/**
 * @file px4lite_startup_tone.c
 * @brief Framework层以绝对时间推进一次性启动音，不阻塞或动态分配。
 * @details 依赖Platform及FreeRTOS快照临界区；不包含BSP/HAL，不涉及页面或RGB。
 *          旋律槽依据PX4 STARTUP，固定表替代运行时MML解析。
 */
/*
 * Startup melody derived from PX4 src/lib/tunes/tune_definition.desc.
 * Copyright (c) 2018 - 2025 PX4 Development Team. All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "px4lite_startup_tone.h"
#include "px4lite_config.h"
#include "px4lite_platform.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static Px4Lite_StartupToneStatus_t s_status; /**< 已发布快照，只在极短临界区复制。 */
#if PX4LITE_ENABLE_STARTUP_TONE
static Px4Lite_StartupToneStatus_t s_player; /**< sensor私有播放状态。 */
/* PX4的note_to_frequency使用note46=880Hz；这里不是常见乐谱的O4A=440Hz。
 * T240/Normal：先9个125000us槽，再8个62500us槽；每槽发声7/8。 */
static const uint16_t s_frequency_hz[17] = {
  1760U, 2349U, 2093U, 1760U, 2349U, 2093U, 1760U, 2349U, 2093U,
  2349U, 2093U, 2349U, 2093U, 2349U, 2093U, 2349U, 2093U
};

/** @brief 仅复制状态，不在临界区驱动硬件或计算音符。 */
static void StartupTone_Publish(void)
{
  taskENTER_CRITICAL();
  s_status = s_player;
  taskEXIT_CRITICAL();
}
#endif

/** @brief 可选硬件失败仅停用试听，不改变传感器注册和关键启动门禁。 */
void Px4Lite_StartupToneInit(uint32_t now_us)
{
#if PX4LITE_ENABLE_STARTUP_TONE
  memset(&s_player, 0, sizeof(s_player));
  s_player.start_us = now_us;
  s_player.note_index = 255U;
  s_player.result = Px4Lite_PlatformBuzzerInit();
  s_player.state = (s_player.result == PX4LITE_OK) ? PX4LITE_TONE_WAITING : PX4LITE_TONE_ERROR;
  if (s_player.result != PX4LITE_OK) { s_player.error_count = 1U; }
  StartupTone_Publish();
#else
  (void)now_us;
  taskENTER_CRITICAL();
  memset(&s_status, 0, sizeof(s_status));
  s_status.note_index = 255U;
  taskEXIT_CRITICAL();
#endif
}

/** @brief 绝对经过时间决定输出；正常单次调用O(1)，无补播或等待循环。 */
void Px4Lite_StartupToneRun(uint32_t now_us)
{
#if PX4LITE_ENABLE_STARTUP_TONE
  uint32_t elapsed_us;
  uint32_t slot_us;
  uint32_t within_us;
  uint16_t frequency_hz = 0U;
  uint16_t previous_frequency_hz = s_player.frequency_hz;
  uint8_t note_index = 255U;
  uint8_t state;
  if ((s_player.state != PX4LITE_TONE_WAITING) && (s_player.state != PX4LITE_TONE_PLAYING)) { return; }
  elapsed_us = now_us - s_player.start_us;
  if (elapsed_us < PX4LITE_STARTUP_TONE_DELAY_MS * 1000U) { return; }
  elapsed_us -= PX4LITE_STARTUP_TONE_DELAY_MS * 1000U;
  state = PX4LITE_TONE_DONE;
  if (elapsed_us < 1625000U) {
    state = PX4LITE_TONE_PLAYING;
    if (elapsed_us < 1125000U) {
      slot_us = 125000U;
      note_index = (uint8_t)(elapsed_us / slot_us);
      within_us = elapsed_us % slot_us;
    } else {
      slot_us = 62500U;
      elapsed_us -= 1125000U;
      note_index = (uint8_t)(9U + elapsed_us / slot_us);
      within_us = elapsed_us % slot_us;
    }
    if (within_us < slot_us - slot_us / 8U) { frequency_hz = s_frequency_hz[note_index]; }
  }
  if (frequency_hz != s_player.frequency_hz) {
    s_player.result = Px4Lite_PlatformBuzzerSetFrequency(frequency_hz);
    if (s_player.result != PX4LITE_OK) {
      /* 错误撤收走同一平台边界，不触碰电机或其他外设。 */
      (void)Px4Lite_PlatformBuzzerSetFrequency(0U);
      s_player.error_count++;
      state = PX4LITE_TONE_ERROR;
      frequency_hz = 0U;
    }
  }
  s_player.frequency_hz = frequency_hz;
  if ((s_player.state != state) || (s_player.note_index != note_index) ||
      (previous_frequency_hz != frequency_hz)) {
    s_player.state = state;
    s_player.note_index = note_index;
    StartupTone_Publish();
  }
#else
  (void)now_us;
#endif
}

/** @brief 提供无硬件在位承诺的快照，供调试或未来App API封装。 */
Px4Lite_Result_t Px4Lite_StartupToneCopyStatus(Px4Lite_StartupToneStatus_t *out)
{
  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  taskENTER_CRITICAL();
  *out = s_status;
  taskEXIT_CRITICAL();
  return PX4LITE_OK;
}
