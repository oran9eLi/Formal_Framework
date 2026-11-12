/**
 * @file px4lite_startup_tone.c
 * @brief Framework层单owner推进启动音、慢渐变和树莓派本机控制/配置收包双鸣/双闪。
 * @details 依赖Platform及FreeRTOS快照临界区；不包含BSP/HAL，不阻塞或动态分配。
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
#if PX4LITE_ENABLE_STARTUP_TONE || PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
static Px4Lite_StartupToneStatus_t s_player; /**< sensor私有播放状态。 */
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
static volatile uint8_t s_feedback_pending; /**< 仅Notify/owner短临界区访问，0～8。 */
static volatile uint32_t s_feedback_received,s_feedback_dropped; /**< 上电请求/队满事实，uint32回绕。 */
static uint32_t s_feedback_started,s_feedback_finished,s_feedback_buzzer_errors; /**< sensor独占累计事实。 */
static uint32_t s_feedback_start_us,s_buzzer_retry_ms; /**< 当前提示槽起点us、蜂鸣器恢复起点ms。 */
static Px4Lite_Result_t s_feedback_buzzer_result; /**< 最近提示蜂鸣器结果。 */
static uint8_t s_feedback_active,s_feedback_on; /**< 当前提示时间轴与响停状态。 */
#if PX4LITE_ENABLE_STARTUP_RGB
static uint8_t s_feedback_rgb_override; /**< RGB启用时的临时提示占用事实。 */
#endif
static uint8_t s_feedback_phase; /**< 当前响/停/响/停槽，0～3；迟到时不跳槽。 */
static uint8_t s_buzzer_ready,s_buzzer_retry_wait; /**< 蜂鸣器就绪与2秒恢复门控。 */
#if !PX4LITE_ENABLE_STARTUP_TONE && PX4LITE_ENABLE_STARTUP_RGB
static uint8_t s_feedback_ever_started; /**< 关闭启动音时，首个提示后才允许RGB硬件运行。 */
#endif
#endif
#if PX4LITE_ENABLE_STARTUP_TONE
/* PX4的note_to_frequency使用note46=880Hz；这里不是常见乐谱的O4A=440Hz。
 * T240/Normal：先9个125000us槽，再8个62500us槽；每槽发声7/8。 */
static const uint16_t s_frequency_hz[17] = {
  1760U, 2349U, 2093U, 1760U, 2349U, 2093U, 1760U, 2349U, 2093U,
  2349U, 2093U, 2349U, 2093U, 2349U, 2093U, 2349U, 2093U
};
#endif

#if PX4LITE_ENABLE_STARTUP_RGB
#if (PX4LITE_STARTUP_RGB_IDLE_COLOR_MS < PX4LITE_STARTUP_RGB_IDLE_UPDATE_MS) || \
    (PX4LITE_STARTUP_RGB_IDLE_COLOR_MS > 60000U) || \
    (PX4LITE_STARTUP_RGB_IDLE_UPDATE_MS < 20U) || (PX4LITE_STARTUP_RGB_IDLE_UPDATE_MS > 1000U)
#error "Startup RGB idle timing must use 20..1000ms update and update..60000ms color duration"
#endif
/** @brief 七色低亮度调色表，通道0～48；播放中按音符轮换，结束后相邻色线性渐变。 */
static const uint8_t s_colors[7][3] = {
  {48U, 0U, 0U}, {48U, 16U, 0U}, {32U, 32U, 0U}, {0U, 48U, 0U},
  {0U, 32U, 32U}, {0U, 0U, 48U}, {32U, 0U, 32U}
};
static uint32_t s_rgb_retry_ms; /**< 最近初始化或失败时刻，平台毫秒，可回绕。 */
static uint8_t s_rgb_color_valid; /**< 当前颜色已被硬件接受；故障后必须重新发送。 */
static uint32_t s_rgb_idle_last_ms; /**< 上轮常亮相位推进时刻，平台真实毫秒，可回绕。 */
static uint32_t s_rgb_idle_phase_ms; /**< 七色渐变环内相位，范围0～7*IDLE_COLOR_MS-1，单位ms。 */
static uint32_t s_rgb_idle_sent_ms; /**< 最近成功提交常亮颜色的时刻，平台真实毫秒。 */
static uint8_t s_rgb_idle_started; /**< 正常播放结束后首次进入常亮渐变时置1。 */
static uint8_t s_rgb_idle_sent; /**< 已提交常亮帧时为1；播放结束首帧立即发送。 */

/**
 * @brief 以真实毫秒累计有界循环相位，从最后音符颜色开始平滑切换且不经过黑色。
 * @note 必须在BUSY或恢复早退前推进，延迟后只输出当前颜色；先对增量取模再相加，
 *       避免长时间运行或毫秒时钟回绕改变七色相位。单次O(1)，不补播历史帧。
 */
static void StartupRgb_IdleColor(uint32_t now_ms, uint8_t color[3])
{
  const uint32_t cycle_ms = 7U * PX4LITE_STARTUP_RGB_IDLE_COLOR_MS;
  uint32_t within_ms;
  uint8_t from, to, channel;
  if (s_rgb_idle_started == 0U) {
    s_rgb_idle_started = 1U;
    s_rgb_idle_phase_ms = 0U;
  } else {
    uint32_t delta_ms = now_ms - s_rgb_idle_last_ms;
    s_rgb_idle_phase_ms = (s_rgb_idle_phase_ms + delta_ms % cycle_ms) % cycle_ms;
  }
  s_rgb_idle_last_ms = now_ms;
  /* 第17音符是七色表第3色（黄）；正常结束后从此色开始，不重启旋律。 */
  from = (uint8_t)((s_rgb_idle_phase_ms / PX4LITE_STARTUP_RGB_IDLE_COLOR_MS + 16U % 7U) % 7U);
  to = (uint8_t)((from + 1U) % 7U);
  within_ms = s_rgb_idle_phase_ms % PX4LITE_STARTUP_RGB_IDLE_COLOR_MS;
  for (channel = 0U; channel < 3U; ++channel) {
    /* 非负加权避免下降通道下溢，四舍五入后保持0～48的原低亮范围。 */
    color[channel] = (uint8_t)(((uint32_t)s_colors[from][channel] *
        (PX4LITE_STARTUP_RGB_IDLE_COLOR_MS - within_ms) +
        (uint32_t)s_colors[to][channel] * within_ms + PX4LITE_STARTUP_RGB_IDLE_COLOR_MS / 2U) /
        PX4LITE_STARTUP_RGB_IDLE_COLOR_MS);
  }
}

/** @brief 故障只撤收RGB，保留错误事实；sensor在2s后按当前目标重建，不补播旧颜色。 */
static void StartupRgb_Fail(uint32_t now_ms, Px4Lite_Result_t result)
{
  Px4Lite_PlatformRgbDeInit();
  s_player.rgb_result = result;
  s_player.rgb_error_count++;
  s_player.rgb_ready = 0U;
  s_player.rgb_busy = 0U;
  s_rgb_color_valid = 0U;
  s_rgb_retry_ms = now_ms;
}

/**
 * @brief 播放时按音调闪灯，正常结束后常亮慢渐变，持续处理尾帧/超时/限速恢复。
 * @note BUSY时保留最新目标，不覆盖DMA缓冲；最短静音7.8125ms可能被10ms调度跨过。
 *       该限制同样作用于原蜂鸣器，灯光始终跟随本轮实际提交的音调状态。
 */
static void StartupRgb_Run(uint32_t now_ms)
{
  uint8_t red = 0U, green = 0U, blue = 0U;
  Px4Lite_Result_t result;
  if ((s_player.state == PX4LITE_TONE_PLAYING) && (s_player.frequency_hz != 0U) &&
      (s_player.note_index < 17U)) {
    const uint8_t *color = s_colors[s_player.note_index % 7U];
    red = color[0]; green = color[1]; blue = color[2];
  } else if (s_player.state == PX4LITE_TONE_DONE) {
    uint8_t color[3];
    StartupRgb_IdleColor(now_ms, color);
    red = color[0]; green = color[1]; blue = color[2];
  }
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
  /* 先推进渐变相位再覆盖提示，结束后直接回到当前相位而不是补播旧灯色。 */
  if(s_feedback_rgb_override) {
    red=s_feedback_on?48U:0U;green=red;blue=red;
  }
#endif
  if (s_player.rgb_ready == 0U) {
    if ((uint32_t)(now_ms - s_rgb_retry_ms) < PX4LITE_STARTUP_RGB_RECOVERY_MS) { return; }
    if (s_player.rgb_error_count != 0U) { s_player.rgb_recovery_count++; }
    s_rgb_retry_ms = now_ms;
    result = Px4Lite_PlatformRgbInit();
    if (result != PX4LITE_OK) { StartupRgb_Fail(now_ms, result); return; }
    s_player.rgb_ready = 1U;
    s_rgb_color_valid = 0U;
  }
  result = Px4Lite_PlatformRgbService(now_ms);
  s_player.rgb_result = result;
  s_player.rgb_busy = (result == PX4LITE_BUSY) ? 1U : 0U;
  if (result == PX4LITE_BUSY) { return; }
  if (result != PX4LITE_OK) { StartupRgb_Fail(now_ms, result); return; }
  if ((s_player.state == PX4LITE_TONE_DONE) && (s_rgb_color_valid != 0U) &&
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
      !s_feedback_rgb_override &&
#endif
      (s_rgb_idle_sent != 0U) &&
      ((uint32_t)(now_ms - s_rgb_idle_sent_ms) < PX4LITE_STARTUP_RGB_IDLE_UPDATE_MS)) { return; }
  if ((s_rgb_color_valid != 0U) && (s_player.rgb_red == red) &&
      (s_player.rgb_green == green) && (s_player.rgb_blue == blue)) { return; }
  result = Px4Lite_PlatformRgbSetColor(red, green, blue);
  s_player.rgb_result = result;
  if (result == PX4LITE_OK) {
    s_player.rgb_red = red; s_player.rgb_green = green; s_player.rgb_blue = blue;
    s_player.rgb_busy = 1U;
    s_rgb_color_valid = 1U;
    if (s_player.state == PX4LITE_TONE_DONE) {
      s_rgb_idle_sent_ms = now_ms;
      s_rgb_idle_sent = 1U;
    }
  } else if (result != PX4LITE_BUSY) {
    StartupRgb_Fail(now_ms, result);
  }
}
#endif

/** @brief 仅复制状态，不在临界区驱动硬件或计算音符。 */
static void StartupTone_Publish(void)
{
  taskENTER_CRITICAL();
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
  s_player.feedback_received=s_feedback_received;s_player.feedback_dropped=s_feedback_dropped;
  s_player.feedback_pending=s_feedback_pending;s_player.feedback_active=s_feedback_active;
  s_player.feedback_started=s_feedback_started;s_player.feedback_finished=s_feedback_finished;
  s_player.feedback_buzzer_errors=s_feedback_buzzer_errors;s_player.feedback_buzzer_result=s_feedback_buzzer_result;
#endif
  s_status = s_player;
  taskEXIT_CRITICAL();
}
#endif

/** @brief sensor启动初始化声光owner；启动音失败仍保留独立提示与RGB故障隔离。 */
void Px4Lite_StartupToneInit(uint32_t now_us)
{
#if PX4LITE_ENABLE_STARTUP_TONE || PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
  memset(&s_player, 0, sizeof(s_player));
  s_player.rgb_result = PX4LITE_NOT_READY;
  s_player.note_index = 255U;
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
  /* Comm可能在sensor启动前已登记请求，不清pending/received/dropped累计。 */
  s_feedback_active=0U;s_feedback_on=0U;
#if PX4LITE_ENABLE_STARTUP_RGB
  s_feedback_rgb_override=0U;
#endif
  s_buzzer_ready=0U;s_buzzer_retry_wait=0U;
#if !PX4LITE_ENABLE_STARTUP_TONE && PX4LITE_ENABLE_STARTUP_RGB
  s_feedback_ever_started=0U;
#endif
  s_feedback_buzzer_result=PX4LITE_NOT_READY;
#endif
#if PX4LITE_ENABLE_STARTUP_TONE
  s_player.start_us = now_us;
  s_player.result = Px4Lite_PlatformBuzzerInit();
  s_player.state = (s_player.result == PX4LITE_OK) ? PX4LITE_TONE_WAITING : PX4LITE_TONE_ERROR;
  if (s_player.result != PX4LITE_OK) { s_player.error_count = 1U; }
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
  s_buzzer_ready=(uint8_t)(s_player.result==PX4LITE_OK);
  s_buzzer_retry_wait=(uint8_t)!s_buzzer_ready;s_buzzer_retry_ms=Px4Lite_PlatformGetMs();
#endif
#else
  (void)now_us;s_player.state=PX4LITE_TONE_DISABLED;
#endif
#if PX4LITE_ENABLE_STARTUP_RGB
  s_rgb_color_valid = 0U;
  s_rgb_idle_started = 0U;
  s_rgb_idle_sent = 0U;
  s_rgb_retry_ms = Px4Lite_PlatformGetMs() - PX4LITE_STARTUP_RGB_RECOVERY_MS;
#if PX4LITE_ENABLE_STARTUP_TONE
  StartupRgb_Run(Px4Lite_PlatformGetMs());
#endif
#endif
  StartupTone_Publish();
#else
  (void)now_us;
  taskENTER_CRITICAL();
  memset(&s_status, 0, sizeof(s_status));
  s_status.note_index = 255U;
  taskEXIT_CRITICAL();
#endif
}

#if PX4LITE_ENABLE_STARTUP_TONE
/** @brief 绝对经过时间决定音调；单次调用O(1)，无补播或等待循环。 */
static void StartupTone_Update(uint32_t now_us)
{
  uint32_t elapsed_us;
  uint32_t slot_us;
  uint32_t within_us;
  uint16_t frequency_hz = 0U;
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
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
      s_buzzer_ready=0U;s_buzzer_retry_wait=1U;s_buzzer_retry_ms=Px4Lite_PlatformGetMs();
#endif
    }
  }
  s_player.frequency_hz = frequency_hz;
  s_player.state = state;
  s_player.note_index = note_index;
}
#endif

/** @brief 任何协议校验均由Comm完成；本入口不启动硬件或抢占已播放的双鸣。 */
void Px4Lite_StartupToneNotifyRpiCommand(void)
{
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
  taskENTER_CRITICAL();
  s_feedback_received++;
  if(s_feedback_pending<PX4LITE_RPI_FEEDBACK_PENDING_MAX) { s_feedback_pending++; }
  else { s_feedback_dropped++; }
  taskEXIT_CRITICAL();
#endif
}

#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
#if PX4LITE_RPI_FEEDBACK_PENDING_MAX < 1U || PX4LITE_RPI_FEEDBACK_PENDING_MAX > 255U || PX4LITE_RPI_FEEDBACK_SLOT_MS < 20U || PX4LITE_RPI_FEEDBACK_SLOT_MS > 1000U
#error "RPI feedback requires bounded pending1..255 and slot20..1000ms"
#endif
/** @brief 只在sensor内控制提示频率；失败后2秒再尝试，RGB不因蜂鸣器故障停止。 */
static void Feedback_SetFrequency(uint16_t frequency_hz,uint32_t now_ms)
{
  Px4Lite_Result_t result;
  if(frequency_hz && !s_buzzer_ready) {
    if(s_buzzer_retry_wait && (uint32_t)(now_ms-s_buzzer_retry_ms)<2000U) {
      s_feedback_buzzer_result=PX4LITE_NOT_READY;return;
    }
    result=Px4Lite_PlatformBuzzerInit();s_feedback_buzzer_result=result;
    if(result!=PX4LITE_OK) {
      s_feedback_buzzer_errors++;s_buzzer_retry_wait=1U;s_buzzer_retry_ms=now_ms;return;
    }
    s_buzzer_ready=1U;s_buzzer_retry_wait=0U;
  }
  if(s_player.frequency_hz==frequency_hz) { return; }
  result=Px4Lite_PlatformBuzzerSetFrequency(frequency_hz);s_feedback_buzzer_result=result;
  if(result!=PX4LITE_OK) {
    (void)Px4Lite_PlatformBuzzerSetFrequency(0U);s_player.frequency_hz=0U;
    s_feedback_buzzer_errors++;s_buzzer_ready=0U;s_buzzer_retry_wait=1U;s_buzzer_retry_ms=now_ms;return;
  }
  s_player.frequency_hz=frequency_hz;
}

/**
 * @brief 按真实经过时间推进响/停/响/停，每轮最多一槽，迟到只延长提示而不跳过第二声。
 * @details 启动旋律优先；关闭启动音时仅首提示lazy初始化。单轮O(1)，无等待或追赶循环。
 */
static void Feedback_Run(uint32_t now_us,uint32_t now_ms)
{
  uint32_t elapsed_us,slot_us=PX4LITE_RPI_FEEDBACK_SLOT_MS*1000U;
  uint8_t pending;
  s_feedback_on=0U;
#if PX4LITE_ENABLE_STARTUP_RGB
  s_feedback_rgb_override=0U;
#endif
  if(s_player.state==PX4LITE_TONE_WAITING || s_player.state==PX4LITE_TONE_PLAYING) { return; }
  if(!s_feedback_active) {
    taskENTER_CRITICAL();pending=s_feedback_pending;
    if(pending) { s_feedback_pending--; }
    taskEXIT_CRITICAL();
    if(!pending) { return; }
    s_feedback_active=1U;s_feedback_started++;s_feedback_start_us=now_us;s_feedback_phase=0U;
#if !PX4LITE_ENABLE_STARTUP_TONE && PX4LITE_ENABLE_STARTUP_RGB
    s_feedback_ever_started=1U;
#endif
  }
  elapsed_us=now_us-s_feedback_start_us;
  if(elapsed_us>=slot_us) { s_feedback_phase++;s_feedback_start_us=now_us; }
  if(s_feedback_phase>=4U) {
    s_feedback_active=0U;s_feedback_finished++;Feedback_SetFrequency(0U,now_ms);
    taskENTER_CRITICAL();pending=s_feedback_pending;taskEXIT_CRITICAL();
#if PX4LITE_ENABLE_STARTUP_RGB
    s_feedback_rgb_override=(uint8_t)(pending!=0U);
    if(!pending) { s_rgb_color_valid=0U; }
#endif
    return;
  }
  s_feedback_on=(uint8_t)(s_feedback_phase==0U || s_feedback_phase==2U);
#if PX4LITE_ENABLE_STARTUP_RGB
  s_feedback_rgb_override=1U;
#endif
  Feedback_SetFrequency(s_feedback_on?PX4LITE_RPI_FEEDBACK_HZ:0U,now_ms);
}
#endif

/** @brief 每轮先决定PWM再决定灯色；正常结束持续慢渐变，所有IO均在快照临界区之外。 */
void Px4Lite_StartupToneRun(uint32_t now_us)
{
#if PX4LITE_ENABLE_STARTUP_TONE || PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
#if PX4LITE_ENABLE_STARTUP_TONE
  StartupTone_Update(now_us);
#endif
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
  Feedback_Run(now_us,Px4Lite_PlatformGetMs());
#endif
#if PX4LITE_ENABLE_STARTUP_RGB
#if !PX4LITE_ENABLE_STARTUP_TONE
  if(s_feedback_ever_started)
#endif
  StartupRgb_Run(Px4Lite_PlatformGetMs());
#endif
  StartupTone_Publish();
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
