/**
 * @file px4lite_startup_tone.c
 * @brief Framework层单owner推进CNS高优先级动作、启动音、渐变、事件声光及背景曲。
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
/* 独立旧夹具未声明新开关时保持原播放器；生产默认由px4lite_config.h显式配置。 */
#ifndef PX4LITE_ENABLE_CNS_CONTROL
#define PX4LITE_ENABLE_CNS_CONTROL 0U
#endif
#if PX4LITE_ENABLE_MOTOR_MUSIC
#include "px4lite_topics.h"
#include "px4lite_control.h"
#include "px4lite_modules.h"
#ifndef PX4LITE_MOTOR_MUSIC_SCORE_INJECTED
#include "px4lite_motor_music_score.h"
#endif
#endif

/** @brief 共享播放器仅在至少一种事件开启时存在，不改变各入口的独立开关。 */
#define STARTUP_FEEDBACK_ENABLED (PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK || PX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK)
/** @brief 蜂鸣器输出服务同时供事件和电机背景使用，RGB仍只受原启动/事件条件控制。 */
#define STARTUP_BUZZER_SERVICE_ENABLED (STARTUP_FEEDBACK_ENABLED || PX4LITE_ENABLE_MOTOR_MUSIC || PX4LITE_ENABLE_CNS_CONTROL)
#define STARTUP_RGB_SERVICE_ENABLED (PX4LITE_ENABLE_STARTUP_RGB && (PX4LITE_ENABLE_STARTUP_TONE || STARTUP_FEEDBACK_ENABLED || PX4LITE_ENABLE_CNS_CONTROL))

static Px4Lite_StartupToneStatus_t s_status; /**< 已发布快照，只在极短临界区复制。 */
#if PX4LITE_ENABLE_STARTUP_TONE || STARTUP_BUZZER_SERVICE_ENABLED
static Px4Lite_StartupToneStatus_t s_player; /**< sensor私有播放状态。 */
#if STARTUP_FEEDBACK_ENABLED || PX4LITE_ENABLE_CNS_CONTROL
/** @brief 单音或静音相位；真实毫秒，不使用RTOS tick。 */
typedef struct {
  uint16_t frequency_hz; /**< 0静音，否则为PWM频率，单位Hz。 */
  uint16_t duration_ms; /**< 本相位最短保持时长，单位ms；迟到不补播。 */
} RpiFeedbackPhase_t;
#endif
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK || PX4LITE_ENABLE_CNS_CONTROL
/** @brief 原通用成功四音由CNS调速复用，避免维护两份节拍。 */
static const RpiFeedbackPhase_t s_feedback_success[] = {
  {1568U,60U},{0U,20U},{1976U,60U},{0U,20U},
  {2349U,80U},{0U,20U},{3136U,160U},{0U,PX4LITE_RPI_FEEDBACK_TAIL_MS}
};
#if STARTUP_RGB_SERVICE_ENABLED
/** @brief 成功四音对应绿/青快闪，通道0～48。 */
static const uint8_t s_feedback_success_colors[4][3] = {
  {0U,48U,8U},{0U,32U,48U},{8U,48U,16U},{0U,48U,48U}
};
#endif
#endif
#if PX4LITE_ENABLE_CNS_CONTROL
/** @brief 跨任务请求邮箱；仅短临界区复制/置标志，所有硬件仍由sensor处理。 */
static struct {
  uint32_t revision; /**< 每次登记递增的代次，防止旧owner提交覆盖新请求。 */
  uint32_t token; /**< 当前非零请求标识；0为取消。 */
  Px4Lite_Result_t result; /**< 当前token公开结果。 */
  uint8_t action; /**< CNS动作1～6；0取消。 */
  uint8_t clear_all; /**< 清旧播放源请求，直到owner消费前保持。 */
} s_cns_request;
static uint32_t s_cns_seen_revision,s_cns_owner_token; /**< sensor当前代次和非零token。 */
static uint32_t s_cns_phase_start_us,s_cns_last_us; /**< 相位起点/上轮服务时刻，单位us。 */
static uint8_t s_cns_active,s_cns_action,s_cns_phase; /**< owner占用、动作1～6和相位索引。 */
static uint8_t s_cns_phase_started,s_cns_owned; /**< 当前相位已实际开始/上轮间隔属于CNS。 */
#if STARTUP_RGB_SERVICE_ENABLED
static uint8_t s_cns_rgb_override,s_cns_rgb_lit,s_cns_clear_rgb; /**< CNS专用灯效/点亮/清旧黑帧标志。 */
#if !PX4LITE_ENABLE_STARTUP_TONE
static uint8_t s_cns_ever_started; /**< 关闭启动音时，首个CNS请求后才允许RGB常规服务。 */
#endif
#endif
static uint8_t CnsTone_Run(uint32_t now_us,uint32_t now_ms);
#endif
#if STARTUP_BUZZER_SERVICE_ENABLED
static uint8_t s_buzzer_ready,s_buzzer_retry_wait; /**< 共用蜂鸣器就绪与恢复门控，sensor独占。 */
static uint32_t s_buzzer_retry_ms; /**< 共用蜂鸣器恢复起点，单位真实ms。 */
#endif
#if PX4LITE_ENABLE_MOTOR_MUSIC
static uint32_t s_music_last_us; /**< 每次服务都更新的微秒基准，长暂停不累计墙钟时间。 */
static uint8_t s_music_owned; /**< 上一服务间隔是否由背景曲谱占用蜂鸣器，含休止。 */
static void MotorMusic_Reset(uint32_t now_us);
#endif
#if STARTUP_FEEDBACK_ENABLED
static volatile uint8_t s_feedback_pending; /**< 仅Notify/owner短临界区访问，0～配置上限。 */
static uint8_t s_feedback_queue[PX4LITE_RPI_FEEDBACK_PENDING_MAX]; /**< 临界区保护的四类事件FIFO，取值Px4Lite_FeedbackKind_t的0～3。 */
static uint8_t s_feedback_read,s_feedback_write; /**< FIFO读写位置，0～上限减1，仅短临界区访问。 */
static volatile uint32_t s_feedback_received,s_feedback_dropped; /**< 上电请求/队满事实，uint32回绕。 */
static uint32_t s_feedback_started,s_feedback_finished,s_feedback_buzzer_errors; /**< sensor独占累计事实。 */
static uint32_t s_feedback_start_us; /**< 当前提示槽起点，单位us。 */
static Px4Lite_Result_t s_feedback_buzzer_result; /**< 最近提示蜂鸣器结果。 */
static uint8_t s_feedback_active,s_feedback_succeeded; /**< 当前旋律占用事实；仅命令事件的结果为1成功/0失败。 */
static uint8_t s_feedback_kind = PX4LITE_FEEDBACK_NONE; /**< 当前或最近事件类别，255为尚无事件。 */
#if STARTUP_RGB_SERVICE_ENABLED
static uint8_t s_feedback_rgb_override; /**< RGB启用时的临时结果提示占用事实。 */
static uint8_t s_feedback_lit; /**< 当前相位要求点亮时为1；独立于蜂鸣器输出故障。 */
#if PX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK
/** @brief 模块插入两音分别用蓝/青色，通道不超过48。 */
static const uint8_t s_feedback_connected_colors[2][3] = {
  {0U,16U,48U},{0U,48U,32U}
};
#endif
#endif
static uint8_t s_feedback_phase; /**< 当前旋律相位，命令成功0～7/失败0～5、模块0～3，迟到时不跳相位。 */
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
/** @brief 用户选定的失败下行旋律，560ms旋律和间隙，另加尾静音。 */
static const RpiFeedbackPhase_t s_feedback_failure[] = {
  {880U,120U},{0U,40U},{659U,140U},{0U,40U},
  {440U,220U},{0U,PX4LITE_RPI_FEEDBACK_TAIL_MS}
};
#endif
#if PX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK
/** @brief 用户选定的插入双音，300ms旋律与间隙，另加尾静音。 */
static const RpiFeedbackPhase_t s_feedback_connected[] = {
  {1047U,80U},{0U,40U},{2093U,180U},{0U,PX4LITE_RPI_FEEDBACK_TAIL_MS}
};
/** @brief 用户选定的拔出双音，400ms旋律与间隙，另加尾静音。 */
static const RpiFeedbackPhase_t s_feedback_disconnected[] = {
  {2093U,140U},{0U,60U},{1047U,200U},{0U,PX4LITE_RPI_FEEDBACK_TAIL_MS}
};
#endif

#if !PX4LITE_ENABLE_STARTUP_TONE && STARTUP_RGB_SERVICE_ENABLED
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

#if STARTUP_RGB_SERVICE_ENABLED
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
#if STARTUP_FEEDBACK_ENABLED
  /* 先推进渐变相位再覆盖提示，结束后直接回到当前相位而不是补播旧灯色。 */
  if(s_feedback_rgb_override) {
    red=0U;green=0U;blue=0U;
    if(s_feedback_lit) {
      switch(s_feedback_kind) {
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
        case PX4LITE_FEEDBACK_COMMAND_SUCCEEDED: {
          const uint8_t *color=s_feedback_success_colors[s_feedback_phase/2U];
          red=color[0];green=color[1];blue=color[2];break;
        }
        case PX4LITE_FEEDBACK_COMMAND_FAILED:
          red=12U;green=1U;break; /* 原低亮暗红失败灯效保持。 */
#endif
#if PX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK
        case PX4LITE_FEEDBACK_MODULE_CONNECTED: {
          const uint8_t *color=s_feedback_connected_colors[s_feedback_phase/2U];
          red=color[0];green=color[1];blue=color[2];break;
        }
        case PX4LITE_FEEDBACK_MODULE_DISCONNECTED:
          red=12U;green=4U;break; /* 双音随低亮橙色闪烁，与命令失败暗红区分。 */
#endif
        default:break;
      }
    }
  }
#endif
#if PX4LITE_ENABLE_CNS_CONTROL
  /* CNS只为调速成功段定义新灯效；急停清旧提示时用黑帧，其他动作保持现有灯色。 */
  if(s_cns_rgb_override) {
    red=0U;green=0U;blue=0U;
    if(s_cns_rgb_lit) {
      const uint8_t *color=s_feedback_success_colors[s_cns_phase/2U];
      red=color[0];green=color[1];blue=color[2];
    }
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
#if STARTUP_FEEDBACK_ENABLED
      !s_feedback_rgb_override &&
#endif
#if PX4LITE_ENABLE_CNS_CONTROL
      !s_cns_rgb_override &&
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
#if PX4LITE_ENABLE_CNS_CONTROL
  uint32_t cns_token;
  uint8_t cns_action;
  Px4Lite_Result_t cns_result;
  taskENTER_CRITICAL();
  cns_token=s_cns_request.token;cns_action=s_cns_request.action;cns_result=s_cns_request.result;
  taskEXIT_CRITICAL();
  s_player.cns_token=cns_token;s_player.cns_action=cns_action;
  s_player.cns_result=cns_token?cns_result:PX4LITE_IDLE;
  s_player.cns_state=(s_player.cns_result==PX4LITE_BUSY)?
      (s_cns_active?PX4LITE_CNS_TONE_PLAYING:PX4LITE_CNS_TONE_PENDING):
      (s_player.cns_result==PX4LITE_OK)?PX4LITE_CNS_TONE_DONE:
      (s_player.cns_result==PX4LITE_IO_ERROR)?PX4LITE_CNS_TONE_ERROR:PX4LITE_CNS_TONE_IDLE;
#endif
  taskENTER_CRITICAL();
#if STARTUP_FEEDBACK_ENABLED
  s_player.feedback_received=s_feedback_received;s_player.feedback_dropped=s_feedback_dropped;
  s_player.feedback_pending=s_feedback_pending;s_player.feedback_active=s_feedback_active;
  s_player.feedback_succeeded=s_feedback_succeeded;s_player.feedback_kind=s_feedback_kind;
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
#if PX4LITE_ENABLE_STARTUP_TONE || STARTUP_BUZZER_SERVICE_ENABLED
  memset(&s_player, 0, sizeof(s_player));
  s_player.rgb_result = PX4LITE_NOT_READY;
  s_player.note_index = 255U;
  s_player.feedback_kind = PX4LITE_FEEDBACK_NONE;
#if PX4LITE_ENABLE_CNS_CONTROL
  s_cns_seen_revision=0U;s_cns_owner_token=0U;s_cns_phase_start_us=now_us;s_cns_last_us=now_us;
  s_cns_active=0U;s_cns_action=0U;s_cns_phase=0U;s_cns_phase_started=0U;s_cns_owned=0U;
#if STARTUP_RGB_SERVICE_ENABLED
  s_cns_rgb_override=0U;s_cns_rgb_lit=0U;s_cns_clear_rgb=0U;
#if !PX4LITE_ENABLE_STARTUP_TONE
  s_cns_ever_started=0U;
#endif
#endif
#endif
#if STARTUP_BUZZER_SERVICE_ENABLED
  s_buzzer_ready=0U;s_buzzer_retry_wait=0U;s_buzzer_retry_ms=Px4Lite_PlatformGetMs();
#endif
#if PX4LITE_ENABLE_MOTOR_MUSIC
  MotorMusic_Reset(now_us);
#endif
#if STARTUP_FEEDBACK_ENABLED
  /* 调用者可能在sensor启动前已登记结果，不清FIFO及received/dropped累计。 */
  s_feedback_active=0U;s_feedback_succeeded=0U;s_feedback_kind=PX4LITE_FEEDBACK_NONE;
#if STARTUP_RGB_SERVICE_ENABLED
  s_feedback_rgb_override=0U;s_feedback_lit=0U;
#endif

#if !PX4LITE_ENABLE_STARTUP_TONE && STARTUP_RGB_SERVICE_ENABLED
  s_feedback_ever_started=0U;
#endif
  s_feedback_buzzer_result=PX4LITE_NOT_READY;
#endif
#if PX4LITE_ENABLE_STARTUP_TONE
  s_player.start_us = now_us;
  s_player.result = Px4Lite_PlatformBuzzerInit();
  s_player.state = (s_player.result == PX4LITE_OK) ? PX4LITE_TONE_WAITING : PX4LITE_TONE_ERROR;
  if (s_player.result != PX4LITE_OK) { s_player.error_count = 1U; }
#if STARTUP_BUZZER_SERVICE_ENABLED
  s_buzzer_ready=(uint8_t)(s_player.result==PX4LITE_OK);
  s_buzzer_retry_wait=(uint8_t)!s_buzzer_ready;s_buzzer_retry_ms=Px4Lite_PlatformGetMs();
#endif
#else
  (void)now_us;s_player.state=PX4LITE_TONE_DISABLED;
#endif
#if STARTUP_RGB_SERVICE_ENABLED
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
  s_status.feedback_kind = PX4LITE_FEEDBACK_NONE;
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
#if STARTUP_BUZZER_SERVICE_ENABLED
      s_buzzer_ready=0U;s_buzzer_retry_wait=1U;s_buzzer_retry_ms=Px4Lite_PlatformGetMs();
#endif
    }
  }
  s_player.frequency_hz = frequency_hz;
  s_player.state = state;
  s_player.note_index = note_index;
}
#endif

#if STARTUP_FEEDBACK_ENABLED
/** @brief 两类调用者只写短临界区共享FIFO，事件分类在锁外完成，不访问声光硬件。 */
static void Feedback_Enqueue(uint8_t kind)
{
  taskENTER_CRITICAL();
  s_feedback_received++;
  if(s_feedback_pending<PX4LITE_RPI_FEEDBACK_PENDING_MAX) {
    s_feedback_queue[s_feedback_write]=kind;
    s_feedback_write++;
    if(s_feedback_write>=PX4LITE_RPI_FEEDBACK_PENDING_MAX) { s_feedback_write=0U; }
    s_feedback_pending++;
  } else { s_feedback_dropped++; }
  taskEXIT_CRITICAL();
}
#endif

/** @brief 调用者确认命令执行结果；独立命令开关关闭不影响模块提示。 */
void Px4Lite_StartupToneNotifyRpiResult(uint8_t succeeded)
{
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
  Feedback_Enqueue((succeeded==1U)?PX4LITE_FEEDBACK_COMMAND_SUCCEEDED:PX4LITE_FEEDBACK_COMMAND_FAILED);
#else
  (void)succeeded;
#endif
}

/** @brief 登记模块已确认连接边沿；只入队，检测/防抖由调用者负责。 */
void Px4Lite_StartupToneNotifyModuleConnection(uint8_t connected)
{
#if PX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK
  Feedback_Enqueue((connected==1U)?PX4LITE_FEEDBACK_MODULE_CONNECTED:PX4LITE_FEEDBACK_MODULE_DISCONNECTED);
#else
  (void)connected;
#endif
}

/** @brief 兼容旧收包入口，未知执行结果只可播放低音，不得误报成功。 */
void Px4Lite_StartupToneNotifyRpiCommand(void)
{
  Px4Lite_StartupToneNotifyRpiResult(0U);
}

/** @brief 只登记最新CNS请求；clear_all粘性位不会被紧随的急停音请求覆盖。 */
void Px4Lite_StartupToneCnsStart(uint8_t action,uint32_t token)
{
#if PX4LITE_ENABLE_CNS_CONTROL
  taskENTER_CRITICAL();
  s_cns_request.revision++;
  s_cns_request.token=token;
  s_cns_request.action=action;
  s_cns_request.result=token?PX4LITE_BUSY:PX4LITE_IDLE;
  taskEXIT_CRITICAL();
#else
  (void)action;(void)token;
#endif
}

/** @brief 取消即撤销旧token的完成资格；输出和队列清理由下一轮sensor执行。 */
void Px4Lite_StartupToneCnsCancel(uint8_t all)
{
#if PX4LITE_ENABLE_CNS_CONTROL
  taskENTER_CRITICAL();
  s_cns_request.revision++;
  s_cns_request.token=0U;s_cns_request.action=0U;s_cns_request.result=PX4LITE_IDLE;
  if(all) { s_cns_request.clear_all=1U; }
  taskEXIT_CRITICAL();
#else
  (void)all;
#endif
}

/** @brief 按token复制当前结果，取消或被替代请求永远不能再返回OK。 */
Px4Lite_Result_t Px4Lite_StartupToneCnsResult(uint32_t token)
{
#if PX4LITE_ENABLE_CNS_CONTROL
  Px4Lite_Result_t result;
  uint32_t current_token;
  taskENTER_CRITICAL();
  current_token=s_cns_request.token;result=s_cns_request.result;
  taskEXIT_CRITICAL();
  return (token && token==current_token)?result:PX4LITE_IDLE;
#else
  (void)token;return PX4LITE_IDLE;
#endif
}

#if STARTUP_FEEDBACK_ENABLED
#if PX4LITE_RPI_FEEDBACK_PENDING_MAX < 1U || PX4LITE_RPI_FEEDBACK_PENDING_MAX > 255U || PX4LITE_RPI_FEEDBACK_TAIL_MS < 20U || PX4LITE_RPI_FEEDBACK_TAIL_MS > 1000U
#error "RPI feedback requires bounded pending1..255 and tail20..1000ms"
#endif
#endif

#if STARTUP_BUZZER_SERVICE_ENABLED
/**
 * @brief sensor唯一蜂鸣器输出边界，共享就绪/2s退避但不混用不同播放源的诊断。
 * @param[in] frequency_hz 请求频率，单位Hz；0为静音。
 * @param[in] now_ms 当前真实毫秒时刻。
 * @param[out] failed 本轮硬件初始化/写入实际失败为1，退避等待不计失败。
 * @retval PX4LITE_IDLE 频率已相同且无需硬件操作，保留调用者原诊断结果。
 */
static Px4Lite_Result_t StartupBuzzer_Apply(uint16_t frequency_hz,uint32_t now_ms,uint8_t *failed)
{
  Px4Lite_Result_t result;
  uint8_t initialized=0U;
  *failed=0U;
  if(frequency_hz && !s_buzzer_ready) {
    if(s_buzzer_retry_wait && (uint32_t)(now_ms-s_buzzer_retry_ms)<2000U) { return PX4LITE_NOT_READY; }
    result=Px4Lite_PlatformBuzzerInit();
    if(result!=PX4LITE_OK) {
      *failed=1U;s_buzzer_retry_wait=1U;s_buzzer_retry_ms=now_ms;return result;
    }
    s_buzzer_ready=1U;s_buzzer_retry_wait=0U;initialized=1U;
  }
  if(s_player.frequency_hz==frequency_hz) { return initialized?PX4LITE_OK:PX4LITE_IDLE; }
  result=Px4Lite_PlatformBuzzerSetFrequency(frequency_hz);
  if(result!=PX4LITE_OK) {
    (void)Px4Lite_PlatformBuzzerSetFrequency(0U);s_player.frequency_hz=0U;
    *failed=1U;s_buzzer_ready=0U;s_buzzer_retry_wait=1U;s_buzzer_retry_ms=now_ms;return result;
  }
  s_player.frequency_hz=frequency_hz;
  return PX4LITE_OK;
}
#endif

#if PX4LITE_ENABLE_CNS_CONTROL
#if PX4LITE_CNS_LOW_TONE_HZ < 200U || PX4LITE_CNS_LOW_TONE_HZ > 5000U || PX4LITE_CNS_HIGH_TONE_HZ < 200U || PX4LITE_CNS_HIGH_TONE_HZ > 5000U
#error "CNS tone frequency must be 200..5000Hz"
#endif
#if PX4LITE_CNS_ARM_TONE_MS < 1U || PX4LITE_CNS_ARM_TONE_MS > 60000U || PX4LITE_CNS_LOCK_TONE_MS < 1U || PX4LITE_CNS_LOCK_TONE_MS > 60000U || PX4LITE_CNS_QUICK_TONE_MS < 1U || PX4LITE_CNS_QUICK_TONE_MS > 60000U || PX4LITE_CNS_QUICK_GAP_MS < 1U || PX4LITE_CNS_QUICK_GAP_MS > 60000U || PX4LITE_CNS_LAND_TONE_MS < 1U || PX4LITE_CNS_LAND_TONE_MS > 60000U || PX4LITE_CNS_LAND_GAP_MS < 1U || PX4LITE_CNS_LAND_GAP_MS > 60000U
#error "CNS tone phase duration must be 1..60000ms"
#endif

/** @brief 仅当前代次可以提交完成；短临界区不执行音符计算或硬件I/O。 */
static void CnsTone_Commit(Px4Lite_Result_t result)
{
  taskENTER_CRITICAL();
  if(s_cns_request.revision==s_cns_seen_revision && s_cns_request.token==s_cns_owner_token) {
    s_cns_request.result=result;
  }
  taskEXIT_CRITICAL();
}

/** @brief 真实输出失败撤收本请求，不因重试等待、取消或旧token增加错误计数。 */
static void CnsTone_Fail(uint32_t now_ms)
{
  uint8_t failed;
  (void)StartupBuzzer_Apply(0U,now_ms,&failed);
  s_player.cns_error_count++;
  s_cns_active=0U;s_cns_phase_started=0U;
  CnsTone_Commit(PX4LITE_IO_ERROR);
}

/** @brief 计算一段有限/循环声光相位；数组边界由动作和phase计数共同约束。 */
static RpiFeedbackPhase_t CnsTone_Phase(uint8_t *count)
{
  RpiFeedbackPhase_t phase;
  phase.frequency_hz=PX4LITE_CNS_HIGH_TONE_HZ;
  phase.duration_ms=PX4LITE_CNS_QUICK_TONE_MS;
  *count=1U;
  switch(s_cns_action) {
    case PX4LITE_CNS_TONE_ARM:
      phase.frequency_hz=PX4LITE_CNS_LOW_TONE_HZ;phase.duration_ms=PX4LITE_CNS_ARM_TONE_MS;break;
    case PX4LITE_CNS_TONE_DISARM:
      phase.duration_ms=PX4LITE_CNS_LOCK_TONE_MS;break;
    case PX4LITE_CNS_TONE_LAND:
      *count=2U;phase.frequency_hz=(s_cns_phase==0U)?PX4LITE_CNS_HIGH_TONE_HZ:0U;
      phase.duration_ms=(s_cns_phase==0U)?PX4LITE_CNS_LAND_TONE_MS:PX4LITE_CNS_LAND_GAP_MS;break;
    case PX4LITE_CNS_TONE_TAKEOFF:
    case PX4LITE_CNS_TONE_ESTOP:
      *count=(s_cns_action==PX4LITE_CNS_TONE_TAKEOFF)?6U:4U;
      if(s_cns_phase&1U) { phase.frequency_hz=0U;phase.duration_ms=PX4LITE_CNS_QUICK_GAP_MS; }
      break;
    case PX4LITE_CNS_TONE_SET_MOTOR_PWM:
      *count=(uint8_t)(sizeof(s_feedback_success)/sizeof(s_feedback_success[0]));
      if(s_cns_phase<*count) { phase=s_feedback_success[s_cns_phase]; }
      break;
    default:break;
  }
  return phase;
}

#if STARTUP_RGB_SERVICE_ENABLED
/** @brief CNS成功段或急停黑帧复用原RGB恢复/超时服务，BUSY不等于硬件失败。 */
static Px4Lite_Result_t CnsTone_Rgb(uint8_t lit,uint32_t now_ms)
{
  uint32_t errors=s_player.rgb_error_count;
  uint8_t red=0U,green=0U,blue=0U;
  s_cns_rgb_override=1U;s_cns_rgb_lit=lit;
  if(lit) {
    const uint8_t *color=s_feedback_success_colors[s_cns_phase/2U];
    red=color[0];green=color[1];blue=color[2];
  }
  StartupRgb_Run(now_ms);
  if(s_player.rgb_error_count!=errors) { return PX4LITE_IO_ERROR; }
  if(!s_player.rgb_ready || !s_rgb_color_valid || s_player.rgb_red!=red ||
      s_player.rgb_green!=green || s_player.rgb_blue!=blue) { return PX4LITE_BUSY; }
  return PX4LITE_OK;
}

/** @brief 暂停的generic灯效不继续换色，但已有DMA仍被服务并保留真实故障。 */
static void CnsTone_RgbFrozen(uint32_t now_ms)
{
  Px4Lite_Result_t result;
  if(!s_player.rgb_ready) { return; }
  result=Px4Lite_PlatformRgbService(now_ms);
  s_player.rgb_result=result;s_player.rgb_busy=(uint8_t)(result==PX4LITE_BUSY);
  if(result!=PX4LITE_OK && result!=PX4LITE_BUSY) { StartupRgb_Fail(now_ms,result); }
}
#endif

/** @brief sensor消费取消全部请求，永久终止旧启动旋律，并清除当时已有事件。 */
static void CnsTone_ClearGeneric(void)
{
#if PX4LITE_ENABLE_STARTUP_TONE
  if(s_player.state==PX4LITE_TONE_WAITING || s_player.state==PX4LITE_TONE_PLAYING) {
    s_player.state=PX4LITE_TONE_DONE;s_player.note_index=255U;
  }
#endif
#if STARTUP_FEEDBACK_ENABLED
  taskENTER_CRITICAL();
  s_feedback_pending=0U;s_feedback_read=0U;s_feedback_write=0U;
  taskEXIT_CRITICAL();
  s_feedback_active=0U;
#if STARTUP_RGB_SERVICE_ENABLED
  s_feedback_rgb_override=0U;s_feedback_lit=0U;
#endif
#endif
#if STARTUP_RGB_SERVICE_ENABLED
  s_cns_clear_rgb=1U;
#endif
}

/**
 * @brief CNS单owner逐相位播放器；本轮拥有输出时返回1，阻止旧音源覆盖。
 * @details 暂停时间只平移旧源基准，不跳相位；每轮最多推进一相位，延迟只延长声音。
 */
static uint8_t CnsTone_Run(uint32_t now_us,uint32_t now_ms)
{
  uint32_t revision,token,elapsed=now_us-s_cns_last_us;
  uint8_t action,clear_all,failed,count;
  Px4Lite_Result_t result;
  RpiFeedbackPhase_t phase;
  (void)elapsed;
  if(s_cns_owned) {
#if PX4LITE_ENABLE_STARTUP_TONE
    if(s_player.state==PX4LITE_TONE_WAITING || s_player.state==PX4LITE_TONE_PLAYING) { s_player.start_us+=elapsed; }
#endif
#if STARTUP_FEEDBACK_ENABLED
    if(s_feedback_active) { s_feedback_start_us+=elapsed; }
#endif
  }
  s_cns_last_us=now_us;s_cns_owned=0U;
  taskENTER_CRITICAL();
  revision=s_cns_request.revision;token=s_cns_request.token;action=s_cns_request.action;
  clear_all=s_cns_request.clear_all;s_cns_request.clear_all=0U;
  taskEXIT_CRITICAL();
  if(revision!=s_cns_seen_revision) {
    uint8_t previous_active=s_cns_active;
    s_cns_seen_revision=revision;s_cns_owner_token=token;s_cns_action=action;
    s_cns_active=(uint8_t)(token!=0U);s_cns_phase=0U;s_cns_phase_started=0U;
    if(clear_all) { CnsTone_ClearGeneric(); }
#if STARTUP_RGB_SERVICE_ENABLED && !PX4LITE_ENABLE_STARTUP_TONE
    s_cns_ever_started=1U;
#endif
    if(!token && (previous_active || clear_all)) {
      (void)StartupBuzzer_Apply(0U,now_ms,&failed);
      if(failed) { s_player.cns_error_count++; }
    }
    if(token && (action<PX4LITE_CNS_TONE_ARM || action>PX4LITE_CNS_TONE_SET_MOTOR_PWM)) {
      CnsTone_Fail(now_ms);s_cns_owned=1U;return 1U;
    }
  }
  if(!s_cns_active) {
#if STARTUP_RGB_SERVICE_ENABLED
    s_cns_rgb_override=0U;s_cns_rgb_lit=0U;
    if(s_cns_clear_rgb) {
      result=CnsTone_Rgb(0U,now_ms);
      if(result==PX4LITE_OK || result==PX4LITE_IO_ERROR) { s_cns_clear_rgb=0U; }
      s_cns_owned=1U;return 1U;
    }
#endif
    return 0U;
  }
  s_cns_owned=1U;
  phase=CnsTone_Phase(&count);
  if(s_cns_phase_started && (uint32_t)(now_us-s_cns_phase_start_us)>=(uint32_t)phase.duration_ms*1000U) {
    s_cns_phase++;s_cns_phase_started=0U;
    if(s_cns_action==PX4LITE_CNS_TONE_LAND && s_cns_phase>=count) { s_cns_phase=0U; }
  }
  if(s_cns_phase>=count) {
    result=StartupBuzzer_Apply(0U,now_ms,&failed);
    if(failed) { CnsTone_Fail(now_ms);return 1U; }
    if(result!=PX4LITE_OK && result!=PX4LITE_IDLE) { return 1U; }
#if STARTUP_RGB_SERVICE_ENABLED
    if(s_cns_action==PX4LITE_CNS_TONE_SET_MOTOR_PWM || s_cns_clear_rgb) {
      result=CnsTone_Rgb(0U,now_ms);
      if(result==PX4LITE_IO_ERROR) { CnsTone_Fail(now_ms);return 1U; }
      if(result!=PX4LITE_OK || s_player.rgb_busy) { return 1U; }
      s_cns_clear_rgb=0U;
    } else { CnsTone_RgbFrozen(now_ms); }
#endif
    s_cns_active=0U;CnsTone_Commit(PX4LITE_OK);return 1U;
  }
  phase=CnsTone_Phase(&count);
  result=StartupBuzzer_Apply(phase.frequency_hz,now_ms,&failed);
  if(failed) { CnsTone_Fail(now_ms);return 1U; }
  if(result!=PX4LITE_OK && result!=PX4LITE_IDLE) { return 1U; }
#if STARTUP_RGB_SERVICE_ENABLED
  if(s_cns_action==PX4LITE_CNS_TONE_SET_MOTOR_PWM || s_cns_clear_rgb) {
    result=CnsTone_Rgb((uint8_t)(s_cns_action==PX4LITE_CNS_TONE_SET_MOTOR_PWM && phase.frequency_hz!=0U),now_ms);
    if(result==PX4LITE_IO_ERROR) { CnsTone_Fail(now_ms);return 1U; }
    if(result!=PX4LITE_OK) { return 1U; }
    s_cns_clear_rgb=0U;
  } else {
    s_cns_rgb_override=0U;s_cns_rgb_lit=0U;CnsTone_RgbFrozen(now_ms);
  }
#endif
  if(!s_cns_phase_started) { s_cns_phase_start_us=now_us;s_cns_phase_started=1U; }
  return 1U;
}
#endif

#if STARTUP_FEEDBACK_ENABLED
/** @brief 事件输出使用共用蜂鸣器恢复，但只更新事件自己的错误事实。 */
static void Feedback_SetFrequency(uint16_t frequency_hz,uint32_t now_ms)
{
  uint8_t failed;
  Px4Lite_Result_t result=StartupBuzzer_Apply(frequency_hz,now_ms,&failed);
  if(result!=PX4LITE_IDLE) { s_feedback_buzzer_result=result; }
  if(failed) { s_feedback_buzzer_errors++; }
}

/** @brief 用事件类别选择固定旋律；数组仅在对应独立功能开启时保留。 */
static const RpiFeedbackPhase_t *Feedback_GetPhases(uint8_t kind,uint8_t *count)
{
  switch(kind) {
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
    case PX4LITE_FEEDBACK_COMMAND_SUCCEEDED:
      *count=(uint8_t)(sizeof(s_feedback_success)/sizeof(s_feedback_success[0]));return s_feedback_success;
    case PX4LITE_FEEDBACK_COMMAND_FAILED:
      *count=(uint8_t)(sizeof(s_feedback_failure)/sizeof(s_feedback_failure[0]));return s_feedback_failure;
#endif
#if PX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK
    case PX4LITE_FEEDBACK_MODULE_CONNECTED:
      *count=(uint8_t)(sizeof(s_feedback_connected)/sizeof(s_feedback_connected[0]));return s_feedback_connected;
#endif
    default:
#if PX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK
      *count=(uint8_t)(sizeof(s_feedback_disconnected)/sizeof(s_feedback_disconnected[0]));return s_feedback_disconnected;
#else
      *count=(uint8_t)(sizeof(s_feedback_failure)/sizeof(s_feedback_failure[0]));return s_feedback_failure;
#endif
  }
}

/**
 * @brief 逐相位播放结果旋律，每轮最多推进一次；迟到只延长，不跳过任何音符或间隙。
 * @details 启动旋律优先；FIFO保留四类事件顺序，关闭启动音时首事件才初始化输出。
 *          单轮O(1)，无阻塞等待或补播循环，队满不影响命令执行。
 */
static void Feedback_Run(uint32_t now_us,uint32_t now_ms)
{
  const RpiFeedbackPhase_t *phases;
  uint8_t count,pending;
  uint32_t elapsed_us;
#if STARTUP_RGB_SERVICE_ENABLED
  s_feedback_rgb_override=0U;s_feedback_lit=0U;
#endif
  if(s_player.state==PX4LITE_TONE_WAITING || s_player.state==PX4LITE_TONE_PLAYING) { return; }
  if(!s_feedback_active) {
    taskENTER_CRITICAL();pending=s_feedback_pending;
    if(pending) {
      s_feedback_kind=s_feedback_queue[s_feedback_read];
      s_feedback_read++;
      if(s_feedback_read>=PX4LITE_RPI_FEEDBACK_PENDING_MAX) { s_feedback_read=0U; }
      s_feedback_pending--;
    }
    taskEXIT_CRITICAL();
    if(!pending) { return; }
    s_feedback_succeeded=(uint8_t)(s_feedback_kind==PX4LITE_FEEDBACK_COMMAND_SUCCEEDED);
    s_feedback_active=1U;s_feedback_started++;s_feedback_start_us=now_us;s_feedback_phase=0U;
#if !PX4LITE_ENABLE_STARTUP_TONE && STARTUP_RGB_SERVICE_ENABLED
    s_feedback_ever_started=1U;
#endif
  }
  phases=Feedback_GetPhases(s_feedback_kind,&count);
  elapsed_us=now_us-s_feedback_start_us;
  if(elapsed_us>=(uint32_t)phases[s_feedback_phase].duration_ms*1000U) {
    s_feedback_phase++;s_feedback_start_us=now_us;
  }
  if(s_feedback_phase>=count) {
    s_feedback_active=0U;s_feedback_finished++;Feedback_SetFrequency(0U,now_ms);
#if STARTUP_RGB_SERVICE_ENABLED
    taskENTER_CRITICAL();pending=s_feedback_pending;taskEXIT_CRITICAL();
    s_feedback_rgb_override=(uint8_t)(pending!=0U);
    if(!pending) { s_rgb_color_valid=0U; }
#endif
    return;
  }
#if STARTUP_RGB_SERVICE_ENABLED
  s_feedback_rgb_override=1U;
  s_feedback_lit=(uint8_t)(phases[s_feedback_phase].frequency_hz!=0U);
#endif
  Feedback_SetFrequency(phases[s_feedback_phase].frequency_hz,now_ms);
}
#endif

#if PX4LITE_ENABLE_MOTOR_MUSIC
#if (PX4LITE_MOTOR_MUSIC_FRESH_MS < PX4LITE_CONTROL_PERIOD_MS) || (PX4LITE_MOTOR_MUSIC_FRESH_MS > 10000U)
#error "Motor music freshness must be control-period..10000ms"
#endif
/** @brief 初始化用户提供的静态曲谱，只校验一次；不访问电机或蜂鸣器。 */
static void MotorMusic_Reset(uint32_t now_us)
{
  uint32_t i;
  s_music_last_us=now_us;s_music_owned=0U;
  s_player.music_result=PX4LITE_INVALID_PARAM;
  if(MOTOR_MUSIC_SCORE_COUNT==0U || MOTOR_MUSIC_SCORE_COUNT>65535U) { return; }
  for(i=0U;i<MOTOR_MUSIC_SCORE_COUNT;++i) {
    uint16_t hz=s_motor_music_score[i].frequency_hz;
    if(!s_motor_music_score[i].duration_ms || (hz && (hz<200U || hz>5000U))) { return; }
  }
  s_player.music_ready=1U;
  s_player.music_remaining_us=(uint32_t)s_motor_music_score[0].duration_ms*1000U;
  s_player.music_result=PX4LITE_OK;
}

/**
 * @brief 只读取同一调度事务内的电机快照/人工锁/控制状态，返回后才计算和写蜂鸣器。
 * @details 不读取私有PWM寄存器或目标变量；1001us即使油门取整为0仍是有效运行输出。
 *          旧topic可能在输出失败时保留，所以必须同时要求当前Control ONLINE且未上锁。
 */
static uint8_t MotorMusic_Gate(void)
{
  Px4Lite_MotorOutputs_t motor;
  Px4Lite_ModuleStatus_t status;
  Px4Lite_Result_t motor_rc,status_rc,lock_rc;
  uint32_t now_ms;
  uint8_t locked=1U,i;
  vTaskSuspendAll();
  motor_rc=Px4Lite_CopyMotor(&motor);
  lock_rc=Px4Lite_ControlGetMotorLocked(&locked);
  status_rc=Px4Lite_GetModuleStatus(PX4LITE_MODULE_CONTROL,&status);
  now_ms=Px4Lite_PlatformGetMs();
  (void)xTaskResumeAll();
  if(motor_rc!=PX4LITE_OK || lock_rc!=PX4LITE_OK || status_rc!=PX4LITE_OK || locked ||
      status.state!=PX4LITE_STATE_ONLINE || !motor.header.valid ||
      !(motor.header.flags&PX4LITE_DATA_VALID) || motor.run_state!=1U ||
      (uint32_t)(now_ms-motor.header.sample_time_ms)>PX4LITE_MOTOR_MUSIC_FRESH_MS ||
      (uint32_t)(now_ms-motor.header.publish_time_ms)>PX4LITE_MOTOR_MUSIC_FRESH_MS) { return 0U; }
  for(i=0U;i<PX4LITE_MOTOR_COUNT;++i) {
    if(motor.pulse_us[i]>PX4LITE_CONTROL_ESC_MIN_PULSE_US &&
       motor.pulse_us[i]<=PX4LITE_CONTROL_ESC_MAX_PULSE_US) { return 1U; }
  }
  return 0U;
}

/**
 * @brief 仅扣除上一服务间隔真正属于背景的时间；暂停和被抢占时每轮仍更新微秒基准。
 * @return 上一间隔由背景占用为1，供本轮停机时只撤收自己的声音。
 * @note 每轮最多推进一相位，迟到不追赶或跳音；长暂停超过_us整圈也不快进曲谱。
 */
static uint8_t MotorMusic_Account(uint32_t now_us)
{
  uint32_t elapsed=now_us-s_music_last_us;
  uint8_t previous=s_music_owned;
  s_music_last_us=now_us;s_music_owned=0U;s_player.music_running=0U;
  if(!previous || !s_player.music_ready) { return previous; }
  if(elapsed>=s_player.music_remaining_us) {
    s_player.music_phase_index++;
    if(s_player.music_phase_index>=MOTOR_MUSIC_SCORE_COUNT) { s_player.music_phase_index=0U; }
    s_player.music_remaining_us=(uint32_t)s_motor_music_score[s_player.music_phase_index].duration_ms*1000U;
  } else { s_player.music_remaining_us-=elapsed; }
  return previous;
}

/** @brief 启动音、当前事件及已排队事件均优先于背景，事件间不插入短音乐。 */
static uint8_t MotorMusic_ForegroundBusy(void)
{
#if PX4LITE_ENABLE_STARTUP_TONE
  if(s_player.state==PX4LITE_TONE_WAITING || s_player.state==PX4LITE_TONE_PLAYING) { return 1U; }
#endif
#if STARTUP_FEEDBACK_ENABLED
  {
    uint8_t pending;
    taskENTER_CRITICAL();pending=s_feedback_pending;taskEXIT_CRITICAL();
    if(s_feedback_active || pending) { return 1U; }
  }
#endif
  return 0U;
}

/** @brief 背景只修改自己的输出结果，不冒充命令提示的错误或完成计数。 */
static uint8_t MotorMusic_SetFrequency(uint16_t frequency_hz,uint32_t now_ms)
{
  uint8_t failed;
  Px4Lite_Result_t result=StartupBuzzer_Apply(frequency_hz,now_ms,&failed);
  if(result!=PX4LITE_IDLE) { s_player.music_result=result; }
  if(failed) { s_player.music_error_count++; }
  return (uint8_t)(result==PX4LITE_OK || result==PX4LITE_IDLE);
}

/** @brief sensor选择背景所有权，停机暂停不复位；高优先提示持有PWM时绝不静音覆盖。 */
static void MotorMusic_Run(uint8_t previous)
{
  s_player.music_gate=MotorMusic_Gate();
  if(MotorMusic_ForegroundBusy()) { return; }
  if(!s_player.music_ready || !s_player.music_gate) {
    if(previous) { (void)MotorMusic_SetFrequency(0U,Px4Lite_PlatformGetMs()); }
    return;
  }
  if(MotorMusic_SetFrequency(s_motor_music_score[s_player.music_phase_index].frequency_hz,
                            Px4Lite_PlatformGetMs())) {
    s_music_owned=1U;s_player.music_running=1U;
  }
}
#endif

/** @brief 先结算背景已播放时间，再按CNS/启动/事件/背景优先级单owner输出。 */
void Px4Lite_StartupToneRun(uint32_t now_us)
{
#if PX4LITE_ENABLE_STARTUP_TONE || STARTUP_BUZZER_SERVICE_ENABLED
#if PX4LITE_ENABLE_MOTOR_MUSIC
  uint8_t previous_music=MotorMusic_Account(now_us);
#endif
#if PX4LITE_ENABLE_CNS_CONTROL
  if(CnsTone_Run(now_us,Px4Lite_PlatformGetMs())) { StartupTone_Publish();return; }
#endif
#if PX4LITE_ENABLE_STARTUP_TONE
  StartupTone_Update(now_us);
#endif
#if STARTUP_FEEDBACK_ENABLED
  Feedback_Run(now_us,Px4Lite_PlatformGetMs());
#endif
#if PX4LITE_ENABLE_MOTOR_MUSIC
  MotorMusic_Run(previous_music);
#endif
#if STARTUP_RGB_SERVICE_ENABLED
#if !PX4LITE_ENABLE_STARTUP_TONE
#if STARTUP_FEEDBACK_ENABLED
  if(s_feedback_ever_started
#if PX4LITE_ENABLE_CNS_CONTROL
     || s_cns_ever_started
#endif
     )
#elif PX4LITE_ENABLE_CNS_CONTROL
  if(s_cns_ever_started)
#endif
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
