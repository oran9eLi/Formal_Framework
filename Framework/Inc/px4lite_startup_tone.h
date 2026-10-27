/**
 * @file px4lite_startup_tone.h
 * @brief Framework层一次性经典启动音试听服务。
 * @details sensor是唯一写者；通过Platform输出，不包含BSP/HAL。
 *          只供当前试听候选，正式操作反馈策略不属于本服务。
 */
#ifndef PX4LITE_STARTUP_TONE_H
#define PX4LITE_STARTUP_TONE_H
#include "px4lite_types.h"

/** @brief 试听状态，只描述软件播放过程，不表示蜂鸣器在位或实际发声。 */
typedef enum {
  PX4LITE_TONE_DISABLED = 0, /**< 构建未启用。 */
  PX4LITE_TONE_WAITING, /**< 硬件就绪，等待启动延时。 */
  PX4LITE_TONE_PLAYING, /**< 正在旋律时间窗内，含音间间隙。 */
  PX4LITE_TONE_DONE, /**< 时间窗结束，已请求静音，不再重播。 */
  PX4LITE_TONE_ERROR /**< 初始化/输出失败，已撤收且不自动重播。 */
} Px4Lite_StartupToneState_t;

/** @brief 一次性播放诊断快照；时间为平台微秒，不是RTOS tick。 */
typedef struct {
  uint32_t start_us; /**< 初始化请求时刻，单位us，可回绕。 */
  uint32_t error_count; /**< 硬件初始化或输出失败累计次数。 */
  Px4Lite_Result_t result; /**< 最近硬件结果。 */
  uint16_t frequency_hz; /**< 最近软件请求的频率，0为静音，单位Hz。 */
  uint8_t state; /**< Px4Lite_StartupToneState_t。 */
  uint8_t note_index; /**< 0～16为当前音符，255为无音符。 */
} Px4Lite_StartupToneStatus_t;

/** @brief sensor启动时调用一次；now_us为平台微秒，不等待旋律。 */
void Px4Lite_StartupToneInit(uint32_t now_us);
/** @brief sensor每10ms调用；延迟执行跳过过期音符，固定时间窗结束后静音。 */
void Px4Lite_StartupToneRun(uint32_t now_us);
/** @brief 复制软件诊断快照，out不能为空；调用方不得据此认定电气验收通过。 */
Px4Lite_Result_t Px4Lite_StartupToneCopyStatus(Px4Lite_StartupToneStatus_t *out);
#endif
