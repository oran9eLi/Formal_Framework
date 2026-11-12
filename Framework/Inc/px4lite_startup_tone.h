/**
 * @file px4lite_startup_tone.h
 * @brief Framework层启动旋律、常亮渐变及树莓派收包声光提示的单owner服务。
 * @details sensor是唯一写者；通过Platform输出，不包含BSP/HAL。
 *          Comm只登记有界提示请求；所有硬件输出仍由sensor推进，不表示命令执行成功。
 */
#ifndef PX4LITE_STARTUP_TONE_H
#define PX4LITE_STARTUP_TONE_H
#include "px4lite_types.h"

/** @brief 试听状态，只描述软件播放过程，不表示蜂鸣器在位或实际发声。 */
typedef enum {
  PX4LITE_TONE_DISABLED = 0, /**< 构建未启用。 */
  PX4LITE_TONE_WAITING, /**< 硬件就绪，等待启动延时。 */
  PX4LITE_TONE_PLAYING, /**< 正在旋律时间窗内，含音间间隙。 */
  PX4LITE_TONE_DONE, /**< 启动音已结束且不重播；后续独立收包提示可临时占用声光。 */
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
  uint32_t rgb_error_count; /**< RGB初始化、DMA或超时失败累计次数，不计资源忙。 */
  uint32_t rgb_recovery_count; /**< 首次失败后尝试重新初始化的次数。 */
  Px4Lite_Result_t rgb_result; /**< 最近RGB服务结果；不改变蜂鸣器result。 */
  uint8_t rgb_ready; /**< 1表示原始输出初始化成功，不代表灯珠在位。 */
  uint8_t rgb_busy; /**< 1表示最近提交的帧尚待DMA完成确认。 */
  uint8_t rgb_red; /**< 最近成功入发送流程的红通道，0～255。 */
  uint8_t rgb_green; /**< 最近成功入发送流程的绿通道，0～255。 */
  uint8_t rgb_blue; /**< 最近成功入发送流程的蓝通道，0～255。 */
  uint32_t feedback_received; /**< 上电累计提示请求，允许自然回绕；含队满丢弃。 */
  uint32_t feedback_started; /**< 已开始的软件双鸣时间轴累计次数，不保证硬件在位。 */
  uint32_t feedback_finished; /**< 四槽时间轴结束累计次数，含硬件失败，不代表实际双鸣完成。 */
  uint32_t feedback_dropped; /**< 待播队列满时丢弃的提示数，不影响指令收发/执行。 */
  uint32_t feedback_buzzer_errors; /**< 提示期间蜂鸣器初始化/输出失败次数。 */
  Px4Lite_Result_t feedback_buzzer_result; /**< 最近提示蜂鸣器结果，独立于启动音result。 */
  uint8_t feedback_pending; /**< 最近发布时待播数，0～配置上限（默认8），不含当前提示。 */
  uint8_t feedback_active; /**< 正在软件双鸣时间窗为1，不代表已实际发声。 */
} Px4Lite_StartupToneStatus_t;

/** @brief sensor启动时调用一次；now_us为平台微秒，不等待旋律。 */
void Px4Lite_StartupToneInit(uint32_t now_us);
/** @brief sensor每10ms推进；启动音后播放排队双鸣/白灯双闪，再回当前渐变。 */
void Px4Lite_StartupToneRun(uint32_t now_us);
/**
 * @brief Comm任务登记一次本机控制/合法配置收包提示，只做短临界区计数，不访问硬件。
 * @details 启动旋律期间延后；待播上限8，满时只累计dropped；禁用开关时无操作。
 *          本接口不校验协议、不表示执行成功，不在ISR调用，Init不会清掉已登记请求。
 */
void Px4Lite_StartupToneNotifyRpiCommand(void);
/** @brief 复制软件诊断快照，out不能为空；调用方不得据此认定电气验收通过。 */
Px4Lite_Result_t Px4Lite_StartupToneCopyStatus(Px4Lite_StartupToneStatus_t *out);
#endif
