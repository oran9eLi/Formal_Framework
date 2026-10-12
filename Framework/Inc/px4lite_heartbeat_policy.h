/**
 * @file px4lite_heartbeat_policy.h
 * @brief Framework 心跳期限与新鲜度纯策略；不持有状态、不访问 HAL/BSP/RTOS。
 */
#ifndef PX4LITE_HEARTBEAT_POLICY_H
#define PX4LITE_HEARTBEAT_POLICY_H
#include <stdint.h>
#include "px4lite_config.h"
/** @brief watchdog 策略阶段；失败后本次启动不允许重新进入运行态。 */
typedef enum {
  PX4LITE_WATCHDOG_OFF = 0,     /**< 尚未交接给任务。 */
  PX4LITE_WATCHDOG_WAITING,     /**< 等待首轮必需心跳。 */
  PX4LITE_WATCHDOG_RUNTIME,     /**< 硬件运行窗口已成功配置。 */
  PX4LITE_WATCHDOG_FAILED       /**< 首轮超时或硬件交接失败。 */
} Px4Lite_WatchdogPhase_t;
/** @brief 单次健康检查允许的硬件动作；实际寄存器操作仍由 Platform/BSP 完成。 */
typedef enum {
  PX4LITE_WATCHDOG_NO_ACTION = 0, /**< 不喂狗。 */
  PX4LITE_WATCHDOG_ENTER_RUNTIME, /**< 尝试配置运行窗口，调用者须记录成功/失败阶段。 */
  PX4LITE_WATCHDOG_FEED           /**< 运行态健康，可刷新。 */
} Px4Lite_WatchdogAction_t;
/**
 * @brief 判定首轮期限与运行态喂狗动作，时刻均为 ms。
 * @param[in,out] phase 策略阶段；首轮到期自动锁存 FAILED。
 * @param[in,out] last_check_ms 上次检查时刻，交接时初始化为 start_ms；同毫秒不允许交接或喂狗。
 * @param[in] start_ms 调度器交接时刻。
 * @param[in] now_ms 当前时刻。
 * @param[in] healthy 本轮必需心跳是否全部健康。
 * @return 允许动作；不直接访问硬件。到期优先于迟到的首轮健康。
 */
static inline Px4Lite_WatchdogAction_t Px4Lite_WatchdogEvaluate(Px4Lite_WatchdogPhase_t *phase,
                                                               uint32_t *last_check_ms, uint32_t start_ms,
                                                               uint32_t now_ms, uint8_t healthy)
{
  /* TIM6 可能停走而 SysTick 仍调度任务；此时必须让独立 IWDG 自行到期。 */
  if (now_ms == *last_check_ms) { return PX4LITE_WATCHDOG_NO_ACTION; }
  *last_check_ms = now_ms;
  if (*phase == PX4LITE_WATCHDOG_WAITING) {
    if ((uint32_t)(now_ms - start_ms) >= PX4LITE_WATCHDOG_STARTUP_WINDOW_MS) {
      *phase = PX4LITE_WATCHDOG_FAILED;
      return PX4LITE_WATCHDOG_NO_ACTION;
    }
    return healthy ? PX4LITE_WATCHDOG_ENTER_RUNTIME : PX4LITE_WATCHDOG_NO_ACTION;
  }
  return ((*phase == PX4LITE_WATCHDOG_RUNTIME) && healthy) ? PX4LITE_WATCHDOG_FEED : PX4LITE_WATCHDOG_NO_ACTION;
}
/**
 * @brief 计算任务期限 max(最小值, 3 倍周期, 调用者下限)。
 * @param[in] period_ms 正常周期，单位 ms，不得为 0。
 * @param[in] min_timeout_ms 下限，单位 ms。
 * @return 毫秒期限；非法/超过半个回绕周期返回 0。
 */
static inline uint32_t Px4Lite_HeartbeatTimeoutMs(uint32_t period_ms, uint32_t min_timeout_ms)
{
  uint32_t timeout;
  if ((period_ms == 0U) || (period_ms > 0x7fffffffUL / 3U) || (min_timeout_ms > 0x7fffffffUL)) { return 0U; }
  timeout = period_ms * 3U;
  if (timeout < PX4LITE_TASK_HEARTBEAT_MIN_MS) { timeout = PX4LITE_TASK_HEARTBEAT_MIN_MS; }
  if (timeout < min_timeout_ms) { timeout = min_timeout_ms; }
  return timeout;
}
/**
 * @brief 在临界区外检查心跳快照；缺首帧、空/越界集合、过期均判不健康。
 * @param[in] heartbeat_ms 最近工作完成毫秒数组。
 * @param[in] timeout_ms 每槽位期限毫秒数组，必需项不得为 0。
 * @param[in] count 数组长度，1—32。
 * @param[in] seen_mask 已有心跳的槽位位图。
 * @param[in] required_mask 实际创建的必需任务位图。
 * @param[in] now_ms 当前真实毫秒，差值支持无符号回绕。
 * @return 1 全部必需项新鲜，0 不健康或参数非法。
 */
static inline uint8_t Px4Lite_HeartbeatsCheck(const uint32_t *heartbeat_ms, const uint32_t *timeout_ms,
                                            uint32_t count, uint32_t seen_mask, uint32_t required_mask, uint32_t now_ms)
{
  uint32_t i;
  uint32_t allowed;
  if ((heartbeat_ms == 0) || (timeout_ms == 0) || (count == 0U) || (count > 32U) || (required_mask == 0U)) { return 0U; }
  allowed = (count == 32U) ? UINT32_MAX : ((1UL << count) - 1U);
  if (((required_mask & ~allowed) != 0U) || ((seen_mask & required_mask) != required_mask)) { return 0U; }
  for (i = 0U; i < count; ++i) {
    if ((required_mask & (1UL << i)) == 0U) { continue; }
    if ((timeout_ms[i] == 0U) || ((uint32_t)(now_ms - heartbeat_ms[i]) > timeout_ms[i])) { return 0U; }
  }
  return 1U;
}
#endif
