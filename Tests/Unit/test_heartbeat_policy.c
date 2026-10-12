/** @file test_heartbeat_policy.c @brief 心跳期限、首帧、可选集合及回绕的纯策略回归。 */
#include <assert.h>
#include <stdio.h>
#include "px4lite_heartbeat_policy.h"
int main(void)
{
  uint32_t heartbeat[3] = {1000U, 1000U, 1000U};
  uint32_t timeout[3] = {500U, 3000U, 2000U};
  Px4Lite_WatchdogPhase_t phase = PX4LITE_WATCHDOG_WAITING;
  uint32_t last_check_ms = 0U;
  unsigned i;
  assert(Px4Lite_WatchdogEvaluate(&phase, &last_check_ms, 0U, 0U, 1U) == PX4LITE_WATCHDOG_NO_ACTION);
  assert(Px4Lite_WatchdogEvaluate(&phase, &last_check_ms, 0U, 9998U, 0U) == PX4LITE_WATCHDOG_NO_ACTION);
  /* 首轮等待中停时，剩余任务随后补心跳也不允许交接。 */
  assert(Px4Lite_WatchdogEvaluate(&phase, &last_check_ms, 0U, 9998U, 1U) == PX4LITE_WATCHDOG_NO_ACTION);
  assert(Px4Lite_WatchdogEvaluate(&phase, &last_check_ms, 0U, 9999U, 1U) == PX4LITE_WATCHDOG_ENTER_RUNTIME);
  assert(Px4Lite_WatchdogEvaluate(&phase, &last_check_ms, 0U, 10000U, 1U) == PX4LITE_WATCHDOG_NO_ACTION);
  assert(phase == PX4LITE_WATCHDOG_FAILED);
  assert(Px4Lite_WatchdogEvaluate(&phase, &last_check_ms, 0U, 10001U, 1U) == PX4LITE_WATCHDOG_NO_ACTION);
  phase = PX4LITE_WATCHDOG_RUNTIME;
  assert(Px4Lite_WatchdogEvaluate(&phase, &last_check_ms, 0U, 11000U, 0U) == PX4LITE_WATCHDOG_NO_ACTION);
  assert(Px4Lite_WatchdogEvaluate(&phase, &last_check_ms, 0U, 12000U, 1U) == PX4LITE_WATCHDOG_FEED);
  /* 模拟 TIM6 停走、SysTick 仍在调度 health：相同毫秒不可再次喂狗。 */
  for (i = 0U; i < 100U; ++i) {
    assert(Px4Lite_WatchdogEvaluate(&phase, &last_check_ms, 0U, 12000U, 1U) == PX4LITE_WATCHDOG_NO_ACTION);
  }
  assert(Px4Lite_WatchdogEvaluate(&phase, &last_check_ms, 0U, 12001U, 1U) == PX4LITE_WATCHDOG_FEED);
  last_check_ms = UINT32_MAX;
  assert(Px4Lite_WatchdogEvaluate(&phase, &last_check_ms, 0U, 0U, 1U) == PX4LITE_WATCHDOG_FEED);
  phase = PX4LITE_WATCHDOG_WAITING;
  last_check_ms = UINT32_MAX - 4999U;
  assert(Px4Lite_WatchdogEvaluate(&phase, &last_check_ms, UINT32_MAX - 4999U, 4999U, 1U) == PX4LITE_WATCHDOG_ENTER_RUNTIME);
  assert(Px4Lite_WatchdogEvaluate(&phase, &last_check_ms, UINT32_MAX - 4999U, 5000U, 0U) == PX4LITE_WATCHDOG_NO_ACTION);
  assert(phase == PX4LITE_WATCHDOG_FAILED);
  assert(Px4Lite_HeartbeatTimeoutMs(10U, 0U) == 500U);
  assert(Px4Lite_HeartbeatTimeoutMs(200U, 0U) == 600U);
  assert(Px4Lite_HeartbeatTimeoutMs(1000U, 0U) == 3000U);
  assert(Px4Lite_HeartbeatTimeoutMs(10U, 2000U) == 2000U);
  assert(Px4Lite_HeartbeatTimeoutMs(0U, 0U) == 0U);
  assert(Px4Lite_HeartbeatTimeoutMs(UINT32_MAX, 0U) == 0U);
  assert(Px4Lite_HeartbeatsCheck(heartbeat, timeout, 3U, 7U, 7U, 1500U));
  assert(!Px4Lite_HeartbeatsCheck(heartbeat, timeout, 3U, 7U, 7U, 1501U));
  assert(!Px4Lite_HeartbeatsCheck(heartbeat, timeout, 3U, 3U, 7U, 1000U));
  assert(Px4Lite_HeartbeatsCheck(heartbeat, timeout, 3U, 2U, 2U, 4000U));
  assert(!Px4Lite_HeartbeatsCheck(heartbeat, timeout, 3U, 2U, 2U, 4001U));
  assert(!Px4Lite_HeartbeatsCheck(heartbeat, timeout, 3U, 7U, 0U, 1000U));
  assert(!Px4Lite_HeartbeatsCheck(heartbeat, timeout, 3U, 15U, 15U, 1000U));
  assert(!Px4Lite_HeartbeatsCheck(0, timeout, 3U, 7U, 7U, 1000U));
  heartbeat[0] = UINT32_MAX - 199U;
  assert(Px4Lite_HeartbeatsCheck(heartbeat, timeout, 3U, 1U, 1U, 300U));
  assert(!Px4Lite_HeartbeatsCheck(heartbeat, timeout, 3U, 1U, 1U, 301U));
  puts("Heartbeat policy tests passed");
  return 0;
}
