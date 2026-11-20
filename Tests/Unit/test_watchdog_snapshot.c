/**
 * @file test_watchdog_snapshot.c
 * @brief 通过真实 F407 适配层检验任务抢占时的看门狗时间基准。
 * @details 用可控毫秒时钟和临界区边界注入调度，用计数替代 IWDG 寄存器访问。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "px4lite_platform.h"

typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } TIM_HandleTypeDef;
typedef struct { int unused; } DMA_HandleTypeDef;
typedef struct { uint32_t CNT, SR; } HostTimer;
#define UART_WORDLENGTH_8B 0U
#define UART_WORDLENGTH_9B 0x1000U
#define UART_PARITY_NONE 0U
#define UART_PARITY_EVEN 0x0400U
#define UART_PARITY_ODD 0x0600U
#define UART_STOPBITS_1 0U
#define UART_STOPBITS_2 0x2000U
#define UART_HWCONTROL_NONE 0U
#define UART_HWCONTROL_RTS 0x0100U
#define UART_HWCONTROL_CTS 0x0200U
#define UART_HWCONTROL_RTS_CTS 0x0300U
static HostTimer s_host_timer;
#define TIM6 (&s_host_timer)
#define TIM_SR_UIF 1U
static uint32_t __get_PRIMASK(void) { return 0U; }
static void __disable_irq(void) {}
static void __enable_irq(void) {}
static uint32_t HAL_GetTick(void) { return 0U; }
static uint32_t HAL_GetUIDw0(void) { return 0U; }
static uint32_t HAL_GetUIDw1(void) { return 0U; }
static uint32_t HAL_GetUIDw2(void) { return 0U; }

static uint32_t s_clock_ms;
static unsigned s_critical;
static unsigned s_sensor_ready;
static unsigned s_runtime_entries;
static unsigned s_feeds;

/** @brief 模拟 Health 获得临界区前，sensor 先完成并发布较新心跳。 */
static void TestEnterCritical(void)
{
  assert(s_critical == 0U);
  if (s_sensor_ready != 0U) {
    s_sensor_ready = 0U;
    ++s_clock_ms;
    Px4Lite_PlatformHeartbeat(PX4LITE_HEARTBEAT_SENSOR, s_clock_ms);
  }
  s_critical = 1U;
}

/** @brief 校验同步调用成对退出。 */
static void TestExitCritical(void)
{
  assert(s_critical == 1U);
  s_critical = 0U;
}
#define taskENTER_CRITICAL() TestEnterCritical()
#define taskEXIT_CRITICAL() TestExitCritical()

#include "../../Framework/Src/px4lite_platform_f407.c"

uint32_t BSP_Time_GetTickMs(void) { return s_clock_ms; }
uint8_t BSP_Watchdog_IsStarted(void) { return 1U; }
void BSP_Watchdog_BootProgress(void) {}
void BSP_Watchdog_SealBootProgress(void) {}

/** @brief 记录真实平台策略的启动交接，验证硬件操作处于临界区外。 */
BSP_Status_t BSP_Watchdog_EnterRuntime(void)
{
  assert(s_critical == 0U);
  ++s_runtime_entries;
  return BSP_STATUS_OK;
}

/** @brief 记录真实平台策略允许的续期。 */
void BSP_WatchdogRefresh(void)
{
  assert(s_critical == 0U);
  ++s_feeds;
}

/** @brief 覆盖抢占时的首次交接、正常续期、同毫秒和心跳超时。 */
int main(void)
{
  assert(Px4Lite_PlatformHeartbeatConfigure(PX4LITE_HEARTBEAT_SENSOR, 10U, 0U) == PX4LITE_OK);
  assert(Px4Lite_PlatformHeartbeatConfigure(PX4LITE_HEARTBEAT_HEALTH, 100U, 0U) == PX4LITE_OK);
  s_clock_ms = 1000U;
  assert(Px4Lite_PlatformWatchdogBeginStartup(s_clock_ms) == PX4LITE_OK);

  s_clock_ms = 1100U;
  Px4Lite_PlatformHeartbeat(PX4LITE_HEARTBEAT_HEALTH, s_clock_ms);
  s_sensor_ready = 1U;
  Px4Lite_PlatformWatchdogFeed();
  assert(s_sensor_ready == 0U && s_runtime_entries == 1U && s_feeds == 0U);

  Px4Lite_PlatformWatchdogFeed();
  assert(s_runtime_entries == 1U && s_feeds == 0U);

  s_clock_ms += 100U;
  Px4Lite_PlatformHeartbeat(PX4LITE_HEARTBEAT_HEALTH, s_clock_ms);
  s_sensor_ready = 1U;
  Px4Lite_PlatformWatchdogFeed();
  assert(s_sensor_ready == 0U && s_runtime_entries == 1U && s_feeds == 1U);

  s_clock_ms += Px4Lite_HeartbeatTimeoutMs(10U, 0U) + 1U;
  Px4Lite_PlatformHeartbeat(PX4LITE_HEARTBEAT_HEALTH, s_clock_ms);
  Px4Lite_PlatformWatchdogFeed();
  assert(s_runtime_entries == 1U && s_feeds == 1U);

  puts("Watchdog snapshot tests passed");
  return 0;
}
