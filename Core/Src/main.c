/**
 * @file main.c
 * @brief 系统组合根，初始化硬件、Framework、Business 任务并启动调度器。
 *
 * @details
 * 本文件只负责启动顺序编排，不承载业务逻辑。外设初始化由 BSP 和 Platform
 * Adapter 负责，Framework/Business 任务创建分别由各自模块完成。
 */

#include "main.h"
#include "core_fatal.h"
#include "bsp_pwm.h"
#include "bsp_watchdog.h"
#include "px4lite_config.h"
#include "bsp.h"
#include "business_task_registry.h"
#include "debug_console.h"
#include "px4lite_app.h"
#include "px4lite_platform.h"
#include "FreeRTOS.h"
#include "task.h"

static void SystemClock_Config(void);

/**
 * @brief 初始化硬件与框架服务，然后启动 FreeRTOS 调度器。
 *
 * @return 正常情况下不会返回；若调度器退出则进入 `Error_Handler()`。
 */
int main(void)
{
  BSP_Watchdog_CaptureResetCause();
#if PX4LITE_ENABLE_HARDWARE_WATCHDOG
  if (BSP_Watchdog_StartBoot() != BSP_STATUS_OK) { Error_Handler(); }
#endif
  if (HAL_Init() != HAL_OK) { Error_Handler(); }
  BSP_Watchdog_BootProgress();
  SystemClock_Config();
  BSP_Watchdog_BootProgress();

  DebugConsole_Init();
  BSP_Watchdog_BootProgress();
  if (BSP_Init() != BSP_STATUS_OK) {
    DBG_BOOT_PRINT("boot failed: BSP_Init");
    Error_Handler();
  }
  BSP_Watchdog_BootProgress();

  if (Px4Lite_PlatformInit() != PX4LITE_OK) {
    DBG_BOOT_PRINT("boot failed: Px4Lite_PlatformInit");
    Error_Handler();
  }
  BSP_Watchdog_BootProgress();
  if (Px4Lite_AppInit() != pdPASS) {
    DBG_BOOT_PRINT("boot failed: Px4Lite_AppInit");
    Error_Handler();
  }
  BSP_Watchdog_BootProgress();
  if (Business_AppInit() != pdPASS) {
    DBG_BOOT_PRINT("boot failed: Business_AppInit");
    Error_Handler();
  }
  BSP_Watchdog_BootProgress();

  DBG_BOOT_PRINT("Formal framework started: sensors + business + display");
  if (Px4Lite_PlatformWatchdogBeginStartup(Px4Lite_PlatformGetMs()) != PX4LITE_OK) { Error_Handler(); }
  vTaskStartScheduler();
  Error_Handler();
}

/**
 * @brief 配置 STM32F407 系统时钟为 168 MHz。
 *
 * @note 本函数只在启动阶段调用，失败时进入 `Error_Handler()`。
 */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clock;
  RCC_OscInitTypeDef oscillator;

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  oscillator.HSEState       = RCC_HSE_ON;
  oscillator.PLL.PLLState   = RCC_PLL_ON;
  oscillator.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  oscillator.PLL.PLLM       = 8U;
  oscillator.PLL.PLLN       = 336U;
  oscillator.PLL.PLLP       = RCC_PLLP_DIV2;
  oscillator.PLL.PLLQ       = 7U;
  if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) { Error_Handler(); }

  clock.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clock.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  clock.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  clock.APB1CLKDivider = RCC_HCLK_DIV4;
  clock.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_5) != HAL_OK) { Error_Handler(); }

  if (HAL_GetREVID() >= 0x1001U) { __HAL_FLASH_PREFETCH_BUFFER_ENABLE(); }
}

/**
 * @brief 进入不可恢复错误的停机路径。
 *
 * @note 先撤收电机信号，再清理 DMA 和停机；不依赖调度器或 HAL 时基。
 */
void Error_Handler(void)
{
  Core_FatalHalt();
}

/** @brief 致命异常最先执行的输出撤收；后续现场记录失败也不会保留 PWM。 */
void Core_FatalQuiesce(void)
{
  __disable_irq();
  BSP_PWM_EmergencyStop();
}

/** @brief 不可恢复停机，不恢复中断、不喂狗。 */
void Core_FatalHalt(void)
{
  Core_FatalQuiesce();
  BSP_EmergencyStopDma();
  for (;;) {}
}

#ifdef USE_FULL_ASSERT
/**
 * @brief 将 full assert 失败路由到统一错误处理。
 *
 * @param[in] file 断言失败文件名，当前未使用。
 * @param[in] line 断言失败行号，当前未使用。
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
  Error_Handler();
}
#endif
