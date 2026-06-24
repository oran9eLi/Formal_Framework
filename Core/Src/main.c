/**
 * @file main.c
 * @brief 系统组合根，初始化硬件、Framework、Business 任务并启动调度器。
 *
 * @details
 * 本文件只负责启动顺序编排，不承载业务逻辑。外设初始化由 BSP 和 Platform
 * Adapter 负责，Framework/Business 任务创建分别由各自模块完成。
 */

#include "main.h"
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
  HAL_Init();
  SystemClock_Config();

  DebugConsole_Init();
  BSP_Init();

  if ((Px4Lite_PlatformInit() != PX4LITE_OK) || (Px4Lite_AppInit() != pdPASS) || (Business_AppInit() != pdPASS)) { Error_Handler(); }

  DBG_BOOT_PRINT("Formal framework started: sensors + business + display");
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
 * @note 当前策略为关闭中断后停在死循环；后续可在此扩展故障持久化。
 */
void Error_Handler(void)
{
  __disable_irq();
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
