/**
 * @file main.c
 * @brief Initialize hardware, framework services, tasks, and start the scheduler.
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
 * @brief Initialize hardware and framework services, then start the FreeRTOS scheduler.
 */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    DebugConsole_Init();
    BSP_Init();

    if ((Px4Lite_PlatformInit() != PX4LITE_OK) ||
        (Px4Lite_AppInit() != pdPASS) ||
        (Business_AppInit() != pdPASS))
    {
        Error_Handler();
    }

    DBG_BOOT_PRINT("Formal framework started: sensors + business + display");
    vTaskStartScheduler();
    Error_Handler();
}

/**
 * @brief Configure the STM32F407 system clock for 168 MHz operation.
 */
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef clock;
    RCC_OscInitTypeDef oscillator;

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscillator.HSEState = RCC_HSE_ON;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscillator.PLL.PLLM = 8U;
    oscillator.PLL.PLLN = 336U;
    oscillator.PLL.PLLP = RCC_PLLP_DIV2;
    oscillator.PLL.PLLQ = 7U;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK)
    {
        Error_Handler();
    }

    clock.ClockType = RCC_CLOCKTYPE_SYSCLK |
                      RCC_CLOCKTYPE_HCLK |
                      RCC_CLOCKTYPE_PCLK1 |
                      RCC_CLOCKTYPE_PCLK2;
    clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clock.APB1CLKDivider = RCC_HCLK_DIV4;
    clock.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_GetREVID() >= 0x1001U)
    {
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    }
}

/**
 * @brief Enter the current fatal-stop path after an unrecoverable error.
 */
void Error_Handler(void)
{
    __disable_irq();
    for (;;)
    {
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief Route full-assert failures to the fatal error handler.
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    Error_Handler();
}
#endif
