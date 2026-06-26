/**
 * @file stm32f4xx_hal.h (test stub)
 * @brief 主机侧单元测试用的最小 HAL 桩。
 *
 * @details
 * bsp_lora.h 需要 UART_HandleTypeDef 类型才能声明 BSP_LoRa_GetUartHandle()。
 * 被测 lora_e22.c 不解引用该句柄(只通过 BSP 接口收发)，故桩成占位结构体即可。
 */
#ifndef STM32F4XX_HAL_TEST_STUB_H
#define STM32F4XX_HAL_TEST_STUB_H

#include <stdint.h>

typedef struct {
  uint32_t dummy;
} UART_HandleTypeDef;

#endif /* STM32F4XX_HAL_TEST_STUB_H */
