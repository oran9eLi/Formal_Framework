/**
 * @file stm32f4xx.h (test stub)
 * @brief 主机侧单元测试用的最小 CMSIS 桩，仅提供 lora_e22.c 用到的临界区 intrinsic。
 *
 * @details
 * 通过把本目录放在 -I 路径最前，shadow 掉真实 CMSIS 头，避免在 x86 gcc 上编译
 * ARM 专用的 core_cm4.h。被测代码只用到 __get_PRIMASK/__disable_irq/__enable_irq，
 * 在单线程测试里桩成无副作用即可。
 */
#ifndef STM32F4XX_TEST_STUB_H
#define STM32F4XX_TEST_STUB_H

#include <stdint.h>

static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void     __disable_irq(void) { }
static inline void     __enable_irq(void)  { }

#endif /* STM32F4XX_TEST_STUB_H */
