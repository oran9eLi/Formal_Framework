/** @file test_pwm_fatal.c
 * @brief 用寄存器替身执行真实 BSP fatal 撤收，验证四路位掩码及重复调用。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "core_fatal.h"
typedef struct { uint32_t MODER, OTYPER, PUPDR, BSRR; } GPIO_TypeDef;
typedef struct { uint32_t CCER; } TIM_TypeDef;
typedef struct { uint32_t CFGR, AHB1ENR, APB1ENR; } RCC_TypeDef;
typedef struct { uint32_t Prescaler, CounterMode, Period, ClockDivision, AutoReloadPreload; } TIM_InitTypeDef;
typedef struct { TIM_TypeDef *Instance; TIM_InitTypeDef Init; } TIM_HandleTypeDef;
typedef struct { uint32_t OCMode, Pulse, OCPolarity, OCFastMode; } TIM_OC_InitTypeDef;
static GPIO_TypeDef gpio_a, gpio_d;
static TIM_TypeDef tim3, tim4;
static RCC_TypeDef rcc;
#define GPIOA (&gpio_a)
#define GPIOD (&gpio_d)
#define TIM3 (&tim3)
#define TIM4 (&tim4)
#define RCC (&rcc)
#define RCC_AHB1ENR_GPIOAEN 1U
#define RCC_AHB1ENR_GPIODEN 8U
#define RCC_APB1ENR_TIM3EN 2U
#define RCC_APB1ENR_TIM4EN 4U
#define TIM_CCER_CC1E 1U
#define TIM_CCER_CC2E 16U
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define GPIO_PIN_6 64U
#define GPIO_PIN_7 128U
#define GPIO_PIN_12 4096U
#define GPIO_PIN_13 8192U
#define RCC_CFGR_PPRE1 0x1c00U
#define RCC_HCLK_DIV1 0U
#define TIM_COUNTERMODE_UP 0U
#define TIM_CLOCKDIVISION_DIV1 0U
#define TIM_AUTORELOAD_PRELOAD_DISABLE 0U
#define TIM_OCMODE_PWM1 0U
#define TIM_OCPOLARITY_HIGH 0U
#define TIM_OCFAST_DISABLE 0U
#define HAL_OK 0
#define __DSB() ((void)0)
#define __HAL_TIM_SET_COMPARE(h,c,p) ((void)(h),(void)(c),(void)(p))
uint32_t HAL_RCC_GetPCLK1Freq(void);
int HAL_TIM_PWM_Init(TIM_HandleTypeDef *h);
int HAL_TIM_PWM_DeInit(TIM_HandleTypeDef *h);
int HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *h, TIM_OC_InitTypeDef *o, uint32_t c);
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t c);
int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *h, uint32_t c);
#include "../../Bsp/Src/bsp_pwm.c"
int main(void)
{
  assert(Core_FatalRamRangeValid(0x20000000UL, 32U));
  assert(Core_FatalRamRangeValid(0x1000ffe0UL, 32U));
  assert(!Core_FatalRamRangeValid(0x2001ffe4UL, 32U));
  assert(!Core_FatalRamRangeValid(0xffffffffUL, 32U));
  assert(!Core_FatalRamRangeValid(0UL, 32U));
  memset(&gpio_a, 0xff, sizeof(gpio_a));
  memset(&gpio_d, 0xff, sizeof(gpio_d));
  tim3.CCER = tim4.CCER = 0xffffU;
  BSP_PWM_EmergencyStop();
  assert((rcc.AHB1ENR & 9U) == 9U);
  assert(gpio_a.BSRR == (0xc0UL << 16));
  assert(gpio_d.BSRR == (0x3000UL << 16));
  assert(gpio_a.MODER == ((0xffffffffUL & ~(0xfUL << 12)) | (0x5UL << 12)));
  assert(gpio_d.MODER == ((0xffffffffUL & ~(0xfUL << 24)) | (0x5UL << 24)));
  assert((gpio_a.OTYPER & 0xc0U) == 0U);
  assert((gpio_d.OTYPER & 0x3000U) == 0U);
  assert((tim3.CCER & 0x11U) == 0U && (tim4.CCER & 0x11U) == 0U);
  assert((tim3.CCER & ~0x11U) == (0xffffU & ~0x11U));
  BSP_PWM_EmergencyStop();
  assert(gpio_a.BSRR == (0xc0UL << 16));
  puts("PWM fatal register tests passed");
  return 0;
}
