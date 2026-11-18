/**
 * @file bsp_buzzer.c
 * @brief BSP层独占PE5/AF3/TIM9_CH1的原始PWM输出。
 * @details 初始化及释放经HAL MSP完成；不使用DMA/IRQ，不复用电机或TIM6时基。
 *          本文件不解释旋律、启动状态或学生操作；可选输出失败不阻塞系统启动。
 */
#include "bsp_buzzer.h"
#include "bsp_config.h"

#if BSP_ENABLE_BUZZER
static TIM_HandleTypeDef s_buzzer_tim;
static uint32_t s_counter_hz; /**< 实际TIM9计数频率，单位Hz。 */
static uint8_t s_initialized; /**< HAL通道成功启动后为1。 */
#endif

/** @brief 配置1MHz附近计数时钟及零占空比；APB2分频时TIM输入时钟倍频。 */
BSP_Status_t MX_TIM9_Init(void)
{
#if BSP_ENABLE_BUZZER
  TIM_OC_InitTypeDef channel = {0};
  uint32_t clock_hz = HAL_RCC_GetPCLK2Freq();
  uint32_t divider;
  if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U) { clock_hz *= 2U; }
  divider = clock_hz / 1000000U;
  if ((divider == 0U) || (divider > 65536U)) { return BSP_STATUS_ERROR; }
  s_counter_hz = clock_hz / divider;
  s_buzzer_tim.Instance = BSP_BUZZER_TIM;
  s_buzzer_tim.Init.Prescaler = divider - 1U;
  s_buzzer_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
  s_buzzer_tim.Init.Period = 999U;
  s_buzzer_tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  s_buzzer_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&s_buzzer_tim) != HAL_OK) { return BSP_STATUS_ERROR; }
  channel.OCMode = TIM_OCMODE_PWM1;
  channel.Pulse = 0U;
  channel.OCPolarity = TIM_OCPOLARITY_HIGH;
  channel.OCFastMode = TIM_OCFAST_DISABLE;
  return (HAL_TIM_PWM_ConfigChannel(&s_buzzer_tim, &channel, BSP_BUZZER_CHANNEL) == HAL_OK)
      ? BSP_STATUS_OK : BSP_STATUS_ERROR;
#else
  return BSP_STATUS_ERROR;
#endif
}

/** @brief 建立完整硬件输出链；配置失败立即撤收PE5，保留可重复初始化入口。 */
BSP_Status_t BSP_Buzzer_Init(void)
{
#if BSP_ENABLE_BUZZER
  if (s_initialized != 0U) { return BSP_Buzzer_SetFrequency(0U); }
  if (MX_TIM9_Init() != BSP_STATUS_OK) {
    BSP_Buzzer_EmergencyStop();
    if (s_buzzer_tim.Instance != 0) { (void)HAL_TIM_PWM_DeInit(&s_buzzer_tim); }
    return BSP_STATUS_ERROR;
  }
  s_buzzer_tim.Instance->EGR = TIM_EGR_UG;
  __HAL_TIM_CLEAR_FLAG(&s_buzzer_tim, TIM_FLAG_UPDATE);
  if (HAL_TIM_PWM_Start(&s_buzzer_tim, BSP_BUZZER_CHANNEL) != HAL_OK) {
    BSP_Buzzer_EmergencyStop();
    (void)HAL_TIM_PWM_DeInit(&s_buzzer_tim);
    return BSP_STATUS_ERROR;
  }
  s_initialized = 1U;
  return BSP_STATUS_OK;
#else
  return BSP_STATUS_ERROR;
#endif
}

/** @brief 用零比较值保证正常静音，变频期间停止计数以避免旧ARR/CCR混用。 */
BSP_Status_t BSP_Buzzer_SetFrequency(uint16_t frequency_hz)
{
#if BSP_ENABLE_BUZZER
  uint32_t period;
  if (s_initialized == 0U) { return BSP_STATUS_ERROR; }
  if (frequency_hz == 0U) {
    __HAL_TIM_SET_COMPARE(&s_buzzer_tim, BSP_BUZZER_CHANNEL, 0U);
    s_buzzer_tim.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&s_buzzer_tim, TIM_FLAG_UPDATE);
    return BSP_STATUS_OK;
  }
  if ((frequency_hz < BSP_BUZZER_MIN_HZ) || (frequency_hz > BSP_BUZZER_MAX_HZ)) {
    return BSP_STATUS_ERROR;
  }
  period = (s_counter_hz + frequency_hz / 2U) / frequency_hz;
  if ((period < 2U) || (period > 65536U)) { return BSP_STATUS_ERROR; }
  /* HAL的DISABLE宏在CC1E仍开启时不会停计数；TIM9由本模块独占。 */
  CLEAR_BIT(s_buzzer_tim.Instance->CR1, TIM_CR1_CEN);
  __HAL_TIM_SET_AUTORELOAD(&s_buzzer_tim, period - 1U);
  __HAL_TIM_SET_COMPARE(&s_buzzer_tim, BSP_BUZZER_CHANNEL, period / 2U);
  s_buzzer_tim.Instance->EGR = TIM_EGR_UG;
  __HAL_TIM_CLEAR_FLAG(&s_buzzer_tim, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE(&s_buzzer_tim);
  return BSP_STATUS_OK;
#else
  (void)frequency_hz;
  return BSP_STATUS_ERROR;
#endif
}

/** @brief 释放可选蜂鸣器；MSP反初始化仍保留PE5低电平，避免浮空误驱动。 */
BSP_Status_t BSP_Buzzer_DeInit(void)
{
#if BSP_ENABLE_BUZZER
  BSP_Status_t result;
  if (s_initialized == 0U) { return BSP_STATUS_OK; }
  (void)BSP_Buzzer_SetFrequency(0U);
  (void)HAL_TIM_PWM_Stop(&s_buzzer_tim, BSP_BUZZER_CHANNEL);
  result = (HAL_TIM_PWM_DeInit(&s_buzzer_tim) == HAL_OK) ? BSP_STATUS_OK : BSP_STATUS_ERROR;
  s_initialized = 0U;
  return result;
#else
  return BSP_STATUS_OK;
#endif
}

/** @brief 致命异常下先断开复用输出，再停TIM9，不等待或恢复中断。 */
void BSP_Buzzer_EmergencyStop(void)
{
#if BSP_ENABLE_BUZZER
  __HAL_RCC_GPIOE_CLK_ENABLE();
  GPIOE->BSRR = (uint32_t)GPIO_PIN_5 << 16U;
  GPIOE->OTYPER &= ~(uint32_t)GPIO_PIN_5;
  GPIOE->MODER = (GPIOE->MODER & ~(3UL << 10U)) | (1UL << 10U);
  if ((RCC->APB2ENR & RCC_APB2ENR_TIM9EN) != 0U) {
    TIM9->CCER &= ~TIM_CCER_CC1E;
    TIM9->CR1 &= ~TIM_CR1_CEN;
    TIM9->CCR1 = 0U;
  }
  __DSB();
#endif
}
