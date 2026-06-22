/**
 * @file bsp_pwm.c
 * @brief 实现 TIM3/TIM4 四路电机 PWM 原始输出。
 *
 * @details
 * 本文件只管理定时器基础频率、PWM 通道和比较值。Framework 的 Control 模块负责
 * 目标油门、急停和安全状态，BSP 不解释业务命令。
 */

#include "bsp_pwm.h"

#include "bsp_config.h"

#if BSP_PWM_FREQ == 0U
#error "BSP_PWM_FREQ must be non-zero"
#endif

static TIM_HandleTypeDef s_tim_motor12;
static TIM_HandleTypeDef s_tim_motor34;
static uint32_t s_arr;

/**
 * @brief PWM 通道到 TIM 句柄和 HAL 通道的映射。
 */
typedef struct {
  TIM_HandleTypeDef *htim; /**< TIM 句柄。 */
  uint32_t channel;        /**< HAL TIM 通道。 */
} Bsp_PwmMap_t;

static const Bsp_PwmMap_t s_pwm_map[BSP_PWM_MOTOR_COUNT] = {
  {&s_tim_motor12, BSP_PWM_MOTOR1_CH},
  {&s_tim_motor12, BSP_PWM_MOTOR2_CH},
  {&s_tim_motor34, BSP_PWM_MOTOR3_CH},
  {&s_tim_motor34, BSP_PWM_MOTOR4_CH},
};

/**
 * @brief 计算 TIM 输入时钟频率。
 */
static uint32_t Bsp_PwmGetTimerClockHz(void)
{
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();

  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_HCLK_DIV1) { pclk1 *= 2U; }
  return pclk1;
}

/**
 * @brief 初始化一个 TIM PWM 基础配置。
 */
static BSP_Status_t Bsp_PwmInitBase(TIM_HandleTypeDef *htim, TIM_TypeDef *instance, uint32_t prescaler)
{
  htim->Instance               = instance;
  htim->Init.Prescaler         = prescaler;
  htim->Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim->Init.Period            = s_arr;
  htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  return (HAL_TIM_PWM_Init(htim) == HAL_OK) ? BSP_STATUS_OK : BSP_STATUS_ERROR;
}

/**
 * @brief 配置并启动一个 PWM 输出通道。
 */
static BSP_Status_t Bsp_PwmStartChannel(TIM_HandleTypeDef *htim, uint32_t channel)
{
  TIM_OC_InitTypeDef oc;

  oc.OCMode     = TIM_OCMODE_PWM1;
  oc.Pulse      = 0U;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(htim, &oc, channel) != HAL_OK) { return BSP_STATUS_ERROR; }
  if (HAL_TIM_PWM_Start(htim, channel) != HAL_OK) { return BSP_STATUS_ERROR; }
  return BSP_STATUS_OK;
}

BSP_Status_t BSP_PWM_Init(void)
{
  uint32_t timer_hz;
  uint32_t prescaler;
  uint8_t i;

  timer_hz  = Bsp_PwmGetTimerClockHz();
  prescaler = (timer_hz / 1000000U) - 1U;
  s_arr     = (1000000U / BSP_PWM_FREQ) - 1U;

  if (Bsp_PwmInitBase(&s_tim_motor12, BSP_PWM_MOTOR1_TIM, prescaler) != BSP_STATUS_OK) { return BSP_STATUS_ERROR; }
  if (Bsp_PwmInitBase(&s_tim_motor34, BSP_PWM_MOTOR3_TIM, prescaler) != BSP_STATUS_OK) { return BSP_STATUS_ERROR; }

  for (i = 0U; i < (uint8_t)BSP_PWM_MOTOR_COUNT; i++) {
    if (Bsp_PwmStartChannel(s_pwm_map[i].htim, s_pwm_map[i].channel) != BSP_STATUS_OK) { return BSP_STATUS_ERROR; }
  }

  return BSP_STATUS_OK;
}

BSP_Status_t BSP_PWM_DeInit(void)
{
  uint8_t i;

  for (i = 0U; i < (uint8_t)BSP_PWM_MOTOR_COUNT; i++) {
    (void)HAL_TIM_PWM_Stop(s_pwm_map[i].htim, s_pwm_map[i].channel);
  }
  if (HAL_TIM_PWM_DeInit(&s_tim_motor12) != HAL_OK) { return BSP_STATUS_ERROR; }
  if (HAL_TIM_PWM_DeInit(&s_tim_motor34) != HAL_OK) { return BSP_STATUS_ERROR; }
  return BSP_STATUS_OK;
}

BSP_Status_t BSP_PWM_SetPulseUs(BSP_PwmChannel_t channel, uint16_t pulse_us)
{
  uint32_t pulse;

  if (channel >= BSP_PWM_MOTOR_COUNT) { return BSP_STATUS_ERROR; }

  pulse = (uint32_t)pulse_us;
  if (pulse > s_arr) { pulse = s_arr; }
  __HAL_TIM_SET_COMPARE(s_pwm_map[channel].htim, s_pwm_map[channel].channel, pulse);
  return BSP_STATUS_OK;
}

TIM_HandleTypeDef *BSP_PWM_GetTimHandle(BSP_PwmChannel_t channel)
{
  if (channel >= BSP_PWM_MOTOR_COUNT) { return 0; }
  return s_pwm_map[channel].htim;
}
