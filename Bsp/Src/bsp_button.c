/**
 * @file bsp_button.c
 * @brief 实现板载四个按键的 GPIO 输入读取。
 *
 * @details
 * WKUP 为高电平按下，其余 KEY0/KEY1/KEY2 为低电平按下。按键去抖和业务边沿
 * 判断由上层 Control 模块负责，BSP 不持有业务状态。
 */

#include "bsp_button.h"

#include "bsp_config.h"
#include "stm32f4xx_hal.h"

/**
 * @brief 单个按键的 GPIO 映射。
 */
typedef struct {
  GPIO_TypeDef *port; /**< GPIO 端口。 */
  uint16_t pin;       /**< GPIO 引脚。 */
  uint8_t active_high;/**< 1 表示高电平按下，0 表示低电平按下。 */
} Bsp_ButtonMap_t;

static const Bsp_ButtonMap_t s_button_map[BSP_BUTTON_COUNT] = {
  {BSP_BTN_WKUP_PORT, BSP_BTN_WKUP_PIN, 1U},
  {BSP_BTN_KEY0_PORT, BSP_BTN_KEY0_PIN, 0U},
  {BSP_BTN_KEY1_PORT, BSP_BTN_KEY1_PIN, 0U},
  {BSP_BTN_KEY2_PORT, BSP_BTN_KEY2_PIN, 0U},
};

BSP_Status_t BSP_Button_Init(void)
{
  GPIO_InitTypeDef gpio;
  uint8_t i;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  gpio.Mode  = GPIO_MODE_INPUT;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  for (i = 0U; i < (uint8_t)BSP_BUTTON_COUNT; i++) {
    gpio.Pin  = s_button_map[i].pin;
    gpio.Pull = (s_button_map[i].active_high != 0U) ? GPIO_PULLDOWN : GPIO_PULLUP;
    HAL_GPIO_Init(s_button_map[i].port, &gpio);
  }

  return BSP_STATUS_OK;
}

uint8_t BSP_Button_IsPressed(BSP_ButtonId_t button)
{
  GPIO_PinState level;

  if (button >= BSP_BUTTON_COUNT) { return 0U; }

  level = HAL_GPIO_ReadPin(s_button_map[button].port, s_button_map[button].pin);
  if (s_button_map[button].active_high != 0U) { return (level == GPIO_PIN_SET) ? 1U : 0U; }
  return (level == GPIO_PIN_RESET) ? 1U : 0U;
}
