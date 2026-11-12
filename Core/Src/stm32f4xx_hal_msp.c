/**
 * @file stm32f4xx_hal_msp.c
 * @brief Configure MCU support resources used by HAL peripherals.
 */

#include "stm32f4xx_hal.h"
#include "bsp_config.h"

void HAL_MspInit(void)
{
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

void HAL_MspDeInit(void)
{
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef gpio;

  gpio.Mode  = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;

  if (huart->Instance == BSP_DBG_UART) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    gpio.Pin       = BSP_DBG_TX_PIN;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Alternate = BSP_DBG_TX_AF;
    HAL_GPIO_Init(BSP_DBG_TX_PORT, &gpio);

    gpio.Pin       = BSP_DBG_RX_PIN;
    gpio.Alternate = BSP_DBG_RX_AF;
    HAL_GPIO_Init(BSP_DBG_RX_PORT, &gpio);
#if (BSP_ENABLE_RPI_UART == 1U)
  } else if (huart->Instance == BSP_RPI_UART) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART6_CLK_ENABLE();

    gpio.Pin       = BSP_RPI_TX_PIN;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Alternate = BSP_RPI_TX_AF;
    HAL_GPIO_Init(BSP_RPI_TX_PORT, &gpio);

    gpio.Pin       = BSP_RPI_RX_PIN;
    gpio.Pull      = GPIO_PULLUP; /* UART RX 空闲为高，须上拉 */
    gpio.Alternate = BSP_RPI_RX_AF;
    HAL_GPIO_Init(BSP_RPI_RX_PORT, &gpio);

    HAL_NVIC_SetPriority(BSP_RPI_IRQn, BSP_RPI_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(BSP_RPI_IRQn);
#endif
  } else if (huart->Instance == BSP_GNSS_UART) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    gpio.Pin       = BSP_GNSS_TX_PIN | BSP_GNSS_RX_PIN;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Alternate = BSP_GNSS_TX_AF;
    HAL_GPIO_Init(BSP_GNSS_TX_PORT, &gpio);

    HAL_NVIC_SetPriority(BSP_GNSS_IRQn, BSP_GNSS_IRQ_PRIORITY, 0U);
    HAL_NVIC_SetPriority(BSP_GNSS_RX_DMA_IRQn, BSP_GNSS_IRQ_PRIORITY, 0U);
    /* GNSS仅设优先级；UART/DMA完整挂接后由BSP开放中断。 */
  } else if (huart->Instance == BSP_LORA_UART) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Pin       = BSP_LORA_TX_PIN;
    gpio.Alternate = BSP_LORA_TX_AF;
    HAL_GPIO_Init(BSP_LORA_TX_PORT, &gpio);

    gpio.Pin       = BSP_LORA_RX_PIN;
    gpio.Pull      = GPIO_PULLUP; /* UART RX 空闲为高，须上拉；不能用下拉(会被当作起始位/break，收不到) */
    gpio.Alternate = BSP_LORA_RX_AF;
    HAL_GPIO_Init(BSP_LORA_RX_PORT, &gpio);

    HAL_NVIC_SetPriority(BSP_LORA_IRQn, BSP_LORA_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(BSP_LORA_IRQn);
    HAL_NVIC_SetPriority(BSP_LORA_RX_DMA_IRQn, BSP_LORA_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(BSP_LORA_RX_DMA_IRQn);
    HAL_NVIC_SetPriority(BSP_LORA_TX_DMA_IRQn, BSP_LORA_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(BSP_LORA_TX_DMA_IRQn);
  } else if (huart->Instance == BSP_REMOTEID_UART) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Pin       = BSP_REMOTEID_TX_PIN;
    gpio.Alternate = BSP_REMOTEID_TX_AF;
    HAL_GPIO_Init(BSP_REMOTEID_TX_PORT, &gpio);

    gpio.Pin       = BSP_REMOTEID_RX_PIN;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Alternate = BSP_REMOTEID_RX_AF;
    HAL_GPIO_Init(BSP_REMOTEID_RX_PORT, &gpio);

    HAL_NVIC_SetPriority(BSP_REMOTEID_IRQn, BSP_REMOTEID_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(BSP_REMOTEID_IRQn);
    HAL_NVIC_SetPriority(BSP_REMOTEID_TX_DMA_IRQn, BSP_REMOTEID_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(BSP_REMOTEID_TX_DMA_IRQn);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  if (huart->Instance == BSP_DBG_UART) {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(BSP_DBG_TX_PORT, BSP_DBG_TX_PIN | BSP_DBG_RX_PIN);
#if (BSP_ENABLE_RPI_UART == 1U)
  } else if (huart->Instance == BSP_RPI_UART) {
    __HAL_RCC_USART6_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(BSP_RPI_IRQn);
    HAL_GPIO_DeInit(BSP_RPI_TX_PORT, BSP_RPI_TX_PIN | BSP_RPI_RX_PIN);
#endif
  } else if (huart->Instance == BSP_GNSS_UART) {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(BSP_GNSS_IRQn);
    HAL_NVIC_DisableIRQ(BSP_GNSS_RX_DMA_IRQn);
    HAL_GPIO_DeInit(BSP_GNSS_TX_PORT, BSP_GNSS_TX_PIN | BSP_GNSS_RX_PIN);
  } else if (huart->Instance == BSP_LORA_UART) {
    __HAL_RCC_USART3_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(BSP_LORA_IRQn);
    HAL_NVIC_DisableIRQ(BSP_LORA_RX_DMA_IRQn);
    HAL_NVIC_DisableIRQ(BSP_LORA_TX_DMA_IRQn);
    HAL_GPIO_DeInit(BSP_LORA_TX_PORT, BSP_LORA_TX_PIN | BSP_LORA_RX_PIN);
  } else if (huart->Instance == BSP_REMOTEID_UART) {
    __HAL_RCC_UART4_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(BSP_REMOTEID_IRQn);
    HAL_NVIC_DisableIRQ(BSP_REMOTEID_TX_DMA_IRQn);
    HAL_GPIO_DeInit(BSP_REMOTEID_TX_PORT, BSP_REMOTEID_TX_PIN | BSP_REMOTEID_RX_PIN);
  }
}

/** @brief 按外设实例配置RGB、蜂鸣器和电机PWM资源，各可选输出独立使能。 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
#if BSP_ENABLE_RGB
  if (htim->Instance == BSP_RGB_TIM) {
    GPIO_InitTypeDef rgb_gpio = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    /* TIM8由RGB独占，重建时清除遗留从模式/中断，避免旧触发配置阻止启动。 */
    __HAL_RCC_TIM8_FORCE_RESET();
    __HAL_RCC_TIM8_RELEASE_RESET();
    __HAL_RCC_DMA2_CLK_ENABLE();
    HAL_GPIO_WritePin(BSP_RGB_PORT, BSP_RGB_PIN, GPIO_PIN_RESET);
    rgb_gpio.Pin = BSP_RGB_PIN;
    rgb_gpio.Mode = GPIO_MODE_AF_PP;
    rgb_gpio.Pull = GPIO_PULLDOWN;
    rgb_gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    rgb_gpio.Alternate = BSP_RGB_AF;
    HAL_GPIO_Init(BSP_RGB_PORT, &rgb_gpio);
    HAL_NVIC_DisableIRQ(BSP_RGB_DMA_IRQn);
    HAL_NVIC_SetPriority(BSP_RGB_DMA_IRQn, BSP_RGB_IRQ_PRIORITY, 0U);
    /* DMA句柄挂接、清理挂起标志后由BSP开放NVIC，避免半初始化IRQ。 */
    return;
  }
#endif
#if BSP_ENABLE_BUZZER
  if (htim->Instance == BSP_BUZZER_TIM) {
    GPIO_InitTypeDef buzzer_gpio = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_TIM9_CLK_ENABLE();
    HAL_GPIO_WritePin(BSP_BUZZER_PORT, BSP_BUZZER_PIN, GPIO_PIN_RESET);
    buzzer_gpio.Pin = BSP_BUZZER_PIN;
    buzzer_gpio.Mode = GPIO_MODE_AF_PP;
    buzzer_gpio.Pull = GPIO_PULLDOWN;
    buzzer_gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    buzzer_gpio.Alternate = BSP_BUZZER_AF;
    HAL_GPIO_Init(BSP_BUZZER_PORT, &buzzer_gpio);
    return;
  }
#endif
#if (BSP_ENABLE_PWM == 1U)
  GPIO_InitTypeDef gpio;

  gpio.Mode  = GPIO_MODE_AF_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;

  if (htim->Instance == BSP_PWM_MOTOR1_TIM) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin       = BSP_PWM_MOTOR1_PIN;
    gpio.Alternate = BSP_PWM_MOTOR1_AF;
    HAL_GPIO_Init(BSP_PWM_MOTOR1_PORT, &gpio);

    gpio.Pin       = BSP_PWM_MOTOR2_PIN;
    gpio.Alternate = BSP_PWM_MOTOR2_AF;
    HAL_GPIO_Init(BSP_PWM_MOTOR2_PORT, &gpio);
  } else if (htim->Instance == BSP_PWM_MOTOR3_TIM) {
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    gpio.Pin       = BSP_PWM_MOTOR3_PIN;
    gpio.Alternate = BSP_PWM_MOTOR3_AF;
    HAL_GPIO_Init(BSP_PWM_MOTOR3_PORT, &gpio);

    gpio.Pin       = BSP_PWM_MOTOR4_PIN;
    gpio.Alternate = BSP_PWM_MOTOR4_AF;
    HAL_GPIO_Init(BSP_PWM_MOTOR4_PORT, &gpio);
  }
#else
  (void)htim;
#endif
}

/** @brief 释放可选输出后固定对应引脚为低，不关闭共享DMA控制器时钟。 */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim)
{
#if BSP_ENABLE_RGB
  if (htim->Instance == BSP_RGB_TIM) {
    GPIO_InitTypeDef rgb_gpio = {0};
    HAL_NVIC_DisableIRQ(BSP_RGB_DMA_IRQn);
    HAL_NVIC_ClearPendingIRQ(BSP_RGB_DMA_IRQn);
    __HAL_RCC_GPIOC_CLK_ENABLE();
    HAL_GPIO_WritePin(BSP_RGB_PORT, BSP_RGB_PIN, GPIO_PIN_RESET);
    rgb_gpio.Pin = BSP_RGB_PIN;
    rgb_gpio.Mode = GPIO_MODE_OUTPUT_PP;
    rgb_gpio.Pull = GPIO_PULLDOWN;
    rgb_gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BSP_RGB_PORT, &rgb_gpio);
    __HAL_RCC_TIM8_CLK_DISABLE();
    /* DMA2时钟可能由其他外设使用，不在释放单个Stream时关闭。 */
    return;
  }
#endif
#if BSP_ENABLE_BUZZER
  if (htim->Instance == BSP_BUZZER_TIM) {
    GPIO_InitTypeDef buzzer_gpio = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();
    HAL_GPIO_WritePin(BSP_BUZZER_PORT, BSP_BUZZER_PIN, GPIO_PIN_RESET);
    buzzer_gpio.Pin = BSP_BUZZER_PIN;
    buzzer_gpio.Mode = GPIO_MODE_OUTPUT_PP;
    buzzer_gpio.Pull = GPIO_PULLDOWN;
    buzzer_gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BSP_BUZZER_PORT, &buzzer_gpio);
    __HAL_RCC_TIM9_CLK_DISABLE();
    return;
  }
#endif
#if (BSP_ENABLE_PWM == 1U)
  if (htim->Instance == BSP_PWM_MOTOR1_TIM) {
    __HAL_RCC_TIM3_CLK_DISABLE();
    HAL_GPIO_DeInit(BSP_PWM_MOTOR1_PORT, BSP_PWM_MOTOR1_PIN | BSP_PWM_MOTOR2_PIN);
  } else if (htim->Instance == BSP_PWM_MOTOR3_TIM) {
    __HAL_RCC_TIM4_CLK_DISABLE();
    HAL_GPIO_DeInit(BSP_PWM_MOTOR3_PORT, BSP_PWM_MOTOR3_PIN | BSP_PWM_MOTOR4_PIN);
  }
#else
  (void)htim;
#endif
}

/**
 * @brief 初始化 IMU 专用 SPI2 时钟与 GPIO，其他 SPI 实例保持其原有初始化路径。
 * @details 先将两个 CS 输出锁存为高，再配置推挽输出，避免初始化时误选中器件。
 *          采用轮询收发，不分配 DMA，不开启 SPI2 IRQ。
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef gpio = {0};

  if (hspi->Instance != BSP_IMU_SPI_INS) { return; }
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_SPI2_CLK_ENABLE();

  HAL_GPIO_WritePin(BSP_IMU_SPI_ACCEL_CS_PORT, BSP_IMU_SPI_ACCEL_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BSP_IMU_SPI_GYRO_CS_PORT, BSP_IMU_SPI_GYRO_CS_PIN, GPIO_PIN_SET);
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pin = BSP_IMU_SPI_ACCEL_CS_PIN;
  HAL_GPIO_Init(BSP_IMU_SPI_ACCEL_CS_PORT, &gpio);
  gpio.Pin = BSP_IMU_SPI_GYRO_CS_PIN;
  HAL_GPIO_Init(BSP_IMU_SPI_GYRO_CS_PORT, &gpio);

  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Pin = BSP_IMU_SPI_SCK_PIN;
  gpio.Alternate = BSP_IMU_SPI_SCK_AF;
  HAL_GPIO_Init(BSP_IMU_SPI_SCK_PORT, &gpio);
  gpio.Pin = BSP_IMU_SPI_MISO_PIN;
  gpio.Alternate = BSP_IMU_SPI_MISO_AF;
  HAL_GPIO_Init(BSP_IMU_SPI_MISO_PORT, &gpio);
  gpio.Pin = BSP_IMU_SPI_MOSI_PIN;
  gpio.Alternate = BSP_IMU_SPI_MOSI_AF;
  HAL_GPIO_Init(BSP_IMU_SPI_MOSI_PORT, &gpio);
}

/** @brief 释放 SPI2 数据 GPIO 和时钟，保留两个片选为高电平输出。 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance != BSP_IMU_SPI_INS) { return; }
  HAL_GPIO_WritePin(BSP_IMU_SPI_ACCEL_CS_PORT, BSP_IMU_SPI_ACCEL_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BSP_IMU_SPI_GYRO_CS_PORT, BSP_IMU_SPI_GYRO_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_DeInit(BSP_IMU_SPI_SCK_PORT, BSP_IMU_SPI_SCK_PIN);
  HAL_GPIO_DeInit(BSP_IMU_SPI_MISO_PORT, BSP_IMU_SPI_MISO_PIN);
  HAL_GPIO_DeInit(BSP_IMU_SPI_MOSI_PORT, BSP_IMU_SPI_MOSI_PIN);
  __HAL_RCC_SPI2_CLK_DISABLE();
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef gpio;

  if (hi2c->Instance != BSP_I2C_INS) { return; }

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  gpio.Mode  = GPIO_MODE_AF_OD;
  gpio.Pull  = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;

  gpio.Pin       = BSP_I2C_SCL_PIN;
  gpio.Alternate = BSP_I2C_SCL_AF;
  HAL_GPIO_Init(BSP_I2C_SCL_PORT, &gpio);

  gpio.Pin       = BSP_I2C_SDA_PIN;
  gpio.Alternate = BSP_I2C_SDA_AF;
  HAL_GPIO_Init(BSP_I2C_SDA_PORT, &gpio);
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != BSP_I2C_INS) { return; }

  __HAL_RCC_I2C1_CLK_DISABLE();
  HAL_GPIO_DeInit(BSP_I2C_SCL_PORT, BSP_I2C_SCL_PIN);
  HAL_GPIO_DeInit(BSP_I2C_SDA_PORT, BSP_I2C_SDA_PIN);
}

/** @brief 配置启用组的模拟输入；ADC1 内部参考保持可用，停用组不占用 GPIO。 */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
#if BSP_ENABLE_POWER1 || BSP_ENABLE_POWER2
  GPIO_InitTypeDef gpio;

  gpio.Mode  = GPIO_MODE_ANALOG;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
#endif

  if (hadc->Instance == BSP_ADC_INS) {
    __HAL_RCC_ADC1_CLK_ENABLE();

#if BSP_ENABLE_POWER1
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = BSP_ADC_PIN;
    HAL_GPIO_Init(BSP_ADC_PORT, &gpio);
#endif
  } else if (hadc->Instance == BSP_ADC2_INS) {
#if BSP_ENABLE_POWER2
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();

    gpio.Pin = BSP_ADC2_PIN;
    HAL_GPIO_Init(BSP_ADC2_PORT, &gpio);
#endif
  } else if (hadc->Instance == BSP_ADC_CURRENT_INS) {
    /* 仅初始化启用组；PC0 停用后不配置其引脚。 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();

#if BSP_ENABLE_POWER1
    gpio.Pin = BSP_ADC_CURRENT1_PIN;
    HAL_GPIO_Init(BSP_ADC_CURRENT1_PORT, &gpio);
#endif
#if BSP_ENABLE_POWER2
    gpio.Pin = BSP_ADC_CURRENT2_PIN;
    HAL_GPIO_Init(BSP_ADC_CURRENT2_PORT, &gpio);
#endif
  }
}

/** @brief 释放对应 ADC 与启用组 GPIO；不得重置已停用组的可复用引脚。 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == BSP_ADC_INS) {
    __HAL_RCC_ADC1_CLK_DISABLE();
#if BSP_ENABLE_POWER1
    HAL_GPIO_DeInit(BSP_ADC_PORT, BSP_ADC_PIN);
#endif
  } else if (hadc->Instance == BSP_ADC2_INS) {
#if BSP_ENABLE_POWER2
    __HAL_RCC_ADC2_CLK_DISABLE();
    HAL_GPIO_DeInit(BSP_ADC2_PORT, BSP_ADC2_PIN);
#endif
  } else if (hadc->Instance == BSP_ADC_CURRENT_INS) {
    __HAL_RCC_ADC3_CLK_DISABLE();
#if BSP_ENABLE_POWER1
    HAL_GPIO_DeInit(BSP_ADC_CURRENT1_PORT, BSP_ADC_CURRENT1_PIN);
#endif
#if BSP_ENABLE_POWER2
    HAL_GPIO_DeInit(BSP_ADC_CURRENT2_PORT, BSP_ADC_CURRENT2_PIN);
#endif
  }
}

/**
 * @brief Configure one FSMC GPIO pin for the LCD 8080 parallel bus.
 */
static void HAL_DisplayFsmcPinInit(GPIO_TypeDef *port, uint16_t pin, uint8_t alternate)
{
  GPIO_InitTypeDef gpio;

  gpio.Pin       = pin;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_PULLUP;
  gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = alternate;
  HAL_GPIO_Init(port, &gpio);
}

/**
 * @brief Initialize MCU GPIO resources used by the ATK-MD0700 LCD bus.
 */
void HAL_DisplayLcdMspInit(void)
{
#if (BSP_DISPLAY_ENABLE == 1U) && (BSP_DISPLAY_USE_FSMC_8080 == 1U)
  GPIO_InitTypeDef gpio;

  BSP_LCD_FSMC_GPIO_CLK_ENABLE();
  BSP_LCD_BL_GPIO_CLK_ENABLE();

  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D0_PORT, BSP_LCD_FSMC_D0_PIN, BSP_LCD_FSMC_D0_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D1_PORT, BSP_LCD_FSMC_D1_PIN, BSP_LCD_FSMC_D1_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D2_PORT, BSP_LCD_FSMC_D2_PIN, BSP_LCD_FSMC_D2_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D3_PORT, BSP_LCD_FSMC_D3_PIN, BSP_LCD_FSMC_D3_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D4_PORT, BSP_LCD_FSMC_D4_PIN, BSP_LCD_FSMC_D4_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D5_PORT, BSP_LCD_FSMC_D5_PIN, BSP_LCD_FSMC_D5_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D6_PORT, BSP_LCD_FSMC_D6_PIN, BSP_LCD_FSMC_D6_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D7_PORT, BSP_LCD_FSMC_D7_PIN, BSP_LCD_FSMC_D7_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D8_PORT, BSP_LCD_FSMC_D8_PIN, BSP_LCD_FSMC_D8_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D9_PORT, BSP_LCD_FSMC_D9_PIN, BSP_LCD_FSMC_D9_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D10_PORT, BSP_LCD_FSMC_D10_PIN, BSP_LCD_FSMC_D10_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D11_PORT, BSP_LCD_FSMC_D11_PIN, BSP_LCD_FSMC_D11_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D12_PORT, BSP_LCD_FSMC_D12_PIN, BSP_LCD_FSMC_D12_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D13_PORT, BSP_LCD_FSMC_D13_PIN, BSP_LCD_FSMC_D13_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D14_PORT, BSP_LCD_FSMC_D14_PIN, BSP_LCD_FSMC_D14_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D15_PORT, BSP_LCD_FSMC_D15_PIN, BSP_LCD_FSMC_D15_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_CS_PORT, BSP_LCD_FSMC_CS_PIN, BSP_LCD_FSMC_CS_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_RS_PORT, BSP_LCD_FSMC_RS_PIN, BSP_LCD_FSMC_RS_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_RD_PORT, BSP_LCD_FSMC_RD_PIN, BSP_LCD_FSMC_RD_AF);
  HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_WR_PORT, BSP_LCD_FSMC_WR_PIN, BSP_LCD_FSMC_WR_AF);

  gpio.Pin   = BSP_LCD_BL_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BSP_LCD_BL_PORT, &gpio);
  HAL_GPIO_WritePin(BSP_LCD_BL_PORT,
                    BSP_LCD_BL_PIN,
                    (BSP_LCD_BL_ACTIVE_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}

/**
 * @brief Initialize MCU GPIO resources used by the GT911 software I2C port.
 */
void HAL_DisplayTouchMspInit(void)
{
#if (BSP_TOUCH_ENABLE == 1U) && (BSP_TOUCH_USE_SOFT_I2C == 1U)
  GPIO_InitTypeDef gpio;

  BSP_TOUCH_GPIO_CLK_ENABLE();

  gpio.Pin   = BSP_TOUCH_SCL_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BSP_TOUCH_SCL_PORT, &gpio);

  gpio.Pin   = BSP_TOUCH_SDA_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_OD;
  gpio.Pull  = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BSP_TOUCH_SDA_PORT, &gpio);

  gpio.Pin   = BSP_TOUCH_RST_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BSP_TOUCH_RST_PORT, &gpio);

  gpio.Pin   = BSP_TOUCH_INT_PIN;
  gpio.Mode  = GPIO_MODE_INPUT;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BSP_TOUCH_INT_PORT, &gpio);

  HAL_GPIO_WritePin(BSP_TOUCH_SCL_PORT, BSP_TOUCH_SCL_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BSP_TOUCH_SDA_PORT, BSP_TOUCH_SDA_PIN, GPIO_PIN_SET);
#endif
}

/**
 * @brief Temporarily drive the GT911 INT line during address selection.
 */
void HAL_DisplayTouchSetIntOutput(uint8_t level)
{
#if (BSP_TOUCH_ENABLE == 1U) && (BSP_TOUCH_USE_SOFT_I2C == 1U)
  GPIO_InitTypeDef gpio;

  gpio.Pin   = BSP_TOUCH_INT_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BSP_TOUCH_INT_PORT, &gpio);
  HAL_GPIO_WritePin(BSP_TOUCH_INT_PORT, BSP_TOUCH_INT_PIN, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
  (void)level;
#endif
}

/**
 * @brief Restore the GT911 INT line to input mode after reset sequencing.
 */
void HAL_DisplayTouchSetIntInput(void)
{
#if (BSP_TOUCH_ENABLE == 1U) && (BSP_TOUCH_USE_SOFT_I2C == 1U)
  GPIO_InitTypeDef gpio;

  gpio.Pin   = BSP_TOUCH_INT_PIN;
  gpio.Mode  = GPIO_MODE_INPUT;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BSP_TOUCH_INT_PORT, &gpio);
#endif
}
