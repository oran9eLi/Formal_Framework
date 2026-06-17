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

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    if (huart->Instance == BSP_DBG_UART)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART1_CLK_ENABLE();

        gpio.Pin = BSP_DBG_TX_PIN;
        gpio.Pull = GPIO_NOPULL;
        gpio.Alternate = BSP_DBG_TX_AF;
        HAL_GPIO_Init(BSP_DBG_TX_PORT, &gpio);

        gpio.Pin = BSP_DBG_RX_PIN;
        gpio.Alternate = BSP_DBG_RX_AF;
        HAL_GPIO_Init(BSP_DBG_RX_PORT, &gpio);
    }
    else if (huart->Instance == BSP_GNSS_UART)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();

        gpio.Pin = BSP_GNSS_TX_PIN | BSP_GNSS_RX_PIN;
        gpio.Pull = GPIO_PULLUP;
        gpio.Alternate = BSP_GNSS_TX_AF;
        HAL_GPIO_Init(BSP_GNSS_TX_PORT, &gpio);

        HAL_NVIC_SetPriority(BSP_GNSS_IRQn,
                             BSP_GNSS_IRQ_PRIORITY,
                             0U);
        HAL_NVIC_EnableIRQ(BSP_GNSS_IRQn);
        HAL_NVIC_SetPriority(BSP_GNSS_RX_DMA_IRQn,
                             BSP_GNSS_IRQ_PRIORITY,
                             0U);
        HAL_NVIC_EnableIRQ(BSP_GNSS_RX_DMA_IRQn);
    }
    else if (huart->Instance == BSP_LORA_UART)
    {
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
        gpio.Pull      = GPIO_PULLUP;
        gpio.Alternate = BSP_LORA_RX_AF;
        HAL_GPIO_Init(BSP_LORA_RX_PORT, &gpio);

        HAL_NVIC_SetPriority(BSP_LORA_IRQn,
                             BSP_LORA_IRQ_PRIORITY, 0U);
        HAL_NVIC_EnableIRQ(BSP_LORA_IRQn);
        HAL_NVIC_SetPriority(BSP_LORA_RX_DMA_IRQn,
                             BSP_LORA_IRQ_PRIORITY, 0U);
        HAL_NVIC_EnableIRQ(BSP_LORA_RX_DMA_IRQn);
        HAL_NVIC_SetPriority(BSP_LORA_TX_DMA_IRQn,
                             BSP_LORA_IRQ_PRIORITY, 0U);
        HAL_NVIC_EnableIRQ(BSP_LORA_TX_DMA_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == BSP_DBG_UART)
    {
        __HAL_RCC_USART1_CLK_DISABLE();
        HAL_GPIO_DeInit(BSP_DBG_TX_PORT,
                        BSP_DBG_TX_PIN | BSP_DBG_RX_PIN);
    }
    else if (huart->Instance == BSP_GNSS_UART)
    {
        __HAL_RCC_USART2_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(BSP_GNSS_IRQn);
        HAL_NVIC_DisableIRQ(BSP_GNSS_RX_DMA_IRQn);
        HAL_GPIO_DeInit(BSP_GNSS_TX_PORT,
                        BSP_GNSS_TX_PIN | BSP_GNSS_RX_PIN);
    }
    else if (huart->Instance == BSP_LORA_UART)
    {
        __HAL_RCC_USART3_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(BSP_LORA_IRQn);
        HAL_NVIC_DisableIRQ(BSP_LORA_RX_DMA_IRQn);
        HAL_NVIC_DisableIRQ(BSP_LORA_TX_DMA_IRQn);
        HAL_GPIO_DeInit(BSP_LORA_TX_PORT,
                        BSP_LORA_TX_PIN | BSP_LORA_RX_PIN);
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef gpio;

    if (hi2c->Instance != BSP_I2C_INS)
    {
        return;
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio.Pin = BSP_I2C_SCL_PIN;
    gpio.Alternate = BSP_I2C_SCL_AF;
    HAL_GPIO_Init(BSP_I2C_SCL_PORT, &gpio);

    gpio.Pin = BSP_I2C_SDA_PIN;
    gpio.Alternate = BSP_I2C_SDA_AF;
    HAL_GPIO_Init(BSP_I2C_SDA_PORT, &gpio);
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != BSP_I2C_INS)
    {
        return;
    }

    __HAL_RCC_I2C1_CLK_DISABLE();
    HAL_GPIO_DeInit(BSP_I2C_SCL_PORT, BSP_I2C_SCL_PIN);
    HAL_GPIO_DeInit(BSP_I2C_SDA_PORT, BSP_I2C_SDA_PIN);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef gpio;

    if (hadc->Instance != BSP_ADC_INS)
    {
        return;
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    gpio.Pin = BSP_ADC_PIN;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BSP_ADC_PORT, &gpio);
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != BSP_ADC_INS)
    {
        return;
    }

    __HAL_RCC_ADC1_CLK_DISABLE();
    HAL_GPIO_DeInit(BSP_ADC_PORT, BSP_ADC_PIN);
}

/**
 * @brief Configure one FSMC GPIO pin for the LCD 8080 parallel bus.
 */
static void HAL_DisplayFsmcPinInit(GPIO_TypeDef *port,
                                   uint16_t pin,
                                   uint8_t alternate)
{
    GPIO_InitTypeDef gpio;

    gpio.Pin = pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
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

    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D0_PORT,
                           BSP_LCD_FSMC_D0_PIN,
                           BSP_LCD_FSMC_D0_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D1_PORT,
                           BSP_LCD_FSMC_D1_PIN,
                           BSP_LCD_FSMC_D1_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D2_PORT,
                           BSP_LCD_FSMC_D2_PIN,
                           BSP_LCD_FSMC_D2_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D3_PORT,
                           BSP_LCD_FSMC_D3_PIN,
                           BSP_LCD_FSMC_D3_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D4_PORT,
                           BSP_LCD_FSMC_D4_PIN,
                           BSP_LCD_FSMC_D4_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D5_PORT,
                           BSP_LCD_FSMC_D5_PIN,
                           BSP_LCD_FSMC_D5_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D6_PORT,
                           BSP_LCD_FSMC_D6_PIN,
                           BSP_LCD_FSMC_D6_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D7_PORT,
                           BSP_LCD_FSMC_D7_PIN,
                           BSP_LCD_FSMC_D7_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D8_PORT,
                           BSP_LCD_FSMC_D8_PIN,
                           BSP_LCD_FSMC_D8_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D9_PORT,
                           BSP_LCD_FSMC_D9_PIN,
                           BSP_LCD_FSMC_D9_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D10_PORT,
                           BSP_LCD_FSMC_D10_PIN,
                           BSP_LCD_FSMC_D10_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D11_PORT,
                           BSP_LCD_FSMC_D11_PIN,
                           BSP_LCD_FSMC_D11_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D12_PORT,
                           BSP_LCD_FSMC_D12_PIN,
                           BSP_LCD_FSMC_D12_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D13_PORT,
                           BSP_LCD_FSMC_D13_PIN,
                           BSP_LCD_FSMC_D13_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D14_PORT,
                           BSP_LCD_FSMC_D14_PIN,
                           BSP_LCD_FSMC_D14_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_D15_PORT,
                           BSP_LCD_FSMC_D15_PIN,
                           BSP_LCD_FSMC_D15_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_CS_PORT,
                           BSP_LCD_FSMC_CS_PIN,
                           BSP_LCD_FSMC_CS_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_RS_PORT,
                           BSP_LCD_FSMC_RS_PIN,
                           BSP_LCD_FSMC_RS_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_RD_PORT,
                           BSP_LCD_FSMC_RD_PIN,
                           BSP_LCD_FSMC_RD_AF);
    HAL_DisplayFsmcPinInit(BSP_LCD_FSMC_WR_PORT,
                           BSP_LCD_FSMC_WR_PIN,
                           BSP_LCD_FSMC_WR_AF);

    gpio.Pin = BSP_LCD_BL_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BSP_LCD_BL_PORT, &gpio);
    HAL_GPIO_WritePin(BSP_LCD_BL_PORT,
                      BSP_LCD_BL_PIN,
                      BSP_LCD_BL_ACTIVE_LEVEL);
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

    gpio.Pin = BSP_TOUCH_SCL_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BSP_TOUCH_SCL_PORT, &gpio);

    gpio.Pin = BSP_TOUCH_SDA_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BSP_TOUCH_SDA_PORT, &gpio);

    gpio.Pin = BSP_TOUCH_RST_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BSP_TOUCH_RST_PORT, &gpio);

    gpio.Pin = BSP_TOUCH_INT_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BSP_TOUCH_INT_PORT, &gpio);

    HAL_GPIO_WritePin(BSP_TOUCH_SCL_PORT,
                      BSP_TOUCH_SCL_PIN,
                      GPIO_PIN_SET);
    HAL_GPIO_WritePin(BSP_TOUCH_SDA_PORT,
                      BSP_TOUCH_SDA_PIN,
                      GPIO_PIN_SET);
#endif
}

/**
 * @brief Temporarily drive the GT911 INT line during address selection.
 */
void HAL_DisplayTouchSetIntOutput(uint8_t level)
{
#if (BSP_TOUCH_ENABLE == 1U) && (BSP_TOUCH_USE_SOFT_I2C == 1U)
    GPIO_InitTypeDef gpio;

    gpio.Pin = BSP_TOUCH_INT_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BSP_TOUCH_INT_PORT, &gpio);
    HAL_GPIO_WritePin(BSP_TOUCH_INT_PORT,
                      BSP_TOUCH_INT_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
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

    gpio.Pin = BSP_TOUCH_INT_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BSP_TOUCH_INT_PORT, &gpio);
#endif
}
