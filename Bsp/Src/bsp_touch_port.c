/**
 * @file bsp_touch_port.c
 * @brief Implement the software I2C GPIO port used by the GT911 touch driver.
 */

#include "bsp_touch_port.h"

#include "bsp_config.h"
#include "stm32f4xx_hal.h"

void HAL_DisplayTouchMspInit(void);
void HAL_DisplayTouchSetIntOutput(uint8_t level);
void HAL_DisplayTouchSetIntInput(void);

static uint8_t s_touch_port_last_error;

/**
 * @brief Delay one software I2C half-period using a small CPU loop.
 */
static void BSP_TouchPort_Delay(void)
{
    volatile uint8_t i;

    for (i = 0U; i < 80U; i++)
    {
        __NOP();
    }
}

/**
 * @brief Drive the software I2C SCL line.
 */
static void BSP_TouchPort_WriteScl(uint8_t level)
{
    HAL_GPIO_WritePin(BSP_TOUCH_SCL_PORT,
                      BSP_TOUCH_SCL_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Drive the software I2C SDA line.
 */
static void BSP_TouchPort_WriteSda(uint8_t level)
{
    HAL_GPIO_WritePin(BSP_TOUCH_SDA_PORT,
                      BSP_TOUCH_SDA_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Stop a register transaction and store a diagnostic error code.
 */
static BSP_TouchPortResult_t BSP_TouchPort_Fail(uint8_t code)
{
    s_touch_port_last_error = code;
    return BSP_TOUCH_PORT_ERROR;
}

/**
 * @brief Generate a software I2C start condition.
 */
static void BSP_TouchPort_Start(void)
{
    BSP_TouchPort_WriteSda(1U);
    BSP_TouchPort_WriteScl(1U);
    BSP_TouchPort_Delay();
    BSP_TouchPort_WriteSda(0U);
    BSP_TouchPort_Delay();
    BSP_TouchPort_WriteScl(0U);
}

/**
 * @brief Generate a software I2C stop condition.
 */
static void BSP_TouchPort_Stop(void)
{
    BSP_TouchPort_WriteScl(0U);
    BSP_TouchPort_WriteSda(0U);
    BSP_TouchPort_Delay();
    BSP_TouchPort_WriteScl(1U);
    BSP_TouchPort_Delay();
    BSP_TouchPort_WriteSda(1U);
    BSP_TouchPort_Delay();
}

/**
 * @brief Wait for a software I2C ACK bit.
 */
static uint8_t BSP_TouchPort_WaitAck(void)
{
    uint16_t timeout = 0U;

    BSP_TouchPort_WriteSda(1U);
    BSP_TouchPort_Delay();
    BSP_TouchPort_WriteScl(1U);
    BSP_TouchPort_Delay();
    while (BSP_TouchPort_ReadSda() != 0U)
    {
        timeout++;
        if (timeout > 1000U)
        {
            BSP_TouchPort_Stop();
            return 1U;
        }
        BSP_TouchPort_Delay();
    }
    BSP_TouchPort_WriteScl(0U);

    return 0U;
}

/**
 * @brief Send an ACK or NACK bit after reading one byte.
 */
static void BSP_TouchPort_Ack(uint8_t ack)
{
    BSP_TouchPort_WriteScl(0U);
    BSP_TouchPort_WriteSda(ack ? 0U : 1U);
    BSP_TouchPort_Delay();
    BSP_TouchPort_WriteScl(1U);
    BSP_TouchPort_Delay();
    BSP_TouchPort_WriteScl(0U);
    BSP_TouchPort_WriteSda(1U);
}

/**
 * @brief Write one byte on the software I2C bus.
 */
static void BSP_TouchPort_SendByte(uint8_t data)
{
    uint8_t i;

    for (i = 0U; i < 8U; i++)
    {
        BSP_TouchPort_WriteSda((data & 0x80U) ? 1U : 0U);
        data <<= 1;
        BSP_TouchPort_Delay();
        BSP_TouchPort_WriteScl(1U);
        BSP_TouchPort_Delay();
        BSP_TouchPort_WriteScl(0U);
    }
    BSP_TouchPort_WriteSda(1U);
}

/**
 * @brief Read one byte from the software I2C bus.
 */
static uint8_t BSP_TouchPort_ReadByte(uint8_t ack)
{
    uint8_t i;
    uint8_t data = 0U;

    BSP_TouchPort_WriteSda(1U);
    for (i = 0U; i < 8U; i++)
    {
        data <<= 1;
        BSP_TouchPort_WriteScl(0U);
        BSP_TouchPort_Delay();
        BSP_TouchPort_WriteScl(1U);
        BSP_TouchPort_Delay();
        if (BSP_TouchPort_ReadSda() != 0U)
        {
            data++;
        }
    }
    BSP_TouchPort_WriteScl(0U);
    BSP_TouchPort_Ack(ack);

    return data;
}

/**
 * @brief Initialize the GPIO resources used by the software I2C touch port.
 */
BSP_TouchPortResult_t BSP_TouchPort_Init(void)
{
    HAL_DisplayTouchMspInit();
    BSP_TouchPort_Recover();
    s_touch_port_last_error = 0U;

    return BSP_TOUCH_PORT_OK;
}

/**
 * @brief Drive the touch reset line high or low.
 */
void BSP_TouchPort_WriteReset(uint8_t level)
{
    HAL_GPIO_WritePin(BSP_TOUCH_RST_PORT,
                      BSP_TOUCH_RST_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Drive the touch INT line during GT911 address selection.
 */
void BSP_TouchPort_SetIntOutput(uint8_t level)
{
    HAL_DisplayTouchSetIntOutput(level);
}

/**
 * @brief Restore the touch INT line to input mode.
 */
void BSP_TouchPort_SetIntInput(void)
{
    HAL_DisplayTouchSetIntInput();
}

/**
 * @brief Read the current touch INT line level.
 */
uint8_t BSP_TouchPort_ReadInt(void)
{
    return (uint8_t)(HAL_GPIO_ReadPin(BSP_TOUCH_INT_PORT,
                                      BSP_TOUCH_INT_PIN) == GPIO_PIN_SET);
}

/**
 * @brief Read the current touch SCL line level.
 */
uint8_t BSP_TouchPort_ReadScl(void)
{
    return (uint8_t)(HAL_GPIO_ReadPin(BSP_TOUCH_SCL_PORT,
                                      BSP_TOUCH_SCL_PIN) == GPIO_PIN_SET);
}

/**
 * @brief Read the current touch SDA line level.
 */
uint8_t BSP_TouchPort_ReadSda(void)
{
    return (uint8_t)(HAL_GPIO_ReadPin(BSP_TOUCH_SDA_PORT,
                                      BSP_TOUCH_SDA_PIN) == GPIO_PIN_SET);
}

/**
 * @brief Recover a stuck software I2C bus by clocking SCL.
 */
void BSP_TouchPort_Recover(void)
{
    uint8_t i;

    BSP_TouchPort_WriteSda(1U);
    BSP_TouchPort_Delay();
    for (i = 0U; i < 9U; i++)
    {
        BSP_TouchPort_WriteScl(1U);
        BSP_TouchPort_Delay();
        BSP_TouchPort_WriteScl(0U);
        BSP_TouchPort_Delay();
    }
    BSP_TouchPort_Stop();
}

/**
 * @brief Write bytes to an 8-bit addressed touch register.
 */
BSP_TouchPortResult_t BSP_TouchPort_WriteReg8(
    uint8_t addr,
    uint8_t reg,
    const uint8_t *buf,
    uint8_t len)
{
    uint8_t i;

    if ((buf == 0) && (len != 0U))
    {
        return BSP_TouchPort_Fail(0x01U);
    }

    BSP_TouchPort_Start();
    BSP_TouchPort_SendByte((uint8_t)((addr << 1) | 0U));
    if (BSP_TouchPort_WaitAck() != 0U)
    {
        return BSP_TouchPort_Fail(0x11U);
    }
    BSP_TouchPort_SendByte(reg);
    if (BSP_TouchPort_WaitAck() != 0U)
    {
        return BSP_TouchPort_Fail(0x12U);
    }
    for (i = 0U; i < len; i++)
    {
        BSP_TouchPort_SendByte(buf[i]);
        if (BSP_TouchPort_WaitAck() != 0U)
        {
            return BSP_TouchPort_Fail(0x13U);
        }
    }
    BSP_TouchPort_Stop();
    s_touch_port_last_error = 0U;

    return BSP_TOUCH_PORT_OK;
}

/**
 * @brief Read bytes from an 8-bit addressed touch register.
 */
BSP_TouchPortResult_t BSP_TouchPort_ReadReg8(
    uint8_t addr,
    uint8_t reg,
    uint8_t *buf,
    uint8_t len)
{
    uint8_t i;

    if ((buf == 0) || (len == 0U))
    {
        return BSP_TouchPort_Fail(0x02U);
    }

    BSP_TouchPort_Start();
    BSP_TouchPort_SendByte((uint8_t)((addr << 1) | 0U));
    if (BSP_TouchPort_WaitAck() != 0U)
    {
        return BSP_TouchPort_Fail(0x21U);
    }
    BSP_TouchPort_SendByte(reg);
    if (BSP_TouchPort_WaitAck() != 0U)
    {
        return BSP_TouchPort_Fail(0x22U);
    }
    BSP_TouchPort_Start();
    BSP_TouchPort_SendByte((uint8_t)((addr << 1) | 1U));
    if (BSP_TouchPort_WaitAck() != 0U)
    {
        return BSP_TouchPort_Fail(0x23U);
    }
    for (i = 0U; i < len; i++)
    {
        buf[i] = BSP_TouchPort_ReadByte((uint8_t)(i + 1U < len));
    }
    BSP_TouchPort_Stop();
    s_touch_port_last_error = 0U;

    return BSP_TOUCH_PORT_OK;
}

/**
 * @brief Write bytes to a 16-bit addressed touch register.
 */
BSP_TouchPortResult_t BSP_TouchPort_WriteReg16(
    uint8_t addr,
    uint16_t reg,
    const uint8_t *buf,
    uint8_t len)
{
    uint8_t i;

    if ((buf == 0) && (len != 0U))
    {
        return BSP_TouchPort_Fail(0x03U);
    }

    BSP_TouchPort_Start();
    BSP_TouchPort_SendByte((uint8_t)((addr << 1) | 0U));
    if (BSP_TouchPort_WaitAck() != 0U)
    {
        return BSP_TouchPort_Fail(0x31U);
    }
    BSP_TouchPort_SendByte((uint8_t)(reg >> 8));
    if (BSP_TouchPort_WaitAck() != 0U)
    {
        return BSP_TouchPort_Fail(0x32U);
    }
    BSP_TouchPort_SendByte((uint8_t)(reg & 0xFFU));
    if (BSP_TouchPort_WaitAck() != 0U)
    {
        return BSP_TouchPort_Fail(0x33U);
    }
    for (i = 0U; i < len; i++)
    {
        BSP_TouchPort_SendByte(buf[i]);
        if (BSP_TouchPort_WaitAck() != 0U)
        {
            return BSP_TouchPort_Fail(0x34U);
        }
    }
    BSP_TouchPort_Stop();
    s_touch_port_last_error = 0U;

    return BSP_TOUCH_PORT_OK;
}

/**
 * @brief Read bytes from a 16-bit addressed touch register.
 */
BSP_TouchPortResult_t BSP_TouchPort_ReadReg16(
    uint8_t addr,
    uint16_t reg,
    uint8_t *buf,
    uint8_t len)
{
    uint8_t i;

    if ((buf == 0) || (len == 0U))
    {
        return BSP_TouchPort_Fail(0x04U);
    }

    BSP_TouchPort_Start();
    BSP_TouchPort_SendByte((uint8_t)((addr << 1) | 0U));
    if (BSP_TouchPort_WaitAck() != 0U)
    {
        return BSP_TouchPort_Fail(0x41U);
    }
    BSP_TouchPort_SendByte((uint8_t)(reg >> 8));
    if (BSP_TouchPort_WaitAck() != 0U)
    {
        return BSP_TouchPort_Fail(0x42U);
    }
    BSP_TouchPort_SendByte((uint8_t)(reg & 0xFFU));
    if (BSP_TouchPort_WaitAck() != 0U)
    {
        return BSP_TouchPort_Fail(0x43U);
    }
    BSP_TouchPort_Start();
    BSP_TouchPort_SendByte((uint8_t)((addr << 1) | 1U));
    if (BSP_TouchPort_WaitAck() != 0U)
    {
        return BSP_TouchPort_Fail(0x44U);
    }
    for (i = 0U; i < len; i++)
    {
        buf[i] = BSP_TouchPort_ReadByte((uint8_t)(i + 1U < len));
    }
    BSP_TouchPort_Stop();
    s_touch_port_last_error = 0U;

    return BSP_TOUCH_PORT_OK;
}

/**
 * @brief Return the last low-level software I2C diagnostic error code.
 */
uint8_t BSP_TouchPort_GetLastError(void)
{
    return s_touch_port_last_error;
}
