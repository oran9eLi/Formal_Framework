/**
 * @file bsp_touch_port.c
 * @brief 实现 GT911 触摸驱动使用的软件 I2C GPIO 端口接口。
 */

#include "bsp_touch_port.h"

#include "bsp_config.h"
#include "stm32f4xx_hal.h"

void HAL_DisplayTouchMspInit(void);
void HAL_DisplayTouchSetIntOutput(uint8_t level);
void HAL_DisplayTouchSetIntInput(void);

static uint8_t s_touch_port_last_error;

/**
 * @brief 使用短 CPU 循环延时一个软件 I2C 半周期。
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
 * @brief 驱动软件 I2C SCL 引脚。
 */
static void BSP_TouchPort_WriteScl(uint8_t level)
{
    HAL_GPIO_WritePin(BSP_TOUCH_SCL_PORT,
                      BSP_TOUCH_SCL_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 驱动软件 I2C SDA 引脚。
 */
static void BSP_TouchPort_WriteSda(uint8_t level)
{
    HAL_GPIO_WritePin(BSP_TOUCH_SDA_PORT,
                      BSP_TOUCH_SDA_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 终止一次寄存器事务并记录诊断错误码。
 */
static BSP_TouchPortResult_t BSP_TouchPort_Fail(uint8_t code)
{
    s_touch_port_last_error = code;
    return BSP_TOUCH_PORT_ERROR;
}

/**
 * @brief 产生软件 I2C START 条件。
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
 * @brief 产生软件 I2C STOP 条件。
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
 * @brief 等待软件 I2C ACK 位。
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
 * @brief 读取一个字节后发送 ACK 或 NACK。
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
 * @brief 在软件 I2C 总线上写入一个字节。
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
 * @brief 从软件 I2C 总线读取一个字节。
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
 * @brief 初始化软件 I2C 触摸端口使用的 GPIO 资源。
 */
BSP_TouchPortResult_t BSP_TouchPort_Init(void)
{
    HAL_DisplayTouchMspInit();
    BSP_TouchPort_Recover();
    s_touch_port_last_error = 0U;

    return BSP_TOUCH_PORT_OK;
}

/**
 * @brief 驱动触摸 reset 引脚高低电平。
 */
void BSP_TouchPort_WriteReset(uint8_t level)
{
    HAL_GPIO_WritePin(BSP_TOUCH_RST_PORT,
                      BSP_TOUCH_RST_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 在 GT911 地址选择阶段驱动 INT 引脚。
 */
void BSP_TouchPort_SetIntOutput(uint8_t level)
{
    HAL_DisplayTouchSetIntOutput(level);
}

/**
 * @brief 将触摸 INT 引脚恢复为输入模式。
 */
void BSP_TouchPort_SetIntInput(void)
{
    HAL_DisplayTouchSetIntInput();
}

/**
 * @brief 读取当前触摸 INT 引脚电平。
 */
uint8_t BSP_TouchPort_ReadInt(void)
{
    return (uint8_t)(HAL_GPIO_ReadPin(BSP_TOUCH_INT_PORT,
                                      BSP_TOUCH_INT_PIN) == GPIO_PIN_SET);
}

/**
 * @brief 读取当前触摸 SCL 引脚电平。
 */
uint8_t BSP_TouchPort_ReadScl(void)
{
    return (uint8_t)(HAL_GPIO_ReadPin(BSP_TOUCH_SCL_PORT,
                                      BSP_TOUCH_SCL_PIN) == GPIO_PIN_SET);
}

/**
 * @brief 读取当前触摸 SDA 引脚电平。
 */
uint8_t BSP_TouchPort_ReadSda(void)
{
    return (uint8_t)(HAL_GPIO_ReadPin(BSP_TOUCH_SDA_PORT,
                                      BSP_TOUCH_SDA_PIN) == GPIO_PIN_SET);
}

/**
 * @brief 通过 SCL 时钟脉冲恢复卡死的软件 I2C 总线。
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
 * @brief 向 8-bit 地址触摸寄存器写入字节。
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
 * @brief 从 8-bit 地址触摸寄存器读取字节。
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
 * @brief 向 16-bit 地址触摸寄存器写入字节。
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
 * @brief 从 16-bit 地址触摸寄存器读取字节。
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
 * @brief 返回最近一次底层软件 I2C 诊断错误码。
 */
uint8_t BSP_TouchPort_GetLastError(void)
{
    return s_touch_port_last_error;
}
