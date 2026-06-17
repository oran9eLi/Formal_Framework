/**
 * @file bsp_i2c.c
 * @brief Implement board I2C access helpers.
 */

#include "bsp_i2c.h"

#include "bsp_config.h"
#include "stm32f4xx_hal.h"

static I2C_HandleTypeDef s_hi2c;
static uint8_t s_initialized;

static BSP_Status_t BSP_I2C_MapHalStatus(HAL_StatusTypeDef status)
{
    if (status == HAL_OK)
    {
        return BSP_STATUS_OK;
    }
    if (status == HAL_BUSY)
    {
        return BSP_STATUS_BUSY;
    }
    if (status == HAL_TIMEOUT)
    {
        return BSP_STATUS_TIMEOUT;
    }
    return BSP_STATUS_ERROR;
}

BSP_Status_t BSP_I2C_Init(void)
{
    if (s_initialized != 0U)
    {
        return BSP_STATUS_OK;
    }

    s_hi2c.Instance = BSP_I2C_INS;
    s_hi2c.Init.ClockSpeed = BSP_I2C_SPEED;
    s_hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    s_hi2c.Init.OwnAddress1 = 0U;
    s_hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    s_hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    s_hi2c.Init.OwnAddress2 = 0U;
    s_hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    s_hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&s_hi2c) != HAL_OK)
    {
        return BSP_STATUS_ERROR;
    }

    s_initialized = 1U;
    return BSP_STATUS_OK;
}

#define BSP_I2C_RECOVER_CLOCKS 9U

/*
 * Short open-loop delay (a few microseconds) used only by bus recovery.
 */
static void BSP_I2C_DelayShort(void)
{
    volatile uint32_t i;

    for (i = 0U; i < 400U; i++)
    {
        __NOP();
    }
}

/*
 * Release a hung I2C bus. Drive SCL/SDA as open-drain GPIO, issue nine clock
 * pulses to flush a slave still holding SDA low, generate a STOP, then
 * re-initialize the peripheral so the next transfer starts from a clean state.
 */
static void BSP_I2C_BusRecover(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t i;

    HAL_I2C_DeInit(&s_hi2c);

    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = 0U;

    gpio.Pin = BSP_I2C_SCL_PIN;
    HAL_GPIO_Init(BSP_I2C_SCL_PORT, &gpio);
    gpio.Pin = BSP_I2C_SDA_PIN;
    HAL_GPIO_Init(BSP_I2C_SDA_PORT, &gpio);

    HAL_GPIO_WritePin(BSP_I2C_SDA_PORT, BSP_I2C_SDA_PIN, GPIO_PIN_SET);

    for (i = 0U; i < BSP_I2C_RECOVER_CLOCKS; i++)
    {
        HAL_GPIO_WritePin(BSP_I2C_SCL_PORT, BSP_I2C_SCL_PIN, GPIO_PIN_RESET);
        BSP_I2C_DelayShort();
        HAL_GPIO_WritePin(BSP_I2C_SCL_PORT, BSP_I2C_SCL_PIN, GPIO_PIN_SET);
        BSP_I2C_DelayShort();
    }

    /* STOP condition: SDA goes low to high while SCL is high. */
    HAL_GPIO_WritePin(BSP_I2C_SDA_PORT, BSP_I2C_SDA_PIN, GPIO_PIN_RESET);
    BSP_I2C_DelayShort();
    HAL_GPIO_WritePin(BSP_I2C_SCL_PORT, BSP_I2C_SCL_PIN, GPIO_PIN_SET);
    BSP_I2C_DelayShort();
    HAL_GPIO_WritePin(BSP_I2C_SDA_PORT, BSP_I2C_SDA_PIN, GPIO_PIN_SET);
    BSP_I2C_DelayShort();

    s_initialized = 0U;
    BSP_I2C_Init();
}

/*
 * Public bus recovery. Flush a slave still holding SDA low and re-initialize
 * the peripheral so a stuck bus can be released from outside this file.
 */
BSP_Status_t BSP_I2C_Recover(void)
{
    BSP_I2C_BusRecover();
    return (s_initialized != 0U) ? BSP_STATUS_OK : BSP_STATUS_ERROR;
}

/*
 * Release the I2C peripheral. After this the bus consumes no resources until
 * the next BSP_I2C_Init().
 */
BSP_Status_t BSP_I2C_DeInit(void)
{
    if (s_initialized == 0U)
    {
        return BSP_STATUS_OK;
    }

    if (HAL_I2C_DeInit(&s_hi2c) != HAL_OK)
    {
        return BSP_STATUS_ERROR;
    }

    s_initialized = 0U;
    return BSP_STATUS_OK;
}

/*
 * Map the HAL status and, on a bus-level failure, recover the bus so a single
 * stuck transfer cannot permanently block every later transfer.
 */
static BSP_Status_t BSP_I2C_Finish(HAL_StatusTypeDef status)
{
    /*
     * Only a stuck bus (software timeout or a latched BUSY flag) needs a full
     * bus recovery. A plain NACK (HAL_ERROR / acknowledge failure) is a normal
     * transient on a shared bus and must not trigger a disruptive DeInit/Init.
     */
    if ((status == HAL_TIMEOUT) || (status == HAL_BUSY))
    {
        BSP_I2C_BusRecover();
    }
    //---------------------------------------------------
    else if (status==HAL_ERROR)
    {
        uint32_t err = (s_hi2c.Instance != 0) ? s_hi2c.ErrorCode : 0U;
        if ((err & (HAL_I2C_ERROR_BERR |    // 总线错误
                    HAL_I2C_ERROR_ARLO |    // 仲裁丢失  
                    HAL_I2C_ERROR_OVR)) != 0U)  // 溢出
        {
            BSP_I2C_BusRecover();
        }
    }
    //---------------------------------------------------

    return BSP_I2C_MapHalStatus(status);
}

BSP_Status_t BSP_I2C_IsDeviceReady(uint16_t dev_addr,
                                   uint32_t timeout_ms)
{
    BSP_I2C_Init();
    if (s_initialized == 0U)
    {
        return BSP_STATUS_ERROR;
    }

    return BSP_I2C_Finish(
        HAL_I2C_IsDeviceReady(&s_hi2c, dev_addr, 2U, timeout_ms));
}

BSP_Status_t BSP_I2C_MemRead(uint16_t dev_addr,
                             uint16_t reg_addr,
                             uint8_t *data,
                             uint16_t len,
                             uint32_t timeout_ms)
{
    BSP_I2C_Init();
    if ((s_initialized == 0U) || (data == 0) || (len == 0U))
    {
        return BSP_STATUS_ERROR;
    }

    return BSP_I2C_Finish(
        HAL_I2C_Mem_Read(&s_hi2c,
                         dev_addr,
                         reg_addr,
                         I2C_MEMADD_SIZE_8BIT,
                         data,
                         len,
                         timeout_ms));
}

BSP_Status_t BSP_I2C_MemWrite(uint16_t dev_addr,
                              uint16_t reg_addr,
                              const uint8_t *data,
                              uint16_t len,
                              uint32_t timeout_ms)
{
    BSP_I2C_Init();
    if ((s_initialized == 0U) || (data == 0) || (len == 0U))
    {
        return BSP_STATUS_ERROR;
    }

    return BSP_I2C_Finish(
        HAL_I2C_Mem_Write(&s_hi2c,
                          dev_addr,
                          reg_addr,
                          I2C_MEMADD_SIZE_8BIT,
                          (uint8_t *)data,
                          len,
                          timeout_ms));
}
