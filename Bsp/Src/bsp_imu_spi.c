/**
 * @file bsp_imu_spi.c
 * @brief IMU 专用 SPI2 初始化、双片选与同步原始事务。
 * @details 属于 BSP 层，依赖 bsp_config.h 和 HAL SPI/GPIO；由 sensor 任务独占。
 *          不解析传感器协议，不重置 SD SPI3，不使用 DMA 或全局中断屏蔽。
 */
#include "bsp_imu_spi.h"
#include "bsp_config.h"

static SPI_HandleTypeDef s_imu_spi;
static uint8_t s_imu_spi_initialized;

/** @brief 将 HAL 原始结果映射为公共 BSP 状态，不隐藏超时或忙。 */
static BSP_Status_t BSP_IMU_SPI_MapStatus(HAL_StatusTypeDef status)
{
  if (status == HAL_OK) { return BSP_STATUS_OK; }
  if (status == HAL_BUSY) { return BSP_STATUS_BUSY; }
  if (status == HAL_TIMEOUT) { return BSP_STATUS_TIMEOUT; }
  return BSP_STATUS_ERROR;
}

/** @brief 释放两个片选，仅在本模块完成 GPIO 初始化后调用。 */
static void BSP_IMU_SPI_ReleaseAll(void)
{
  HAL_GPIO_WritePin(BSP_IMU_SPI_ACCEL_CS_PORT, BSP_IMU_SPI_ACCEL_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BSP_IMU_SPI_GYRO_CS_PORT, BSP_IMU_SPI_GYRO_CS_PIN, GPIO_PIN_SET);
}

/** @brief 初始化专用 SPI2：模式 3、8 bit、MSB、软件片选、APB1/32。 */
BSP_Status_t MX_IMU_SPI_Init(void)
{
  HAL_StatusTypeDef status;

  if (s_imu_spi_initialized != 0U) { return BSP_STATUS_OK; }
  s_imu_spi.Instance = BSP_IMU_SPI_INS;
  s_imu_spi.Init.Mode = SPI_MODE_MASTER;
  s_imu_spi.Init.Direction = SPI_DIRECTION_2LINES;
  s_imu_spi.Init.DataSize = SPI_DATASIZE_8BIT;
  s_imu_spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
  s_imu_spi.Init.CLKPhase = SPI_PHASE_2EDGE;
  s_imu_spi.Init.NSS = SPI_NSS_SOFT;
  s_imu_spi.Init.BaudRatePrescaler = BSP_IMU_SPI_PRESCALER;
  s_imu_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  s_imu_spi.Init.TIMode = SPI_TIMODE_DISABLE;
  s_imu_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  s_imu_spi.Init.CRCPolynomial = 7U;
  status = HAL_SPI_Init(&s_imu_spi);
  /* HAL MSP 已将 CS 置高；失败也保持不选中任何器件。 */
  BSP_IMU_SPI_ReleaseAll();
  if (status != HAL_OK) { return BSP_IMU_SPI_MapStatus(status); }
  s_imu_spi_initialized = 1U;
  return BSP_STATUS_OK;
}

/** @brief 统一板级初始化入口，不在 I2C 使用时主动占用 SPI GPIO。 */
BSP_Status_t BSP_IMU_SPI_Init(void)
{
  return MX_IMU_SPI_Init();
}

/** @brief 由资源所属任务执行 SPI2 恢复，保留真实 DeInit 失败结果。 */
BSP_Status_t BSP_IMU_SPI_Recover(void)
{
  HAL_StatusTypeDef status;

  if (s_imu_spi_initialized == 0U) { return BSP_IMU_SPI_Init(); }
  BSP_IMU_SPI_ReleaseAll();
  status = HAL_SPI_DeInit(&s_imu_spi);
  s_imu_spi_initialized = 0U;
  if (status != HAL_OK) { return BSP_IMU_SPI_MapStatus(status); }
  return BSP_IMU_SPI_Init();
}

/** @brief 在严格有界的单一 CS 事务中原样收发，不处理寄存器协议。 */
BSP_Status_t BSP_IMU_SPI_Transfer(uint8_t device, const uint8_t *tx,
                               uint8_t *rx, uint16_t length,
                               uint32_t timeout_ms)
{
  BSP_Status_t result;
  HAL_StatusTypeDef status;
  GPIO_TypeDef *port;
  uint16_t pin;

  if ((device > BSP_IMU_SPI_DEVICE_GYRO) || (tx == 0) || (rx == 0) ||
      (length == 0U) || (length > BSP_IMU_SPI_MAX_TRANSFER) ||
      (timeout_ms == 0U) || (timeout_ms > BSP_IMU_SPI_MAX_TIMEOUT_MS)) { return BSP_STATUS_ERROR; }
  result = BSP_IMU_SPI_Init();
  if (result != BSP_STATUS_OK) { return result; }
  port = (device == BSP_IMU_SPI_DEVICE_ACCEL) ? BSP_IMU_SPI_ACCEL_CS_PORT : BSP_IMU_SPI_GYRO_CS_PORT;
  pin = (device == BSP_IMU_SPI_DEVICE_ACCEL) ? BSP_IMU_SPI_ACCEL_CS_PIN : BSP_IMU_SPI_GYRO_CS_PIN;
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(&s_imu_spi, (uint8_t *)tx, rx, length, timeout_ms);
  BSP_IMU_SPI_ReleaseAll();
  return BSP_IMU_SPI_MapStatus(status);
}
