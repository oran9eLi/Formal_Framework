/**
 * @file test_bmi088_spi_bus.c
 * @brief 以 HAL 替身验证 SPI2 独占配置、双片选、失败释放及参数边界。
 * @details 主机测试直接编译生产 BSP 实现；不模拟 BMI088 协议解析。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* 本测试替换 HAL/板级常量，不包含 MCU 寄存器定义。 */
#define BSP_CONFIG_H
typedef struct { uint8_t unused; } GPIO_TypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct {
  uint32_t Mode, Direction, DataSize, CLKPolarity, CLKPhase, NSS;
  uint32_t BaudRatePrescaler, FirstBit, TIMode, CRCCalculation, CRCPolynomial;
} SPI_InitTypeDef;
typedef struct { void *Instance; SPI_InitTypeDef Init; } SPI_HandleTypeDef;
#define SPI_MODE_MASTER 1U
#define SPI_DIRECTION_2LINES 2U
#define SPI_DATASIZE_8BIT 8U
#define SPI_POLARITY_HIGH 3U
#define SPI_PHASE_2EDGE 4U
#define SPI_NSS_SOFT 5U
#define SPI_FIRSTBIT_MSB 6U
#define SPI_TIMODE_DISABLE 0U
#define SPI_CRCCALCULATION_DISABLE 0U
static int test_spi2;
static GPIO_TypeDef test_port;
#define BSP_IMU_SPI_INS ((void *)&test_spi2)
#define BSP_IMU_SPI_PRESCALER 32U
#define BSP_IMU_SPI_ACCEL_CS_PORT (&test_port)
#define BSP_IMU_SPI_GYRO_CS_PORT (&test_port)
#define BSP_IMU_SPI_ACCEL_CS_PIN 16U
#define BSP_IMU_SPI_GYRO_CS_PIN 32U

static uint8_t test_accel_high = 1U, test_gyro_high = 1U;
static uint32_t test_inits, test_deinits, test_transfers, test_gpio_writes;
static uint8_t test_selected_device;
static HAL_StatusTypeDef test_next_transfer_status = HAL_OK;
static HAL_StatusTypeDef test_next_init_status = HAL_OK;
static HAL_StatusTypeDef test_next_deinit_status = HAL_OK;

/** @brief 校验生产初始化的模式，令不正确配置在主机测试时失败。 */
static HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *spi)
{
  assert(spi->Instance == BSP_IMU_SPI_INS);
  assert(spi->Init.Mode == SPI_MODE_MASTER);
  assert(spi->Init.Direction == SPI_DIRECTION_2LINES);
  assert(spi->Init.DataSize == SPI_DATASIZE_8BIT);
  assert(spi->Init.CLKPolarity == SPI_POLARITY_HIGH);
  assert(spi->Init.CLKPhase == SPI_PHASE_2EDGE);
  assert(spi->Init.NSS == SPI_NSS_SOFT);
  assert(spi->Init.BaudRatePrescaler == BSP_IMU_SPI_PRESCALER);
  assert(spi->Init.FirstBit == SPI_FIRSTBIT_MSB);
  assert(spi->Init.TIMode == SPI_TIMODE_DISABLE);
  assert(spi->Init.CRCCalculation == SPI_CRCCALCULATION_DISABLE);
  test_inits++;
  return test_next_init_status;
}

/** @brief 记录恢复中的外设释放，不触碰真实硬件。 */
static HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *spi)
{
  assert(spi->Instance == BSP_IMU_SPI_INS);
  assert(test_accel_high && test_gyro_high);
  test_deinits++;
  return test_next_deinit_status;
}

/** @brief 验证任何时刻均不得同时选中两个器件。 */
static void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState level)
{
  assert(port == &test_port);
  test_gpio_writes++;
  if (pin == BSP_IMU_SPI_ACCEL_CS_PIN) { test_accel_high = (uint8_t)level; }
  else { assert(pin == BSP_IMU_SPI_GYRO_CS_PIN); test_gyro_high = (uint8_t)level; }
  assert(test_accel_high || test_gyro_high);
}

/** @brief 事务替身验证片选连续有效，并将原始数据完整返回给调用者。 */
static HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *tx,
                                               uint8_t *rx, uint16_t length,
                                               uint32_t timeout_ms)
{
  assert(spi->Instance == BSP_IMU_SPI_INS);
  assert(timeout_ms == 4U);
  assert((test_accel_high == 0U) != (test_gyro_high == 0U));
  test_selected_device = (test_accel_high == 0U) ? 0U : 1U;
  memcpy(rx, tx, length);
  test_transfers++;
  return test_next_transfer_status;
}

#include "../../Bsp/Src/bsp_imu_spi.c"

/** @brief 验证参数拒绝不碰硬件、成功/失败片选释放和恢复行为。 */
int main(void)
{
  uint8_t tx[33] = {0x80U, 0x5AU, 0xC3U};
  uint8_t rx[33] = {0};
  assert(BSP_IMU_SPI_Transfer(2U, tx, rx, 3U, 4U) == BSP_STATUS_ERROR);
  assert(BSP_IMU_SPI_Transfer(0U, 0, rx, 3U, 4U) == BSP_STATUS_ERROR);
  assert(BSP_IMU_SPI_Transfer(0U, tx, 0, 3U, 4U) == BSP_STATUS_ERROR);
  assert(BSP_IMU_SPI_Transfer(0U, tx, rx, 0U, 4U) == BSP_STATUS_ERROR);
  assert(BSP_IMU_SPI_Transfer(0U, tx, rx, 33U, 4U) == BSP_STATUS_ERROR);
  assert(BSP_IMU_SPI_Transfer(0U, tx, rx, 3U, 0U) == BSP_STATUS_ERROR);
  assert(BSP_IMU_SPI_Transfer(0U, tx, rx, 3U, 21U) == BSP_STATUS_ERROR);
  assert(test_inits == 0U && test_gpio_writes == 0U);
  assert(BSP_IMU_SPI_Transfer(0U, tx, rx, 3U, 4U) == BSP_STATUS_OK);
  assert(test_inits == 1U && test_transfers == 1U && test_selected_device == 0U);
  assert(memcmp(tx, rx, 3U) == 0 && test_accel_high && test_gyro_high);
  assert(BSP_IMU_SPI_Transfer(1U, tx, rx, 32U, 4U) == BSP_STATUS_OK);
  assert(test_inits == 1U && test_selected_device == 1U);
  test_next_transfer_status = HAL_TIMEOUT;
  assert(BSP_IMU_SPI_Transfer(0U, tx, rx, 3U, 4U) == BSP_STATUS_TIMEOUT);
  assert(test_accel_high && test_gyro_high);
  test_next_transfer_status = HAL_BUSY;
  assert(BSP_IMU_SPI_Transfer(1U, tx, rx, 3U, 4U) == BSP_STATUS_BUSY);
  assert(test_accel_high && test_gyro_high);
  test_next_transfer_status = HAL_ERROR;
  assert(BSP_IMU_SPI_Transfer(1U, tx, rx, 3U, 4U) == BSP_STATUS_ERROR);
  assert(test_accel_high && test_gyro_high);
  assert(BSP_IMU_SPI_Recover() == BSP_STATUS_OK);
  assert(test_deinits == 1U && test_inits == 2U);
  test_next_deinit_status = HAL_ERROR;
  assert(BSP_IMU_SPI_Recover() == BSP_STATUS_ERROR);
  assert(test_accel_high && test_gyro_high);
  test_next_init_status = HAL_ERROR;
  assert(BSP_IMU_SPI_Transfer(0U, tx, rx, 3U, 4U) == BSP_STATUS_ERROR);
  assert(test_accel_high && test_gyro_high);
  test_next_init_status = HAL_OK;
  assert(BSP_IMU_SPI_Recover() == BSP_STATUS_OK);
  puts("BMI088 SPI bus tests passed");
  return 0;
}
