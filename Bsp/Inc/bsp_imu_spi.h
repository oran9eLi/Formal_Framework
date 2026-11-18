/**
 * @file bsp_imu_spi.h
 * @brief IMU 专用 SPI2 板级原始收发接口。
 * @details 属于 BSP 层，依赖板级 GPIO/时钟和 HAL SPI；仅由 sensor 任务调用。
 *          本层负责片选与完整事务，不解释 BMI088 寄存器、读写位或 dummy 字节。
 */
#ifndef BSP_IMU_SPI_H
#define BSP_IMU_SPI_H

#include <stdint.h>
#include "bsp_status.h"

/** @brief 加速度器件片选编号；不是芯片寄存器地址。 */
#define BSP_IMU_SPI_DEVICE_ACCEL 0U
/** @brief 陀螺仪器件片选编号；不是芯片寄存器地址。 */
#define BSP_IMU_SPI_DEVICE_GYRO 1U
/** @brief 单次同步事务最大字节数，包含地址及协议填充字节。 */
#define BSP_IMU_SPI_MAX_TRANSFER 32U
/** @brief 单次同步事务允许的最大超时，单位 ms，拒绝无限等待。 */
#define BSP_IMU_SPI_MAX_TIMEOUT_MS 20U

/**
 * @brief 按 MX 风格初始化 IMU 专用 SPI2，重复调用不改变已初始化资源。
 * @return 板级初始化结果。
 * @note 仅被选中 SPI 接口时调用；不申请 DMA、中断或动态内存。
 */
BSP_Status_t MX_IMU_SPI_Init(void);

/** @brief 懒初始化 IMU SPI2；返回成功或 HAL 映射错误。 */
BSP_Status_t BSP_IMU_SPI_Init(void);

/**
 * @brief 在 sensor 任务中释放片选并重新初始化专用 SPI2。
 * @return 重新初始化结果；不触碰内置 SD 使用的 SPI3。
 */
BSP_Status_t BSP_IMU_SPI_Recover(void);

/**
 * @brief 在一个片选有效期间完成一次原始全双工收发。
 * @param device 器件编号：0=加速度，1=陀螺。
 * @param tx 发送缓冲，至少 length 字节，不得为空。
 * @param rx 接收缓冲，至少 length 字节，不得为空。
 * @param length 事务字节数，范围 1..BSP_IMU_SPI_MAX_TRANSFER。
 * @param timeout_ms 同步传输超时，单位 ms，范围 1..BSP_IMU_SPI_MAX_TIMEOUT_MS。
 * @return HAL 状态对应的 BSP 返回值；传输失败也先释放片选再返回。
 * @note 单一 sensor 任务所有；禁止 ISR 调用，不保存调用方缓冲指针。
 */
BSP_Status_t BSP_IMU_SPI_Transfer(uint8_t device, const uint8_t *tx,
                               uint8_t *rx, uint16_t length,
                               uint32_t timeout_ms);

#endif
