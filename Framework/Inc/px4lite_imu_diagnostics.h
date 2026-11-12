/**
 * @file px4lite_imu_diagnostics.h
 * @brief 平台适配器发布的通用IMU只读诊断，不暴露BSP/Driver私有类型。
 * @details sensor为唯一写者，Debug只复制缓存；读取不采样、不恢复、不打印。
 */
#ifndef PX4LITE_IMU_DIAGNOSTICS_H
#define PX4LITE_IMU_DIAGNOSTICS_H

#include <stdint.h>

/** @brief 当前选定IMU的设备事实，不代替Health最终状态及样本时效判断。 */
typedef struct {
  uint32_t rx_sequence;       /**< 最近完整样本序号；0表示尚无样本。 */
  uint32_t sample_time_ms;    /**< 最近完整样本的真实毫秒时间。 */
  uint32_t error_count;       /**< 驱动累计错误次数。 */
  uint32_t reinit_count;      /**< 初始化尝试次数，含首次。 */
  uint32_t fifo_drop_count;   /**< BMI088陀螺FIFO积压/溢出丢弃次数，MPU为0。 */
  uint8_t driver;             /**< 0=MPU6050，1=BMI088。 */
  uint8_t bus;                /**< 1=I2C，2=SPI。 */
  uint8_t accel_chip_id;      /**< MPU为WHO_AM_I；BMI加速度期望0x1E。 */
  uint8_t gyro_chip_id;       /**< MPU同一器件ID；BMI陀螺期望0x0F。 */
  uint8_t accel_addr_7bit;    /**< 加速度7bit I2C地址；SPI为0。 */
  uint8_t gyro_addr_7bit;     /**< 陀螺7bit I2C地址；SPI为0。 */
  uint8_t last_stage;         /**< 最近操作阶段，取值按driver对应驱动枚举。 */
  uint8_t last_result;        /**< 最近操作结果，取值按driver对应驱动枚举。 */
  uint8_t last_bsp_status;    /**< 最近BSP原始访问结果码。 */
  uint8_t device_ready;       /**< BMI双ID/配置通过；MPU最近操作成功；不代表样本新鲜。 */
} Px4Lite_ImuDiagnostics_t;

#endif
