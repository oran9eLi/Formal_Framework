/**
 * @file px4lite_bmi088_uplink.h
 * @brief BMI088原始寄存器采样及解析六轴的独立树莓派上行契约。
 * @details Sensor经Adapter仅发布完整同次快照；Comm冻结配对并逐字段编码。
 * 不访问BSP/Driver、不消费估计器FIFO、不在运行期分配内存。
 */
#ifndef PX4LITE_BMI088_UPLINK_H
#define PX4LITE_BMI088_UPLINK_H

#include "px4lite_types.h"

#define PX4LITE_BMI088_UPLINK_RAW_TYPE 0x8030U /**< 原始寄存器采样，本地TUNNEL类型。 */
#define PX4LITE_BMI088_UPLINK_PARSED_TYPE 0x8031U /**< 同次解析六轴，本地TUNNEL类型。 */
#define PX4LITE_BMI088_UPLINK_STATUS_TYPE 0x8032U /**< 链路/设备事实，本地TUNNEL类型。 */
#define PX4LITE_BMI088_UPLINK_PAYLOAD_SIZE 64U /**< 三种内层载荷均64字节。 */

/** @brief 已验证量程的同次BMI088事实；原始芯片轴与解析机体系各自保留。 */
typedef struct {
  uint32_t sample_sequence;    /**< Driver完整采样序号，重启后从头开始。 */
  uint32_t sample_time_ms;     /**< Driver本轮服务入口时刻，真实ms，0合法。 */
  uint32_t reinit_count;       /**< Driver初始化尝试次数，含首次。 */
  uint32_t driver_error_count; /**< Driver累计真实错误次数。 */
  uint32_t fifo_drop_count;    /**< Driver因积压/溢出丢弃陀螺FIFO的次数。 */
  int32_t accel_mg[3];         /**< 机体系X/Y/Z加速度，单位mg。 */
  int32_t gyro_mdps[3];        /**< 机体系X/Y/Z角速度，单位mdps，未扣估计器零偏。 */
  int16_t temperature_cdeg;    /**< 器件温度，单位0.01℃。 */
  uint16_t sample_period_us;   /**< 相邻发布周期，单位us。 */
  uint8_t bus;                /**< 1=I2C，2=SPI。 */
  uint8_t accel_addr_7bit;     /**< 加速度7bit I2C地址；SPI为0。 */
  uint8_t gyro_addr_7bit;      /**< 陀螺7bit I2C地址；SPI为0。 */
  uint8_t accel_chip_id;       /**< 加速度ID，已验证应为0x1E。 */
  uint8_t gyro_chip_id;        /**< 陀螺ID，已验证应为0x0F。 */
  uint8_t accel_status;        /**< 本轮读取ACC_STATUS原字节。 */
  uint8_t gyro_fifo_status;    /**< 本轮读取GYRO_FIFO_STATUS原字节。 */
  uint8_t gyro_frame_count;    /**< 本轮FIFO真实帧数1..4，解析采用最后帧。 */
  uint8_t raw_accel[9];        /**< ACC 0x12起9字节，含三轴及24bit sensor_time。 */
  uint8_t raw_temperature[2];  /**< ACC 0x22起两字节。 */
  uint8_t raw_gyro[24];        /**< GYR 0x3F读出的完整FIFO，使用帧数外字节为0。 */
  int8_t axis_map[3];          /**< 有符号芯片轴号映射到机体系X/Y/Z，取±1..±3。 */
} Px4Lite_Bmi088Sample_t;

/** @brief 一次Comm待发视图；指针只在下一次Peek/Result/Init前有效。 */
typedef struct {
  const uint8_t *payload; /**< 已逐字段编码的小端载荷，不含MAVLink外层。 */
  uint16_t payload_type;  /**< 0x8030、0x8031或0x8032。 */
  uint8_t length;         /**< 固定64字节。 */
} Px4Lite_Bmi088UplinkView_t;

/** @brief Sensor所有者复制完整同次样本到单槽邮箱；不解析、不编码、不发送。 */
void Px4Lite_Bmi088UplinkPublish(const Px4Lite_Bmi088Sample_t *sample);
/** @brief Sensor撤销当前源并换代，使Comm旧冻结配对失去授权；无硬件IO。 */
void Px4Lite_Bmi088UplinkInvalidate(void);
/** @brief 调度前或Comm生命周期初始化，仅复位Comm状态，不清并发Sensor邮箱。 */
void Px4Lite_Bmi088UplinkInit(uint32_t now_ms);
/** @brief Comm按配置抽样并借用下一包，重试重新计算真实年龄，不追赶100Hz历史。 */
Px4Lite_Result_t Px4Lite_Bmi088UplinkPeek(uint32_t now_ms, Px4Lite_Bmi088UplinkView_t *out);
/** @brief Comm记录实际UART提交结果；成功不代表树莓派收到，失败有界重试。 */
void Px4Lite_Bmi088UplinkResult(Px4Lite_Result_t result, uint32_t now_ms);

#endif
