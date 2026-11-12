/**
 * @file sensor_bmi088.h
 * @brief BMI088双器件六轴测量、初始化阶段和恢复事实接口。
 * @details 属于Sensor Driver层，仅依赖BSP原始总线及时间；由sensor任务独占推进。
 * 不访问HAL，不判定最终OFFLINE/FAILED，不假定模块的机体安装方向。
 */
#ifndef SENSOR_BMI088_H
#define SENSOR_BMI088_H

#include <stdint.h>

/** @brief 实际接线对应的总线；软件选择不能改变模块焊接接口。 */
typedef enum {
  BMI088_BUS_I2C = 1, /**< 共享I2C，分别探测加速度与陀螺地址。 */
  BMI088_BUS_SPI = 2  /**< SPI四线，分别使用两个器件片选。 */
} Bmi088_Bus_t;

/** @brief 驱动操作结果；NO_DATA不属于通信错误。 */
typedef enum {
  BMI088_RESULT_OK = 0,       /**< 操作成功或初始化请求已接受。 */
  BMI088_RESULT_NO_DATA,      /**< 初始化等待中或尚无完整新鲜六轴样本。 */
  BMI088_RESULT_BAD_ID,       /**< 任一器件标识不匹配。 */
  BMI088_RESULT_IO_ERROR,     /**< 总线传输失败。 */
  BMI088_RESULT_TIMEOUT,      /**< 有界总线事务超时。 */
  BMI088_RESULT_INVALID_PARAM,/**< 参数错误。 */
  BMI088_RESULT_CONFIG_ERROR  /**< 寄存器读回或设备错误检查未通过。 */
} Bmi088_Result_t;

/** @brief 诊断最近操作阶段；不等同于Framework健康状态。 */
typedef enum {
  BMI088_STAGE_NONE = 0,      /**< 未执行或最近测量成功。 */
  BMI088_STAGE_BUS,           /**< 总线初始化或恢复。 */
  BMI088_STAGE_POWER_WAIT,    /**< 上电、复位或模式切换等待。 */
  BMI088_STAGE_PROBE_ACCEL,   /**< 加速度接口探测与ID检查。 */
  BMI088_STAGE_PROBE_GYRO,    /**< 陀螺接口探测与ID检查。 */
  BMI088_STAGE_RESET,         /**< 器件软复位。 */
  BMI088_STAGE_CONFIG,        /**< 量程、采样率、电源及FIFO配置。 */
  BMI088_STAGE_VERIFY,        /**< 配置读回与器件错误检查。 */
  BMI088_STAGE_DATA_READ,     /**< 新样本就绪与测量读取。 */
  BMI088_STAGE_RETRY_WAIT     /**< 出错后的限速恢复等待。 */
} Bmi088_Stage_t;

/**
 * @brief 一次成功采样轮次的真实寄存器返回字节及设备访问事实。
 * @details 字节顺序与芯片返回顺序一致，不含I2C地址、SPI命令或dummy字节；
 *          陀螺保留本轮整个FIFO突发读取，未使用的尾部显式清零。
 *          与所在快照的解析量共享序号和时间，不表示两个MEMS硬件同步采样。
 */
typedef struct {
  uint8_t accel[9];         /**< 加速度0x12..0x1A返回值，含XYZ及24bit sensor_time，单位字节。 */
  uint8_t gyro[24];         /**< FIFO端口0x3F本轮返回值，1..4帧，每帧XYZ共6字节；剩余尾部为0。 */
  uint8_t temperature[2];   /**< 加速度0x22/0x23温度返回值，保留原11bit编码与低位。 */
  uint8_t accel_status;     /**< 读取加速度前的0x03原始状态字节。 */
  uint8_t gyro_fifo_status; /**< 读取FIFO前的0x0E原始状态字节。 */
  uint8_t gyro_frame_count; /**< 本次实际读取的完整陀螺FIFO帧数，范围1..4。 */
  uint8_t bus;              /**< 实际访问总线，I2C=1、SPI=2。 */
  uint8_t accel_addr_7bit;   /**< 加速度7bit I2C地址0x18/0x19，SPI为0。 */
  uint8_t gyro_addr_7bit;    /**< 陀螺7bit I2C地址0x68/0x69，SPI为0。 */
  uint8_t accel_chip_id;    /**< 本次初始化已确认的加速度ID，期望0x1E。 */
  uint8_t gyro_chip_id;     /**< 本次初始化已确认的陀螺ID，期望0x0F。 */
} Bmi088_RawSample_t;

/** @brief 芯片原始轴向的解析量与原字节同次快照；不宣称两个MEMS硬件同步。 */
typedef struct {
  uint32_t rx_sequence;    /**< 成功发布序号，非零，重初始化不回退。 */
  uint32_t sample_time_ms; /**< 本轮采样服务入口时刻，真实毫秒。 */
  float accel_g[3];        /**< X/Y/Z加速度，单位g，固定量程±12g。 */
  float gyro_dps[3];       /**< X/Y/Z角速度，单位degree/s，固定量程±1000。 */
  float temperature_c;    /**< 加速度器件内部温度，单位℃。 */
  uint32_t error_count;    /**< 累计通信、ID或配置错误次数。 */
  uint32_t reinit_count;   /**< 本样本所属初始化尝试序号，首次初始化计入。 */
  uint32_t fifo_drop_count;/**< 生成本样本时已累计的FIFO积压/溢出丢弃次数。 */
  Bmi088_RawSample_t raw; /**< 本轮成功采样的真实原字节；与上述解析量在同一临界区发布。 */
} Bmi088_Snapshot_t;

/** @brief 无硬件访问的状态缓存；ID/配置验证属于通信自检，非MEMS激励自检。 */
typedef struct {
  uint32_t rx_sequence;       /**< 最近完整样本序号。 */
  uint32_t sample_time_ms;    /**< 最近完整样本时间，单位ms。 */
  uint32_t error_count;       /**< 累计真实错误数，不含暂未就绪。 */
  uint32_t reinit_count;      /**< 已启动的初始化尝试次数，首次计入。 */
  uint32_t fifo_drop_count;   /**< 陀螺FIFO积压或溢出导致的丢弃次数。 */
  uint32_t accel_sensor_time; /**< 最近发布的加速度24bit硬件时间，39.0625us/LSB。 */
  Bmi088_Bus_t bus;          /**< 当前选择的总线，I2C=1、SPI=2。 */
  Bmi088_Result_t last_result;/**< 最近操作结果。 */
  Bmi088_Stage_t last_stage;  /**< 最近操作阶段。 */
  uint8_t accel_chip_id;      /**< 最近加速度ID，期望0x1E。 */
  uint8_t gyro_chip_id;       /**< 最近陀螺ID，期望0x0F。 */
  uint8_t accel_addr_7bit;    /**< I2C加速度地址0x18/0x19；SPI为0。 */
  uint8_t gyro_addr_7bit;     /**< I2C陀螺地址0x68/0x69；SPI为0。 */
  uint8_t last_bsp_status;    /**< 最近BSP返回码。 */
  uint8_t initialized;        /**< 双ID、配置读回均通过为1，否则0。 */
} Bmi088_Status_t;

/**
 * @brief 接受初始化请求并启动分阶段流程，不等待芯片复位或数据就绪。
 * @param[in] bus 实际连接总线。
 * @return OK仅表示总线初始化成功且流程已接受；完成需看initialized与新样本。
 * @note 仅由sensor任务调用；Service推进后续ID、复位、配置和等待。
 */
Bmi088_Result_t Sensor_BMI088_Init(Bmi088_Bus_t bus);
/** @brief 在sensor任务推进一阶段或读取一轮；now_ms为真实毫秒。 */
Bmi088_Result_t Sensor_BMI088_Service(uint32_t now_ms);
/** @brief 原子复制同次解析量和真实原字节；初始化/恢复或未采样时返回NO_DATA。 */
Bmi088_Result_t Sensor_BMI088_CopySnapshot(Bmi088_Snapshot_t *out);
/** @brief 复制轻量状态，不访问总线；参数非空时返回OK。 */
Bmi088_Result_t Sensor_BMI088_GetStatus(Bmi088_Status_t *out);
/** @brief 仅设置恢复请求，实际总线操作在后续Service内执行。 */
void Sensor_BMI088_RequestReinit(void);

#endif
