/**
 * @file px4lite_faults.h
 * @brief 定义系统级故障码和默认故障元数据。
 */

#ifndef PX4LITE_FAULTS_H
#define PX4LITE_FAULTS_H

#include <stdint.h>
#include "px4lite_types.h"

/**
 * @brief Framework 通用严重度等级。
 */
typedef enum {
  PX4LITE_SEVERITY_INFO = 0,
  PX4LITE_SEVERITY_WARNING,
  PX4LITE_SEVERITY_ERROR,
  PX4LITE_SEVERITY_CRITICAL,
  PX4LITE_SEVERITY_FATAL
} Px4Lite_Severity_t;

/**
 * @brief Framework 统一故障码。
 *
 * @details
 * 高字节按功能域分段，低字节表示域内具体故障。故障码用于模块状态、告警和
 * MAVLink STATUSTEXT 等对外输出。
 */
typedef enum {
  PX4LITE_FAULT_NONE = 0x0000U,

  PX4LITE_FAULT_SYSTEM_SELF_CHECK = 0x0101U,
  PX4LITE_FAULT_SYSTEM_HEAP       = 0x0102U,
  PX4LITE_FAULT_SYSTEM_STACK      = 0x0103U,
  PX4LITE_FAULT_SYSTEM_TASK_LOST  = 0x0104U,

  PX4LITE_FAULT_SENSOR_INIT    = 0x0201U,
  PX4LITE_FAULT_SENSOR_OFFLINE = 0x0202U,
  PX4LITE_FAULT_SENSOR_INVALID = 0x0203U,
  PX4LITE_FAULT_SENSOR_NO_FIX  = 0x0204U,
  PX4LITE_FAULT_SENSOR_TIMEOUT = 0x0205U,

  PX4LITE_FAULT_COMM_OFFLINE = 0x0301U,
  PX4LITE_FAULT_COMM_TIMEOUT = 0x0302U,
  PX4LITE_FAULT_COMM_FRAME   = 0x0303U,

  PX4LITE_FAULT_DISPLAY_OFFLINE = 0x0401U,
  PX4LITE_FAULT_DISPLAY_REFRESH = 0x0402U,

  PX4LITE_FAULT_STORAGE_NOT_READY = 0x0501U,
  PX4LITE_FAULT_STORAGE_WRITE     = 0x0502U,
  PX4LITE_FAULT_STORAGE_FULL      = 0x0503U,

  PX4LITE_FAULT_PROTOCOL_PARSE = 0x0601U,
  PX4LITE_FAULT_PROTOCOL_CRC   = 0x0602U,

  PX4LITE_FAULT_APP_INVALID_DATA = 0x0701U,

  PX4LITE_FAULT_ESTIMATOR_INPUT   = 0x0801U,
  PX4LITE_FAULT_ESTIMATOR_DIVERGE = 0x0802U,

  PX4LITE_FAULT_MOTOR_DISCONNECT = 0x0901U,
  PX4LITE_FAULT_MOTOR_POWER_LOW  = 0x0902U,
  PX4LITE_FAULT_MOTOR_DEAD       = 0x0903U,
  PX4LITE_FAULT_MOTOR_CHARGE     = 0x0904U,

  PX4LITE_FAULT_POWER_CHARGE = 0x0A01U
} Px4Lite_FaultCode_t;

/**
 * @brief 故障码默认元数据定义。
 */
typedef struct {
  Px4Lite_FaultCode_t code;    /**< 故障码。 */
  Px4Lite_ModuleId_t owner;    /**< 默认归属模块。 */
  Px4Lite_Severity_t severity; /**< 默认严重度。 */
  uint8_t latched;             /**< 1 表示故障需要显式清除。 */
  uint8_t reserved[3];         /**< 对齐预留。 */
  uint32_t raise_delay_ms;     /**< 故障确认延迟，单位 ms。 */
  uint32_t clear_delay_ms;     /**< 故障清除延迟，单位 ms。 */
} Px4Lite_FaultDefinition_t;

#endif
