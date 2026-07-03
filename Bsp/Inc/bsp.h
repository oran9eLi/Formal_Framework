/**
 * @file bsp.h
 * @brief 声明板级统一初始化入口和启动诊断快照。
 *
 * @details
 * 本文件属于 BSP 层，只暴露板级初始化聚合入口和只读调试快照。Driver、Framework
 * 或 Business 不得绕过各自边界直接调用具体 BSP 初始化函数。
 */

#ifndef BSP_H
#define BSP_H

#include <stdint.h>

#include "bsp_status.h"

#define BSP_INIT_I2C_MASK    (1UL << 0) /**< I2C 总线初始化结果位。 */
#define BSP_INIT_ADC_MASK    (1UL << 1) /**< ADC 采样初始化结果位。 */
#define BSP_INIT_GNSS_MASK   (1UL << 2) /**< GNSS UART/DMA 初始化结果位。 */
#define BSP_INIT_RTC_MASK    (1UL << 3) /**< RTC 初始化结果位。 */
#define BSP_INIT_BUTTON_MASK (1UL << 4) /**< 板载按键初始化结果位。 */
#define BSP_INIT_PWM_MASK    (1UL << 5) /**< PWM 输出初始化结果位。 */

/**
 * @brief BSP 统一初始化诊断快照。
 *
 * @details
 * 本结构体只记录 `BSP_Init()` 已启用、已尝试和失败的板级资源位图，供 Debug
 * 模块只读打印使用。上层模块不得根据该结构体绕过正式模块状态、告警或恢复流程。
 */
typedef struct {
  uint32_t enabled_mask;   /**< 编译期启用的 BSP 资源位图，使用 `BSP_INIT_*_MASK`。 */
  uint32_t attempted_mask; /**< 本次 `BSP_Init()` 已尝试初始化的资源位图。 */
  uint32_t failed_mask;    /**< 本次 `BSP_Init()` 初始化失败的资源位图。 */
  BSP_Status_t result;     /**< `BSP_Init()` 聚合后的最终返回值。 */
} BSP_InitDebugInfo_t;

/**
 * @brief 从统一入口初始化所有启用的板级外设。
 *
 * @details
 * 这是唯一板级集中初始化路径。传感器驱动和平台适配器不应重复调用各个
 * `BSP_*_Init()`，而应依赖外设已在这里初始化。函数会尝试所有启用资源，并返回
 * 聚合状态，避免后一个成功覆盖前一个失败。
 *
 * @return BSP 初始化聚合结果。
 */
BSP_Status_t BSP_Init(void);

/**
 * @brief 复制最近一次 BSP 统一初始化诊断快照。
 *
 * @param[out] out 输出缓冲区，允许为 NULL；为 NULL 时函数不执行任何操作。
 *
 * @note 本接口只读，不重新初始化任何硬件，供 Debug 自测逐项确认启动链路使用。
 */
void BSP_GetInitDebugInfo(BSP_InitDebugInfo_t *out);

/**
 * @brief 在不可恢复错误路径中停止仍可能写内存的 DMA/异步发送。
 *
 * @note 仅供 `Error_Handler()` 等停机路径调用，不执行协议恢复或业务告警。
 */
void BSP_EmergencyStopDma(void);

#endif
