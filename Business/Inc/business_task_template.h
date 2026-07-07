/**
 * @file business_task_template.h
 * @brief Business 层任务、业务服务和平台钩子接口。
 *
 * @details
 * Business 层只通过 `app_data_api.h` 读取 Framework 快照，不直接访问 BSP、Sensor
 * 或 DMA buffer。本文件中的任务入口由 FreeRTOS 调度，服务函数应保持非阻塞或
 * 预算化执行。
 */

#ifndef BUSINESS_TASK_TEMPLATE_H
#define BUSINESS_TASK_TEMPLATE_H

#include <stdint.h>

/**
 * @brief Business 服务返回值。
 */
typedef enum {
  BUSINESS_SERVICE_OK = 0,    /**< 本次服务完成且无错误。 */
  BUSINESS_SERVICE_BUSY,      /**< 本次预算用完，仍有工作留到下一周期。 */
  BUSINESS_SERVICE_IDLE,      /**< 当前无待处理工作。 */
  BUSINESS_SERVICE_NOT_READY, /**< 依赖数据或模块尚未就绪。 */
  BUSINESS_SERVICE_ERROR      /**< 服务执行出错。 */
} Business_ServiceResult_t;

/**
 * @brief Business 层组件编号。
 */
typedef enum {
  BUSINESS_COMPONENT_SYSTEM = 0,  /**< 系统业务组件。 */
  BUSINESS_COMPONENT_ACQUISITION, /**< 业务采集/事件分发组件。 */
  BUSINESS_COMPONENT_DISPLAY,     /**< 显示业务组件。 */
  BUSINESS_COMPONENT_COUNT        /**< 组件数量，必须保持为最后一项。 */
} Business_ComponentId_t;

/**
 * @brief Business 层日志记录。
 */
typedef struct {
  uint32_t timestamp_ms; /**< 日志时间，单位：ms。 */
  uint16_t source_id;    /**< 日志来源组件或模块 ID。 */
  uint8_t level;         /**< 日志等级，具体含义由业务层定义。 */
  uint8_t reserved;      /**< 保留字段，保持结构体对齐。 */
  const char *field;     /**< 日志字段名，指向静态或调用期有效字符串。 */
  const char *value;     /**< 日志字段值，指向静态或调用期有效字符串。 */
} Business_LogRecord_t;

/**
 * @brief 获取 Business 层使用的平台毫秒时间。
 *
 * @return 当前系统毫秒时间，单位：ms。
 */
uint32_t Business_PlatformGetMs(void);

/**
 * @brief 执行一次非阻塞应用注册表轮询。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 */
void Business_RegistryPoll(uint32_t now_ms);

/**
 * @brief 更新一个 Business 组件的心跳和在线状态。
 *
 * @param[in] component_id Business 组件编号。
 */
void Business_StatusHeartbeat(Business_ComponentId_t component_id);

/**
 * @brief 将一条 Business 日志记录转换为事件总线消息。
 *
 * @param[in] record 日志记录，不能为 NULL。
 *
 * @return 写入结果。
 */
Business_ServiceResult_t Business_LogWrite(const Business_LogRecord_t *record);

/**
 * @brief 复制一份新的导航快照并发布业务采集事件。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 执行结果。
 */
Business_ServiceResult_t Business_AcquisitionRunOnce(uint32_t now_ms);

/**
 * @brief 轮询显示触摸输入。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 执行结果。
 */
Business_ServiceResult_t Business_DisplayPollTouch(uint32_t now_ms);

/**
 * @brief 轮询显示按键输入(KEY0 弹出隐藏页)。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 执行结果。
 */
Business_ServiceResult_t Business_DisplayPollKey(uint32_t now_ms);

/**
 * @brief 查询显示层是否有待处理的整页重绘(切页触发)。
 *
 * @return 非 0 表示需要在常规刷新周期之外立即触发一次刷新。
 */
uint8_t Business_DisplayNeedsImmediateRefresh(void);

/**
 * @brief 准备一份版本一致的显示快照。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 执行结果。
 */
Business_ServiceResult_t Business_DisplayPrepareSnapshot(uint32_t now_ms);

/**
 * @brief 执行一步预算化显示刷新。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 * @param[in] budget_us 本步最大执行预算，单位：us。
 *
 * @return 执行结果；仍有页面工作时返回 `BUSINESS_SERVICE_BUSY`。
 *
 * @note 本函数必须在预算内返回，避免显示刷新阻塞传感器采集。
 */
Business_ServiceResult_t Business_DisplayRefreshStep(uint32_t now_ms, uint32_t budget_us);

/**
 * @brief 将显示服务结果映射为 Framework 显示模块状态。
 *
 * @param[in] result 显示服务结果。
 * @param[in] now_ms 当前系统毫秒时间。
 */
void Business_DisplayReportResult(Business_ServiceResult_t result, uint32_t now_ms);

/**
 * @brief Business 系统任务入口。
 *
 * @param[in] argument FreeRTOS 任务参数，当前未使用。
 *
 * @note 周期为 1000 ms，用于启动日志、注册表轮询和系统业务心跳。
 */
void Business_SystemTask(void *argument);

/**
 * @brief Business 采集任务入口。
 *
 * @param[in] argument FreeRTOS 任务参数，当前未使用。
 *
 * @note 周期为 200 ms，用于复制 Navigation 快照并向 EventBus 分发。
 */
void Business_AcquisitionTask(void *argument);

/**
 * @brief Business 显示服务任务入口。
 *
 * @param[in] argument FreeRTOS 任务参数，当前未使用。
 *
 * @note 周期为 10 ms，触摸优先，显示刷新按步进预算执行。
 */
void Business_DisplayServiceTask(void *argument);

#endif
