/**
 * @file px4lite_runtime_stats.h
 * @brief Framework 常驻资源统计接口；不依赖串口调试开关。
 * @details 固定登记任务；低频每次只测一个栈，不在 ISR/Health 中扫描，不访问 LVGL。
 */
#ifndef PX4LITE_RUNTIME_STATS_H
#define PX4LITE_RUNTIME_STATS_H
#include "px4lite_types.h"
#include "FreeRTOS.h"
#include "task.h"
#define PX4LITE_RUNTIME_TASK_CAPACITY 16U /**< 固定任务登记容量。 */
/** @brief 任务栈测量副本；word 为 StackType_t 单位。 */
typedef struct {
  const char *name; /**< 启动期常量任务名。 */
  uint32_t measured_ms; /**< 最近扫描时间，ms。 */
  uint16_t configured_words; /**< 配置栈深度，word。 */
  uint16_t minimum_free_words; /**< 历史最低剩余栈，word。 */
  uint8_t measured; /**< 1=已有测量。 */
  uint8_t warning; /**< 剩余小于100 word或配置的25%。 */
  uint16_t reserved; /**< 对齐。 */
} Px4Lite_TaskRuntime_t;
/** @brief 由唯一显示任务提交的资源事实；时间均为us。 */
typedef struct {
  uint32_t measured_ms; /**< 最近提交时间，ms。 */
  uint32_t service_us; /**< 触摸、取数、布局、渲染和flush整轮耗时。 */
  uint32_t service_max_us; /**< 整轮历史最大耗时。 */
  uint32_t render_us; /**< lv_timer_handler耗时扣除同步flush。 */
  uint32_t render_max_us; /**< 渲染历史最大耗时。 */
  uint32_t flush_us; /**< 本轮像素写入总耗时。 */
  uint32_t flush_max_us; /**< 单次像素写入最大耗时。 */
  uint32_t overruns; /**< 整轮超过配置预算的次数。 */
  uint32_t lvgl_used; /**< LVGL当前使用字节。 */
  uint32_t lvgl_peak; /**< LVGL历史最大使用字节。 */
  uint32_t lvgl_max_free; /**< LVGL最大空闲块字节。 */
  uint8_t lvgl_fragmentation; /**< LVGL碎片率，0..100%。 */
  uint8_t reserved[3]; /**< 对齐。 */
} Px4Lite_DisplayRuntime_t;
/** @brief 常驻资源摘要；消费者复制，不执行栈扫描。 */
typedef struct {
  uint32_t measured_ms; /**< 汇总时间，ms。 */
  uint32_t heap_total; /**< RTOS堆配置字节。 */
  uint32_t heap_free; /**< RTOS当前空闲字节。 */
  uint32_t heap_minimum; /**< RTOS历史最小空闲字节。 */
  uint32_t registration_errors; /**< 任务登记失败次数。 */
  uint32_t imu_overflows; /**< 生产IMU FIFO累计溢出。 */
  uint32_t rx_drops; /**< 生产LoRa接收队列累计丢弃。 */
  uint32_t rx_overflows; /**< 生产LoRa DMA累计溢出。 */
  uint32_t comm_period_max_ms; /**< comm实测相邻服务最大间隔，ms。 */
  uint16_t imu_count; /**< FIFO当前元素数。 */
  uint16_t imu_peak; /**< FIFO历史峰值元素数。 */
  uint8_t task_count; /**< 已登记任务数。 */
  uint8_t reserved[3]; /**< 对齐。 */
  Px4Lite_DisplayRuntime_t display; /**< 显示所有者最近提交。 */
} Px4Lite_RuntimeSnapshot_t;
/** @brief 调度前清空固定登记表。 */
void Px4Lite_RuntimeInit(void);
/** @brief 调度前登记任务；拒绝重复、空句柄及容量超限。 */
BaseType_t Px4Lite_RuntimeRegister(TaskHandle_t handle, const char *name, uint16_t words);
/** @brief biz_system每轮调用；临界区外扫描一个任务栈。 */
void Px4Lite_RuntimeStep(uint32_t now_ms);
/** @brief comm所有者提交服务时间，不改变调度。 */
void Px4Lite_RuntimeReportComm(uint32_t now_ms);
/** @brief 显示所有者提交固定摘要。 */
void Px4Lite_RuntimeReportDisplay(const Px4Lite_DisplayRuntime_t *sample);
/** @brief 原子复制常驻摘要。 */
Px4Lite_Result_t Px4Lite_RuntimeCopy(Px4Lite_RuntimeSnapshot_t *out);
/** @brief 原子复制指定登记行，index范围0..task_count-1。 */
Px4Lite_Result_t Px4Lite_RuntimeCopyTask(uint8_t index, Px4Lite_TaskRuntime_t *out);
#endif
