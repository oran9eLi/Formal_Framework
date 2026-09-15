/** @file task.h
 * @brief PC 单线程控制回归的调度接口替身。
 */
#ifndef INC_TASK_H
#define INC_TASK_H
#ifndef vTaskSuspendAll
#define vTaskSuspendAll() ((void)0)
#define xTaskResumeAll() (0)
#endif
#endif
