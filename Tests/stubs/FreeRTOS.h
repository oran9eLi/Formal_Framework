/** @file FreeRTOS.h
 * @brief PC 控制逻辑测试不启动调度器；允许专用用例替换同步钩子。
 */
#ifndef INC_FREERTOS_H
#define INC_FREERTOS_H
#ifndef taskENTER_CRITICAL
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#endif
#endif
