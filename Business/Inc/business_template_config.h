/**
 * @file business_template_config.h
 * @brief 配置 Business 任务周期、栈大小、优先级和显示刷新参数。
 *
 * @details
 * Business 配置只描述应用层任务行为。Framework 固定任务配置在
 * `px4lite_config.h`，BSP 硬件资源配置在 `bsp_config.h`。
 */

#ifndef BUSINESS_TEMPLATE_CONFIG_H
#define BUSINESS_TEMPLATE_CONFIG_H

/* 架构上启用显示；真实显示驱动未就绪时，display adapter 返回 NOT_READY。 */
#define BUSINESS_ENABLE_DISPLAY              1U

#define BUSINESS_EVENT_QUEUE_LENGTH          8U
#define BUSINESS_SYSTEM_PERIOD_MS         1000U
#define BUSINESS_ACQUISITION_PERIOD_MS      200U
#define BUSINESS_PRIORITY_SYSTEM      (tskIDLE_PRIORITY + 2U)
#define BUSINESS_PRIORITY_ACQUISITION (tskIDLE_PRIORITY + 2U)
#define BUSINESS_SYSTEM_TASK_STACK_WORDS     384U
#define BUSINESS_ACQUISITION_TASK_STACK_WORDS 512U

#define BUSINESS_DISPLAY_SERVICE_PERIOD_MS   10U
#define BUSINESS_DISPLAY_REFRESH_PERIOD_MS  200U
#define BUSINESS_DISPLAY_REFRESH_BUDGET_US 2000U
#define BUSINESS_PRIORITY_DISPLAY     (tskIDLE_PRIORITY + 1U)
#define BUSINESS_DISPLAY_TASK_STACK_WORDS   768U

#endif
