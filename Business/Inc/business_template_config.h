/**
 * @file business_template_config.h
 * @brief Configure business task periods, stacks, and display servicing.
 */

#ifndef BUSINESS_TEMPLATE_CONFIG_H
#define BUSINESS_TEMPLATE_CONFIG_H

/*
 * Display is enabled in the architecture. Until the real driver is ready,
 * business_display_placeholder.c returns NOT_READY.
 */
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
