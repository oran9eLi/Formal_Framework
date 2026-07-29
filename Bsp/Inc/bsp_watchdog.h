/**
 * @file bsp_watchdog.h
 * @brief 声明独立看门狗(IWDG)启动、刷新与复位原因板级接口。
 *
 * @details
 * 本文件属于 BSP 层，是访问 IWDG 的唯一入口。业务任务不得直接访问 IWDG 寄存器，
 * 也不得绕过 `Px4Lite_PlatformWatchdogFeed()` 直接调用 `BSP_WatchdogRefresh()`：
 * 喂狗必须经过平台层的任务心跳门控（见《强制开发规范》13.4）。
 */

#ifndef BSP_WATCHDOG_H
#define BSP_WATCHDOG_H

#include <stdint.h>

#include "bsp_status.h"

#define BSP_RESET_CAUSE_NONE     0x00U /**< 未捕获到任何复位标志。 */
#define BSP_RESET_CAUSE_IWDG     0x01U /**< 独立看门狗复位。 */
#define BSP_RESET_CAUSE_WWDG     0x02U /**< 窗口看门狗复位。 */
#define BSP_RESET_CAUSE_SOFT     0x04U /**< 软件复位。 */
#define BSP_RESET_CAUSE_POR      0x08U /**< 上电/掉电复位。 */
#define BSP_RESET_CAUSE_PIN      0x10U /**< 外部 NRST 引脚复位。 */
#define BSP_RESET_CAUSE_BOR      0x20U /**< 欠压复位。 */
#define BSP_RESET_CAUSE_LPWR     0x40U /**< 低功耗管理复位。 */

/**
 * @brief 捕获并清除本次上电的复位原因标志。
 *
 * @details
 * 必须在 `BSP_Init()` 最早期调用一次。RCC_CSR 的复位标志会一直保留到写 RMVF
 * 或发生上电复位，若不在启动早期读取并清除，下次复位时将无法区分本次与历史标志。
 * 读到的结果由 `BSP_Watchdog_GetResetCause()` 提供给上层记录。
 */
void BSP_Watchdog_CaptureResetCause(void);

/**
 * @brief 返回最近一次捕获的复位原因位图。
 *
 * @return `BSP_RESET_CAUSE_*` 位或组合；未调用捕获函数时返回 `BSP_RESET_CAUSE_NONE`。
 *
 * @note 本接口只读，可重复调用，不产生副作用。
 */
uint8_t BSP_Watchdog_GetResetCause(void);

/**
 * @brief 启动独立看门狗。
 *
 * @details
 * IWDG 一旦启动便无法通过软件关闭，只能由复位停止。因此本函数不在 `BSP_Init()`
 * 中调用，而由平台层在所有必需任务心跳首次全部健康后调用一次，避免启动阶段
 * 长耗时初始化（SD/LCD/GNSS）期间无人喂狗导致开机复位循环。
 *
 * 超时由 `BSP_WATCHDOG_PRESCALER_CODE` 与 `BSP_WATCHDOG_RELOAD` 决定；LSI 在
 * STM32F407 上标称 32 kHz、实际 17~47 kHz，选型必须按最快 47 kHz 校核最短超时。
 *
 * @return 启动结果；重复调用返回 `BSP_STATUS_OK` 且不重复配置。
 */
BSP_Status_t BSP_Watchdog_Start(void);

/**
 * @brief 查询独立看门狗是否已启动。
 *
 * @return 1 表示已启动，0 表示尚未启动。
 */
uint8_t BSP_Watchdog_IsStarted(void);

/**
 * @brief 刷新独立看门狗计数器。
 *
 * @note 调用方必须是 `Px4Lite_PlatformWatchdogFeed()`。未启动时本函数为空操作，
 *       便于在关闭硬件看门狗的构建中保持调用链一致。
 */
void BSP_WatchdogRefresh(void);

#endif
