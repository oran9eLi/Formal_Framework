/**
 * @file bsp_watchdog.h
 * @brief 声明独立看门狗(IWDG)启动、刷新与复位原因板级接口。
 *
 * @details
 * 本文件属于 BSP 层，是访问 IWDG 的唯一入口。业务任务不得直接访问 IWDG 寄存器，
 * 也不得绕过 `Px4Lite_PlatformWatchdogFeed()` 直接调用 `BSP_WatchdogRefresh()`：
 * 运行喂狗必须经过平台层心跳门控；调度前仅 Core/BSP 有限进展点可用 BootProgress。
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
 * 必须在 main 最早期、HAL 初始化前调用一次。RCC_CSR 的复位标志会一直保留到写 RMVF
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

/** @brief main 最早期启动 IWDG 启动窗口；失败锁存，重复成功调用幂等。 */
BSP_Status_t BSP_Watchdog_StartBoot(void);
/** @brief 调度前有限初始化进展点刷新；关闭后永久无效。 */
void BSP_Watchdog_BootProgress(void);
/** @brief 交接或初始化失败时关闭本次启动的进展刷新。 */
void BSP_Watchdog_SealBootProgress(void);
/** @brief 初始心跳健康后切换运行窗口；必须已经封闭 boot progress。 */
BSP_Status_t BSP_Watchdog_EnterRuntime(void);
/** @brief 硬件已发出启动命令返回 1；不代表运行窗口配置成功。 */
uint8_t BSP_Watchdog_IsStarted(void);
/** @brief 仅供 Platform 运行态健康门控后刷新；未进入运行态或失败时不操作硬件。 */
void BSP_WatchdogRefresh(void);
#endif
