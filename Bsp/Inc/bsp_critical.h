/**
 * @file bsp_critical.h
 * @brief 提供跨层可复用的短临界区进入和退出接口。
 *
 * @details
 * 本文件属于 BSP 层，只封装 PRIMASK 保存、关中断、数据同步屏障和按原状态恢复中断。
 * Driver、Framework 适配器或业务边界文件需要保护短快照复制时，应优先使用本接口，
 * 不得在上层文件中散落 CMSIS 关中断细节或无条件重新开中断。
 */

#ifndef BSP_CRITICAL_H
#define BSP_CRITICAL_H

#include <stdint.h>
#include "stm32f4xx.h"

/**
 * @brief 保存当前中断状态并进入短临界区。
 *
 * @return 进入临界区前的 PRIMASK 值；0 表示进入前中断处于允许状态。
 *
 * @note 临界区内只允许复制快照、更新标志或计数，不得执行解析、打印或阻塞 I/O。
 */
__STATIC_INLINE uint32_t BSP_Critical_Enter(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  __DMB();

  return primask;
}

/**
 * @brief 退出短临界区并按进入前状态恢复中断。
 *
 * @param[in] primask `BSP_Critical_Enter()` 返回的 PRIMASK 值。
 */
__STATIC_INLINE void BSP_Critical_Exit(uint32_t primask)
{
  __DMB();

  if (primask == 0U) { __enable_irq(); }
}

#endif
