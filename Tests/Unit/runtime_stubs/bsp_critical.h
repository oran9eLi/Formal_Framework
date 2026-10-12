/** @file bsp_critical.h @brief 驱动单线程测试专用临界区替身。 */
#ifndef TEST_RUNTIME_BSP_CRITICAL_H
#define TEST_RUNTIME_BSP_CRITICAL_H
#include <stdint.h>
static inline uint32_t BSP_Critical_Enter(void) { return 0U; }
static inline void BSP_Critical_Exit(uint32_t primask) { (void)primask; }
#endif
