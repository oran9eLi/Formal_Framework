/**
 * @file lv_port_disp.h
 * @brief LVGL display port owned by the Display module.
 *
 * @details
 * This port bridges LVGL dirty-area flushing to the SSD1963 GRAM writer. It
 * must only be driven from DisplayTask through the Display module facade.
 */

#ifndef LV_PORT_DISP_H
#define LV_PORT_DISP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t LvPortDisp_Init(void);
/** @brief 显示所有者每轮设置预算；0表示本轮只服务输入。 */
void LvPortDisp_BeginStep(uint32_t started_us, uint32_t budget_us);
/** @brief 获取本轮同步像素写入累计和单次最大耗时，us。 */
void LvPortDisp_GetFlushTime(uint32_t *total_us, uint32_t *maximum_us);

#ifdef __cplusplus
}
#endif

#endif
