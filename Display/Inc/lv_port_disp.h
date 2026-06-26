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
uint8_t LvPortDisp_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif
