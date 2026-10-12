/**
 * @file lv_port_indev.h
 * @brief LVGL input port owned by the Display module.
 */

#ifndef LV_PORT_INDEV_H
#define LV_PORT_INDEV_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 启动时仅注册一次输入设备，硬件恢复由显示周期推进。 */
uint8_t LvPortIndev_Init(void);
/** @brief 返回输入设备是否已注册。 */
uint8_t LvPortIndev_IsRegistered(void);

#ifdef __cplusplus
}
#endif

#endif
