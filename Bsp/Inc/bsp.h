/**
 * @file bsp.h
 * @brief Declare board-level initialization.
 */

#ifndef BSP_H
#define BSP_H

#include "bsp_status.h"

/**
 * @brief Initialize every enabled board peripheral from one central entry.
 *
 * This is the single board-level initialization path. Sensor drivers and the
 * platform adapter must not call individual BSP_*_Init() functions; they rely
 * on the peripheral already being initialized here.
 */
BSP_Status_t BSP_Init(void);

#endif
