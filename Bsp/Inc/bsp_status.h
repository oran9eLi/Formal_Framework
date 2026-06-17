/**
 * @file bsp_status.h
 * @brief Define common board-support return codes.
 */

#ifndef BSP_STATUS_H
#define BSP_STATUS_H

typedef enum
{
    BSP_STATUS_OK = 0,
    BSP_STATUS_ERROR,
    BSP_STATUS_BUSY,
    BSP_STATUS_TIMEOUT
} BSP_Status_t;

#endif
