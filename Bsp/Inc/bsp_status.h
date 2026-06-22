/**
 * @file bsp_status.h
 * @brief 定义 BSP 层通用返回码。
 */

#ifndef BSP_STATUS_H
#define BSP_STATUS_H

/**
 * @brief BSP 对外接口统一返回码。
 */
typedef enum {
  BSP_STATUS_OK = 0,
  BSP_STATUS_ERROR,
  BSP_STATUS_BUSY,
  BSP_STATUS_TIMEOUT
} BSP_Status_t;

#endif
