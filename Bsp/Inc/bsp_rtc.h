/**
 * @file bsp_rtc.h
 * @brief 声明 STM32F407 RTC 日期时间读写接口。
 *
 * @details
 * BSP RTC 只负责 RTC 外设、备份域有效标记和 UTC 日期时间读写，不参与 GNSS
 * 校时策略、显示时区转换或业务状态判定。
 */

#ifndef BSP_RTC_H
#define BSP_RTC_H

#include <stdint.h>
#include "bsp_status.h"

/**
 * @brief RTC UTC 日期时间。
 */
typedef struct {
  uint16_t year;   /**< 完整年份，范围 2000 到 2099。 */
  uint8_t month;   /**< 月份，范围 1 到 12。 */
  uint8_t day;     /**< 日期，范围 1 到 31。 */
  uint8_t hours;   /**< 小时，范围 0 到 23。 */
  uint8_t minutes; /**< 分钟，范围 0 到 59。 */
  uint8_t seconds; /**< 秒，范围 0 到 59。 */
} BSP_RTC_DateTime_t;

/**
 * @brief 初始化 RTC 外设和备份域访问。
 *
 * @return 初始化结果。
 */
BSP_Status_t BSP_RTC_Init(void);

/**
 * @brief 判断 RTC 是否已有有效时间标记。
 *
 * @return 1 表示 RTC 已由本固件写入过有效日期时间，0 表示未校时或备份域失效。
 */
uint8_t BSP_RTC_IsTimeValid(void);

/**
 * @brief 读取 RTC 当前 UTC 日期时间。
 *
 * @param[out] out 输出日期时间，不能为 NULL。
 *
 * @return 读取结果。
 */
BSP_Status_t BSP_RTC_ReadDateTime(BSP_RTC_DateTime_t *out);

/**
 * @brief 写入 RTC UTC 日期时间并更新备份域有效标记。
 *
 * @param[in] date_time 待写入日期时间，不能为 NULL。
 *
 * @return 写入结果。
 */
BSP_Status_t BSP_RTC_WriteDateTime(const BSP_RTC_DateTime_t *date_time);

#endif
