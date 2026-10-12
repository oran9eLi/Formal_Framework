/**
 * @file display_gt911.h
 * @brief Declare the GT911 capacitive touch driver.
 */

#ifndef DISPLAY_GT911_H
#define DISPLAY_GT911_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  DISPLAY_GT911_OK = 0,
  DISPLAY_GT911_NOT_READY,
  DISPLAY_GT911_NO_POINT,
  DISPLAY_GT911_NO_DATA,
  DISPLAY_GT911_ERROR
} Display_Gt911Result_t;

typedef struct {
  uint8_t addr;
  uint8_t last_info;
  uint8_t int_level;
  uint8_t scl_level;
  uint8_t sda_level;
  uint8_t last_result;
  uint8_t last_error;
  uint8_t cfg_version;
  uint8_t cfg_verified;
  uint8_t init_error;
  uint8_t touch_status;
  uint8_t orig_ver;
  uint8_t cfg_bytes[4];
} Display_Gt911Debug_t;

/**
 * @brief 启动非阻塞触摸阶段机；就绪前返回 NOT_READY，后续由 Service 推进。
 */
Display_Gt911Result_t Display_Gt911_Init(void);
/** @brief 触摸独立运行状态，供同一显示任务读取。 */
typedef struct {
  uint8_t ready; /**< 1 表示控制器已就绪。 */
  uint8_t recovering; /**< 1 表示正在初始化或退避等待恢复。 */
  uint8_t last_error; /**< 最近故障码；恢复成功清零。 */
  uint32_t recovery_count; /**< 累计初始化尝试次数，含首次启动。 */
} Display_Gt911Status_t;
/** @brief 显示任务每周期推进一次硬件恢复阶段；now_ms 为真实毫秒。 */
void Display_Gt911_Service(uint32_t now_ms);
/** @brief 复制纯软件状态，不访问硬件。 */
void Display_Gt911_GetStatus(Display_Gt911Status_t *out);

/**
 * @brief Scan one touch point and return landscape display coordinates.
 */
Display_Gt911Result_t Display_Gt911_Scan(uint16_t *x, uint16_t *y);
/**
 * @brief Copy touch diagnostic state.
 */
void Display_Gt911_GetDebug(Display_Gt911Debug_t *debug);

#ifdef __cplusplus
}
#endif

#endif
