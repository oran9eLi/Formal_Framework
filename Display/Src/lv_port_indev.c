/**
 * @file lv_port_indev.c
 * @brief LVGL pointer input port for GT911 touch scanning.
 *
 * @details
 * GT911 reads are performed only when LVGL input processing runs in
 * DisplayTask. The callback keeps the last valid coordinate while the touch is
 * released, which matches LVGL pointer device semantics.
 */

#include "lv_port_indev.h"

#include "display_gt911.h"
#include "lvgl.h"

static lv_indev_drv_t s_indev_drv;
static lv_indev_t *s_indev;
static uint16_t s_last_x;
static uint16_t s_last_y;
static Display_Gt911Result_t s_init_result = DISPLAY_GT911_NOT_READY;

/**
 * @brief Read one GT911 point for LVGL.
 */
static void LvPortIndev_Read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
  uint16_t x;
  uint16_t y;

  (void)drv;
  if (data == 0) {
    return;
  }

  if (Display_Gt911_Scan(&x, &y) == DISPLAY_GT911_OK) {
    s_last_x = x;
    s_last_y = y;
    data->state = LV_INDEV_STATE_PR;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }

  data->point.x = (lv_coord_t)s_last_x;
  data->point.y = (lv_coord_t)s_last_y;
}

/**
 * @brief Initialize LVGL pointer input registration.
 */
uint8_t LvPortIndev_Init(void)
{
  s_init_result = Display_Gt911_Init();

  if (s_indev == 0) {
    lv_indev_drv_init(&s_indev_drv);
    s_indev_drv.type    = LV_INDEV_TYPE_POINTER;
    s_indev_drv.read_cb = LvPortIndev_Read;
    s_indev             = lv_indev_drv_register(&s_indev_drv);
  }

  return ((s_indev != 0) && (s_init_result == DISPLAY_GT911_OK)) ? 1U : 0U;
}
