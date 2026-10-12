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
static uint8_t s_registration_attempted;
static uint16_t s_last_x;
static uint16_t s_last_y;

/**
 * @brief 读取触摸点；初始化或运行故障均立即向 LVGL 返回释放。
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
 * @brief 启动时仅尝试注册一次输入设备；触摸失败不影响注册，运行期不重复分配。
 */
uint8_t LvPortIndev_Init(void)
{
  /* 注册与硬件在位无关，运行期恢复不再分配 LVGL 输入设备。 */
  (void)Display_Gt911_Init();

  if (s_registration_attempted == 0U) {
    s_registration_attempted = 1U;
    lv_indev_drv_init(&s_indev_drv);
    s_indev_drv.type    = LV_INDEV_TYPE_POINTER;
    s_indev_drv.read_cb = LvPortIndev_Read;
    s_indev             = lv_indev_drv_register(&s_indev_drv);
  }

  return (s_indev != 0) ? 1U : 0U;
}

/** @brief 查询启动时输入设备注册结果，不分配资源。 */
uint8_t LvPortIndev_IsRegistered(void)
{ return (s_indev != 0) ? 1U : 0U; }
