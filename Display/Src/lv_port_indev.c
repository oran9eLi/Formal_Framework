/**
 * @file lv_port_indev.c
 * @brief Display 层 GT911 到 LVGL 指针输入的适配。
 * @details 仅显示任务读取触点。空轮询沿用上一触点，零触点报告结束手势；
 *          通信故障取消当前手势，禁止从中断或其他任务调用。
 */

#include "lv_port_indev.h"

#include "display_gt911.h"
#include "lvgl.h"

static lv_indev_drv_t s_indev_drv;
static lv_indev_t *s_indev;
static uint8_t s_registration_attempted;
static uint16_t s_last_x;
static uint16_t s_last_y;
static uint8_t s_pressed; /**< 最近一次有效报告是否有触点，0 松开、1 按下。 */

/**
 * @brief 区分暂未收到新帧和手指离屏，避免拖动中凭空产生松手或点击事件。
 */
static void LvPortIndev_Read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
  uint16_t x;
  uint16_t y;
  Display_Gt911Result_t result;

  (void)drv;
  if (data == 0) {
    return;
  }

  result = Display_Gt911_Scan(&x, &y);
  if (result == DISPLAY_GT911_OK) {
    s_last_x = x;
    s_last_y = y;
    s_pressed = 1U;
  } else if (result != DISPLAY_GT911_NO_DATA) {
    if (s_pressed && result != DISPLAY_GT911_NO_POINT && s_indev) {
      lv_indev_wait_release(s_indev);
    }
    s_pressed = 0U;
  }

  data->state = s_pressed ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
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
