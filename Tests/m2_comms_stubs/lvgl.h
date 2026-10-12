#ifndef M2_TEST_LVGL_H
#define M2_TEST_LVGL_H
#include <stdint.h>
typedef int16_t lv_coord_t;
typedef struct { int unused; } lv_indev_t;
typedef struct { int state; struct { lv_coord_t x,y; } point; } lv_indev_data_t;
typedef struct lv_indev_drv_t { int type; void (*read_cb)(struct lv_indev_drv_t *,lv_indev_data_t *); } lv_indev_drv_t;
#define LV_INDEV_TYPE_POINTER 1
#define LV_INDEV_STATE_PR 1
#define LV_INDEV_STATE_REL 0
void lv_indev_drv_init(lv_indev_drv_t *);
lv_indev_t *lv_indev_drv_register(lv_indev_drv_t *);
#endif
