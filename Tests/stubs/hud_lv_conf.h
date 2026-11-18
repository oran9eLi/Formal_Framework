/**
 * @file hud_lv_conf.h
 * @brief 真实LVGL HUD主机像素回归配置，仅替换不可在电脑访问的CCM裸地址。
 * @details 颜色深度、字体、绘图算法与圆缓存数量沿用生产配置；不进入固件工程。
 */
#include "../../Third_Party/LVGL/lv_conf.h"
#undef LV_MEM_ADR
#define LV_MEM_ADR 0
