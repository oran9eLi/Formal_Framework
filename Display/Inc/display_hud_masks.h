/**
 * @file display_hud_masks.h
 * @brief 姿态HUD专用固定圆抗锯齿表与完整圆环绘制。
 * @details 只属Display层、显示任务独占；复用原生LVGL半径像素，不改全局缓存/其他控件。
 * 不读取传感器或BSP，不建立运行期堆缓存；不支持的几何回退原生LVGL。
 */
#ifndef DISPLAY_HUD_MASKS_H
#define DISPLAY_HUD_MASKS_H
#include "lvgl.h"

/**
 * @brief 初始化HUD固定半径mask；93/101/102/104/115px使用Flash表，其他半径原生回退。
 * @param param 输出mask描述符，寿命覆盖对应绘制调用。
 * @param rect 绝对屏幕矩形，单位px。
 * @param radius 圆角半径，单位px；仍按矩形短边一半限制。
 * @param inv true保留圆外，false保留圆内。
 * @note 必须与Display_HudMaskRadiusFree配对，不得对固定表调用原生free。
 */
void Display_HudMaskRadiusInit(lv_draw_mask_radius_param_t *param,const lv_area_t *rect,lv_coord_t radius,bool inv);
/** @brief 释放原生回退mask；固定Flash表无需释放，不改变共享描述符。 */
void Display_HudMaskRadiusFree(lv_draw_mask_radius_param_t *param);
/**
 * @brief 按原生完整360°圆环语义绘制HUD两道固定环，保持重复外mask的AA混合。
 * @param dc 当前LVGL绘制上下文；不保存其指针。
 * @param dsc 完整圆环描述符，颜色为RGB565转换前的原生lv_color_t。
 * @param center 圆心绝对坐标，px。
 * @param radius 外半径，px；仅104/2px宽与115/22px宽使用固定表。
 * @note 其他半径/宽度或图像源按原生lv_draw_arc(0,360)回退，不修改其他页面策略。
 */
void Display_HudDrawFullRing(lv_draw_ctx_t *dc,const lv_draw_arc_dsc_t *dsc,const lv_point_t *center,uint16_t radius);
#endif
