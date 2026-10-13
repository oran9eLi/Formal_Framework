/** @file display_observation.h
 * @brief Display固定坐标缓存绘图；只读App观察，不分配LVGL对象。
 * @details 由显示任务刷新与绘制，复用姿态对象；Freeze保留唯一坐标缓存。
 */
#ifndef DISPLAY_OBSERVATION_H
#define DISPLAY_OBSERVATION_H
#include "lvgl.h"
#include "app_experiment.h"
/** @brief 更新固定坐标；前后版本不一致时撤销该轮，不画混合环。 */
void Display_ObservationUpdate(const App_ObservationStatus_t *status,lv_obj_t *object);
/** @brief 在已存在对象DRAW_POST中绘制；不读取生产或观察环。 */
void Display_ObservationDraw(lv_obj_t *object,lv_draw_ctx_t *context);
#endif
