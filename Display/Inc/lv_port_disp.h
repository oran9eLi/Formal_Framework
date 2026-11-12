/**
 * @file lv_port_disp.h
 * @brief LVGL display port owned by the Display module.
 *
 * @details
 * This port bridges LVGL dirty-area flushing to the SSD1963 GRAM writer. It
 * must only be driven from DisplayTask through the Display module facade.
 */

#ifndef LV_PORT_DISP_H
#define LV_PORT_DISP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t LvPortDisp_Init(void);
/** @brief 显示所有者每轮设置预算；0表示本轮只服务输入。 */
void LvPortDisp_BeginStep(uint32_t started_us, uint32_t budget_us);
/** @brief 获取本轮同步像素写入累计和单次最大耗时，us。 */
void LvPortDisp_GetFlushTime(uint32_t *total_us, uint32_t *maximum_us);

/**
 * @brief 为姿态圆圈建立完整离屏帧；只在LVGL旧脏区已清空时调用。
 * @return 1为本帧将离屏捕获；0保持原普通刷新路径。
 */
uint8_t LvPortDisp_BeginHudFrame(uint16_t x,uint16_t y,uint16_t width,uint16_t height);
/**
 * @brief 旧脏区全部绘制完后集中呈现圆圈；不等待下一个任务节拍。
 * @return 0尚在准备/无帧，1完整提交，2丢弃不完整帧并需重试。
 * @note 一帧提交是有界的HUD专属同步单元，中断可抢占，耗时计入flush统计。
 */
uint8_t LvPortDisp_FinishHudFrame(void);
/** @brief 切页、布局或恢复时撤销离屏批次，不写入任何待呈现像素。 */
void LvPortDisp_AbortHudFrame(void);

#ifdef __cplusplus
}
#endif

#endif
