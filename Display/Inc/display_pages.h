#ifndef DISPLAY_PAGES_H
#define DISPLAY_PAGES_H

#include "display.h"
#include "display_text.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DISPLAY_PAGES_FOOTER_Y       420U
#define DISPLAY_PAGES_FOOTER_HEIGHT  60U
#define DISPLAY_PAGES_NAV_SIDE_WIDTH 150U

/**
 * @brief       页面绘制层读取变量缓存的回调函数类型
 */
typedef uint32_t (*Display_PagesValueReader_t)(Display_HmiVariableId_t id, uint32_t default_value);

/**
 * @brief       获取上一页页面 ID
 * @param       page: 当前页面 ID
 * @retval      Display_HmiPage_t: 上一页页面 ID
 */
Display_HmiPage_t Display_PagesGetPrevPage(Display_HmiPage_t page);

/**
 * @brief       获取下一页页面 ID
 * @param       page: 当前页面 ID
 * @retval      Display_HmiPage_t: 下一页页面 ID
 */
Display_HmiPage_t Display_PagesGetNextPage(Display_HmiPage_t page);

/**
 * @brief       绘制页面头部网络和连接状态
 * @param       page: 当前页面 ID
 * @param       read_value: 变量缓存读取回调
 * @retval      Display_Result_t: 绘制结果
 */
Display_Result_t Display_PagesDrawHeader(Display_HmiPage_t page, Display_PagesValueReader_t read_value);

/**
 * @brief       绘制当前页面固定骨架
 * @param       page: 当前页面 ID
 * @param       read_value: 变量缓存读取回调
 * @retval      Display_Result_t: 绘制结果
 */
Display_Result_t Display_PagesDrawStatic(Display_HmiPage_t page, Display_PagesValueReader_t read_value);

/**
 * @brief       绘制一个变量字段
 * @param       variable: 变量配置项
 * @param       value: 变量原始值
 * @retval      Display_Result_t: 绘制结果
 */
Display_Result_t Display_PagesDrawField(const Display_HmiVariableConfig_t *variable, uint32_t value);

Display_Result_t Display_PagesDrawAlarmIcon(uint32_t now_ms, Display_PagesValueReader_t read_value, uint8_t force_draw);

/**
 * @brief       更新自检页错误码表的当前激活故障列表
 * @param       faults: 故障数组，每项为 (source_id << 16) | fault_code
 * @param       count:  故障条数，超过显示上限时截断
 * @note        故障恢复后不在列表内，自检错误表对应行随之消失
 */
void Display_PagesSetSelfCheckFaults(const uint32_t *faults, uint16_t count);

/**
 * @brief       向消息日志区追加一条消息（普通消息最新在最下，告警状态显示在标题行）
 * @param       msg: 消息条目类型
 * @param       time_hhmmss: 时间戳，编码 HHMMSS（开机运行时间占位）
 */
void Display_PagesPushLogMessage(Display_LogMsg_t msg, uint32_t time_hhmmss);

/**
 * @brief       获取消息日志缓冲版本号（每追加一条 +1，用于驱动重绘）
 * @retval      uint32_t: 当前版本号
 */
uint32_t Display_PagesGetLogVersion(void);

#ifdef __cplusplus
}
#endif

#endif
