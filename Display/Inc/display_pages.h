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

/**
 * @brief       绘制 Motor 页单个竖向油门滑条动态部分
 * @param       variable: 电机 PWM 变量配置项
 * @param       old_value: 上一次已绘制油门百分比，范围 0 到 100
 * @param       value: 当前油门百分比，范围 0 到 100
 * @param       full_redraw: 非 0 表示重绘完整滑条轨道
 * @param       draw_percent: 非 0 表示同步刷新底部百分比文本
 * @retval      Display_Result_t: 绘制结果
 */
Display_Result_t Display_PagesDrawMotorSliderField(const Display_HmiVariableConfig_t *variable, uint32_t old_value, uint32_t value, uint8_t full_redraw, uint8_t draw_percent);

/**
 * @brief       绘制页眉动态区：左侧日期/时间，右侧丢包率
 * @param       read_value: 变量缓存读取回调
 * @param       force: 非 0 强制全部重绘，否则仅重绘发生变化的部分
 * @retval      Display_Result_t: 绘制结果
 */
Display_Result_t Display_PagesDrawHeaderDynamic(Display_PagesValueReader_t read_value, uint8_t force);

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
void Display_PagesClearLogMessages(void);

typedef struct {
  Display_LogMsg_t msg;
  uint32_t time_hhmmss;
} Display_MessageLogEntry_t;

uint16_t Display_PagesCopyLogMessages(Display_MessageLogEntry_t *entries, uint16_t max_count, Display_MessageLogEntry_t *alarm_entry, uint8_t *alarm_valid);

/**
 * @brief       获取消息日志缓冲版本号（每追加一条 +1，用于驱动重绘）
 * @retval      uint32_t: 当前版本号
 */
uint32_t Display_PagesGetLogVersion(void);

/* ============================ LoRa 连接页（隐藏页）数据接口 ============================ */

#define DISPLAY_LORA_MAX_NODES     16U /**< 列表缓存的最大在线节点数。 */
#define DISPLAY_LORA_ROWS_PER_PAGE 7U  /**< 列表每页显示行数。 */

/**
 * @brief LoRa 在线节点条目。
 * @note  业务层只传入“在线”节点；离线节点不进入列表，由调用方过滤。
 */
typedef struct {
  uint16_t node_id;          /**< 节点地址/ID，列表与详情显示为 0xXX。 */
  uint16_t label;            /**< 节点序号，列表显示为“节点 NN”。 */
  uint32_t last_comm_hhmmss; /**< 上次通信时间，编码 HHMMSS；未知为 0。 */
} Display_LoraNode_t;

/**
 * @brief 选中行连接命令回调。
 * @param node_id 目标节点 ID。
 * @param connect 1 表示请求连接，0 表示请求断开。
 * @note  由业务/通信层注册；显示层在按钮按下时调用，不直接访问 LoRa 硬件。
 */
typedef void (*Display_LoraConnectHandler_t)(uint16_t node_id, uint8_t connect);

/**
 * @brief       更新 LoRa 连接页的在线节点列表（只在线节点）。
 * @param       nodes: 节点数组，超过 DISPLAY_LORA_MAX_NODES 时截断。
 * @param       count: 节点条数。
 * @note        选中项按 node_id 跟随；原选中节点不在新列表时回退到首行。
 */
void Display_PagesSetLoraNodes(const Display_LoraNode_t *nodes, uint16_t count);

/**
 * @brief       设置 LoRa 当前连接状态（由业务/通信层回写真实状态）。
 * @param       connected: 1 表示已连接，0 表示未连接。
 * @param       node_id:   已连接的节点 ID，connected 为 0 时忽略。
 */
void Display_PagesSetLoraConnected(uint8_t connected, uint16_t node_id);

/**
 * @brief       注册连接/断开命令回调。
 * @param       handler: 回调函数，传 0 清除。
 */
void Display_PagesSetLoraConnectHandler(Display_LoraConnectHandler_t handler);

/**
 * @brief       获取 LoRa 连接页内容版本号（列表/选中/连接状态变化即 +1，用于驱动重绘）。
 * @retval      uint32_t: 当前版本号。
 */
uint32_t Display_PagesGetLoraVersion(void);

/**
 * @brief LoRa 连接页触摸处理结果。
 */
typedef enum {
  DISPLAY_LORA_TOUCH_NONE = 0, /**< 未命中任何控件，无需重绘。 */
  DISPLAY_LORA_TOUCH_REDRAW,   /**< 选中行/翻页变化，需重绘内容。 */
  DISPLAY_LORA_TOUCH_COMMAND   /**< 连接/断开按钮触发，已调用回调，需重绘内容。 */
} Display_LoraTouchResult_t;

/**
 * @brief       处理 LoRa 连接页的一次按下触摸（点行选中 / 翻页 / 连接按钮）。
 * @param       x: 触摸 X 坐标。
 * @param       y: 触摸 Y 坐标。
 * @retval      Display_LoraTouchResult_t: 处理结果。
 */
Display_LoraTouchResult_t Display_PagesLoraHandleTouch(uint16_t x, uint16_t y);

/**
 * @brief       绘制 LoRa 连接页动态内容（列表行、选中、翻页、详情、按钮）。
 * @note        固定边框由 Display_PagesDrawStatic 在切页时绘制。
 */
void Display_PagesDrawLoraContent(void);

/**
 * @brief       查询 LoRa 连接页内容是否需要重绘（版本号变化）。
 * @retval      uint8_t: 非 0 表示需要调用 Display_PagesDrawLoraContent。
 */
uint8_t Display_PagesLoraContentDirty(void);

#ifdef __cplusplus
}
#endif

#endif
