#ifndef DISPLAY_GFX_H
#define DISPLAY_GFX_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Display_Gfx 是屏幕点阵图元层，只负责像素、线条、矩形、圆形、网格。
 * 进度条和状态灯这类非文字、非图片内容。
 *
 * ATK-MD0700 横屏坐标。
 *   X: 0 ~ 799
 *   Y: 0 ~ 479
 *   颜色格式：RGB565
 */

#define DISPLAY_GFX_WIDTH          800U
#define DISPLAY_GFX_HEIGHT         480U

#define DISPLAY_GFX_COLOR_BLACK    0x0000U
#define DISPLAY_GFX_COLOR_WHITE    0xFFFFU
#define DISPLAY_GFX_COLOR_RED      0xF800U
#define DISPLAY_GFX_COLOR_GREEN    0x07E0U
#define DISPLAY_GFX_COLOR_BLUE     0x001FU
#define DISPLAY_GFX_COLOR_YELLOW   0xFFE0U
#define DISPLAY_GFX_COLOR_CYAN     0x07FFU
#define DISPLAY_GFX_COLOR_MAGENTA  0xF81FU
#define DISPLAY_GFX_COLOR_GRAY     0x8410U
#define DISPLAY_GFX_COLOR_DARK     0x4208U

typedef enum {
  DISPLAY_GFX_OK = 0,       /* 绘图成功 */
  DISPLAY_GFX_NOT_READY,    /* 底层画点或填充端口尚未注册*/
  DISPLAY_GFX_PARAM_ERROR   /* 入参超出范围或不合法 */
} Display_GfxResult_t;

/**
 * @brief       底层画点回调函数类型
 */
typedef void (*Display_GfxDrawPixelFn_t)(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief       底层矩形填充回调函数类型
 */
typedef void (*Display_GfxFillRectFn_t)(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);

typedef struct
{
  Display_GfxDrawPixelFn_t draw_pixel; /* 底层画点函数，后续接 atk_md0700_draw_point() */
  Display_GfxFillRectFn_t fill_rect;   /* 底层填充矩形函数，后续接 atk_md0700_fill() 或同类接口*/
} Display_GfxPort_t;

/**
 * @brief       注册底层绘图端口
 * @param       port: 底层绘图端口配置指针
 * @retval      Display_GfxResult_t: 注册结果
 */
Display_GfxResult_t Display_GfxInit(const Display_GfxPort_t *port);

/**
 * @brief       获取点阵图元层是否可用
 * @param       无
 * @retval      uint8_t: 0 表示不可用，1 表示可用
 */
uint8_t Display_GfxIsReady(void);

/**
 * @brief       将 8 位 RGB 颜色转换为 RGB565
 * @param       red: 红色分量，范围 0-255
 * @param       green: 绿色分量，范围 0-255
 * @param       blue: 蓝色分量，范围 0-255
 * @retval      uint16_t: RGB565 颜色值
 */
uint16_t Display_GfxMakeRgb565(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief       清屏
 * @param       color: 填充颜色，格式为 RGB565
 * @retval      Display_GfxResult_t: 绘图结果
 */
Display_GfxResult_t Display_GfxClear(uint16_t color);

/**
 * @brief       画单个像素点
 * @param       x: 像素点 X 坐标
 * @param       y: 像素点 Y 坐标
 * @param       color: 像素颜色，格式为 RGB565
 * @retval      Display_GfxResult_t: 绘图结果
 */
Display_GfxResult_t Display_GfxDrawPixel(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief       绘制一个 ASCII 字符
 * @param       x: 字符左上角 X 坐标
 * @param       y: 字符左上角 Y 坐标
 * @param       ch: 待绘制字符
 * @param       color: 字符颜色，格式为 RGB565
 * @param       scale: 字符缩放倍数
 * @retval      Display_GfxResult_t: 绘图结果
 */
Display_GfxResult_t Display_GfxDrawChar(uint16_t x, uint16_t y, char ch, uint16_t color, uint8_t scale);

/**
 * @brief       绘制 ASCII 字符串
 * @param       x: 字符串左上角 X 坐标
 * @param       y: 字符串左上角 Y 坐标
 * @param       text: 字符串指针
 * @param       color: 字符串颜色，格式为 RGB565
 * @param       scale: 字符缩放倍数
 * @retval      Display_GfxResult_t: 绘图结果
 */
Display_GfxResult_t Display_GfxDrawString(uint16_t x, uint16_t y, const char *text, uint16_t color, uint8_t scale);

/**
 * @brief       填充矩形区域
 * @param       x: 矩形左上角 X 坐标
 * @param       y: 矩形左上角 Y 坐标
 * @param       width: 矩形宽度
 * @param       height: 矩形高度
 * @param       color: 填充颜色，格式为 RGB565
 * @retval      Display_GfxResult_t: 绘图结果
 */
Display_GfxResult_t Display_GfxFillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);

/**
 * @brief       画水平线
 * @param       x: 起点 X 坐标
 * @param       y: 起点 Y 坐标
 * @param       length: 线段长度
 * @param       color: 线段颜色，格式为 RGB565
 * @retval      Display_GfxResult_t: 绘图结果
 */
Display_GfxResult_t Display_GfxDrawHLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color);

/**
 * @brief       画垂直线
 * @param       x: 起点 X 坐标
 * @param       y: 起点 Y 坐标
 * @param       length: 线段长度
 * @param       color: 线段颜色，格式为 RGB565
 * @retval      Display_GfxResult_t: 绘图结果
 */
Display_GfxResult_t Display_GfxDrawVLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color);

/**
 * @brief       画任意直线
 * @param       x1: 起点 X 坐标
 * @param       y1: 起点 Y 坐标
 * @param       x2: 终点 X 坐标
 * @param       y2: 终点 Y 坐标
 * @param       color: 线段颜色，格式为 RGB565
 * @retval      Display_GfxResult_t: 绘图结果
 */
Display_GfxResult_t Display_GfxDrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

/**
 * @brief       画矩形边框
 * @param       x: 矩形左上角 X 坐标
 * @param       y: 矩形左上角 Y 坐标
 * @param       width: 矩形宽度
 * @param       height: 矩形高度
 * @param       color: 边框颜色，格式为 RGB565
 * @retval      Display_GfxResult_t: 绘图结果
 */
Display_GfxResult_t Display_GfxDrawRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);

/**
 * @brief       画带填充色的矩形框
 * @param       x: 矩形左上角 X 坐标
 * @param       y: 矩形左上角 Y 坐标
 * @param       width: 矩形宽度
 * @param       height: 矩形高度
 * @param       border_color: 边框颜色，格式为 RGB565
 * @param       fill_color: 填充颜色，格式为 RGB565
 * @retval      Display_GfxResult_t: 绘图结果
 */
Display_GfxResult_t Display_GfxDrawFrame(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t border_color, uint16_t fill_color);

/**
 * @brief       画圆形边框
 * @param       x0: 圆心 X 坐标
 * @param       y0: 圆心 Y 坐标
 * @param       radius: 半径
 * @param       color: 边框颜色，格式为 RGB565
 * @retval      Display_GfxResult_t: 绘图结果
 */
Display_GfxResult_t Display_GfxDrawCircle(uint16_t x0, uint16_t y0, uint16_t radius, uint16_t color);

/**
 * @brief       画实心圆
 * @param       x0: 圆心 X 坐标
 * @param       y0: 圆心 Y 坐标
 * @param       radius: 半径
 * @param       color: 填充颜色，格式为 RGB565
 * @retval      Display_GfxResult_t: 绘图结果
 */
Display_GfxResult_t Display_GfxFillCircle(uint16_t x0, uint16_t y0, uint16_t radius, uint16_t color);

/**
 * @brief       画进度条
 * @param       x: 进度条左上角 X 坐标
 * @param       y: 进度条左上角 Y 坐标
 * @param       width: 进度条宽度
 * @param       height: 进度条高度
 * @param       value: 当前值
 * @param       max_value: 最大值
 * @param       active_color: 已完成区域颜色，格式为 RGB565
 * @param       empty_color: 未完成区域颜色，格式为 RGB565
 * @param       border_color: 边框颜色，格式为 RGB565
 * @retval      Display_GfxResult_t: 绘图结果
 */
Display_GfxResult_t Display_GfxDrawProgressBar(uint16_t x,
                                               uint16_t y,
                                               uint16_t width,
                                               uint16_t height,
                                               uint16_t value,
                                               uint16_t max_value,
                                               uint16_t active_color,
                                               uint16_t empty_color,
                                               uint16_t border_color);

/**
 * @brief       画状态圆点
 * @param       x: 圆心 X 坐标
 * @param       y: 圆心 Y 坐标
 * @param       radius: 半径
 * @param       fill_color: 填充颜色，格式为 RGB565
 * @param       border_color: 边框颜色，格式为 RGB565
 * @retval      Display_GfxResult_t: 绘图结果
 */
Display_GfxResult_t Display_GfxDrawStatusDot(uint16_t x,
                                             uint16_t y,
                                             uint16_t radius,
                                             uint16_t fill_color,
                                             uint16_t border_color);

/**
 * @brief       画规则网格
 * @param       x: 网格左上角 X 坐标
 * @param       y: 网格左上角 Y 坐标
 * @param       width: 网格宽度
 * @param       height: 网格高度
 * @param       cell_width: 单元格宽度
 * @param       cell_height: 单元格高度
 * @param       color: 网格线颜色，格式为 RGB565
 * @retval      Display_GfxResult_t: 绘图结果
 */
Display_GfxResult_t Display_GfxDrawGrid(uint16_t x,
                                        uint16_t y,
                                        uint16_t width,
                                        uint16_t height,
                                        uint16_t cell_width,
                                        uint16_t cell_height,
                                        uint16_t color);

#ifdef __cplusplus
}
#endif

#endif
