/**
 * @file display_gfx.c
 * @brief Implement basic RGB565 drawing primitives for the display renderer.
 */

#include "display_gfx.h"

static Display_GfxPort_t s_gfx_port;
static uint8_t s_gfx_ready = 0U;

static const uint8_t s_gfx_font_5x7[][5] = {
    {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU}, /* 0 */
    {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U}, /* 1 */
    {0x42U, 0x61U, 0x51U, 0x49U, 0x46U}, /* 2 */
    {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U}, /* 3 */
    {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U}, /* 4 */
    {0x27U, 0x45U, 0x45U, 0x45U, 0x39U}, /* 5 */
    {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U}, /* 6 */
    {0x01U, 0x71U, 0x09U, 0x05U, 0x03U}, /* 7 */
    {0x36U, 0x49U, 0x49U, 0x49U, 0x36U}, /* 8 */
    {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}, /* 9 */
    {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU}, /* A */
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U}, /* B */
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U}, /* C */
    {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU}, /* D */
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U}, /* E */
    {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U}, /* F */
    {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU}, /* G */
    {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU}, /* H */
    {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U}, /* I */
    {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U}, /* J */
    {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U}, /* K */
    {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U}, /* L */
    {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU}, /* M */
    {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU}, /* N */
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU}, /* O */
    {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U}, /* P */
    {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU}, /* Q */
    {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U}, /* R */
    {0x46U, 0x49U, 0x49U, 0x49U, 0x31U}, /* S */
    {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U}, /* T */
    {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU}, /* U */
    {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU}, /* V */
    {0x7FU, 0x20U, 0x18U, 0x20U, 0x7FU}, /* W */
    {0x63U, 0x14U, 0x08U, 0x14U, 0x63U}, /* X */
    {0x07U, 0x08U, 0x70U, 0x08U, 0x07U}, /* Y */
    {0x61U, 0x51U, 0x49U, 0x45U, 0x43U}  /* Z */
};

/*
 * 判断一个逻辑坐标点是否位于屏幕范围内。
 */
static uint8_t Display_GfxIsPointInScreen(uint16_t x, uint16_t y)
{
  return (uint8_t)((x < DISPLAY_GFX_WIDTH) && (y < DISPLAY_GFX_HEIGHT));
}

/*
 * 获取 ASCII 字符对应的 5x7 点阵字模。
 */
static const uint8_t *Display_GfxGetGlyph(char ch)
{
  static const uint8_t blank[5]   = {0U, 0U, 0U, 0U, 0U};
  static const uint8_t dash[5]    = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
  static const uint8_t colon[5]   = {0x00U, 0x36U, 0x36U, 0x00U, 0x00U};
  static const uint8_t slash[5]   = {0x20U, 0x10U, 0x08U, 0x04U, 0x02U};
  static const uint8_t dot[5]     = {0x00U, 0x60U, 0x60U, 0x00U, 0x00U};
  static const uint8_t percent[5] = {0x23U, 0x13U, 0x08U, 0x64U, 0x62U};
  static const uint8_t degree[5]  = {0x02U, 0x05U, 0x02U, 0x00U, 0x00U};

  if ((ch >= '0') && (ch <= '9')) { return s_gfx_font_5x7[(uint8_t)(ch - '0')]; }

  if ((ch >= 'a') && (ch <= 'z')) { ch = (char)(ch - 'a' + 'A'); }

  if ((ch >= 'A') && (ch <= 'Z')) { return s_gfx_font_5x7[10U + (uint8_t)(ch - 'A')]; }

  if (ch == '-') { return dash; }

  if (ch == ':') { return colon; }

  if (ch == '/') { return slash; }

  if (ch == '.') { return dot; }

  if (ch == '%') { return percent; }

  if ((uint8_t)ch == 0xB0U) { return degree; }

  return blank;
}

/*
 * 通过底层端口写入单个像素点。
 */
static Display_GfxResult_t Display_GfxWritePixel(uint16_t x, uint16_t y, uint16_t color)
{
  if (s_gfx_ready == 0U) { return DISPLAY_GFX_NOT_READY; }

  if (Display_GfxIsPointInScreen(x, y) == 0U) { return DISPLAY_GFX_PARAM_ERROR; }

  if (s_gfx_port.draw_pixel != 0) {
    s_gfx_port.draw_pixel(x, y, color);
  } else {
    s_gfx_port.fill_rect(x, y, 1U, 1U, color);
  }

  return DISPLAY_GFX_OK;
}

/*
 * 绘制圆形算法中的四象限对称点。
 */
static void Display_GfxPlotCirclePoints(uint16_t x0, uint16_t y0, int32_t x, int32_t y, uint16_t color)
{
  int32_t px;
  int32_t py;

  px = (int32_t)x0 + x;
  py = (int32_t)y0 + y;
  if ((px >= 0) && (py >= 0) && (px < (int32_t)DISPLAY_GFX_WIDTH) && (py < (int32_t)DISPLAY_GFX_HEIGHT)) { (void)Display_GfxWritePixel((uint16_t)px, (uint16_t)py, color); }

  px = (int32_t)x0 - x;
  py = (int32_t)y0 + y;
  if ((px >= 0) && (py >= 0) && (px < (int32_t)DISPLAY_GFX_WIDTH) && (py < (int32_t)DISPLAY_GFX_HEIGHT)) { (void)Display_GfxWritePixel((uint16_t)px, (uint16_t)py, color); }

  px = (int32_t)x0 + x;
  py = (int32_t)y0 - y;
  if ((px >= 0) && (py >= 0) && (px < (int32_t)DISPLAY_GFX_WIDTH) && (py < (int32_t)DISPLAY_GFX_HEIGHT)) { (void)Display_GfxWritePixel((uint16_t)px, (uint16_t)py, color); }

  px = (int32_t)x0 - x;
  py = (int32_t)y0 - y;
  if ((px >= 0) && (py >= 0) && (px < (int32_t)DISPLAY_GFX_WIDTH) && (py < (int32_t)DISPLAY_GFX_HEIGHT)) { (void)Display_GfxWritePixel((uint16_t)px, (uint16_t)py, color); }

  px = (int32_t)x0 + y;
  py = (int32_t)y0 + x;
  if ((px >= 0) && (py >= 0) && (px < (int32_t)DISPLAY_GFX_WIDTH) && (py < (int32_t)DISPLAY_GFX_HEIGHT)) { (void)Display_GfxWritePixel((uint16_t)px, (uint16_t)py, color); }

  px = (int32_t)x0 - y;
  py = (int32_t)y0 + x;
  if ((px >= 0) && (py >= 0) && (px < (int32_t)DISPLAY_GFX_WIDTH) && (py < (int32_t)DISPLAY_GFX_HEIGHT)) { (void)Display_GfxWritePixel((uint16_t)px, (uint16_t)py, color); }

  px = (int32_t)x0 + y;
  py = (int32_t)y0 - x;
  if ((px >= 0) && (py >= 0) && (px < (int32_t)DISPLAY_GFX_WIDTH) && (py < (int32_t)DISPLAY_GFX_HEIGHT)) { (void)Display_GfxWritePixel((uint16_t)px, (uint16_t)py, color); }

  px = (int32_t)x0 - y;
  py = (int32_t)y0 - x;
  if ((px >= 0) && (py >= 0) && (px < (int32_t)DISPLAY_GFX_WIDTH) && (py < (int32_t)DISPLAY_GFX_HEIGHT)) { (void)Display_GfxWritePixel((uint16_t)px, (uint16_t)py, color); }
}

/*
 * 注册底层画点和填充矩形端口。
 */
Display_GfxResult_t Display_GfxInit(const Display_GfxPort_t *port)
{
  if ((port == 0) || ((port->draw_pixel == 0) && (port->fill_rect == 0))) {
    s_gfx_ready           = 0U;
    s_gfx_port.draw_pixel = 0;
    s_gfx_port.fill_rect  = 0;
    return DISPLAY_GFX_PARAM_ERROR;
  }

  s_gfx_port  = *port;
  s_gfx_ready = 1U;
  return DISPLAY_GFX_OK;
}

/*
 * 查询图元层是否已经完成端口注册。
 */
uint8_t Display_GfxIsReady(void)
{
  return s_gfx_ready;
}

/*
 * 将 8 位 RGB 分量转换为 RGB565 颜色值。
 */
uint16_t Display_GfxMakeRgb565(uint8_t red, uint8_t green, uint8_t blue)
{
  uint16_t color;

  color = (uint16_t)(((uint16_t)(red & 0xF8U) << 8) | ((uint16_t)(green & 0xFCU) << 3) | ((uint16_t)blue >> 3));
  return color;
}

/*
 * 使用底层填充接口清空整屏。
 */
Display_GfxResult_t Display_GfxClear(uint16_t color)
{
  return Display_GfxFillRect(0U, 0U, DISPLAY_GFX_WIDTH, DISPLAY_GFX_HEIGHT, color);
}

/*
 * 绘制单个像素点。
 */
Display_GfxResult_t Display_GfxDrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  return Display_GfxWritePixel(x, y, color);
}

/*
 * 按指定缩放倍数绘制一个 ASCII 字符串
 */
Display_GfxResult_t Display_GfxDrawChar(uint16_t x, uint16_t y, char ch, uint16_t color, uint8_t scale)
{
  const uint8_t *glyph;
  uint8_t col;
  uint8_t row;

  if (s_gfx_ready == 0U) { return DISPLAY_GFX_NOT_READY; }

  if (scale == 0U) { return DISPLAY_GFX_PARAM_ERROR; }

  if (((uint32_t)x + (5U * scale)) > DISPLAY_GFX_WIDTH || ((uint32_t)y + (7U * scale)) > DISPLAY_GFX_HEIGHT) { return DISPLAY_GFX_PARAM_ERROR; }

  glyph = Display_GfxGetGlyph(ch);
  for (col = 0U; col < 5U; col++) {
    for (row = 0U; row < 7U; row++) {
      if ((glyph[col] & (uint8_t)(1U << row)) != 0U) { (void)Display_GfxFillRect((uint16_t)(x + ((uint16_t)col * scale)), (uint16_t)(y + ((uint16_t)row * scale)), scale, scale, color); }
    }
  }

  return DISPLAY_GFX_OK;
}

/*
 * 从左到右绘制 ASCII 字符串。
 */
Display_GfxResult_t Display_GfxDrawString(uint16_t x, uint16_t y, const char *text, uint16_t color, uint8_t scale)
{
  uint16_t cursor_x = x;

  if (text == 0) { return DISPLAY_GFX_PARAM_ERROR; }

  while (*text != '\0') {
    if (Display_GfxDrawChar(cursor_x, y, *text, color, scale) != DISPLAY_GFX_OK) { return DISPLAY_GFX_PARAM_ERROR; }

    cursor_x = (uint16_t)(cursor_x + (6U * scale));
    text++;
  }

  return DISPLAY_GFX_OK;
}

/*
 * 填充并裁剪一个屏幕逻辑矩形框
 */
Display_GfxResult_t Display_GfxFillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
  uint16_t clipped_width;
  uint16_t clipped_height;
  uint16_t row;
  uint16_t col;

  if (s_gfx_ready == 0U) { return DISPLAY_GFX_NOT_READY; }

  if ((width == 0U) || (height == 0U) || (x >= DISPLAY_GFX_WIDTH) || (y >= DISPLAY_GFX_HEIGHT)) { return DISPLAY_GFX_PARAM_ERROR; }

  clipped_width  = width;
  clipped_height = height;
  if (((uint32_t)x + clipped_width) > DISPLAY_GFX_WIDTH) { clipped_width = (uint16_t)(DISPLAY_GFX_WIDTH - x); }
  if (((uint32_t)y + clipped_height) > DISPLAY_GFX_HEIGHT) { clipped_height = (uint16_t)(DISPLAY_GFX_HEIGHT - y); }

  if (s_gfx_port.fill_rect != 0) {
    s_gfx_port.fill_rect(x, y, clipped_width, clipped_height, color);
    return DISPLAY_GFX_OK;
  }

  for (row = 0U; row < clipped_height; row++) {
    for (col = 0U; col < clipped_width; col++) { s_gfx_port.draw_pixel((uint16_t)(x + col), (uint16_t)(y + row), color); }
  }

  return DISPLAY_GFX_OK;
}

/*
 * 绘制水平线段。
 */
Display_GfxResult_t Display_GfxDrawHLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color)
{
  return Display_GfxFillRect(x, y, length, 1U, color);
}

/*
 * 绘制垂直线段。
 */
Display_GfxResult_t Display_GfxDrawVLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color)
{
  return Display_GfxFillRect(x, y, 1U, length, color);
}

/*
 * 使用 Bresenham 算法绘制任意直线。
 */
Display_GfxResult_t Display_GfxDrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
  int32_t sx;
  int32_t sy;
  int32_t dx;
  int32_t dy;
  int32_t err;
  int32_t err2;
  int32_t x;
  int32_t y;

  if (s_gfx_ready == 0U) { return DISPLAY_GFX_NOT_READY; }

  if ((Display_GfxIsPointInScreen(x1, y1) == 0U) || (Display_GfxIsPointInScreen(x2, y2) == 0U)) { return DISPLAY_GFX_PARAM_ERROR; }

  x   = (int32_t)x1;
  y   = (int32_t)y1;
  dx  = ((int32_t)x1 > (int32_t)x2) ? ((int32_t)x1 - (int32_t)x2) : ((int32_t)x2 - (int32_t)x1);
  dy  = ((int32_t)y1 > (int32_t)y2) ? ((int32_t)y1 - (int32_t)y2) : ((int32_t)y2 - (int32_t)y1);
  sx  = (x1 < x2) ? 1 : -1;
  sy  = (y1 < y2) ? 1 : -1;
  err = dx - dy;

  while (1) {
    (void)Display_GfxWritePixel((uint16_t)x, (uint16_t)y, color);
    if ((x == (int32_t)x2) && (y == (int32_t)y2)) { break; }

    err2 = err * 2;
    if (err2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (err2 < dx) {
      err += dx;
      y += sy;
    }
  }

  return DISPLAY_GFX_OK;
}

/*
 * 绘制矩形边框。
 */
Display_GfxResult_t Display_GfxDrawRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
  Display_GfxResult_t result;

  if (s_gfx_ready == 0U) { return DISPLAY_GFX_NOT_READY; }

  if ((width < 2U) || (height < 2U) || (x >= DISPLAY_GFX_WIDTH) || (y >= DISPLAY_GFX_HEIGHT) || (((uint32_t)x + width) > DISPLAY_GFX_WIDTH) || (((uint32_t)y + height) > DISPLAY_GFX_HEIGHT)) { return DISPLAY_GFX_PARAM_ERROR; }

  result = Display_GfxDrawHLine(x, y, width, color);
  if (result != DISPLAY_GFX_OK) { return result; }

  result = Display_GfxDrawHLine(x, (uint16_t)(y + height - 1U), width, color);
  if (result != DISPLAY_GFX_OK) { return result; }

  result = Display_GfxDrawVLine(x, y, height, color);
  if (result != DISPLAY_GFX_OK) { return result; }

  result = Display_GfxDrawVLine((uint16_t)(x + width - 1U), y, height, color);
  if (result != DISPLAY_GFX_OK) { return result; }

  return DISPLAY_GFX_OK;
}

/*
 * 绘制带边框和填充色的矩形框。
 */
Display_GfxResult_t Display_GfxDrawFrame(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t border_color, uint16_t fill_color)
{
  Display_GfxResult_t result;

  if ((width < 2U) || (height < 2U)) { return DISPLAY_GFX_PARAM_ERROR; }

  result = Display_GfxFillRect(x, y, width, height, fill_color);
  if (result != DISPLAY_GFX_OK) { return result; }

  return Display_GfxDrawRect(x, y, width, height, border_color);
}

/*
 * 绘制空心圆。
 */
Display_GfxResult_t Display_GfxDrawCircle(uint16_t x0, uint16_t y0, uint16_t radius, uint16_t color)
{
  int32_t x;
  int32_t y;
  int32_t decision;

  if (s_gfx_ready == 0U) { return DISPLAY_GFX_NOT_READY; }

  if ((radius == 0U) || (Display_GfxIsPointInScreen(x0, y0) == 0U)) { return DISPLAY_GFX_PARAM_ERROR; }

  x        = 0;
  y        = (int32_t)radius;
  decision = 3 - (2 * (int32_t)radius);

  while (x <= y) {
    Display_GfxPlotCirclePoints(x0, y0, x, y, color);
    if (decision < 0) {
      decision += (4 * x) + 6;
    } else {
      decision += (4 * (x - y)) + 10;
      y--;
    }
    x++;
  }

  return DISPLAY_GFX_OK;
}

/*
 * 绘制实心圆。
 */
Display_GfxResult_t Display_GfxFillCircle(uint16_t x0, uint16_t y0, uint16_t radius, uint16_t color)
{
  int32_t x;
  int32_t y;
  int32_t decision;
  int32_t left;
  int32_t top;
  uint16_t length;

  if (s_gfx_ready == 0U) { return DISPLAY_GFX_NOT_READY; }

  if ((radius == 0U) || (Display_GfxIsPointInScreen(x0, y0) == 0U)) { return DISPLAY_GFX_PARAM_ERROR; }

  x        = 0;
  y        = (int32_t)radius;
  decision = 3 - (2 * (int32_t)radius);

  while (x <= y) {
    left   = (int32_t)x0 - x;
    length = (uint16_t)((2 * x) + 1);
    top    = (int32_t)y0 + y;
    if ((left >= 0) && (top >= 0) && (top < (int32_t)DISPLAY_GFX_HEIGHT)) { (void)Display_GfxDrawHLine((uint16_t)left, (uint16_t)top, length, color); }
    top = (int32_t)y0 - y;
    if ((left >= 0) && (top >= 0) && (top < (int32_t)DISPLAY_GFX_HEIGHT)) { (void)Display_GfxDrawHLine((uint16_t)left, (uint16_t)top, length, color); }

    left   = (int32_t)x0 - y;
    length = (uint16_t)((2 * y) + 1);
    top    = (int32_t)y0 + x;
    if ((left >= 0) && (top >= 0) && (top < (int32_t)DISPLAY_GFX_HEIGHT)) { (void)Display_GfxDrawHLine((uint16_t)left, (uint16_t)top, length, color); }
    top = (int32_t)y0 - x;
    if ((left >= 0) && (top >= 0) && (top < (int32_t)DISPLAY_GFX_HEIGHT)) { (void)Display_GfxDrawHLine((uint16_t)left, (uint16_t)top, length, color); }

    if (decision < 0) {
      decision += (4 * x) + 6;
    } else {
      decision += (4 * (x - y)) + 10;
      y--;
    }
    x++;
  }

  return DISPLAY_GFX_OK;
}

/*
 * 绘制带边框的水平进度条。
 */
Display_GfxResult_t Display_GfxDrawProgressBar(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t value, uint16_t max_value, uint16_t active_color, uint16_t empty_color, uint16_t border_color)
{
  uint16_t inner_width;
  uint16_t inner_height;
  uint16_t active_width;
  uint32_t scaled_width;
  Display_GfxResult_t result;

  if ((width < 4U) || (height < 4U) || (max_value == 0U)) { return DISPLAY_GFX_PARAM_ERROR; }

  if (value > max_value) { value = max_value; }

  inner_width  = (uint16_t)(width - 2U);
  inner_height = (uint16_t)(height - 2U);
  scaled_width = ((uint32_t)inner_width * value) / max_value;
  active_width = (uint16_t)scaled_width;

  result = Display_GfxDrawRect(x, y, width, height, border_color);
  if (result != DISPLAY_GFX_OK) { return result; }

  result = Display_GfxFillRect((uint16_t)(x + 1U), (uint16_t)(y + 1U), inner_width, inner_height, empty_color);
  if (result != DISPLAY_GFX_OK) { return result; }

  if (active_width > 0U) {
    result = Display_GfxFillRect((uint16_t)(x + 1U), (uint16_t)(y + 1U), active_width, inner_height, active_color);
    if (result != DISPLAY_GFX_OK) { return result; }
  }

  return DISPLAY_GFX_OK;
}

/*
 * 绘制状态圆点。
 */
Display_GfxResult_t Display_GfxDrawStatusDot(uint16_t x, uint16_t y, uint16_t radius, uint16_t fill_color, uint16_t border_color)
{
  Display_GfxResult_t result;

  if (radius < 2U) { return DISPLAY_GFX_PARAM_ERROR; }

  result = Display_GfxFillCircle(x, y, radius, fill_color);
  if (result != DISPLAY_GFX_OK) { return result; }

  return Display_GfxDrawCircle(x, y, radius, border_color);
}

/*
 * 绘制规则网格线。
 */
Display_GfxResult_t Display_GfxDrawGrid(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t cell_width, uint16_t cell_height, uint16_t color)
{
  uint16_t offset;
  Display_GfxResult_t result;

  if ((width == 0U) || (height == 0U) || (cell_width == 0U) || (cell_height == 0U)) { return DISPLAY_GFX_PARAM_ERROR; }

  for (offset = 0U; offset <= width; offset = (uint16_t)(offset + cell_width)) {
    result = Display_GfxDrawVLine((uint16_t)(x + offset), y, height, color);
    if (result != DISPLAY_GFX_OK) { return result; }

    if ((uint16_t)(width - offset) < cell_width) { break; }
  }

  for (offset = 0U; offset <= height; offset = (uint16_t)(offset + cell_height)) {
    result = Display_GfxDrawHLine(x, (uint16_t)(y + offset), width, color);
    if (result != DISPLAY_GFX_OK) { return result; }

    if ((uint16_t)(height - offset) < cell_height) { break; }
  }

  return DISPLAY_GFX_OK;
}
