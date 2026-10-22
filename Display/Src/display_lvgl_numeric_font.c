/**
 * @file display_lvgl_numeric_font.c
 * @brief Display层数字等宽字体代理；字形位图仍由现有Montserrat提供。
 * @note 数字与数字相邻字符之间禁用kerning；所有适配数据只读，无新增堆分配。
 *       仅依赖LVGL，不改变数据精度、协议、任务周期或绘制预算。
 */
#include "display_lvgl_numeric_font.h"

/** @brief 只读字体配置；步进覆盖当前字库的数字实际位图宽度及未加kerning步进。 */
typedef struct {
  const lv_font_t *base; /**< 提供原始位图与度量的字库。 */
  uint16_t advance_px; /**< 等宽字符的水平步进，单位px。 */
  uint8_t hexadecimal; /**< 0仅0-9；1同时包含A-F和ASCII空格。 */
} Display_NumericFontConfig_t;

static const Display_NumericFontConfig_t s_numeric_12 = { &lv_font_montserrat_12, 8U, 0U };
static const Display_NumericFontConfig_t s_numeric_14 = { &lv_font_montserrat_14, 10U, 0U };
static const Display_NumericFontConfig_t s_numeric_16 = { &lv_font_montserrat_16, 11U, 0U };
static const Display_NumericFontConfig_t s_hex_14 = { &lv_font_montserrat_14, 12U, 1U };

/** @brief 判定会随数值变化的十进制数字。 */
static uint8_t Display_NumericIsDigit(uint32_t letter)
{
  return (uint8_t)(letter >= '0' && letter <= '9');
}

/** @brief HEX行的空格与半字节字符必须占同一格，稳定逐字节列位置。 */
static uint8_t Display_NumericIsFixed(const Display_NumericFontConfig_t *config, uint32_t letter)
{
  return (uint8_t)(Display_NumericIsDigit(letter) ||
    (config->hexadecimal && ((letter >= 'A' && letter <= 'F') || letter == ' ')));
}

/**
 * @brief 复用原始度量，仅统一数字步进并将原字形居中；不缩放或复制位图。
 * @note 若后一个字符是数字，同样关闭前一个字符的kerning，避免前缀随数字变化。
 */
static bool Display_NumericGlyphDsc(const lv_font_t *font, lv_font_glyph_dsc_t *out,
                                   uint32_t letter, uint32_t next)
{
  const Display_NumericFontConfig_t *config = (const Display_NumericFontConfig_t *)font->dsc;
  uint8_t fixed = Display_NumericIsFixed(config, letter);
  if (!config->base->get_glyph_dsc(config->base, out, letter,
      (fixed || Display_NumericIsFixed(config, next)) ? 0U : next)) { return false; }
  if (fixed) {
    out->ofs_x = (int16_t)(((int32_t)config->advance_px - out->box_w) / 2);
    out->adv_w = config->advance_px;
  }
  return true;
}

/** @brief 位图直接转交原字体，避免新增Flash字形或字体解析缓冲。 */
static const uint8_t *Display_NumericGlyphBitmap(const lv_font_t *font, uint32_t letter)
{
  const Display_NumericFontConfig_t *config = (const Display_NumericFontConfig_t *)font->dsc;
  return config->base->get_glyph_bitmap(config->base, letter);
}

/* 行高、基线和下划线参数保持与当前原始Montserrat一致；字库更新时须一起复核。 */
const lv_font_t display_lvgl_numeric_12 = {
  Display_NumericGlyphDsc, Display_NumericGlyphBitmap, 15, 3, LV_FONT_SUBPX_NONE,
  -1, 1, &s_numeric_12, NULL, NULL
};
const lv_font_t display_lvgl_numeric_14 = {
  Display_NumericGlyphDsc, Display_NumericGlyphBitmap, 16, 3, LV_FONT_SUBPX_NONE,
  -1, 1, &s_numeric_14, NULL, NULL
};
const lv_font_t display_lvgl_numeric_16 = {
  Display_NumericGlyphDsc, Display_NumericGlyphBitmap, 18, 3, LV_FONT_SUBPX_NONE,
  -1, 1, &s_numeric_16, NULL, NULL
};
const lv_font_t display_lvgl_hex_14 = {
  Display_NumericGlyphDsc, Display_NumericGlyphBitmap, 16, 3, LV_FONT_SUBPX_NONE,
  -1, 1, &s_hex_14, NULL, NULL
};

/** @brief 将当前标签/直接绘制使用的字体映射为只读数字代理，允许重复调用。 */
const lv_font_t *Display_LvglNumericFont(const lv_font_t *font)
{
  if (font == &lv_font_montserrat_12) { return &display_lvgl_numeric_12; }
  if (font == &lv_font_montserrat_14) { return &display_lvgl_numeric_14; }
  if (font == &lv_font_montserrat_16) { return &display_lvgl_numeric_16; }
  return font;
}
