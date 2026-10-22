/**
 * @file display_lvgl_numeric_font.c
 * @brief Display层数字等宽字体代理；字形位图仍由现有Montserrat提供。
 * @note 仅十进制数字及其相邻字符禁用kerning；HEX列使用独立配置。
 *       所有适配数据只读，无新增堆分配；不改变数据、协议、任务周期或绘制预算。
 */
#include "display_lvgl_numeric_font.h"

/** @brief 只读字体配置；普通文本按数字边界选紧凑步进，HEX列单独覆盖A-F。 */
typedef struct {
  const lv_font_t *base; /**< 提供原始位图与度量的字库。 */
  uint16_t advance_px; /**< 等宽字符的水平步进，单位px。 */
  uint8_t hex_cells; /**< 0仅固定0-9；1还固定A-F及ASCII空格，用于原始HEX行。 */
} Display_NumericFontConfig_t;

/* 普通数字步进覆盖0-9的最大位图宽度及未加kerning步进，不受A-F字母宽度影响。 */
static const Display_NumericFontConfig_t s_numeric_12 = { &lv_font_montserrat_12, 8U, 0U };
static const Display_NumericFontConfig_t s_numeric_14 = { &lv_font_montserrat_14, 10U, 0U };
static const Display_NumericFontConfig_t s_numeric_16 = { &lv_font_montserrat_16, 11U, 0U };
static const Display_NumericFontConfig_t s_hex_14 = { &lv_font_montserrat_14, 12U, 1U };

/** @brief 普通文本仅将0-9识别为数字，避免扩大FAILED等单词的字母间距。 */
static uint8_t Display_NumericIsDigit(uint32_t letter)
{
  return (uint8_t)(letter >= '0' && letter <= '9');
}

/** @brief 仅HEX行将A-F和空格纳入同宽单格；普通英文保留原始字距。 */
static uint8_t Display_NumericIsFixed(const Display_NumericFontConfig_t *config, uint32_t letter)
{
  return (uint8_t)(Display_NumericIsDigit(letter) ||
    (config->hex_cells && ((letter >= 'A' && letter <= 'F') || letter == ' ')));
}

/**
 * @brief 复用原始度量，仅统一所选字符步进并将原字形居中；不缩放或复制位图。
 * @note 若后一个字符固定，同样关闭前一个字符的kerning，避免前缀随数字变化。
 *       普通字母之间完整保留原字库的步进、偏移和kerning。
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
