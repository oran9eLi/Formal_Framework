/**
 * @file display_logo.c
 * @brief Draw the built-in startup logo on the ATK-MD0700 display.
 */

#include "display_logo.h"
#include "display_gfx.h"
#include "display_pages.h"

#define DISPLAY_LOGO_CENTER_X  ((uint16_t)((DISPLAY_GFX_WIDTH - DISPLAY_LOGO_WIDTH) / 2U))
#define DISPLAY_LOGO_CENTER_Y  ((uint16_t)((DISPLAY_GFX_HEIGHT - DISPLAY_LOGO_HEIGHT) / 2U))
#define DISPLAY_LOGO_BG_COLOR  0x1C9FU

/**
 * @brief Draw the centered startup logo using the built-in RGB565 bitmap.
 */
Display_Result_t Display_PagesDrawLogoLayout(void)
{
    uint16_t row;
    uint16_t col;
    uint32_t offset;
    uint16_t color;
    uint8_t hi;
    uint8_t lo;

    if (Display_GfxIsReady() == 0U) {
        return DISPLAY_NOT_READY;
    }

    (void)Display_GfxClear(DISPLAY_LOGO_BG_COLOR);

    for (row = 0U; row < DISPLAY_LOGO_HEIGHT; row++) {
        for (col = 0U; col < DISPLAY_LOGO_WIDTH; col++) {
            offset = ((uint32_t)row * (uint32_t)DISPLAY_LOGO_WIDTH + (uint32_t)col) * 2U;
            hi = s_logo_data[offset];
            lo = s_logo_data[offset + 1U];
            color = (uint16_t)(((uint16_t)hi << 8U) | (uint16_t)lo);
            if (color != DISPLAY_LOGO_TRANSPARENT_COLOR) {
                (void)Display_GfxDrawPixel(
                    (uint16_t)(DISPLAY_LOGO_CENTER_X + col),
                    (uint16_t)(DISPLAY_LOGO_CENTER_Y + row),
                    color);
            }
        }
    }

    return DISPLAY_OK;
}
