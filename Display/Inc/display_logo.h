#ifndef DISPLAY_LOGO_H
#define DISPLAY_LOGO_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define DISPLAY_HEADER_LOGO_WIDTH             64U
#define DISPLAY_HEADER_LOGO_HEIGHT            64U
#define DISPLAY_HEADER_LOGO_TRANSPARENT_COLOR 0x0000U

/* Raw RGB565 (little-endian) pixel data — 0x0000 is the transparent colour. */
extern const uint8_t display_logo_rgb565_data[DISPLAY_HEADER_LOGO_WIDTH * DISPLAY_HEADER_LOGO_HEIGHT * 2U];

#ifdef __cplusplus
}
#endif

#endif
