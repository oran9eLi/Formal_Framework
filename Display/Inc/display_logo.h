#ifndef DISPLAY_LOGO_H
#define DISPLAY_LOGO_H

#include <stdint.h>
#include "display.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DISPLAY_LOGO_WIDTH   550U
#define DISPLAY_LOGO_HEIGHT  326U
#define DISPLAY_LOGO_TRANSPARENT_COLOR 0x0000U

extern const uint8_t s_logo_data[];

Display_Result_t Display_PagesDrawLogoLayout(void);

#ifdef __cplusplus
}
#endif

#endif
