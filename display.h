#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <stdint.h>

/* *********************************************************************** */

typedef enum display_mode_e {
    DISPLAY_MODE_OFF,           /* disable display */
    DISPLAY_MODE_RUN_ANIM,      /* show RUN animation */
    DISPLAY_MODE_RUN_ANIM_LOOP, /* change last char of RUN animation */
    DISPLAY_MODE_STATIC,        /* show first 4 chars */
    DISPLAY_MODE_SCROLL,        /* scroll through buffer */
    DISPLAY_MODE_SCROLL_ONCE,   /* scroll once, then switch to RUN_ANIM */
} display_mode_t;

/* *********************************************************************** */

void display_show_string    (const char * p_string, uint8_t append);
void display_show_hex       (uint8_t value, uint8_t append);
void display_set_mode       (display_mode_t mode);
void display_update         (void);

/* *********************************************************************** */

#endif /* DISPLAY_H_ */
