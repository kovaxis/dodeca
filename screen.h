
#ifndef FONT_H
#define FONT_H

#include "common.h"

void scr_clear(void);
void scr_draw_bat_sprite(BatStatus bat_status);
void scr_draw(int orient, int pos, int ch);
void scr_show(void);
void scr_force_swap(void);
void scr_resend_frontbuf(void);

#endif
