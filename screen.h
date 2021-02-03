
#ifndef FONT_H
#define FONT_H

#include "common.h"

void scr_clear(void);
void scr_draw_lowbat(int orient);
void scr_draw(int orient, int pos, int ch);
void scr_show(void);
void scr_force_swap(void);

#endif
