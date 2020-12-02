
#ifndef FONT_H
#define FONT_H

#include "common.h"

const int ORIENT_000 = 0;
const int ORIENT_072 = 1;
const int ORIENT_144 = 2;
const int ORIENT_216 = 3;
const int ORIENT_288 = 4;

void scr_clear(void);
void scr_draw(int orient, int pos, int ch);
void scr_show(void);

#endif
