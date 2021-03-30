#ifndef DODECATONE_H
#define DODECATONE_H

#include "common.h"

extern volatile bool tone_playing;
void dodecaToneSetup(void);
void dodecaTonePlay(Tone* sequence);
void dodecaToneStop(void);

#endif
