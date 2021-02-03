/*
 * Tiny4kOLED - Drivers for SSD1306 controlled dot matrix OLED/PLED 128x32 displays
 *
 * Based on ssd1306xled, re-written and extended by Stephen Denne
 * from 2017-04-25 at https://github.com/datacute/Tiny4kOLED
 *
 */
#ifndef TINY4KOLED_H
#define TINY4KOLED_H

#if defined(TwoWire_h) || defined(USIWire_h)
#include "T4K_Wire.h"
#elif defined(TinyWireM_h)
#include "T4K_TinyWireM.h"
#elif defined(TinyI2CMaster_h)
#include "T4K_tiny-i2c.h"
#else
#include <Wire.h>
#include "T4K_Wire.h"
#endif

#endif
