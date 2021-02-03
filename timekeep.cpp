
#include "TimerOne.h"

#include "common.h"

// Start at a weird offset to catch timer overflow errors early.
unsigned int _millis_counter = 0xe800;

static void on_periodic()
{
    _millis_counter += TIMER_PERIOD;
    digitalWrite(DEBUG_LED, LOW);
    digitalWrite(DEBUG_LED, HIGH);
}

void timekeep_init()
{
    Timer1.initialize(TIMER_PERIOD * 1000ul);
    Timer1.attachInterrupt(on_periodic);
}
