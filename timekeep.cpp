
#include "TimerOne.h"

#include "common.h"

// Timer1 supports periods up to about 8.3 seconds.
// Timer2 supports periods up to 32 milliseconds.
#define TIMER_PERIOD_MS 100

#define TIMER_PERIOD_US ( TIMER_PERIOD_MS * 1000ul )

// Start at a weird offset to catch timer overflow errors early.
unsigned int _millis_counter = 0xe800;

static void on_periodic() {
    _millis_counter += TIMER_PERIOD_MS;
}

void timekeep_init() {
    Timer1.initialize(TIMER_PERIOD_US);
    Timer1.attachInterrupt(on_periodic);
}
