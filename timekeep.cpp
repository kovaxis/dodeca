
#include "TimerOne.h"  // Library: "TimerOne" v1.1.0
#include "common.h"

// Start at a weird offset to catch timer overflow errors early.
volatile unsigned int _millis_counter = 0xe800;

static void on_periodic() { _millis_counter += TIMER_PERIOD; }

void timekeep_init() {
    Timer1.initialize(TIMER_PERIOD * 1000ul);
    Timer1.attachInterrupt(on_periodic);
}
