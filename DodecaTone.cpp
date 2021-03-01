// From Arduino core Tone.cpp, modified and specialized to produce the alarm beep pattern.

#include "common.h"

volatile uint8_t *tone_PINx_reg;
volatile uint8_t tone_pin_bitmask;

void dodecaToneSetup() {
    digitalWrite(TONE_PIN, LOW);
    pinMode(TONE_PIN, OUTPUT);
    // Warning: constant output HIGH could burn the buzzer! (there's no DC blocking capacitor)

    // Precompute this for quick toggling (so ISR is short)
    uint8_t port = digitalPinToPort(TONE_PIN);
    tone_PINx_reg = portInputRegister(port);
    tone_pin_bitmask = digitalPinToBitMask(TONE_PIN);
}

volatile long toggle_count;

volatile bool alarm_pattern;
volatile bool make_sound;
volatile int remaining_beeps;
volatile bool beep_period_silence;
volatile bool beep_seq_silence;
volatile long initial_toggle_count;

void dodecaTone(unsigned int frequency, unsigned long duration) {
    TCCR2A = 0;
    TCCR2B = 0;
    bitWrite(TCCR2A, WGM21, 1);
    bitWrite(TCCR2B, CS20, 1);

    uint8_t prescalarbits;
    uint32_t ocr = 0;
    // 8 bit timer, scan through prescalars to find the best fit
    ocr = F_CPU / frequency / 2 - 1;
    prescalarbits = 0b001;  // ck/1: same for both timers
    if (ocr > 255)
    {
        ocr = F_CPU / frequency / 2 / 8 - 1;
        prescalarbits = 0b010;  // ck/8: same for both timers

        if (ocr > 255)
        {
            ocr = F_CPU / frequency / 2 / 32 - 1;
            prescalarbits = 0b011;
        }

        if (ocr > 255)
        {
            ocr = F_CPU / frequency / 2 / 64 - 1;
            prescalarbits = 0b100;

            if (ocr > 255)
            {
                ocr = F_CPU / frequency / 2 / 128 - 1;
                prescalarbits = 0b101;
            }

            if (ocr > 255)
            {
                ocr = F_CPU / frequency / 2 / 256 - 1;
                prescalarbits = 0b110;
                if (ocr > 255)
                {
                    // can't do any better than /1024
                    ocr = F_CPU / frequency / 2 / 1024 - 1;
                    prescalarbits = 0b111;
                }
            }
        }
    }
    TCCR2B = (TCCR2B & 0b11111000) | prescalarbits;

    // Calculate the toggle count
    toggle_count = 2 * frequency * duration / 1000;

    // Set the OCR for the given timer,
    // set the toggle count,
    // then turn on the interrupts
    OCR2A = ocr;
    bitWrite(TIMSK2, OCIE2A, 1);

    alarm_pattern = false;
    make_sound = true;
}

void dodecaAlarm() {
    dodecaTone(BEEP_FREQUENCY, BEEP_PERIOD / 2);
    alarm_pattern = true;
    remaining_beeps = NUMBER_OF_BEEPS;
    beep_period_silence = false;
    beep_seq_silence = false;
    initial_toggle_count = toggle_count;
}

void dodecaNoTone() {
    bitWrite(TIMSK2, OCIE2A, 0); // disable interrupt
    TCCR2A = (1 << WGM20);
    TCCR2B = (TCCR2B & 0b11111000) | (1 << CS22);
    OCR2A = 0;

    digitalWrite(TONE_PIN, LOW);
}

ISR(TIMER2_COMPA_vect) {
    if (toggle_count != 0) {
        if (make_sound) {
            *tone_PINx_reg = tone_pin_bitmask;  // toggle the pin
        }
        toggle_count--;
        return;
    }
    if (!alarm_pattern) {
        // Duration expired
        dodecaNoTone();
        return;
    }

    // Playing an alarm pattern

    toggle_count = initial_toggle_count;    // Reset

    // Handle states:

    if (!beep_period_silence) {
        // Was playing beep, now should play silence.
        make_sound = false;
        digitalWrite(TONE_PIN, LOW);
    } else {
        // Was silent, now make sound (if not on a sequence of silences).
        make_sound = !beep_seq_silence;
        remaining_beeps--;
    }
    beep_period_silence = !beep_period_silence;

    if (remaining_beeps == 0) {
        if (!beep_seq_silence) {
            // Was playing a sequence of beeps, now play a sequence of silences.
            make_sound = false;
            digitalWrite(TONE_PIN, LOW);
        } else {
            // Was silent, now make sound.
            make_sound = true;
        }
        beep_seq_silence = !beep_seq_silence;
        remaining_beeps = NUMBER_OF_BEEPS;
    }
}
