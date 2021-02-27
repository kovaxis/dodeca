
#include "common.h"
#include <avr/power.h>
#include "T4K/T4K.h"

#include "BMA400.h"
#include "LowPower.h"
#include "screen.h"
#include <EnableInterrupt.h>    // Library: "EnableInterrupt", v1.1.0

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

static byte current_face = 0;
static byte current_orient = ORIENT_000;
static Instant timer_ref = Instant();
static int remaining_seconds = 0;
static Vec3<int> rollavg_buf[ROLLAVG_LEN];
static byte rollavg_idx = 0;
static int low_battery_frames = -1;

#ifdef DISPLAY_VOLTAGE_ON_SCREEN
static int battery_display_voltage = 0;
#endif

/** Turns off the currently active screen, and if not 0, turns `screen` on. */
void select_screen(int screen)
{
    static Instant last_screen_change = Instant();
    if (screen != SSD1306)
    {
        unsigned long millis_debounce = last_screen_change.elapsed().as_millis();
        if (millis_debounce < 0 || millis_debounce >= OLED_POWER_DEBOUNCE)
        {
            if (SSD1306 != 0)
            {
                oled.forceOff();
            }
            SSD1306 = screen;
            if (screen != 0)
            {
                scr_clear();
                scr_force_swap();
                oled.fill(0);
                oled.forceOn();
            }
            last_screen_change = Instant();
        }
    }
}

static void draw_bat_charge(BatStatus bat_status) {
    scr_clear();
    scr_draw_bat_sprite(bat_status);
    scr_show();
}

static void drawScreen(int seconds, bool show)
{
    //Debug hatch pattern to test screen
#ifdef DEBUG_DRAW_HATCH
    for (int y = 0; y < 8; y += 1)
    {
        oled.setCursor(0, y);
        oled.startData();
        for (int x = 0; x < 64; x += 1)
        {
            oled.sendData(DEBUG_HATCH_PATTERN1);
            oled.sendData(DEBUG_HATCH_PATTERN2);
        }
        oled.endData();
    }
    return;
#endif

    byte orient = current_orient;

    if (low_battery_frames != -1)
    {
        //Draw low battery indicator instead of the normal timer
        if (low_battery_frames == 0)
        {
            scr_clear();
            draw_bat_charge(BAT_LOW);
            scr_show();
        }

        if (low_battery_frames % LOW_BATTERY_BLINK_PERIOD < LOW_BATTERY_BLINK_ONFRAMES)
        {
            oled.on();
        }
        else
        {
            oled.off();
        }

        //Advance low battery frame animation
        low_battery_frames += 1;
        if (low_battery_frames >= LOW_BATTERY_FRAMES)
        {
            low_battery_frames = -1;
        }

        return;
    }

    static int last_seconds = -1;
    static bool last_show = false;
    static byte last_orient = -1;
    if (last_seconds == seconds && last_show == show && last_orient == current_orient)
    {
        return;
    }
    else
    {
        last_seconds = seconds;
        last_show = show;
        last_orient = orient;
    }

    if (show)
    {
        oled.on();
        scr_clear();
#ifdef DISPLAY_VOLTAGE_ON_SCREEN
        int voltage = battery_display_voltage;
        scr_draw(orient, 4, voltage % 10);
        voltage /= 10;
        scr_draw(orient, 3, voltage % 10);
        voltage /= 10;
        scr_draw(orient, 1, voltage % 10);
        voltage /= 10;
        scr_draw(orient, 0, voltage % 10);
#else
        int time = seconds;
        scr_draw(orient, 4, time % 10);
        time /= 10;
        scr_draw(orient, 3, time % 6);
        time /= 6;
        scr_draw(orient, 2, 10);
        scr_draw(orient, 1, time % 10);
        time /= 10;
        scr_draw(orient, 0, time % 10);
#endif
        scr_show();
    }
    else
    {
        oled.off();
    }
}

BlueDot_BMA400 bma400 = BlueDot_BMA400(BMA400_ADDRESS);

void setup()
{
    //select_adc_pin(BATTERY_VOLTAGE_PIN);
    //Switch off analog comparator
    ACSR = 0x80;
    //Switch off analog-to-digital
    ADCSRA = 0;
    //Switch off digital pin input buffers
    DIDR0 = 0b00111111;
    DIDR1 = 0b00000011;
    //Switch off unused modules
    power_adc_disable();
    power_spi_disable();
    power_timer2_disable();

#ifndef DEBUG_PROFILE_IDLE
    power_timer0_disable();
#endif

    timekeep_init();

#ifdef DEBUG_SERIAL
    Serial.begin(115200);
#else
    power_usart0_disable();
#endif

    //Disable pull-up on RX pin, since when the USB2SERIAL chip is off its TX pin is at 0V
    digitalWrite(0, LOW);

    pinMode(ACCEL_INT_PIN, INPUT);
    // Use RISING. Int1 pin stays high, so no need to use CHANGE to avoid missing the interrupt.
    enableInterrupt(ACCEL_INT_PIN, on_wakeup, RISING);

    pinMode(BATTERY_CHARGING_PIN, INPUT_PULLUP);
    pinMode(BATTERY_CHARGED_PIN, INPUT_PULLUP);
    enableInterrupt(BATTERY_CHARGING_PIN, on_wakeup, CHANGE);
    enableInterrupt(BATTERY_CHARGED_PIN, on_wakeup, CHANGE);

#ifdef DEBUG_LED
    pinMode(DEBUG_LED, OUTPUT);
    digitalWrite(DEBUG_LED, HIGH);
#endif

    bma400.setPowerMode(BMA400_NORMAL);
    bma400.setMeasurementRange(BMA400_2G);
    bma400.setOutputDataRate(BMA400_12_5HZ);
    bma400.setOversamplingRate(BMA400_OSR_HIGHEST);
    bma400.enableWakeupInterrupts(1, BMA400_MANUAL, false, true);

    byte bma400_id = bma400.init();
#ifdef DEBUG_SERIAL
    Serial.print(F("Communication with BMA400:\t"));
    if (bma400_id == BMA400_CHIP_ID)
    {
        Serial.println(F("Successful"));
    }
    else
    {
        Serial.println(F("Failed"));
    }
#endif

#ifdef DEBUG_SERIAL
    Serial.print(F("Reading Power Mode:\t\t"));
    uint8_t powerMode = bma400.readPowerMode();
    switch (powerMode)
    {
    case 0:
        Serial.println(F("Sleep Mode"));
        break;
    case 1:
        Serial.println(F("Low Power Mode"));
        break;
    case 2:
        Serial.println(F("Normal Power Mode"));
        break;
    }

    Serial.print(F("Reading Measurement Range:\t"));
    uint8_t measurementRange = bma400.readMeasurementRange();
    switch (measurementRange)
    {
    case 0:
        Serial.println(F("2g"));
        break;
    case 1:
        Serial.println(F("4g"));
        break;
    case 2:
        Serial.println(F("8g"));
        break;
    case 3:
        Serial.println(F("16g"));
        break;
    }

    Serial.print(F("Reading Output Data Rate:\t"));
    uint8_t outputDataRate = bma400.readOutputDataRate();
    switch (outputDataRate)
    {
    case 5:
        Serial.println(F("12.5Hz"));
        break;
    case 6:
        Serial.println(F("25Hz"));
        break;
    case 7:
        Serial.println(F("50Hz"));
        break;
    case 8:
        Serial.println(F("100Hz"));
        break;
    case 9:
        Serial.println(F("200Hz"));
        break;
    case 10:
        Serial.println(F("400Hz"));
        break;
    case 11:
        Serial.println(F("800Hz"));
        break;
    }

    Serial.print(F("Reading Oversampling Range:\t"));
    uint8_t oversamplingRate = bma400.readOversamplingRate();
    switch (oversamplingRate)
    {
    case 0:
        Serial.println(F("OSR 0"));
        break;
    case 1:
        Serial.println(F("OSR 1"));
        break;
    case 2:
        Serial.println(F("OSR 2"));
        break;
    case 3:
        Serial.println(F("OSR 3"));
        break;
    }
#endif

    // Initialize low screen
    SSD1306 = SSD1306_LOW;
    oled.begin(128, 64, sizeof(tiny4koled_init_128x64b), tiny4koled_init_128x64b);
    oled.fill(0);

    // Initialize high screen
    SSD1306 = SSD1306_HIGH;
    oled.begin(128, 64, sizeof(tiny4koled_init_128x64b), tiny4koled_init_128x64b);
    oled.fill(0);

    // Note: both screens begin off
}

static void on_wakeup()
{
    //Do nothing. The chip stops sleeping and everything is handled in loop().
}

static void face_to_normal(byte face, Vec3<int> *normal)
{
    if (face < NORMAL_COUNT)
    {
        memcpy_P(normal, NORMALS + face, sizeof(Vec3<int>));
    }
    else
    {
        memcpy_P(normal, NORMALS + face - NORMAL_COUNT, sizeof(Vec3<int>));
        normal->mul_mut(-1);
    }
}

// Taken from the Arduino source for `analogRead` from `wiring_analog.c`.
static void select_adc_pin(uint8_t pin)
{
#if defined(analogPinToChannel)
#if defined(__AVR_ATmega32U4__)
    if (pin >= 18)
        pin -= 18; // allow for channel or pin numbers
#endif
    pin = analogPinToChannel(pin);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    if (pin >= 54)
        pin -= 54; // allow for channel or pin numbers
#elif defined(__AVR_ATmega32U4__)
    if (pin >= 18)
        pin -= 18; // allow for channel or pin numbers
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
    if (pin >= 24)
        pin -= 24; // allow for channel or pin numbers
#else
    if (pin >= 14)
        pin -= 14; // allow for channel or pin numbers
#endif

#if defined(ADCSRB) && defined(MUX5)
    // the MUX5 bit of ADCSRB selects whether we're reading from channels
    // 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
    ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#endif

    // set the analog reference (high two bits of ADMUX) and select the
    // channel (low 4 bits).  this also sets ADLAR (left-adjust result)
    // to 0 (the default).
    uint8_t analog_reference = INTERNAL;
#if defined(ADMUX)
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = (analog_reference << 4) | (pin & 0x07);
#else
    ADMUX = (analog_reference << 6) | (pin & 0x07);
#endif
#endif
}

// Taken from the Arduino source for `analogRead` from `wiring_analog.c`.
// Carry out ADC conversion from a pin selected by `select_adc_pin`.
// The idea is to sleep for a millisecond or some other arbitrary period of time between calls to
// `select_adc_pin` and `analog_read`, allowing current flow into the ADC even through heavy
// resistances.
static int analog_read()
{
    uint8_t low, high;

#if defined(ADCSRA) && defined(ADCL)
    // start the conversion
    sbi(ADCSRA, ADSC);

    // ADSC is cleared when the conversion finishes
    while (bit_is_set(ADCSRA, ADSC))
    {
    }

    // we have to read ADCL first; doing so locks both ADCL
    // and ADCH until ADCH is read.  reading ADCL second would
    // cause the results of each conversion to be discarded,
    // as ADCL and ADCH would be locked when it completed.
    low = ADCL;
    high = ADCH;
#else
    // we dont have an ADC, return 0
    low = 0;
    high = 0;
#endif

    // combine the two bytes
    return (high << 8) | low;
}

static bool check_battery_low()
{
    unsigned long start;
    //Enable the ADC and select the battery channel
    //It is important to first call `power_adc_enable` and _then_ set the ADCSRA.ADEN bit.
    power_adc_enable();
    ADCSRA = 0x80;

    select_adc_pin(BATTERY_VOLTAGE_PIN);
    // Now sample-and-hold capacitor is charging, and 1.1V bandgap reference is stabilizing.
    // Bandgap takes up to 70us (from datasheet),
    // and cap, 5*R*C = 5 * 1/(1/1M + 1/4M7) * 14pF = 58us
    delayMicroseconds(70);

    //Carry out conversion and disable ADC
    int voltage = analog_read();
    ADCSRA = 0x00;
    power_adc_disable();

#if defined(DEBUG_SERIAL) || defined(DISPLAY_VOLTAGE_ON_SCREEN)
    float actual_voltage = ((float)voltage + 0.5) / 1024. * BATTERY_REFERENCE_VOLTAGE / BATTERY_V_SCALE_RATIO;
#ifdef DEBUG_SERIAL
    Serial.print(F("Battery voltage: "));
    Serial.print(actual_voltage);
    Serial.println(F("V"));
#endif
#ifdef DISPLAY_VOLTAGE_ON_SCREEN
    battery_display_voltage = actual_voltage * 100.;
#endif
#endif
    return voltage <= BATTERY_LOW_THRESHOLD;
}

static BatStatus get_charge_status() {
    /*
    Charger is a TP4056 board. It had two leds, charging and charged, that light up respectively, one at a time.
    Also when there's no battery, charged is lit and charging flashes. We ignore that case.
    As the leds are turned on by the TP4056 with open drain, and the arduino pins have pullups,
    LOW means that the led would be lit.
    */

    if (digitalRead(BATTERY_CHARGED_PIN) == LOW) {
        return BAT_CHARGED;
    }
    if (digitalRead(BATTERY_CHARGING_PIN) == LOW) {
        return BAT_CHARGING;
    }
    return BAT_NOT_CHARGING;
}

static bool change_face(const Vec3<int> &acc)
{
    //Check acceleration magnitude
    if (acc.magsq() < (unsigned long)MIN_ABS_ACC * MIN_ABS_ACC || acc.magsq() > (unsigned long)MAX_ABS_ACC * MAX_ABS_ACC)
    {
        //Unreliable readings: the device is not still (or not on earth)
        return false;
    }

    //Calculate average
    Vec3<long> avg_wide = Vec3<long>();
    for (int i = 0; i < ROLLAVG_LEN; i += 1)
    {
        avg_wide.add_mut(rollavg_buf[i].to_wide());
    }
    avg_wide.div_mut(ROLLAVG_LEN);
    Vec3<int> avg = avg_wide.to_narrow();

    //Calculate deviation
    unsigned long distsq = 0;
    for (int i = 0; i < ROLLAVG_LEN; i += 1)
    {
        Vec3<int> dist = rollavg_buf[i];
        dist.sub_mut(avg);
        distsq += dist.magsq();
    }
    distsq /= ROLLAVG_LEN;

    //Check standard deviation
    if (distsq > (unsigned long)MAX_AVG_DEVIATION * MAX_AVG_DEVIATION)
    {
        //Unreliable readings: accleration hasn't settled
        return false;
    }

    //Find the face normal that matches the current acceleration most closely
    int maxdot = 0;
    int8_t active_normal = -1;
    for (int8_t i = 0; i < NORMAL_COUNT; i += 1)
    {
        Vec3<int> normal;
        memcpy_P(&normal, NORMALS + i, sizeof(Vec3<int>));
        int thisdot = normal.dot(avg) >> 10;
        int dotmag = abs(thisdot);
        if (dotmag > maxdot)
        {
            active_normal = i;
            if (thisdot < 0)
            {
                //Negative dot product indicates the opposite face
                active_normal += NORMAL_COUNT;
            }
            maxdot = dotmag;
        }
    }

    if (active_normal == -1)
    {
        //It is literally impossible under sensible configs for no normal to have been chosen.
        //Only 3 cases are possible:
        //  - `acc` is null, which would mean `acc` is under `MIN_ABS_ACC` and we would have
        //      returned already.
        //  - `dotmag` was exactly -32768 for _all_ faces (abs(-32768) == -32768), but this would
        //      imply an acceleration of 32g (impossible, +/-16g is the max range).
        //  - There are no normals (NORMAL_COUNT == 0).
        //But whatever
        return false;
    }

    //At this point, readings are already reliable

    if (current_face == active_normal)
    {
        //No face change needed, but readings are reliable
        return true;
    }

    //Check distance from current face to acceleration reading
    Vec3<int> delta;
    face_to_normal(current_face, &delta);
    delta.sub_mut(acc);
    if (delta.magsq() < (unsigned long)MIN_FACE_DIST * MIN_FACE_DIST)
    {
        //Still too close to the current face
        //However, the readings are reliable
        return true;
    }

    //Changed face!
    int face_time = pgm_read_word(FACE_TIMES + active_normal);
    if (face_time == 0)
    {
        remaining_seconds = 0;
    }
    else
    {
        if (remaining_seconds < 0)
        {
            remaining_seconds = 0;
        }
        remaining_seconds += face_time;
    }
    timer_ref = Instant();
    if (active_normal != 0 && active_normal != NORMAL_COUNT)
    {
        // Also change screen orientation (to stay facing up)
        current_orient = pgm_read_byte(SCREEN_ORIENTATIONS + active_normal);
        // Also check battery
        if (check_battery_low())
        {
            low_battery_frames = 0;
        }
    }
    current_face = active_normal;
    return true;
}

static void deep_sleep(Vec3<int> &cur_acc)
{
    //Compute the minimum distance to a face (that is not the current face)
    Vec3<int> cur_face_normal;
    unsigned long min_distsq = 0xffffffff;
    byte min_face = 0;
    for (byte i = 0; i < NORMAL_COUNT; i += 1)
    {
        Vec3<int> normal;
        memcpy_P(&normal, NORMALS + i, sizeof(Vec3<int>));
        unsigned long distsq = normal.distsq(cur_acc);
        if (i == current_face)
        {
            cur_face_normal = normal;
        }
        else
        {
            if (distsq < min_distsq)
            {
                min_distsq = distsq;
                min_face = i;
            }
        }
        normal.mul_mut(-1);
        distsq = normal.distsq(cur_acc);
        if (i + NORMAL_COUNT == current_face)
        {
            cur_face_normal = normal;
        }
        else
        {
            if (distsq < min_distsq && current_face != i + NORMAL_COUNT)
            {
                min_distsq = distsq;
                min_face = i + NORMAL_COUNT;
            }
        }
    }

    //Compute the distance to the edge of the closest face sphere
    int neighbor_dist = (int)sqrt(min_distsq) - MIN_FACE_DIST;

    //Compute the distance to the edge of the current face sphere
    int frontier_dist = MIN_FACE_DIST - (int)sqrt(cur_face_normal.distsq(cur_acc));

    unsigned int threshold = max(max(neighbor_dist, frontier_dist) + SLEEP_TOLERANCE_ACC, SLEEP_MIN_THRESHOLD);

#ifdef DEBUG_SERIAL
    Serial.print(F("at dist "));
    Serial.print((unsigned int)sqrt(min_distsq));
    Serial.print(F(" from face "));
    Serial.print(min_face);
    Serial.print(F(" (threshold = "));
    Serial.print(threshold);
    Serial.println(F(")"));
#endif

    //Shutdown status LED
#ifdef DEBUG_LED
    digitalWrite(DEBUG_LED, LOW);
#endif

    //Enter accelerometer low-power mode
    bma400.setWakeupThreshold(threshold);
    bma400.setWakeupRef(cur_acc.x, cur_acc.y, cur_acc.z);
    bma400.setPowerMode(BMA400_LOWPOWER);

    //Enter atmega328p deep sleep until the accelerometer interrupt wakes us up
    //Note that adc was already turned off at setup. The `LowPower` library is passed an `ADC_ON`
    //value so that it doesn't turn them back on after sleeping.
    LowPower.powerDown(SLEEP_FOREVER, ADC_ON, BOD_OFF);

    //Ramp up the accelerometer
    bma400.setPowerMode(BMA400_NORMAL);

    //Turn LED back on
#ifdef DEBUG_LED
    digitalWrite(DEBUG_LED, HIGH);
#endif

    //Refresh timer reference, since no timekeeping is done while deep-sleeping
    timer_ref = Instant();
}

void loop()
{

    //Throttle reads
    {
//Sleep with a half eye open
#ifdef DEBUG_PROFILE_IDLE
        static unsigned long last_wakeup = micros();
        static unsigned long active_micros = 0;
        static unsigned long idle_micros = 0;
#endif
        static Instant next_read = Instant();
        if (Instant().lt(next_read))
        {
#ifdef DEBUG_PROFILE_IDLE
            if (active_micros + idle_micros >= 1000000)
            {
                Serial.print(F("Active/idle: "));
                Serial.print(active_micros);
                Serial.print(F("/"));
                Serial.println(idle_micros);
                active_micros = 0;
                idle_micros = 0;
            }
            unsigned long before_idle = micros();
            active_micros += before_idle - last_wakeup;
#endif
            do
            {
                //Keeping TIMER1 on will enable the periodic timer interrupt we're using to keep
                //track of time, meaning that timekeeping will continue.
                //Note that adc, timer0, timer2, spi and usart were already turned off at setup.
                //The `LowPower` library is passed `_ON` values so that it doesn't turn them back
                //on after sleeping.
                LowPower.idle(SLEEP_FOREVER, ADC_ON, TIMER2_ON, TIMER1_ON, TIMER0_ON, SPI_ON, USART0_ON, TWI_OFF);
            } while (Instant().lt(next_read));
#ifdef DEBUG_PROFILE_IDLE
            unsigned long after_idle = micros();
            idle_micros += after_idle - before_idle;
            last_wakeup = after_idle;
#endif
        }
        else
        {
            next_read = Instant();
        }
        next_read = next_read.delayed_by(Duration::from_millis(1000 / READ_RATE));
    }

    //Read accelerometer
    bma400.readData();
    Vec3<int> acc = Vec3<int>{bma400.raw_acc_x, bma400.raw_acc_y, bma400.raw_acc_z};

    //Smooth readings using a rolling average
    //Add current reading to the buffer
    rollavg_buf[rollavg_idx] = acc;
    rollavg_idx = (rollavg_idx + 1) % ROLLAVG_LEN;

    //Whether the current measures can be trusted, or otherwise are just noise
    bool reliable = change_face(acc);

    //Act based on current face
    if (current_face == 0 || current_face == NORMAL_COUNT)
    {
        // Home face
        
        BatStatus bat_status = get_charge_status();
        if (bat_status == BAT_NOT_CHARGING) {
            select_screen(0);
        } else {
            int screen_pointing_up = SSD1306_LOW;     // TODO
            select_screen(screen_pointing_up);
            draw_bat_charge(bat_status);
        }

        if (!reliable)
        {
            timer_ref = Instant();
        }
        if (timer_ref.elapsed().gt(SLEEP_TIMEOUT))
        {
            //Go to sleep
            deep_sleep(acc);
        }
    }
    else
    {
        // Active face
        Instant now = Instant();
        while (now.elapsed_since(timer_ref).gt(1000))
        {
            remaining_seconds -= 1;
            if (remaining_seconds <= -10000)
            {
                remaining_seconds = remaining_seconds % 10000;
            }
            timer_ref = timer_ref.delayed_by(Duration::from_millis(1000));
        }
        int display_time;
        bool show = true;
        if (remaining_seconds <= 0)
        {
            display_time = -remaining_seconds;
            show = now.elapsed_since(timer_ref).as_millis() % ALARM_BLINK_PERIOD >= ALARM_BLINK_OFFTIME;
        }
        else
        {
            display_time = remaining_seconds;
            show = now.elapsed_since(timer_ref).as_millis() % COUNTDOWN_BLINK_PERIOD >= COUNTDOWN_BLINK_OFFTIME;
        }
        if (display_time >= 3600)
        {
            display_time /= 60;
        }
        if (current_face < NORMAL_COUNT)
        {
            select_screen(SSD1306_LOW);
        }
        else
        {
            select_screen(SSD1306_HIGH);
        }
        drawScreen(display_time, show);
    }

    //Debug-print current accelerometer readings for face normal calibration
#ifdef DEBUG_PRINT_NORMALS

#ifdef DEBUG_SERIAL
    Serial.print(bma400.raw_acc_x);
    Serial.print(F(", "));
    Serial.print(bma400.raw_acc_y);
    Serial.print(F(", "));
    Serial.print(bma400.raw_acc_z);
    Serial.println();
#endif

#endif
}
