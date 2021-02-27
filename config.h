
#ifndef CONFIG_H
#define CONFIG_H

// ---- Setup parameters ----

// I2C address of the BMA400 accelerometer (0x14 or 0x15, last bit is configured through SDO).
// Tests show wiring SDO to ground consumes less power.
const uint8_t BMA400_ADDRESS = 0x14;

// Screen address for the lower faces.
const uint8_t SSD1306_LOW = 0x3c;
// Screen address for the higher faces.
const uint8_t SSD1306_HIGH = 0x3d;

// Arduino pin mapped to the INT1 pin on the BMA400 accelerometer.
const uint8_t ACCEL_INT_PIN = 4;

// On which Arduino analog pin to read battery voltage.
const uint8_t BATTERY_VOLTAGE_PIN = A1;

// Arduino pins connected to the leds of the battery charger.
const uint8_t BATTERY_CHARGING_PIN = 5;
const uint8_t BATTERY_CHARGED_PIN = 6;


// ---- Timing ----

// The timer resolution in milliseconds. Lower values consume more power.
// Timer1 supports periods up to about 8.3 seconds.
// Timer2 supports periods up to 32 milliseconds.
const int TIMER_PERIOD = 100;

// ---- Accelerometer behaviour ----
// All acceleration units are in mibi-g, where 1024 mibi-g is equivalent to the acceleration of
// gravity on earth.

// Maximum amount of times per second to poll the accelerometer.
// Note that this is only a maximum, since timer1 might be running at a different rate, therefore
// not waking up the arduino on time to do exactly `READ_RATE` reads per second.
// Additionally, the accelerometer might be operating at a different rate, and might not update
// its acceleration values as fast as `READ_RATE`.
const int READ_RATE = 10;

// Size of the accelerometer rolling average buffer.
const int ROLLAVG_LEN = 4;

// Measuring less than this acceleration will mark the measures as unreliable.
// Unreliable measures will not send the Arduino to sleep or change the current face/orientation.
const int MIN_ABS_ACC = 974;

// Measuring more than this acceleration will mark the measures as unreliable.
const int MAX_ABS_ACC = 1074;

// If rolling average deviation is over this limit, mark the measures as unreliable.
const int MAX_AVG_DEVIATION = 20;

// Acceleration readings must be this distance away from the normal of the current face in order
// to be able to change the face.
// However, readings are still considered "reliable" if they are closer to the current face than
// MIN_FACE_DIST.
// Faces are about 1g apart from each other.
const int MIN_FACE_DIST = 700;

// ---- Sleep behaviour ----

// After how many milliseconds still in a home position to go to sleep.
const int SLEEP_TIMEOUT = 500;

// When sleeping, the accelerometer is set so that it wakes up as soon as acceleration has moved
// enough to possibly step into the active sphere of a face.
// However, this extra dead space is given to account for noise.
const int SLEEP_TOLERANCE_ACC = 40;

// In extreme cases where the device sleeps completely tilted, what is the minimum threshold for
// wakeup.
const int SLEEP_MIN_THRESHOLD = 100;

// ---- Display behaviour ----

// X coordinate of the top-left of the screen area.
const int SCRBUF_X = 40;

// Page Y of the top-left of the screen area.
// Effective Y is this value multiplied by 8.
const int SCRBUF_PAGEY = 0;

// Width of the screen area in pixels.
const int SCRBUF_WIDTH = 48;

// Height of the screen area in pages (8 pixels).
const int SCRBUF_PAGES = 6;

// Minimum amount of time between switching power on an OLED, in milliseconds.
const int OLED_POWER_DEBOUNCE = 100;

// Blinking period while counting down. Should be a divisor of 1000. In practice, clamped to the
// next multiple of TIMER_PERIOD.
const int COUNTDOWN_BLINK_PERIOD = 1000;

// How much of the blink period to hide the numbers. In practice, clamped to the next multiple of
// TIMER_PERIOD.
const int COUNTDOWN_BLINK_OFFTIME = 0;

// Blinking period once the countdown has finished and the alarm is ringing. Should be a divisor of
// 1000. In practice, clamped to the next multiple of TIMER_PERIOD.
const int ALARM_BLINK_PERIOD = 500;

// How much of the blink period to hide the numbers. In practice, clamped to the next multiple of
// TIMER_PERIOD.
const int ALARM_BLINK_OFFTIME = 100;

// ---- Debug options ----

// Whether to include and activate the Serial library and send debug messages through it.
#define DEBUG_SERIAL

// Whether to profile the active/idle times and report them back through serial once a second.
//#define DEBUG_PROFILE_IDLE

// Whether to fill the screen with a hatch pattern instead of standard text.
//#define DEBUG_DRAW_HATCH
const uint8_t DEBUG_HATCH_PATTERN1 = 0b10101010;
const uint8_t DEBUG_HATCH_PATTERN2 = 0b01010101;

// Whether to print current acceleration through serial, for calibration purposes.
// Requires `DEBUG_SERIAL` defined in `common.h`.
//#define DEBUG_PRINT_NORMALS

// Whether to keep the status LED on while not sleeping, and on which pin.
//#define DEBUG_LED LED_BUILTIN

// Whether to display the measured battery voltage on screen instead of the current time.
#define DISPLAY_VOLTAGE_ON_SCREEN

// ---- Battery settings ----

// What battery voltage to consider "low".
const float BATTERY_LOW_VOLTAGE = 3.4;

// The resistance of the first resistor in the voltage divider (wiring to the battery).
// The unit does not matter as long as it's consistent with `BATTERY_DIV_R2`.
const float BATTERY_DIV_R1 = 4.7;
// The resistance of the second resistor in the voltage divider (wiring to ground).
const float BATTERY_DIV_R2 = 1.;
// The ADC reference voltage in use.
const float BATTERY_REFERENCE_VOLTAGE = 1.1;

// Intermediate value.
const float BATTERY_V_SCALE_RATIO = BATTERY_DIV_R2 / (BATTERY_DIV_R1 + BATTERY_DIV_R2);
// The ADC raw value to consider as "low battery" (in the range [0, 1023]).
const int BATTERY_LOW_THRESHOLD = (int)(BATTERY_LOW_VOLTAGE * BATTERY_V_SCALE_RATIO / BATTERY_REFERENCE_VOLTAGE * 1024);

// For how many frames to display the "low battery" icon.
const int LOW_BATTERY_FRAMES = 56;

// The low battery blink period in frames.
const int LOW_BATTERY_BLINK_PERIOD = 8;

// For how many frames to keep the battery icon on.
const int LOW_BATTERY_BLINK_ONFRAMES = 6;

// ---- Face properties ----

const int NORMAL_COUNT = 6;
const int FACE_COUNT = (NORMAL_COUNT * 2);

// Normals in accelerometer space for each face pair, all with approximate magnitude 1024.
const PROGMEM Vec3<int> NORMALS[NORMAL_COUNT] = {
    //Orientation 0 (home)
    {4, -906, -477},
    {0, -33, -1023},
    {852, -289, -489},
    {557, -751, 417},
    {-516, -773, 429},
    {-855, -309, -472},
};

const PROGMEM byte SCREEN_ORIENTATIONS[FACE_COUNT] = {
    ORIENT_000,
    ORIENT_000,
    ORIENT_288,
    ORIENT_216,
    ORIENT_144,
    ORIENT_072,

    ORIENT_000,
    ORIENT_000,
    ORIENT_072,
    ORIENT_144,
    ORIENT_216,
    ORIENT_288,
};

const PROGMEM int FACE_TIMES[FACE_COUNT] = {
    0,
    5,
    10,
    15,
    20,
    30,

    0,
    1 * 60,
    2 * 60,
    3 * 60,
    4 * 60,
    5 * 60,

    /*0,
    7*60,
    10*60,
    15*60,
    20*60,
    25*60,*/
};

#endif
