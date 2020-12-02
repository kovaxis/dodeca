
#include <avr/power.h>
#include <Tiny4kOLED.h>

#include "BMA400.h"
#include "LowPower.h"
#include "common.h"
#include "font.h"

// Whether to fill the screen with a hatch pattern instead of standard text.
//#define FILL_HATCH

// Whether to print current acceleration through serial, for calibration purposes.
// Requires `USE_SERIAL` defined in `common.h`.
//#define PRINT_NORMALS

// Whether to keep the status LED on while not sleeping, and on which pin.
#define STATUS_LED LED_BUILTIN

// I2C address of the BMA400 accelerometer.
#define BMA400_ADDRESS 0x14

// Arduino pin mapped to the INT1 pin on the BMA400 accelerometer.
#define WAKEUP_INT_PIN 2

// Minimum amount of time between switching power on an OLED, in milliseconds.
#define OLED_POWER_DEBOUNCE 100

// ---- Accelerometer behaviour ----
// All acceleration units are in mibi-g, where 1024 mibi-g is equivalent to the acceleration of
// gravity on earth.

// Maximum amount of times per second to poll the accelerometer.
// Note that this is only a maximum, since timer1 might be running at a different rate, therefore
// not waking up the arduino on time to do exactly `READ_RATE` reads per second.
// Additionally, the accelerometer might be operating at a different rate, and might not update
// its acceleration values as fast as `READ_RATE`.
#define READ_RATE 10

// Size of the accelerometer rolling average buffer.
#define ROLLAVG_LEN 4

// Measuring less than this acceleration will mark the measures as unreliable.
// Unreliable measures will not send the Arduino to sleep or change the current face/orientation.
#define MIN_ABS_ACC 974

// Measuring more than this acceleration will mark the measures as unreliable.
#define MAX_ABS_ACC 1074

// If rolling average deviation is over this limit, mark the measures as unreliable.
#define MAX_AVG_DEVIATION 20

// If the acceleration component orthogonal to a face is smaller than this value, do not change
// active face. However, the reading is still considered "reliable", and as such may send the
// arduino to sleep.
// This behaviour enables the device to go to sleep even if slightly tilted.
#define MIN_FACE_ACC 940

// Acceleration difference required to wake up the accelerometer and arduino after entering deep
// sleep.
#define WAKEUP_THRESHOLD 400

#define NORMAL_COUNT 6
#define FACE_COUNT NORMAL_COUNT * 2

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
  1*60,
  2*60,
  3*60,
  4*60,
  5*60,

  /*0,
  7*60,
  10*60,
  15*60,
  20*60,
  25*60,*/
};

static byte current_face = 0;
static byte current_orient = ORIENT_000;
static Instant timer_ref = Instant();
static int remaining_seconds = 0;
static bool screen_on;
static Vec3<int> rollavg_buf[ROLLAVG_LEN];
static byte rollavg_idx = 0;

void select_screen(int screen) {
  static Instant last_screen_change = Instant();
  bool enable = screen == 0;
  if (enable != screen_on) {
    unsigned long millis_debounce = last_screen_change.elapsed().as_millis();
    if (millis_debounce < 0 || millis_debounce >= OLED_POWER_DEBOUNCE) {
      if (enable) {
        oled.on();
      }else{
        oled.off();
      }
      last_screen_change = Instant();
      screen_on = enable;
    }
  }
}

static void drawScreen(int seconds, bool show) {
  #ifndef FILL_HATCH
  
  byte orient = current_orient;
  
  static int last_seconds = -1;
  static bool last_show = false;
  static byte last_orient = -1;
  if (last_seconds == seconds && last_show == show && last_orient == current_orient) {
    return;
  }else{
    last_seconds = seconds;
    last_show = show;
    last_orient = orient;
  }
  
  scr_clear();
  if (show) {
    int time = seconds;
    scr_draw(orient, 4, time%10);
    time /= 10;
    scr_draw(orient, 3, time%6);
    time /= 6;
    scr_draw(orient, 2, 10);
    scr_draw(orient, 1, time%10);
    time/= 10;
    scr_draw(orient, 0, time%10);
  }
  scr_show();

  #endif

  //Hatch pattern
  #ifdef FILL_HATCH
  for (int y = 0; y < 8; y += 1) {
    oled.setCursor(0, i);
    oled.startData();
    for (int x = 0; x < 64; x += 1) {
      oled.sendData(0b10101010);
      oled.sendData(0b01010101);
    }
    oled.endData();
  }
  #endif
}

BlueDot_BMA400 bma400 = BlueDot_BMA400();

void setup() {
  ADCSRA = 0;
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer2_disable();

  timekeep_init();

  #ifdef USE_SERIAL
  Serial.begin(115200);
  #else
  power_usart0_disable();
  #endif
  
  pinMode(WAKEUP_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(WAKEUP_INT_PIN), on_accelerometer_wakeup, RISING);
  
  #ifdef STATUS_LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
  #endif
  
  //Choose I2C Address (0x14 or 0x15, last bit is configured through SDO)
  //TODO: Optimize power usage by checking whether SDO is pull-up or pull-down.
  bma400.I2CAddress = BMA400_ADDRESS;
  
  //0x00:     In sleep mode no measurements are performed, but power consumption is at a minimum
  //0x01:     low power mode 
  //0x02:     normal mode
  bma400.setPowerMode(0x02);
  
  //0x00:     2g
  //0x01:     4g 
  //0x02:     8g
  //0x03:     16g
  bma400.setMeasurementRange(0x00);
  
  //0x05:     12.5 Hz
  //0x06:     25 Hz
  //0x07:     50 Hz
  //0x08:     100 Hz
  //0x09:     200 Hz
  //0x0A:     400 Hz
  //0x0B:     800 Hz
  bma400.setOutputDataRate(0x05);
  
  //0x00:     lowest oversampling rate, lowest power consumption, lowest accuracy
  //0x01:     
  //0x02:     
  //0x03:     highest oversampling rate, highest power consumption, highest accuracy
  bma400.setOversamplingRate(0x00);
  
  byte bma400_id = bma400.init();
  #ifdef USE_SERIAL
  Serial.print(F("Communication with BMA400:\t"));
  if (bma400_id == BMA400_CHIP_ID) {
    Serial.println(F("Successful"));   
  } else {
    Serial.println(F("Failed"));
  }
  #endif

  // AUTOWAKEUP_0: AAAA AAAA
  //  - A: Wakeup timeout MSB
  bma400.writeByte(0x2c, 0b00000000);
  
  // AUTOWAKEUP_1: AAAA _BC_
  //  - A: Wakeup timeout LSB
  //  - B: Wakeup timeout enable
  //  - C: Wakeup interrupt enable
  bma400.writeByte(0x2d, 0b00000010);
  
  // WKUP_INT_CONFIG0: ABCD DDEE
  //  - A: Wakeup on Z
  //  - B: Wakeup on Y
  //  - C: Wakeup on X
  //  - D: Number of samples for wakeup (-1)
  //  - E: Wakeup refupdate (0b01 is "update when going to sleep")
  bma400.writeByte(0x2f, 0b11100001);
  
  // WKUP_INT_CONFIG1: AAAA AAAA
  //  - A: Wakeup threshold (8 most-significant bits)
  bma400.writeByte(0x30, WAKEUP_THRESHOLD >> 4);
  
  // INT1_MAP: ABCD EFGH
  //  - A: Map data-ready interrupt to INT1
  //  - B: Map FIFO-watermark interrupt to INT1
  //  - C: Map FIFO-full interrupt to INT1
  //  - D: Map interrupt-overrun interrupt to INT1
  //  - E: Map generic-2 interrupt to INT1
  //  - F: Map generic-1 interrupt to INT1
  //  - G: Map orientation-change interrupt to INT1
  //  - H: Map wakeup interrupt to INT1
  bma400.writeByte(0x21, 0b00000001);

  // INT12_IO_CTRL: _AB_ _CD_
  //  - A: Use open-drive for INT2
  //  - B: Active level for INT2
  //  - C: Use open-drive for INT1
  //  - D: Active level for INT1
  bma400.writeByte(0x24, 0b00000010);
  
  #ifdef USE_SERIAL
  Serial.print(F("Reading Power Mode:\t\t"));
  uint8_t powerMode = bma400.readPowerMode();
  switch(powerMode) {
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
  switch(measurementRange) {
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
  switch(outputDataRate) {
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
  switch(oversamplingRate) {
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
  
  oled.begin(128, 64, sizeof(tiny4koled_init_128x64b), tiny4koled_init_128x64b);
  oled.fill(0);
  select_screen(0);
}

static void on_accelerometer_wakeup() {
  //Do nothing
}

void loop() {
  //Throttle reads
  {
    //Sleep with a half eye open
    static Instant next_read = Instant();
    if (Instant().lt(next_read)) {
      do {
        //Keeping TIMER0 on will enable timing interrupts for `millis()` and `micros()`, meaning that we will
        //be woken up even though we're using `SLEEP_FOREVER`, and timekeeping will stay precise.
        //Note that adc, timer0, timer2, spi and usart were already turned off at setup. The
        //`LowPower` library is passed `_ON` values so that it doesn't turn them back on after
        //sleeping.
        LowPower.idle(SLEEP_FOREVER, ADC_ON, TIMER2_ON, TIMER1_ON, TIMER0_ON, SPI_ON, USART0_ON, TWI_OFF);
      } while(Instant().lt(next_read));
    }else{
      next_read = Instant();
    }
    next_read = next_read.delayed_by(Duration::from_millis(1000/READ_RATE));
  }

  //Read accelerometer
  bma400.readData();
  Vec3<int> acc = Vec3<int>{bma400.raw_acc_x, bma400.raw_acc_y, bma400.raw_acc_z};

  //Smooth readings using a rolling average
  //Add current reading to the buffer
  rollavg_buf[rollavg_idx] = acc;
  rollavg_idx = (rollavg_idx + 1) % ROLLAVG_LEN;

  //Whether the current measures can be trusted, or otherwise are just noise
  bool reliable = false;
  if (acc.magsq() >= (unsigned long) MIN_ABS_ACC*MIN_ABS_ACC && acc.magsq() <= (unsigned long) MAX_ABS_ACC*MAX_ABS_ACC) {
    //Calculate average
    Vec3<long> avg_wide = Vec3<long>();
    for(int i = 0; i < ROLLAVG_LEN; i += 1) {
      avg_wide.add_mut(rollavg_buf[i].to_wide());
    }
    avg_wide.div_mut(ROLLAVG_LEN);
    Vec3<int> avg = avg_wide.to_narrow();

    //Calculate deviation
    unsigned long distsq = 0;
    for(int i = 0; i < ROLLAVG_LEN; i += 1) {
      Vec3<int> dist = rollavg_buf[i];
      dist.sub_mut(avg);
      distsq += dist.magsq();
    }
    distsq /= ROLLAVG_LEN;

    //If deviation is under an acceptable threshold, use it
    if (distsq < (unsigned long) MAX_AVG_DEVIATION*MAX_AVG_DEVIATION) {
      //Detect upward-facing face
      int maxdot = 0;
      int8_t active_normal = -1;
      for(int8_t i = 0; i < NORMAL_COUNT; i += 1) {
        Vec3<int> normal;
        memcpy_P(&normal, NORMALS + i, sizeof(Vec3<int>));
        int thisdot = normal.dot(avg) >> 10;
        int dotmag = abs(thisdot);
        if (dotmag > maxdot) {
          active_normal = i;
          if (thisdot < 0) {
            active_normal += NORMAL_COUNT;
          }
          maxdot = dotmag;
        }
      }
      if (active_normal != -1) {
        reliable = true;
        if (maxdot >= MIN_FACE_ACC && current_face != active_normal) {
          //Changed face!
          current_face = active_normal;
          remaining_seconds = pgm_read_word(FACE_TIMES + current_face);
          timer_ref = Instant();
          if (active_normal != 0 && active_normal != 6) {
            // Also change screen orientation (to stay facing up)
            current_orient = pgm_read_byte(SCREEN_ORIENTATIONS + active_normal);
          }
        }
      }
    }
  }

  //Act based on current face
  if (current_face == 0 || current_face == 6) {
    // Home face
    select_screen(-1);
    if (!reliable) {
      timer_ref = Instant();
    }
    if (timer_ref.elapsed().gt(2000)) {
      //Go to sleep

      //Shutdown status LED
      #ifdef STATUS_LED
      digitalWrite(STATUS_LED, LOW);
      #endif

      //Enter accelerometer low-power mode
      bma400.setPowerMode(0x1);

      //Enter atmega328p deep sleep until the accelerometer interrupt wakes us up
      //Note that adc was already turned off at setup. The `LowPower` library is passed an `ADC_ON`
      //value so that it doesn't turn them back on after sleeping.
      LowPower.powerDown(SLEEP_FOREVER, ADC_ON, BOD_OFF);

      //Ramp up the accelerometer
      bma400.setPowerMode(0x2);

      //Turn on LED
      #ifdef STATUS_LED
      digitalWrite(STATUS_LED, HIGH);
      #endif

      //Refresh timer reference
      timer_ref = Instant();
    }
  }else{
    // Active face
    Instant now = Instant();
    while (now.elapsed_since(timer_ref).gt(1000)) {
      remaining_seconds -= 1;
      if (remaining_seconds <= -10000) {
        remaining_seconds = remaining_seconds % 10000;
      }
      timer_ref = timer_ref.delayed_by(Duration::from_millis(1000));
    }
    int display_time;
    bool show = true;
    if (remaining_seconds <= 0) {
      display_time = -remaining_seconds;
      show = now.elapsed_since(timer_ref).as_millis() % 500 >= 250;
    }else{
      display_time = remaining_seconds;
    }
    drawScreen(display_time, show);
    select_screen(0);
  }

  //Debug-print current accelerometer readings for face normal calibration
  #ifdef PRINT_NORMALS
  
  #ifdef USE_SERIAL
  Serial.print(bma400.raw_acc_x);
  Serial.print(F(", "));
  Serial.print(bma400.raw_acc_y);
  Serial.print(F(", "));
  Serial.print(bma400.raw_acc_z);
  Serial.println();
  #endif
  
  #endif
}
