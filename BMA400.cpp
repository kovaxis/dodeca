

#if defined(_AVR_)
#include <util/delay.h>
#endif

#include "BMA400.h"
#include "Wire.h"

enum Register {
    REG_CHIP_ID = 0x00,
    REG_STATUS = 0x03,

    REG_ACC_X_LSB = 0x04,
    REG_ACC_X_MSB = 0x05,
    REG_ACC_Y_LSB = 0x06,
    REG_ACC_Y_MSB = 0x07,
    REG_ACC_Z_LSB = 0x08,
    REG_ACC_Z_MSB = 0x09,

    REG_TEMP_ADDR = 0x11,
    REG_ACC_CONFIG0 = 0x19,
    REG_ACC_CONFIG1 = 0x1A,

    REG_INT1_MAP = 0x21,
    REG_INT12_IO_CTRL = 0x24,

    REG_AUTOWAKEUP_0 = 0x2c,
    REG_AUTOWAKEUP_1 = 0x2d,

    REG_WKUP_INT_CONFIG0 = 0x2f,
    REG_WKUP_INT_CONFIG1 = 0x30,
    REG_WKUP_INT_CONFIG2 = 0x31,
    REG_WKUP_INT_CONFIG3 = 0x32,
    REG_WKUP_INT_CONFIG4 = 0x33,
};

BlueDot_BMA400::BlueDot_BMA400(uint8_t i2c_address) {
    this->i2c_address = i2c_address;
}

uint8_t BlueDot_BMA400::init(void) {
    Wire.begin();

    return checkID();
}
//##########################################################################
// SET UP FUNCTIONS
//##########################################################################
uint8_t BlueDot_BMA400::checkID(void) {
    uint8_t chipID;
    chipID = readByte(REG_CHIP_ID);
    return chipID;
}
//##########################################################################
void BlueDot_BMA400::setPowerMode(uint8_t power_mode) {
    uint8_t reg;
    reg = readByte(REG_ACC_CONFIG0);
    reg = reg & 0b11111100;

    reg = reg | power_mode;
    writeByte(REG_ACC_CONFIG0, reg);
}
//##########################################################################
void BlueDot_BMA400::setMeasurementRange(uint8_t range) {
    uint8_t reg;
    reg = readByte(REG_ACC_CONFIG1);
    reg = reg & 0b00111111;

    reg = reg | (range << 6);
    writeByte(REG_ACC_CONFIG1, reg);
}
//##########################################################################
void BlueDot_BMA400::setOutputDataRate(uint8_t data_rate) {
    uint8_t reg;
    reg = readByte(REG_ACC_CONFIG1);
    reg = reg & 0b11110000;

    reg = reg | data_rate;
    writeByte(REG_ACC_CONFIG1, reg);
}
//##########################################################################
void BlueDot_BMA400::setOversamplingRate(uint8_t osr) {
    uint8_t reg;
    reg = readByte(REG_ACC_CONFIG1);
    reg = reg & 0b11001111;

    reg = reg | (osr << 4);
    writeByte(REG_ACC_CONFIG1, reg);
}

// Enables wakeup interrupts on the INT1 pin.
// `sample_count`: Samples required to generate an interrupt. [1, 8]
// `refupdate`: The auto reference update mode (see `RefUpdate`).
// `open_drive`: If `true`, leaves the pin floating while inactive. The pin is
// always pushed/pulled
//      when active.
// `active_high`: If `true`, the pin is pushed high when active. If `false`, the
// pin is pulled low
//      when active.
void BlueDot_BMA400::enableWakeupInterrupts(uint8_t sample_count,
                                            uint8_t refupdate, bool open_drive,
                                            bool active_high) {
    // AUTOWAKEUP_0: AAAA AAAA
    //  - A: Wakeup timeout MSB
    writeByte(REG_AUTOWAKEUP_0, 0x00);

    // AUTOWAKEUP_1: AAAA _BC_
    //  - A: Wakeup timeout LSB
    //  - B: Wakeup timeout enable
    //  - C: Wakeup interrupt enable
    writeByte(REG_AUTOWAKEUP_1, 0b00000010);

    // WKUP_INT_CONFIG0: ABCD DDEE
    //  - A: Wakeup on Z
    //  - B: Wakeup on Y
    //  - C: Wakeup on X
    //  - D: Number of samples for wakeup (-1)
    //  - E: Wakeup refupdate:
    //    0b00: Manual update
    //    0b01: Update once
    //    0b10: Update at 25Hz, requiring fast movement to wake up
    writeByte(REG_WKUP_INT_CONFIG0,
              0b11100000 | ((sample_count - 1) << 2) | refupdate);

    // INT1_MAP: ABCD EFGH
    //  - A: Map data-ready interrupt to INT1
    //  - B: Map FIFO-watermark interrupt to INT1
    //  - C: Map FIFO-full interrupt to INT1
    //  - D: Map interrupt-overrun interrupt to INT1
    //  - E: Map generic-2 interrupt to INT1
    //  - F: Map generic-1 interrupt to INT1
    //  - G: Map orientation-change interrupt to INT1
    //  - H: Map wakeup interrupt to INT1
    writeByte(REG_INT1_MAP, 0b00000001);

    // INT12_IO_CTRL: _AB_ _CD_
    //  - A: Use open-drive for INT2
    //  - B: Active level for INT2
    //  - C: Use open-drive for INT1
    //  - D: Active level for INT1
    writeByte(REG_INT12_IO_CTRL, (open_drive << 2) | (active_high << 1));
}

// Set the wakeup interrupt threshold.
// `threshold`: [0, 2047] corresponds to [0, max_range].
void BlueDot_BMA400::setWakeupThreshold(uint16_t threshold) {
    // WKUP_INT_CONFIG1: AAAA AAAA
    //  - A: Wakeup threshold (8 most-significant bits)
    writeByte(REG_WKUP_INT_CONFIG1, threshold >> 4);
}

// Set the wakeup reference.
// `x`, `y`, `z`: [-2048, 2047] correspond to [-max_range, +max_range].
void BlueDot_BMA400::setWakeupRef(uint16_t x, uint16_t y, uint16_t z) {
    // WKUP_INT_CONFIG2: AAAA AAAA
    //  - A: Wakeup reference X (8 most-significant bits)
    writeByte(REG_WKUP_INT_CONFIG2, x >> 4);
    // WKUP_INT_CONFIG3: AAAA AAAA
    //  - A: Wakeup reference Y (8 most-significant bits)
    writeByte(REG_WKUP_INT_CONFIG3, y >> 4);
    // WKUP_INT_CONFIG4: AAAA AAAA
    //  - A: Wakeup reference T (8 most-significant bits)
    writeByte(REG_WKUP_INT_CONFIG4, z >> 4);
}
//##########################################################################
uint8_t BlueDot_BMA400::readPowerMode(void) {
    uint8_t value;
    value = readByte(REG_STATUS);
    value = value & 0b00000110;
    value = value >> 1;

    return value;
}
//##########################################################################
uint8_t BlueDot_BMA400::readMeasurementRange(void) {
    uint8_t value;
    value = readByte(REG_ACC_CONFIG1);
    value = value & 0b11000000;
    value = value >> 6;

    return value;
}
//##########################################################################
uint8_t BlueDot_BMA400::readOutputDataRate(void) {
    uint8_t value;
    value = readByte(REG_ACC_CONFIG1);
    value = value & 0b00001111;

    return value;
}
//##########################################################################
uint8_t BlueDot_BMA400::readOversamplingRate(void) {
    uint8_t value;
    value = readByte(REG_ACC_CONFIG1);
    value = value & 0b00110000;
    value = value >> 4;

    return value;
}
//##########################################################################
// DATA READ FUNCTIONS
//##########################################################################
void BlueDot_BMA400::readData(void) {
    uint8_t rawData[6];
    Wire.beginTransmission(i2c_address);
    Wire.write(REG_ACC_X_LSB);
    Wire.endTransmission();
    Wire.requestFrom(i2c_address, (uint8_t)6);

    uint8_t acc_x_lsb = Wire.read();
    uint8_t acc_x_msb = Wire.read();
    uint8_t acc_y_lsb = Wire.read();
    uint8_t acc_y_msb = Wire.read();
    uint8_t acc_z_lsb = Wire.read();
    uint8_t acc_z_msb = Wire.read();

    raw_acc_x = ((acc_x_lsb | ((int)acc_x_msb << 8)) << 4) >> 4;
    raw_acc_y = ((acc_y_lsb | ((int)acc_y_msb << 8)) << 4) >> 4;
    raw_acc_z = ((acc_z_lsb | ((int)acc_z_msb << 8)) << 4) >> 4;
}
//##########################################################################
// BASIC FUNCTIONS
//##########################################################################
void BlueDot_BMA400::writeByte(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(i2c_address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}
//##########################################################################
uint8_t BlueDot_BMA400::readByte(uint8_t reg) {
    uint8_t value;
    Wire.beginTransmission(i2c_address);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(i2c_address, (uint8_t)1);
    value = Wire.read();
    return value;
}
