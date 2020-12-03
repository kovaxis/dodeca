

#include <Arduino.h>

#define BMA400_CHIP_ID 0x90

enum PowerMode
{
    BMA400_SLEEP = 0x0,
    BMA400_LOWPOWER = 0x1,
    BMA400_NORMAL = 0x2,
};

enum Range
{
    BMA400_2G = 0x0,
    BMA400_4G = 0x1,
    BMA400_8G = 0x2,
    BMA400_16G = 0x3,
};

enum DataRate
{
    BMA400_12_5HZ = 0x5,
    BMA400_25HZ = 0x6,
    BMA400_50HZ = 0x7,
    BMA400_100HZ = 0x8,
    BMA400_200HZ = 0x9,
    BMA400_400HZ = 0xa,
    BMA400_800HZ = 0xb,
};

enum Oversampling
{
    BMA400_OSR_LOWEST = 0x0,
    BMA400_OSR_LOW = 0x1,
    BMA400_OSR_HIGH = 0x2,
    BMA400_OSR_HIGHEST = 0x3,
};

enum RefUpdate
{
    BMA400_MANUAL,
    BMA400_ONCE,
    BMA400_CONTINUOUS,
};

class BlueDot_BMA400
{
public:
    uint8_t i2c_address;
    int16_t raw_acc_x;
    int16_t raw_acc_y;
    int16_t raw_acc_z;

    BlueDot_BMA400(uint8_t i2c_address);
    uint8_t init();
    uint8_t checkID();

    void setPowerMode(uint8_t power_mode);
    void setMeasurementRange(uint8_t range);
    void setOutputDataRate(uint8_t data_rate);
    void setOversamplingRate(uint8_t osr);

    void enableWakeupInterrupts(uint8_t sample_count, uint8_t refupdate, bool open_drive, bool active_high);
    void setWakeupThreshold(uint16_t thres);
    void setWakeupRef(uint16_t x, uint16_t y, uint16_t z);

    uint8_t readPowerMode();
    uint8_t readMeasurementRange();
    uint8_t readOutputDataRate();
    uint8_t readOversamplingRate();

    void readData();

    uint8_t readByte(uint8_t reg);
    void writeByte(uint8_t reg, uint8_t value);
};
