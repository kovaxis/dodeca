

#include <Arduino.h>



#define BMA400_CHIP_ID 0x90
#define BMA400_CHIP_ID_REGISTER			0x00





 


enum Coefficients
{
	BMA400_STATUS		=		0x03,
	
	BMA400_ACC_X_LSB	=		0x04,
	BMA400_ACC_X_MSB	=		0x05,
	BMA400_ACC_Y_LSB	=		0x06,
	BMA400_ACC_Y_MSB	=		0x07,
	BMA400_ACC_Z_LSB	=		0x08,
	BMA400_ACC_Z_MSB	=		0x09,
	
	BMA400_TEMP_ADDR	=		0x11,
	BMA400_ACC_CONFIG0  =		0x19,
	BMA400_ACC_CONFIG1  =		0x1A,
	
};


class BlueDot_BMA400 
{
 public: 
  
  uint8_t I2CAddress;
  int16_t raw_acc_x;
  int16_t raw_acc_y;
  int16_t raw_acc_z;
  
  
  
  BlueDot_BMA400();
  uint8_t init(void);
  uint8_t checkID(void);
  
  void setPowerMode(uint8_t power_mode);
  void setMeasurementRange(uint8_t range);
  void setOutputDataRate(uint8_t data_rate);
  void setOversamplingRate(uint8_t osr);
  
  uint8_t readPowerMode();
  uint8_t readMeasurementRange();
  uint8_t readOutputDataRate();
  uint8_t readOversamplingRate();
  
  void readData();
  
  uint8_t readByte(uint8_t reg);
  void writeByte(uint8_t reg, uint8_t value);


};
