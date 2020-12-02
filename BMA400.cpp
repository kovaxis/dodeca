




#if defined(_AVR_)
#include <util/delay.h>
#endif

#include "BMA400.h"
#include "Wire.h"


BlueDot_BMA400::BlueDot_BMA400()
{
	I2CAddress;
}



uint8_t BlueDot_BMA400::init(void)
{
	Wire.begin();
	
	return checkID();

}
//##########################################################################
//SET UP FUNCTIONS
//##########################################################################
uint8_t BlueDot_BMA400::checkID(void)
{
	uint8_t chipID;
	chipID = readByte(BMA400_CHIP_ID);
	return chipID;		
	
}
//##########################################################################
void BlueDot_BMA400::setPowerMode(uint8_t power_mode)
{
	uint8_t reg;
	reg = readByte(BMA400_ACC_CONFIG0);
	reg = reg & 0b11111100;
	
	reg = reg | power_mode;
	writeByte(BMA400_ACC_CONFIG0, reg);	
	
}
//##########################################################################
void BlueDot_BMA400::setMeasurementRange(uint8_t range)
{
	uint8_t reg;
	reg = readByte(BMA400_ACC_CONFIG1);
	reg = reg & 0b00111111;
	
	reg = reg | (range << 6);
	writeByte(BMA400_ACC_CONFIG1, reg);	
	
}
//##########################################################################
void BlueDot_BMA400::setOutputDataRate(uint8_t data_rate)
{
	uint8_t reg;
	reg = readByte(BMA400_ACC_CONFIG1);
	reg = reg & 0b11110000;
	
	reg = reg | data_rate;
	writeByte(BMA400_ACC_CONFIG1, reg);	
	
}
//##########################################################################
void BlueDot_BMA400::setOversamplingRate(uint8_t osr)
{
	uint8_t reg;
	reg = readByte(BMA400_ACC_CONFIG1);
	reg = reg & 0b11001111;
	
	reg = reg | (osr << 4);
	writeByte(BMA400_ACC_CONFIG1, reg);	
	
}
//##########################################################################
uint8_t BlueDot_BMA400::readPowerMode(void)
{
	uint8_t value;
	value = readByte(BMA400_STATUS);
	value = value & 0b00000110;
	value = value >> 1;
	
	return value;		
	
}
//##########################################################################
uint8_t BlueDot_BMA400::readMeasurementRange(void)
{
	uint8_t value;
	value = readByte(BMA400_ACC_CONFIG1);
	value = value & 0b11000000;
	value = value >> 6;
	
	return value;		
	
}
//##########################################################################
uint8_t BlueDot_BMA400::readOutputDataRate(void)
{
	uint8_t value;
	value = readByte(BMA400_ACC_CONFIG1);
	value = value & 0b00001111;	
	
	return value;	
	
}
//##########################################################################
uint8_t BlueDot_BMA400::readOversamplingRate(void)
{
	uint8_t value;
	value = readByte(BMA400_ACC_CONFIG1);
	value = value & 0b00110000;
	value = value >> 4;
	
	return value;	
	
}
//##########################################################################
//DATA READ FUNCTIONS
//##########################################################################
void BlueDot_BMA400::readData(void)
{
	uint8_t rawData[6];
	Wire.beginTransmission(I2CAddress);
	Wire.write(BMA400_ACC_X_LSB);
	Wire.endTransmission();
	Wire.requestFrom(I2CAddress, (uint8_t) 6);
	
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
//BASIC FUNCTIONS
//##########################################################################
void BlueDot_BMA400::writeByte(uint8_t reg, uint8_t value)
{
	Wire.beginTransmission(I2CAddress);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();	
}
//##########################################################################
uint8_t BlueDot_BMA400::readByte(uint8_t reg)
{
	uint8_t value;
	Wire.beginTransmission(I2CAddress);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(I2CAddress, (uint8_t) 1);		
	value = Wire.read();		
	return value;	
}
