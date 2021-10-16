#pragma once

#include "i2c.hpp"

volatile uint8_t dev_adr = 93 << 1;

volatile const uint8_t TEMP_OUT_L = 0x2B;
volatile const uint8_t TEMP_OUT_H = 0x2C;
volatile const uint8_t CTRL_REG1  = 0x10;
volatile const uint8_t STATUS_REG = 0x27;

volatile const uint8_t PRESS_OUT_XL = 0x28;
volatile const uint8_t PRESS_OUT_L = 0x29;
volatile const uint8_t PRESS_OUT_H = 0x2A;

void readRegValue(uint8_t* result , uint8_t reg)
{
	I2C_ReadFromMem( dev_adr , reg, result , 1);
}

void writeRegValue(uint8_t reg , uint8_t value)
{
	I2C_WriteToMem(dev_adr, reg , value);
}

//HAL_StatusTypeDef readTemperatureRaw(int16_t* result)
//{
//	return  HAL_I2C_Mem_Read(&hi2c1, dev_adr , TEMP_OUT_L | 0x80 , 1 , (uint8_t*)result , 2 , 20 );
//}
//
//HAL_StatusTypeDef readPressureRaw(int32_t* result)
//{
//	return  HAL_I2C_Mem_Read(&hi2c1, dev_adr , PRESS_OUT_XL | 0x80 , 1 , (uint8_t*)result , 3 , 20 );
//}
//
//float getTemperatureC(int16_t raw_temp)
//{
//	return 42.5 + (float)raw_temp / 480;
//}
//
//float readPressureMillibars(int32_t raw_pressure)
//{
//  return (float)raw_pressure / 4096;
//}
