#pragma once

#include "i2c.hpp"

//using reg_t = volatile const uint8_t;

typedef volatile const uint8_t reg_t;

reg_t dev_adr = 93 << 1;

reg_t TEMP_OUT_L = 0x2B;
reg_t TEMP_OUT_H = 0x2C;
reg_t CTRL_REG1  = 0x10;
reg_t CTRL_REG2  = 0x11;
reg_t STATUS_REG = 0x27;

reg_t PRESS_OUT_XL = 0x28;
reg_t PRESS_OUT_L = 0x29;
reg_t PRESS_OUT_H = 0x2A;

reg_t hts22_adr = 95 << 1;

reg_t hts22_CTRL_REG1 = 0x20;
reg_t hts22_TEMP_OUT_L = 0x2A;
reg_t hts22_TEMP_OUT_H = 0x2B;
reg_t hts22_THUMIDITY_OUT_L = 0x28;
reg_t hts22_THUMIDITY_OUT_H = 0x29;

reg_t hts22_H0_rH_x2 = 0x30;
reg_t hts22_H1_rH_x2 = 0x31;
reg_t hts22_H0_T0_OUT_0 = 0x36;
reg_t hts22_H0_T0_OUT_1 = 0x37;
reg_t hts22_H1_T0_OUT_0 = 0x3A;
reg_t hts22_H1_T0_OUT_1 = 0x3B;

void resetLPS22()
{
	reg_t soft_reset_val = 0x4;
	reg_t mem_reset_val = 0x80;
	reg_t auto_incremet_disable_val = 0x10;

	I2C_WriteToMem(dev_adr, CTRL_REG2 , soft_reset_val);
	I2C_WriteToMem(dev_adr, CTRL_REG2 , mem_reset_val);
	I2C_WriteToMem(dev_adr, CTRL_REG2 , auto_incremet_disable_val);
}

void setupLPS22()
{
	static reg_t config = 0x30;

	I2C_WriteToMem(dev_adr, CTRL_REG1 , config);
}

void setupHTS22()
{
	static reg_t config = 0x83;

	I2C_WriteToMem(hts22_adr, hts22_CTRL_REG1 , config);
}

void HTS221_Get_Temperature(int16_t *value)
{
	 int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
	 int16_t T0_degC, T1_degC;
	 uint8_t buffer[4], tmp;
	 int32_t tmp32;

	/*1. Read from 0x32 & 0x33 registers the value of coefficients T0_degC_x8 and T1_degC_x8*/
	 I2C_ReadFromMem(hts22_adr, 0x32, &buffer[0] , 1);
	 I2C_ReadFromMem(hts22_adr, 0x32, &buffer[1] , 1);

	/*2. Read from 0x35 register the value of the MSB bits of T1_degC and T0_degC */
	 I2C_ReadFromMem(hts22_adr, 0x35, &tmp , 1);

	/*Calculate the T0_degC and T1_degC values*/
	 T0_degC_x8_u16 = (((uint16_t)(tmp & 0x03)) << 8) | ((uint16_t)buffer[0]);
	 T1_degC_x8_u16 = (((uint16_t)(tmp & 0x0C)) << 6) | ((uint16_t)buffer[1]);

	 T0_degC = T0_degC_x8_u16>>3;
	 T1_degC = T1_degC_x8_u16>>3;


	/*3. Read from 0x3C & 0x3D registers the value of T0_OUT*/
	 I2C_ReadFromMem(hts22_adr, 0x3C , &buffer[0] , 1);
	 I2C_ReadFromMem(hts22_adr, 0x3D , &buffer[1] , 1);

	/*4. Read from 0x3E & 0x3F registers the value of T1_OUT*/
	 I2C_ReadFromMem(hts22_adr, 0x3E , &buffer[2] , 1);
	 I2C_ReadFromMem(hts22_adr, 0x3F , &buffer[3] , 1);

	 T0_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];
	 T1_out = (((uint16_t)buffer[3])<<8) | (uint16_t)buffer[2];

	/* 5.Read from 0x2A & 0x2B registers the value T_OUT (ADC_OUT).*/
	 I2C_ReadFromMem(hts22_adr, hts22_TEMP_OUT_L , &buffer[0] , 1);
	 I2C_ReadFromMem(hts22_adr, hts22_TEMP_OUT_H , &buffer[1] , 1);

	 T_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];

	/* 6. Compute the Temperature value by linear interpolation*/
	 tmp32 = ((int32_t)(T_out - T0_out)) * ((int32_t)(T1_degC - T0_degC)*10);
	 *value = tmp32 /(T1_out - T0_out) + T0_degC*10;
}

void HTS221_Get_Humidity(uint16_t* value)
{
	int16_t H0_T0_out, H1_T0_out, H_T_out;
	int16_t H0_rh, H1_rh;
	uint8_t buffer[2];
	int32_t tmp;

	/* 1. Read H0_rH and H1_rH coefficients*/
//	if(HTS221_ReadReg(HTS221_H0_RH_X2, 2, buffer))

	I2C_ReadFromMem(hts22_adr, hts22_H0_rH_x2 , &buffer[0] , 1);
	I2C_ReadFromMem(hts22_adr, hts22_H1_rH_x2 , &buffer[1] , 1);

	H0_rh = buffer[0]>>1;
	H1_rh = buffer[1]>>1;

	/*2. Read H0_T0_OUT */
//	if(HTS221_ReadReg(HTS221_H0_T0_OUT_L, 2, buffer))

	I2C_ReadFromMem(hts22_adr, hts22_H0_T0_OUT_0 , &buffer[0] , 1);
	I2C_ReadFromMem(hts22_adr, hts22_H0_T0_OUT_1 , &buffer[1] , 1);

	H0_T0_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];

	/*3. Read H1_T0_OUT */
//	if(HTS221_ReadReg(HTS221_H1_T0_OUT_L, 2, buffer))

	I2C_ReadFromMem(hts22_adr, hts22_H1_T0_OUT_0 , &buffer[0] , 1);
	I2C_ReadFromMem(hts22_adr, hts22_H1_T0_OUT_1 , &buffer[1] , 1);

	H1_T0_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];

	/*4. Read H_T_OUT */
//	if(HTS221_ReadReg(HTS221_HR_OUT_L_REG, 2, buffer))

	I2C_ReadFromMem(hts22_adr, hts22_THUMIDITY_OUT_L , &buffer[0] , 1);
	I2C_ReadFromMem(hts22_adr, hts22_THUMIDITY_OUT_H , &buffer[1] , 1);

	H_T_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];

	/*5. Compute the RH [%] value by linear interpolation */
	tmp = ((int32_t)(H_T_out - H0_T0_out)) * ((int32_t)(H1_rh - H0_rh)*10);
	*value = (tmp/(H1_T0_out - H0_T0_out) + H0_rh*10);

	/* Saturation condition*/
	 if(*value>1000) *value = 1000;

}

void readRegValue(uint8_t* result , uint8_t reg)
{
	I2C_ReadFromMem( dev_adr , reg, result , 1);
}

void writeRegValue(uint8_t reg , uint8_t value)
{
	I2C_WriteToMem(dev_adr, reg , value);
}

void readTemperatureRaw(int16_t* result)
{
	uint8_t tempL = 0;
	uint8_t tempH = 0;

	I2C_ReadFromMem(dev_adr , TEMP_OUT_L , &tempL , 1 );
	I2C_ReadFromMem(dev_adr , TEMP_OUT_H , &tempH , 1 );

	*result = ( tempH << 8 ) | tempL;
}

void readPressureRaw(uint32_t* result)
{
	uint8_t press_xl = 0;
	uint8_t press_l = 0;
	uint8_t press_H = 0;

	I2C_ReadFromMem(dev_adr , PRESS_OUT_XL  , &press_xl , 1 );
	I2C_ReadFromMem(dev_adr , PRESS_OUT_L  , &press_l  , 1 );
	I2C_ReadFromMem(dev_adr , PRESS_OUT_H , &press_H , 1 );

	*result = ( (press_H << 16) ) | (press_l << 8) | press_xl ;
}

float getTemperatureC(int16_t raw_temp)
{
	return 42.5 + (float)raw_temp / 480;
}

float readPressureMillibars(uint32_t raw_pressure)
{
  return (float)raw_pressure / 4096;
}
