#include "HTS22.hpp"

namespace Device
{

HTS22::HTS22(I2C_Bus& bus):
		I2C_Sensor{bus , dev_adr}
{
}

uint16_t HTS22::concatWord(uint8_t high_byte , uint8_t low_byte)
{
	return ( static_cast<uint16_t>( high_byte ) << 8 ) |
		     static_cast<uint16_t>( low_byte );
}

void HTS22::enable(const HTS22_OutputDataRate data_rate)
{
	static uint8_t config = /*0x83*/ 0;

	config |= static_cast<uint8_t>(data_rate);
	config |= 0x80; //power up bit

	writeByteToRegister(CTRL_REG1, config);
}

int16_t HTS22::getTemperature()
{
	 int16_t T0_out, T1_out;
	 int16_t T_out, T0_degC_x8;
	 int16_t T1_degC_x8;
	 int16_t T0_degC;
	 int16_t T1_degC;
	 uint8_t buffer[4];
	 uint8_t tmp;
	 int32_t tmp32;

	/*1. Read from 0x32 & 0x33 registers the value of coefficients T0_degC_x8 and T1_degC_x8*/

	 readByteFromRegister(0x32, &buffer[0]);
	 readByteFromRegister(0x33, &buffer[1]);

	/*2. Read from 0x35 register the value of the MSB bits of T1_degC and T0_degC */

	 readByteFromRegister(0x35 , &tmp);

	/*Calculate the T0_degC and T1_degC values*/

	 T0_degC_x8 = concatWord(tmp & 0x03, buffer[0]);

	 T1_degC_x8 = ( static_cast<uint16_t>( tmp & 0x0C) << 6 ) |
			 	 	static_cast<uint16_t>( buffer[1] );

	 T0_degC = T0_degC_x8 >> 3;
	 T1_degC = T1_degC_x8 >> 3;

	/*3. Read from 0x3C & 0x3D registers the value of T0_OUT*/

	 readByteFromRegister(0x3C, &buffer[0]);
	 readByteFromRegister(0x3D, &buffer[1]);

	/*4. Read from 0x3E & 0x3F registers the value of T1_OUT*/

	 readByteFromRegister(0x3E, &buffer[2]);
	 readByteFromRegister(0x3F, &buffer[3]);

	 T0_out = concatWord(buffer[1] , buffer[0]);
	 T1_out = concatWord(buffer[3] , buffer[2]);

	/* 5.Read from 0x2A & 0x2B registers the value T_OUT (ADC_OUT).*/

	 readByteFromRegister(TEMP_OUT_L, &buffer[0] );
	 readByteFromRegister(TEMP_OUT_H, &buffer[1] );

	 T_out = concatWord(buffer[1] , buffer[0]);

	/* 6. Compute the Temperature value by linear interpolation*/
	 tmp32 = ( static_cast<int32_t>( T_out - T0_out) ) *
			 ( static_cast<int32_t>( T1_degC - T0_degC ) * 10 );

	 return tmp32 / (T1_out - T0_out) + T0_degC * 10;
}

uint16_t HTS22::getHumidity()
{
	int16_t H0_T0_out, H1_T0_out, H_T_out;
	int16_t H0_rh, H1_rh;
	uint8_t buffer[2];
	int32_t tmp;

	uint16_t result{0};

	/* 1. Read H0_rH and H1_rH coefficients*/

	readByteFromRegister(H0_rH_x2, &buffer[0]);
	readByteFromRegister(H1_rH_x2, &buffer[1]);

	H0_rh = buffer[0] >> 1;
	H1_rh = buffer[1] >> 1;

	/*2. Read H0_T0_OUT */

	readByteFromRegister(H0_T0_OUT_0, &buffer[0]);
	readByteFromRegister(H0_T0_OUT_1, &buffer[1]);

	H0_T0_out = concatWord(buffer[1], buffer[0]);

	/*3. Read H1_T0_OUT */

	readByteFromRegister(H1_T0_OUT_0, &buffer[0]);
	readByteFromRegister(H1_T0_OUT_1, &buffer[1]);

	H1_T0_out = concatWord(buffer[1], buffer[0]);

	/*4. Read H_T_OUT */

	readByteFromRegister(THUMIDITY_OUT_L, &buffer[0]);
	readByteFromRegister(THUMIDITY_OUT_H, &buffer[1]);

	H_T_out = concatWord(buffer[1], buffer[0]);

	/*5. Compute the RH [%] value by linear interpolation */
	tmp = static_cast<int32_t>( H_T_out - H0_T0_out) *
		  static_cast<int32_t>(H1_rh - H0_rh) *
		  10;

	result = tmp / ( H1_T0_out - H0_T0_out ) + H0_rh * 10 ;

	//MAYBE DELETE IT ?
	/* Saturation condition*/
	 if( result > 1000 )
	 {
		 return 1000;
	 }

	 return result;
}

}