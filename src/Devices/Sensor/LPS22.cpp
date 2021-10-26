#include "LPS22.hpp"

namespace Device
{

LPS_22::LPS_22(I2C_Bus& bus):
		I2C_Sensor{bus , dev_adr}
{
}

uint32_t LPS_22::readPressureRaw()
{
	static uint8_t press_xl = 0;
	static uint8_t press_l = 0;
	static uint8_t press_H = 0;

	readByteFromRegister(PRESS_OUT_XL,&press_xl);
	readByteFromRegister(PRESS_OUT_L, &press_l);
	readByteFromRegister(PRESS_OUT_H, &press_H);

	return ( press_H << 16 )  | ( press_l << 8 ) | press_xl ;
}

void LPS_22::enable(const LPS22_OutputDataBitRate rate)
{
	writeByteToRegister(CTRL_REG1, static_cast<uint8_t>( rate ) << 4);
}

}
