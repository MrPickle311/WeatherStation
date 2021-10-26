#pragma once

#include "../../Peripheral/I2C_Bus.hpp"

namespace Device
{

class I2C_Sensor
{
protected:
	I2C_Bus& i2c_bus_;
private:
	uint8_t dev_adr_;
protected:
	void readByteFromRegister(uint8_t reg , uint8_t* target);
	void writeByteToRegister(uint8_t reg , uint8_t what);
protected:
	I2C_Sensor(I2C_Bus& bus , uint8_t device_address);
};

}
