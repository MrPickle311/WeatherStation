#include "I2CSensor.hpp"

namespace Device
{

void I2C_Sensor::readByteFromRegister(uint8_t reg , uint8_t* target)
{
	i2c_bus_.readFromExternRegister(dev_adr_, reg, target, 1);
}

void I2C_Sensor::writeByteToRegister(uint8_t reg , uint8_t what)
{
	i2c_bus_.writeToExternRegister(dev_adr_ , reg , what);
}

I2C_Sensor::I2C_Sensor(I2C_Bus& bus, uint8_t device_address):
		i2c_bus_{bus} ,
		dev_adr_{device_address}
{
}



}
