#include "HTS22.hpp"

namespace Device
{

HTS22::HTS22(I2C_Bus& bus):
		I2C_Sensor{bus , dev_adr}
{
}

void HTS22::enable(const HTS22_OutputDataRate)
{

}

int16_t HTS22::getTemperature()
{

}

uint16_t HTS22::getHumidity()
{

}

}
