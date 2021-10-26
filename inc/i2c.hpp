#pragma once

#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"
#include "../src/Peripheral/GPIO_Controller.hpp"
#include "../src/Peripheral/I2C_Bus.hpp"

void enableGPIOB()
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
}

void enableI2CClock()
{
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
}

void setupI2Clocks()
{
	RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;

	enableGPIOB();
	enableI2CClock();
}

void pullUpI2CPins()
{
	using namespace Device;

	GPIO_Device<GPIO_Port::B,7>::getInstance().setHigh();
	GPIO_Device<GPIO_Port::B,6>::getInstance().setHigh();
}

void remapI2CPins()
{
	AFIO->MAPR |= AFIO_MAPR_I2C1_REMAP;
}

void setupI2CPins()
{
	using namespace Device;
	//select alternate functions for pb9 and pb8 , up to 50 mhz , open-drain

	//sda
	GPIO_Device<GPIO_Port::B,7>::getInstance().setAlternateOpenDrain(PinFrequency::F_50MHz);

	//scl
	GPIO_Device<GPIO_Port::B,6>::getInstance().setAlternateOpenDrain(PinFrequency::F_50MHz);

	pullUpI2CPins();
//	remapI2CPins(); not working
}

void setupI2C()
{
	setupI2Clocks();
	setupI2CPins();
	Device::I2C_Bus::get(I2C1).resetBus();
	Device::I2C_Bus::get(I2C1).setupSlowSpeedMode(32'000'000);
	Device::I2C_Bus::get(I2C1).enable();
}
