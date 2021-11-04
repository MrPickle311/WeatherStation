#pragma once

#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"
#include "../src/Peripheral/GPIO_Controller.hpp"
#include "../src/Peripheral/I2C_Bus.hpp"
#include "../src/Peripheral/RCC_Controller.hpp"

void setupI2Clocks()
{
	RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;

	auto&& rcc {Device::RCC_Controller::getInstance()};

	rcc.enableGPIOPort(Device::GPIO_Enable::B);
	rcc.enableI2C1Bus();
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

	//datasheet s.168
	//pic
	//https://www.google.com/url?sa=i&url=https%3A%2F%2Fos.mbed.com%2Fteams%2FTVZ-Mechatronics-Team%2Fwiki%2FDigital-inputs-and-outputs&psig=AOvVaw01tFywh_vUHhvY2q2heOJM&ust=1636124430988000&source=images&cd=vfe&ved=0CAsQjRxqFwoTCMiMvM78_vMCFQAAAAAdAAAAABBc
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
	Device::I2C_Bus::get(I2C1).resetBus();//datasheet s.772 , see "Note: "
	Device::I2C_Bus::get(I2C1).setupSlowSpeedMode(32'000'000);// standard speed* 100kHz datasheet s.752


	Device::I2C_Bus::get(I2C1).enable();
}
