#pragma once

#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"
#include "../src/Devices/GPIO_Controller.hpp"
#include "../src/Devices/I2C_Bus.hpp"

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


//	I2C1->CR1 = 0x0;
//	I2C1->CR2 = 0x0;

	enableGPIOB();
	enableI2CClock();
}

void pullUpI2CPins()
{
//	GPIOB->ODR |= GPIO_ODR_ODR8 | GPIO_ODR_ODR9;
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
//	GPIOB->CRL |= GPIO_CRL_CNF7 | GPIO_CRL_MODE7;

	//scl
	GPIO_Device<GPIO_Port::B,6>::getInstance().setAlternateOpenDrain(PinFrequency::F_50MHz);
//	GPIOB->CRL |= GPIO_CRL_CNF6 | GPIO_CRL_MODE6;

	pullUpI2CPins();
//	remapI2CPins(); not working
}

void resetI2CBus()
{
	Device::I2C_Bus::get(I2C1).resetBus();
//	I2C1->CR1 |= I2C_CR1_SWRST;
//	I2C1->CR1 &= ~I2C_CR1_SWRST;
}

void setupI2CSpeed()
{
	I2C1->CR2 |= ( 32 << 0 );
	I2C1->CCR = ( 160 << 0 );
	I2C1->TRISE = 33;
}

void enableI2C()
{
	Device::I2C_Bus::get(I2C1).enable();
//	I2C1->CR1 |= I2C_CR1_PE;
}

void I2C_Start()
{
	I2C1->CR1 |= I2C_CR1_ACK;//here bcs when PE cleared , bit number 10 is cleared also
	I2C1->CR1 |= I2C_CR1_START;//generate start

	while (!(I2C1->SR1 & I2C_SR1_SB ))  // wait for generate START signal
	{
		asm volatile ("nop");
	}
}

void I2C_Stop()
{
	I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C_ClearACK()
{
	I2C1->CR1 &= ~I2C_CR1_ACK;
}

void I2C_SetACK()
{
	I2C1->CR1 |= I2C_CR1_ACK;
}


void I2C_SendByte (uint8_t data)
{

	while ( ! ( I2C1->SR1 & I2C_SR1_TXE ) )  // wait for TXE bit to set
	{
		asm volatile ("nop");
	}

	I2C1->DR = data;

	while ( ! ( I2C1->SR1 & I2C_SR1_BTF ))  // wait for BTF bit to set
	{
		asm volatile ("nop");
	}
}

uint8_t I2C_ReadByte()
{
	while (!(I2C1->SR1 & (1<<6)))  // wait for "Data register not empty"
	{
		asm volatile ("nop");
	}

	return I2C1->DR;
}

void I2C_SendAddress(uint8_t address)
{
	I2C1->DR = address;  //  send the address

	while (!(I2C1->SR1 & I2C_SR1_ADDR ))  // wait for ADDR bit to set
	{
		asm volatile ("nop");
	}

	volatile uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit
}

void I2C_Read (uint8_t address, uint8_t *buffer, uint8_t size)
{
	int remaining_bytes_count = size;

	if (size == 1)
	{
		I2C_SendAddress(address);

		I2C_ClearACK();  // clear the ACK bit
		I2C_Stop();  // Stop I2C

		buffer[size-remaining_bytes_count] = I2C_ReadByte();
	}
	else
	{
		I2C_SendAddress(address);

		while ( remaining_bytes_count > 2 )
		{
			buffer[size-remaining_bytes_count] = I2C_ReadByte();

			I2C_SetACK();  // Set the ACK bit to Acknowledge the data received

			--remaining_bytes_count;
		}

		// Read the penultimate byte
		buffer[size-remaining_bytes_count] = I2C_ReadByte();

		I2C_ClearACK();  // clear the ACK bit

		I2C_Stop();  // Stop I2C

		--remaining_bytes_count;

		// Read the Last BYTE
		buffer[size-remaining_bytes_count] = I2C_ReadByte();
	}
}

void I2C_WriteToMem (uint8_t address, uint8_t reg, uint8_t data)
{
	I2C_Start();
	I2C_SendAddress(address);
	I2C_SendByte(reg);
	I2C_SendByte(data);
	I2C_Stop();
}

void I2C_ReadFromMem (uint8_t address, uint8_t reg, uint8_t* target, uint8_t size)
{
	I2C_Start();
	I2C_SendAddress(address);
	I2C_SendByte(reg);
	I2C_Start();  // repeated start
	I2C_Read( address | 0x1 , target, size);
	I2C_Stop();
}

void setupI2C()
{
	setupI2Clocks();
	setupI2CPins();
	resetI2CBus();
	setupI2CSpeed();
	enableI2C();
}
