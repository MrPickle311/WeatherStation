#pragma once

#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"

void uasrtSendByte (uint8_t byte)
{
   while (! ( USART2->SR & USART_SR_TXE ) ) // wait until buffer ready
   {
	   asm volatile ("nop");
   }

   USART2->DR = byte;//push byte
}

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
	enableGPIOB();
	enableI2CClock();
}

void pullUpI2CPins()
{
	GPIOB->ODR |= GPIO_ODR_ODR8 | GPIO_ODR_ODR9;
}

void remapI2CPins()
{
	AFIO->MAPR |= AFIO_MAPR_I2C1_REMAP;
}

void setupI2CPins()
{
	//select alternate functions for pb9 and pb8 , up to 50 mhz , open-drain

	//sda
	GPIOB->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_CNF9_0 | GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0;

	//scl
	GPIOB->CRH |= GPIO_CRH_CNF8_1 | GPIO_CRH_CNF8_0 | GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0;

	pullUpI2CPins();
	remapI2CPins();
}

void resetI2CBus()
{
	I2C1->CR1 |= I2C_CR1_SWRST;
	I2C1->CR1 &= ~I2C_CR1_SWRST;
}

void setupI2CSpeed()
{
	I2C1->CR2 |= ( 32 << 0 );
	I2C1->CCR = ( 160 << 0 );
	I2C1->TRISE = 33;
}

void enableI2C()
{
	I2C1->CR1 |= I2C_CR1_PE;
}

void I2C_Start()
{
	I2C1->CR1 |= I2C_CR1_ACK;//enable ack generation
	I2C1->CR1 |= I2C_CR1_START;//generate start
}

void I2C_Stop()
{
	I2C1->CR1 |= I2C_CR1_STOP;
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

void I2C_SendAddress(uint8_t address)
{
	I2C1->DR = address;  //  send the address

	while (!(I2C1->SR1 & I2C_SR1_ADDR ))  // wait for ADDR bit to set
	{
		asm volatile ("nop");
	}

	volatile uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit
}

void I2C_Read (uint8_t Address, uint8_t *buffer, uint8_t size)
{
/**** STEPS FOLLOWED  ************
1. If only 1 BYTE needs to be Read
	a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
	b) the Acknowledge disable is made during EV6 (before ADDR flag is cleared) and the STOP condition generation is made after EV6
	c) Wait for the RXNE (Receive Buffer not Empty) bit to set
	d) Read the data from the DR
2. If Multiple BYTES needs to be read
  a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
	b) Clear the ADDR bit by reading the SR1 and SR2 Registers
	c) Wait for the RXNE (Receive buffer not empty) bit to set
	d) Read the data from the DR
	e) Generate the Acknowlegment by settint the ACK (bit 10 in SR1)
	f) To generate the nonacknowledge pulse after the last received data byte, the ACK bit must be cleared just after reading the
		 second last data byte (after second last RxNE event)
	g) In order to generate the Stop/Restart condition, software must set the STOP/START bit
	   after reading the second last data byte (after the second last RxNE event)
*/

	int remaining = size;

/**** STEP 1 ****/
	if (size == 1)
	{
		/**** STEP 1-a ****/
		I2C1->DR = Address;  //  send the address
		while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set

		/**** STEP 1-b ****/
		I2C1->CR1 &= ~(1<<10);  // clear the ACK bit
		uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit.... EV6 condition
		I2C1->CR1 |= (1<<9);  // Stop I2C

		/**** STEP 1-c ****/
		while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set

		/**** STEP 1-d ****/
		buffer[size-remaining] = I2C1->DR;  // Read the data from the DATA REGISTER

	}

/**** STEP 2 ****/
	else
	{
		/**** STEP 2-a ****/
		I2C1->DR = Address;  //  send the address
		while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set

		/**** STEP 2-b ****/
		volatile uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit

		while (remaining>2)
		{
			/**** STEP 2-c ****/
			while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set

			/**** STEP 2-d ****/
			buffer[size-remaining] = I2C1->DR;  // copy the data into the buffer

			/**** STEP 2-e ****/
			I2C1->CR1 |= 1<<10;  // Set the ACK bit to Acknowledge the data received

			remaining--;
		}

		// Read the SECOND LAST BYTE
		while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2C1->DR;

		/**** STEP 2-f ****/
		I2C1->CR1 &= ~(1<<10);  // clear the ACK bit

		/**** STEP 2-g ****/
		I2C1->CR1 |= (1<<9);  // Stop I2C

		remaining--;

		// Read the Last BYTE
		while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2C1->DR;  // copy the data into the buffer
	}

}

void MPU_Write (uint8_t address, uint8_t reg, uint8_t data)
{
	I2C_Start ();

	I2C_SendAddress(address);
	uasrtSendByte(90);
//ok

	I2C_SendByte(reg);

	I2C_SendByte(data);
	I2C_Stop ();
}

void MPU_Read (uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t size)
{
	I2C_Start ();
	I2C_SendAddress(address);
	I2C_SendByte(reg);
	I2C_Start ();  // repeated start
	I2C_Read (address+0x01, buffer, size);
	I2C_Stop ();
}

void setupI2C()
{
	setupI2Clocks();
	setupI2CPins();
	resetI2CBus();
	setupI2CSpeed();
	enableI2C();
}
