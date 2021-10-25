#include "I2C_Bus.hpp"
#include <cmath>

namespace Device
{

void I2C_Bus::resetControlRegisters()
{
	i2c_->CR1 = 0x0;
	i2c_->CR2 = 0x0;
}

void I2C_Bus::waitForBitSet(uint8_t bit_mask)
{
	while (!(i2c_->SR1 & bit_mask ))
	{
		asm volatile ("nop");
	}
}

void I2C_Bus::clearStatusFlags()
{
	volatile uint8_t temp = I2C1->SR1 | I2C1->SR2;
}

uint8_t I2C_Bus::readByte()
{
	waitForBitSet(I2C_SR1_RXNE);// wait for "Data register not empty"

	return i2c_->DR;
}

void I2C_Bus::putByteToBuffer(uint8_t* buffer , uint16_t pos)
{
	buffer[pos] = readByte();
}

void I2C_Bus::readSingleByte(uint8_t address , uint8_t* buffer)
{
	sendAddress(address);

	clearACK();
	stop();

	putByteToBuffer(buffer, 0);
}

void I2C_Bus::startReading(uint8_t address, uint8_t* buffer , uint16_t size)
{
	auto remaining_bytes_count {size};

	sendAddress(address);

	while ( remaining_bytes_count > 2 )
	{
		putByteToBuffer(buffer, size - remaining_bytes_count);

		setACK();  // Set the ACK bit to Acknowledge the data received

		--remaining_bytes_count;
	}
}

void I2C_Bus::finishReading(uint8_t* buffer , uint16_t size)
{
	// Read the penultimate byte
	putByteToBuffer(buffer, size - 2);

	clearACK();
	stop();

	// Read the Last byte
	putByteToBuffer(buffer, size - 1);
}

void I2C_Bus::beginTransmission(uint8_t address , uint8_t reg_adr)
{
	start();
	sendAddress(address);
	sendByte(reg_adr);
}

I2C_Bus::I2C_Bus(volatile I2C_TypeDef* bus):
		i2c_{bus}
{
	resetControlRegisters();
}

void I2C_Bus::resetBus()
{
	i2c_->CR1 |= I2C_CR1_SWRST;
	i2c_->CR1 &= ~I2C_CR1_SWRST;
}

void I2C_Bus::setupSlowSpeedMode(uint32_t periph_fraquency)
{
	auto tpclk { 1'000'000'000 / periph_fraquency };

	auto ccr{ static_cast<uint16_t>( std::round( 5000 / tpclk ) ) };

	auto mhz_fraquency {static_cast<uint8_t>( std::round(periph_fraquency / 1'000'000) )};

	auto trise { static_cast<uint8_t>( std::round(1'000 / tpclk ) ) + 1 };

	i2c_->CR2 |= mhz_fraquency << 0 ;
	i2c_->CCR =  ccr << 0;
	i2c_->TRISE = trise << 0;
}

void I2C_Bus::enable()
{
	i2c_->CR1 |= I2C_CR1_PE;
}

void I2C_Bus::start()
{
	i2c_->CR1 |= I2C_CR1_ACK;//here bcs when PE cleared , bit number 10 is cleared also
	i2c_->CR1 |= I2C_CR1_START;//generate start

	waitForBitSet(I2C_SR1_SB); // wait for generate START signal
}

void I2C_Bus::stop()
{
	i2c_->CR1 |= I2C_CR1_STOP;
}

void I2C_Bus::clearACK()
{
	i2c_->CR1 &= ~I2C_CR1_ACK;
}

void I2C_Bus::setACK()
{
	i2c_->CR1 |= I2C_CR1_ACK;
}

void I2C_Bus::sendByte (uint8_t data)
{
	waitForBitSet(I2C_SR1_TXE);  // wait for TXE bit to set

	i2c_->DR = data;

	waitForBitSet(I2C_SR1_BTF); // wait for Byte Transfer Finished bit to set
}

void I2C_Bus::sendAddress(uint8_t address)
{
	i2c_->DR = address;  //  send the address

	waitForBitSet(I2C_SR1_ADDR);  // wait for ADDR bit to set

	clearStatusFlags(); // read SR1 and SR2 to clear the ADDR bit
}

void I2C_Bus::readBytes(uint8_t address, uint8_t *buffer, uint16_t size)
{
	if (size == 1)
	{
		readSingleByte(address, buffer);
		return;
	}

	startReading(address, buffer, size);
	finishReading(buffer, size);
}

void I2C_Bus::writeToExternRegister(uint8_t address, uint8_t reg, uint8_t data)
{
	beginTransmission(address, reg);

	sendByte(data);
	stop();
}

void I2C_Bus::readFromExternRegister (uint8_t address, uint8_t reg, uint8_t* target, uint8_t size)
{
	beginTransmission(address, reg);

	start();  // repeated start
	readBytes( address | 0x1 , target, size);
	stop();
}

}
