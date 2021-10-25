#pragma once

#include "DeviceTraits.hpp"
#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"

namespace Device
{

class I2C_Bus : public Multiton<I2C_Bus , I2C_TypeDef* >
{
	friend class Multiton<I2C_Bus , I2C_TypeDef*>;
private:
	volatile I2C_TypeDef* i2c_;
private:
	void waitForBitSet(uint8_t bit_mask);
	void resetControlRegisters();
	void clearStatusFlags();
	uint8_t readByte();
	void putByteToBuffer(uint8_t* buffer , uint16_t pos);

	void readSingleByte(uint8_t address , uint8_t* buffer);

	void startReading(uint8_t address, uint8_t* buffer , uint16_t size);
	void finishReading(uint8_t* buffer , uint16_t size);

	void beginTransmission(uint8_t address , uint8_t reg_adr);
private:
	I2C_Bus(volatile I2C_TypeDef* bus);
public:
	void resetBus();
	void setupSlowSpeedMode(uint32_t periph_fraquency);
	void enable();

	void start();
	void stop();

	void clearACK();
	void setACK();

	void sendByte (uint8_t data);

	void sendAddress(uint8_t address);
	void readBytes(uint8_t address, uint8_t *buffer, uint16_t size);

	void writeToExternRegister(uint8_t address, uint8_t reg, uint8_t data);
	void readFromExternRegister (uint8_t address, uint8_t reg, uint8_t* target, uint8_t size);
};

}
