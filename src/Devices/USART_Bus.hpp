#pragma once

#include "DeviceTraits.hpp"
#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"

namespace Device
{

class USART_Bus : public Multiton<USART_Bus , USART_TypeDef* >
{
	friend class Multiton<USART_Bus , USART_TypeDef* >;
private:
	USART_TypeDef* usart_;
private:
	void resetControlRegisters();
private:
	USART_Bus(USART_TypeDef* usart);
public:
	void enableBus();

	void enableTransmitter();
	void enableReceiver();

	void enableDMAForTransmitter();
	void enableDMAForReceiver();

	void setupBaudRate(uint32_t baud_value, uint32_t peripheral_frequency);
};

}
