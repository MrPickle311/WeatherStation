#pragma once

#include "DeviceTraits.hpp"
#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"

namespace Device
{
//static std::map<Key*, ObjectType> instances_;

class USART_Bus : public Multiton<USART_Bus , USART_TypeDef* >
{
	friend class Multiton<USART_Bus , USART_TypeDef* >;
	friend class std::pair<USART_Bus , USART_TypeDef* >;
	friend class std::tuple<USART_Bus , USART_TypeDef* >;
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
