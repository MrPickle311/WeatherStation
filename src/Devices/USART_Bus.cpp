#include "USART_Bus.hpp"
#include <cmath>

namespace Device
{
USART_Bus::USART_Bus(volatile USART_TypeDef* usart):
		usart_{usart}
{
	resetControlRegisters();
}

void USART_Bus::resetControlRegisters()
{
	CLEAR_REG(usart_->CR1);
	CLEAR_REG(usart_->CR2);
	CLEAR_REG(usart_->CR3);
}

void USART_Bus::enableBus()
{
	usart_->CR1 |= USART_CR1_UE;
}

void USART_Bus::enableTransmitter()
{
	usart_->CR1 |=  USART_CR1_TE ;  // enable rx and tx
}

void USART_Bus::enableReceiver()
{
	usart_->CR1 |=  USART_CR1_RE ;  // enable rx and tx
}

void USART_Bus::enableDMAForTransmitter()
{
	usart_->CR3 |= USART_CR3_DMAT; // enable dma mode
}

void USART_Bus::enableDMAForReceiver()
{
	usart_->CR3 |= USART_CR3_DMAR;
}

void USART_Bus::setupBaudRate(uint32_t baud_value, uint32_t peripheral_frequency)
{
	double temp_result { peripheral_frequency / ( baud_value * 16 ) };

	uint16_t mantisa { static_cast<uint16_t>( std::trunc(temp_result) ) };

	uint8_t fraction { static_cast<uint8_t>( std::round( ( temp_result - mantisa ) * 16 ) )  };

	usart_->BRR = (mantisa << 4) | (fraction << 0);
}


}
