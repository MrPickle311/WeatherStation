#pragma once

#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"
#include <any>
#include "../Peripheral/DeviceTraits.hpp"

namespace Device
{

class DMA_Controller
{

};

enum class DMADirection : uint8_t
{
	MemoryToPeriph,
	PeriphToMemory
};

class DMA_ChannelController : public Multiton<DMA_ChannelController,DMA_Channel_TypeDef*>
{
	friend class Multiton<DMA_ChannelController , DMA_Channel_TypeDef* >;
private:
	volatile DMA_Channel_TypeDef* channel_;
private:
	DMA_ChannelController(volatile DMA_Channel_TypeDef* channel);
private:
	void setControlRegisterBit(uint8_t bit_mask);
public:
	void enableHalfTransferCompleteInterrupt();
	void enableTransferCompleteInterrupt();
	void enableTransferErrorInterrupt();

	void setDirection(DMADirection direction);

	void enableCircularMode();

	void enableMemoryIncrement();
	void enablePeripheralIncrement();

	void setPeripheralAddress(__IO uint32_t* periph_adr );

	template<typename T>
	void setMemoryAddress(T* mem_adr)
	{
		channel_->CMAR =  reinterpret_cast<uint32_t>(mem_adr);
	}

	void start(uint16_t bytes_count);
};



}
