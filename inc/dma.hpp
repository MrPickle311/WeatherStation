#pragma once

#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"
#include "../src/Peripheral/DMAController.hpp"
#include "../src/Peripheral/RCC_Controller.hpp"

void enableOutputDMAChannel(DMA_Channel_TypeDef* channel)
{
	using namespace Device;

	RCC_Controller::getInstance().enableDMAController();

	auto&& dma {DMA_ChannelController::get(channel)};

	dma.enableTransferCompleteInterrupt();
	dma.setDirection(DMADirection::MemoryToPeriph);
	dma.enableMemoryIncrement();

	//set periph size = 8 bit
	channel->CCR &= ~DMA_CCR_PSIZE;

	//set mem size = 8 bit
	channel->CCR &= ~DMA_CCR_MSIZE;

	//set prior = 0
	channel->CCR &= ~DMA_CCR_PL;
}

void dma1_init()
{
//	enableOutputDMAChannel(DMA1_Channel7);
	enableOutputDMAChannel(DMA1_Channel4);//datasheet s.282
}
