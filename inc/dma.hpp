#pragma once

#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"
#include "../src/Peripheral/DMAController.hpp"

void dma1_clk_enable()
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
}

void dma1_ch7_init()
{
	using namespace Device;

	dma1_clk_enable();

	auto&& dma {DMA_ChannelController::get(DMA1_Channel7)};

	auto&& dma {DMA_ChannelController::get(DMA1_Channel4)};
	dma.enableTransferCompleteInterrupt();
	dma.setDirection(DMADirection::MemoryToPeriph);
	dma.enableMemoryIncrement();

	//enable memory increment
	DMA1_Channel7->CCR |= DMA_CCR_MINC;

	//set periph size = 8 bit
	DMA1_Channel7->CCR &= ~DMA_CCR_PSIZE;

	//set mem size = 8 bit
	DMA1_Channel7->CCR &= ~DMA_CCR_MSIZE;

	//set prior = 0
	DMA1_Channel7->CCR &= ~DMA_CCR_PL;

}

void dma1_ch4_init()
{
	using namespace Device;

	dma1_clk_enable();

	auto&& dma {DMA_ChannelController::get(DMA1_Channel4)};
	dma.enableTransferCompleteInterrupt();
	dma.setDirection(DMADirection::MemoryToPeriph);
	dma.enableMemoryIncrement();

	//set periph size = 8 bit
	DMA1_Channel4->CCR &= ~DMA_CCR_PSIZE;

	//set mem size = 8 bit
	DMA1_Channel4->CCR &= ~DMA_CCR_MSIZE;

	//set prior = 0
	DMA1_Channel4->CCR &= ~DMA_CCR_PL;
}
