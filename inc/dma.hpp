#pragma once

#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"
#include "../src/Devices/DMAController.hpp"


void dma1_clk_enable()
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
}

//uart tx dma

void enable_dma1_ch7_irq()
{
	//enable half transfer cpl , transfer cpl and transfer error irq's

	DMA1_Channel7->CCR |= DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_TEIE;
}

void dma1_ch7_init()
{
	dma1_clk_enable();
	enable_dma1_ch7_irq();

	//read from periph
//	DMA1_Channel7->CCR &= ~DMA_CCR_DIR ;

	//read from memory
	DMA1_Channel7->CCR |= DMA_CCR_DIR ;

	//enable circ mode
//	DMA1_Channel7->CCR |= DMA_CCR_CIRC;

	//enable memory increment
	DMA1_Channel7->CCR |= DMA_CCR_MINC;

	//set periph size = 8 bit
	DMA1_Channel7->CCR &= ~DMA_CCR_PSIZE;

	//set mem size = 8 bit
	DMA1_Channel7->CCR &= ~DMA_CCR_MSIZE;

	//set prior = 0
	DMA1_Channel7->CCR &= ~DMA_CCR_PL;

}

void dma1_ch7_config(uint32_t periph_adr , uint32_t mem_adr /*, uint16_t data_size*/)
{
//	DMA1_Channel7->CNDTR = data_size;// set bytes count to transfer

	DMA1_Channel7->CPAR = periph_adr;//set in periph src adr

	DMA1_Channel7->CMAR = mem_adr;//set in memory destination adr

//	DMA1_Channel7->CCR |= DMA_CCR_EN;
}

void dma1_ch7_start(uint8_t bytes_count)
{
	DMA1_Channel7->CNDTR = bytes_count;// set bytes count to transfer
	DMA1_Channel7->CCR |= DMA_CCR_EN;
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

void dma1_ch4_config(uint32_t periph_adr , uint32_t mem_adr /*, uint16_t data_size*/)
{
//	DMA1_Channel7->CNDTR = data_size;// set bytes count to transfer

	DMA1_Channel4->CPAR = periph_adr;//set in periph src adr

	DMA1_Channel4->CMAR = mem_adr;//set in memory destination adr

//	DMA1_Channel7->CCR |= DMA_CCR_EN;
}

void dma1_ch4_start(uint8_t bytes_count)
{
	DMA1_Channel4->CNDTR = bytes_count;// set bytes count to transfer
	DMA1_Channel4->CCR |= DMA_CCR_EN;
}





