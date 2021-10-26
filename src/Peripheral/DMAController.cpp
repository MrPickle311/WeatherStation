#include "../Peripheral/DMAController.hpp"

namespace Device
{

DMA_ChannelController::DMA_ChannelController(volatile DMA_Channel_TypeDef* channel):
		channel_{channel}
{}

void DMA_ChannelController::setControlRegisterBit(uint8_t bit_mask)
{
	channel_->CCR |= bit_mask;
}

void DMA_ChannelController::enableHalfTransferCompleteInterrupt()
{
	setControlRegisterBit(DMA_CCR_HTIE);
}

void DMA_ChannelController::enableTransferCompleteInterrupt()
{
	setControlRegisterBit(DMA_CCR_TCIE);
}

void DMA_ChannelController::enableTransferErrorInterrupt()
{
	setControlRegisterBit(DMA_CCR_TEIE);
}

void DMA_ChannelController::setDirection(DMADirection direction)
{
	if(direction == DMADirection::PeriphToMemory)
	{
		CLEAR_BIT(channel_->CCR , DMA_CCR_DIR);
		return;
	}

	setControlRegisterBit(DMA_CCR_DIR);
}

void DMA_ChannelController::enableCircularMode()
{
	setControlRegisterBit(DMA_CCR_CIRC);
}

void DMA_ChannelController::enableMemoryIncrement()
{
	setControlRegisterBit(DMA_CCR_MINC);
}

void DMA_ChannelController::enablePeripheralIncrement()
{
	setControlRegisterBit(DMA_CCR_PINC);
}

void DMA_ChannelController::setPeripheralAddress(__IO uint32_t* periph_adr )
{
	channel_->CPAR = reinterpret_cast<uint32_t>(periph_adr);
}


}
