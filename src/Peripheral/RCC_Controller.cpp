#include "../Peripheral/RCC_Controller.hpp"

namespace Device
{


FlashController::FlashController()
{

}

void FlashController::setLatency(FlashLatency latency)
{
	FLASH->ACR |= static_cast<uint8_t>(latency);//s.58
}

void FlashController::enablePrefetchBuffer()
{
	FLASH->ACR |= FLASH_ACR_PRFTBE;//datasheet s.60 & 58 above latency info
}

#define RCC_CFGR_PLLXTPRE_HSI_DIV2 0x0

PLL_Loop::PLL_Loop()
{
	RCC->CFGR |= RCC_CFGR_PLLMULL16_Msk | RCC_CFGR_PLLXTPRE_HSI_DIV2;
}

void PLL_Loop::enable()
{
	RCC->CR |= RCC_CR_PLLON;
}

void PLL_Loop::waitForReady()
{
	while( (RCC->CR & RCC_CR_PLLRDY) == 0)
	{
		asm volatile ("nop");
	}
}

void PLL_Loop::setAsSystemClock()
{
	RCC->CFGR |= RCC_CFGR_SW_PLL;
}

void PLL_Loop::waitUntilSetAsSystemClock()
{
	while( (RCC->CFGR & RCC_CFGR_SWS_PLL) == 0)
	{
		asm volatile ("nop");
	}
}

RCC_Controller::RCC_Controller()
{
	//APB2 & APB1 & AHB PRESCALERS
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_HPRE_DIV1 ;
}

void RCC_Controller::enableHighSpeedClock()
{
	RCC->CR |= RCC_CR_HSION;
}

void RCC_Controller::waitUntilHighSpeedClockReady()
{
	while( (RCC->CR & RCC_CR_HSIRDY) == 0)
	{
		asm volatile ("nop");
	}
}

void RCC_Controller::enableGPIOPort(GPIO_Enable port)
{
	RCC->APB2ENR |= static_cast<uint8_t>(port);
}

void RCC_Controller::enableDMAController()
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
}

void RCC_Controller::enableI2C1Bus()
{
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
}

void RCC_Controller::enableUSART1Bus()
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
}

void RCC_Controller::enableUSART2Bus()
{
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
}

void RCC_Controller::enableTimer1()
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;//oblokuj zegar dla TIM1
}

}
