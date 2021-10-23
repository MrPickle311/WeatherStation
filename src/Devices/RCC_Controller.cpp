#include "RCC_Controller.hpp"

namespace Device
{


FlashController::FlashController()
{

}

void FlashController::setLatency(FlashLatency latency)
{
	FLASH->ACR |= static_cast<uint8_t>(latency);
}

void FlashController::enablePrefetchBuffer()
{
	FLASH->ACR |= FLASH_ACR_PRFTBE;
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

}
