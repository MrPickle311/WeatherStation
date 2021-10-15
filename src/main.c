/**
  ******************************************************************************
  * @file    main.c
  * @author  Damian Wojcik
  * @version V1.0
  * @date    13.03.2021
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"

#define RCC_CFGR_PLLXTPRE_HSI_DIV2 0x0

int main(void)
{
	RCC->CR |= RCC_CR_HSION;

	while( (RCC->CR & RCC_CR_HSIRDY) == 0)
	{
		asm volatile ("nop");
	}

	//power interface clock
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	//prefetch buffer for 64 MHz
	FLASH->ACR |= FLASH_ACR_LATENCY_1 | FLASH_ACR_PRFTBE;

	//APB2 & APB1 & AHB PRESCALERS
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_HPRE_DIV1 ;

	//Configure the MAIN PLL
	RCC->CFGR |= RCC_CFGR_PLLMULL16_Msk | RCC_CFGR_PLLXTPRE_HSI_DIV2;

	//run pll loop
	RCC->CR |= RCC_CR_PLLON;

	while( (RCC->CR & RCC_CR_PLLRDY) == 0)
	{
		asm volatile ("nop");
	}

	//select clock source and wait for response

	RCC->CFGR |= RCC_CFGR_SW_PLL;

	while( (RCC->CR & RCC_CFGR_SWS_PLL) == 0)
	{
		asm volatile ("nop");
	}
}
