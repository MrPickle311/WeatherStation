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
#include "../inc/i2c.hpp"
#include  "string.h"
#include "stdio.h"
#include "sensor.hpp"

#define RCC_CFGR_PLLXTPRE_HSI_DIV2 0x0

void clk_en()
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

		while( (RCC->CFGR & RCC_CFGR_SWS_PLL) == 0)
		{
			asm volatile ("nop");
		}
}

void gpioa_en()
{
	//enable gpioa clock
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	//pa5 push pull 10 mhz
	GPIOA->CRL = GPIO_CRL_MODE5_0 ;
}

void turn_on_led()
{
	GPIOA->BSRR |= GPIO_BSRR_BS5;
}

void usart2_gpio_tx_en()
{
	//pa2 alternate function , push pull , up to 50 MHz ( usart 2 works with 32 mhz )
	GPIOA->CRL   |= GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1;
}

void usart2_gpio_rx_en()
{
	GPIOA->CRL &= ~( GPIO_CRL_MODE3_0 | GPIO_CRL_MODE3_1 );   // Intput Mode For PA3

	GPIOA->CRL |= GPIO_CRL_CNF3_1 ;  // Input Pull Up/ Down For PA3

	GPIOA->ODR |= GPIO_ODR_ODR3;  // Pull Up for PA3
}

void usart2_gpioa_en()
{
	usart2_gpio_tx_en();
	usart2_gpio_rx_en();
}

void usart2_clk_en()
{
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
}

void usart2_config()
{
	CLEAR_REG(USART2->CR1);

	USART2->CR1 |= USART_CR1_UE;//enable usart2

	USART2->BRR = (6 << 0) | (17 << 4);//mantisa = 17 , fraction = 6

	USART2->CR1 |=  USART_CR1_RE  | USART_CR1_TE ;  // enable rx and tx
}

void usart2Setup (void)
{
	usart2_gpioa_en();
	usart2_clk_en();
	usart2_config();
}

uint8_t UART2_GetChar (void)
{
	static uint8_t temp;

	while (! ( USART2->SR & USART_SR_RXNE ) )  // Wait for RXNE to SET
	{
		asm volatile ("nop");
	}

	temp = USART2->DR;  // read the data.
	return temp;
}

void usartSendByte (uint8_t byte)
{
   while (! ( USART2->SR & USART_SR_TXE ) ) // wait until buffer ready
   {
	   asm volatile ("nop");
   }

   USART2->DR = byte;//push byte
}

void printText(volatile char* txt)//txt musi byc zakonczony '\0'
{
	while(*txt)
	{
		usartSendByte(*txt);
		++txt;
	}
	usartSendByte('\r');
	usartSendByte('\n');
}

void getAndShowTemperature()
{
	char txt[40];

	float result = 0;

	int16_t raw_res = 0;
	uint16_t hum_raw = 0;
	uint32_t pressure_raw = 0;

	HTS221_Get_Temperature(&raw_res);
	HTS221_Get_Humidity(&hum_raw);
	readPressureRaw(&pressure_raw);

	result = (float)raw_res / 10.0 - 6.2 ; // tempereature
	sprintf(txt , "Temperature %f C \n\r" , result);
	printText(txt);

	result = (float)hum_raw / 10.0;// humidity
	sprintf(txt , "Humidity %f % \n\r" , result);
	printText(txt);

	result = readPressureMillibars(pressure_raw);// pressure
	sprintf(txt , "Pressure %f hPa \n\r" , result);
	printText(txt);
}

int main(void)
{
	clk_en();
	gpioa_en();
	turn_on_led();
	usart2Setup();
	setupI2C();

//	resetLPS22();
	setupLPS22();
	setupHTS22();

	getAndShowTemperature();

	while(1)
	{

	}

}

