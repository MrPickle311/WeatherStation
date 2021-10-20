#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"
#include "../inc/i2c.hpp"
#include  "string.h"
#include "stdio.h"
#include "sensor.hpp"
#include "dma.hpp"
#include <string>
#include <sstream>

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
	USART2->CR3 |= USART_CR3_DMAT; // enable dma mode
	USART2->CR3 |= USART_CR3_DMAR;

//	USART2->CR1 |= USART_CR1_RXNEIE | USART_CR1_TXEIE;
	NVIC_EnableIRQ(USART2_IRQn);

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
	 // tempereature

	dma_print_text(size);
//	printText(txt);

	// humidity
//	size = sprintf(uart_tx_buffer , "Humidity %f % \n\r" , result);
//	dma_print_text(size);
//	printText(txt);

	// pressure
//	size = sprintf(uart_tx_buffer , "Pressure %f hPa \n\r" , result);
//	dma_print_text(size);
//	printText(txt);
}

void timer1Setup()//500ms
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;//oblokuj zegar dla TIM1
	TIM1->PSC = 64000 - 1;//preskaler
	TIM1->ARR = 500 - 1;//odliczana wartosc

	TIM1-> CR1 |= TIM_CLOCKDIVISION_DIV1;//podzielnik zegara 1
	TIM1-> DIER |= TIM_DIER_UIE ;//odpal przerwania

	TIM1->CR1 |= TIM_CR1_CEN;//odpal licznik
	NVIC_EnableIRQ(TIM1_UP_IRQn);////odpal przerwania
}


extern "C" {

void DMA1_Channel7_IRQHandler(void)
{
//	GPIOA->BSRR |= GPIO_BSRR_BS5;
	//transfer complete flag
	if( DMA1->ISR & DMA_ISR_TCIF7 )
	{
		DMA1->IFCR |= DMA_IFCR_CTCIF7;//reset flag
	}
	else if (DMA1->ISR & DMA_ISR_HTIF7)//half transfer cpl
	{
		DMA1->IFCR |= DMA_IFCR_CHTIF7;//reset flag
	}
	else if ( DMA1->ISR & DMA_ISR_TEIF7)//error
	{
		DMA1->IFCR |= DMA_IFCR_CTEIF7;//reset flag
	}
}

std::string uart_tx_buffer;

void loadTemperature()
{
	static uint8_t size = 0;
	static float result = 0;
	static int16_t raw_temp = 0;

	HTS221_Get_Temperature(&raw_temp);
	result = (float)raw_temp / 10.0 - 6.2 ;
	size = sprintf(uart_tx_buffer , "t:%f;" , result);
}

void loadPressure()
{
	static uint8_t size = 0;
	static float result = 0;
	static uint32_t pressure_raw = 0;
	result = readPressureMillibars(pressure_raw);

	readPressureRaw(&pressure_raw);
}

void loadHumidity()
{
	static uint8_t size = 0;
	static float result = 0;
	static uint16_t hum_raw = 0;

	result = (float)hum_raw / 10.0;

	HTS221_Get_Humidity(&hum_raw);
}

void aqcuireData()
{
	static uint8_t size = 0;
	static float result = 0;
}

void sendData(uint8_t size)
{
	dma1_ch7_start(size);
}

__attribute__((interrupt)) void TIM1_UP_IRQHandler(void)
{
	if(TIM1->SR & TIM_SR_UIF)
	{
		TIM1->SR =~TIM_SR_UIF;
	}
}


}

template<typename T>
class bg{
public:
	static void g(){
		GPIOA->BSRR |= GPIO_BSRR_BS5;
	}
};

extern "C"
{

void USART2_IRQHandler(void)
{
	bg<int>::g();
}

}

int main(void)
{
	SystemInit();
	clk_en();
	gpioa_en();
	usart2Setup();

	uart_tx_buffer.resize(40);

	timer1Setup()

	DMA1->IFCR = 0xffff;

	dma1_ch7_init();
//	dma1_ch6_init();

	NVIC_SetPriority(DMA1_Channel7_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);

//	NVIC_SetPriority(DMA1_Channel6_IRQn, 0);
//	NVIC_EnableIRQ(DMA1_Channel6_IRQn);

	NVIC_SetPriority(TIM1_UP_IRQn , 0);

//	dma1_ch6_config((uint32_t)&USART2->DR , (uint32_t)uart_tx_buffer, 3);
	dma1_ch7_config( (uint32_t)&USART2->DR , (uint32_t)uart_tx_buffer);

	__enable_irq();

	setupI2C();

//	resetLPS22();
	setupLPS22();
	setupHTS22();


	getAndShowTemperature();

	while(1)
	{

	}

}

