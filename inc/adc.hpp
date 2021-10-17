#pragma once

#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"

/************** STEPS TO FOLLOW *****************

1. Enable ADC and GPIO clock
2. Set the prescalar in the Common Control Register (CCR)
3. Set the Scan Mode and Resolution in the Control Register 1 (CR1)
4. Set the Continuous Conversion, EOC, and Data Alignment in Control Reg 2 (CR2)
5. Set the Sampling Time for the channels in ADC_SMPRx
6. Set the Regular channel sequence length in ADC_SQR1
7. Set the Respective GPIO PINs in the Analog Mode

************************************************/

void enable_adc_clocks()
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->CFGR | RCC_CFGR_ADCPRE;// 64 / 8 = 8
}

void setup_adc_gpio_settings()
{
	//PA0 analog
	GPIOA->CRL &= ~(0b1111) ;
}

void setup_sequence()
{
	ADC1->SQR1 |= (0 << 0); // SEQ1 dla kanału 0 , najpierw konwersja na kanale 0
}

void setup_adc_dma()
{
	ADC1->CR2 |= ADC_CR2_DMA;
}

void setup_adc_settings()
{
	ADC1->CR1 = ADC_CR1_SCAN;

	//set the continuous conversion

	ADC1->CR2 |= ADC_CR2_CONT;

	ADC1->CR1 |= ADC_CR1_EOCIE;//to chyba jest odpalenie przerwania albo innego eventu , a EOC jest na stałe TURNON

	ADC1->SMPR2 = ADC_SAMPLETIME_1CYCLE5_SMPR2ALLCHANNELS ;

	ADC1->SQR1 |= ADC_SQR1_L_1;//3 konwersje na przedział czasu

	setup_adc_gpio_settings();
	setup_adc_dma();
	setup_sequence();
}

void enable_adc()
{
	ADC1->CR2 |= ADC_CR2_ADON;

	uint32_t delay = 100000;//kod do zmiany
	while (delay--);
	{
		asm volatile ("nop");
	}
}

void adc_start()
{
	CLEAR_REG(ADC1->SR);

	ADC1->CR2 |= ADC_CR2_EXTTRIG;  // Conversion on external event enabled

	ADC1->CR2 |= ADC_CR2_SWSTART; // start regular conversion


}

