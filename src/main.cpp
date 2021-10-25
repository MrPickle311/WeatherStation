#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"
#include "../inc/i2c.hpp"
#include  "string.h"
#include "stdio.h"
#include "sensor.hpp"
#include "dma.hpp"
#include <string>
#include <sstream>
#include <map>
#include <functional>
#include "../src/Devices/RCC_Controller.hpp"
#include "../src/Devices/USART_Bus.hpp"
#include "../src/Devices/Timer.hpp"

void clk_en()
{
	auto&& rcc_controller {Device::RCC_Controller::getInstance()};

	rcc_controller.enableHighSpeedClock();
	rcc_controller.waitUntilHighSpeedClockReady();

	//power interface clock
	//the following setting enables PWR_CR and PWR_CSR registers
	//i dont need it now , but i will make voltage control later
	//RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	//fit flash mem access to SYSCLK = 64 MHz
	//prefetch -> explanation page 58 in the user manual
	auto&& flash_controler {Device::FlashController::getInstance()};
	flash_controler.enablePrefetchBuffer();
	flash_controler.setLatency(Device::FlashLatency::TwoWaitStates);

	auto&& pll_loop {Device::PLL_Loop::getInstance()};

	pll_loop.enable();
	pll_loop.waitForReady();
	pll_loop.setAsSystemClock();
	pll_loop.waitUntilSetAsSystemClock();
}

void gpioa_en()
{
	using namespace Device;
	//enable gpioa clock
	RCC_Controller::getInstance().enableGPIOPort(GPIO_Enable::A);

	//pa5 push pull 10 mhz
	GPIO_Device<GPIO_Port::A,5>::getInstance().setOutputPushPull(PinFrequency::F_10MHz);
}

void turn_on_led()
{
	Device::GPIO_Device<Device::GPIO_Port::A,5>::getInstance().setHigh();
}

void usart2_gpio_tx_en()
{
	using namespace Device;
	//( usart 2 works with 32 mhz )
	GPIO_Device<GPIO_Port::A,2>::getInstance().setAlternatePushPull(PinFrequency::F_50MHz);
}

void usart1_gpio_tx_en()
{
	using namespace Device;
	GPIO_Device<GPIO_Port::A,9>::getInstance().setAlternatePushPull(PinFrequency::F_50MHz);

}

void usart2_gpio_rx_en()
{
	using namespace Device;

	auto&& device {GPIO_Device<GPIO_Port::A,3>::getInstance()};

	device.setInputPullUpPullDown();
	device.setHigh();
}

void usart1_gpio_rx_en()
{
	using namespace Device;

	auto&& device {GPIO_Device<GPIO_Port::A,10>::getInstance()};

	device.setInputPullUpPullDown();
	device.setHigh();
}

void usart2_gpioa_en()
{
	usart2_gpio_tx_en();
	usart2_gpio_rx_en();
}

void usart1_gpioa_en()
{
	usart1_gpio_tx_en();
	usart1_gpio_rx_en();
}

void usart2_clk_en()
{
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
}

void usart1_clk_en()
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
}

void usart2_config()
{
	using namespace Device;
	auto&& usart {USART_Bus::get(USART2)};
	usart.enableBus();
	usart.enableDMAForTransmitter();
	usart.setupBaudRate(115200, 32'000'000);
	usart.enableTransmitter();
}

void usart1_config()
{
	using namespace Device;
	auto&& usart {USART_Bus::get(USART1)};
	usart.enableBus();
	usart.enableDMAForTransmitter();
	usart.setupBaudRate(115200, 64'000'000);
	usart.enableTransmitter();
}

void usart2Setup (void)
{
	usart2_gpioa_en();
	usart2_clk_en();
	usart2_config();
}

void usart1Setup (void)
{
	usart1_gpioa_en();
	usart1_clk_en();
	usart1_config();
}

void timer1Setup()//500ms
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;//oblokuj zegar dla TIM1

	auto&& timer{Device::Timer::get(TIM1)};

	timer.setPrescaler(64'000);
	timer.setTriggerValue(500);

	timer.setClockDivision(Device::TimerClockDivision::Div1);
	timer.enableUpInterrupt();
	timer.enable();

	NVIC_EnableIRQ(TIM1_UP_IRQn);////odpal przerwania
}


std::string uart_tx_buffer;
extern "C" {

void DMA1_Channel7_IRQHandler(void)
{
	if( DMA1->ISR & DMA_ISR_TCIF7 ) //transfer complete flag
	{
		DMA1_Channel7->CCR &= ~DMA_CCR_EN;
		DMA1->IFCR |= DMA_IFCR_CTCIF7;//reset flag
		uart_tx_buffer.clear();
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

void DMA1_Channel4_IRQHandler(void)
{
	if( DMA1->ISR & DMA_ISR_TCIF4 )
	{
		DMA1_Channel4->CCR &= ~DMA_CCR_EN;
		DMA1->IFCR |= DMA_IFCR_CTCIF4;//reset flag
		uart_tx_buffer.clear();
	}
	else if (DMA1->ISR & DMA_ISR_HTIF4)//half transfer cpl
	{
		DMA1->IFCR |= DMA_IFCR_CHTIF4;//reset flag
	}
	else if ( DMA1->ISR & DMA_ISR_TEIF4)//error
	{
		DMA1->IFCR |= DMA_IFCR_CTEIF4;//reset flag
	}
}

}

void loadTemperature()
{
	static uint8_t size = 0;
	static float result = 0;
	static int16_t raw_temp = 0;

	HTS221_Get_Temperature(&raw_temp);
	result = (float)raw_temp / 10.0;

	uart_tx_buffer.append("t:");
	uart_tx_buffer.append(std::to_string(result));
	uart_tx_buffer.append(";");
}

void loadPressure()
{
	static uint8_t size = 0;
	static float result = 0;
	static uint32_t pressure_raw = 0;

	readPressureRaw(&pressure_raw);

	result = readPressureMillibars(pressure_raw);

	uart_tx_buffer.append("p:");
	uart_tx_buffer.append(std::to_string(result));
	uart_tx_buffer.append(";");

}

void loadHumidity()
{
	static uint8_t size = 0;
	static float result = 0;
	static uint16_t hum_raw = 0;

	HTS221_Get_Humidity(&hum_raw);

	result = (float)hum_raw / 10.0;

	uart_tx_buffer.append("h:");
	uart_tx_buffer.append(std::to_string(result));
	uart_tx_buffer.append(";");
}

void sendData(std::string_view str_to_send)
{
	static auto dma{Device::DMA_ChannelController::get(DMA1_Channel4)};

	dma.setMemoryAddress(&str_to_send[0]);
	dma.setPeripheralAddress(&USART1->DR);

	dma1_ch4_start(str_to_send.size());
}

std::map< int , std::function<void()> > callbacks;
uint8_t nmbr = 0 ;

extern "C" {

__attribute__((interrupt)) void TIM1_UP_IRQHandler(void)
{
	if(TIM1->SR & TIM_SR_UIF)
	{
		TIM1->SR =~TIM_SR_UIF;

		callbacks.at(nmbr)();
		nmbr = ( nmbr + 1 ) % 3;

		sendData(uart_tx_buffer);
		turn_on_led();

	}
}

}

int main(void)
{
	SystemInit();
	clk_en();
	gpioa_en();
	usart2Setup();
	usart1Setup();

	callbacks[0] =  loadTemperature;
	callbacks[1] =  loadPressure;
	callbacks[2] =  loadHumidity;

	uart_tx_buffer.reserve(40);


	DMA1->IFCR = 0xffff;

//	dma1_ch7_init();
	dma1_ch4_init();

//	NVIC_SetPriority(DMA1_Channel7_IRQn, 0);
//	NVIC_EnableIRQ(DMA1_Channel7_IRQn);

	NVIC_SetPriority(DMA1_Channel4_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);

	NVIC_SetPriority(TIM1_UP_IRQn , 0);
	setupI2C();

	setupLPS22();
	setupHTS22();


	timer1Setup();
//	dma1_ch6_config((uint32_t)&USART2->DR , (uint32_t)uart_tx_buffer, 3);

	__enable_irq();

	while(1)
	{

	}

}

