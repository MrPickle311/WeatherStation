#pragma once

#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"

#include "../Peripheral/DeviceTraits.hpp"


namespace Device
{

enum class FlashLatency : uint8_t
{
	ZeroWaitState = 0x0,// 0 < SYSCLK 24 MHz
	OneWaitState = FLASH_ACR_LATENCY_0 , //24 MHz < SYSCLK  48 MHz
	TwoWaitStates = FLASH_ACR_LATENCY_1 // 48 MHz < SYSCLK  72 MHz
};

class FlashController : public Singleton<FlashController>
{
	friend class Singleton<FlashController>;
private:
	FlashController();
public:
	void setLatency(FlashLatency latency);
	void enablePrefetchBuffer();
};

class PLL_Loop : public Singleton<PLL_Loop>
{
	friend class Singleton<PLL_Loop>;
private:
	PLL_Loop();
public:
	void enable();
	void waitForReady();
	void setAsSystemClock();
	void waitUntilSetAsSystemClock();
};

enum class GPIO_Enable
{
	A = RCC_APB2ENR_IOPAEN ,
	B = RCC_APB2ENR_IOPBEN ,
	C = RCC_APB2ENR_IOPCEN ,
	D = RCC_APB2ENR_IOPDEN
};

class RCC_Controller : public Singleton<RCC_Controller>
{
	friend class Singleton<RCC_Controller>;
private:
	RCC_Controller();
public:
	void enableHighSpeedClock();
	void waitUntilHighSpeedClockReady();
	void enableGPIOPort(GPIO_Enable port);
};

}

