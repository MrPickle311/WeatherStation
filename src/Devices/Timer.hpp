#pragma once

#include "DeviceTraits.hpp"
#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"

namespace Device
{

enum TimerClockDivision : uint32_t
{
	Div1 = TIM_CLOCKDIVISION_DIV1 ,
	Div2 = TIM_CLOCKDIVISION_DIV2 ,
	Div4 = TIM_CLOCKDIVISION_DIV4
};

class Timer : public Multiton<Timer, TIM_TypeDef*>
{
	friend class Multiton<Timer , TIM_TypeDef*>;
private:
	TIM_TypeDef* timer_;
private:
	Timer(TIM_TypeDef*);
public:
	void setPrescaler(uint16_t presc);
	void setTriggerValue(uint16_t trig);

	void setClockDivision(TimerClockDivision div);
	void enableUpInterrupt();

	void enable();
};

}
