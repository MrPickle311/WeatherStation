#include "../Peripheral/Timer.hpp"

namespace Device
{

Timer::Timer(TIM_TypeDef* timer):
		timer_{timer}
{
}

void Timer::setPrescaler(uint16_t presc)
{
	timer_->PSC = presc - 1;//preskaler
}

void Timer::setTriggerValue(uint16_t trig)
{
	timer_->ARR = trig - 1;//odliczana wartosc
}

void Timer::setClockDivision(TimerClockDivision div)
{
	timer_-> CR1 |= static_cast<uint32_t>(div);
}

void Timer::enableUpInterrupt()
{
	timer_-> DIER |= TIM_DIER_UIE ;
}

void Timer::enable()
{
	timer_->CR1 |= TIM_CR1_CEN;
}

}
