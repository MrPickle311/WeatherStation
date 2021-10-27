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

void Timer::timeout()
{
	if(on_timeout_handler_)
	{
		on_timeout_handler_();
	}
}

}

extern "C" {

__attribute__((interrupt)) void TIM1_UP_IRQHandler(void)
{
	if(TIM1->SR & TIM_SR_UIF)
	{
		TIM1->SR =~TIM_SR_UIF;
		Device::Timer::get(TIM1).timeout();
	}
}


}
