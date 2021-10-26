#pragma once

#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"

#include "../Peripheral/DeviceTraits.hpp"

namespace Device
{

enum class GPIO_Port
{
	A = GPIOA_BASE ,
	B = GPIOB_BASE ,
	C = GPIOC_BASE ,
	D = GPIOD_BASE
};

enum class PinFrequency : uint8_t
{
	F_2MHz = 0b10 ,
	F_10MHz = 0b01,
	F_50MHz = 0b11
};

template<auto Port , auto Pin>
class GPIO_Device : public Singleton<GPIO_Device<Port,Pin>>
{
private:
	GPIO_TypeDef* port_ = reinterpret_cast<GPIO_TypeDef*>(Port);
	static constexpr uint8_t bits_in_pin_subregister {4};
private:
	constexpr __IO uint32_t* getControlRegister()
	{
		if(Pin < 8 )
		{
			return &port_->CRL;
		}

		return &port_->CRH;
	}
	uint8_t getShiftPosition()
	{
		if( Pin < 8 )
		{
			return Pin * bits_in_pin_subregister;
		}

		return ( Pin - 8 ) * bits_in_pin_subregister;
	}
	void resetState()
	{
		*getControlRegister() &= ~( 0b1111 << getShiftPosition() );
	}
	void setPushPull(PinFrequency freq , uint8_t cnf_mask)
	{
		resetState();
		*getControlRegister() |= ( cnf_mask | static_cast<uint32_t>(freq) ) << getShiftPosition();
	}
	void setInput(uint8_t cnf_mask)
	{
		resetState();
		*getControlRegister() |= ( cnf_mask << getShiftPosition() );
	}
public:
	void setHigh()
	{
		port_->BSRR |= 0x1 << Pin;
	}
	void setLow()
	{
		port_->BRR |= 0x1 << Pin;
	}
	void setOutputPushPull(PinFrequency freq)
	{
		setPushPull(freq, 0b0000);
	}
	void setAlternatePushPull(PinFrequency freq)
	{
		setPushPull(freq, 0b1000);
	}
	void setAlternateOpenDrain(PinFrequency freq)
	{
		setPushPull(freq, 0b1100);
	}
	void setInputPullUpPullDown()
	{
		setInput(0b1000);
	}
	void setAnalog()
	{
		resetState();
	}
};

}