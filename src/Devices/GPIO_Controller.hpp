#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"
#include "DeviceTraits.hpp"

namespace Device
{

enum class GPIO_Port
{
	A = GPIOA_BASE ,
	B = GPIOB_BASE ,
	C = GPIOC_BASE ,
	D = GPIOD_BASE
};

template<auto Port , auto Pin>
class GPIO_Device : public Singleton<GPIO_Device<Port,Pin>>
{
private:
	GPIO_TypeDef* port_ = reinterpret_cast<GPIO_TypeDef*>(Port);
private:
	constexpr __IO uint32_t* getControlRegister()
	{
		if(Pin < 8 )
		{
			return port_->CRL;
		}

		return port_->CRH;
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

};

}
