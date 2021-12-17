#pragma once

#include "../TypeTraits/Singleton.hpp"
#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"
namespace Peripheral
{

class USART_Bus_Base :
    public TypeTraits::Multiton<USART_Bus_Base, USART_TypeDef*>
{
    friend class Multiton<USART_Bus_Base, USART_TypeDef*>;
    friend class USART_Bus_ConfigController;
    friend class USART_Bus_IOController;

private:
    volatile USART_TypeDef* usart_;

private:
    void resetControlRegisters();
    void waitForFlagSet(uint32_t flag);

private:
    USART_Bus_Base(USART_TypeDef* usart);
};


class USART_Bus_ConfigController :
    public TypeTraits::Multiton<USART_Bus_ConfigController, USART_TypeDef*>
{
    friend class Multiton<USART_Bus_ConfigController, USART_TypeDef*>;

private:
    USART_Bus_Base& base_;

private:
    USART_Bus_ConfigController(USART_TypeDef* usart);

public:
    void enableBus();

    void enableTransmitter();
    void enableReceiver();

    void enableDMAForTransmitter();
    void enableDMAForReceiver();

    void setupBaudRate(uint32_t baud_value, uint32_t peripheral_frequency);
};

class USART_Bus_IOController :
    public TypeTraits::Multiton<USART_Bus_IOController, USART_TypeDef*>
{

    friend class Multiton<USART_Bus_ConfigController, USART_TypeDef*>;

private:
    USART_Bus_Base& base_;

private:
    USART_Bus_IOController(USART_TypeDef* usart);

public:
    void sendByte(uint8_t byte);
    uint8_t getByte();
    //    void sendBytes(uint8_t* bytes, uint16_t count);
    //    void printText(uint8_t* text);
};

} // namespace Peripheral
