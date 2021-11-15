#pragma once

#include "../TypeTraits/Singleton.hpp"
#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"
namespace Device
{

class USART_Bus : public Multiton<USART_Bus, USART_TypeDef*>
{
    friend class Multiton<USART_Bus, USART_TypeDef*>;

private:
    volatile USART_TypeDef* usart_;

private:
    void resetControlRegisters();
    void waitForFlagSet(uint32_t flag);

private:
    USART_Bus(volatile USART_TypeDef* usart);

public:
    void enableBus();

    void enableTransmitter();
    void enableReceiver();

    void enableDMAForTransmitter();
    void enableDMAForReceiver();

    void setupBaudRate(uint32_t baud_value, uint32_t peripheral_frequency);

    void sendByte(uint8_t byte);
    uint8_t getByte();
    void sendBytes(uint8_t* bytes, uint16_t count);
    void printText(uint8_t* text);
};


} // namespace Device
