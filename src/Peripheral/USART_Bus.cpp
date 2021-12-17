#include "../Peripheral/USART_Bus.hpp"

#include <cmath>

namespace Peripheral
{
USART_Bus_Base::USART_Bus_Base(USART_TypeDef* usart) : usart_{usart} {}


void USART_Bus_Base::resetControlRegisters()
{
    CLEAR_REG(usart_->CR1);
    CLEAR_REG(usart_->CR2);
    CLEAR_REG(usart_->CR3);
}

void USART_Bus_Base::waitForFlagSet(uint32_t flag)
{
    while (!(usart_->SR & flag)) // wait until buffer ready
    {
        asm volatile("nop");
    }
}

USART_Bus_ConfigController::USART_Bus_ConfigController(USART_TypeDef* usart) :
    base_{USART_Bus_Base::get(usart)}
{
    base_.resetControlRegisters();
}


void USART_Bus_ConfigController::enableBus()
{
    base_.usart_->CR1 |= USART_CR1_UE;
}

void USART_Bus_ConfigController::enableTransmitter()
{
    base_.usart_->CR1 |= USART_CR1_TE; // enable rx and tx
}

void USART_Bus_ConfigController::enableReceiver()
{
    base_.usart_->CR1 |= USART_CR1_RE; // enable rx and tx
}

void USART_Bus_ConfigController::enableDMAForTransmitter()
{
    base_.usart_->CR3 |= USART_CR3_DMAT; // enable dma mode
}

void USART_Bus_ConfigController::enableDMAForReceiver()
{
    base_.usart_->CR3 |= USART_CR3_DMAR;
}

void USART_Bus_ConfigController::setupBaudRate(uint32_t baud_value,
                                               uint32_t peripheral_frequency)
{
    double temp_result{peripheral_frequency / (baud_value * 16)};

    uint16_t mantisa{static_cast<uint16_t>(std::trunc(temp_result))};

    uint8_t fraction{
        static_cast<uint8_t>(std::round((temp_result - mantisa) * 16))};

    base_.usart_->BRR = (mantisa << 4) | (fraction << 0);
}


void USART_Bus_IOController::sendByte(uint8_t byte)
{
    base_.waitForFlagSet(USART_SR_TXE);

    base_.usart_->DR = byte;
}

uint8_t USART_Bus_IOController::getByte()
{
    base_.waitForFlagSet(USART_SR_RXNE);

    return base_.usart_->DR;
}


USART_Bus_IOController::USART_Bus_IOController(USART_TypeDef* usart) :
    base_{USART_Bus_Base::get(usart)}
{}

// void USART_Bus::printText(uint8_t* txt)
//{
//    while (*txt)
//    {
//        sendByte(*txt);
//        ++txt;
//    }
//    sendByte('\r');
//    sendByte('\n');
//}

} // namespace Peripheral
