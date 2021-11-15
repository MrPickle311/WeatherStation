#pragma once

#include "../TypeTraits/Singleton.hpp"
#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"

#include <list>
#include <type_traits>
#include <vector>

namespace Device
{

class I2C_Bus : public Multiton<I2C_Bus, I2C_TypeDef*>
{
    friend class Multiton<I2C_Bus, I2C_TypeDef*>;

private:
    volatile I2C_TypeDef* i2c_;

private:
    void waitForBitSet(uint8_t bit_mask);
    void resetControlRegisters();
    void clearStatusFlags();


private:
    I2C_Bus(volatile I2C_TypeDef* bus);

public:
    void resetBus();
    void setupSlowSpeedMode(uint32_t periph_fraquency);
    void enable();

    void start();
    void stop();

    void clearACK();
    void setACK();

    void sendByte(uint8_t data);
    uint8_t readByte();
    void sendAddress(uint8_t address);

    void beginTransmission(uint8_t address, uint8_t reg_adr);
};


} // namespace Device
