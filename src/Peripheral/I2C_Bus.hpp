#pragma once

#include "../TypeTraits/Singleton.hpp"
#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"

#include <list>
#include <type_traits>
#include <vector>

namespace Peripheral
{

class I2C_Bus_Base : public TypeTraits::Multiton<I2C_Bus_Base, I2C_TypeDef*>
{
    friend class TypeTraits::Multiton<I2C_Bus_Base, I2C_TypeDef*>;

protected:
    volatile I2C_TypeDef* i2c_;

protected:
    void waitForBitSet(uint8_t bit_mask);
    void resetControlRegisters();
    void clearStatusFlags();

    // I have finished here
protected:
    I2C_Bus_Base
};

class I2C_Bus_ConfigController :
    public TypeTraits::Multiton<I2C_Bus_ConfigController, I2C_TypeDef*>,
    public I2C_Bus_Base
{
    friend class TypeTraits::Multiton<I2C_Bus_ConfigController, I2C_TypeDef*>;

private:
    I2C_Bus_ConfigController(volatile I2C_TypeDef* bus);

public:
    void resetBus();
    void setupSlowSpeedMode(uint32_t periph_fraquency);
    void enable();
};

class I2C_Bus_IOController : public I2C_Bus_Base
{
    void start();
    void stop();

    void clearACK();
    void setACK();

    // only primitive operations
    void sendByte(uint8_t data);
    uint8_t readByte();
    void sendAddress(uint8_t address);

    void beginTransmission(uint8_t address, uint8_t reg_adr);
};

} // namespace Peripheral
