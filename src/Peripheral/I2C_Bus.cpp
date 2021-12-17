#include "I2C_Bus.hpp"

#include <cmath>

namespace Peripheral
{

void I2C_Bus_Base::resetControlRegisters()
{
    i2c_->CR1 = 0x0;
    i2c_->CR2 = 0x0;
}

void I2C_Bus_Base::waitForBitSet(uint8_t bit_mask)
{
    while (!(i2c_->SR1 & bit_mask))
    {
        asm volatile("nop");
    }
}

void I2C_Bus_Base::clearStatusFlags()
{
    [[maybe_unused]] volatile uint8_t temp = i2c_->SR1 | I2C1->SR2;
}


I2C_Bus_Base::I2C_Bus_Base(I2C_TypeDef* bus) : i2c_{bus}
{
    resetControlRegisters();
}

void I2C_Bus_ConfigController::resetBus()
{
    base_.i2c_->CR1 |= I2C_CR1_SWRST;
    base_.i2c_->CR1 &= ~I2C_CR1_SWRST;
}

void I2C_Bus_ConfigController::setupSlowSpeedMode(uint32_t periph_fraquency)
{
    auto tpclk{1'000'000'000 / periph_fraquency}; // clock duration in ns

    auto ccr{static_cast<uint16_t>(std::round(5000 / tpclk))};

    auto mhz_fraquency{
        static_cast<uint8_t>(std::round(periph_fraquency / 1'000'000))};

    auto trise{static_cast<uint8_t>(std::round(1'000 / tpclk) + 1)};

    base_.i2c_->CR2 |= mhz_fraquency << 0;

    // CCR Controls the SCL clock in master mode.
    base_.i2c_->CCR = ccr << 0;

    // rising edge duration , max is 1000 ns ,
    // for example if SYS_CLK = 64 MHz -> trise = 1000s * 10^-9 / 64 * 10^6 1/s
    base_.i2c_->TRISE = trise << 0;
}

I2C_Bus_ConfigController::I2C_Bus_ConfigController(I2C_TypeDef* bus) :
    base_{I2C_Bus_Base::get(bus)}
{}

void I2C_Bus_ConfigController::enable()
{
    base_.i2c_->CR1 |= I2C_CR1_PE;
}

void I2C_Bus_IOController::start()
{
    setACK();
    //    i2c_->CR1 |=
    //        I2C_CR1_ACK; // here bcs when PE cleared , bit number 10 is
    //        cleared also
    base_.i2c_->CR1 |= I2C_CR1_START; // generate start

    base_.waitForBitSet(I2C_SR1_SB); // wait for generate START signal
}

void I2C_Bus_IOController::stop()
{
    base_.i2c_->CR1 |= I2C_CR1_STOP;
}

void I2C_Bus_IOController::clearACK()
{
    base_.i2c_->CR1 &= ~I2C_CR1_ACK;
}

void I2C_Bus_IOController::setACK()
{
    base_.i2c_->CR1 |= I2C_CR1_ACK;
}

uint8_t I2C_Bus_IOController::readByte()
{
    base_.waitForBitSet(I2C_SR1_RXNE); // wait for "Data register not empty"

    return base_.i2c_->DR;
}


void I2C_Bus_IOController::beginTransmission(uint8_t address, uint8_t reg_adr)
{
    start();
    sendAddress(address);
    sendByte(reg_adr);
}
void I2C_Bus_IOController::sendByte(uint8_t data)
{
    base_.waitForBitSet(I2C_SR1_TXE); // wait for TXE bit to set

    base_.i2c_->DR = data;

    // BTF = ACK
    base_.waitForBitSet(
        I2C_SR1_BTF); // wait for Byte Transfer Finished bit to set
}

void I2C_Bus_IOController::sendAddress(uint8_t address)
{
    base_.i2c_->DR = address; //  send the address

    base_.waitForBitSet(I2C_SR1_ADDR); // wait for ADDR bit to set

    base_.clearStatusFlags(); // read SR1 and SR2 to clear the ADDR bit
}

I2C_Bus_IOController::I2C_Bus_IOController(I2C_TypeDef* bus) :
    base_{I2C_Bus_Base::get(bus)}
{}
} // namespace Peripheral
