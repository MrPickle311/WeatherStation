#include "../Peripheral/I2C_Bus.hpp"

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


I2C_Bus::I2C_Bus(volatile I2C_TypeDef* bus) : i2c_{bus}
{
    resetControlRegisters();
}

void I2C_Bus_ConfigController::resetBus()
{
    i2c_->CR1 |= I2C_CR1_SWRST;
    i2c_->CR1 &= ~I2C_CR1_SWRST;
}

void I2C_Bus_ConfigController::setupSlowSpeedMode(uint32_t periph_fraquency)
{
    auto tpclk{1'000'000'000 / periph_fraquency}; // clock duration in ns

    auto ccr{static_cast<uint16_t>(std::round(5000 / tpclk))};

    auto mhz_fraquency{
        static_cast<uint8_t>(std::round(periph_fraquency / 1'000'000))};

    auto trise{static_cast<uint8_t>(std::round(1'000 / tpclk) + 1)};

    i2c_->CR2 |= mhz_fraquency << 0;

    // CCR Controls the SCL clock in master mode.
    // datasheet s.782
    i2c_->CCR = ccr << 0;

    // rising edge duration , max is 1000 ns ,
    // for example if SYS_CLK = 64 MHz -> trise = 1000s * 10^-9 / 64 * 10^6 1/s
    // + 1 datasheet s.783
    i2c_->TRISE = trise << 0;
}

void I2C_Bus_ConfigController::enable()
{
    i2c_->CR1 |= I2C_CR1_PE;
}

void I2C_Bus_IOController::start()
{
    setACK();
    //    i2c_->CR1 |=
    //        I2C_CR1_ACK; // here bcs when PE cleared , bit number 10 is
    //        cleared also
    i2c_->CR1 |= I2C_CR1_START; // generate start

    waitForBitSet(I2C_SR1_SB); // wait for generate START signal
}

void I2C_Bus_IOController::stop()
{
    i2c_->CR1 |= I2C_CR1_STOP;
}

void I2C_Bus_IOController::clearACK()
{
    i2c_->CR1 &= ~I2C_CR1_ACK;
}

void I2C_Bus_IOController::setACK()
{
    i2c_->CR1 |= I2C_CR1_ACK;
}

uint8_t I2C_Bus_IOController::readByte()
{
    waitForBitSet(I2C_SR1_RXNE); // wait for "Data register not empty"

    return i2c_->DR;
}


void I2C_Bus_IOController::beginTransmission(uint8_t address, uint8_t reg_adr)
{
    start();
    sendAddress(address);
    sendByte(reg_adr);
}
void I2C_Bus_IOController::sendByte(uint8_t data)
{
    waitForBitSet(I2C_SR1_TXE); // wait for TXE bit to set

    i2c_->DR = data;

    // BTF = ACK
    waitForBitSet(I2C_SR1_BTF); // wait for Byte Transfer Finished bit to set
}

void I2C_Bus_IOController::sendAddress(uint8_t address)
{
    i2c_->DR = address; //  send the address

    waitForBitSet(I2C_SR1_ADDR); // wait for ADDR bit to set

    clearStatusFlags(); // read SR1 and SR2 to clear the ADDR bit
}


} // namespace Peripheral
