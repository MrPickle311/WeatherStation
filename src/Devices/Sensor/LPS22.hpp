#pragma once

#include "I2CSensor.hpp"

namespace Device
{

enum class LPS22_OutputDataBitRate : uint8_t
{
    Rate_1Hz = 0b001,
    Rate_10Hz = 0b010,
    Rate_25Hz = 0b011,
    Rate_50Hz = 0b100,
    Rate_75Hz = 0b101
};

template <typename I2CBusType>
class LPS_22 : public I2C_Sensor<I2CBusType>
{
    using typename I2C_Sensor<I2CBusType>::addr_t;
    constexpr static addr_t dev_adr{0x5D << 1};

    constexpr static addr_t TEMP_OUT_L{0x2B};
    constexpr static addr_t TEMP_OUT_H{0x2C};
    constexpr static addr_t CTRL_REG1{0x10};
    constexpr static addr_t CTRL_REG2{0x11};
    constexpr static addr_t STATUS_REG{0x27};

    constexpr static addr_t PRESS_OUT_XL{0x28};
    constexpr static addr_t PRESS_OUT_L{0x29};
    constexpr static addr_t PRESS_OUT_H{0x2A};

public:
    LPS_22(I2CBusType& bus) : I2C_Sensor<I2CBusType>{bus, dev_adr} {}

public:
    void enable(const LPS22_OutputDataBitRate rate)
    {
        this->writeByteToRegister(CTRL_REG1, static_cast<uint8_t>(rate) << 4);
    }
    uint32_t readPressureRaw()
    {
        static uint8_t press_xl = 0;
        static uint8_t press_l = 0;
        static uint8_t press_H = 0;

        this->readByteFromRegister(PRESS_OUT_XL, &press_xl);
        this->readByteFromRegister(PRESS_OUT_L, &press_l);
        this->readByteFromRegister(PRESS_OUT_H, &press_H);

        return (press_H << 16) | (press_l << 8) | press_xl;
    }
};


} // namespace Device
