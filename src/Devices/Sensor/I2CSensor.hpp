#pragma once

#include <cstdint>


namespace Device
{

template <typename I2CBusType>
class I2C_Sensor
{
public:
    using addr_t = volatile uint8_t;

protected:
    I2CBusType& i2c_bus_;

private:
    uint8_t dev_adr_;

protected:
    void readByteFromRegister(uint8_t reg, uint8_t* target)
    {
        i2c_bus_.readFromExternRegister(dev_adr_, reg, target, 1);
    }

    void writeByteToRegister(uint8_t reg, uint8_t what)
    {
        i2c_bus_.writeToExternRegister(dev_adr_, reg, what);
    }
    uint16_t concatWord(uint8_t high_byte, uint8_t low_byte)
    {
        return (static_cast<uint16_t>(high_byte) << 8) |
               static_cast<uint16_t>(low_byte);
    }

protected:
    I2C_Sensor(I2CBusType& bus, uint8_t device_address) :
        i2c_bus_{bus}, dev_adr_{device_address}
    {}
};

} // namespace Device
