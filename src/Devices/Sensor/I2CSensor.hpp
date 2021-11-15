#pragma once

#include <cstdint>
#include <map>

namespace Device
{

template <typename I2CDeviceQueueType,
          typename OutputTransactionType,
          typename InputTransactionType>
class I2C_Sensor
{
public:
    using addr_t = volatile uint8_t;
    using KeyType = std::string;

protected:
    I2CDeviceQueueType& i2c_bus_;

    // replace these maps with :
    // https://xuhuisun.com/post/c++-weekly-2-constexpr-map/
    std::map<KeyType, OutputTransactionType> out_transactions_;
    std::map<KeyType, InputTransactionType> in_transactions_;

private:
    uint8_t dev_adr_;

protected:
    //    void readByteFromRegister(uint8_t reg, uint8_t* target)
    //    {
    //        i2c_bus_.readFromExternRegister(dev_adr_, reg, target, 1);
    //    }
    //
    //    void writeByteToRegister(uint8_t reg, uint8_t what)
    //    {
    //        i2c_bus_.writeToExternRegister(dev_adr_, reg, what);
    //    }
    uint16_t concatWord(uint8_t high_byte, uint8_t low_byte)
    {
        return (static_cast<uint16_t>(high_byte) << 8) |
               static_cast<uint16_t>(low_byte);
    }

protected:
    I2C_Sensor(I2CDeviceQueueType& bus, uint8_t device_address) :
        i2c_bus_{bus}, dev_adr_{device_address}
    {}
};

} // namespace Device
