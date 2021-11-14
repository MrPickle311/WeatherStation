#pragma once

#include "../Peripheral/DeviceTraits.hpp"
#include "CircularBuffer.hpp"
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

struct OutputTransactionEntity
{
    uint8_t register_address_;
    uint8_t data_to_write_;
};

struct InputTransactionEntity
{
    uint8_t register_to_read_;
    uint8_t* destination_;
};

template <typename DirectionType, typename SizeType = uint16_t>
class Transaction
{
private:
    uint8_t address_;
    std::vector<DirectionType> entities_;

public:
    using IteratorType = typename std::vector<DirectionType>::iterator;

public:
    Transaction(uint8_t address) : address_{address} {}
    void addEntity(DirectionType&& entity)
    {
        entities_.push_back(entity);
    }
    DirectionType& operator[](uint16_t idx)
    {
        return entities_.at(idx);
    }
    uint8_t deviceAddress() const
    {
        return address_;
    }
    IteratorType begin()
    {
        return entities_.begin();
    }
    IteratorType end()
    {
        return entities_.end();
    }
};

using OutputTransaction = Transaction<OutputTransactionEntity>;
using InputTransaction = Transaction<InputTransactionEntity>;

class I2C_OStream : public Multiton<I2C_OStream, I2C_Bus>
{
    friend class Multiton<I2C_OStream, I2C_Bus>;

private:
    I2C_Bus& bus_base_;

private:
    I2C_OStream(I2C_Bus& bus_base);

private:
    void writeToExternRegister(uint8_t address, uint8_t reg, uint8_t data);

public:
    void processTransaction(OutputTransaction& transaction);
};

class I2C_IStream : public Multiton<I2C_IStream, I2C_Bus>
{
    friend class Multiton<I2C_IStream, I2C_Bus>;

private:
    I2C_Bus& bus_base_;

private:
    void putByteToBuffer(uint8_t* buffer, uint16_t pos);
    void readSingleByte(uint8_t address, uint8_t* buffer);
    void startReading(uint8_t address, uint8_t* buffer, uint16_t size);
    void finishReading(uint8_t* buffer, uint16_t size);
    void readFromExternRegister(uint8_t address,
                                uint8_t reg,
                                uint8_t* target,
                                uint8_t size);
    void readBytes(uint8_t address, uint8_t* buffer, uint16_t size);
    I2C_IStream(I2C_Bus& bus_base);

public:
    void processTransaction(InputTransaction& transaction);
};

class I2C_DeviceQueue : public Multiton<I2C_DeviceQueue, I2C_Bus>
{
    friend class Multiton<I2C_DeviceQueue, I2C_Bus>;

private:
    std::list<OutputTransaction> out_transactions_;
    std::list<InputTransaction> in_transactions_;

    volatile I2C_Bus& i2c_;

private:
    template <typename TransactionType, typename I2C_Stream>
    void tryProcessTransaction(std::list<TransactionType>& transaction_list)
    {
        if (!transaction_list.empty())
        {
            auto&& transaction{transaction_list.front()};
//            I2C_Stream::get(const_cast<I2C_Bus&>(i2c_))
//                .processTransaction(transaction);
            transaction_list.pop_front();
        }
    }

private:
    I2C_DeviceQueue(I2C_Bus& i2c);

public:
    void tryProcessTransactions()
    {
        tryProcessTransaction<OutputTransaction, I2C_OStream>(out_transactions_);

        tryProcessTransaction<InputTransaction, I2C_IStream>(in_transactions_);
    }
    template <typename TransactionType>
    void addTransaction(TransactionType&& transaction)
    {
        if constexpr (std::is_same<TransactionType, OutputTransaction>::value)
        {
            out_transactions_.push_back(std::forward(transaction));
        }

        if constexpr (std::is_same<TransactionType, InputTransaction>::value)
        {
            in_transactions_.push_back(std::forward(transaction));
        }
    }
};


} // namespace Device
