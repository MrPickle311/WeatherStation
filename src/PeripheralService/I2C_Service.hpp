#pragma once

#include <cstdint>
#include <list>
#include <type_traits>
#include <vector>

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

template <typename I2CBusType>
class I2C_OStream : public Multiton<I2C_OStream, I2CBusType>
{
    friend class Multiton<I2C_OStream, I2CBusType>;

private:
    I2CBusType& bus_base_;

private:
    I2C_OStream(I2CBusType& bus_base);

private:
    void writeToExternRegister(uint8_t address, uint8_t reg, uint8_t data);

public:
    void processTransaction(OutputTransaction& transaction);
};

template <typename I2CBusType>
class I2C_IStream : public Multiton<I2C_IStream, I2CBusType>
{
    friend class Multiton<I2C_IStream, I2CBusType>;

private:
    I2CBusType& bus_base_;

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
    I2C_IStream(I2CBusType& bus_base);

public:
    void processTransaction(InputTransaction& transaction);
};

template <typename I2CBusType>
class I2C_DeviceQueue : public Multiton<I2C_DeviceQueue, I2CBusType>
{
    friend class Multiton<I2C_DeviceQueue, I2CBusType>;

private:
    std::list<OutputTransaction> out_transactions_;
    std::list<InputTransaction> in_transactions_;

    I2CBusType& i2c_;

private:
    template <typename TransactionType, typename I2C_Stream>
    void tryProcessTransaction(std::list<TransactionType>& transaction_list)
    {
        if (!transaction_list.empty())
        {
            auto&& transaction{transaction_list.front()};
            //            I2C_Stream::get(const_cast<I2CBusType&>(i2c_))
            //                .processTransaction(transaction);
            transaction_list.pop_front();
        }
    }

private:
    I2C_DeviceQueue(I2CBusType& i2c);

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
