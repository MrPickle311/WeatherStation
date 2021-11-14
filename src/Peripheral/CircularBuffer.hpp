#pragma once

#include <memory>
#include <optional>

namespace Device
{

template <typename DataType, typename SizeType = size_t>
class CircularBuffer
{
public:
    explicit CircularBuffer(SizeType size) :
        buf_(std::unique_ptr<DataType[]>(new DataType[size])),
        max_size_(size),
        head_{0},
        tail_{0},
        full_{false}
    {}

public:
    void put(DataType item)
    {
        buf_[head_] = item;

        if (full_)
        {
            tail_ = (tail_ + 1) % max_size_;
        }

        head_ = (head_ + 1) % max_size_;

        full_ = head_ == tail_;
    }
    std::optional<DataType> get()
    {
        if (empty())
        {
            return std::nullopt;
        }

        auto val = buf_[tail_];
        full_ = false;
        tail_ = (tail_ + 1) % max_size_;

        return val;
    }
    void reset()
    {
        head_ = tail_;
        full_ = false;
    }
    bool empty() const
    {
        return !full_ && (head_ == tail_);
    }
    bool full() const
    {
        return full_;
    }
    SizeType capacity() const
    {
        return max_size_;
    }
    SizeType size() const
    {
        SizeType size{max_size_};

        if (!full_)
        {
            if (head_ >= tail_)
            {
                size = head_ - tail_;
            }
            else
            {
                size = max_size_ + head_ - tail_;
            }
        }

        return size;
    }

private:
    std::unique_ptr<DataType[]> buf_;
    SizeType head_;
    SizeType tail_;
    const SizeType max_size_;
    bool full_;
};
} // namespace Device
