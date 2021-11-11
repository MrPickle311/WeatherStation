#pragma once

#include "../Processing/Event.hpp"
#include "../Processing/TaskQueue.hpp"

namespace Program
{

template <typename StreamDeviceType>
class BluetoothOStream
{
private:
    StreamDeviceType& stream_;

public:
    BluetoothOStream(StreamDeviceType& stream) : stream_{stream} {}
    BluetoothOStream(const BluetoothPort& other) = delete;
    BluetoothOStream(BluetoothPort&& other) = delete;
    BluetoothOStream& operator=(const BluetoothPort& other) = delete;
    BluetoothOStream& operator=(BluetoothPort&& other) = delete;
    virtual ~BluetoothOStream() = default;


    static BluetoothPort& getInstance();
};


template <typename StreamDeviceType>
class BluetoothIStream
{
private:
    StreamDeviceType& stream_;

public:
    BluetoothIStream(StreamDeviceType& stream) : stream_{stream} {}

public:
    void waitForCommands();
    void parseIncomingData(const std::vector<uint8_t>& data);
    void postTaskToTaskQueue(BluetoothEvent&& event);
};

} // namespace Program
