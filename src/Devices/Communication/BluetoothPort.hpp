#pragma once

#include "../../Processing/Event.hpp"
#include "../../Processing/TaskQueue.hpp"
#include "../Processing/Dispatcher.hpp"

namespace Program
{

template <typename UARTDeviceType>
class BluetoothOStream
{
private:
    UARTDeviceType& stream_;

public:
    BluetoothOStream(UARTDeviceType& stream) : stream_{stream} {}
    BluetoothOStream(const BluetoothOStream& other) = delete;
    BluetoothOStream(BluetoothOStream&& other) = delete;
    BluetoothOStream& operator=(const BluetoothOStream& other) = delete;
    BluetoothOStream& operator=(BluetoothOStream&& other) = delete;
    virtual ~BluetoothOStream() = default;
};


template <typename UARTDeviceType>
class BluetoothIStream
{
private:
    UARTDeviceType& stream_;

public:
    BluetoothIStream(UARTDeviceType& stream) : stream_{stream}
    {
        //        using std::placeholders::_1;
        //
        //        UartDevice::getInstance().on_data_arrived_action =
        //            std::bind(&BluetoothPort::parseIncomingData, this, _1);
    }

public:
    void waitForCommands()
    {
        UartDevice::getInstance().waitForData(2);
    }
    void parseIncomingData(const std::vector<uint8_t>& data)
    {
        //        if (data[1] == 'f')
        //        {
        //            Dispatcher::getInstance().post(MoveForward{});
        //        }
        //
        //        if (data[1] == 'b')
        //        {
        //            Dispatcher::getInstance().post(MoveBackwards{});
        //        }
        //
        //        if (data[1] == 'r')
        //        {
        //            Dispatcher::getInstance().post(TurnRight{});
        //        }
        //
        //        if (data[1] == 'l')
        //        {
        //            Dispatcher::getInstance().post(TurnLeft{});
        //        }

        //        TaskQueue::getInstance().addTask(
        //            [&data]
        //            {
        //                HAL_UART_Transmit(&huart2,
        //                                  const_cast<uint8_t*>(data.data()),
        //                                  data.size(),
        //                                  100);
        //            });
    }
    void postTaskToTaskQueue(Processing::BluetoothEvent&& event)
    {
        //        TaskQueue::getInstance().addTask(
        //            []
        //            {
        //            });
    }
};

} // namespace Program
