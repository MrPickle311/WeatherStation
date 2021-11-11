#include "BluetoothPort.hpp"

#include "../Processing/Dispatcher.hpp"

namespace Program
{

BluetoothPort::BluetoothPort()
{
	using std::placeholders::_1;

	UartDevice::getInstance().on_data_arrived_action =
			std::bind(&BluetoothPort::parseIncomingData , this , _1 );
}

void BluetoothPort::parseIncomingData(const std::vector<uint8_t>& data)
{
//	Dispatcher::getInstance().post(BluetoothDataArrivedEvent{});

	if(data[1] == 'f')
	{
		Dispatcher::getInstance().post(MoveForward{});
	}

	if(data[1] == 'b')
	{
		Dispatcher::getInstance().post(MoveBackwards{});
	}

	if(data[1] == 'r')
	{
		Dispatcher::getInstance().post(TurnRight{});
	}

	if(data[1] == 'l')
	{
		Dispatcher::getInstance().post(TurnLeft{});
	}

	TaskQueue::getInstance().addTask([&data]{
			HAL_UART_Transmit(&huart2, const_cast<uint8_t*>(data.data()) , data.size(), 100);
	});
}

void BluetoothPort::postTaskToTaskQueue(BluetoothEvent&& event)
{
	TaskQueue::getInstance().addTask([]{});
}

BluetoothPort& BluetoothPort::getInstance()
{
	static BluetoothPort port;

	return port;
}

void BluetoothPort::waitForCommands()
{
	UartDevice::getInstance().waitForData(2);
}


}
