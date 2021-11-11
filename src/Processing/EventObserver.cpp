#include "EventObserver.hpp"
#include "TaskQueue.hpp"

namespace Processing
{

void EventCallbacksStorage::subscribeEventResponse(EventDescriptorType event_type , EventCallbackType&& callback )
{
	events_group_[event_type] = std::move(callback);
}


void BluetoothEventObserver::handle(const IEvent& event)
{
	TaskQueue::getInstance().addTask( events_group_.at(event.what()) );
//											HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//											HAL_UART_Transmit(&huart2, const_cast<uint8_t*>(nm) , 2 , 200);
}

}
