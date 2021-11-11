#include "Event.hpp"

namespace Processing
{

IEvent::EventGroupType BluetoothEvent::group() const
{
	return BluetoothEvent::event_group;
}

IEvent::EventDescriptorType BluetoothDataArrivedEvent::what() const
{
	return BluetoothDataArrivedEvent::event_descriptor;
}

}
