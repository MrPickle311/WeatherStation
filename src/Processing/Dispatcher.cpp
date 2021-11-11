#include "Dispatcher.hpp"

namespace Processing
{

bool Dispatcher::containsObserver(Dispatcher::DescriptorType type) const
{
	return ! (observers_.find(type) == observers_.end());
}

void Dispatcher::subscribeEventGroup(Dispatcher::DescriptorType descriptor , SlotType&& slot )
{
	observers_[descriptor] = slot;
}

void Dispatcher::post( const IEvent& event ) const
{
	auto group{event.group()};

	if(!containsObserver(group))
	{
		return;
	}

	observers_.at(group)(event);
}

Dispatcher&  Dispatcher::getInstance()
{
	static Dispatcher dispatcher;

	return dispatcher;
}

}
