#include "TaskQueue.hpp"

namespace Processing
{

TaskQueue::TaskType TaskQueue::getTask()
{
	auto task{std::move(tasks_.front())};
	tasks_.pop();
	return task;
}

bool TaskQueue::isEmpty() const
{
	return tasks_.empty();
}

TaskQueue& TaskQueue::getInstance()
{
	static TaskQueue task_queue{};
	return task_queue;
}

}
