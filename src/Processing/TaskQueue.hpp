#pragma once

#include <queue>
#include <any>
#include <functional>

namespace Processing
{

class TaskQueue
{
private:
	std::queue<std::function<void()>> tasks_;
public:

	using TaskType = std::function<void()>;

	virtual ~TaskQueue() = default;
public:
	template<typename TaskT>
	void addTask(TaskT&& task)
	{
		tasks_.push(task);
	}
	TaskType getTask();
	bool isEmpty() const;

	static TaskQueue& getInstance();
};

}
