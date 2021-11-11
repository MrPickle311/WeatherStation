#pragma once

#include <stdint-gcc.h>
#include <functional>

namespace Processing
{

class Task
{
public:
	using PriorityType = uint8_t;
	using TaskType 	    =  std::function<void()>;
private:
	TaskType  task_;
	PriorityType priority_;
public:
	Task( TaskType   fn , PriorityType priority );
	virtual ~Task() = default;
public:
	void operator() ();
	friend bool operator< (const Task& lhs ,const Task& rhs );
};

/*

class ITask
{
public:
	virtual ~ITask() = default;
	virtual std::size_t getSize() const = 0;
	virtual void exec() = 0;
};

template <typename TaskType>
class TaskBound : public Task
{
public:

    // Size is minimal number of elements of size equal to sizeof(Task)
    // that will be able to store this TaskBound object
    static const std::size_t size =
        ((sizeof(TaskBound<typename std::decay<TaskType>::type>) - 1) /
                                                     sizeof(Task)) + 1;

    explicit TaskBound(const TaskType& task)
      : task_(task)
    {
    }

    explicit TaskBound(TaskType&& task)
      : task_(std::move(task))
    {
    }

    virtual ~TaskBound() {}

    virtual std::size_t getSize() const
    {
        return size;
    }

    virtual void exec()
    {
        task_();
    }

private:
    TaskType task_;
};

*/

}
