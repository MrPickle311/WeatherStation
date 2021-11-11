#include "Task.hpp"

namespace Processing
{

Task::Task(TaskType fn , PriorityType priority):
		task_{fn} ,
		priority_{priority}
{

}

bool operator< (const Task& lhs ,const Task& rhs )
{
	return lhs.priority_ < rhs.priority_;
}

void Task::operator() ()
{
	task_();
}

}
