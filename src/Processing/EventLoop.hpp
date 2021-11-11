#include "TaskQueue.hpp"

namespace Processing
{

template <typename EnableIRQFunctionType, typename DisableIRQFunctionType>
class EventLoop
{
private:
    TaskQueue& queue_;

    EnableIRQFunctionType interrupt_enabler_;
    DisableIRQFunctionType interrupt_disabler_;

    std::function<void()> current_task_handler_;
    bool is_task_present_to_invoke_;

private:
    bool queueHasTask()
    {
        return !queue_.isEmpty();
    }
    void tryAcquireHandler()
    {
        if (queueHasTask())
        {
            current_task_handler_ = std::move(queue_.getTask());
            is_task_present_to_invoke_ = true;
        }
    }
    void tryInvokeHandler()
    {
        if (is_task_present_to_invoke_)
        {
            current_task_handler_();
            is_task_present_to_invoke_ = false;
        }
    }

public:
    EventLoop(TaskQueue& queue,
              EnableIRQFunctionType interrupt_enabler,
              DisableIRQFunctionType interrupt_disabler) :
        queue_{queue},
        interrupt_enabler_{std::forward(interrupt_enabler)},
        interrupt_disabler_{interrupt_disabler},
        current_task_handler_{},
        is_task_present_to_invoke_{false}
    {}
    virtual ~EventLoop() = default;
    EventLoop(const EventLoop& other) = delete;
    EventLoop(EventLoop&& other) = delete;
    EventLoop& operator=(const EventLoop& other) = delete;
    EventLoop& operator=(EventLoop&& other) = delete;

public:
    void start()
    {
        interrupt_disabler_();

        tryAcquireHandler();

        interrupt_enabler_();

        tryInvokeHandler();
    }
    //    static EventLoop& getInstance(TaskQueue& queue)
    //    {
    //        static EventLoop event_loop{queue};
    //        return event_loop;
    //    }
};


template <typename EnableIRQFunctionType, typename DisableIRQFunctionType>
class EventLoopBuilder
{};


} // namespace Processing
