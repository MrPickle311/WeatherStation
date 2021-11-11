#pragma once

#include <functional>

namespace Program
{

template <typename ForwardPinType, typename BackwardPinType, typename PWMTimerType>
class Engine
{
private:
    auto static constexpr pwm_max_value{PWMTimerType::pwm_max_value};

private:
    PWMTimerType& pwm_timer_;

    ForwardPinType& forward_pin_;
    BackwardPinType& backward_pin_;

public:
    Engine(PWMTimerType& pwm_timer,
           ForwardPinType& forward_pin,
           BackwardPinType& backward_pin) :
        pwm_timer_{pwm_timer},
        forward_pin_{forward_pin},
        backward_pin_{backward_pin}
    {}

public:
    void setDriveForward()
    {
        forward_pin_::setHigh();
        backward_pin_::setLow();
    }
    void setDriveBackwards()
    {
        forward_pin_::setLow();
        backward_pin_::setHigh();
    }
    void runEngine()
    {
        pwm_timer_.setPwmSignalFilling(pwm_max_value);
    }
    void stopEngine()
    {
        pwm_timer_.setPwmSignalFilling(0);
    }
};

template <typename LeftEngineType, typename RightEngineType>
class Engines
{
private:
    LeftEngineType& left_engine_;
    RightEngineType& right_engine_;

public:
    using ActionHandlerType = std::function<void()>;
    ActionHandlerType turn_engines_on_handler_;
    ActionHandlerType turn_engines_off_handler_;

public:
    Engines(LeftEngineType& left_engine, RightEngineType& right_engine) :
        left_engine_{left_engine}, right_engine_{right_engine}
    {}
    virtual ~Engines() = default;
    Engines& operator=(Engines&& other) = delete;
    Engines& operator=(const Engines& other) = delete;
    Engines(Engines&& other) = delete;
    Engines(const Engines& other) = delete;

private:
    void runEngines()
    {
        left_engine_.runEngine();
        right_engine_.runEngine();
    }
    void stopEngines()
    {
        left_engine_.stopEngine();
        right_engine_.stopEngine();
    }

public:
    void driveForward()
    {
        left_engine_.setDriveForward();
        right_engine_.setDriveForward();
        runEngines();
    }
    void driveBackwards()
    {
        left_engine_.setDriveBackward();
        right_engine_.setDriveBackward();
        runEngines();
    }
    void turnLeft()
    {
        left_engine_.setDriveBackward();
        right_engine_.setDriveForward();
        runEngines();
    }
    void turnRight()
    {
        left_engine_.setDriveForward();
        right_engine_.setDriveBackward();
        runEngines();
    }
};

template <typename EnginesType, typename TimerType>
class TimedEngines
{
    using TimeoutType = uint16_t;

private:
    EnginesType& engines_;
    TimerType& timer_;

    TimeoutType timeout_;

public:
    TimedEngines(EnginesType& engines, TimerType& timer) :
        engines_{engines}, timer_{timer}
    {
        timer_.on_timeout_handler_ = [engines = &engines_]
        {
            engines.stopEngines();
        };
    }

public:
    void setTimetout(TimeoutType timeout)
    {
        timeout_ = timeout;
    }
    void driveForward()
    {
        engines_.driveForward();
        timer_.startTimer();
    }
    void driveBackwards()
    {
        engines_.driveBackwards();
        timer_.startTimer();
    }
    void turnLeft()
    {
        engines_.turnRight();
        timer_.startTimer();
    }
    void turnRight()
    {
        engines_.turnRight();
        timer_.startTimer();
    }
};

} // namespace Program
