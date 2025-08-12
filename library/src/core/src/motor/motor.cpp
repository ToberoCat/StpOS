//
// Created by tobias on 12/26/24.
//
#include "kipr/motor/motor.h"
#include "libstp/motor/motor.h"

#include <cassert>
#include <unordered_map>

#include "libstp/utility/pid.h"
#include "libstp/utility/timing.h"
#include "libstp/_config.h"
#include "libstp/async/algorithm.h"

constexpr int MIN_VELOCITY = -1500;
constexpr int MAX_VELOCITY = 1500;
constexpr int MIN_SAFETY_VELOCITY = -1000;
constexpr int MAX_SAFETY_VELOCITY = 1000;
// Minimum interval between velocity warning logs (in milliseconds)
constexpr unsigned long LOG_THROTTLE_MS = 1000;

int libstp::motor::Motor::getCurrentPositionEstimate() const
{
    return get_motor_position_counter(port) * reversePolarity;
}

void libstp::motor::Motor::resetPositionEstimate() const
{
    clear_motor_position_counter(port);
}

void libstp::motor::Motor::setVelocity(int velocity) const
{
    SPDLOG_TRACE("[Motor {}] Setting velocity to: {}", port, velocity);
    
    // Static variables to track when we last logged warnings
    static std::unordered_map<int, unsigned long> lastBoundsLogTime;
    static std::unordered_map<int, unsigned long> lastSafetyLogTime;
    
    // Current time
    const unsigned long currentTime = libstp::utility::get_current_time_millis();
    
    if (velocity < MIN_VELOCITY || velocity > MAX_VELOCITY)
    {
        // Log an error for out-of-bounds velocity, but throttle the logs
        if (lastBoundsLogTime.find(port) == lastBoundsLogTime.end() || 
            currentTime - lastBoundsLogTime[port] > LOG_THROTTLE_MS) 
        {
            SPDLOG_ERROR("Velocity out of bounds for motor {}. Setting velocity to nearest bound: {}", 
                         port, velocity);
            lastBoundsLogTime[port] = currentTime;
        }
        velocity = std::clamp(velocity, MIN_VELOCITY, MAX_VELOCITY);
    }

    if (velocity < MIN_SAFETY_VELOCITY || velocity > MAX_SAFETY_VELOCITY)
    {
        // Log a warning for safety bounds, but throttle the logs
        if (lastSafetyLogTime.find(port) == lastSafetyLogTime.end() || 
            currentTime - lastSafetyLogTime[port] > LOG_THROTTLE_MS) 
        {
            SPDLOG_WARN("Velocity is out of safety bounds for motor {}. This may cause the tick estimation to be inaccurate: {}", 
                        port, velocity);
            lastSafetyLogTime[port] = currentTime;
        }
    }
    mav(port, reversePolarity * velocity);
}

libstp::async::AsyncAlgorithm<int> libstp::motor::Motor::moveWhile(datatype::ConditionalFunction condition, int velocity) const
{
    const auto result = condition(true);
    if (const auto* mResult = dynamic_cast<datatype::MotorTicksConditionalResult*>(result.get()))
    {
        SPDLOG_INFO("Condition is a MotorTicksConditionalResult");
        co_await moveByTicks(static_cast<int>(mResult->target), velocity);
    } else
    {
        SPDLOG_INFO("Condition is not a MotorTicksConditionalResult");
        while (condition(false)->is_loop_running())
        {
            setVelocity(velocity);
            co_yield 1;
        }
    }
    
    stop();
}

void libstp::motor::Motor::stop() const
{
    //off(port);
    freeze(port); // Active braking
}

void libstp::motor::Motor::stopAllMotors()
{
    ao();
}

libstp::async::AsyncAlgorithm<int> libstp::motor::Motor::moveByTicks(const int ticks, const int velocity) const
{
    const auto currentTicks = get_motor_position_counter(port);
    return moveToTicks(currentTicks + reversePolarity * ticks, velocity);
}

libstp::async::AsyncAlgorithm<int> libstp::motor::Motor::moveToTicks(const int ticks, int velocity) const
{
    SPDLOG_DEBUG("Moving motor to ticks: {}", ticks);
    
    // Static variables to track when we last logged warnings
    static std::unordered_map<int, unsigned long> lastVelocityZeroLogTime;
    static std::unordered_map<int, unsigned long> lastNegativeVelocityLogTime;
    static std::unordered_map<int, unsigned long> lastBoundsLogTime;
    static std::unordered_map<int, unsigned long> lastSafetyLogTime;
    
    // Current time
    const unsigned long currentTime = libstp::utility::get_current_time_millis();
    
    if (velocity == 0)
    {
        if (lastVelocityZeroLogTime.find(port) == lastVelocityZeroLogTime.end() || 
            currentTime - lastVelocityZeroLogTime[port] > LOG_THROTTLE_MS) 
        {
            SPDLOG_ERROR(
                "Velocity is 0 for motor {}. Using velocity of 500 instead", port);
            lastVelocityZeroLogTime[port] = currentTime;
        }
        velocity = 500;
    }
    
    if (velocity < 0)
    {
        if (lastNegativeVelocityLogTime.find(port) == lastNegativeVelocityLogTime.end() || 
            currentTime - lastNegativeVelocityLogTime[port] > LOG_THROTTLE_MS) 
        {
            SPDLOG_WARN(
                "Negative velocity for motor {} with for_ticks(). Consider using positive values", port);
            lastNegativeVelocityLogTime[port] = currentTime;
        }
    }

    if (velocity < MIN_VELOCITY || velocity > MAX_VELOCITY)
    {
        if (lastBoundsLogTime.find(port) == lastBoundsLogTime.end() || 
            currentTime - lastBoundsLogTime[port] > LOG_THROTTLE_MS) 
        {
            SPDLOG_ERROR("Velocity out of bounds for motor {}. Setting velocity to nearest bound", port);
            lastBoundsLogTime[port] = currentTime;
        }
        velocity = std::clamp(velocity, MIN_VELOCITY, MAX_VELOCITY);
    }

    if (velocity < MIN_SAFETY_VELOCITY || velocity > MAX_SAFETY_VELOCITY)
    {
        if (lastSafetyLogTime.find(port) == lastSafetyLogTime.end() || 
            currentTime - lastSafetyLogTime[port] > LOG_THROTTLE_MS) 
        {
            SPDLOG_WARN("Velocity is out of safety bounds for motor {}. This may cause the tick estimation to be inaccurate: {}", 
                        port, velocity);
            lastSafetyLogTime[port] = currentTime;
        }
    }

    move_to_position(port, velocity, ticks);
    co_yield 1;
    while (get_motor_done(port) == 0)
    {
        // There's still a bug (race condition) where the mrp function doesn't get sent to the stm32
        // This is a workaround resend the command every 10ms until the motor is done
        // Can be removed once lock issue has been resolved
        move_to_position(port, velocity, ticks);
        co_yield 1;
    }

    stop();
}

libstp::motor::Motor::Motor(const int port, const bool reversePolarity): port(port)
{
    this->reversePolarity = reversePolarity ? -1 : 1;
    SPDLOG_TRACE("Port: {}, Reverse Polarity: {}", port, reversePolarity);
}
