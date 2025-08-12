//
// Created by tobias on 12/26/24.
//
#include "kipr/servo/servo.h"
#include "libstp/servo/servo.h"

#include <cmath>

#include "libstp/utility/timing.h"
#include "libstp/_config.h"
#include "libstp/async/algorithm.h"

constexpr int MIN_POSITION = 0;
constexpr int MAX_POSITION = 2047;

void libstp::servo::Servo::setPosition(const int position)
{
    set_servo_position(port, math::clampInt(position, MIN_POSITION, MAX_POSITION));
}

int libstp::servo::Servo::getPosition()
{
    return get_servo_position(port);
}

libstp::async::AsyncAlgorithm<int> libstp::servo::Servo::slowlySetPosition(const int targetPosition,
                                                                           const std::chrono::milliseconds duration,
                                                                           math::InterpolationFunction interpolationFunction)
{
    const int startPosition = getPosition();
    if (startPosition == targetPosition)
    {
        co_return 0;
    }
    if (duration.count() <= 0)
    {
        throw std::invalid_argument(
            "The target position is the same as the current position or the duration is invalid.");
    }
    
    const int clampedTarget = math::clampInt(targetPosition, MIN_POSITION, MAX_POSITION);
    
    using clock = std::chrono::steady_clock;
    const auto startTime = clock::now();
    const auto endTime = startTime + duration;
    
    const std::chrono::duration<double> total = endTime - startTime;
    while (true)
    {
        const auto now = clock::now();
        if (now >= endTime)
        {
            break;
        }
    
        std::chrono::duration<double> elapsed = now - startTime;
        double normalizedTime = elapsed.count() / total.count();
        normalizedTime = std::clamp(normalizedTime, 0.0, 1.0);
    
        const float interpolatedPosition = interpolationFunction(
            static_cast<float>(startPosition),
            static_cast<float>(clampedTarget),
            static_cast<float>(normalizedTime)
        );
    
        const int newPosition = static_cast<int>(std::round(interpolatedPosition));
        setPosition(newPosition);
    
        co_yield 1;
    }

    setPosition(clampedTarget);
}

void libstp::servo::Servo::shake(const int centerPosition,
                                 float amplitude,
                                 const float speedHz,
                                 const datatype::ConditionalFunction& conditional)
{
    const int clampedCenter = math::clampInt(centerPosition, MIN_POSITION, MAX_POSITION);
    const auto maxPossibleAmplitudePos = static_cast<float>(MAX_POSITION - clampedCenter);
    const auto maxPossibleAmplitudeNeg = static_cast<float>(clampedCenter - MIN_POSITION);
    const float maxPossibleAmplitude = std::min(maxPossibleAmplitudePos, maxPossibleAmplitudeNeg);

    float clampedAmplitude = amplitude;
    bool amplitudeAdjusted = false;

    if (clampedAmplitude > maxPossibleAmplitude)
    {
        clampedAmplitude = maxPossibleAmplitude;
        amplitudeAdjusted = true;
    }

    if (clampedAmplitude < 0.0f)
    {
        clampedAmplitude = 0.0f;
        amplitudeAdjusted = true;
    }


    if (amplitudeAdjusted)
    {
        SPDLOG_WARN(
            "Requested amplitude of {} exceeds the maximum possible amplitude of {} for centerPosition {}. Amplitude has been adjusted accordingly.",
            amplitude, clampedAmplitude, clampedCenter);
    }

    if (clampedAmplitude == 0.0f || speedHz <= 0.0f)
    {
        SPDLOG_WARN("Shake parameters are invalid. Servo will not be moved.");
        setPosition(clampedCenter);
        return;
    }

    using clock = std::chrono::steady_clock;
    const auto startTime = clock::now();

    constexpr auto twoPi = static_cast<float>(2.0 * M_PI);
    const float angularFrequency = twoPi * speedHz;

    while (conditional(false)->is_loop_running())
    {
        const auto now = clock::now();
        std::chrono::duration<float> elapsed = now - startTime;
        const float t = elapsed.count();

        const float offset = clampedAmplitude * std::sin(angularFrequency * t);

        const float newPositionF = static_cast<float>(clampedCenter) + offset;
        int newPosition = static_cast<int>(std::round(newPositionF));

        newPosition = math::clampInt(newPosition, MIN_POSITION, MAX_POSITION);

        setPosition(newPosition);

        utility::msleep();
    }

    setPosition(clampedCenter);
}

void libstp::servo::Servo::disable()
{
    disable_servo(port);
}


void libstp::servo::Servo::enable()
{
    enable_servo(port);
}

void libstp::servo::Servo::disableAllServos()
{
    disable_servos();
}
