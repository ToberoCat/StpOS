//
// Created by tobias on 12/26/24.
//

#include "libstp/datatype/functions.h"

#include <libstp/_config.h>
#include <optional>
#include "libstp/math/math.h"

libstp::datatype::ConditionalFunction libstp::datatype::forTime(const std::chrono::milliseconds& timeInMs)
{
    std::optional<std::chrono::time_point<std::chrono::high_resolution_clock>> start;
    return [timeInMs, start](bool typeCheckOnly) mutable
    {
        if (typeCheckOnly)
        {
            return std::make_shared<TimedConditionalResult>(
                static_cast<float>(timeInMs.count()),
                0.0f
            );
        }
        
        const auto now = std::chrono::high_resolution_clock::now();

        if (!start.has_value())
        {
            start = now;
        }

        return std::make_shared<TimedConditionalResult>(
            static_cast<float>(timeInMs.count()),
            static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(now - start.value()).count())
        );
    };
}

libstp::datatype::ConditionalFunction libstp::datatype::forSeconds(const float& seconds)
{
    SPDLOG_DEBUG("[CallLog] forSeconds called with seconds: {}", seconds);
    return forTime(std::chrono::milliseconds(static_cast<int>(seconds * 1000)));
}

libstp::datatype::ConditionalFunction libstp::datatype::forDistance(const float& distanceCm)
{
    SPDLOG_DEBUG("[CallLog] forDistance called with distanceCm: {}", distanceCm);
    return [distanceCm](bool)
    {
        return std::make_shared<DistanceConditionalResult>(distanceCm);
    };
}

libstp::datatype::ConditionalFunction libstp::datatype::forCCWRotation(const float& rotationInDegrees)
{
    SPDLOG_DEBUG("[CallLog] forCCWRotation called with rotationInDegrees: {}", rotationInDegrees);
    return forCWRotation(-rotationInDegrees);
}

libstp::datatype::ConditionalFunction libstp::datatype::forCWRotation(const float& rotationInDegrees)
{
    SPDLOG_DEBUG("[CallLog] forCWRotation called with rotationInDegrees: {}", rotationInDegrees);
    return [rotationInDegrees](bool)
    {
        return std::make_shared<RotationConditionalResult>(rotationInDegrees);
    };
}

libstp::datatype::ConditionalFunction libstp::datatype::forTicks(const int& ticks)
{
    SPDLOG_DEBUG("[CallLog] forTicks called with ticks: {}", ticks);
    return [ticks](bool)
    {
        return std::make_shared<MotorTicksConditionalResult>(static_cast<float>(ticks));
    };
}

libstp::datatype::ConditionalFunction libstp::datatype::forAbsoluteTicks(const int& ticks)
{
    SPDLOG_DEBUG("[CallLog] forAbsoluteTicks called with ticks: {}", ticks);
    return [ticks](bool)
    {
        return std::make_shared<MotorTicksConditionalResult>(static_cast<float>(ticks));
    };
}

libstp::datatype::ConditionalFunction libstp::datatype::whileTrue(const std::function<bool()>& condition)
{
    SPDLOG_DEBUG("[CallLog] whileTrue called with condition");
    return [condition](bool)
    {
        return std::make_shared<UndefinedConditionalResult>(condition());
    };
}

libstp::datatype::ConditionalFunction libstp::datatype::whileFalse(const std::function<bool()>& condition)
{
    SPDLOG_DEBUG("[CallLog] whileFalse called with condition");
    return [condition](bool)
    {
        return std::make_shared<UndefinedConditionalResult>(!condition());
    };
}

libstp::datatype::SpeedFunction libstp::datatype::generator(const std::function<Speed()>& generator)
{
    SPDLOG_DEBUG("[CallLog] generator called");
    return [generator](const std::shared_ptr<ConditionalResult>&)
    {
        return generator();
    };
}

libstp::datatype::SpeedFunction libstp::datatype::constant(const Speed& speed)
{
    SPDLOG_DEBUG("[CallLog] constant called with speed: ({}, {}, {})", speed.forwardPercent, speed.strafePercent,
                 speed.angularPercent);
    return [speed](const std::shared_ptr<ConditionalResult>&)
    {
        return speed;
    };
}

libstp::datatype::SpeedFunction libstp::datatype::lerp(const Speed& startSpeed, const Speed& endSpeed)
{
    SPDLOG_DEBUG("[CallLog] lerp called");
    return [startSpeed, endSpeed](const std::shared_ptr<ConditionalResult>& result)
    {
        const auto progress = result->progress();
        return Speed{
            math::lerp(startSpeed.forwardPercent, endSpeed.forwardPercent, progress),
            math::lerp(startSpeed.angularPercent, endSpeed.angularPercent, progress)
        };
    };
}
