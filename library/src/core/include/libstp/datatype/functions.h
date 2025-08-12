//
// Created by tobias on 12/26/24.
//

#pragma once

#include <chrono>
#include <functional>
#include <memory>

#include "conditions.h"
#include "speed.h"

namespace libstp::datatype
{
    typedef std::function<std::shared_ptr<ConditionalResult>(bool)> ConditionalFunction;
    typedef std::function<Speed(std::shared_ptr<ConditionalResult>)> SpeedFunction;

    // DefinedConditionals
    ConditionalFunction forTime(const std::chrono::milliseconds& timeInMs);

    ConditionalFunction forSeconds(const float& seconds);

    ConditionalFunction forDistance(const float& distanceCm);

    ConditionalFunction forCCWRotation(const float& rotationInDegrees);

    ConditionalFunction forCWRotation(const float& rotationInDegrees);

    ConditionalFunction forTicks(const int& ticks);
    
    ConditionalFunction forAbsoluteTicks(const int& ticks);

    // UndefinedConditionals
    ConditionalFunction whileTrue(const std::function<bool()>& condition);

    ConditionalFunction whileFalse(const std::function<bool()>& condition);

    // SpeedFunctions
    SpeedFunction generator(const std::function<Speed()>& generator);
    
    SpeedFunction constant(const Speed& speed);

    SpeedFunction lerp(const Speed& startSpeed, const Speed& endSpeed);

}
