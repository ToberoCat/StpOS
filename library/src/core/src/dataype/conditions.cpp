//
// Created by tobias on 2/8/25.
//
#include "libstp/datatype/conditions.h"
#include "libstp/_config.h"

#include <stdexcept>
#include <sstream>
#include <iomanip>

#include "libstp/math/math.h"

libstp::datatype::UndefinedConditionalResult::UndefinedConditionalResult(const bool conditionMet): _conditionMet(conditionMet)
{
}

libstp::datatype::UndefinedConditionalResult::UndefinedConditionalResult(const UndefinedConditionalResult& other): _conditionMet(other._conditionMet)
{
}

libstp::datatype::UndefinedConditionalResult& libstp::datatype::UndefinedConditionalResult::operator=(
    const UndefinedConditionalResult& other)
{
    if (this != &other)
    {
        _conditionMet = other._conditionMet;
    }
    return *this;
}

float libstp::datatype::UndefinedConditionalResult::progress() const
{
    throw std::runtime_error("UndefinedConditionalResult does not have a progress");
}

bool libstp::datatype::UndefinedConditionalResult::is_loop_running() const
{
    return _conditionMet;
}

void libstp::datatype::UndefinedConditionalResult::update(motion::DifferentialDriveState& state)
{
}

std::string libstp::datatype::UndefinedConditionalResult::to_string() const
{
    std::ostringstream oss;
    oss << "UndefinedConditionalResult: condition_met=" << (_conditionMet ? "true" : "false");
    return oss.str();
}

libstp::datatype::DefinedConditionalResult::DefinedConditionalResult(const float target): target(target), current(0), _is_loop_running(false)
{
}

float libstp::datatype::DefinedConditionalResult::progress() const
{
    return current / target;
}

bool libstp::datatype::DefinedConditionalResult::is_loop_running() const
{
    return _is_loop_running;
}

void libstp::datatype::DefinedConditionalResult::update(motion::DifferentialDriveState& state)
{
}

std::string libstp::datatype::DefinedConditionalResult::to_string() const
{
    std::ostringstream oss;
    oss << "DefinedConditionalResult: target=" << target 
        << ", current=" << current
        << ", progress=" << std::fixed << std::setprecision(2) << (progress() * 100.0f) << "%"
        << ", running=" << (_is_loop_running ? "true" : "false");
    return oss.str();
}

libstp::datatype::TimedConditionalResult::TimedConditionalResult(const float target, const float current): DefinedConditionalResult(target)
{
    this->current = current;
}

float libstp::datatype::TimedConditionalResult::progress() const
{
    return current / target;
}

bool libstp::datatype::TimedConditionalResult::is_loop_running() const
{
    return current < target;
}

void libstp::datatype::TimedConditionalResult::update(motion::DifferentialDriveState& state)
{
}

std::string libstp::datatype::TimedConditionalResult::to_string() const
{
    std::ostringstream oss;
    oss << "TimedConditionalResult: target=" << target << "s"
        << ", current=" << current << "s"
        << ", remaining=" << (target - current) << "s"
        << ", progress=" << std::fixed << std::setprecision(2) << (progress() * 100.0f) << "%"
        << ", running=" << (is_loop_running() ? "true" : "false");
    return oss.str();
}

void libstp::datatype::DistanceConditionalResult::updateResult(const float distance)
{
    const auto targetDistanceM = target / 100.0f;
    current = distance;
    _is_loop_running = std::abs(distance) <= targetDistanceM;
    SPDLOG_TRACE("Distance Conditional Result - Target: {}, Current: {}", targetDistanceM, current);
}

libstp::datatype::DistanceConditionalResult::DistanceConditionalResult(const float targetCm): DefinedConditionalResult(targetCm)
{
}

void libstp::datatype::DistanceConditionalResult::update(motion::DifferentialDriveState& state)
{
    const auto [forwardDistance, sidewaysDistance] = state.computeDrivenDistance();
    updateResult(std::sqrt(forwardDistance * forwardDistance + sidewaysDistance * sidewaysDistance));
}

std::string libstp::datatype::DistanceConditionalResult::to_string() const
{
    std::ostringstream oss;
    oss << "DistanceConditionalResult: target=" << target << "cm"
        << ", current=" << current << "m"
        << ", progress=" << std::fixed << std::setprecision(2) << (progress() * 100.0f) << "%"
        << ", running=" << (_is_loop_running ? "true" : "false");
    return oss.str();
}

libstp::datatype::RotationConditionalResult::RotationConditionalResult(float target):
    DefinedConditionalResult(target)
{
    this->target = target;
}

void libstp::datatype::RotationConditionalResult::update(motion::DifferentialDriveState& state)
{
    state.desiredHeading = DEG_TO_RAD * target;
    current = state.currentHeading;
    // 0.01 rad ~= 0.5 deg
    _is_loop_running = std::abs(current) <= std::abs(state.desiredHeading) - 0.01f;
    SPDLOG_TRACE("Rotation Conditional Result - Target: {}, Current: {}", target * DEG_TO_RAD, current);
}

std::string libstp::datatype::RotationConditionalResult::to_string() const
{
    std::ostringstream oss;
    oss << "RotationConditionalResult: target=" << target << "°"
        << ", current=" << RAD_TO_DEG * current << "°"
        << ", remaining=" << RAD_TO_DEG * (target * DEG_TO_RAD - current) << "°"
        << ", progress=" << std::fixed << std::setprecision(2) << (progress() * 100.0f) << "%"
        << ", running=" << (_is_loop_running ? "true" : "false");
    return oss.str();
}

libstp::datatype::MotorTicksConditionalResult::MotorTicksConditionalResult(const float target): DefinedConditionalResult(target)
{
}

void libstp::datatype::MotorTicksConditionalResult::update(motion::DifferentialDriveState& state)
{
}

std::string libstp::datatype::MotorTicksConditionalResult::to_string() const
{
    std::ostringstream oss;
    oss << "MotorTicksConditionalResult: target=" << target << " ticks"
        << ", current=" << current << " ticks"
        << ", remaining=" << (target - current) << " ticks"
        << ", progress=" << std::fixed << std::setprecision(2) << (progress() * 100.0f) << "%"
        << ", running=" << (_is_loop_running ? "true" : "false");
    return oss.str();
}

