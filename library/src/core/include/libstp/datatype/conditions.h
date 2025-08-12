//
// Created by tobias on 12/26/24.
//

#pragma once

#include "libstp/motion/differential_drive_state.h"
#include <string>

namespace libstp::datatype
{
    class ConditionalResult
    {
    public:
        virtual ~ConditionalResult() = default;

        virtual void update(motion::DifferentialDriveState& state) = 0;
        [[nodiscard]] virtual float progress() const = 0;
        [[nodiscard]] virtual bool is_loop_running() const = 0;
        [[nodiscard]] virtual std::string to_string() const = 0;
    };

    class UndefinedConditionalResult final : public ConditionalResult
    {
    public:
        bool _conditionMet;

        explicit UndefinedConditionalResult(bool conditionMet);

        UndefinedConditionalResult(const UndefinedConditionalResult& other);

        UndefinedConditionalResult& operator=(const UndefinedConditionalResult& other);

        [[nodiscard]] float progress() const override;

        [[nodiscard]] bool is_loop_running() const override;

        void update(motion::DifferentialDriveState& state) override;
        
        [[nodiscard]] std::string to_string() const override;
    };

    class DefinedConditionalResult : public ConditionalResult
    {
    public:
        float target;
        float current;
        bool _is_loop_running = true;

        explicit DefinedConditionalResult(float target);

        [[nodiscard]] float progress() const override;

        [[nodiscard]] bool is_loop_running() const override;

        void update(motion::DifferentialDriveState& state) override;
        
        [[nodiscard]] std::string to_string() const override;
    };

    class TimedConditionalResult final : public DefinedConditionalResult
    {
    public:
        TimedConditionalResult(float target, float current);

        [[nodiscard]] float progress() const override;

        [[nodiscard]] bool is_loop_running() const override;

        void update(motion::DifferentialDriveState& state) override;
        
        [[nodiscard]] std::string to_string() const override;
    };

    class DistanceConditionalResult : public DefinedConditionalResult
    {
    protected:
        void updateResult(float distance);

    public:
        explicit DistanceConditionalResult(float targetCm);

        void update(motion::DifferentialDriveState& state) override;
        
        [[nodiscard]] std::string to_string() const override;
    };

    class RotationConditionalResult final : public DefinedConditionalResult
    {
        bool isAccurate;
    public:
        explicit RotationConditionalResult(float target);

        void update(motion::DifferentialDriveState& state) override;
        
        [[nodiscard]] std::string to_string() const override;
    };

    class MotorTicksConditionalResult final : public DefinedConditionalResult
    {
    public:
        explicit MotorTicksConditionalResult(float target);

        void update(motion::DifferentialDriveState& state) override;
        
        [[nodiscard]] std::string to_string() const override;
    };
}
