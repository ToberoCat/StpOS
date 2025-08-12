//
// Created by tobias on 12/26/24.
//

#pragma once

#include <chrono>

namespace libstp::utility
{
    struct PidParameters
    {
        float Kp;
        float Ki;
        float Kd;


        PidParameters() : PidParameters(0.0f, 0.0f, 0.0f)
        {
        }

        PidParameters(const float kp, const float ki, const float kd)
            : Kp(kp),
              Ki(ki),
              Kd(kd)
        {
        }
    };

    class PIDController
    {
    public:
        PIDController() : PIDController(0.0f, 0.0f, 0.0f)
        {
        }

        PIDController(const float Kp, const float Ki, const float Kd) : PIDController(PidParameters(Kp, Ki, Kd))
        {
        }

        explicit PIDController(const PidParameters& parameters)
            : parameters(parameters), integral(0.0),
              previous_error(0.0),
              last_time(std::chrono::steady_clock::now())
        {
        }

        void setParameters(const PidParameters parameters)
        {
            this->parameters = parameters;
            reset();
        }

        void reset()
        {
            integral = 0.0;
            previous_error = 0.0;
            last_time = std::chrono::steady_clock::now();
        }

        float calculate(const float error)
        {
            const auto current_time = std::chrono::steady_clock::now();
            const std::chrono::duration<double> time_diff = current_time - last_time;
            last_time = current_time;

            integral += error * time_diff.count();
            const double derivative = (error - previous_error) / time_diff.count();
            previous_error = error;

            return static_cast<float>(error * parameters.Kp + parameters.Ki * integral + parameters.Kd * derivative);
        }

    private:
        PidParameters parameters;

        double integral;
        double previous_error;
        std::chrono::steady_clock::time_point last_time;
    };
}
