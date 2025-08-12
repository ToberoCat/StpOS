//
// Created by tobias on 12/26/24.
//

#pragma once
#include <chrono>

#include "libstp/async/algorithm.h"
#include "libstp/datatype/functions.h"
#include "libstp/math/math.h"

namespace libstp::servo
{
    class Servo
    {
        int port;

    public:
        virtual ~Servo() = default;

        explicit Servo(const int port) : port(port)
        {
        }

        virtual void setPosition(int position);

        [[nodiscard]] virtual int getPosition();

        virtual async::AsyncAlgorithm<int> slowlySetPosition(int targetPosition,
                                       std::chrono::milliseconds duration,
                                       math::InterpolationFunction interpolationFunction);

        void shake(int centerPosition,
                   float amplitude,
                   float speedHz, const datatype::ConditionalFunction& conditional);

        virtual void disable();

        virtual void enable();

        static void disableAllServos();
    };
}
