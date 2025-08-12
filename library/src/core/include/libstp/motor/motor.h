//
// Created by tobias on 12/26/24.
//

#pragma once
#include <spdlog/spdlog.h>

#include "libstp/async/algorithm.h"
#include "libstp/datatype/functions.h"

namespace libstp::motor
{
    class Motor
    {
    protected:
        int port;
        int reversePolarity;

    public:
        explicit Motor(const int port, const bool reversePolarity = false);

        [[nodiscard]] int getCurrentPositionEstimate() const;

        void resetPositionEstimate() const;

        void setVelocity(int velocity) const;

        async::AsyncAlgorithm<int> moveWhile(datatype::ConditionalFunction condition, int velocity) const;

        async::AsyncAlgorithm<int> moveByTicks(int ticks, int velocity = 500) const;

        async::AsyncAlgorithm<int> moveToTicks(int ticks, int velocity = 500) const;
        
        void stop() const;

        static void stopAllMotors();
    };
}
