//
// Created by tobias on 1/12/25.
//

#pragma once
#include "libstp/datatype/conditions.h"
#include "libstp/motion/differential_drive.h"

namespace libstp::device::omni_wheeled::datatype
{
    class ForwardDistanceConditionalResult final : public libstp::datatype::DistanceConditionalResult
    {
    public:
        explicit ForwardDistanceConditionalResult(const float targetCm)
            : DistanceConditionalResult(targetCm)
        {
        }

        void update(motion::DifferentialDriveState& state) override
        {
            auto [forwardDistance, strafeDistance] = state.computeDrivenDistance();
            updateResult(forwardDistance);
        }
    };

    class SideDistanceConditionalResult final : public libstp::datatype::DistanceConditionalResult
    {
    public:
        explicit SideDistanceConditionalResult(const float targetCm)
            : DistanceConditionalResult(targetCm)
        {
        }

        void update(motion::DifferentialDriveState& state) override
        {
            auto [forwardDistance, strafeDistance] = state.computeDrivenDistance();
            updateResult(strafeDistance);
        }
    };
}
