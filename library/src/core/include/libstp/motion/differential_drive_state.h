//
// Created by tobias on 1/15/25.
//

#pragma once
#include <functional>

namespace libstp::motion
{
    struct DifferentialDriveState
    {
        float currentHeading, desiredHeading;
        std::function<std::pair<float, float>()> computeDrivenDistance;
        
        // Speed ramp state variables
        float rampedForwardMs = 0.0f;
        float rampedStrafeMs = 0.0f;
        float rampedOmegaRad = 0.0f;

        explicit DifferentialDriveState(const std::function<std::pair<float, float>()>& compute_driven_distance)
            : DifferentialDriveState(0, 0, compute_driven_distance)
        {
        }

        DifferentialDriveState(const float current_heading, const float desired_heading,
                               const std::function<std::pair<float, float>()>& compute_driven_distance)
            : currentHeading(current_heading),
              desiredHeading(desired_heading),
              computeDrivenDistance(compute_driven_distance),
              rampedForwardMs(0.0f),
              rampedStrafeMs(0.0f),
              rampedOmegaRad(0.0f)
        {
        }
    };
}
