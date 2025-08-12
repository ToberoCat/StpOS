//
// Created by tobias on 12/26/24.
//

#pragma once
#include "sensor.h"

namespace libstp::sensor
{
    class IrLightSensor final : public LightSensor
    {
    public:
        explicit IrLightSensor(const int& port, const float& calibrationFactor = 0.3f);
        
        int getValue() override;

        bool isOnBlack() override;
    };
}
