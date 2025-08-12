//
// Created by tobias on 2/7/25.
//
#include "libstp/sensor/ir_light_sensor.h"

#include <numeric>

#include "kipr/analog/analog.h"
#include "libstp/_config.h"

libstp::sensor::IrLightSensor::IrLightSensor(const int& port, const float& calibrationFactor):
    LightSensor(port, calibrationFactor)
{
}

int libstp::sensor::IrLightSensor::getValue()
{
    return analog(port);
}

bool libstp::sensor::IrLightSensor::isOnBlack()
{
    return getValue() > whiteThreshold;
}
