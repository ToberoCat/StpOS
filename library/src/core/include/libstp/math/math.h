//
// Created by tobias on 12/26/24.
//

#pragma once

#include <functional>
#include <Eigen/Dense>

#define RAD_TO_DEG (180 / M_PI)
#define DEG_TO_RAD (M_PI / 180)
#define M_PIf 3.14159265358979f

namespace libstp::math {
    using InterpolationFunction = std::function<float(float, float, float)>;

    float lerp(float a, float b, float t);

    float easeInOut(float a, float b, float t);

    float clampf(float value, float min, float max);

    double clampDouble(double value, double min, double max);

    int clampInt(int value, int min, int max);

    int sign(int value);

    float signf(float value);

    float minimalAngleDifference(float a, float b);

    std::tuple<double, double, double> quaternionToEuler(Eigen::Quaterniond q);
}